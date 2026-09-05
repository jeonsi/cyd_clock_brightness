/*
 * ble_time.h - BLE CTS(Current Time Service) 시간 동기화
 * (항상 컴파일되고, 시간 소스가 BLE로 선택된 부팅에서만 ble_time_begin()이 불린다)
 *
 * 구조: ESP32가 BLE 페리페럴로 광고한다. 광고에는 CTS(0x1805) 솔리시테이션과
 * ANCS 솔리시테이션을 함께 싣는데, 아이폰은 ANCS를 찾는 액세서리만 설정 >
 * Bluetooth 목록에 띄워 주기 때문이다(스마트워치들이 쓰는 방식; ANCS 알림
 * 자체는 사용하지 않음). 목록에서 이 기기를 선택하면 아이폰(센트럴)이 연결해 오고,
 * 우리가 암호화(페어링)를 요청하면 아이폰에 페어링 대화상자가 뜬다. GATT의
 * 클라이언트/서버 역할은 연결 방향과 무관하므로, 같은 연결 위에서 ESP32가
 * GATT 클라이언트가 되어 아이폰의 Current Time 특성(0x2A2B)을 읽는다.
 * (아이폰은 페어링된 기기에만 CTS 읽기를 허용한다.)
 *
 * - 첫 동기화 전에는 10초마다, 그 뒤에는 NTP_SYNC_INTERVAL_MS마다 다시 읽음
 * - 연결이 끊기면 다시 광고를 시작하고, 페어링된 아이폰이 근처에 오면 재연결됨
 * - CTS가 주는 것은 폰의 '현지 시간'이므로 TZ_INFO 기준의 현지 시간으로
 *   해석해 시스템 클럭(UTC)을 맞춘다. 폰의 시간대가 TZ_INFO와 달라지면 그
 *   차이만큼 틀리게 표시된다.
 *
 * 필요 라이브러리: NimBLE-Arduino 2.x (Library Manager에서 "NimBLE-Arduino")
 *
 * 안전장치: 수신 페이로드를 hex로 로그하고, 이미 유효한 시계 대비
 * CTS_MAX_JUMP_S 이상 튀는 값은 거부하며, CTS 읽기와 ANCS 탐색이 한 연결에서
 * 동시에 돌지 않도록 직렬화한다(ancs_busy). 동시 실행 시 NimBLE가 응답을
 * 엇갈려 배달해 ANCS 특성 선언이 CTS 시간(연도 11024)으로 파싱되어 시계를
 * 덮어쓴 사고가 esp32-c3-clock 쪽에서 실측됨. (esp32-c3-clock과 동일 코드)
 */
#ifndef BLE_TIME_H
#define BLE_TIME_H

#include <Arduino.h>
#include "clock_config.h"

#include <NimBLEDevice.h>
#include <sys/time.h>
#include <time.h>

#ifndef CTS_MAX_JUMP_S
#define CTS_MAX_JUMP_S (24 * 3600)   // 유효한 시계 대비 이보다 크게 튀는 CTS 값은 무시
#endif

static uint16_t cts_conn            = BLE_HS_CONN_HANDLE_NONE;
static bool     cts_read_pending    = false;
static bool     cts_synced_once     = false;
static uint32_t cts_sync_count      = 0;     // 성공한 동기화 횟수 (듀티사이클용)
static uint32_t cts_last_attempt_ms = 0;
static bool     ancs_attempted      = false;    // 이 연결에서 ANCS 구독을 시도했는가
static bool     ancs_busy           = false;    // ANCS 탐색/구독 절차가 진행 중인가

static void ancs_subscribe_begin(uint16_t conn);   // 아래 ANCS 절 참고

// Current Time(0x2A2B) 읽기 응답. read_by_uuid는 일치하는 속성마다 한 번,
// 마지막에 BLE_HS_EDONE 상태로 한 번 더 호출된다.
static bool cts_got_attr = false;   // this procedure returned at least one attribute

static int cts_read_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                       struct ble_gatt_attr *attr, void *arg) {
  (void)conn_handle; (void)arg;
  int status = error ? error->status : 0;

  if (attr != NULL && attr->om != NULL) {
    cts_got_attr = true;
    // Exact Time 256: year(2,LE) month day hours minutes seconds
    //                 day_of_week fractions256 adjust_reason
    uint8_t buf[10];
    uint16_t len = 0;
    ble_hs_mbuf_to_flat(attr->om, buf, sizeof(buf), &len);
    char hex[3 * sizeof(buf) + 1] = "";
    for (uint16_t i = 0; i < len && i < sizeof(buf); i++)
      snprintf(hex + i * 3, 4, "%02X ", buf[i]);
    Serial.printf("BLE CTS: attr handle %u, %d bytes: %s\n",
                  attr->handle, OS_MBUF_PKTLEN(attr->om), hex);
    if (len >= 7) {
      struct tm t = {};
      t.tm_year  = (buf[0] | (buf[1] << 8)) - 1900;
      t.tm_mon   = buf[2] - 1;
      t.tm_mday  = buf[3];
      t.tm_hour  = buf[4];
      t.tm_min   = buf[5];
      t.tm_sec   = buf[6];
      t.tm_isdst = -1;
      time_t epoch = mktime(&t);        // 폰의 현지 시간을 TZ_INFO로 해석
      time_t now = time(nullptr);
      if (epoch <= 1000000000) {        // sanity: 2001년 이후만 유효
        Serial.println("BLE CTS: nonsense timestamp, ignored");
      } else if (now > 1735689600 /* 2025-01-01: 시계가 이미 유효 */ &&
                 (epoch > now + CTS_MAX_JUMP_S || epoch < now - CTS_MAX_JUMP_S)) {
        // 유효하게 돌던 시계가 하루 이상 튀는 값은 폰/링크 이상으로 보고
        // 버린다 - 10초 뒤 재시도가 이어진다. (실제로 재동기화에서 2년 전
        // 날짜가 내려온 사례가 있음; 위의 hex 로그로 원인을 추적)
        Serial.printf("BLE CTS: implausible %04d-%02d-%02d %02d:%02d:%02d (clock jump > %d s), ignored\n",
                      buf[0] | (buf[1] << 8), buf[2], buf[3], buf[4], buf[5], buf[6],
                      (int)CTS_MAX_JUMP_S);
      } else {
        struct timeval tv;
        tv.tv_sec  = epoch;
        tv.tv_usec = (len >= 9) ? (suseconds_t)((uint32_t)buf[8] * 1000000UL / 256UL) : 0;
        settimeofday(&tv, NULL);
        cts_synced_once = true;
        cts_sync_count++;
        Serial.printf("BLE CTS sync: %04d-%02d-%02d %02d:%02d:%02d\n",
                      buf[0] | (buf[1] << 8), buf[2], buf[3], buf[4], buf[5], buf[6]);
      }
    }
  }

  if (status != 0) {                    // BLE_HS_EDONE(완료) 포함: 절차 종료
    cts_read_pending = false;
    if (status == BLE_HS_EDONE) {
      // ANCS 구독은 CTS 읽기가 "끝까지 완료(EDONE)"된 뒤에만 시작한다.
      // 원래는 실패(0x0105: 페어링 전 읽기 거부) 직후에도 시작했는데, 그러면
      // 곧이어 암호화 완료로 재시도되는 CTS 읽기와 ANCS 특성 탐색이 한 연결
      // 위에서 동시에 돌고, NimBLE가 응답을 서로 엇갈려 배달한다 - ANCS 특성
      // 선언(19바이트, 앞 2바이트가 0x2B10)이 CTS 콜백으로 들어와 연도
      // 11024년으로 파싱되어 시계를 덮어쓴 사고의 원인 (hex 로그로 확인).
      if (!ancs_attempted && cts_conn != BLE_HS_CONN_HANDLE_NONE)
        ancs_subscribe_begin(cts_conn);
      // 속성을 하나도 못 받았다면 폰의 GATT에서 0x2A2B를 못 찾은 것
      // (페어링 직후 iOS가 서비스 노출을 갱신하는 중일 수 있음 - 10초 뒤 재시도).
      if (!cts_got_attr)
        Serial.println("BLE CTS: Current Time characteristic not found (will retry)");
    } else if (!cts_synced_once) {
      // 흔한 실패: 아직 페어링(암호화) 전이라 읽기가 거부된 경우(0x0105 등).
      // 10초 뒤 재시도되고, 페어링이 끝나면 즉시 다시 읽는다.
      Serial.printf("BLE CTS read failed (status 0x%04x, will retry)\n", status);
    }
  }
  return 0;
}

static void cts_request_read(void) {
  // ancs_busy: ANCS 탐색/CCCD 쓰기가 도는 동안 읽기를 시작하지 않는다
  // (연결당 GATT 절차는 하나 - 위의 동시 실행 사고 참고). 10초 뒤 재시도된다.
  if (cts_conn == BLE_HS_CONN_HANDLE_NONE || cts_read_pending || ancs_busy) return;
  cts_read_pending = true;
  cts_got_attr = false;
  cts_last_attempt_ms = millis();
  Serial.println("BLE CTS: reading Current Time...");
  static const ble_uuid16_t uuid_current_time = BLE_UUID16_INIT(0x2A2B);
  int rc = ble_gattc_read_by_uuid(cts_conn, 1, 0xFFFF,
                                  &uuid_current_time.u, cts_read_cb, NULL);
  if (rc != 0) {
    cts_read_pending = false;
    Serial.printf("BLE CTS read request failed (rc %d)\n", rc);
  }
}

// ---- ANCS 구독 (iOS 자동 재연결용) -----------------------------------------
// iOS는 "본딩 + 광고"만으로는 먼저 연결을 걸어 주지 않는다. 시스템이 재연결을
// 관리해 주는 것은 알림(ANCS) 세션을 맺은 액세서리(스마트워치 방식)라서,
// 페어링 후 아이폰의 ANCS Notification Source 특성을 구독해 둔다. 최초 1회
// 아이폰에 "알림 공유" 허용 요청이 뜨고, 허용하면 이후 전원을 껐다 켜도
// 아이폰이 광고를 보고 스스로 재연결한다. 알림 데이터 자체는 사용하지 않는다.
static const ble_uuid128_t UUID_ANCS_SVC = BLE_UUID128_INIT(
  0xD0, 0x00, 0x2D, 0x12, 0x1E, 0x4B, 0x0F, 0xA4,
  0x99, 0x4E, 0xCE, 0xB5, 0x31, 0xF4, 0x05, 0x79);   // 7905F431-...-122D00D0
static const ble_uuid128_t UUID_ANCS_NOTIF_SRC = BLE_UUID128_INIT(
  0xBD, 0x1D, 0xA2, 0x99, 0xE6, 0x25, 0x58, 0x8C,
  0xD9, 0x42, 0x01, 0x63, 0x0D, 0x12, 0xBF, 0x9F);   // 9FBF120D-...-99A21DBD

static uint16_t ancs_start = 0, ancs_end = 0;   // ANCS 서비스 핸들 범위
static uint16_t ancs_ns_val = 0;                // Notification Source 값 핸들

static int ancs_cccd_write_cb(uint16_t conn, const struct ble_gatt_error *error,
                              struct ble_gatt_attr *attr, void *arg) {
  (void)conn; (void)attr; (void)arg;
  ancs_busy = false;                              // 절차 종료(성공/실패 공통)
  int status = error ? error->status : 0;
  if (status == 0)
    Serial.println("BLE ANCS: subscribed (iOS will auto-reconnect from now on)");
  else
    Serial.printf("BLE ANCS: subscribe failed (status 0x%04x)\n", status);
  return 0;
}

static int ancs_dsc_cb(uint16_t conn, const struct ble_gatt_error *error,
                       uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc,
                       void *arg) {
  (void)chr_val_handle; (void)arg;
  static bool cccd_found;
  if (dsc != NULL && dsc->uuid.u.type == BLE_UUID_TYPE_16 &&
      dsc->uuid.u16.value == 0x2902) {
    cccd_found = true;
    static const uint8_t on[2] = { 0x01, 0x00 };    // notifications on
    int rc = ble_gattc_write_flat(conn, dsc->handle, on, sizeof(on),
                                  ancs_cccd_write_cb, NULL);
    if (rc != 0) {
      Serial.printf("BLE ANCS: CCCD write start failed (rc %d)\n", rc);
      ancs_busy = false;
    }
    return BLE_HS_EDONE;                            // stop descriptor discovery
  }
  if (dsc == NULL && !cccd_found) {
    Serial.println("BLE ANCS: CCCD descriptor not found");
    ancs_busy = false;                              // 절차가 CCCD 없이 끝남
  }
  if (dsc == NULL) cccd_found = false;              // reset for the next attempt
  return 0;
}

static int ancs_chr_cb(uint16_t conn, const struct ble_gatt_error *error,
                       const struct ble_gatt_chr *chr, void *arg) {
  (void)arg;
  if (chr != NULL) {
    ancs_ns_val = chr->val_handle;
    Serial.printf("BLE ANCS: Notification Source at handle %u\n", ancs_ns_val);
    int rc = ble_gattc_disc_all_dscs(conn, chr->val_handle, ancs_end, ancs_dsc_cb, NULL);
    if (rc != 0) {
      Serial.printf("BLE ANCS: dsc discovery start failed (rc %d)\n", rc);
      ancs_busy = false;
    }
    return BLE_HS_EDONE;
  }
  if (ancs_ns_val == 0) {
    Serial.printf("BLE ANCS: Notification Source not found (status 0x%04x)\n",
                  error ? error->status : 0);
    ancs_busy = false;
  }
  return 0;
}

static int ancs_svc_cb(uint16_t conn, const struct ble_gatt_error *error,
                       const struct ble_gatt_svc *service, void *arg) {
  (void)arg;
  if (service != NULL) {
    ancs_start = service->start_handle;
    ancs_end   = service->end_handle;
    Serial.printf("BLE ANCS: service found (handles %u..%u)\n", ancs_start, ancs_end);
    int rc = ble_gattc_disc_chrs_by_uuid(conn, ancs_start, ancs_end,
                                         &UUID_ANCS_NOTIF_SRC.u, ancs_chr_cb, NULL);
    if (rc != 0) {
      Serial.printf("BLE ANCS: chr discovery start failed (rc %d)\n", rc);
      ancs_busy = false;
    }
    return BLE_HS_EDONE;
  }
  if (ancs_start == 0) {
    Serial.printf("BLE ANCS: service not found (status 0x%04x)\n",
                  error ? error->status : 0);
    ancs_busy = false;
  }
  return 0;
}

static void ancs_subscribe_begin(uint16_t conn) {
  ancs_start = ancs_end = ancs_ns_val = 0;
  ancs_attempted = true;
  ancs_busy = true;
  Serial.println("BLE ANCS: subscribing (for iOS auto-reconnect)...");
  int rc = ble_gattc_disc_svc_by_uuid(conn, &UUID_ANCS_SVC.u, ancs_svc_cb, NULL);
  if (rc != 0) {
    Serial.printf("BLE ANCS: svc discovery start failed (rc %d)\n", rc);
    ancs_busy = false;
  }
}

class CtsServerCallbacks : public NimBLEServerCallbacks {
  // 주의: 이미 페어링된 폰이 재연결할 때는 암호화 완료(onAuthenticationComplete)가
  // 연결 콜백보다 먼저 도착하기도 한다. 어느 순서로 와도 동작하도록 두 콜백
  // 모두 연결 핸들을 기록하고, 암호화가 서 있으면 곧바로 읽는다.
  void onConnect(NimBLEServer * srv, NimBLEConnInfo & info) override {
    (void)srv;
    cts_conn = info.getConnHandle();
    Serial.printf("BLE: phone connected (%s)\n",
                  info.isEncrypted() ? "already encrypted" : "requesting encryption");
    if (info.isEncrypted()) {
      cts_request_read();
    } else {
      // 아이폰의 CTS는 암호화된(페어링된) 연결에서만 읽을 수 있다.
      // 최초 1회는 아이폰에 페어링 대화상자가 뜬다.
      NimBLEDevice::startSecurity(cts_conn);
    }
  }

  void onDisconnect(NimBLEServer * srv, NimBLEConnInfo & info, int reason) override {
    (void)srv; (void)info;
    Serial.printf("BLE: phone disconnected (reason %d)\n", reason);
    cts_conn = BLE_HS_CONN_HANDLE_NONE;
    cts_read_pending = false;
    ancs_attempted = false;
    ancs_busy = false;
    NimBLEDevice::startAdvertising();   // 페어링된 폰이 돌아오면 재연결되도록
  }

  void onAuthenticationComplete(NimBLEConnInfo & info) override {
    cts_conn = info.getConnHandle();   // 연결 콜백보다 먼저 올 수 있음
    Serial.printf("BLE: pairing/encryption %s\n", info.isEncrypted() ? "OK" : "FAILED");
    if (info.isEncrypted()) cts_request_read();
  }
};

static void ble_time_begin(void) {
  // Wi-Fi 모드에서는 configTzTime()이 하던 일: 현지 시간 해석용 타임존 설정
  setenv("TZ", TZ_INFO, 1);
  tzset();

  NimBLEDevice::init(BLE_DEVICE_NAME);
  NimBLEDevice::setSecurityAuth(true /*bond*/, false /*mitm*/, true /*secure conn*/);

  NimBLEServer * srv = NimBLEDevice::createServer();
  srv->setCallbacks(new CtsServerCallbacks(), true);

  NimBLEAdvertising * adv = NimBLEDevice::getAdvertising();
  NimBLEAdvertisementData ad;
  ad.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);
  // Service Solicitation: "이 서비스를 가진 상대를 찾는 액세서리"라는 표시.
  //   0x14 = 16-bit 솔리시테이션: CTS(0x1805) - 우리가 실제로 읽을 서비스
  //   0x15 = 128-bit 솔리시테이션: ANCS - 아이폰은 ANCS를 찾는 액세서리만
  //          설정 > Bluetooth 목록(기타 기기)에 띄워 주므로, 목록에 뜨게
  //          하려면 이것이 필요함(ANCS 알림 자체는 사용하지 않음)
  static const uint8_t solicit_cts[] = { 0x03, 0x14, 0x05, 0x18 };
  static const uint8_t solicit_ancs[] = {
    0x11, 0x15,
    // ANCS UUID 7905F431-B5CE-4E99-A40F-4B1E122D00D0 (리틀엔디언)
    0xD0, 0x00, 0x2D, 0x12, 0x1E, 0x4B, 0x0F, 0xA4,
    0x99, 0x4E, 0xCE, 0xB5, 0x31, 0xF4, 0x05, 0x79,
  };
  ad.addData(solicit_cts, sizeof(solicit_cts));
  ad.addData(solicit_ancs, sizeof(solicit_ancs));
  adv->setAdvertisementData(ad);
  // 이름은 광고 패킷(31바이트)에 자리가 없어 스캔 응답으로 보냄 -
  // 아이폰 목록에는 똑같이 이름이 표시된다.
  NimBLEAdvertisementData sr;
  sr.setName(BLE_DEVICE_NAME);
  adv->setScanResponseData(sr);
  adv->start();

  Serial.printf("BLE: advertising as '%s' (pair from the iPhone's Bluetooth settings)\n",
                BLE_DEVICE_NAME);
}

static inline bool ble_time_connected(void) {
  return cts_conn != BLE_HS_CONN_HANDLE_NONE;
}

// 스택을 내리고 라디오를 끈다. 본딩 키는 NVS에 있어 그대로 남고,
// deinit(true)가 서버/광고 객체까지 지워 주므로 다음 ble_time_begin()이
// 처음처럼 다시 만들 수 있다.
static void ble_time_end(void) {
  cts_conn = BLE_HS_CONN_HANDLE_NONE;
  cts_read_pending = false;
  ancs_attempted = false;
  ancs_busy = false;
  NimBLEDevice::deinit(true);
  Serial.println("BLE: stack stopped");
}

// timer_cb(100ms)에서 호출: 첫 동기화 전에는 10초마다, 그 뒤에는
// NTP_SYNC_INTERVAL_MS마다 다시 읽는다(연결되어 있을 때만).
static void ble_time_tick(void) {
  if (cts_conn == BLE_HS_CONN_HANDLE_NONE || cts_read_pending) return;
  uint32_t due = cts_synced_once ? (uint32_t)NTP_SYNC_INTERVAL_MS : 10000u;
  if (millis() - cts_last_attempt_ms >= due) cts_request_read();
}

#endif /* BLE_TIME_H */
