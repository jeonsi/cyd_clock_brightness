/*
 * ble_time.h - BLE CTS(Current Time Service) 시간 동기화 (TIME_SYNC_BLE 1일 때만 컴파일)
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
 */
#ifndef BLE_TIME_H
#define BLE_TIME_H

#include <Arduino.h>
#include "clock_config.h"

#if TIME_SYNC_BLE

#include <NimBLEDevice.h>
#include <sys/time.h>
#include <time.h>

static uint16_t cts_conn            = BLE_HS_CONN_HANDLE_NONE;
static bool     cts_read_pending    = false;
static bool     cts_synced_once     = false;
static uint32_t cts_last_attempt_ms = 0;
static bool     ancs_attempted      = false;    // 이 연결에서 ANCS 구독을 시도했는가

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
    Serial.printf("BLE CTS: attr handle %u, %d bytes\n",
                  attr->handle, OS_MBUF_PKTLEN(attr->om));
    // Exact Time 256: year(2,LE) month day hours minutes seconds
    //                 day_of_week fractions256 adjust_reason
    uint8_t buf[10];
    uint16_t len = 0;
    ble_hs_mbuf_to_flat(attr->om, buf, sizeof(buf), &len);
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
      if (epoch > 1000000000) {         // sanity: 2001년 이후
        struct timeval tv;
        tv.tv_sec  = epoch;
        tv.tv_usec = (len >= 9) ? (suseconds_t)((uint32_t)buf[8] * 1000000UL / 256UL) : 0;
        settimeofday(&tv, NULL);
        cts_synced_once = true;
        Serial.printf("BLE CTS sync: %04d-%02d-%02d %02d:%02d:%02d\n",
                      buf[0] | (buf[1] << 8), buf[2], buf[3], buf[4], buf[5], buf[6]);
      }
    }
  }

  if (status != 0) {                    // BLE_HS_EDONE(완료) 포함: 절차 종료
    cts_read_pending = false;
    // ATT는 연결당 GATT 절차를 하나씩만 허용하므로, CTS 읽기가 끝난 지금
    // ANCS 구독(자동 재연결용)을 시작한다. 연결당 1회.
    if (!ancs_attempted && cts_conn != BLE_HS_CONN_HANDLE_NONE)
      ancs_subscribe_begin(cts_conn);
    if (status == BLE_HS_EDONE) {
      // 완료. 속성을 하나도 못 받았다면 폰의 GATT에서 0x2A2B를 못 찾은 것
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
  if (cts_conn == BLE_HS_CONN_HANDLE_NONE || cts_read_pending) return;
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
    if (rc != 0) Serial.printf("BLE ANCS: CCCD write start failed (rc %d)\n", rc);
    return BLE_HS_EDONE;                            // stop descriptor discovery
  }
  if (dsc == NULL && !cccd_found)
    Serial.println("BLE ANCS: CCCD descriptor not found");
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
    if (rc != 0) Serial.printf("BLE ANCS: dsc discovery start failed (rc %d)\n", rc);
    return BLE_HS_EDONE;
  }
  if (ancs_ns_val == 0)
    Serial.printf("BLE ANCS: Notification Source not found (status 0x%04x)\n",
                  error ? error->status : 0);
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
    if (rc != 0) Serial.printf("BLE ANCS: chr discovery start failed (rc %d)\n", rc);
    return BLE_HS_EDONE;
  }
  if (ancs_start == 0)
    Serial.printf("BLE ANCS: service not found (status 0x%04x)\n",
                  error ? error->status : 0);
  return 0;
}

static void ancs_subscribe_begin(uint16_t conn) {
  ancs_start = ancs_end = ancs_ns_val = 0;
  ancs_attempted = true;
  Serial.println("BLE ANCS: subscribing (for iOS auto-reconnect)...");
  int rc = ble_gattc_disc_svc_by_uuid(conn, &UUID_ANCS_SVC.u, ancs_svc_cb, NULL);
  if (rc != 0) Serial.printf("BLE ANCS: svc discovery start failed (rc %d)\n", rc);
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

// timer_cb(100ms)에서 호출: 첫 동기화 전에는 10초마다, 그 뒤에는
// NTP_SYNC_INTERVAL_MS마다 다시 읽는다(연결되어 있을 때만).
static void ble_time_tick(void) {
  if (cts_conn == BLE_HS_CONN_HANDLE_NONE || cts_read_pending) return;
  uint32_t due = cts_synced_once ? (uint32_t)NTP_SYNC_INTERVAL_MS : 10000u;
  if (millis() - cts_last_attempt_ms >= due) cts_request_read();
}

#else  /* !TIME_SYNC_BLE: Wi-Fi 빌드에서는 전부 no-op */

static inline void ble_time_begin(void) {}
static inline void ble_time_tick(void) {}
static inline bool ble_time_connected(void) { return false; }

#endif /* TIME_SYNC_BLE */
#endif /* BLE_TIME_H */
