# CYD Clock (밝기 조절 지원)

ESP32 CYD(Cheap Yellow Display, **ESP32-2432S028R**) 보드용 디지털 탁상시계입니다.
NTP로 시간을 동기화하고, 7세그먼트(DSEG) 폰트로 시간·초·날짜를 표시하며,
화면을 터치하면 백라이트 밝기 슬라이더가 나타납니다.

[Random Nerd Tutorials의 ESP32 CYD LVGL Digital Clock](https://RandomNerdTutorials.com/esp32-cyd-lvgl-digital-clock/) 예제를 기반으로 크게 수정한 버전입니다.

## 화면 구성

```
        2026-08-23 (일)          ← 날짜(DSEG 26px) + 한글 요일(나눔고딕, 초록색)

                        AM
        11:58           ──       ← HH:MM (DSEG Bold 68px, 12시간제)
                        42       ← 초 (DSEG 30px)

          음 7.11  입추          ← 음력 날짜 + 현재 절기 (숫자는 DSEG 26px, 한글은 나눔고딕 22px)
```

- 흰 배경에 주황(`#FF3300`) 세그먼트, 요일만 초록(`#33CC66`)
- **일요일과 한국 공휴일에는 요일이 빨간색(`#E60000`)으로 표시** — 2026~2030년 공휴일(설날·추석 등 음력 명절, 대체공휴일, 2026년 재지정된 제헌절 포함)이 `korean_calendar.h`의 `KR_HOLIDAYS[]` 테이블(이름 포함)로 내장되어 있으며, 이후 연도는 행을 추가하면 됨
- 모든 필드는 폰트에서 실측한 고정 폭을 사용하므로 숫자가 바뀌어도 레이아웃이 흔들리지 않음
- 화면 아무 곳이나 터치하면 하단에 밝기 슬라이더 패널이 뜨고, 4초간 조작이 없으면 자동으로 사라짐
- **맨 아래 줄에 음력 날짜와 현재 절기 표시** — 절기는 당일뿐 아니라 다음 절기 전까지 계속 표시(예: 입추~처서 사이에는 "입추"). 윤달은 "음 윤5.1"처럼 표시. **절기 당일에는 절기 이름이 초록색으로 강조**
- **명절·공휴일 이름 표시** — 공휴일에는 하단 줄 앞에 이름이 빨간색으로 표시(예: "추석  음 8.15  추분"). 정월대보름·단오·칠석 같은 비공휴일 세시명절은 초록색으로 표시
- **조도 센서 자동 밝기** — CYD 내장 LDR(GPIO 34)로 주변 밝기를 감지해 어두우면 화면을 자동으로 어둡게 함. 슬라이더 설정값이 상한이 되고, 완전한 어둠에서는 그 15%(`BL_AUTO_MIN_PCT`)까지 내려감. 슬라이더 조작 중에는 자동 조절이 일시 중지됨
- **비차단 부팅** — 부팅 시 Wi-Fi 연결/시간 동기화 진행 상황을 화면에 표시하고, 실패해도 검은 화면에 멈추지 않고 30초마다 무한 재시도
- **아날로그 화면** — 좌우로 스와이프하면 디지털 ↔ 아날로그 화면이 전환되고 선택이 NVS에 저장됨. 아날로그 화면은 왼쪽에 시침·분침·초침(주황)이 있는 다이얼, 오른쪽에 AM/PM·요일·날짜·음력·절기·명절 정보를 세로로 표시. 탭은 여전히 밝기 슬라이더

```
      11  12  1        AM (일)
   10          2
   9    •——    3        08-23     ← 아날로그 화면 (스와이프로 전환)
   8           4       음 7.11
      7  6  5            입추
```

## 주요 기능

### 시간 관리 (SNTP)
- ESP32 시스템 클럭을 SNTP로 디시플린. 30분마다 재동기화
- `SNTP_SYNC_MODE_SMOOTH` — 시간이 점프하지 않고 서서히 보정됨
- NTP 서버: `kr.pool.ntp.org` → `pool.ntp.org` → `time.google.com`
- 타임존: `KST-9` (서울, DST 없음). `TZ_INFO`로 변경 가능
- 부팅은 비차단: 화면에 Wi-Fi 연결/시간 동기화 진행 상황(경과 시간, 재시도 횟수)을 표시하고, 첫 동기화가 완료되면 시계 화면으로 전환

### 백라이트 밝기 조절
- 터치 → 슬라이더 표시 → LEDC PWM(5 kHz, 10-bit)으로 백라이트(GPIO 21) 제어
- 인지 밝기 보정: 퍼센트를 제곱해서 duty로 변환 → 슬라이더 저역대가 균등하게 느껴짐
- 최소 5%로 제한해 화면이 완전히 꺼지는 것을 방지
- 설정값은 NVS(`Preferences`)에 저장되어 재부팅 후에도 유지. 값이 실제로 바뀌었을 때만 기록해 NVS 마모 최소화
- 조도 센서(`AUTO_BL`)와 결합: 슬라이더 값은 상한이 되고 실제 duty는 주변 밝기에 따라 스케일됨(EMA 평활 + 2% 데드밴드). 슬라이더 패널이 열려 있는 동안은 스케일링이 일시 중지되어 설정 범위를 그대로 볼 수 있음
- ESP32 Arduino core 2.x / 3.x의 LEDC API 차이를 모두 지원

### 터치 (XPT2046)
- CYD는 터치 컨트롤러가 디스플레이와 **별도의 SPI 버스**에 연결되어 있어 TFT_eSPI의 내장 `TOUCH_CS`를 쓸 수 없음 → `XPT2046_Touchscreen` 라이브러리로 직접 처리
- IRQ 핀을 의도적으로 사용하지 않고 폴링 — IRQ 래치 방식은 드래그 중 압력이 약해지면 드래그가 끊김
- 60 ms 릴리즈 디바운스 — 저항막 패널 특유의 드래그 중 순간 접촉 끊김을 흡수
- 이 패널에서 실측한 raw 좌표 범위(`620..3440` / `500..3600`)로 캘리브레이션되어 있음
- LVGL 9는 포인터 좌표 회전을 자체 처리하므로, 읽기 콜백은 네이티브 240×320 좌표만 보고함

## 하드웨어

| 항목 | 값 |
|---|---|
| 보드 | ESP32-2432S028R (CYD) |
| 디스플레이 | ILI9341 240×320, 가로(270° 회전)로 사용 |
| 터치 | XPT2046 (저항막) — MOSI 32, MISO 39, CLK 25, CS 33 |
| 백라이트 | GPIO 21 (TFT_eSPI의 `TFT_BL`) |
| 조도 센서 | LDR — GPIO 34 (입력 전용, ADC1) |

## 빌드 방법

Arduino IDE 기준:

1. **라이브러리 설치** (Library Manager)
   - `lvgl` (v9)
   - `TFT_eSPI` — CYD용 `User_Setup.h` 설정 필요 ([RNT 가이드](https://RandomNerdTutorials.com/esp32-cyd-lvgl-digital-clock/) 참고)
   - `XPT2046_Touchscreen` (Paul Stoffregen)
2. **lv_conf.h** 에서 `LV_FONT_MONTSERRAT_20` 활성화 (AM/PM·밝기 % 표시에 사용)
3. `secrets.h.example`을 `secrets.h`로 복사하고 **Wi-Fi SSID/비밀번호**를 입력 (`secrets.h`는 gitignore되어 커밋되지 않음)
4. 필요 시 `TZ_INFO`(타임존) 수정
5. 파일을 UTF-8로 저장(Arduino IDE 기본값) 후 업로드

## 폰트 (`clock_fonts.h`)

다섯 개의 LVGL 폰트가 하나의 헤더에 번들되어 있습니다:

| 폰트 | 용도 |
|---|---|
| `font_dseg_bold_68` | HH:MM (DSEG7 Classic Bold 68px, `0-9:`) |
| `font_dseg_68` | (예비) DSEG7 Classic Regular 68px |
| `font_dseg_30` | 초 (Regular 30px, `0-9`) |
| `font_dseg_26` | 날짜 (Regular 26px, `0-9-`) |
| `font_kr_26` | 한글 요일 (나눔고딕 26px, `일월화수목금토` 만 포함) |

음력/절기 줄에는 별도의 `lunar_font.h`에 담긴 두 폰트가 사용됩니다:

- `font_dseg_lunar_26` — 음력 숫자용. 양력 날짜와 동일한 DSEG7 Classic Regular 26px에 점(`.`) 글리프를 추가한 것. 번들 폰트와 똑같이 숫자 7의 F세그먼트를 제거(fontTools로 TTF 윤곽선 수술)했고, DSEG 원본의 점은 advance가 0(LCD처럼 직전 숫자에 겹침)이라 날짜 구분자로 읽히도록 7.5px로 패치함
- `font_kr_lunar_22` — "음"·"윤"·절기 명칭·명절/공휴일 이름용 나눔고딕 22px 서브셋

특이사항:

- DSEG의 기본 숫자 **7**은 A·B·C·F 세그먼트를 켜는데, 이 번들은 F를 제거해 A·B·C 세 개만 켜지도록 FontForge 소스에서 다시 래스터라이즈했습니다.
- 새 폰트를 추가하려면 `lv_font_conv`를 사용합니다. 예:

  ```bash
  npx lv_font_conv \
    --font DSEG7Classic-Bold.ttf \
    --size 68 --bpp 4 --format lvgl \
    --symbols "0123456789:" \
    --lv-include lvgl.h --no-compress \
    -o font_dseg_bold_68.c
  ```

  생성된 `.c`를 기존 폰트들과 같은 방식으로 `clock_fonts.h`에 병합하면 됩니다(내부 static 심볼에 폰트별 접미사 필요).

## 음력 · 절기 (`korean_calendar.h`)

음력 날짜와 절기는 `struct tm`만으로 계산할 수 없어 테이블 방식을 사용합니다:

- **음력 테이블 (2025~2045)** — KASI(한국천문연구원) 데이터 기반의 [`korean-lunar-calendar`](https://pypi.org/project/korean-lunar-calendar/) 파이썬 패키지로 생성. 연도별로 설날의 양력 날짜, 윤달 위치, 대월(30일) 비트마스크를 저장. C 구현은 2025~2045년 전체 7,642일에 대해 원본 라이브러리와 전수 대조 검증됨
- **절기 테이블 (2026~2035)** — 한국 절입시간(KST) 기준 날짜. 중국 표준시 기준 근사 공식과 1일 차이 나는 해가 있어 공식 대신 실제 값을 내장
- 테이블 범위를 벗어난 연도에는 해당 부분만 조용히 표시가 생략되므로, 연도가 다가오면 테이블에 행을 추가하면 됩니다

## 설정 튜닝

`cyd_clock_brightness.ino` 상단의 매크로로 조정할 수 있습니다:

| 매크로 | 기본값 | 설명 |
|---|---|---|
| `TZ_INFO` | `"KST-9"` | POSIX 타임존 문자열 |
| `NTP_SYNC_INTERVAL_MS` | 30분 | SNTP 재동기화 주기 |
| `BL_MIN_PCT` | 5 | 밝기 하한(%) |
| `BL_DEFAULT` | 80 | 최초 부팅 시 밝기(%) |
| `BL_PANEL_TIMEOUT_MS` | 4000 | 슬라이더 자동 숨김 시간 |
| `SHOW_GHOST_SEGMENTS` | 0 | 1로 켜면 꺼진 세그먼트를 옅게 표시 |
| `AUTO_BL` | 1 | 0으로 끄면 조도 센서 자동 밝기 비활성화 |
| `LDR_RAW_BRIGHT/DARK` | 300/3600 | 조도 센서 ADC 보정값. `LDR_DEBUG 1`로 raw 값을 찍어 조정 (센서를 가리면 어두움, 조명을 비추면 밝음 값) |
| `BL_AUTO_MIN_PCT` | 15 | 완전한 어둠에서 유지할 밝기(슬라이더 설정값 대비 %) |
| `WIFI_RETRY_MS` | 30초 | Wi-Fi 연결 재시도 주기 |
| `TOUCH_GESTURE_MIN_DIST` | 20 | 스와이프로 인식할 최소 이동 거리(px). LVGL 기본 50은 저항막에서 둔함 |
| `TOUCH_GESTURE_MIN_VEL` | 1 | 스와이프 최소 속도. 높이면 천천히 끄는 동작은 무시됨 |
| `TOUCH_RAW_MIN/MAX_X/Y` | 실측값 | 터치 캘리브레이션. 패널마다 다르므로 `TOUCH_DEBUG 1`로 raw 값을 찍어 조정 |
| `TOUCH_DEBUG` | 0 | 1로 켜면 시리얼로 터치 좌표 출력 |

## 라이선스

- 프로젝트 코드: [MIT License](LICENSE)
- 번들 폰트: SIL Open Font License 1.1 — 전문과 저작권 고지는 [OFL.txt](OFL.txt) 참고
  - DSEG © keshikan
  - NanumGothic © NAVER
- 원본 예제: Rui Santos & Sara Santos, [Random Nerd Tutorials](https://RandomNerdTutorials.com/)
