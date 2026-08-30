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
```

- 흰 배경에 주황(`#FF3300`) 세그먼트, 요일만 초록(`#33CC66`)
- **일요일과 한국 공휴일에는 요일이 빨간색(`#E60000`)으로 표시** — 2026~2030년 공휴일(설날·추석 등 음력 명절, 대체공휴일, 2026년 재지정된 제헌절 포함)이 코드에 테이블로 내장되어 있으며, 이후 연도는 `HOLIDAYS_KR[]` 테이블에 날짜를 추가하면 됨
- 모든 필드는 폰트에서 실측한 고정 폭을 사용하므로 숫자가 바뀌어도 레이아웃이 흔들리지 않음
- 화면 아무 곳이나 터치하면 하단에 밝기 슬라이더 패널이 뜨고, 4초간 조작이 없으면 자동으로 사라짐

## 주요 기능

### 시간 관리 (SNTP)
- ESP32 시스템 클럭을 SNTP로 디시플린. 30분마다 재동기화
- `SNTP_SYNC_MODE_SMOOTH` — 시간이 점프하지 않고 서서히 보정됨
- NTP 서버: `kr.pool.ntp.org` → `pool.ntp.org` → `time.google.com`
- 타임존: `KST-9` (서울, DST 없음). `TZ_INFO`로 변경 가능
- 부팅 시 첫 동기화가 끝날 때까지 대기 후 화면을 그림

### 백라이트 밝기 조절
- 터치 → 슬라이더 표시 → LEDC PWM(5 kHz, 10-bit)으로 백라이트(GPIO 21) 제어
- 인지 밝기 보정: 퍼센트를 제곱해서 duty로 변환 → 슬라이더 저역대가 균등하게 느껴짐
- 최소 5%로 제한해 화면이 완전히 꺼지는 것을 방지
- 설정값은 NVS(`Preferences`)에 저장되어 재부팅 후에도 유지. 값이 실제로 바뀌었을 때만 기록해 NVS 마모 최소화
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
| `TOUCH_RAW_MIN/MAX_X/Y` | 실측값 | 터치 캘리브레이션. 패널마다 다르므로 `TOUCH_DEBUG 1`로 raw 값을 찍어 조정 |
| `TOUCH_DEBUG` | 0 | 1로 켜면 시리얼로 터치 좌표 출력 |

## 라이선스

- DSEG © Keshikan — SIL Open Font License 1.1
- NanumGothic © NAVER — SIL Open Font License 1.1
- 원본 예제: Rui Santos & Sara Santos, [Random Nerd Tutorials](https://RandomNerdTutorials.com/)
