/*  Based on Rui Santos & Sara Santos - Random Nerd Tutorials
    https://RandomNerdTutorials.com/esp32-cyd-lvgl-digital-clock/

    MODIFIED:
      - time kept by the ESP32 system clock, disciplined by SNTP
      - seven-segment (DSEG) digits for the time, seconds and date
      - HH:MM drawn in the Bold weight, everything else Regular
      - larger time and seconds, small AM/PM, Korean weekday
      - the weekday turns red on Sundays and Korean public holidays
        (built-in table for 2026-2030, substitute holidays included)
      - bottom line shows the Korean lunar date and the current solar
        term, e.g. "음 7.11  입추" (tables in korean_calendar.h,
        NanumGothic subset font in lunar_font.h)
      - holiday / festival names on the bottom line (설날, 추석, 단오, ...)
        and the solar term highlighted in green on its 절입 day
      - non-blocking boot: the panel shows Wi-Fi / NTP progress and retries
        forever instead of hanging on a black screen
      - ambient auto-brightness from the CYD's LDR on GPIO 34; the slider
        sets the ceiling, darkness dims toward BL_AUTO_MIN_PCT of it
      - analog face (lv_scale dial + line needles) with a compact info
        column; swipe left/right to switch faces, choice kept in NVS
      - background color selectable from swatches on the brightness panel
        (white / ivory / dark gray / black, THEMES table, kept in NVS)
      - every field has a fixed width so nothing shifts when the digits change
      - touch the screen to bring up a backlight brightness slider
        (XPT2046 touch + LEDC PWM on the backlight pin, value kept in NVS)

    REQUIRED FILES in the same folder as this .ino:
      clock_fonts.h     - DSEG7 68/30/26 px + NanumGothic 26 px
                          AND font_dseg_bold_68 (see below)
      lunar_font.h      - NanumGothic 22 px subset + DSEG 26 px with '.'
      korean_calendar.h - lunar calendar, solar terms, holiday tables
      secrets.h         - Wi-Fi credentials (copy secrets.h.example)

    The bold face is a new addition. Generate it with:

      npx lv_font_conv \
        --font DSEG7Classic-Bold.ttf \
        --size 68 --bpp 4 --format lvgl \
        --symbols "0123456789:" \
        --lv-include lvgl.h --no-compress \
        -o font_dseg_bold_68.c

    then merge the generated file into clock_fonts.h the same way the other
    fonts are bundled there. Without it this sketch will not link.

    The XPT2046 touch controller is driven directly over SPI (pressure
    hysteresis + median/EMA filtering) - no touch library is required.

    Save this file as UTF-8 (the Arduino IDE default).
*/

#include <lvgl.h>
#include <TFT_eSPI.h>

#include <SPI.h>
#include <Preferences.h>

#include <WiFi.h>
#include "time.h"
#include "esp_sntp.h"

#include "clock_fonts.h"
#include "lunar_font.h"
#include "korean_calendar.h"

// Wi-Fi credentials live in secrets.h (gitignored).
// Copy secrets.h.example to secrets.h and fill in your own.
#include "secrets.h"
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

// POSIX TZ string. "KST-9" = UTC+9, no DST (Seoul).
#define TZ_INFO "KST-9"

// How often SNTP re-syncs the system clock (milliseconds)
#define NTP_SYNC_INTERVAL_MS (30 * 60 * 1000)

// Panel size in its native, unrotated orientation
#define SCREEN_WIDTH  240
#define SCREEN_HEIGHT 320

#define DRAW_BUF_SIZE (SCREEN_WIDTH * SCREEN_HEIGHT / 10 * (LV_COLOR_DEPTH / 8))
uint32_t draw_buf[DRAW_BUF_SIZE / 4];

// HH:MM uses the Bold weight; the smaller fields stay Regular so the time
// stands out. Swap in font_dseg_68 here for the thinner Regular weight.
#define FONT_TIME     &font_dseg_bold_68   // HH:MM
#define FONT_SEC      &font_dseg_30        // seconds
#define FONT_DATENUM  &font_dseg_26        // date digits
#define FONT_KR       &font_kr_26          // weekday
#define FONT_AMPM     &lv_font_montserrat_20   // must be enabled in lv_conf.h
#define FONT_LUNAR         &font_kr_lunar_22     // "음"/"윤" prefix + solar term
#define FONT_LUNAR_NUM     &font_dseg_lunar_26   // lunar date digits, DSEG like the solar date
#define FONT_LUNAR_NUM_SM  &font_dseg_lunar_20   // smaller variant for the analog info column

// Same orange as the solar date digits. Point this at a gray (e.g. 0x777777)
// to demote the line to secondary information.
#define LUNAR_COLOR   lv_color_hex(0xFF3300)

#define SHOW_GHOST_SEGMENTS 0
#define GHOST_COLOR lv_color_hex(0xDDDDDD)

#define WEEKDAY_BASELINE_NUDGE 4

// ======================= Backlight (PWM) =================================
// TFT_eSPI's User_Setup for the CYD defines TFT_BL (GPIO 21 on the
// ESP32-2432S028R). Fall back to 21 if it is not defined.
#ifdef TFT_BL
  #define BL_PIN TFT_BL
#else
  #define BL_PIN 21
#endif

#define BL_FREQ      5000   // 5 kHz - above audible range, no visible flicker
#define BL_RES       10     // 10-bit duty (0..1023)
#define BL_CHANNEL   0      // only used on ESP32 Arduino core 2.x
#define BL_MIN_PCT   5      // never let the user turn the panel fully black
#define BL_DEFAULT   80

#define BL_PANEL_TIMEOUT_MS 4000   // auto-hide the slider after this idle time

// ======================= Ambient light (LDR) ==============================
// The CYD has a photoresistor on GPIO 34 (input-only, ADC1). It scales the
// user's brightness setting: the slider sets the ceiling, ambient light
// decides how much of it is used, down to BL_AUTO_MIN_PCT in full darkness.
// While the slider panel is open the scaling is suspended (factor 100%) so
// the user sees the true range they are setting.
#define AUTO_BL          1     // 0 disables ambient dimming entirely
#define LDR_PIN          34
// Raw ADC endpoints of the mapping, with 11 dB attenuation. Calibrate with
// LDR_DEBUG 1: cover the sensor for the dark value, shine a lamp at it for
// the bright one. Swapped endpoints (bright > dark) also work.
#define LDR_RAW_BRIGHT   300
#define LDR_RAW_DARK     3600
#define BL_AUTO_MIN_PCT  15    // % of the user setting kept in full darkness
#define LDR_DEBUG        0

// ======================= Boot / connectivity ==============================
#define WIFI_RETRY_MS   (30 * 1000)   // re-issue WiFi.begin() this often

// ======================= Touch (XPT2046) =================================
// The CYD wires the touch controller to its own SPI bus, separate from the
// display, so TFT_eSPI's built-in TOUCH_CS support cannot be used here.
// The XPT2046 is driven directly over SPI (no library): the stock
// XPT2046_Touchscreen library hardcodes its pressure threshold at 400,
// which a stylus tip rarely reaches - pen drags kept dropping out - and
// its 3-sample averaging leaves visible jitter on this panel.
#define XPT2046_MOSI 32
#define XPT2046_MISO 39
#define XPT2046_CLK  25
#define XPT2046_CS   33

// Measured on this panel with TOUCH_DEBUG and a stylus at the four screen
// corners, using the raw driver below (the old library's interleaved
// sampling read a noticeably different range - recalibrate after any
// change to the sampling scheme):
//   top    rx ~326/303   bottom rx ~3836/3837
//   left   ry ~3717/3719 right  ry ~170/138
#define TOUCH_RAW_MIN_X 315
#define TOUCH_RAW_MAX_X 3837
#define TOUCH_RAW_MIN_Y 154
#define TOUCH_RAW_MAX_Y 3718

// IMPORTANT: LVGL 9 rotates pointer coordinates itself, inside
// indev_pointer_proc(), to match the display rotation. The read callback must
// therefore report coordinates in the panel's NATIVE 240x320 space and must
// not apply any rotation of its own. There is deliberately no swap-axes flag
// here; only these two direction flags remain.
#define TOUCH_INVERT_X 1
#define TOUCH_INVERT_Y 1

// Constant offset correction, in native px, applied after the inversion
// above. With the corner-measured TOUCH_RAW_* calibration these stay 0;
// they remain as a quick fix for small drift. Native X maps to the
// SCREEN-VERTICAL axis under the 270-degree rotation: if the pointer
// registers BELOW where you actually touch, make TOUCH_TRIM_X more
// positive; TOUCH_TRIM_Y is the same idea for the screen-horizontal axis.
#define TOUCH_TRIM_X 0
#define TOUCH_TRIM_Y 0

// Momentary contact loss during a drag is common on a resistive panel. Hold
// the last position for this long before reporting a release, otherwise LVGL
// sees a stream of press/release pairs instead of one continuous drag.
#define TOUCH_RELEASE_DEBOUNCE_MS 60

// Pressure (Z) hysteresis: a touch starts only above TOUCH_Z_PRESS, but
// stays alive down to TOUCH_Z_RELEASE. A finger easily exceeds the press
// level; a stylus hovers around it, and without the low hold level its
// drags kept breaking up. Raise TOUCH_Z_PRESS if resting dust or a palm
// triggers phantom touches.
#define TOUCH_Z_PRESS      300
#define TOUCH_Z_RELEASE    100

// Per-frame noise handling: TOUCH_SAMPLES reads per axis, sorted, middle
// half averaged (kills outlier spikes), then an EMA across frames.
// Lower alpha = smoother but laggier cursor.
#define TOUCH_SAMPLES      8
#define TOUCH_FILTER_ALPHA 0.4f

// Swipe (face switch) recognition, done directly in touchscreen_read() so it
// works no matter which widget sits under the finger (LVGL press events stop
// at clickable widgets like the analog dial and don't bubble to the screen).
// LVGL's own gesture detector is not used either: it requires a minimum
// *instantaneous velocity* at release, which pressure wobble on a resistive
// panel rarely satisfies. A swipe here is pure displacement between press
// and release: at least this many px of screen-horizontal travel, and more
// horizontal than vertical by 2:1. A tap only wanders a few px.
#define TOUCH_SWIPE_MIN_PX 30

// Set to 1 to print raw + mapped touch coordinates on the serial monitor.
// Note the mapped values are in the unrotated 240x320 space, so they will not
// line up visually with where you pressed on the rotated screen.
#define TOUCH_DEBUG 1

SPIClass touchscreenSPI = SPIClass(VSPI);

// ---- Raw XPT2046 driver ---------------------------------------------------
// Polling over SPI at the indev read rate; the PENIRQ pin is not used (an
// IRQ latch dies mid-drag whenever pressure dips).

// One 12-bit conversion: command byte, then a leading busy bit and 12 data
// bits arrive over the next two bytes.
static inline uint16_t xpt_conv(uint8_t cmd) {
  touchscreenSPI.transfer(cmd);
  uint16_t hi = touchscreenSPI.transfer(0);
  uint16_t lo = touchscreenSPI.transfer(0);
  return (((hi << 8) | lo) >> 3) & 0x0FFF;
}

// Sort a small sample buffer and average its middle half - median-style
// filtering that discards the occasional wild outlier entirely.
static int16_t xpt_median_avg(int16_t * v, int n) {
  for (int i = 1; i < n; i++) {
    int16_t t = v[i];
    int j = i - 1;
    while (j >= 0 && v[j] > t) { v[j + 1] = v[j]; j--; }
    v[j + 1] = t;
  }
  int32_t sum = 0;
  int lo = n / 4, hi = n - n / 4;
  for (int i = lo; i < hi; i++) sum += v[i];
  return (int16_t)(sum / (hi - lo));
}

// One touch frame: returns the pressure z; fills *rx/*ry (only when z is at
// least TOUCH_Z_RELEASE) with coordinates in the same raw space the
// XPT2046_Touchscreen library produced at rotation 0, so the TOUCH_RAW_*
// calibration keeps its meaning: raw_x = 4095 - conv(0xD1), raw_y =
// conv(0x91). Careful when comparing with the library source: it pipelines
// commands through transfer16(), so each of its reads returns the result of
// the PREVIOUS command - its "0xD1" data slots actually hold 0x91
// conversions and vice versa. The conversions here are not pipelined
// (command then two data bytes), so command and result correspond directly.
static int xpt_frame(int32_t * rx, int32_t * ry) {
  touchscreenSPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  digitalWrite(XPT2046_CS, LOW);

  int z1 = xpt_conv(0xB1);            // Z1, reference kept on
  int z2 = xpt_conv(0xC1);            // Z2
  int z = z1 + 4095 - z2;

  if (z >= TOUCH_Z_RELEASE) {
    int16_t xs[TOUCH_SAMPLES], ys[TOUCH_SAMPLES];
    xpt_conv(0xD1);                   // first X after driver switch is noisy
    for (int i = 0; i < TOUCH_SAMPLES; i++) xs[i] = (int16_t)xpt_conv(0xD1);
    xpt_conv(0x91);                   // same for Y
    for (int i = 0; i < TOUCH_SAMPLES; i++) ys[i] = (int16_t)xpt_conv(0x91);
    *rx = 4095 - xpt_median_avg(xs, TOUCH_SAMPLES);
    *ry = xpt_median_avg(ys, TOUCH_SAMPLES);
  }

  xpt_conv(0x90);                     // PD=00: power down between frames
  digitalWrite(XPT2046_CS, HIGH);
  touchscreenSPI.endTransaction();
  return z;
}

// tm_wday: 0 = Sunday
static const char * const WEEKDAY_KR[7] = {"일", "월", "화", "수", "목", "금", "토"};

// ======================= Calendar colors ==================================
// The holiday date table itself lives in korean_calendar.h (kr_holiday_name,
// kr_is_red_day, kr_lunar_festival).
#define WEEKDAY_COLOR_NORMAL  lv_color_hex(0x33CC66)
#define WEEKDAY_COLOR_HOLIDAY lv_color_hex(0xE60000)

#define EVENT_HOLIDAY_COLOR   WEEKDAY_COLOR_HOLIDAY   // 설날, 추석, ...
#define EVENT_FESTIVAL_COLOR  lv_color_hex(0x33CC66)  // 정월대보름, 단오, 칠석
#define TERM_TODAY_COLOR      lv_color_hex(0x33CC66)  // 절기 당일 강조

// ======================= Analog face ======================================
// Swipe left/right anywhere on the clock to switch between the digital and
// analog faces; the choice is stored in NVS. A tap still opens the
// brightness slider.
#define ANALOG_DIAL_SIZE  188
#define ANALOG_HOUR_LEN   48
#define ANALOG_MIN_LEN    68
#define ANALOG_SEC_LEN    78

// ======================= Background themes ================================
// A row of swatches on the brightness panel picks the background color; the
// choice is stored in NVS. Ink that must adapt to stay readable (dial ticks
// and numerals, hour/minute needles, boot text) is part of each theme; the
// orange segments and the green/red calendar colors read fine on all of
// them. Note SHOW_GHOST_SEGMENTS' GHOST_COLOR is tuned for light themes.
typedef struct {
  uint32_t bg;
  uint32_t dial_major;   // major ticks, hour numerals, boot text
  uint32_t dial_minor;   // minute ticks
  uint32_t needle_hm;    // hour and minute needles
} theme_t;

static const theme_t THEMES[] = {
  { 0xFFFFFF, 0x444444, 0xBBBBBB, 0x333333 },  // white (default)
  { 0xFFF3DC, 0x444444, 0xBBBBBB, 0x333333 },  // warm ivory
  { 0xE9E9E9, 0x444444, 0xB0B0B0, 0x333333 },  // light gray
  { 0xDCE9F8, 0x445566, 0xA9BCD0, 0x334455 },  // pale blue
  { 0x202020, 0xCCCCCC, 0x666666, 0xDDDDDD },  // dark gray
  { 0x0E1626, 0xC9D4E4, 0x51617A, 0xDCE4F0 },  // navy
  { 0x1F130A, 0xD8C4AE, 0x6B584A, 0xE8D8C4 },  // dark brown
  { 0x000000, 0xCCCCCC, 0x666666, 0xDDDDDD },  // black
};
#define THEME_COUNT ((int)(sizeof(THEMES) / sizeof(THEMES[0])))

static lv_obj_t * label_ampm;
static lv_obj_t * label_hm;
static lv_obj_t * label_sec;
static lv_obj_t * label_datenum;
static lv_obj_t * label_wd;
static lv_obj_t * label_lunar_event; // holiday / festival name
static lv_obj_t * label_lunar_pre;   // "음" / "음 윤"
static lv_obj_t * label_lunar_num;   // "7.11" in DSEG
static lv_obj_t * label_lunar_term;  // current solar term

// The two faces are full-screen sibling containers; exactly one is visible.
static lv_obj_t * face_digital;
static lv_obj_t * face_analog;
static lv_obj_t * a_scale;
static lv_obj_t * a_needle_h;
static lv_obj_t * a_needle_m;
static lv_obj_t * a_needle_s;
static lv_obj_t * label_a_ampm;
static lv_obj_t * label_a_wd;
static lv_obj_t * label_a_date;       // "08-23"
static lv_obj_t * label_a_lunar_pre;  // "음" / "음 윤"
static lv_obj_t * label_a_lunar_num;  // "7.11" in DSEG
static lv_obj_t * label_a_term;
static lv_obj_t * label_a_event;

static int      face_mode = 0;        // 0 = digital, 1 = analog (kept in NVS)
static uint32_t last_gesture_ms = 0;
static int      theme_idx = 0;        // index into THEMES (kept in NVS)
static lv_obj_t * theme_btns[THEME_COUNT];

enum boot_state_t { BOOT_WIFI, BOOT_NTP, BOOT_DONE };
static boot_state_t boot_state = BOOT_WIFI;
static lv_obj_t *   boot_label;
static uint32_t     boot_t0;
static uint32_t     wifi_attempt_ms;
static int          wifi_attempts = 1;

static lv_obj_t * bl_panel;
static lv_obj_t * bl_slider;
static lv_obj_t * bl_pct_label;

static Preferences prefs;
static int      bl_pct = BL_DEFAULT;
static int      bl_saved_pct = -1;
static uint32_t bl_last_touch_ms = 0;
static int      bl_auto_factor = 100;   // % of bl_pct, driven by the LDR
static float    ldr_ema = -1.0f;

void log_print(lv_log_level_t level, const char * buf) {
  LV_UNUSED(level);
  Serial.println(buf);
  Serial.flush();
}

void time_sync_notification_cb(struct timeval * tv) {
  LV_UNUSED(tv);
  struct tm t;
  time_t now = time(nullptr);
  localtime_r(&now, &t);
  char buf[32];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &t);
  Serial.printf("NTP sync: %s\n", buf);
}

// ---- Backlight -----------------------------------------------------------
// Perceived brightness is roughly the square of the duty cycle, so squaring
// the percentage makes the low end of the slider feel evenly spaced.
static uint32_t bl_pct_to_duty(int pct) {
  if (pct < BL_MIN_PCT) pct = BL_MIN_PCT;
  if (pct > 100) pct = 100;
  float f = pct / 100.0f;
  f = f * f;
  uint32_t maxd = (1u << BL_RES) - 1u;
  uint32_t d = (uint32_t)(f * maxd + 0.5f);
  return d < 8 ? 8 : d;   // keep a faint glow at the very bottom
}

static void bl_apply(void) {
  int pct = bl_pct * bl_auto_factor / 100;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(BL_PIN, bl_pct_to_duty(pct));
#else
  ledcWrite(BL_CHANNEL, bl_pct_to_duty(pct));
#endif
}

#if AUTO_BL
// Called from timer_cb, i.e. every 200 ms. EMA smoothing keeps the panel
// from pumping when a hand or shadow passes over the sensor.
static void auto_bl_update(void) {
  int raw = analogRead(LDR_PIN);
  ldr_ema = (ldr_ema < 0) ? (float)raw : ldr_ema + 0.1f * ((float)raw - ldr_ema);

#if LDR_DEBUG
  static uint32_t dbg_ms = 0;
  if (millis() - dbg_ms > 1000) {
    dbg_ms = millis();
    Serial.printf("LDR raw=%d ema=%.0f factor=%d%%\n", raw, ldr_ema, bl_auto_factor);
  }
#endif

  int target;
  if (bl_panel && !lv_obj_has_flag(bl_panel, LV_OBJ_FLAG_HIDDEN)) {
    target = 100;   // full range while the user is adjusting
  } else {
    float x = ((float)LDR_RAW_DARK - ldr_ema)
            / ((float)LDR_RAW_DARK - (float)LDR_RAW_BRIGHT);  // 1 bright .. 0 dark
    if (x < 0.0f) x = 0.0f;
    if (x > 1.0f) x = 1.0f;
    target = BL_AUTO_MIN_PCT + (int)(x * (100 - BL_AUTO_MIN_PCT) + 0.5f);
  }

  // 2% deadband so tiny ADC noise does not cause continuous PWM rewrites
  if (target - bl_auto_factor >= 2 || bl_auto_factor - target >= 2) {
    bl_auto_factor = target;
    bl_apply();
  }
}
#endif

static void bl_begin(void) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(BL_PIN, BL_FREQ, BL_RES);
#else
  ledcSetup(BL_CHANNEL, BL_FREQ, BL_RES);
  ledcAttachPin(BL_PIN, BL_CHANNEL);
#endif
  bl_apply();
}

// NVS has a limited erase budget, so only write when the value really changed
static void bl_store(void) {
  if (bl_pct != bl_saved_pct) {
    prefs.putInt("bl", bl_pct);
    bl_saved_pct = bl_pct;
    Serial.printf("Brightness saved: %d%%\n", bl_pct);
  }
}

static void bl_set_pct(int pct) {
  bl_pct = pct;
  bl_apply();
  char buf[8];
  snprintf(buf, sizeof(buf), "%d%%", bl_pct);
  lv_label_set_text(bl_pct_label, buf);
}

static void bl_panel_show(void) {
  lv_obj_remove_flag(bl_panel, LV_OBJ_FLAG_HIDDEN);
  bl_last_touch_ms = millis();
}

static void bl_slider_cb(lv_event_t * e) {
  LV_UNUSED(e);
  bl_set_pct(lv_slider_get_value(bl_slider));
  bl_last_touch_ms = millis();
}

// Repaint everything whose color belongs to the theme. Safe to call before
// the analog face exists (during boot only the background matters).
static void theme_apply(void) {
  const theme_t * th = &THEMES[theme_idx];
  lv_obj_set_style_bg_color(lv_screen_active(), lv_color_hex(th->bg), 0);
  if (a_scale) {
    lv_obj_set_style_line_color(a_scale, lv_color_hex(th->dial_minor), LV_PART_ITEMS);
    lv_obj_set_style_line_color(a_scale, lv_color_hex(th->dial_major), LV_PART_INDICATOR);
    lv_obj_set_style_text_color(a_scale, lv_color_hex(th->dial_major), LV_PART_INDICATOR);
    lv_obj_set_style_line_color(a_needle_h, lv_color_hex(th->needle_hm), 0);
    lv_obj_set_style_line_color(a_needle_m, lv_color_hex(th->needle_hm), 0);
  }
}

static void face_apply(void) {
  if (face_mode == 0) {
    lv_obj_remove_flag(face_digital, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(face_analog, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_add_flag(face_digital, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(face_analog, LV_OBJ_FLAG_HIDDEN);
  }
}

// A tap anywhere on the clock face opens the panel. Swipes are detected in
// touchscreen_read() (which stamps last_gesture_ms before this fires), so
// the CLICKED at the end of a swipe is filtered out here.
static void screen_click_cb(lv_event_t * e) {
  LV_UNUSED(e);
  if (millis() - last_gesture_ms < 600) return;
  bl_panel_show();
}

// Pressing the panel background (not the slider) just keeps it alive
static void bl_panel_press_cb(lv_event_t * e) {
  LV_UNUSED(e);
  bl_last_touch_ms = millis();
}

// ---- Touch -> LVGL -------------------------------------------------------
static void touchscreen_read(lv_indev_t * indev, lv_indev_data_t * data) {
  LV_UNUSED(indev);

  static int32_t  last_x = 0, last_y = 0;
  static uint32_t last_ok_ms = 0;
  static bool     pressed = false;
  static int32_t  press_x = 0, press_y = 0;
  static bool     swipe_armed = false;
  static float    flt_x = 0, flt_y = 0;

  int32_t rx = 0, ry = 0;
  int z = xpt_frame(&rx, &ry);
  // Pressure hysteresis: firm to start, light to keep - a stylus tip's low
  // pressure can hold a drag it could never have started.
  bool contact = (z >= (pressed ? TOUCH_Z_RELEASE : TOUCH_Z_PRESS));

  if (contact) {
    // Native 240x320 space. LVGL applies the 270-degree rotation afterwards.
    int32_t x = map(rx, TOUCH_RAW_MIN_X, TOUCH_RAW_MAX_X, 0, SCREEN_WIDTH  - 1);
    int32_t y = map(ry, TOUCH_RAW_MIN_Y, TOUCH_RAW_MAX_Y, 0, SCREEN_HEIGHT - 1);

#if TOUCH_INVERT_X
    x = (SCREEN_WIDTH  - 1) - x;
#endif
#if TOUCH_INVERT_Y
    y = (SCREEN_HEIGHT - 1) - y;
#endif

    x += TOUCH_TRIM_X;
    y += TOUCH_TRIM_Y;

    if (x < 0) x = 0;
    if (x >= SCREEN_WIDTH)  x = SCREEN_WIDTH  - 1;
    if (y < 0) y = 0;
    if (y >= SCREEN_HEIGHT) y = SCREEN_HEIGHT - 1;

    if (!pressed) {
      // New press: remember where it started and seed the position filter.
      // Swipes are ignored while the brightness panel is open, so dragging
      // the slider never flips faces.
      flt_x = (float)x;
      flt_y = (float)y;
      press_x = x;
      press_y = y;
      swipe_armed = (bl_panel == NULL) ||
                    lv_obj_has_flag(bl_panel, LV_OBJ_FLAG_HIDDEN);
    } else {
      // EMA across frames: irons out the remaining wobble during drags
      flt_x += TOUCH_FILTER_ALPHA * ((float)x - flt_x);
      flt_y += TOUCH_FILTER_ALPHA * ((float)y - flt_y);
    }

#if TOUCH_DEBUG
    Serial.printf("z=%d raw=(%ld,%ld) -> native=(%d,%d)\n",
                  z, (long)rx, (long)ry, (int)(flt_x + 0.5f), (int)(flt_y + 0.5f));
#endif

    last_x = (int32_t)(flt_x + 0.5f);
    last_y = (int32_t)(flt_y + 0.5f);
    last_ok_ms = millis();
    pressed = true;
  }
  else if (pressed && (millis() - last_ok_ms) < TOUCH_RELEASE_DEBOUNCE_MS) {
    // Momentary pressure dip mid-drag: hold the last position.
  }
  else {
    if (pressed && swipe_armed && face_digital) {
      // Release edge: displacement-based swipe check, in native coordinates.
      // The display is rotated 270 degrees, so screen-horizontal movement is
      // the native Y axis.
      int32_t dh = last_y - press_y;
      int32_t dv = last_x - press_x;
      if (LV_ABS(dh) >= TOUCH_SWIPE_MIN_PX && LV_ABS(dh) > 2 * LV_ABS(dv)) {
        last_gesture_ms = millis();   // the CLICKED that follows is a swipe tail
        face_mode = !face_mode;
        face_apply();
        prefs.putInt("face", face_mode);
      }
    }
    pressed = false;
  }

  data->point.x = last_x;
  data->point.y = last_y;
  data->state = pressed ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
}

// ---- Width helpers -------------------------------------------------------
static int32_t glyph_width(const lv_font_t * font, uint32_t c) {
  return lv_font_get_glyph_width(font, c, 0);
}

static char widest_digit(const lv_font_t * font) {
  char best = '0';
  int32_t m = 0;
  for (char c = '0'; c <= '9'; c++) {
    int32_t w = glyph_width(font, (uint32_t)c);
    if (w > m) { m = w; best = c; }
  }
  return best;
}

static int32_t max_digit_width(const lv_font_t * font) {
  return glyph_width(font, (uint32_t)widest_digit(font));
}

static int32_t text_width(const lv_font_t * font, const char * txt) {
  lv_point_t p;
  lv_text_get_size(&p, txt, font, 0, 0, LV_COORD_MAX, LV_TEXT_FLAG_NONE);
  return p.x;
}

static int32_t max_weekday_width(const lv_font_t * font) {
  int32_t m = 0;
  for (int i = 0; i < 7; i++) {
    char buf[16];
    snprintf(buf, sizeof(buf), "(%s)", WEEKDAY_KR[i]);
    int32_t w = text_width(font, buf);
    if (w > m) m = w;
  }
  return m;
}

// Transparent, unpadded, non-scrollable container.
// Not clickable, so presses fall through to the screen and open the panel.
static lv_obj_t * make_box(lv_obj_t * parent) {
  lv_obj_t * o = lv_obj_create(parent);
  lv_obj_remove_style_all(o);
  lv_obj_remove_flag(o, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_remove_flag(o, LV_OBJ_FLAG_CLICKABLE);
  return o;
}
// -------------------------------------------------------------------------

static void timer_cb(lv_timer_t * timer) {
  LV_UNUSED(timer);

  static int last_sec  = -1;
  static int last_min  = -1;
  static int last_mday = -1;

  time_t now = time(nullptr);
  struct tm t;
  localtime_r(&now, &t);

  if (t.tm_sec != last_sec) {
    last_sec = t.tm_sec;
    char buf[8];
    strftime(buf, sizeof(buf), "%S", &t);
    lv_label_set_text(label_sec, buf);

    // Needles share the 0..3600 value space (one unit per second): the
    // minute needle creeps with the seconds, the hour needle advances once
    // per minute. Updated even while hidden so a face switch is never stale.
    lv_scale_set_line_needle_value(a_scale, a_needle_s, ANALOG_SEC_LEN,
                                   t.tm_sec * 60);
    lv_scale_set_line_needle_value(a_scale, a_needle_m, ANALOG_MIN_LEN,
                                   t.tm_min * 60 + t.tm_sec);
    lv_scale_set_line_needle_value(a_scale, a_needle_h, ANALOG_HOUR_LEN,
                                   (t.tm_hour % 12) * 300 + t.tm_min * 5);
  }

  if (t.tm_min != last_min) {
    last_min = t.tm_min;
    char buf[8];

    int h12 = t.tm_hour % 12;
    if (h12 == 0) h12 = 12;
    snprintf(buf, sizeof(buf), "%d:%02d", h12, t.tm_min);
    lv_label_set_text(label_hm, buf);

    char ampm[4];
    strftime(ampm, sizeof(ampm), "%p", &t);    // "AM" / "PM"
    lv_label_set_text(label_ampm, ampm);
    lv_label_set_text(label_a_ampm, ampm);
  }

  if (t.tm_mday != last_mday) {
    last_mday = t.tm_mday;
    char buf[16];
    snprintf(buf, sizeof(buf), "%04d-%02d-%02d",
             t.tm_year + 1900, t.tm_mon + 1, t.tm_mday);
    lv_label_set_text(label_datenum, buf);
    snprintf(buf, sizeof(buf), "(%s)", WEEKDAY_KR[t.tm_wday]);
    lv_label_set_text(label_wd, buf);
    lv_label_set_text(label_a_wd, buf);
    lv_color_t wdc = kr_is_red_day(&t) ? WEEKDAY_COLOR_HOLIDAY : WEEKDAY_COLOR_NORMAL;
    lv_obj_set_style_text_color(label_wd, wdc, 0);
    lv_obj_set_style_text_color(label_a_wd, wdc, 0);

    snprintf(buf, sizeof(buf), "%02d-%02d", t.tm_mon + 1, t.tm_mday);
    lv_label_set_text(label_a_date, buf);

    // Bottom line: [holiday/festival name] 음 7.11  입추.
    // Hidden labels are skipped by the flex row, so empty parts leave no gap.
    // Each part is dropped silently once its table runs out of years.
    uint32_t ymd = (uint32_t)(t.tm_year + 1900) * 10000u
                 + (uint32_t)(t.tm_mon + 1) * 100u
                 + (uint32_t)t.tm_mday;

    klc_date_t ld;
    bool have_lunar = klc_solar_to_lunar(&t, &ld);

    const char * ev = kr_holiday_name(ymd);
    lv_color_t evc = EVENT_HOLIDAY_COLOR;
    if (!ev && have_lunar) {
      ev = kr_lunar_festival(&ld);
      evc = EVENT_FESTIVAL_COLOR;
    }
    if (ev) {
      lv_label_set_text(label_lunar_event, ev);
      lv_obj_set_style_text_color(label_lunar_event, evc, 0);
      lv_obj_remove_flag(label_lunar_event, LV_OBJ_FLAG_HIDDEN);
      lv_label_set_text(label_a_event, ev);
      lv_obj_set_style_text_color(label_a_event, evc, 0);
      lv_obj_remove_flag(label_a_event, LV_OBJ_FLAG_HIDDEN);
    } else {
      lv_obj_add_flag(label_lunar_event, LV_OBJ_FLAG_HIDDEN);
      lv_obj_add_flag(label_a_event, LV_OBJ_FLAG_HIDDEN);
    }

    if (have_lunar) {
      char nbuf[12];
      const char * pre = ld.leap ? "음 윤" : "음";
      snprintf(nbuf, sizeof(nbuf), "%d.%d", ld.month, ld.day);
      lv_label_set_text(label_lunar_pre, pre);
      lv_label_set_text(label_lunar_num, nbuf);
      lv_label_set_text(label_a_lunar_pre, pre);
      lv_label_set_text(label_a_lunar_num, nbuf);
      lv_obj_remove_flag(label_lunar_pre, LV_OBJ_FLAG_HIDDEN);
      lv_obj_remove_flag(label_lunar_num, LV_OBJ_FLAG_HIDDEN);
      lv_obj_remove_flag(label_a_lunar_pre, LV_OBJ_FLAG_HIDDEN);
      lv_obj_remove_flag(label_a_lunar_num, LV_OBJ_FLAG_HIDDEN);
    } else {
      lv_obj_add_flag(label_lunar_pre, LV_OBJ_FLAG_HIDDEN);
      lv_obj_add_flag(label_lunar_num, LV_OBJ_FLAG_HIDDEN);
      lv_obj_add_flag(label_a_lunar_pre, LV_OBJ_FLAG_HIDDEN);
      lv_obj_add_flag(label_a_lunar_num, LV_OBJ_FLAG_HIDDEN);
    }

    bool term_today = false;
    const char * term = kst_current_term(&t, &term_today);
    if (term) {
      lv_color_t tc = term_today ? TERM_TODAY_COLOR : LUNAR_COLOR;
      lv_label_set_text(label_lunar_term, term);
      lv_obj_set_style_text_color(label_lunar_term, tc, 0);
      lv_obj_remove_flag(label_lunar_term, LV_OBJ_FLAG_HIDDEN);
      lv_label_set_text(label_a_term, term);
      lv_obj_set_style_text_color(label_a_term, tc, 0);
      lv_obj_remove_flag(label_a_term, LV_OBJ_FLAG_HIDDEN);
    } else {
      lv_obj_add_flag(label_lunar_term, LV_OBJ_FLAG_HIDDEN);
      lv_obj_add_flag(label_a_term, LV_OBJ_FLAG_HIDDEN);
    }
  }

#if AUTO_BL
  auto_bl_update();
#endif

  // Hide the brightness panel once the user stops touching it
  if (bl_panel && !lv_obj_has_flag(bl_panel, LV_OBJ_FLAG_HIDDEN) &&
      (millis() - bl_last_touch_ms) > BL_PANEL_TIMEOUT_MS) {
    lv_obj_add_flag(bl_panel, LV_OBJ_FLAG_HIDDEN);
    bl_store();
  }
}

// ---- Theme swatches on the brightness panel -------------------------------
static void theme_btn_refresh(void) {
  for (int i = 0; i < THEME_COUNT; i++) {
    lv_obj_set_style_border_color(theme_btns[i],
        i == theme_idx ? lv_color_hex(0xFF3300) : lv_color_hex(0x777777), 0);
    lv_obj_set_style_border_width(theme_btns[i], i == theme_idx ? 2 : 1, 0);
  }
}

static void theme_btn_cb(lv_event_t * e) {
  theme_idx = (int)(intptr_t)lv_event_get_user_data(e);
  theme_apply();
  theme_btn_refresh();
  prefs.putInt("theme", theme_idx);
  bl_last_touch_ms = millis();
}

// Floating brightness control, parented to the top layer so it draws over
// the clock without disturbing its layout.
static void create_brightness_panel(void) {
  bl_panel = lv_obj_create(lv_layer_top());
  lv_obj_set_size(bl_panel, 300, 92);
  lv_obj_align(bl_panel, LV_ALIGN_BOTTOM_MID, 0, -8);
  lv_obj_remove_flag(bl_panel, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_bg_color(bl_panel, lv_color_hex(0x181818), 0);
  lv_obj_set_style_bg_opa(bl_panel, LV_OPA_90, 0);
  lv_obj_set_style_border_color(bl_panel, lv_color_hex(0x555555), 0);
  lv_obj_set_style_border_width(bl_panel, 1, 0);
  lv_obj_set_style_radius(bl_panel, 8, 0);
  lv_obj_set_style_pad_all(bl_panel, 8, 0);
  lv_obj_add_event_cb(bl_panel, bl_panel_press_cb, LV_EVENT_PRESSED, NULL);

  // Neither bundled Korean subset covers generic UI text (weekdays and
  // calendar names only), so this label stays numeric.
  bl_pct_label = lv_label_create(bl_panel);
  lv_obj_set_style_text_font(bl_pct_label, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(bl_pct_label, lv_color_hex(0xFFFFFF), 0);
  lv_obj_align(bl_pct_label, LV_ALIGN_TOP_LEFT, 0, -2);

  // Background color swatches in a full-width row below the % label; the
  // current one gets an orange ring. Created after the slider so they win
  // the hit test over its extended click area.
  for (int i = 0; i < THEME_COUNT; i++) {
    lv_obj_t * b = lv_obj_create(bl_panel);
    lv_obj_remove_style_all(b);
    lv_obj_set_size(b, 24, 24);
    lv_obj_align(b, LV_ALIGN_TOP_LEFT, i * 37, 26);
    lv_obj_set_style_radius(b, 6, 0);
    lv_obj_set_style_bg_color(b, lv_color_hex(THEMES[i].bg), 0);
    lv_obj_set_style_bg_opa(b, LV_OPA_COVER, 0);
    lv_obj_set_ext_click_area(b, 6);
    lv_obj_add_event_cb(b, theme_btn_cb, LV_EVENT_CLICKED, (void *)(intptr_t)i);
    theme_btns[i] = b;
  }
  theme_btn_refresh();

  bl_slider = lv_slider_create(bl_panel);
  lv_slider_set_range(bl_slider, BL_MIN_PCT, 100);
  lv_slider_set_value(bl_slider, bl_pct, LV_ANIM_OFF);
  lv_obj_set_size(bl_slider, 264, 14);
  lv_obj_align(bl_slider, LV_ALIGN_BOTTOM_MID, 0, 0);
  lv_obj_set_style_bg_color(bl_slider, lv_color_hex(0x404040), LV_PART_MAIN);
  lv_obj_set_style_bg_color(bl_slider, lv_color_hex(0xFF3300), LV_PART_INDICATOR);
  lv_obj_set_style_bg_color(bl_slider, lv_color_hex(0xFF8855), LV_PART_KNOB);
  // Enlarge the touch area so the knob is easy to grab on a resistive panel
  // (kept small enough not to reach into the swatch row above)
  lv_obj_set_ext_click_area(bl_slider, 16);
  lv_obj_add_event_cb(bl_slider, bl_slider_cb, LV_EVENT_VALUE_CHANGED, NULL);

  lv_obj_add_flag(bl_panel, LV_OBJ_FLAG_HIDDEN);
}

// ============ Analog face: [ dial ][ AM (일) / 08-23 / 음 7.11 / 입추 ] ====
static void create_analog_face(void) {
  face_analog = make_box(lv_screen_active());
  lv_obj_set_size(face_analog, lv_pct(100), lv_pct(100));

  // ---- Dial: a round lv_scale with 60 minute ticks, hour numerals on the
  // majors. Rotation 270 puts value 0 at 12 o'clock. The value space is
  // 0..3600 (one unit per second) rather than 0..60, so the minute and hour
  // needles can creep smoothly between ticks instead of jumping once a
  // minute; the 61 ticks still land on every 60th unit, so the dial itself
  // looks the same.
  a_scale = lv_scale_create(face_analog);
  lv_obj_set_size(a_scale, ANALOG_DIAL_SIZE, ANALOG_DIAL_SIZE);
  lv_obj_align(a_scale, LV_ALIGN_LEFT_MID, 8, 0);
  lv_scale_set_mode(a_scale, LV_SCALE_MODE_ROUND_INNER);
  lv_scale_set_range(a_scale, 0, 3600);
  lv_scale_set_angle_range(a_scale, 360);
  lv_scale_set_rotation(a_scale, 270);
  lv_scale_set_total_tick_count(a_scale, 61);
  lv_scale_set_major_tick_every(a_scale, 5);
  static const char * hour_txts[] = {"12", "1", "2", "3", "4", "5", "6",
                                     "7", "8", "9", "10", "11", NULL};
  lv_scale_set_text_src(a_scale, hour_txts);
  lv_scale_set_label_show(a_scale, true);

  // Tick/numeral/needle colors are theme-dependent and applied by
  // theme_apply() at the end of this function.
  lv_obj_set_style_arc_width(a_scale, 0, LV_PART_MAIN);
  lv_obj_set_style_line_width(a_scale, 1, LV_PART_ITEMS);
  lv_obj_set_style_length(a_scale, 5, LV_PART_ITEMS);
  lv_obj_set_style_line_width(a_scale, 3, LV_PART_INDICATOR);
  lv_obj_set_style_length(a_scale, 9, LV_PART_INDICATOR);
  lv_obj_set_style_text_font(a_scale, &lv_font_montserrat_20, LV_PART_INDICATOR);

  // ---- Needles (children of the scale; lv_scale positions the points)
  a_needle_h = lv_line_create(a_scale);
  lv_obj_set_style_line_width(a_needle_h, 5, 0);
  lv_obj_set_style_line_rounded(a_needle_h, true, 0);

  a_needle_m = lv_line_create(a_scale);
  lv_obj_set_style_line_width(a_needle_m, 3, 0);
  lv_obj_set_style_line_rounded(a_needle_m, true, 0);

  a_needle_s = lv_line_create(a_scale);
  lv_obj_set_style_line_width(a_needle_s, 2, 0);
  lv_obj_set_style_line_color(a_needle_s, lv_color_hex(0xFF3300), 0);

  // Center cap over the needle pivots
  lv_obj_t * cap = lv_obj_create(a_scale);
  lv_obj_remove_style_all(cap);
  lv_obj_remove_flag(cap, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_size(cap, 12, 12);
  lv_obj_set_style_radius(cap, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_color(cap, lv_color_hex(0xFF3300), 0);
  lv_obj_set_style_bg_opa(cap, LV_OPA_COVER, 0);
  lv_obj_center(cap);

  // ---- Info column on the right (~116 px wide)
  static lv_style_t style_a_kr;
  lv_style_init(&style_a_kr);
  lv_style_set_text_font(&style_a_kr, FONT_LUNAR);

  lv_obj_t * col = make_box(face_analog);
  lv_obj_set_size(col, 116, LV_SIZE_CONTENT);
  lv_obj_align(col, LV_ALIGN_RIGHT_MID, -2, 0);
  lv_obj_set_flex_flow(col, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(col, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_row(col, 6, 0);

  // [ AM ][ (일) ]
  lv_obj_t * wdrow = make_box(col);
  lv_obj_set_size(wdrow, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
  lv_obj_set_flex_flow(wdrow, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(wdrow, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_column(wdrow, 6, 0);

  label_a_ampm = lv_label_create(wdrow);
  lv_label_set_text(label_a_ampm, "AM");
  lv_obj_set_style_text_font(label_a_ampm, FONT_AMPM, 0);

  label_a_wd = lv_label_create(wdrow);
  lv_label_set_text(label_a_wd, "(일)");
  lv_obj_set_style_text_font(label_a_wd, FONT_KR, 0);
  lv_obj_set_style_text_color(label_a_wd, WEEKDAY_COLOR_NORMAL, 0);

  label_a_date = lv_label_create(col);
  lv_label_set_text(label_a_date, "01-01");
  lv_obj_set_style_text_font(label_a_date, FONT_DATENUM, 0);

  // [ 음 ][ 7.11 ]
  lv_obj_t * lrow = make_box(col);
  lv_obj_set_size(lrow, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
  lv_obj_set_flex_flow(lrow, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(lrow, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_column(lrow, 4, 0);
  lv_obj_set_style_text_color(lrow, LUNAR_COLOR, 0);

  label_a_lunar_pre = lv_label_create(lrow);
  lv_label_set_text(label_a_lunar_pre, "");
  lv_obj_add_style(label_a_lunar_pre, &style_a_kr, 0);
  lv_obj_set_style_translate_y(label_a_lunar_pre, 3, 0);

  label_a_lunar_num = lv_label_create(lrow);
  lv_label_set_text(label_a_lunar_num, "");
  lv_obj_set_style_text_font(label_a_lunar_num, FONT_LUNAR_NUM_SM, 0);

  label_a_term = lv_label_create(col);
  lv_label_set_text(label_a_term, "");
  lv_obj_add_style(label_a_term, &style_a_kr, 0);
  lv_obj_set_style_text_color(label_a_term, LUNAR_COLOR, 0);
  lv_obj_add_flag(label_a_term, LV_OBJ_FLAG_HIDDEN);

  label_a_event = lv_label_create(col);
  lv_label_set_text(label_a_event, "");
  lv_obj_add_style(label_a_event, &style_a_kr, 0);
  lv_obj_set_width(label_a_event, 116);          // long names wrap
  lv_obj_set_style_text_align(label_a_event, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_add_flag(label_a_event, LV_OBJ_FLAG_HIDDEN);

  theme_apply();   // paint the dial and needles for the restored theme
}

void lv_create_main_gui(void) {

  // 테마 배경 + 주황 세그먼트
  lv_obj_set_style_bg_color(lv_screen_active(), lv_color_hex(THEMES[theme_idx].bg), 0);
  lv_obj_set_style_bg_opa(lv_screen_active(), LV_OPA_COVER, 0);
  lv_obj_set_style_text_color(lv_screen_active(), lv_color_hex(0xFF3300), 0);
  lv_obj_add_event_cb(lv_screen_active(), screen_click_cb, LV_EVENT_CLICKED, NULL);

  // Everything of the digital face hangs off this container so one flag
  // flip swaps the whole face.
  face_digital = make_box(lv_screen_active());
  lv_obj_set_size(face_digital, lv_pct(100), lv_pct(100));

  // ---- Styles
  static lv_style_t style_time;
  lv_style_init(&style_time);
  lv_style_set_text_font(&style_time, FONT_TIME);

  static lv_style_t style_sec;
  lv_style_init(&style_sec);
  lv_style_set_text_font(&style_sec, FONT_SEC);

  static lv_style_t style_ampm;
  lv_style_init(&style_ampm);
  lv_style_set_text_font(&style_ampm, FONT_AMPM);

  static lv_style_t style_datenum;
  lv_style_init(&style_datenum);
  lv_style_set_text_font(&style_datenum, FONT_DATENUM);

  static lv_style_t style_kr;
  lv_style_init(&style_kr);
  lv_style_set_text_font(&style_kr, FONT_KR);

  // ---- Fixed field widths, measured from the fonts themselves.
  // DSEG is a monospaced seven-segment face, so these never change at runtime.
  // The bold face is wider than the regular one, and this picks that up
  // automatically - no constants to retune.

  // The hour field never shows 2-9 in its leading cell: 12-hour time runs
  // 1:00-12:59, so the leading digit is either absent or a '1', whose ink
  // (segments B and C) sits at the far right of its monospaced cell.
  // Reserve only that ink instead of the whole cell, otherwise a one-digit
  // hour leaves a dead zone on the left and the display looks pushed right.
  // The label is right-aligned with LONG_CLIP, so at 10:00-12:59 only the
  // blank left part of the '1' cell is clipped and nothing ever moves.
  lv_font_glyph_dsc_t g1;
  int32_t lead_blank = 0;
  if (lv_font_get_glyph_dsc(FONT_TIME, &g1, '1', 0)) lead_blank = g1.ofs_x;

  const int32_t w_hm    = 4 * max_digit_width(FONT_TIME) + glyph_width(FONT_TIME, ':') + 4
                          - lead_blank;
  const int32_t w_sec   = 2 * max_digit_width(FONT_SEC) + 2;
  const int32_t w_ampm  = LV_MAX(text_width(FONT_AMPM, "AM"),
                                 text_width(FONT_AMPM, "PM")) + 2;
  const int32_t w_col   = LV_MAX(w_sec, w_ampm);
  const int32_t h_time  = lv_font_get_line_height(FONT_TIME);
  const int32_t w_dnum  = 8 * max_digit_width(FONT_DATENUM)
                          + 2 * glyph_width(FONT_DATENUM, '-') + 2;
  const int32_t w_wd    = max_weekday_width(FONT_KR) + 2;

  // Bold digits are wider, so the time row can outgrow the 320 px landscape
  // width. Shout about it on the serial monitor rather than silently clipping.
  Serial.printf("time row width: %ld px (screen %d px)\n",
                (long)(w_hm + w_col + 8), SCREEN_HEIGHT);

  // ================= Date row: [ 2026-08-21 ][ (금) ] =================
  lv_obj_t * date_row = make_box(face_digital);
  lv_obj_set_size(date_row, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
  // Row offsets place the whole block (date / time / lunar line) with equal
  // top and bottom margins: content spans y 28..212 on the 240 px panel,
  // ~28 px of margin on each side. Derived from the font line heights
  // (date 27, time 69, lunar 27) with the inter-row gaps kept as designed.
  lv_obj_align(date_row, LV_ALIGN_CENTER, 0, -78);
  lv_obj_set_flex_flow(date_row, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(date_row, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_column(date_row, 8, 0);

  label_datenum = lv_label_create(date_row);
  lv_label_set_text(label_datenum, "2026-01-01");
  lv_obj_add_style(label_datenum, &style_datenum, 0);
  lv_obj_set_width(label_datenum, w_dnum);
  lv_obj_set_style_text_align(label_datenum, LV_TEXT_ALIGN_CENTER, 0);
  // lv_obj_set_style_text_color(label_datenum, lv_color_hex(0x992000), 0);

  label_wd = lv_label_create(date_row);
  lv_label_set_text(label_wd, "(일)");
  lv_obj_add_style(label_wd, &style_kr, 0);
  lv_obj_set_width(label_wd, w_wd);
  lv_obj_set_style_text_align(label_wd, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_color(label_wd, WEEKDAY_COLOR_NORMAL, 0);
  lv_obj_set_style_translate_y(label_wd, WEEKDAY_BASELINE_NUDGE, 0);

  // ================= Time row: [ HH:MM ][ AM/PM over SS ] =================
  lv_obj_t * time_row = make_box(face_digital);
  lv_obj_set_size(time_row, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
  lv_obj_align(time_row, LV_ALIGN_CENTER, 0, -1);
  lv_obj_set_flex_flow(time_row, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(time_row, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_column(time_row, 8, 0);

  // HH:MM, with the unlit segments painted underneath
  lv_obj_t * hm_box = make_box(time_row);
  lv_obj_set_size(hm_box, w_hm, h_time);

#if SHOW_GHOST_SEGMENTS
  // Same width/align/clip as label_hm so the segments coincide. The blank
  // lead-in of the first 8 gets clipped like the '1' cell above; its left
  // segments (E/F) lose lead_blank pixels - acceptable for a ghost layer.
  lv_obj_t * label_ghost = lv_label_create(hm_box);
  lv_label_set_text(label_ghost, "88:88");
  lv_obj_add_style(label_ghost, &style_time, 0);
  lv_obj_set_style_text_color(label_ghost, GHOST_COLOR, 0);
  lv_obj_set_width(label_ghost, w_hm);
  lv_obj_set_style_text_align(label_ghost, LV_TEXT_ALIGN_RIGHT, 0);
  lv_label_set_long_mode(label_ghost, LV_LABEL_LONG_CLIP);
  lv_obj_center(label_ghost);
#endif

  label_hm = lv_label_create(hm_box);
  lv_label_set_text(label_hm, "12:00");
  lv_obj_add_style(label_hm, &style_time, 0);
  lv_obj_set_width(label_hm, w_hm);
  lv_obj_set_style_text_align(label_hm, LV_TEXT_ALIGN_RIGHT, 0);
  // "10:00".."12:59" is wider than w_hm by exactly the blank lead-in of the
  // '1' cell; clip that instead of letting the label wrap to a second line.
  lv_label_set_long_mode(label_hm, LV_LABEL_LONG_CLIP);
  lv_obj_center(label_hm);

  // Right column: AM/PM pinned to the top, seconds pinned to the bottom
  lv_obj_t * col = make_box(time_row);
  lv_obj_set_size(col, w_col, h_time);
  lv_obj_set_flex_flow(col, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(col, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

  label_ampm = lv_label_create(col);
  lv_label_set_text(label_ampm, "AM");
  lv_obj_add_style(label_ampm, &style_ampm, 0);
  lv_obj_set_width(label_ampm, w_col);
  lv_obj_set_style_text_align(label_ampm, LV_TEXT_ALIGN_CENTER, 0);

  label_sec = lv_label_create(col);
  lv_label_set_text(label_sec, "00");
  lv_obj_add_style(label_sec, &style_sec, 0);
  lv_obj_set_width(label_sec, w_col);
  lv_obj_set_style_text_align(label_sec, LV_TEXT_ALIGN_CENTER, 0);
  // lv_obj_set_style_text_color(label_sec, lv_color_hex(0x992000), 0);

  // ============ Bottom row: [ 음 ][ 7.11 ][ 입추 ] ====================
  // Digits use the same DSEG face/size as the solar date; the Korean parts
  // stay NanumGothic. Flex bottom-aligns the labels, then the Korean ones
  // get the same baseline nudge treatment as the weekday.
  static lv_style_t style_lunar;
  lv_style_init(&style_lunar);
  lv_style_set_text_font(&style_lunar, FONT_LUNAR);

  static lv_style_t style_lunar_num;
  lv_style_init(&style_lunar_num);
  lv_style_set_text_font(&style_lunar_num, FONT_LUNAR_NUM);

  lv_obj_t * lunar_row = make_box(face_digital);
  lv_obj_set_size(lunar_row, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
  lv_obj_align(lunar_row, LV_ALIGN_BOTTOM_MID, 0, -28);
  lv_obj_set_flex_flow(lunar_row, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(lunar_row, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_column(lunar_row, 8, 0);
  lv_obj_set_style_text_color(lunar_row, LUNAR_COLOR, 0);

  label_lunar_event = lv_label_create(lunar_row);
  lv_label_set_text(label_lunar_event, "");
  lv_obj_add_style(label_lunar_event, &style_lunar, 0);
  lv_obj_set_style_translate_y(label_lunar_event, 3, 0);
  lv_obj_set_style_pad_right(label_lunar_event, 6, 0);
  lv_obj_add_flag(label_lunar_event, LV_OBJ_FLAG_HIDDEN);

  label_lunar_pre = lv_label_create(lunar_row);
  lv_label_set_text(label_lunar_pre, "");
  lv_obj_add_style(label_lunar_pre, &style_lunar, 0);
  lv_obj_set_style_translate_y(label_lunar_pre, 3, 0);

  label_lunar_num = lv_label_create(lunar_row);
  lv_label_set_text(label_lunar_num, "");
  lv_obj_add_style(label_lunar_num, &style_lunar_num, 0);

  label_lunar_term = lv_label_create(lunar_row);
  lv_label_set_text(label_lunar_term, "");
  lv_obj_add_style(label_lunar_term, &style_lunar, 0);
  lv_obj_set_style_translate_y(label_lunar_term, 3, 0);
  lv_obj_set_style_pad_left(label_lunar_term, 6, 0);

  create_analog_face();
  face_apply();         // show the face restored from NVS

  create_brightness_panel();
  bl_set_pct(bl_pct);   // syncs the label with the restored value

  // Poll 5x per second so the second boundary is never more than ~200 ms late
  lv_timer_t * timer = lv_timer_create(timer_cb, 200, NULL);
  lv_timer_ready(timer);
}

// ---- Boot state machine ---------------------------------------------------
// The display comes up immediately with a status line; Wi-Fi and the first
// NTP sync happen in the background so a dead router can never leave the
// panel black with no explanation. Wi-Fi retries forever (a clock has
// nothing better to do), visibly counting attempts.
static void sntp_begin(void) {
  sntp_set_time_sync_notification_cb(time_sync_notification_cb);
  sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH);   // slew the clock instead of jumping
  configTzTime(TZ_INFO, "kr.pool.ntp.org", "pool.ntp.org", "time.google.com");
  sntp_set_sync_interval(NTP_SYNC_INTERVAL_MS);
  sntp_restart();                              // apply the new interval
}

static void boot_poll(void) {
  static uint32_t last_ui_ms = 0;
  char buf[64];

  switch (boot_state) {

    case BOOT_WIFI:
      if (WiFi.status() == WL_CONNECTED) {
        Serial.print("Connected to Wi-Fi network with IP Address: ");
        Serial.println(WiFi.localIP());
        sntp_begin();
        boot_t0 = millis();
        lv_label_set_text(boot_label, "Waiting for time sync...");
        boot_state = BOOT_NTP;
        break;
      }
      if (millis() - wifi_attempt_ms > WIFI_RETRY_MS) {
        wifi_attempt_ms = millis();
        wifi_attempts++;
        Serial.printf("Wi-Fi retry #%d\n", wifi_attempts);
        WiFi.disconnect();
        WiFi.begin(ssid, password);
      }
      if (millis() - last_ui_ms > 1000) {
        last_ui_ms = millis();
        snprintf(buf, sizeof(buf), "Connecting to Wi-Fi... %lus (try %d)",
                 (unsigned long)((millis() - boot_t0) / 1000), wifi_attempts);
        lv_label_set_text(boot_label, buf);
      }
      break;

    case BOOT_NTP: {
      struct tm t;
      if (getLocalTime(&t, 0)) {
        lv_obj_delete(boot_label);
        boot_label = NULL;
        lv_create_main_gui();
        boot_state = BOOT_DONE;
        break;
      }
      if (millis() - last_ui_ms > 1000) {
        last_ui_ms = millis();
        snprintf(buf, sizeof(buf), "Waiting for time sync... %lus",
                 (unsigned long)((millis() - boot_t0) / 1000));
        lv_label_set_text(boot_label, buf);
      }
      break;
    }

    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  String LVGL_Arduino = String("LVGL Library Version: ") + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();
  Serial.println(LVGL_Arduino);

  // Restore the last brightness before the panel is even lit
  prefs.begin("clock", false);
  bl_pct = prefs.getInt("bl", BL_DEFAULT);
  bl_saved_pct = bl_pct;
  face_mode = prefs.getInt("face", 0);
  theme_idx = prefs.getInt("theme", 0);
  if (theme_idx < 0 || theme_idx >= THEME_COUNT) theme_idx = 0;

  // Touch controller on its own SPI bus, driven directly (see xpt_frame)
  touchscreenSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
  pinMode(XPT2046_CS, OUTPUT);
  digitalWrite(XPT2046_CS, HIGH);

  // Start LVGL
  lv_init();
  // Let LVGL read real elapsed time instead of being fed a fixed 5 ms per loop
  lv_tick_set_cb((lv_tick_get_cb_t)millis);
  lv_log_register_print_cb(log_print);

  // Create a display object
  lv_display_t * disp;
  disp = lv_tft_espi_create(SCREEN_WIDTH, SCREEN_HEIGHT, draw_buf, sizeof(draw_buf));
  lv_display_set_rotation(disp, LV_DISPLAY_ROTATION_270);

  // After the display init, so TFT_eSPI's own digitalWrite(TFT_BL, HIGH)
  // does not fight the PWM channel.
  bl_begin();

#if AUTO_BL
  analogSetPinAttenuation(LDR_PIN, ADC_11db);
#endif

  lv_indev_t * indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, touchscreen_read);

  // Boot status screen, replaced by the clock once time is known.
  // Uses the restored theme so a dark clock does not boot blinding white.
  lv_obj_set_style_bg_color(lv_screen_active(), lv_color_hex(THEMES[theme_idx].bg), 0);
  lv_obj_set_style_bg_opa(lv_screen_active(), LV_OPA_COVER, 0);
  boot_label = lv_label_create(lv_screen_active());
  lv_obj_set_style_text_font(boot_label, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(boot_label, lv_color_hex(THEMES[theme_idx].dial_major), 0);
  lv_label_set_text(boot_label, "Connecting to Wi-Fi...");
  lv_obj_center(boot_label);

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid, password);
  boot_t0 = wifi_attempt_ms = millis();
}

void loop() {
  lv_task_handler();  // let the GUI do its work
  if (boot_state != BOOT_DONE) boot_poll();
  delay(5);
}
