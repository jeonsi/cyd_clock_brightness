/*  Based on Rui Santos & Sara Santos - Random Nerd Tutorials
    https://RandomNerdTutorials.com/esp32-cyd-lvgl-digital-clock/

    MODIFIED:
      - time kept by the ESP32 system clock, disciplined by SNTP
      - slanted seven-segment (DSEG Italic) digits for the time, seconds and date
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
      - a consistent raised, top-lit look on every face: bezel + shadows
        on the analog dial, a card plate behind the digital / calendar /
        stopwatch / timer content, embossed HH:MM, raised buttons
      - analog face (one large centered lv_scale dial + line needles),
        and a monthly calendar face (holiday-aware, today
        highlighted); swipe left/right to cycle faces, choice kept in NVS
      - stopwatch and countdown-timer faces; the timer rings the CYD's
        speaker (GPIO 26) and flashes the screen until tapped
      - wake-up alarms (up to ALARM_COUNT, each with weekday selection,
        one-shot mode, H/M buttons with hold-to-repeat, ON/OFF, kept in
        NVS); ring like the timer, bell shown on the clock faces
      - 12 / 24-hour format toggle on the brightness panel, kept in NVS
      - background color selectable from swatches on the brightness panel
        (white / ivory / dark gray / black, THEMES table, kept in NVS)
      - every field has a fixed width so nothing shifts when the digits change
      - touch the screen to bring up a backlight brightness slider
        (XPT2046 touch + LEDC PWM on the backlight pin, value kept in NVS)

    REQUIRED FILES in the same folder as this .ino:
      clock_fonts.h     - DSEG7 68/30/26 px + DSEG14 20 px + NanumGothic 26 px
                          AND font_dseg_bold_68 (see below)
      lunar_font.h      - NanumGothic 22 px subset + DSEG 26 px with '.'
      korean_calendar.h - lunar calendar, solar terms, holiday tables
      secrets.h         - Wi-Fi credentials (copy secrets.h.example)
      clock_config.h    - all tunables (timezone, colors, themes, touch
                          calibration, sounds, ...) with per-item notes

    The bold face is a new addition. Generate it with:

      npx lv_font_conv \
        --font DSEG7Classic-BoldItalic.ttf \
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
#include "clock_config.h"   // every tunable, with notes on what each does

// Wi-Fi credentials live in secrets.h (gitignored).
// Copy secrets.h.example to secrets.h and fill in your own.
#include "secrets.h"
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

// Panel size in its native, unrotated orientation
#define SCREEN_WIDTH  240
#define SCREEN_HEIGHT 320

#define DRAW_BUF_SIZE (SCREEN_WIDTH * SCREEN_HEIGHT / 10 * (LV_COLOR_DEPTH / 8))
uint32_t draw_buf[DRAW_BUF_SIZE / 4];

// ======================= Backlight (PWM) =================================
// TFT_eSPI's User_Setup for the CYD defines TFT_BL (GPIO 21 on the
// ESP32-2432S028R). Fall back to 21 if it is not defined.
#ifdef TFT_BL
  #define BL_PIN TFT_BL
#else
  #define BL_PIN 21
#endif

#define BL_CHANNEL   0      // only used on ESP32 Arduino core 2.x
// BL_FREQ / BL_RES / BL_MIN_PCT / BL_DEFAULT / BL_PANEL_TIMEOUT_MS: clock_config.h

// ======================= Ambient light (LDR) ==============================
// The CYD has a photoresistor on GPIO 34 (input-only, ADC1). It scales the
// user's brightness setting: the slider sets the ceiling, ambient light
// decides how much of it is used, down to BL_AUTO_MIN_PCT in full darkness.
// While the slider panel is open the scaling is suspended (factor 100%) so
// the user sees the true range they are setting.
#define LDR_PIN          34
// AUTO_BL / LDR_RAW_* / BL_AUTO_MIN_PCT / LDR_DEBUG: clock_config.h

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

// Calibration (TOUCH_RAW_*), axis flags, trims, debounce, pressure
// thresholds, filtering, swipe threshold and TOUCH_DEBUG: clock_config.h.
//
// Coordinate contract: LVGL 9 rotates pointer coordinates itself, inside
// indev_pointer_proc(), to match the display rotation, so the read callback
// reports coordinates in the panel's NATIVE 240x320 space and never swaps
// axes - only the two TOUCH_INVERT_* direction flags apply.
//
// Swipes are detected in touchscreen_read() rather than via LVGL gestures:
// press events stop at clickable widgets (the analog dial) and don't bubble
// to the screen, and LVGL's gesture detector needs an instantaneous release
// velocity that pressure wobble on a resistive panel rarely provides. A
// swipe is pure displacement between press and release on the UNFILTERED
// coordinates; anything past TOUCH_SWIPE_MIN_PX is a gesture (larger axis
// decides face switch vs calendar month), less is a tap.

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

// ======================= Depth / 3-D look ================================
// Every face uses the same lighting: light from the top, shadows falling
// to the bottom-right. Raised elements (the analog bezel, the card behind
// each face's content, buttons, the brightness panel) get a light-to-dark
// vertical gradient plus a soft drop shadow; the analog face and the
// digital HH:MM get offset shadow copies for extra relief. Radial
// gradients are not enabled in this LVGL build, so it is all linear +
// shadow. Card and bezel tones derive from the theme background in
// theme_apply(). Shadow sizes, colors, analog geometry and the THEMES
// table are in clock_config.h.
#define MAX_CARDS             4    // plates registered for theme recoloring

static lv_obj_t * label_ampm;
static lv_obj_t * label_hm;
static lv_obj_t * label_sec;
static lv_obj_t * label_datenum;
static lv_obj_t * label_wd;
static lv_obj_t * hm_box;            // HH:MM cell, resized on the 12/24 h toggle
static lv_obj_t * label_hm_sh;       // emboss shadow under HH:MM
static lv_obj_t * cards[MAX_CARDS];  // raised plates behind face content
static int        card_count = 0;
static lv_obj_t * label_ghost;       // ghost "88:88" (SHOW_GHOST_SEGMENTS) or NULL
static lv_obj_t * lbl_h24;           // "12H" / "24H" on the brightness panel
static bool       h24 = false;       // 24-hour format (kept in NVS)
static bool       time_fmt_dirty = false;
static int32_t    w_hm_12 = 0, w_hm_24 = 0;
static lv_obj_t * label_lunar_event; // holiday / festival name
static lv_obj_t * label_lunar_pre;   // "음" / "음 윤"
static lv_obj_t * label_lunar_num;   // "7.11" in DSEG
static lv_obj_t * label_lunar_term;  // current solar term

// The faces are full-screen sibling containers; exactly one is visible.
// Swiping left/right cycles digital -> analog -> calendar -> stopwatch ->
// timer -> alarm -> digital. On the calendar face, swiping up/down browses to the
// next/previous month; leaving the face snaps back to the current month.
#define FACE_COUNT 6
#define FACE_CAL   2
#define FACE_SW    3
#define FACE_TM    4
#define FACE_ALARM 5

// ======================= Stopwatch / Timer ================================
// Buttons use LVGL's built-in FontAwesome symbols (play/pause/refresh), so
// no extra font glyphs are needed. The countdown timer beeps and flashes an
// overlay until tapped or TIMER_ALARM_MS passes. The hourly chime is a
// Casio-style pip-pip played with blocking delays (~180 ms, once an hour)
// because its 60 ms steps are finer than the UI tick.
// SPK_PIN (26 = on-board amp / 27 = piezo on CN1), tones, alarm timeout,
// BOOT_BEEP, SPK_TEST and the chime settings: clock_config.h.
#define SPK_CHANNEL     2                        // ESP32 Arduino core 2.x only
static lv_obj_t * face_digital;
static lv_obj_t * face_analog;
static lv_obj_t * face_cal;
static lv_obj_t * label_c_title;      // "2026.8"
static lv_obj_t * label_c_wd[7];      // 일..토 header
static lv_obj_t * label_c_day[42];    // 6 rows x 7 columns
static int        cal_off = 0;        // viewed month, relative to current

static lv_obj_t * face_sw;            // stopwatch
static lv_obj_t * label_sw_time;      // "12:34"
static lv_obj_t * label_sw_frac;      // ".7" tenths
static lv_obj_t * lbl_sw_start;       // play/pause symbol on the start button
static bool       sw_running = false;
static uint32_t   sw_accum_ms = 0;    // accumulated while stopped
static uint32_t   sw_t0 = 0;          // millis() at the last start
static lv_timer_t * sw_fast_timer;    // hundredths refresh, runs only while needed
static lv_obj_t * row_sw_lap[SW_LAPS];    // lap lines (number + time), oldest first
static lv_obj_t * label_sw_lap_no[SW_LAPS];
static lv_obj_t * label_sw_lap[SW_LAPS];  // lap (interval) time
static lv_obj_t * label_sw_lap_cum[SW_LAPS]; // cumulative time at that lap
static uint32_t   sw_lap_ms[SW_LAPS];     // lap (interval) times, oldest first
static uint32_t   sw_lap_split[SW_LAPS];  // elapsed total when each lap was taken
static int        sw_lap_no[SW_LAPS];     // lap numbers matching sw_lap_ms
static int        sw_lap_shown = 0;       // how many lines are filled
static int        sw_lap_count = 0;       // laps taken since reset
static uint32_t   sw_last_split = 0;      // elapsed at the previous lap

static lv_obj_t * face_tm;            // countdown timer
static lv_obj_t * label_tm_time;      // "05:00"
static lv_obj_t * lbl_tm_start;
static lv_obj_t * alarm_overlay;
static bool       tm_running = false;
static uint32_t   tm_left_ms = 0;
static uint32_t   tm_last_ms = 0;     // millis() of the previous countdown tick
static bool       alarm_on = false;
static uint32_t   alarm_t0 = 0;
static uint32_t   alarm_limit_ms = 0;   // ring at most this long (timer vs wake alarm)

static lv_obj_t * face_alarm;           // wake-up alarm clock
static lv_obj_t * label_al_time;        // "07:30" in the big DSEG face
static lv_obj_t * label_al_ampm;
static lv_obj_t * lbl_al_toggle;        // bell + ON/OFF on the toggle button
static lv_obj_t * bell_box[2][ALARM_COUNT]; // per-alarm bell icons: [0] digital, [1] analog
// alarm_t itself is declared in clock_config.h: the Arduino IDE inserts
// auto-generated function prototypes right after the #includes, and a
// prototype taking alarm_t* needs the type to exist by then.
static alarm_t    alarms[ALARM_COUNT];   // all kept in NVS
static int        al_sel = 0;            // alarm being edited on the face
static lv_obj_t * btn_al_sel[ALARM_COUNT];
static lv_obj_t * btn_al_day[7];
static lv_obj_t * btn_al_once;
// Weekday buttons run Monday..Sunday; entry i shows tm_wday AL_DAY_ORDER[i].
static const int  AL_DAY_ORDER[7] = { 1, 2, 3, 4, 5, 6, 0 };
static bool       al_dirty = false;      // NVS write pending
static uint32_t   al_dirty_ms = 0;
static lv_obj_t * a_scale;
static lv_obj_t * a_bezel;            // raised rim behind the dial
static lv_obj_t * a_face;             // dial face inside the rim
static lv_obj_t * a_cap;              // center cap over the pivots
static lv_obj_t * a_needle_h;
static lv_obj_t * a_needle_m;
static lv_obj_t * a_needle_s;
static lv_obj_t * a_needle_h_sh;      // shadow copies, drawn under the needles
static lv_obj_t * a_needle_m_sh;
static lv_obj_t * a_needle_s_sh;

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

static void bl_panel_hide(void) {
  lv_obj_add_flag(bl_panel, LV_OBJ_FLAG_HIDDEN);
  bl_store();
}

static void bl_slider_cb(lv_event_t * e) {
  LV_UNUSED(e);
  bl_set_pct(lv_slider_get_value(bl_slider));
  bl_last_touch_ms = millis();
}

static bool theme_is_dark(void) {
  uint32_t bg = THEMES[theme_idx].bg;
  uint32_t r = (bg >> 16) & 0xFF, g = (bg >> 8) & 0xFF, b = bg & 0xFF;
  return (299 * r + 587 * g + 114 * b) / 1000 < 128;
}

// Repaint the calendar face: month title, weekday header and the 6x7 day
// grid. The viewed month is the current one shifted by cal_off. Sundays
// and public holidays are red, Saturdays blue; today sits on an orange pad
// with white text, and only when the real current month is shown. Colors
// are picked per theme lightness, so this also runs on theme changes.
static void cal_refresh(const struct tm * t) {
  if (!face_cal) return;

  int mt  = (t->tm_year + 1900) * 12 + t->tm_mon + cal_off;
  int y   = mt / 12;
  int mon = mt % 12;                    // 0-based
  bool this_month = (cal_off == 0);

  char buf[16];
  snprintf(buf, sizeof(buf), "%04d.%d", y, mon + 1);
  lv_label_set_text(label_c_title, buf);

  lv_color_t ink  = lv_color_hex(THEMES[theme_idx].dial_major);
  lv_color_t red  = lv_color_hex(theme_is_dark() ? 0xFF5544 : 0xE60000);
  lv_color_t blue = lv_color_hex(theme_is_dark() ? 0x86B7F5 : 0x2B6CB0);

  for (int i = 0; i < 7; i++) {
    lv_obj_set_style_text_color(label_c_wd[i],
        i == 0 ? red : (i == 6 ? blue : ink), 0);
  }

  static const int MDAYS[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  int dim = MDAYS[mon];
  if (mon == 1 && (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0))) dim = 29;
  // 1970-01-01 (day 0 of the epoch) was a Thursday
  int first_wd = (int)((klc_days_from_civil(y, mon + 1, 1) + 4) % 7);

  for (int i = 0; i < 42; i++) {
    int d = i - first_wd + 1;
    if (d < 1 || d > dim) {
      lv_label_set_text(label_c_day[i], "");
      lv_obj_set_style_bg_opa(label_c_day[i], LV_OPA_TRANSP, 0);
      lv_obj_set_style_shadow_opa(label_c_day[i], LV_OPA_TRANSP, 0);
      continue;
    }
    char db[4];
    snprintf(db, sizeof(db), "%d", d);
    lv_label_set_text(label_c_day[i], db);
    if (this_month && d == t->tm_mday) {
      lv_obj_set_style_bg_opa(label_c_day[i], LV_OPA_COVER, 0);
      lv_obj_set_style_shadow_opa(label_c_day[i], LV_OPA_40, 0);
      lv_obj_set_style_text_color(label_c_day[i], lv_color_hex(0xFFFFFF), 0);
    } else {
      uint32_t ymd = (uint32_t)y * 10000u + (uint32_t)(mon + 1) * 100u + (uint32_t)d;
      int wd = i % 7;
      lv_obj_set_style_bg_opa(label_c_day[i], LV_OPA_TRANSP, 0);
      lv_obj_set_style_shadow_opa(label_c_day[i], LV_OPA_TRANSP, 0);
      lv_obj_set_style_text_color(label_c_day[i],
          (wd == 0 || kr_holiday_name(ymd)) ? red : (wd == 6 ? blue : ink), 0);
    }
  }
}

static void cal_refresh_now(void) {
  if (!face_cal) return;
  time_t now = time(nullptr);
  struct tm t;
  localtime_r(&now, &t);
  cal_refresh(&t);
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

    // Bezel and face tones are derived from the background so the disc
    // looks like the same material as the panel, just raised. Light themes
    // get a face a touch brighter than the background; dark themes a touch
    // lighter too, since the rim's highlight needs something to sit on.
    lv_color_t bg = lv_color_hex(th->bg);
    bool dark = theme_is_dark();
    lv_color_t rim_hi = lv_color_lighten(bg, dark ? 90 : 60);
    lv_color_t rim_lo = lv_color_darken(bg,  dark ? 40 : 90);
    lv_color_t face_c = dark ? lv_color_lighten(bg, 22) : lv_color_lighten(bg, 40);
    lv_obj_set_style_bg_color(a_bezel, rim_hi, 0);
    lv_obj_set_style_bg_grad_color(a_bezel, rim_lo, 0);
    // Face: slightly shaded at the top under the rim, lighter toward the
    // bottom - the inverse of the rim, which is what sells the inset.
    lv_obj_set_style_bg_color(a_face, lv_color_darken(face_c, dark ? 20 : 12), 0);
    lv_obj_set_style_bg_grad_color(a_face, lv_color_lighten(face_c, dark ? 10 : 20), 0);
    lv_obj_set_style_shadow_color(a_bezel, lv_color_black(), 0);
  }

  // Cards behind the other faces: same material as the panel, raised.
  {
    lv_color_t bg = lv_color_hex(th->bg);
    bool dark = theme_is_dark();
    lv_color_t hi = lv_color_lighten(bg, dark ? 40 : 30);
    lv_color_t lo = lv_color_darken(bg,  dark ? 25 : 30);
    for (int i = 0; i < card_count; i++) {
      lv_obj_set_style_bg_color(cards[i], hi, 0);
      lv_obj_set_style_bg_grad_color(cards[i], lo, 0);
    }
  }
  cal_refresh_now();   // calendar ink is theme-dependent
}

// The hundredths digits need a much faster refresh than the 100 ms UI tick
// to roll smoothly, so they get their own lv_timer. It only runs while the
// stopwatch is running AND its face is on screen; otherwise it is paused
// and costs nothing.
static void sw_fast_sync(void) {
  if (!sw_fast_timer) return;
  bool on = sw_running && face_sw && !lv_obj_has_flag(face_sw, LV_OBJ_FLAG_HIDDEN);
  if (on) lv_timer_resume(sw_fast_timer);
  else    lv_timer_pause(sw_fast_timer);
}

static void face_apply(void) {
  lv_obj_t * faces[FACE_COUNT] = { face_digital, face_analog, face_cal,
                                   face_sw, face_tm, face_alarm };
  for (int i = 0; i < FACE_COUNT; i++) {
    if (i == face_mode) lv_obj_remove_flag(faces[i], LV_OBJ_FLAG_HIDDEN);
    else                lv_obj_add_flag(faces[i], LV_OBJ_FLAG_HIDDEN);
  }
  sw_fast_sync();
}

// ---- Stopwatch -------------------------------------------------------------
static uint32_t sw_elapsed_ms(void) {
  return sw_accum_ms + (sw_running ? millis() - sw_t0 : 0);
}

static void sw_update_label(void) {
  uint32_t ms = sw_elapsed_ms();
  uint32_t s  = ms / 1000;
  char buf[8];
  snprintf(buf, sizeof(buf), "%02lu:%02lu",
           (unsigned long)((s / 60) % 100), (unsigned long)(s % 60));
  lv_label_set_text(label_sw_time, buf);
  snprintf(buf, sizeof(buf), ".%02lu", (unsigned long)((ms / 10) % 100));
  lv_label_set_text(label_sw_frac, buf);
}

static void sw_fmt_ms(char * buf, size_t n, uint32_t ms) {
  uint32_t sec = ms / 1000;
  snprintf(buf, n, "%02lu:%02lu.%02lu",
           (unsigned long)((sec / 60) % 100), (unsigned long)(sec % 60),
           (unsigned long)((ms / 10) % 100));
}

// Lap lines: [N] [lap time] [cumulative time] with fixed gaps, filled top
// to bottom in the order taken; unused lines are hidden.
static void sw_lap_refresh(void) {
  for (int i = 0; i < SW_LAPS; i++) {
    if (i < sw_lap_shown) {
      char buf[16];
      snprintf(buf, sizeof(buf), "%d", sw_lap_no[i]);
      lv_label_set_text(label_sw_lap_no[i], buf);
      sw_fmt_ms(buf, sizeof(buf), sw_lap_ms[i]);
      lv_label_set_text(label_sw_lap[i], buf);
      sw_fmt_ms(buf, sizeof(buf), sw_lap_split[i]);
      lv_label_set_text(label_sw_lap_cum[i], buf);
      lv_obj_remove_flag(row_sw_lap[i], LV_OBJ_FLAG_HIDDEN);
    } else {
      lv_obj_add_flag(row_sw_lap[i], LV_OBJ_FLAG_HIDDEN);
    }
  }
}

// Record a lap at the current elapsed time: the interval since the previous
// lap (or since start) plus the cumulative total. New laps are appended at
// the bottom; once SW_LAPS lines are full the oldest scrolls off the top.
// Laps are numbered from 1. Used by the LAP button and by Stop, so the
// final segment is captured too.
static void sw_lap_record(void) {
  uint32_t now_ms = sw_elapsed_ms();
  if (sw_lap_shown == SW_LAPS) {
    for (int i = 0; i < SW_LAPS - 1; i++) {
      sw_lap_ms[i]    = sw_lap_ms[i + 1];
      sw_lap_split[i] = sw_lap_split[i + 1];
      sw_lap_no[i]    = sw_lap_no[i + 1];
    }
  } else {
    sw_lap_shown++;
  }
  int k = sw_lap_shown - 1;
  sw_lap_ms[k]    = now_ms - sw_last_split;
  sw_lap_split[k] = now_ms;
  sw_lap_no[k]    = ++sw_lap_count;
  sw_last_split = now_ms;
  sw_lap_refresh();
}

static void sw_lap_cb(lv_event_t * e) {
  LV_UNUSED(e);
  if (millis() - last_gesture_ms < 600) return;
  if (!sw_running) return;   // laps only make sense while running
  sw_lap_record();
}

static void sw_start_cb(lv_event_t * e) {
  LV_UNUSED(e);
  if (millis() - last_gesture_ms < 600) return;   // swipe tail, not a press
  if (sw_running) {
    sw_accum_ms += millis() - sw_t0;
    sw_running = false;
    sw_lap_record();   // the final segment becomes the last lap
  } else {
    sw_t0 = millis();
    sw_running = true;
  }
  lv_label_set_text(lbl_sw_start, sw_running ? LV_SYMBOL_PAUSE : LV_SYMBOL_PLAY);
  sw_update_label();   // freeze the exact value on stop
  sw_fast_sync();
}

static void sw_reset_cb(lv_event_t * e) {
  LV_UNUSED(e);
  if (millis() - last_gesture_ms < 600) return;
  sw_running = false;
  sw_accum_ms = 0;
  sw_lap_shown = 0;
  sw_lap_count = 0;
  sw_last_split = 0;
  lv_label_set_text(lbl_sw_start, LV_SYMBOL_PLAY);
  sw_update_label();
  sw_lap_refresh();
  sw_fast_sync();
}

// ---- Countdown timer -------------------------------------------------------
static void spk_tone(uint32_t hz) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWriteTone(SPK_PIN, hz);
#else
  ledcWriteTone(SPK_CHANNEL, hz);
#endif
}

static void tm_update_label(void) {
  // Round up so a freshly set 5:00 shows 5:00 until it has really elapsed
  uint32_t s = (tm_left_ms + 999) / 1000;
  char buf[8];
  snprintf(buf, sizeof(buf), "%02lu:%02lu",
           (unsigned long)(s / 60), (unsigned long)(s % 60));
  lv_label_set_text(label_tm_time, buf);
}

static void alarm_stop(void) {
  alarm_on = false;
  spk_tone(0);
  if (alarm_overlay) lv_obj_add_flag(alarm_overlay, LV_OBJ_FLAG_HIDDEN);
}

// Write every alarm to NVS (only called when something actually changed).
static void al_save_now(void) {
  al_dirty = false;
  char k[8];
  for (int i = 0; i < ALARM_COUNT; i++) {
    snprintf(k, sizeof(k), "a%d_h", i);  prefs.putInt(k, alarms[i].hh);
    snprintf(k, sizeof(k), "a%d_m", i);  prefs.putInt(k, alarms[i].mm);
    snprintf(k, sizeof(k), "a%d_on", i); prefs.putInt(k, alarms[i].enabled ? 1 : 0);
    snprintf(k, sizeof(k), "a%d_d", i);  prefs.putInt(k, alarms[i].days);
    snprintf(k, sizeof(k), "a%d_1", i);  prefs.putInt(k, alarms[i].once ? 1 : 0);
  }
}

static void alarm_start(uint32_t limit_ms) {
  alarm_on = true;
  alarm_t0 = millis();
  alarm_limit_ms = limit_ms;
  if (alarm_overlay) lv_obj_remove_flag(alarm_overlay, LV_OBJ_FLAG_HIDDEN);
}

static void alarm_overlay_cb(lv_event_t * e) {
  LV_UNUSED(e);
  alarm_stop();
}

static void tm_add_cb(lv_event_t * e) {
  if (millis() - last_gesture_ms < 600) return;
  uint32_t add_min = (uint32_t)(intptr_t)lv_event_get_user_data(e);
  tm_left_ms += add_min * 60000u;
  if (tm_left_ms > TIMER_MAX_MS) tm_left_ms = TIMER_MAX_MS;
  tm_update_label();
}

static void tm_start_cb(lv_event_t * e) {
  LV_UNUSED(e);
  if (millis() - last_gesture_ms < 600) return;
  if (!tm_running && tm_left_ms == 0) return;   // nothing to count down
  tm_running = !tm_running;
  if (tm_running) tm_last_ms = millis();
  lv_label_set_text(lbl_tm_start, tm_running ? LV_SYMBOL_PAUSE : LV_SYMBOL_PLAY);
}

static void tm_reset_cb(lv_event_t * e) {
  LV_UNUSED(e);
  if (millis() - last_gesture_ms < 600) return;
  tm_running = false;
  tm_left_ms = 0;
  lv_label_set_text(lbl_tm_start, LV_SYMBOL_PLAY);
  tm_update_label();
}

// Called every UI tick: advances the countdown (even while the face is
// hidden), drives the alarm blink/beep, and repaints whichever of the two
// time displays is on screen.
static void sw_tm_tick(void) {
  if (tm_running) {
    uint32_t now = millis();
    uint32_t d = now - tm_last_ms;
    tm_last_ms = now;
    if (tm_left_ms > d) {
      tm_left_ms -= d;
    } else {
      tm_left_ms = 0;
      tm_running = false;
      lv_label_set_text(lbl_tm_start, LV_SYMBOL_PLAY);
      alarm_start(TIMER_ALARM_MS);
    }
    tm_update_label();
  }

  if (alarm_on) {
    bool phase = ((millis() - alarm_t0) / 400) & 1;
    spk_tone(phase ? ALARM_TONE_HZ : 0);
    lv_obj_set_style_bg_opa(alarm_overlay, phase ? LV_OPA_50 : LV_OPA_10, 0);
    if (millis() - alarm_t0 > alarm_limit_ms) alarm_stop();
  }

  // Wake-alarm settings are written to NVS a couple of seconds after the
  // last change (or at once by the OK button), so holding an adjust button
  // does not hammer the flash.
  if (al_dirty && millis() - al_dirty_ms > 2000) al_save_now();

  // (the running stopwatch display is repainted by sw_fast_timer)
}

// A tap anywhere on the clock face toggles the panel: open when hidden,
// and - after an accidental tap - a second tap outside the panel dismisses
// it immediately instead of waiting out the 4 s timeout. Taps ON the panel
// only keep it alive (bl_panel_press_cb). Swipes are detected in
// touchscreen_read() (which stamps last_gesture_ms before this fires), so
// the CLICKED at the end of a swipe is filtered out here.
static void screen_click_cb(lv_event_t * e) {
  LV_UNUSED(e);
  if (millis() - last_gesture_ms < 600) return;
  if (face_mode == FACE_ALARM) return;   // editing screen: background taps do nothing
  if (bl_panel && !lv_obj_has_flag(bl_panel, LV_OBJ_FLAG_HIDDEN)) {
    bl_panel_hide();
  } else {
    bl_panel_show();
  }
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
  static int32_t  raw_last_x = 0, raw_last_y = 0;   // unfiltered, for swipes
  static int32_t  peak_dh = 0, peak_dv = 0;         // largest RAW travel this press (swipes)
  static int32_t  peak_flt = 0;                     // largest FILTERED travel (tap test)
  static int      confirm_left = -1;                // press-confirmation frames still to skip
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

    // Press confirmation: the first contact frame(s) come from the
    // low-pressure onset and their coordinates often spike onto a
    // neighbouring button, so they are skipped and the press point is taken
    // from the first stable frame after them. LVGL still sees "released"
    // meanwhile; a contact that vanishes before confirming was just a blip.
    if (!pressed) {
      if (confirm_left < 0) confirm_left = TOUCH_PRESS_CONFIRM_FRAMES;
      if (confirm_left > 0) {
        confirm_left--;
        data->point.x = last_x;
        data->point.y = last_y;
        data->state = LV_INDEV_STATE_RELEASED;
        return;
      }
    }

    raw_last_x = x;
    raw_last_y = y;
    if (pressed) {
      // Track the farthest point reached, not just the last sample: a
      // finger often bounces back a little as it lifts. Raw travel feeds
      // the swipe test (fast flicks must not be understated by the EMA).
      int32_t dh_now = y - press_y, dv_now = x - press_x;
      if (LV_ABS(dh_now) > LV_ABS(peak_dh)) peak_dh = dh_now;
      if (LV_ABS(dv_now) > LV_ABS(peak_dv)) peak_dv = dv_now;
    }

    if (!pressed) {
      // New press: remember where it started and seed the position filter.
      // Swipes are ignored while the brightness panel is open, so dragging
      // the slider never flips faces.
      flt_x = (float)x;
      flt_y = (float)y;
      press_x = x;
      press_y = y;
      peak_dh = peak_dv = 0;
      peak_flt = 0;
      swipe_armed = (bl_panel == NULL) ||
                    lv_obj_has_flag(bl_panel, LV_OBJ_FLAG_HIDDEN);
    } else {
      // EMA across frames: irons out the remaining wobble during drags
      flt_x += TOUCH_FILTER_ALPHA * ((float)x - flt_x);
      flt_y += TOUCH_FILTER_ALPHA * ((float)y - flt_y);
      // Filtered travel feeds the tap test: a single-sample spike at press
      // or release (common on a resistive panel) barely moves the EMA, so
      // it cannot turn a real tap into "ambiguous movement".
      int32_t fh = (int32_t)(flt_y + 0.5f) - press_y;
      int32_t fv = (int32_t)(flt_x + 0.5f) - press_x;
      int32_t ft = LV_MAX(LV_ABS(fh), LV_ABS(fv));
      if (ft > peak_flt) peak_flt = ft;
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
  else if (!pressed && confirm_left >= 0 && confirm_left < TOUCH_PRESS_CONFIRM_FRAMES &&
           pend_z >= TOUCH_Z_SHORT_TAP) {
    // The contact ended before confirmation, i.e. a very short tap that was
    // above threshold for a single frame. Don't lose it: press at that
    // frame's point now; the release debounce below turns it into a normal
    // press/release pair for LVGL a moment later. Only for a FIRM single
    // frame (pend_z >= TOUCH_Z_SHORT_TAP): a light one-frame blip is noise
    // and would be a ghost touch.
    flt_x = (float)pend_x;
    flt_y = (float)pend_y;
    last_x = pend_x;
    last_y = pend_y;
    press_x = pend_x;
    press_y = pend_y;
    peak_dh = peak_dv = peak_flt = max_step = 0;
    swipe_armed = false;         // too short to be a gesture anyway
    confirm_left = -1;
    last_ok_ms = millis();
    pressed = true;
    dispatched = true;
    gesture_mode = false;
  }
  else if (pressed && (millis() - last_ok_ms) < TOUCH_RELEASE_DEBOUNCE_MS) {
    // Momentary pressure dip mid-drag: hold the last position. A tap that
    // ended before the hold window ran out is delivered now, so LVGL sees a
    // press during the debounce and the release right after it.
    if (!dispatched && !gesture_mode) dispatched = true;
  }
  else {
    // Presses that began on a clickable widget (a button, the slider) belong
    // to that widget: no gesture classification for them, so a finger that
    // wobbles or rolls on a button can neither be swallowed as "ambiguous"
    // nor turned into a swipe. Gestures start on the background, labels or
    // the analog dial - everything that lets the press fall through to the
    // screen.
    lv_obj_t * act_obj = lv_indev_get_active_obj();
    bool on_widget = (act_obj != NULL && act_obj != lv_screen_active());
    bool handled = false;   // consumed as a swipe or as ambiguous movement

    if (pressed && swipe_armed && face_digital && (gesture_mode || !on_widget)) {
      // Release edge. The display is rotated 270 degrees, so
      // screen-horizontal movement is the native Y axis. Three outcomes:
      //   raw peak travel >= TOUCH_SWIPE_MIN_PX  -> gesture, larger axis decides
      //   filtered peak  >= TOUCH_TAP_MAX_PX     -> neither: a swipe that broke
      //                                             up early must not become a tap
      //   otherwise                              -> tap (LVGL's CLICKED goes through)
      // Raw travel for the swipe test so fast flicks are not understated;
      // filtered travel for the tap test so spikes do not swallow taps.
      int32_t dh = peak_dh;
      int32_t dv = peak_dv;
      int32_t travel = LV_MAX(LV_ABS(dh), LV_ABS(dv));
      int32_t min_px = TOUCH_SWIPE_MIN_PX;
      if (travel < min_px && peak_flt >= TOUCH_TAP_MAX_PX) {
        last_gesture_ms = millis();   // ambiguous movement: swallow the click, do nothing
      }
      else if (travel >= min_px) {
        last_gesture_ms = millis();   // the CLICKED that follows is a swipe tail
        if (LV_ABS(dh) >= LV_ABS(dv)) {
          // dh > 0 is a rightward swipe on screen (screen x = native y).
          // Swipe left -> next face, swipe right -> previous face.
          face_mode = (face_mode + (dh < 0 ? 1 : FACE_COUNT - 1)) % FACE_COUNT;
          if (cal_off != 0) {         // leaving a browsed month snaps back
            cal_off = 0;
            cal_refresh_now();
          }
          face_apply();
          prefs.putInt("face", face_mode);
        }
        else if (face_mode == FACE_CAL) {
          // Vertical swipe on the calendar browses months: up (dv > 0,
          // since screen y runs against native x) -> next, down -> previous.
          cal_off += (dv > 0) ? 1 : -1;
          if (cal_off >  CAL_OFF_MAX) cal_off =  CAL_OFF_MAX;
          if (cal_off < -CAL_OFF_MAX) cal_off = -CAL_OFF_MAX;
          cal_refresh_now();
        }
        // Vertical swipes on other faces are consumed with no action.
      }
    }
    pressed = false;
    confirm_left = -1;   // next contact starts a fresh confirmation
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

// Raised card: a rounded plate lit from the top with a drop shadow. Create
// it before the content it sits under; theme_apply() colors it.
static lv_obj_t * make_card(lv_obj_t * parent, int32_t w, int32_t h,
                            lv_align_t align, int32_t x, int32_t y,
                            int32_t shadow_w) {
  lv_obj_t * c = make_box(parent);
  lv_obj_set_size(c, w, h);
  lv_obj_align(c, align, x, y);
  lv_obj_set_style_radius(c, CARD_RADIUS, 0);
  lv_obj_set_style_bg_opa(c, LV_OPA_COVER, 0);
  lv_obj_set_style_bg_grad_dir(c, LV_GRAD_DIR_VER, 0);
  lv_obj_set_style_shadow_width(c, shadow_w, 0);
  lv_obj_set_style_shadow_offset_x(c, DEPTH_SHADOW_OFS_X, 0);
  lv_obj_set_style_shadow_offset_y(c, DEPTH_SHADOW_OFS_Y, 0);
  lv_obj_set_style_shadow_opa(c, DEPTH_SHADOW_OPA, 0);
  lv_obj_set_style_shadow_color(c, lv_color_black(), 0);
  if (card_count < MAX_CARDS) cards[card_count++] = c;
  return c;
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

  sw_tm_tick();

  if (t.tm_sec != last_sec) {
    last_sec = t.tm_sec;
    char buf[8];
    strftime(buf, sizeof(buf), "%S", &t);
    lv_label_set_text(label_sec, buf);

    // Needles share the 0..3600 value space (one unit per second): the
    // minute needle creeps with the seconds, the hour needle advances once
    // per minute. Updated even while hidden so a face switch is never stale.
    int32_t v_s = t.tm_sec * 60;
    int32_t v_m = t.tm_min * 60 + t.tm_sec;
    int32_t v_h = (t.tm_hour % 12) * 300 + t.tm_min * 5;
    lv_scale_set_line_needle_value(a_scale, a_needle_s, ANALOG_SEC_LEN, v_s);
    lv_scale_set_line_needle_value(a_scale, a_needle_m, ANALOG_MIN_LEN, v_m);
    lv_scale_set_line_needle_value(a_scale, a_needle_h, ANALOG_HOUR_LEN, v_h);
    // Shadow copies follow the same values; their style translate offsets
    // them down-right, matching the dial's light direction.
    lv_scale_set_line_needle_value(a_scale, a_needle_s_sh, ANALOG_SEC_LEN, v_s);
    lv_scale_set_line_needle_value(a_scale, a_needle_m_sh, ANALOG_MIN_LEN, v_m);
    lv_scale_set_line_needle_value(a_scale, a_needle_h_sh, ANALOG_HOUR_LEN, v_h);
  }

  if (t.tm_min != last_min || time_fmt_dirty) {
    bool first_update   = (last_min == -1);   // GUI just built: don't chime
    bool minute_changed = (t.tm_min != last_min);
    last_min = t.tm_min;
    time_fmt_dirty = false;
    char buf[8];

#if HOURLY_CHIME
    if (!first_update && minute_changed && t.tm_min == 0 && !alarm_on &&
        t.tm_hour >= CHIME_FROM_HOUR && t.tm_hour <= CHIME_TO_HOUR) {
      // Casio-style pip-pip
      spk_tone(CHIME_TONE_HZ);
      delay(CHIME_BEEP_MS);
      spk_tone(0);
      delay(CHIME_GAP_MS);
      spk_tone(CHIME_TONE_HZ);
      delay(CHIME_BEEP_MS);
      spk_tone(0);
    }
#endif

    // Wake-up alarms: each fires at the start of its minute on a selected
    // weekday; 1x alarms then disarm themselves. Several alarms on the same
    // minute ring once together.
    if (minute_changed) {
      bool changed = false;
      for (int i = 0; i < ALARM_COUNT; i++) {
        alarm_t * a = &alarms[i];
        if (!a->enabled || t.tm_hour != a->hh || t.tm_min != a->mm ||
            !((a->days >> t.tm_wday) & 1)) continue;
        if (!alarm_on) alarm_start(ALARM_RING_MS);
        if (a->once) { a->enabled = false; changed = true; }
      }
      if (changed) { al_update_label(); al_mark_dirty(); }
    }

    char ampm[4];
    strftime(ampm, sizeof(ampm), "%p", &t);    // "AM" / "PM"

    if (h24) {
      snprintf(buf, sizeof(buf), "%02d:%02d", t.tm_hour, t.tm_min);
      lv_label_set_text(label_hm, buf);
      lv_label_set_text(label_hm_sh, buf);
      // Empty (not hidden) so the seconds keep their bottom slot in the
      // SPACE_BETWEEN column.
      lv_label_set_text(label_ampm, "");
    } else {
      int h12 = t.tm_hour % 12;
      if (h12 == 0) h12 = 12;
      snprintf(buf, sizeof(buf), "%d:%02d", h12, t.tm_min);
      lv_label_set_text(label_hm, buf);
      lv_label_set_text(label_hm_sh, buf);
      lv_label_set_text(label_ampm, ampm);
    }
  }

  if (t.tm_mday != last_mday) {
    last_mday = t.tm_mday;
    char buf[16];
    snprintf(buf, sizeof(buf), "%04d-%02d-%02d",
             t.tm_year + 1900, t.tm_mon + 1, t.tm_mday);
    lv_label_set_text(label_datenum, buf);
    snprintf(buf, sizeof(buf), "(%s)", WEEKDAY_KR[t.tm_wday]);
    lv_label_set_text(label_wd, buf);
    lv_color_t wdc = kr_is_red_day(&t) ? WEEKDAY_COLOR_HOLIDAY : WEEKDAY_COLOR_NORMAL;
    lv_obj_set_style_text_color(label_wd, wdc, 0);


    cal_refresh(&t);

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
    } else {
      lv_obj_add_flag(label_lunar_event, LV_OBJ_FLAG_HIDDEN);
    }

    if (have_lunar) {
      char nbuf[12];
      const char * pre = ld.leap ? "음 윤" : "음";
      snprintf(nbuf, sizeof(nbuf), "%d.%d", ld.month, ld.day);
      lv_label_set_text(label_lunar_pre, pre);
      lv_label_set_text(label_lunar_num, nbuf);
      lv_obj_remove_flag(label_lunar_pre, LV_OBJ_FLAG_HIDDEN);
      lv_obj_remove_flag(label_lunar_num, LV_OBJ_FLAG_HIDDEN);
    } else {
      lv_obj_add_flag(label_lunar_pre, LV_OBJ_FLAG_HIDDEN);
      lv_obj_add_flag(label_lunar_num, LV_OBJ_FLAG_HIDDEN);
    }

    bool term_today = false;
    const char * term = kst_current_term(&t, &term_today);
    if (term) {
      lv_color_t tc = term_today ? TERM_TODAY_COLOR : LUNAR_COLOR;
      lv_label_set_text(label_lunar_term, term);
      lv_obj_set_style_text_color(label_lunar_term, tc, 0);
      lv_obj_remove_flag(label_lunar_term, LV_OBJ_FLAG_HIDDEN);
    } else {
      lv_obj_add_flag(label_lunar_term, LV_OBJ_FLAG_HIDDEN);
    }
  }

#if AUTO_BL
  auto_bl_update();
#endif

  // Hide the brightness panel once the user stops touching it
  if (bl_panel && !lv_obj_has_flag(bl_panel, LV_OBJ_FLAG_HIDDEN) &&
      (millis() - bl_last_touch_ms) > BL_PANEL_TIMEOUT_MS) {
    bl_panel_hide();
  }
}

// ---- Wake-up alarm ---------------------------------------------------------
// Selected-state look for the weekday and "1x" toggles: orange when on, a
// muted gray when off (make_button's gradient/shadow stay).
static void al_style_toggle(lv_obj_t * b, bool on) {
  lv_color_t base = on ? lv_color_hex(0xFF3300) : lv_color_hex(0x6A6A6A);
  lv_obj_set_style_bg_color(b, base, 0);
  lv_obj_set_style_bg_grad_color(b, lv_color_darken(base, 60), 0);
  lv_obj_set_style_text_opa(lv_obj_get_child(b, 0), on ? LV_OPA_COVER : LV_OPA_60, 0);
}

// One alarm indicator: a bell glyph with a diagonal slash drawn over it
// while that alarm is OFF (LVGL's symbol font has no bell-slash glyph).
static lv_obj_t * make_bell(lv_obj_t * parent) {
  static const lv_point_precise_t slash[2] = { {2, 13}, {13, 2} };
  lv_obj_t * box = make_box(parent);
  lv_obj_set_size(box, 15, 15);
  lv_obj_t * l = lv_label_create(box);
  lv_label_set_text(l, LV_SYMBOL_BELL);
  lv_obj_set_style_text_font(l, &lv_font_montserrat_14, 0);
  lv_obj_center(l);
  lv_obj_t * ln = lv_line_create(box);
  lv_line_set_points(ln, slash, 2);
  lv_obj_set_style_line_width(ln, 2, 0);
  lv_obj_set_style_line_rounded(ln, true, 0);
  lv_obj_set_style_line_color(ln, lv_color_hex(0xFF3300), 0);
  lv_obj_remove_flag(ln, LV_OBJ_FLAG_CLICKABLE);
  return box;
}

// Row of ALARM_COUNT bells in a face's bottom-right corner.
static void make_bell_row(lv_obj_t * face, int which) {
  lv_obj_t * row = make_box(face);
  lv_obj_set_size(row, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
  lv_obj_align(row, LV_ALIGN_BOTTOM_RIGHT, -10, -12);
  lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
  lv_obj_set_style_pad_column(row, 2, 0);
  for (int i = 0; i < ALARM_COUNT; i++) bell_box[which][i] = make_bell(row);
}

static void bells_refresh(void) {
  for (int f = 0; f < 2; f++) {
    for (int i = 0; i < ALARM_COUNT; i++) {
      lv_obj_t * box = bell_box[f][i];
      if (!box) continue;
      bool on = alarms[i].enabled;
      lv_obj_set_style_text_opa(lv_obj_get_child(box, 0), on ? LV_OPA_COVER : LV_OPA_50, 0);
      lv_obj_t * slash = lv_obj_get_child(box, 1);
      if (on) lv_obj_add_flag(slash, LV_OBJ_FLAG_HIDDEN);
      else    lv_obj_remove_flag(slash, LV_OBJ_FLAG_HIDDEN);
    }
  }
}

// Repaint the alarm face for the selected alarm (time in the current 12/24 h
// format, weekday and 1x toggles, ON/OFF button), the [1][2][3] selector
// (a bell marks alarms that are ON) and the per-alarm bell indicators on
// the clock faces (slashed while that alarm is OFF).
static void al_update_label(void) {
  if (!label_al_time) return;
  const alarm_t * a = &alarms[al_sel];

  bool any_on = false;
  for (int i = 0; i < ALARM_COUNT; i++) {
    any_on |= alarms[i].enabled;
    if (btn_al_sel[i]) {
      char sb[12];
      snprintf(sb, sizeof(sb), "%s%d", alarms[i].enabled ? LV_SYMBOL_BELL " " : "", i + 1);
      lv_label_set_text(lv_obj_get_child(btn_al_sel[i], 0), sb);
      al_style_toggle(btn_al_sel[i], i == al_sel);
    }
  }
  for (int i = 0; i < 7; i++) {
    if (btn_al_day[i]) al_style_toggle(btn_al_day[i], (a->days >> AL_DAY_ORDER[i]) & 1);
  }
  if (btn_al_once) al_style_toggle(btn_al_once, a->once);

  char buf[8];
  if (h24) {
    snprintf(buf, sizeof(buf), "%02d:%02d", a->hh, a->mm);
    lv_obj_add_flag(label_al_ampm, LV_OBJ_FLAG_HIDDEN);
  } else {
    int h12 = a->hh % 12;
    if (h12 == 0) h12 = 12;
    snprintf(buf, sizeof(buf), "%d:%02d", h12, a->mm);
    lv_label_set_text(label_al_ampm, a->hh < 12 ? "AM" : "PM");
    lv_obj_remove_flag(label_al_ampm, LV_OBJ_FLAG_HIDDEN);
  }
  lv_label_set_text(label_al_time, buf);
  lv_label_set_text(lbl_al_toggle, a->enabled ? LV_SYMBOL_BELL " ON" : LV_SYMBOL_MUTE " OFF");

  LV_UNUSED(any_on);
  bells_refresh();
}

static void al_sel_cb(lv_event_t * e) {
  if (millis() - last_gesture_ms < TOUCH_TAP_GUARD_MS) return;
  al_sel = (int)(intptr_t)lv_event_get_user_data(e);
  al_update_label();
}

static void al_mark_dirty(void) {
  al_dirty = true;
  al_dirty_ms = millis();
}

// 1x mode keeps exactly one weekday selected: the day of the alarm's next
// occurrence - today if its time is still ahead, otherwise tomorrow.
static void al_once_pick_day(alarm_t * a) {
  time_t now = time(nullptr);
  struct tm t;
  localtime_r(&now, &t);
  int now_min = t.tm_hour * 60 + t.tm_min;
  int al_min  = a->hh * 60 + a->mm;
  int wday = (al_min > now_min) ? t.tm_wday : (t.tm_wday + 1) % 7;
  a->days = (uint8_t)(1 << wday);
}

// H-/H+/M-/M+ : user_data is the step in minutes. A tap steps once; holding
// the button repeats (LV_EVENT_LONG_PRESSED_REPEAT) for fast adjustment.
static void al_adjust_cb(lv_event_t * e) {
  static bool repeated = false;   // a hold already stepped; swallow the release click
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_CLICKED) {
    if (repeated) { repeated = false; return; }
    if (millis() - last_gesture_ms < 600) return;
  } else {
    repeated = true;
  }
  int step = (int)(intptr_t)lv_event_get_user_data(e);
  alarm_t * a = &alarms[al_sel];
  int total = (a->hh * 60 + a->mm + step) % (24 * 60);
  if (total < 0) total += 24 * 60;
  a->hh = total / 60;
  a->mm = total % 60;
  if (a->once) al_once_pick_day(a);   // the next occurrence may have moved to the other day
  al_update_label();
  al_mark_dirty();
}

static void al_toggle_cb(lv_event_t * e) {
  LV_UNUSED(e);
  if (millis() - last_gesture_ms < 600) return;
  alarms[al_sel].enabled = !alarms[al_sel].enabled;
  al_update_label();
  al_mark_dirty();
}

// Weekday toggles (user_data = tm_wday); the last selected day cannot be
// cleared, so the alarm always has at least one day to ring on.
static void al_day_cb(lv_event_t * e) {
  if (millis() - last_gesture_ms < 600) return;
  int d = (int)(intptr_t)lv_event_get_user_data(e);
  alarm_t * a = &alarms[al_sel];
  uint8_t next = a->days ^ (uint8_t)(1 << d);
  if (next == 0) return;
  a->days = next;
  a->once = false;   // hand-picked days mean a repeating alarm again
  al_update_label();
  al_mark_dirty();
}

// "1x": ring at the next matching time, then switch the alarm OFF by itself.
// Turning it on narrows the weekdays to the day of that next occurrence;
// turning it off restores the days that were selected before.
static void al_once_cb(lv_event_t * e) {
  LV_UNUSED(e);
  if (millis() - last_gesture_ms < 600) return;
  alarm_t * a = &alarms[al_sel];
  a->once = !a->once;
  if (a->once) {
    a->days_saved = a->days;
    al_once_pick_day(a);
  } else {
    a->days = a->days_saved ? a->days_saved : 0x7F;
  }
  al_update_label();
  al_mark_dirty();
}

// OK: commit any pending alarm changes to NVS right away and move on to the
// next face (swipes work here too).
static void al_done_cb(lv_event_t * e) {
  LV_UNUSED(e);
  if (millis() - last_gesture_ms < 600) return;
  if (al_dirty) al_save_now();
  face_mode = (face_mode + 1) % FACE_COUNT;
  face_apply();
  prefs.putInt("face", face_mode);
}

static lv_obj_t * make_button(lv_obj_t * parent, const char * txt,
                              lv_event_cb_t cb, void * user_data,
                              int32_t w, int32_t h);

// ---- 12 / 24 hour toggle on the brightness panel ---------------------------
static void time_fmt_apply(void) {
  int32_t w = h24 ? w_hm_24 : w_hm_12;
  lv_obj_set_width(hm_box, w);
  lv_obj_set_width(label_hm, w);
  lv_obj_set_width(label_hm_sh, w);
  if (label_ghost) lv_obj_set_width(label_ghost, w);
  if (lbl_h24) lv_label_set_text(lbl_h24, h24 ? "24H" : "12H");
  time_fmt_dirty = true;   // timer_cb re-renders the hour on its next tick
  al_update_label();
}

static void h24_btn_cb(lv_event_t * e) {
  LV_UNUSED(e);
  if (millis() - last_gesture_ms < 600) return;
  h24 = !h24;
  prefs.putInt("h24", h24 ? 1 : 0);
  time_fmt_apply();
  bl_last_touch_ms = millis();
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
  lv_obj_set_size(bl_panel, 300, 122);
  lv_obj_align(bl_panel, LV_ALIGN_BOTTOM_MID, 0, -8);
  lv_obj_remove_flag(bl_panel, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_bg_color(bl_panel, lv_color_hex(0x181818), 0);
  lv_obj_set_style_bg_opa(bl_panel, LV_OPA_90, 0);
  lv_obj_set_style_border_color(bl_panel, lv_color_hex(0x555555), 0);
  lv_obj_set_style_border_width(bl_panel, 1, 0);
  lv_obj_set_style_radius(bl_panel, 8, 0);
  lv_obj_set_style_shadow_width(bl_panel, 20, 0);
  lv_obj_set_style_shadow_offset_y(bl_panel, 6, 0);
  lv_obj_set_style_shadow_opa(bl_panel, LV_OPA_60, 0);
  lv_obj_set_style_shadow_color(bl_panel, lv_color_black(), 0);
  lv_obj_set_style_pad_all(bl_panel, 8, 0);
  lv_obj_add_event_cb(bl_panel, bl_panel_press_cb, LV_EVENT_PRESSED, NULL);

  // Neither bundled Korean subset covers generic UI text (weekdays and
  // calendar names only), so this label stays numeric.
  bl_pct_label = lv_label_create(bl_panel);
  lv_obj_set_style_text_font(bl_pct_label, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(bl_pct_label, lv_color_hex(0xFFFFFF), 0);
  lv_obj_align(bl_pct_label, LV_ALIGN_TOP_LEFT, 0, -2);

  // 12/24-hour toggle, top-right (the swatch rows start below it)
  lv_obj_t * hb = make_button(bl_panel, h24 ? "24H" : "12H", h24_btn_cb, NULL, 64, 26);
  lv_obj_align(hb, LV_ALIGN_TOP_RIGHT, 0, -4);
  lbl_h24 = lv_obj_get_child(hb, 0);

  // Background color swatches in two full-width rows below the % label
  // (light row, then dark row - mirroring the THEMES order); the current
  // one gets an orange ring. Created after the slider so they win the hit
  // test over its extended click area.
  for (int i = 0; i < THEME_COUNT; i++) {
    lv_obj_t * b = lv_obj_create(bl_panel);
    lv_obj_remove_style_all(b);
    lv_obj_set_size(b, 24, 24);
    lv_obj_align(b, LV_ALIGN_TOP_LEFT, (i % 8) * 37, 26 + (i / 8) * 30);
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

// ============ Analog face: one big dial, centered ==========================
static void create_analog_face(void) {
  face_analog = make_box(lv_screen_active());
  lv_obj_set_size(face_analog, lv_pct(100), lv_pct(100));

  // ---- Raised disc under the dial: bezel ring (with the drop shadow) and
  // the face inside it. Created before the scale so they draw beneath it.
  // Colors come from theme_apply().
  const int32_t bezel_size = ANALOG_DIAL_SIZE + 2 * ANALOG_BEZEL_W;
  a_bezel = lv_obj_create(face_analog);
  lv_obj_remove_style_all(a_bezel);
  lv_obj_remove_flag(a_bezel, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_size(a_bezel, bezel_size, bezel_size);
  lv_obj_align(a_bezel, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_radius(a_bezel, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_opa(a_bezel, LV_OPA_COVER, 0);
  lv_obj_set_style_bg_grad_dir(a_bezel, LV_GRAD_DIR_VER, 0);
  lv_obj_set_style_shadow_width(a_bezel, DEPTH_SHADOW_W, 0);
  lv_obj_set_style_shadow_offset_x(a_bezel, DEPTH_SHADOW_OFS_X, 0);
  lv_obj_set_style_shadow_offset_y(a_bezel, DEPTH_SHADOW_OFS_Y, 0);
  lv_obj_set_style_shadow_opa(a_bezel, DEPTH_SHADOW_OPA, 0);

  a_face = lv_obj_create(face_analog);
  lv_obj_remove_style_all(a_face);
  lv_obj_remove_flag(a_face, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_size(a_face, ANALOG_DIAL_SIZE, ANALOG_DIAL_SIZE);
  lv_obj_align(a_face, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_radius(a_face, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_opa(a_face, LV_OPA_COVER, 0);
  lv_obj_set_style_bg_grad_dir(a_face, LV_GRAD_DIR_VER, 0);

  // ---- Dial: a round lv_scale with 60 minute ticks, hour numerals on the
  // majors. Rotation 270 puts value 0 at 12 o'clock. The value space is
  // 0..3600 (one unit per second) rather than 0..60, so the minute and hour
  // needles can creep smoothly between ticks instead of jumping once a
  // minute; the 61 ticks still land on every 60th unit, so the dial itself
  // looks the same.
  a_scale = lv_scale_create(face_analog);
  // Not clickable (nor its needles below): a tap on the dial must reach the
  // screen so it opens the brightness panel like everywhere else.
  lv_obj_remove_flag(a_scale, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_size(a_scale, ANALOG_DIAL_SIZE, ANALOG_DIAL_SIZE);
  lv_obj_align(a_scale, LV_ALIGN_CENTER, 0, 0);
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

  // ---- Needles (children of the scale; lv_scale positions the points).
  // Shadow copies are created first so they draw under the real needles;
  // lv_scale aligns each line to the scale's origin, so a style translate
  // shifts the whole shadow down-right.
  struct { lv_obj_t ** sh; int32_t w; } needle_sh[3] = {
    { &a_needle_h_sh, 5 }, { &a_needle_m_sh, 3 }, { &a_needle_s_sh, 2 },
  };
  for (int i = 0; i < 3; i++) {
    lv_obj_t * l = lv_line_create(a_scale);
    lv_obj_set_style_line_width(l, needle_sh[i].w, 0);
    lv_obj_set_style_line_rounded(l, true, 0);
    lv_obj_set_style_line_color(l, lv_color_black(), 0);
    lv_obj_set_style_line_opa(l, NEEDLE_SHADOW_OPA, 0);
    lv_obj_set_style_translate_x(l, NEEDLE_SHADOW_OFS_X, 0);
    lv_obj_set_style_translate_y(l, NEEDLE_SHADOW_OFS_Y, 0);
    *needle_sh[i].sh = l;
  }

  a_needle_h = lv_line_create(a_scale);
  lv_obj_remove_flag(a_needle_h, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_style_line_width(a_needle_h, 5, 0);
  lv_obj_set_style_line_rounded(a_needle_h, true, 0);

  a_needle_m = lv_line_create(a_scale);
  lv_obj_remove_flag(a_needle_m, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_style_line_width(a_needle_m, 3, 0);
  lv_obj_set_style_line_rounded(a_needle_m, true, 0);

  a_needle_s = lv_line_create(a_scale);
  lv_obj_remove_flag(a_needle_s, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_style_line_width(a_needle_s, 2, 0);
  lv_obj_set_style_line_color(a_needle_s, lv_color_hex(0xFF3300), 0);

  // Center cap over the needle pivots: lit from the top, with its own
  // small shadow so it sits proud of the needles.
  a_cap = lv_obj_create(a_scale);
  lv_obj_remove_style_all(a_cap);
  lv_obj_remove_flag(a_cap, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_size(a_cap, 14, 14);
  lv_obj_set_style_radius(a_cap, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_color(a_cap, lv_color_hex(0xFF7A55), 0);
  lv_obj_set_style_bg_grad_color(a_cap, lv_color_hex(0xC42600), 0);
  lv_obj_set_style_bg_grad_dir(a_cap, LV_GRAD_DIR_VER, 0);
  lv_obj_set_style_bg_opa(a_cap, LV_OPA_COVER, 0);
  lv_obj_set_style_shadow_width(a_cap, 8, 0);
  lv_obj_set_style_shadow_offset_x(a_cap, 2, 0);
  lv_obj_set_style_shadow_offset_y(a_cap, 3, 0);
  lv_obj_set_style_shadow_opa(a_cap, LV_OPA_50, 0);
  lv_obj_set_style_shadow_color(a_cap, lv_color_black(), 0);
  lv_obj_center(a_cap);

  // Alarm bells in the same bottom-right spot as on the digital face
  make_bell_row(face_analog, 1);
  // Colors are applied by theme_apply() from lv_create_main_gui() once
  // every face exists - calling it here would leave the cards of the faces
  // created later (calendar, stopwatch, timer) at LVGL's default white.
}

// ============ Calendar face: month title, 일..토 header, 6x7 day grid =====
// Static layout only - all texts and colors are filled by cal_refresh().
static void create_calendar_face(void) {
  face_cal = make_box(lv_screen_active());
  lv_obj_set_size(face_cal, lv_pct(100), lv_pct(100));
  // Near-full-screen plate; a lighter shadow since it is mostly clipped
  make_card(face_cal, 312, 232, LV_ALIGN_CENTER, 0, 0, 12);

  label_c_title = lv_label_create(face_cal);
  lv_label_set_text(label_c_title, "");
  lv_obj_set_style_text_font(label_c_title, FONT_LUNAR_NUM, 0);   // DSEG with '.'
  lv_obj_align(label_c_title, LV_ALIGN_TOP_MID, 0, 2);

  for (int i = 0; i < 7; i++) {
    label_c_wd[i] = lv_label_create(face_cal);
    lv_label_set_text(label_c_wd[i], WEEKDAY_KR[i]);
    lv_obj_set_style_text_font(label_c_wd[i], FONT_KR, 0);
    lv_obj_set_width(label_c_wd[i], 45);
    lv_obj_set_style_text_align(label_c_wd[i], LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_pos(label_c_wd[i], 2 + i * 45, 32);
  }

  for (int i = 0; i < 42; i++) {
    lv_obj_t * l = lv_label_create(face_cal);
    lv_label_set_text(l, "");
    lv_obj_set_style_text_font(l, &lv_font_montserrat_20, 0);
    lv_obj_set_width(l, 41);
    lv_obj_set_style_text_align(l, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_pos(l, 4 + (i % 7) * 45, 60 + (i / 7) * 29);
    // today's pad; kept transparent on every other day
    lv_obj_set_style_radius(l, 6, 0);
    lv_obj_set_style_bg_color(l, lv_color_hex(0xFF3300), 0);
    lv_obj_set_style_bg_opa(l, LV_OPA_TRANSP, 0);
    lv_obj_set_style_pad_ver(l, 2, 0);
    // the pad's own shadow, enabled together with the pad in cal_refresh()
    lv_obj_set_style_shadow_width(l, 8, 0);
    lv_obj_set_style_shadow_offset_x(l, 1, 0);
    lv_obj_set_style_shadow_offset_y(l, 3, 0);
    lv_obj_set_style_shadow_color(l, lv_color_black(), 0);
    lv_obj_set_style_shadow_opa(l, LV_OPA_TRANSP, 0);
    label_c_day[i] = l;
  }
}

// Symbol button with a montserrat-20 label; returns the button.
static lv_obj_t * make_button(lv_obj_t * parent, const char * txt,
                              lv_event_cb_t cb, void * user_data,
                              int32_t w, int32_t h) {
  lv_obj_t * b = lv_button_create(parent);
  lv_obj_set_size(b, w, h);
  lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, user_data);
  // Raised: theme color on top fading darker, plus a drop shadow
  lv_color_t base = lv_obj_get_style_bg_color(b, LV_PART_MAIN);
  lv_obj_set_style_bg_grad_color(b, lv_color_darken(base, 60), 0);
  lv_obj_set_style_bg_grad_dir(b, LV_GRAD_DIR_VER, 0);
  lv_obj_set_style_shadow_width(b, 10, 0);
  lv_obj_set_style_shadow_offset_x(b, 2, 0);
  lv_obj_set_style_shadow_offset_y(b, 4, 0);
  lv_obj_set_style_shadow_opa(b, LV_OPA_40, 0);
  lv_obj_set_style_shadow_color(b, lv_color_black(), 0);
  lv_obj_t * l = lv_label_create(b);
  lv_label_set_text(l, txt);
  lv_obj_set_style_text_font(l, &lv_font_montserrat_20, 0);
  lv_obj_center(l);
  return b;
}

// ============ Stopwatch face: [ 12:34 .07 ]  laps x3 (top-down)  [>] [LAP] [reset]
static void create_stopwatch_face(void) {
  face_sw = make_box(lv_screen_active());
  lv_obj_set_size(face_sw, lv_pct(100), lv_pct(100));
  // Display window at the top (y 12..104), lap lines below it, buttons at
  // the bottom.
  make_card(face_sw, 300, 92, LV_ALIGN_CENTER, 0, -62, DEPTH_SHADOW_W);

  lv_obj_t * row = make_box(face_sw);
  lv_obj_set_size(row, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
  lv_obj_align(row, LV_ALIGN_CENTER, 0, -62);
  lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(row, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_column(row, 4, 0);

  label_sw_time = lv_label_create(row);
  lv_label_set_text(label_sw_time, "00:00");
  lv_obj_set_style_text_font(label_sw_time, FONT_TIME, 0);

  label_sw_frac = lv_label_create(row);
  lv_label_set_text(label_sw_frac, ".00");
  lv_obj_set_style_text_font(label_sw_frac, FONT_LUNAR_NUM, 0);

  // Lap lines (oldest first, new ones appended below):
  // [number][gap][lap time][gap][cumulative],
  // DSEG italic like the rest of the digits. The number is right-aligned in
  // a fixed cell so the time columns line up even when laps pass 9; the
  // cumulative column is drawn a little dimmer to tell the two apart.
  for (int i = 0; i < SW_LAPS; i++) {
    lv_obj_t * r = make_box(face_sw);
    lv_obj_set_size(r, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_align(r, LV_ALIGN_TOP_MID, 0, 110 + i * 24);
    lv_obj_set_flex_flow(r, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(r, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(r, SW_LAP_GAP_PX, 0);
    lv_obj_add_flag(r, LV_OBJ_FLAG_HIDDEN);
    row_sw_lap[i] = r;

    label_sw_lap_no[i] = lv_label_create(r);
    lv_label_set_text(label_sw_lap_no[i], "");
    lv_obj_set_style_text_font(label_sw_lap_no[i], FONT_LUNAR_NUM_SM, 0);
    lv_obj_set_width(label_sw_lap_no[i], 34);
    lv_obj_set_style_text_align(label_sw_lap_no[i], LV_TEXT_ALIGN_RIGHT, 0);

    label_sw_lap[i] = lv_label_create(r);
    lv_label_set_text(label_sw_lap[i], "");
    lv_obj_set_style_text_font(label_sw_lap[i], FONT_LUNAR_NUM_SM, 0);

    label_sw_lap_cum[i] = lv_label_create(r);
    lv_label_set_text(label_sw_lap_cum[i], "");
    lv_obj_set_style_text_font(label_sw_lap_cum[i], FONT_LUNAR_NUM_SM, 0);
    lv_obj_set_style_text_opa(label_sw_lap_cum[i], SW_LAP_CUM_OPA, 0);
  }

  lv_obj_t * b = make_button(face_sw, LV_SYMBOL_PLAY, sw_start_cb, NULL, 88, 40);
  lv_obj_align(b, LV_ALIGN_BOTTOM_MID, -100, -6);
  lbl_sw_start = lv_obj_get_child(b, 0);

  b = make_button(face_sw, "LAP", sw_lap_cb, NULL, 88, 40);
  lv_obj_align(b, LV_ALIGN_BOTTOM_MID, 0, -6);

  b = make_button(face_sw, LV_SYMBOL_REFRESH, sw_reset_cb, NULL, 88, 40);
  lv_obj_align(b, LV_ALIGN_BOTTOM_MID, 100, -6);

  sw_fast_timer = lv_timer_create([](lv_timer_t * t) { LV_UNUSED(t); sw_update_label(); },
                                  SW_DISPLAY_MS, NULL);
  lv_timer_pause(sw_fast_timer);   // sw_fast_sync() resumes it when needed

  sw_update_label();   // real format from the start ("00:00" ".00"), not placeholders
}

// ============ Timer face: [ 05:00 ]  [+1m][+5m][+10m]  [>] [reset] ========
static void create_timer_face(void) {
  face_tm = make_box(lv_screen_active());
  lv_obj_set_size(face_tm, lv_pct(100), lv_pct(100));
  make_card(face_tm, 300, 96, LV_ALIGN_CENTER, 0, -46, DEPTH_SHADOW_W);   // display window

  label_tm_time = lv_label_create(face_tm);
  lv_label_set_text(label_tm_time, "00:00");
  lv_obj_set_style_text_font(label_tm_time, FONT_TIME, 0);
  lv_obj_align(label_tm_time, LV_ALIGN_CENTER, 0, -46);

  static const int PRESET_MIN[3] = { 1, 5, 10 };
  static const char * PRESET_TXT[3] = { "+1m", "+5m", "+10m" };
  for (int i = 0; i < 3; i++) {
    lv_obj_t * b = make_button(face_tm, PRESET_TXT[i], tm_add_cb,
                               (void *)(intptr_t)PRESET_MIN[i], 84, 40);
    lv_obj_align(b, LV_ALIGN_BOTTOM_MID, (i - 1) * 96, -66);
  }

  lv_obj_t * b = make_button(face_tm, LV_SYMBOL_PLAY, tm_start_cb, NULL, 100, 44);
  lv_obj_align(b, LV_ALIGN_BOTTOM_MID, -62, -14);
  lbl_tm_start = lv_obj_get_child(b, 0);

  b = make_button(face_tm, LV_SYMBOL_REFRESH, tm_reset_cb, NULL, 100, 44);
  lv_obj_align(b, LV_ALIGN_BOTTOM_MID, 62, -14);
}

// ============ Alarm face ====================================================
//   [ 07:30 AM ]                    card, y 6..90 (selected alarm's time)
//   [1][2][3]                       alarm selector, y 94..120
//   [월][화][수][목][금][토][일]      weekday toggles, y 124..152
//   [H-][H+][M-][M+]                y 158..190
//   [bell ON/OFF] [1x] [OK]         y 196..232 (OK saves and leaves)
static void create_alarm_face(void) {
  face_alarm = make_box(lv_screen_active());
  lv_obj_set_size(face_alarm, lv_pct(100), lv_pct(100));
  make_card(face_alarm, 300, 84, LV_ALIGN_CENTER, 0, -72, DEPTH_SHADOW_W);

  lv_obj_t * row = make_box(face_alarm);
  lv_obj_set_size(row, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
  lv_obj_align(row, LV_ALIGN_CENTER, 0, -72);
  lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(row, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_column(row, 8, 0);

  label_al_time = lv_label_create(row);
  lv_label_set_text(label_al_time, "");
  lv_obj_set_style_text_font(label_al_time, FONT_TIME, 0);

  label_al_ampm = lv_label_create(row);
  lv_label_set_text(label_al_ampm, "AM");
  lv_obj_set_style_text_font(label_al_ampm, FONT_AMPM, 0);
  lv_obj_set_style_pad_bottom(label_al_ampm, 6, 0);

  // Alarm selector [1][2][3]; texts are filled by al_update_label()
  for (int i = 0; i < ALARM_COUNT; i++) {
    lv_obj_t * b = make_button(face_alarm, "", al_sel_cb, (void *)(intptr_t)i, 56, 26);
    lv_obj_align(b, LV_ALIGN_BOTTOM_MID, (i - (ALARM_COUNT - 1) / 2.0f) * 64, -120);
    btn_al_sel[i] = b;
  }

  // Weekday toggles Monday..Sunday (tm_wday via AL_DAY_ORDER), Korean font
  for (int i = 0; i < 7; i++) {
    int d = AL_DAY_ORDER[i];
    lv_obj_t * b = make_button(face_alarm, WEEKDAY_KR[d], al_day_cb,
                               (void *)(intptr_t)d, 38, 28);
    lv_obj_set_style_text_font(lv_obj_get_child(b, 0), FONT_KR, 0);
    lv_obj_align(b, LV_ALIGN_BOTTOM_MID, -132 + i * 44, -88);
    btn_al_day[i] = b;
  }

  static const char * ADJ_TXT[4]  = { "H-", "H+", "M-", "M+" };
  static const int    ADJ_STEP[4] = { -60, 60, -1, 1 };
  for (int i = 0; i < 4; i++) {
    lv_obj_t * b = make_button(face_alarm, ADJ_TXT[i], al_adjust_cb,
                               (void *)(intptr_t)ADJ_STEP[i], 68, 32);
    lv_obj_add_event_cb(b, al_adjust_cb, LV_EVENT_LONG_PRESSED_REPEAT,
                        (void *)(intptr_t)ADJ_STEP[i]);
    lv_obj_align(b, LV_ALIGN_BOTTOM_MID, -114 + i * 76, -50);
  }

  lv_obj_t * b = make_button(face_alarm, LV_SYMBOL_MUTE " OFF", al_toggle_cb, NULL, 100, 36);
  lv_obj_align(b, LV_ALIGN_BOTTOM_MID, -76, -8);
  lbl_al_toggle = lv_obj_get_child(b, 0);

  btn_al_once = make_button(face_alarm, "1x", al_once_cb, NULL, 56, 36);
  lv_obj_align(btn_al_once, LV_ALIGN_BOTTOM_MID, 8, -8);

  b = make_button(face_alarm, LV_SYMBOL_OK " OK", al_done_cb, NULL, 76, 36);
  lv_obj_align(b, LV_ALIGN_BOTTOM_MID, 88, -8);

  al_update_label();
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
  // Plate under the three rows (content spans y 28..212 -> 14..226 plate)
  make_card(face_digital, 312, 212, LV_ALIGN_CENTER, 0, 0, DEPTH_SHADOW_W);

  // One bell per alarm in the plate's bottom-right corner (slashed when OFF)
  make_bell_row(face_digital, 0);

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

  // In 12-hour mode the hour field never shows 2-9 in its leading cell: it
  // runs 1:00-12:59, so the leading digit is either absent or a '1', whose ink
  // (segments B and C) sits at the far right of its monospaced cell.
  // Reserve only that ink instead of the whole cell, otherwise a one-digit
  // hour leaves a dead zone on the left and the display looks pushed right.
  // The label is right-aligned with LONG_CLIP, so at 10:00-12:59 only the
  // blank left part of the '1' cell is clipped and nothing ever moves.
  lv_font_glyph_dsc_t g1;
  int32_t lead_blank = 0;
  if (lv_font_get_glyph_dsc(FONT_TIME, &g1, '1', 0)) lead_blank = g1.ofs_x;

  // 24-hour mode shows a real leading 0/1/2 and needs the whole cell.
  w_hm_24 = 4 * max_digit_width(FONT_TIME) + glyph_width(FONT_TIME, ':') + 4;
  w_hm_12 = w_hm_24 - lead_blank;
  const int32_t w_hm    = h24 ? w_hm_24 : w_hm_12;
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
  Serial.printf("time row width (24 h, worst case): %ld px (screen %d px)\n",
                (long)(w_hm_24 + w_col + 8), SCREEN_HEIGHT);

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
  hm_box = make_box(time_row);
  lv_obj_set_size(hm_box, w_hm, h_time);

#if SHOW_GHOST_SEGMENTS
  // Same width/align/clip as label_hm so the segments coincide. The blank
  // lead-in of the first 8 gets clipped like the '1' cell above; its left
  // segments (E/F) lose lead_blank pixels - acceptable for a ghost layer.
  label_ghost = lv_label_create(hm_box);
  lv_label_set_text(label_ghost, "88:88");
  lv_obj_add_style(label_ghost, &style_time, 0);
  lv_obj_set_style_text_color(label_ghost, GHOST_COLOR, 0);
  lv_obj_set_width(label_ghost, w_hm);
  lv_obj_set_style_text_align(label_ghost, LV_TEXT_ALIGN_RIGHT, 0);
  lv_label_set_long_mode(label_ghost, LV_LABEL_LONG_CLIP);
  lv_obj_center(label_ghost);
#endif

  // Emboss: a translucent black copy of HH:MM, offset down-right, under it
  label_hm_sh = lv_label_create(hm_box);
  lv_label_set_text(label_hm_sh, "12:00");
  lv_obj_add_style(label_hm_sh, &style_time, 0);
  lv_obj_set_width(label_hm_sh, w_hm);
  lv_obj_set_style_text_align(label_hm_sh, LV_TEXT_ALIGN_RIGHT, 0);
  lv_label_set_long_mode(label_hm_sh, LV_LABEL_LONG_CLIP);
  lv_obj_set_style_text_color(label_hm_sh, lv_color_black(), 0);
  lv_obj_set_style_text_opa(label_hm_sh, LV_OPA_30, 0);
  lv_obj_set_style_translate_x(label_hm_sh, NEEDLE_SHADOW_OFS_X, 0);
  lv_obj_set_style_translate_y(label_hm_sh, NEEDLE_SHADOW_OFS_Y, 0);
  lv_obj_center(label_hm_sh);

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
  create_calendar_face();
  create_stopwatch_face();
  create_timer_face();
  create_alarm_face();
  theme_apply();        // now that every face and card exists
  face_apply();         // show the face restored from NVS

  create_brightness_panel();
  bl_set_pct(bl_pct);   // syncs the label with the restored value

  // Full-screen alarm flash on the top layer, above the panel; tap to stop
  alarm_overlay = lv_obj_create(lv_layer_top());
  lv_obj_remove_style_all(alarm_overlay);
  lv_obj_set_size(alarm_overlay, lv_pct(100), lv_pct(100));
  lv_obj_set_style_bg_color(alarm_overlay, lv_color_hex(0xFF3300), 0);
  lv_obj_set_style_bg_opa(alarm_overlay, LV_OPA_30, 0);
  lv_obj_add_flag(alarm_overlay, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(alarm_overlay, alarm_overlay_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_add_flag(alarm_overlay, LV_OBJ_FLAG_HIDDEN);

  // 10 ticks per second: the stopwatch shows tenths, and the second
  // boundary of the clock is never more than ~100 ms late
  lv_timer_t * timer = lv_timer_create(timer_cb, 100, NULL);
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
  if (face_mode < 0 || face_mode >= FACE_COUNT) face_mode = 0;
  theme_idx = prefs.getInt("theme", 0);
  if (theme_idx < 0 || theme_idx >= THEME_COUNT) theme_idx = 0;
  h24 = prefs.getInt("h24", 0) != 0;
  for (int i = 0; i < ALARM_COUNT; i++) {
    char k[8];
    alarm_t * a = &alarms[i];
    snprintf(k, sizeof(k), "a%d_h", i);  a->hh = prefs.getInt(k, ALARM_DEFAULT_HH);
    snprintf(k, sizeof(k), "a%d_m", i);  a->mm = prefs.getInt(k, ALARM_DEFAULT_MM);
    snprintf(k, sizeof(k), "a%d_on", i); a->enabled = prefs.getInt(k, 0) != 0;
    snprintf(k, sizeof(k), "a%d_d", i);  a->days = (uint8_t)(prefs.getInt(k, 0x7F) & 0x7F);
    snprintf(k, sizeof(k), "a%d_1", i);  a->once = prefs.getInt(k, 0) != 0;
    if (a->hh < 0 || a->hh > 23) a->hh = ALARM_DEFAULT_HH;
    if (a->mm < 0 || a->mm > 59) a->mm = ALARM_DEFAULT_MM;
    if (a->days == 0) a->days = 0x7F;
    a->days_saved = 0x7F;
  }

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

  // Speaker for the timer alarm, silent until needed. The channel is pinned
  // to 2 explicitly: auto-allocation could hand out channel 1, which shares
  // an LEDC timer with the backlight on channel 0, and two outputs at
  // different frequencies must not share a timer.
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttachChannel(SPK_PIN, ALARM_TONE_HZ, 10, SPK_CHANNEL);
  ledcWriteTone(SPK_PIN, 0);
#else
  ledcSetup(SPK_CHANNEL, ALARM_TONE_HZ, 10);
  ledcAttachPin(SPK_PIN, SPK_CHANNEL);
  ledcWriteTone(SPK_CHANNEL, 0);
#endif

#if SPK_TEST
  Serial.println("Speaker test 1: LEDC tones 500/1000/2000/3000 Hz, 1 s each");
  static const uint32_t TEST_HZ[4] = { 500, 1000, 2000, 3000 };
  for (int i = 0; i < 4; i++) {
    Serial.printf("  tone %lu Hz\n", (unsigned long)TEST_HZ[i]);
    spk_tone(TEST_HZ[i]);
    delay(1000);
  }
  spk_tone(0);

  // Phase 2 drives the DAC directly, bypassing LEDC and the GPIO matrix -
  // only possible when SPK_PIN is a DAC pin (25 or 26, i.e. the amp path).
  // Sound here but not in phase 1 -> LEDC setup problem; silence in both
  // with verified wiring -> the on-board 8002 amp or its power is the
  // suspect.
#if SPK_PIN == 25 || SPK_PIN == 26
  Serial.println("Speaker test 2: DAC square wave 1 kHz, 2 s");
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcDetach(SPK_PIN);
#else
  ledcDetachPin(SPK_PIN);
#endif
  for (int i = 0; i < 2000; i++) {
    dacWrite(SPK_PIN, 255);
    delayMicroseconds(500);
    dacWrite(SPK_PIN, 0);
    delayMicroseconds(500);
  }
  dacWrite(SPK_PIN, 0);
  Serial.println("Speaker test done");

  // Give the pin back to LEDC for the alarm/chime
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttachChannel(SPK_PIN, ALARM_TONE_HZ, 10, SPK_CHANNEL);
  ledcWriteTone(SPK_PIN, 0);
#else
  ledcAttachPin(SPK_PIN, SPK_CHANNEL);
  ledcWriteTone(SPK_CHANNEL, 0);
#endif
#endif  /* DAC-capable SPK_PIN */
#endif

#if BOOT_BEEP
  // Short double beep so a freshly attached speaker can be verified
  for (int i = 0; i < 2; i++) {
    spk_tone(ALARM_TONE_HZ);
    delay(120);
    spk_tone(0);
    delay(80);
  }
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
