/* Korean lunar calendar + 24 solar terms (절기) for the bottom status line.
 *
 * Both are table-driven; neither can be computed from struct tm alone.
 *
 *   - Lunar table (2025..2045): generated from the KASI-based
 *     `korean-lunar-calendar` Python package. Per lunar year it stores the
 *     solar date of 설날 (lunar 1/1), the leap month (0 = none) and a bitmask
 *     of the 30-day months in sequence order, where a leap month follows its
 *     ordinary month in the sequence.
 *
 *   - Solar-term table (2026..2035): KST 절입 dates (Korean midnight-boundary
 *     dates differ from the Chinese-time formula by a day in some years),
 *     mm*100+dd, 소한 first, 동지 last.
 *
 * Extend the tables as the years run out. Outside their range the lookup
 * functions fail gracefully (return false / NULL) and the clock just leaves
 * that part of the line blank.
 */

#ifndef KOREAN_CALENDAR_H
#define KOREAN_CALENDAR_H

#include <stdint.h>
#include <stdbool.h>
#include <time.h>

/* ======================= Lunar calendar ================================= */

#define KLC_FIRST_YEAR 2025
#define KLC_LAST_YEAR  2045

typedef struct {
  uint8_t  lny_mon;    /* solar month of lunar 1/1 */
  uint8_t  lny_day;    /* solar day of lunar 1/1   */
  uint8_t  leap_month; /* 0 = no leap month        */
  uint16_t big_mask;   /* bit i: i-th month in sequence has 30 days */
} klc_year_t;

static const klc_year_t KLC_YEARS[KLC_LAST_YEAR - KLC_FIRST_YEAR + 1] = {
  {  1, 29,  6, 0x0EA5 },  /* 2025: 13 months */
  {  2, 17,  0, 0x0EA5 },  /* 2026 */
  {  2,  7,  0, 0x0E4A },  /* 2027 */
  {  1, 27,  5, 0x0C96 },  /* 2028: 13 months */
  {  2, 13,  0, 0x0C9B },  /* 2029 */
  {  2,  3,  0, 0x055A },  /* 2030 */
  {  1, 23,  3, 0x0AD5 },  /* 2031: 13 months */
  {  2, 11,  0, 0x0B69 },  /* 2032 */
  {  1, 31, 11, 0x1752 },  /* 2033: 13 months */
  {  2, 19,  0, 0x0752 },  /* 2034 */
  {  2,  8,  0, 0x0B25 },  /* 2035 */
  {  1, 28,  6, 0x164B },  /* 2036: 13 months */
  {  2, 15,  0, 0x0A4B },  /* 2037 */
  {  2,  4,  0, 0x04AB },  /* 2038 */
  {  1, 24,  5, 0x055B },  /* 2039: 13 months */
  {  2, 12,  0, 0x056D },  /* 2040 */
  {  2,  1,  0, 0x0B69 },  /* 2041 */
  {  1, 22,  2, 0x1B52 },  /* 2042: 13 months */
  {  2, 10,  0, 0x0D92 },  /* 2043 */
  {  1, 30,  7, 0x1D25 },  /* 2044: 13 months */
  {  2, 17,  0, 0x0D25 },  /* 2045 */
};

typedef struct {
  int  month;
  int  day;
  bool leap;
} klc_date_t;

/* Days since 1970-01-01 (Howard Hinnant's civil-days algorithm). */
static long klc_days_from_civil(int y, int m, int d) {
  y -= m <= 2;
  long era = (y >= 0 ? y : y - 399) / 400;
  unsigned yoe = (unsigned)(y - era * 400);
  unsigned doy = (unsigned)((153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1);
  unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + (long)doe - 719468;
}

static bool klc_solar_to_lunar(const struct tm * t, klc_date_t * out) {
  int sy = t->tm_year + 1900;
  if (sy < KLC_FIRST_YEAR || sy > KLC_LAST_YEAR) return false;

  long today = klc_days_from_civil(sy, t->tm_mon + 1, t->tm_mday);

  /* Before this year's 설날 we are still in the previous lunar year. */
  int ly = sy;
  const klc_year_t * e = &KLC_YEARS[ly - KLC_FIRST_YEAR];
  long lny = klc_days_from_civil(ly, e->lny_mon, e->lny_day);
  if (today < lny) {
    ly--;
    if (ly < KLC_FIRST_YEAR) return false;
    e = &KLC_YEARS[ly - KLC_FIRST_YEAR];
    lny = klc_days_from_civil(ly, e->lny_mon, e->lny_day);
  }

  long off = today - lny;
  int n_months = e->leap_month ? 13 : 12;
  int month = 1;
  bool leap = false;
  for (int i = 0; i < n_months; i++) {
    int len = ((e->big_mask >> i) & 1) ? 30 : 29;
    if (off < len) {
      out->month = month;
      out->day   = (int)off + 1;
      out->leap  = leap;
      return true;
    }
    off -= len;
    if (!leap && month == e->leap_month) leap = true;
    else { leap = false; month++; }
  }
  return false;  /* past the end of the lunar year: table inconsistency */
}

/* ======================= 24 solar terms ================================= */

#define KST_FIRST_YEAR 2026
#define KST_LAST_YEAR  2035

static const char * const TERM_KR[24] = {
  "소한", "대한", "입춘", "우수", "경칩", "춘분", "청명", "곡우",
  "입하", "소만", "망종", "하지", "소서", "대서", "입추", "처서",
  "백로", "추분", "한로", "상강", "입동", "소설", "대설", "동지",
};

static const uint16_t TERM_DATES[KST_LAST_YEAR - KST_FIRST_YEAR + 1][24] = {
  { 105,120,204,219,305,320,405,420,505,521,606,621,707,723,807,823,907,923,1008,1023,1107,1122,1207,1222 }, /* 2026 */
  { 105,120,204,219,306,321,405,420,506,521,606,621,707,723,808,823,908,923,1008,1024,1108,1122,1207,1222 }, /* 2027 */
  { 106,120,204,219,305,320,404,419,505,520,605,621,706,722,807,822,907,922,1008,1023,1107,1122,1206,1221 }, /* 2028 */
  { 105,120,203,218,305,320,404,420,505,521,605,621,707,722,807,823,907,923,1008,1023,1107,1122,1207,1221 }, /* 2029 */
  { 105,120,204,218,305,320,405,420,505,521,605,621,707,723,807,823,907,923,1008,1023,1107,1122,1207,1222 }, /* 2030 */
  { 105,120,204,219,306,321,405,420,506,521,606,621,707,723,808,823,908,923,1008,1023,1108,1122,1207,1222 }, /* 2031 */
  { 106,120,204,219,305,320,404,419,505,520,605,621,706,722,807,822,907,922,1008,1023,1107,1122,1206,1221 }, /* 2032 */
  { 105,120,203,218,305,320,404,420,505,521,605,621,707,722,807,823,907,923,1008,1023,1107,1122,1207,1221 }, /* 2033 */
  { 105,120,204,218,305,320,405,420,505,521,605,621,707,723,807,823,907,923,1008,1023,1107,1122,1207,1222 }, /* 2034 */
  { 105,120,204,219,306,321,405,420,506,521,606,621,707,723,808,823,908,923,1008,1023,1107,1122,1207,1222 }, /* 2035 */
};

/* Name of the solar-term period the date falls in, i.e. the most recent
 * term on or before it. Jan 1..4 belongs to the previous year's 동지.
 * *is_today is set when the date is the 절입 day itself.
 * NULL outside the table range. */
static const char * kst_current_term(const struct tm * t, bool * is_today) {
  if (is_today) *is_today = false;
  int y = t->tm_year + 1900;
  if (y < KST_FIRST_YEAR || y > KST_LAST_YEAR) return NULL;
  uint16_t md = (uint16_t)((t->tm_mon + 1) * 100 + t->tm_mday);
  const uint16_t * row = TERM_DATES[y - KST_FIRST_YEAR];
  if (md < row[0]) return TERM_KR[23];
  int idx = 0;
  for (int i = 0; i < 24; i++) {
    if (row[i] <= md) idx = i;
    else break;
  }
  if (is_today && row[idx] == md) *is_today = true;
  return TERM_KR[idx];
}

/* ======================= Public holidays ================================ */

/* YYYYMMDD, sorted. Covers 2026-2030 including substitute holidays.
 * 제헌절 is a statutory public holiday again from 2026 (법률 제21338호).
 * Extend this table as the years run out - the lunar holidays (설날, 추석,
 * 부처님오신날) and the substitute days cannot be computed from tm alone. */
typedef struct {
  uint32_t ymd;
  const char * name;
} kr_holiday_t;

static const kr_holiday_t KR_HOLIDAYS[] = {
  /* 2026 */
  { 20260101, "신정" },
  { 20260216, "설날" }, { 20260217, "설날" }, { 20260218, "설날" },
  { 20260301, "삼일절" }, { 20260302, "대체휴일" },
  { 20260505, "어린이날" },
  { 20260524, "부처님오신날" }, { 20260525, "대체휴일" },
  { 20260606, "현충일" },
  { 20260717, "제헌절" },
  { 20260815, "광복절" }, { 20260817, "대체휴일" },
  { 20260924, "추석" }, { 20260925, "추석" }, { 20260926, "추석" },
  { 20261003, "개천절" }, { 20261005, "대체휴일" },
  { 20261009, "한글날" },
  { 20261225, "크리스마스" },
  /* 2027 */
  { 20270101, "신정" },
  { 20270206, "설날" }, { 20270207, "설날" }, { 20270208, "설날" },
  { 20270209, "대체휴일" },
  { 20270301, "삼일절" },
  { 20270505, "어린이날" },
  { 20270513, "부처님오신날" },
  { 20270606, "현충일" },
  { 20270717, "제헌절" }, { 20270719, "대체휴일" },
  { 20270815, "광복절" }, { 20270816, "대체휴일" },
  { 20270914, "추석" }, { 20270915, "추석" }, { 20270916, "추석" },
  { 20271003, "개천절" }, { 20271004, "대체휴일" },
  { 20271009, "한글날" }, { 20271011, "대체휴일" },
  { 20271225, "크리스마스" }, { 20271227, "대체휴일" },
  /* 2028 */
  { 20280101, "신정" },
  { 20280126, "설날" }, { 20280127, "설날" }, { 20280128, "설날" },
  { 20280301, "삼일절" },
  { 20280502, "부처님오신날" },
  { 20280505, "어린이날" },
  { 20280606, "현충일" },
  { 20280717, "제헌절" },
  { 20280815, "광복절" },
  { 20281002, "추석" }, { 20281003, "추석" }, { 20281004, "추석" },
  { 20281005, "대체휴일" },   /* 개천절 coincides with 추석 */
  { 20281009, "한글날" },
  { 20281225, "크리스마스" },
  /* 2029 */
  { 20290101, "신정" },
  { 20290212, "설날" }, { 20290213, "설날" }, { 20290214, "설날" },
  { 20290301, "삼일절" },
  { 20290505, "어린이날" }, { 20290507, "대체휴일" },
  { 20290520, "부처님오신날" }, { 20290521, "대체휴일" },
  { 20290606, "현충일" },
  { 20290717, "제헌절" },
  { 20290815, "광복절" },
  { 20290921, "추석" }, { 20290922, "추석" }, { 20290923, "추석" },
  { 20290924, "대체휴일" },
  { 20291003, "개천절" },
  { 20291009, "한글날" },
  { 20291225, "크리스마스" },
  /* 2030 */
  { 20300101, "신정" },
  { 20300202, "설날" }, { 20300203, "설날" }, { 20300204, "설날" },
  { 20300205, "대체휴일" },
  { 20300301, "삼일절" },
  { 20300505, "어린이날" }, { 20300506, "대체휴일" },
  { 20300509, "부처님오신날" },
  { 20300606, "현충일" },
  { 20300717, "제헌절" },
  { 20300815, "광복절" },
  { 20300911, "추석" }, { 20300912, "추석" }, { 20300913, "추석" },
  { 20301003, "개천절" },
  { 20301009, "한글날" },
  { 20301225, "크리스마스" },
};

/* Holiday name for a YYYYMMDD date, NULL when it is a working day.
 * Called once per day change, so a linear scan costs nothing. */
static const char * kr_holiday_name(uint32_t ymd) {
  for (size_t i = 0; i < sizeof(KR_HOLIDAYS) / sizeof(KR_HOLIDAYS[0]); i++) {
    if (KR_HOLIDAYS[i].ymd == ymd) return KR_HOLIDAYS[i].name;
  }
  return NULL;
}

/* Sunday or a listed public holiday: draw the weekday in red. */
static bool kr_is_red_day(const struct tm * t) {
  if (t->tm_wday == 0) return true;
  uint32_t ymd = (uint32_t)(t->tm_year + 1900) * 10000u
               + (uint32_t)(t->tm_mon + 1) * 100u
               + (uint32_t)t->tm_mday;
  return kr_holiday_name(ymd) != NULL;
}

/* Non-holiday seasonal festivals tied to the lunar date. */
static const char * kr_lunar_festival(const klc_date_t * ld) {
  if (ld->leap) return NULL;
  if (ld->month == 1 && ld->day == 15) return "정월대보름";
  if (ld->month == 5 && ld->day == 5)  return "단오";
  if (ld->month == 7 && ld->day == 7)  return "칠석";
  return NULL;
}

#endif /* KOREAN_CALENDAR_H */
