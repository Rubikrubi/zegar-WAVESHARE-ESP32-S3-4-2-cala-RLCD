// ============================================================================
//  Zegar NTP RLCD v7.0 — dokladnosc + bateria + niezawodnosc
//   Zmiany v7.0:
//   [DOKLADNOSC — cel: dryf < 1 s/dobe]
//    - autokalibracja offsetu RTC (ENABLE_RTC_AUTOTRIM): pomiar dryftu na
//      granicy sekundy PCF85063A (rozdzielczosc ~ms zamiast 1 s), po kazdym
//      wiarygodnym oknie NTP->NTP (>= 6 h) automatyczna korekta rejestru 0x02;
//      offset trwale w NVS ("rtcOffLsb"), RTC_OFFSET_VALUE to tylko start,
//    - kotwiczenie czasu z RTC w POLOWIE sekundy (+0.5 s): sredni blad fazy
//      zegara systemowego po wybudzeniu spada z ~0.5 s do ~0.25 s,
//   [BATERIA]
//    - CPU w dzien 80 MHz (bylo 160; SPI/I2C/WiFi taktowane z APB 80 MHz,
//      wiec bez wplywu na peryferia); 160 MHz tylko na czas audio (MP3),
//    - NTP konczy probe 8 s po zerwaniu WiFi zamiast czekac 50 s z radiem,
//   [NIEZAWODNOSC]
//    - odblokowanie zawieszonej szyny I2C (9x SCL + STOP) przy bledzie sesji
//      RTC — slave trzymajacy SDA po resecie w polowie transakcji,
//    - retry z backoffem takze po nieudanej RECZNEJ synchronizacji bez czasu,
//   [PORZADKI] WebManager::redirectHome() (dedup 6x), komentarze.
//  Pozostale inwarianty czasu/energii/RTC/sleep przeniesione bez zmian.
//
//  Sprzet: Waveshare ESP32-S3-RLCD-4.2 (ST7305 refleksyjny 400x300 landscape,
//          ES8311 audio codec, PCF85063A RTC). Zasilanie: 1x Samsung INR18650-35E.
//  Arduino IDE: plytka "ESP32S3 Dev Module", Flash 8MB, partycja "8M with spiffs".
//  Pliki obok .ino: photo_font.h  (+ MP3 budzikow na partycji danych: /alarm1..3.mp3)
//  Biblioteki: U8g2 (>=2.35), ESP32 core 3.x, ESP32-audioI2S 3.4.5.
// ============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Wire.h>
#include <atomic>
#include <ctype.h>
#include <limits.h>
#include <time.h>
#include <sys/time.h>
#include <FS.h>
#include <SPIFFS.h>
#include <LittleFS.h>
#include <FFat.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <Audio.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_sleep.h>
#include <driver/gpio.h>
#include <driver/rtc_io.h>
#include "esp_sntp.h"
#include "esp_task_wdt.h"
#include "esp_system.h"

#include "photo_font.h"

// Deklaracje wyprzedzajace struktur uzywanych w sygnaturach WOLNYCH funkcji przed
// ich definicja. Arduino auto-generuje prototypy tych funkcji i wstawia je na
// gorze pliku (przed definicja struktury) -> bez tego "DrainState not declared".
struct DrainState;

// ============================================================================
//  [0] WERSJA, BUILD INFO, FLAGI FUNKCJI
// ============================================================================
#define FW_NAME        "Zegar NTP RLCD"
#define FW_VERSION     "7.0"
#define FW_BUILD_DATE  __DATE__
#define FW_BUILD_TIME  __TIME__

// 1=wlaczone, 0=wylaczone (#if usuwa kod z kompilacji -> zero narzutu gdy off).
#define ENABLE_WATCHDOG            1   // sprzetowy reset po zawieszeniu petli (TWDT)
#define ENABLE_LOW_BATTERY_PROTECT 1   // ochrona ogniwa: degradacja + ochronny sen
#define ENABLE_DIAGNOSTICS         1   // strona /info + liczniki resetow
#define ENABLE_RTC_AUTOTRIM        1   // automatyczna kalibracja offsetu RTC z pomiaru NTP

// ============================================================================
//  Watchdog zadania (TWDT). Timeout > najdluzszy light-sleep dzienny (60 s).
//  Deep-sleep RESETUJE chip -> TWDT startuje od nowa w setup().
// ============================================================================
constexpr uint32_t WDT_TIMEOUT_S = 90;

static inline void watchdogSetup() {
#if ENABLE_WATCHDOG
  #if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    esp_task_wdt_config_t cfg = {};
    cfg.timeout_ms     = WDT_TIMEOUT_S * 1000U;
    cfg.idle_core_mask = 0;        // nie nadzoruj IDLE (sny sa celowe) — tylko loopTask
    cfg.trigger_panic  = true;     // timeout -> panic -> reset (odzysk bez nadzoru)
    if (esp_task_wdt_init(&cfg) == ESP_ERR_INVALID_STATE) {
      esp_task_wdt_reconfigure(&cfg);
    }
  #else
    esp_task_wdt_init(WDT_TIMEOUT_S, true);
  #endif
  esp_task_wdt_add(NULL);
  esp_task_wdt_reset();
#endif
}

static inline void watchdogFeed() {
#if ENABLE_WATCHDOG
  esp_task_wdt_reset();
#endif
}

// Detektor brownout sprzetowy (~2.51 V) CELOWO zostaje WLACZONY (chroni FLASH przy
// zalamaniu napiecia). Programowa ochrona ogniwa usypia juz przy ~3.05 V.

// ============================================================================
//  power_runtime — estymator zuzycia baterii (czysta arytmetyka na probkach SoC).
// ============================================================================
#ifndef DRAIN_CAP_MAH
#define DRAIN_CAP_MAH        3400
#endif
#define DRAIN_CHARGE_HYST    5         // wzrost SoC% liczony jako "doladowano" -> reset kotwicy
#define DRAIN_MIN_WINDOW_SEC 43200     // min. 12 h danych (plaskie plateau Li-ion = szum ADC)
#define DRAIN_MIN_DSOC       2         // min. spadek SoC%, by estymata miala sens

struct DrainState {
  bool     valid;
  int64_t  anchorEpoch;
  uint8_t  anchorSoc;
  int64_t  lastEpoch;
  uint8_t  lastSoc;
};

static inline void drainReset(DrainState &d, int64_t epoch, uint8_t soc) {
  d.valid = true;
  d.anchorEpoch = epoch; d.anchorSoc = soc;
  d.lastEpoch   = epoch; d.lastSoc   = soc;
}

static inline void drainOnSample(DrainState &d, int64_t epoch, uint8_t soc) {
  if (!d.valid) { drainReset(d, epoch, soc); return; }
  if (soc > d.anchorSoc + DRAIN_CHARGE_HYST || soc > d.lastSoc + DRAIN_CHARGE_HYST) {
    drainReset(d, epoch, soc); return;   // wykryto ladowanie -> restart okna
  }
  d.lastEpoch = epoch; d.lastSoc = soc;
}

// mAh/dobe. <0 => za malo danych; 0 => brak mierzalnego zuzycia.
static inline float drainMahPerDay(const DrainState &d) {
  if (!d.valid) return -1.0f;
  int64_t dt = d.lastEpoch - d.anchorEpoch;
  if (dt < DRAIN_MIN_WINDOW_SEC) return -1.0f;
  int dSoc = (int)d.anchorSoc - (int)d.lastSoc;
  if (dSoc < DRAIN_MIN_DSOC) return 0.0f;
  return (dSoc / 100.0f) * (float)DRAIN_CAP_MAH / (dt / 86400.0f);
}

// Dni do rozladowania przy biezacym tempie. <0 => nieznane.
static inline float drainDaysRemaining(const DrainState &d, uint8_t curSoc) {
  float perDay = drainMahPerDay(d);
  if (perDay <= 0.0f) return -1.0f;
  return (curSoc / 100.0f) * (float)DRAIN_CAP_MAH / perDay;
}

// ============================================================================
//  [1] KONFIGURACJA — adresy I2C, kalibracja RTC, piny, stale, TZ, serwery, WiFi
// ============================================================================
#define ES8311_ADDR        0x18
#define PCF85063A_ADDR     0x51
#define RTC_FORMAT_MARKER  0xA5

// --- Kalibracja dryftu krysztalu RTC (rejestr Offset 0x02 PCF85063A) ---------
//   offset DODATNI spowalnia zegar; UJEMNY przyspiesza. Offset[LSB]=blad_ppm/4.34.
//   Zmierzony surowy dryft -2 s/24h -> -5 LSB = -21.7 ppm. 0x7B = -5 LSB, MODE=0.
//   RTC_OFFSET_VALUE to wartosc STARTOWA (fabryczna); z ENABLE_RTC_AUTOTRIM biezacy
//   offset zyje w NVS ("rtcOffLsb") i jest korygowany automatycznie z pomiarow NTP.
#define RTC_OFFSET_REG            0x02
#define RTC_OFFSET_VALUE          0x7B    // -5 LSB, MODE=0 (-21.7 ppm; przyspiesza zegar)
#define RTC_OFFSET_LSB_PPM_MODE0  4.34f
#define RTC_OFFSET_LSB_PPM_MODE1  4.069f
#define RTC_OFFSET_RETRIES        4
#define PPM_SEC_PER_DAY           0.0864f

#if ENABLE_RTC_AUTOTRIM
// Pomiar fazowy (granica sekundy RTC vs czas NTP) ma rozdzielczosc ~ms, wiec juz
// jedno okno 6 h daje ppm z niepewnoscia ~0.1 ppm — wystarcza do korekty o 1 LSB
// (4.34 ppm). Kwantyzacja rejestru ogranicza dryf resztkowy do ~±2.2 ppm,
// czyli ~±0.19 s/dobe — ponizej celu 1 s/dobe.
constexpr int64_t AUTOTRIM_MIN_WINDOW_SEC    = 6 * 3600;    // min. okno NTP->NTP
constexpr int64_t AUTOTRIM_MAX_WINDOW_SEC    = 7 * 86400;   // dluzsze = watpliwa kotwica
constexpr float   AUTOTRIM_MAX_PLAUSIBLE_PPM = 150.0f;      // wiecej = bledny pomiar, nie kalibruj
constexpr uint32_t RTC_BOUNDARY_POLL_TIMEOUT_MS = 1300;     // >1 s: przewrot MUSI nadejsc
#endif

// --- Piny: wyswietlacz RLCD (HARDWARE SPI; SPI.begin w DisplayManager::setup) -
#define RLCD_SCK_PIN   11
#define RLCD_MOSI_PIN  12
#define RLCD_DC_PIN    5
#define RLCD_CS_PIN    40
#define RLCD_RST_PIN   41
#define LCD_WIDTH      400
#define LCD_HEIGHT     300

// --- Piny: I2C (kodek + RTC), audio I2S, przyciski, akumulator ----------------
#define I2C_SDA_PIN    13
#define I2C_SCL_PIN    14
#define RTC_INT_PIN    15

#define I2S_MCLK_PIN   16
#define I2S_BCLK_PIN   9
#define I2S_WS_PIN     45
#define I2S_DOUT_PIN   8
#define PA_ENABLE_PIN  46

#define KEY_PIN            18   // krotko: wycisz budzik / wlacz panel WWW; dlugo: nic
#define PREVIEW_BUTTON_PIN 0    // krotko: podglad najblizszego budzika

#define BATTERY_ADC_CHANNEL          ADC_CHANNEL_3   // = GPIO4 na ESP32-S3
#define BATTERY_DIVIDER_RATIO        3.0f
#define BATTERY_VOLTAGE_CALIBRATION  1.017f
#define BATTERY_CAPACITY_MAH         3400    // Samsung INR18650-35E
#define BATTERY_SAMPLE_COUNT         16
#define BATTERY_EMPTY_VOLTAGE        3.00f
#define BATTERY_FULL_VOLTAGE         4.20f
#define BATTERY_FULL_FLOOR           99
#define BATTERY_FULL_HOLD_FLOOR      98
#define BATTERY_READ_INTERVAL_MS     (10UL * 60UL * 1000UL)
#define BATTERY_BUSY_RETRY_MS        (30UL * 1000UL)

// --- Ochrona niskiego stanu ogniwa (dwustopniowa) -----------------------------
constexpr uint8_t  BATTERY_LOW_PERCENT          = 15;
constexpr float    BATTERY_CRITICAL_VOLTAGE     = 3.05f;
constexpr uint8_t  BATTERY_CRITICAL_PERCENT     = 3;
constexpr uint32_t BATTERY_CRITICAL_RECHECK_SEC = 30UL * 60UL;

// --- Parametry logiki zegara / budzikow ---------------------------------------
constexpr int      ALARM_COUNT                = 3;
constexpr int      SOUND_COUNT                = 3;
constexpr int      MAX_AUDIO_VOLUME           = 21;
constexpr uint8_t  DEFAULT_AUDIO_VOLUME       = 10;
constexpr uint8_t  CODEC_VOLUME_PERCENT       = 100;
constexpr uint32_t ALARM_LATE_WINDOW_SEC      = 20;
constexpr uint32_t RING_RETRY_MS              = 3000;
constexpr uint32_t RING_MIN_PLAY_MS           = 750;
constexpr uint8_t  RING_MAX_ERRORS            = 10;
constexpr uint32_t RING_TIMEOUT_MS            = 15UL * 60UL * 1000UL;

constexpr uint32_t WEB_WINDOW_MS              = 5UL * 60UL * 1000UL;
constexpr uint32_t WEB_WIFI_GRACE_MS          = 10000;
constexpr uint32_t WIFI_HINT_FALLBACK_MS      = 5000;
constexpr uint32_t NTP_WIFI_TIMEOUT_MS        = 15000;
constexpr uint32_t NTP_PACKET_TIMEOUT_MS      = 50000;  // lwIP SNTP 3 serwery co ~15 s -> 50 s pokrywa
constexpr uint32_t NTP_WIFI_LOST_GRACE_MS     = 8000;   // zerwane WiFi w trakcie proby: tyle na powrot
constexpr uint32_t NTP_ALARM_CATCHUP_SEC      = 90;

constexpr uint32_t NTP_QUICK_RETRY_MS         = 30000;        // 30 s — seria szybka po starcie
constexpr uint32_t NTP_BACKOFF_MIN_MS         = 60000;        // 1 min
constexpr uint32_t NTP_BACKOFF_MAX_MS         = 3600000;      // 1 h

constexpr int      NTP_HOUR                   = 18;  // codzienna synchronizacja 18:00-18:05 (+ dogonienie do 23:00)
constexpr int      NTP_MIN_WINDOW             = 5;
constexpr uint32_t RTC_WRITE_RETRY_MS         = 5000;
constexpr uint8_t  RTC_WRITE_RETRIES          = 12;
constexpr int      RTC_WAKE_MAX_SKEW_SEC      = 30;

// CPU: w dzien 80 MHz wystarcza (render RLCD jest ograniczony SPI 2 MHz, a SPI/
// I2C/WiFi taktowane sa z APB, ktore przy >=80 MHz CPU i tak chodzi 80 MHz).
// 160 MHz tylko na czas dekodowania MP3 (AudioManager podbija/odpuszcza).
constexpr uint32_t DAY_CPU_MHZ                = 80;
constexpr uint32_t AUDIO_CPU_MHZ              = 160;
constexpr uint32_t NIGHT_CPU_MHZ              = 40;
#define POWER_DOWN_FLASH_IN_SLEEP 0   // NIEKOMPATYBILNE na tej plytce -> zostaje 0 NA STALE.
#define FAST_NTP_RECONNECT 1          // 1=uzyj zapamietanego kanalu+BSSID (krocej radia)
#define ST7305_STATIC_LPM 0           // 0=WYLACZONE. Tryb LPM panelu — nieprzetestowany; nie wysylamy.

constexpr uint64_t IDLE_LIGHT_SLEEP_US        = 900000ULL;    // fallback gdy czas nieznany
constexpr uint64_t IDLE_LIGHT_SLEEP_MAX_SEC   = 60ULL;        // sen do granicy minuty -> 1 wybudzenie/min
constexpr int      NIGHT_START_HOUR           = 23;
constexpr int      NIGHT_START_MIN            = 59;
constexpr int      NIGHT_END_HOUR             = 6;
constexpr int      NIGHT_END_MIN              = 0;
constexpr uint32_t NIGHT_MIN_SLEEP_SEC        = 5;
constexpr uint32_t NIGHT_FALLBACK_CHUNK_SEC   = 3600;         // maks. odcinek snu gdy alarm RTC nie uzbroil sie
constexpr uint32_t NIGHT_FALLBACK_ALARM_CHUNK_SEC = 300;      // jw. gdy najblizszy jest budzik: krocej

constexpr uint32_t BUTTON_DEBOUNCE_MS         = 40;
constexpr uint32_t WEB_LONG_PRESS_MS          = 1500;         // KEY przytrzymany >= tyle -> panel WWW
constexpr uint32_t ALARM_PREVIEW_MS           = 5000;
constexpr uint32_t WAKE_HOLD_MS               = 15000;        // po wybudzeniu przyciskiem w nocy: pokaz tresc 15 s

// --- Stale konfiguracyjne (TZ, NTP, WiFi, dzwieki) ----------------------------
static const char *TZ_POLAND    = "CET-1CEST,M3.5.0/2,M10.5.0/3";
static const char *NTP_SERVER_1 = "pool.ntp.org";
static const char *NTP_SERVER_2 = "time.google.com";
static const char *NTP_SERVER_3 = "time.cloudflare.com";

static const char *DEFAULT_WIFI_SSID = "";
static const char *DEFAULT_WIFI_PASS = "";

static const char *SOUND_FILES[SOUND_COUNT] = { "/alarm1.mp3", "/alarm2.mp3", "/alarm3.mp3" };
static const char *SOUND_NAMES[SOUND_COUNT] = { "Dzwiek 1", "Dzwiek 2", "Dzwiek 3" };

static const char *NVS_NS = "zegar7";   // przestrzen nazw Preferences

// ============================================================================
//  Model danych
// ============================================================================
struct AlarmConfig {
  uint8_t enabled;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint8_t volume;     // 0..MAX_AUDIO_VOLUME
  uint8_t sound;      // indeks 0..SOUND_COUNT-1 -> SOUND_FILES
  char    label[16];
};

// Stan przezywajacy deep-sleep (noc/krytyka). POD w RTC_DATA_ATTR: konstruktor nie
// rusza go przy wybudzeniu (wartosci trwaja), a cold boot zeruje go jawnie w
// App::setup() (czesc pol wymaga sentineli -1, nie zwyklego zera).
struct PersistentState {
  uint8_t    batteryPercent;
  float      batteryVoltage;
  bool       batteryHasReading;
  DrainState drain;
  int        lastSummaryDay;
  uint8_t    wifiBssid[6];
  int32_t    wifiChannel;
  bool       wifiHint;
  uint16_t   bootCount;
  int64_t    coldBootEpoch;
  int32_t    lastRtcDriftSec;
  int64_t    lastNtpSyncEpoch;
  int64_t    prevNtpSyncEpoch;
  int        lastTriggerDay[ALARM_COUNT];   // dedup wyzwolen budzikow
  int        ntpStartedDay;                 // dobowa synchronizacja 18:00
  int        ntpMorningDay;                 // poranne NTP po wybudzeniu 06:00
#if ENABLE_RTC_AUTOTRIM
  bool       rtcPhaseAligned;               // ostatni writeRtc wyrownal faze -> okno mierzalne
  float      lastDriftPpm;                  // ostatni pomiar fazowy (diagnostyka /info)
  int32_t    lastDriftWindowSec;            // dlugosc okna tego pomiaru
  uint16_t   autotrimAdjustCount;           // ile razy offset skorygowano automatycznie
#endif
};
static RTC_DATA_ATTR PersistentState g_persist;

// ============================================================================
//  Singletony sterownikow sprzetu (konstruowane przy starcie statycznym; same
//  nie dotykaja sprzetu az do begin()). Managery owijaja na nich logike.
// ============================================================================
// RST = U8X8_PIN_NONE: U8g2 nigdy sam nie pulsuje resetu — robimy to recznie w
// DisplayManager::setup() przy KAZDYM wejsciu w setup (zimny start ORAZ deep-sleep).
static U8G2_ST7305_300X400_F_4W_HW_SPI display(
  U8G2_R1, RLCD_CS_PIN, RLCD_DC_PIN, U8X8_PIN_NONE);
static WebServer   server(80);
static Preferences prefs;
static Audio       audio;

static const gpio_num_t HOLD_PINS[] = {
  (gpio_num_t)RLCD_DC_PIN, (gpio_num_t)RLCD_CS_PIN, (gpio_num_t)RLCD_SCK_PIN,
  (gpio_num_t)RLCD_MOSI_PIN, (gpio_num_t)RLCD_RST_PIN
};

// Gate atomic odrozniajacy realna odpowiedz serwera NTP od czasu wstepnie
// ustawionego z RTC. Callback SNTP musi byc wolna funkcja C.
static std::atomic<bool> g_ntpGotPacket{false};
static std::atomic<bool> g_ntpAcceptPacket{false};
static void onSntpSync(struct timeval *tv) {
  if (tv && g_ntpAcceptPacket.load(std::memory_order_acquire))
    g_ntpGotPacket.store(true, std::memory_order_release);
}

static void audioInfo(Audio::msg_t m) {
  Serial.printf("Audio %s: %s\n", m.s ? m.s : "", m.msg ? m.msg : "");
}

// ============================================================================
//  [3a] CZAS — konwersje cywilne (czyste funkcje, bez stanu)
// ============================================================================
static int daysFromCivil(int y, unsigned m, unsigned d) {
  y -= m <= 2;
  const int era = (y >= 0 ? y : y - 399) / 400;
  const unsigned yoe = (unsigned)(y - era * 400);
  const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + (int)doe - 719468;
}

static time_t utcEpochFromCivil(int y, int mo, int d, int h, int mi, int s) {
  return (time_t)daysFromCivil(y, (unsigned)mo, (unsigned)d) * 86400 + h * 3600 + mi * 60 + s;
}

static int localDayKey(const struct tm &t) {
  return (t.tm_year + 1900) * 400 + t.tm_yday;
}

static bool isNightMode(const struct tm &t) {
  int now = t.tm_hour * 60 + t.tm_min;
  int start = NIGHT_START_HOUR * 60 + NIGHT_START_MIN;
  int end   = NIGHT_END_HOUR * 60 + NIGHT_END_MIN;
  if (start < end) return now >= start && now < end;
  if (start > end) return now >= start || now < end;
  return false;
}

static bool isSupportedEpoch(time_t epoch) {
  if (epoch < 1700000000) return false;
  struct tm gm;
  gmtime_r(&epoch, &gm);
  int year = gm.tm_year + 1900;
  return year >= 2024 && year <= 2099;
}

static bool getLocalTm(struct tm *out) {
  time_t now = time(nullptr);
  if (!isSupportedEpoch(now)) return false;
  localtime_r(&now, out);
  return true;
}

static void setSystemEpoch(time_t epoch) {
  struct timeval tv = { .tv_sec = epoch, .tv_usec = 0 };
  settimeofday(&tv, nullptr);
}

// Kotwiczenie czasu odczytanego z RTC: odczyt ma rozdzielczosc 1 s i nieznana
// faze (0..1 s po przewrocie), wiec wpis w POLOWIE sekundy minimalizuje
// oczekiwany |blad| fazy zegara systemowego (0.25 s zamiast 0.5 s).
static void setSystemEpochMidSecond(time_t epoch) {
  struct timeval tv = { .tv_sec = epoch, .tv_usec = 500000 };
  settimeofday(&tv, nullptr);
}

static time_t epochForClock(struct tm base, int h, int mi, int s, int addDays) {
  base.tm_hour = h; base.tm_min = mi; base.tm_sec = s;
  base.tm_mday += addDays;     // mktime normalizuje przepelnienie
  base.tm_isdst = -1;          // mktime sam ustali DST
  return mktime(&base);        // poprawne przez noc zmiany czasu (doba != 86400 s)
}

static bool alarmDueNow(const struct tm &nowTm, time_t now, uint8_t h, uint8_t mi,
                        uint8_t s, uint32_t lateWindowSec, int *occurrenceDay) {
  time_t alarmAt = epochForClock(nowTm, h, mi, s, 0);
  if (alarmAt > now) alarmAt = epochForClock(nowTm, h, mi, s, -1);
  time_t delaySec = now - alarmAt;
  if (delaySec < 0 || (uint64_t)delaySec > lateWindowSec) return false;
  if (occurrenceDay) {
    struct tm alarmTm;
    localtime_r(&alarmAt, &alarmTm);
    *occurrenceDay = localDayKey(alarmTm);
  }
  return true;
}

static uint8_t bcd2dec(uint8_t v) { return (v >> 4) * 10 + (v & 0x0F); }
static uint8_t dec2bcd(uint8_t v) { return (uint8_t)(((v / 10) << 4) | (v % 10)); }
static bool    validBcd(uint8_t v) { return (v & 0x0F) <= 9 && ((v >> 4) & 0x0F) <= 9; }
static bool    leapYear(int y) { return (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0); }

static uint8_t daysInMonth(int y, int m) {
  static const uint8_t md[] = { 31,28,31,30,31,30,31,31,30,31,30,31 };
  if (m == 2 && leapYear(y)) return 29;
  return md[m - 1];
}

static bool validDateTime(int y, int mo, int d, int h, int mi, int s) {
  if (y < 2024 || y > 2099 || mo < 1 || mo > 12) return false;
  if (d < 1 || d > daysInMonth(y, mo)) return false;
  return h >= 0 && h <= 23 && mi >= 0 && mi <= 59 && s >= 0 && s <= 59;
}

// --- Dekodowanie/kodowanie rejestru Offset (0x02) -----------------------------
static int8_t decodeRtcOffset(uint8_t raw) {
  int8_t v = (int8_t)(raw & 0x7F);
  if (v & 0x40) v -= 0x80;            // sign-extend (7-bit U2)
  return v;
}
static uint8_t encodeRtcOffset(int lsb, bool mode1) {
  if (lsb > 63)  lsb = 63;
  if (lsb < -64) lsb = -64;
  uint8_t reg = (uint8_t)(lsb & 0x7F);
  if (mode1) reg |= 0x80;
  return reg;
}
static float rtcOffsetPpm(int8_t lsb, bool mode1) {
  return (float)lsb * (mode1 ? RTC_OFFSET_LSB_PPM_MODE1 : RTC_OFFSET_LSB_PPM_MODE0);
}
// Sugerowana DOCELOWA wartosc offsetu po zmierzeniu dryfu (do /info — recznie).
static int suggestedTotalOffsetLsb(float driftSecPerDay, int currentLsb) {
  float lsbF  = (driftSecPerDay / PPM_SEC_PER_DAY) / RTC_OFFSET_LSB_PPM_MODE0;
  int   delta = (int)(lsbF + (lsbF >= 0 ? 0.5f : -0.5f));
  int   total = currentLsb + delta;
  if (total > 63)  total = 63;
  if (total < -64) total = -64;
  return total;
}

static const char *resetReasonText(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "zasilanie (zimny start)";
    case ESP_RST_EXT:       return "zewnetrzny pin RST";
    case ESP_RST_SW:        return "software (esp_restart)";
    case ESP_RST_PANIC:     return "panic / wyjatek";
    case ESP_RST_INT_WDT:   return "watchdog przerwan";
    case ESP_RST_TASK_WDT:  return "watchdog zadania (TWDT)";
    case ESP_RST_WDT:       return "inny watchdog";
    case ESP_RST_DEEPSLEEP: return "deep-sleep (planowe)";
    case ESP_RST_BROWNOUT:  return "brownout (spadek napiecia)";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "nieznany";
  }
}

// ============================================================================
//  [9a] POMOCNICZE FUNKCJE PANELU WWW (czyste, bez stanu)
// ============================================================================
static String htmlEscape(const String &v) {
  String out; out.reserve(v.length() + 8);
  for (size_t i = 0; i < v.length(); i++) {
    char c = v[i];
    if (c == '&') out += F("&amp;");
    else if (c == '<') out += F("&lt;");
    else if (c == '>') out += F("&gt;");
    else if (c == '"') out += F("&quot;");
    else if (c == '\'') out += F("&#39;");
    else out += c;
  }
  return out;
}

static bool parseClock(const String &v, uint8_t *h, uint8_t *mi, uint8_t *s) {
  int hh = 0, mm = 0, ss = 0, consumed = 0;
  int n = sscanf(v.c_str(), "%d:%d:%d%n", &hh, &mm, &ss, &consumed);
  if (n == 2) { ss = 0; consumed = 0; if (sscanf(v.c_str(), "%d:%d%n", &hh, &mm, &consumed) != 2) return false; }
  if (n < 2 || hh < 0 || hh > 23 || mm < 0 || mm > 59 || ss < 0 || ss > 59) return false;
  while (consumed < (int)v.length() && isspace((unsigned char)v[consumed])) consumed++;
  if (consumed != (int)v.length()) return false;
  *h = (uint8_t)hh; *mi = (uint8_t)mm; *s = (uint8_t)ss;
  return true;
}

static String currentTimeText() {
  struct tm t;
  if (!getLocalTm(&t)) return "czas nieustawiony";
  char buf[48];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
           t.tm_year + 1900, t.tm_mon + 1, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec);
  return String(buf);
}

static bool isMp3NameChar(char c) {
  return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
         (c >= '0' && c <= '9') || c == '_' || c == '-' || c == '.';
}

// keepCase: kasowanie musi uzyc dokladnie nazwy z listy (FS rozroznia wielkosc liter).
static bool normalizeMp3Path(String name, String &path, bool keepCase) {
  name.trim(); name.replace("\\", "/");
  int slash = name.lastIndexOf('/');
  if (slash >= 0) name = name.substring(slash + 1);
  name.trim();
  if (name.length() < 5 || name.length() > 25) return false;   // SPIFFS: pelna sciezka <=31 znakow
  String lower = name; lower.toLowerCase();
  if (!lower.endsWith(".mp3")) return false;
  if (!keepCase) name = lower;
  for (size_t i = 0; i < name.length(); i++) if (!isMp3NameChar(name[i])) return false;
  path = String("/") + name;
  return true;
}

static String formatFileSize(size_t bytes) {
  if (bytes < 1024) return String((uint32_t)bytes) + " B";
  if (bytes < 1024UL * 1024UL) return String(bytes / 1024.0f, 1) + " KB";
  return String(bytes / (1024.0f * 1024.0f), 2) + " MB";
}

// ============================================================================
//  Sterownik kodeka ES8311 (sekwencja rejestrow — port z es8311.c). Samowystarczalny.
// ============================================================================
class ES8311Codec {
public:
  explicit ES8311Codec(TwoWire *bus = &Wire) : wire_(bus) {}

  bool begin(int32_t sda, int32_t scl, uint32_t freq) {
    if (sda < 0 || scl < 0) return false;
    wire_->begin(sda, scl, freq);
    wire_->beginTransmission(ES8311_ADDR);
    if (wire_->endTransmission() != 0) return false;

    bool ok = true;
    ok &= write(0x00, 0x1F);
    delay(20);
    ok &= write(0x00, 0x00);
    ok &= write(0x00, 0x80);
    ok &= write(0x01, 0x3F);
    ok &= write(0x06, read(0x06) & ~(1 << 5));
    ok &= configSampleRate48k();
    ok &= configBits16();
    ok &= write(0x0D, 0x01);
    ok &= write(0x0E, 0x02);
    ok &= write(0x12, 0x00);
    ok &= write(0x13, 0x10);
    ok &= write(0x1C, 0x6A);
    ok &= write(0x37, 0x08);
    return ok;
  }

  bool configBits16() { return write(0x09, 0x0C) && write(0x0A, 0x0C); }

  // Glosnosc 0..100% -> rejestr 0x32 (0..0xD7 ~ +12 dB; mniejsze ryzyko przesterowania niz 0xFF).
  bool setVolume(uint8_t percent) {
    if (percent > 100) percent = 100;
    uint8_t reg = (uint8_t)(((uint16_t)percent * 215u) / 100u);
    return write(0x32, reg);
  }

  // Pelne uspienie: wycisz DAC i wprowadz uklad w reset (bloki analogowe off).
  bool powerDown() {
    write(0x32, 0x00);
    return write(0x00, 0x1F);
  }

private:
  TwoWire *wire_;
  bool write(uint8_t reg, uint8_t value) {
    wire_->beginTransmission(ES8311_ADDR);
    wire_->write(reg);
    wire_->write(value);
    return wire_->endTransmission() == 0;
  }
  uint8_t read(uint8_t reg) {
    wire_->beginTransmission(ES8311_ADDR);
    wire_->write(reg);
    wire_->endTransmission(false);
    wire_->requestFrom((uint16_t)ES8311_ADDR, (uint8_t)1, true);
    return wire_->available() ? wire_->read() : 0;
  }
  bool configSampleRate48k() {
    bool ok = true;
    ok &= write(0x02, 0x00);
    ok &= write(0x03, 0x10);
    ok &= write(0x04, 0x10);
    ok &= write(0x05, 0x00);
    ok &= write(0x06, (read(0x06) & 0xE0) | 0x03);
    ok &= write(0x07, 0x00);
    ok &= write(0x08, 0xFF);
    return ok;
  }
};

static ES8311Codec codec;   // singleton kodeka (po definicji klasy)

// ============================================================================
//  Koordynator (forward) — kazdy manager trzyma App& jako mediator do rodzenstwa.
// ============================================================================
class App;

// ============================================================================
//  [3b] RtcManager — PCF85063A: czas (z wyrownaniem fazy), offset, alarm wybudz.
// ============================================================================
class RtcManager {
public:
  explicit RtcManager(App &app) : app_(app) {}

  bool readToSystem();
  bool readEpoch(time_t *epoch);                    // requireCurrentFormat = true
  bool writeRtc(time_t epoch, bool audioActive);    // audioActive: pomija wyrownanie fazy
  bool ensureOffsetCalibration();
  bool setWakeAlarm(time_t epoch);
  bool disableWakeAlarm();
#if ENABLE_RTC_AUTOTRIM
  void setTargetOffsetLsb(int8_t lsb) { targetOffsetReg_ = encodeRtcOffset(lsb, false); }
  bool applyOffsetLsb(int8_t lsb);                  // zapis+weryfikacja rejestru 0x02
  bool measureDriftUs(int64_t *driftUs);            // faza RTC vs czas systemowy (~ms)
#endif

  // Diagnostyka offsetu (do /info).
  uint8_t offsetRaw() const      { return offsetRaw_; }
  uint8_t targetOffsetRaw() const { return targetOffsetReg_; }
  int8_t  offsetLsb() const      { return offsetLsb_; }
  float   offsetPpm() const      { return offsetPpm_; }
  bool    offsetVerified() const { return offsetVerified_; }

private:
  App    &app_;
  uint8_t targetOffsetReg_ = RTC_OFFSET_VALUE;   // docelowa zawartosc 0x02 (NVS moze nadpisac)
  uint8_t offsetRaw_      = 0xFF;
  int8_t  offsetLsb_      = 0;
  float   offsetPpm_      = 0.0f;
  bool    offsetVerified_ = false;

  bool beginSession();
  bool writeReg(uint8_t reg, uint8_t value);
  bool readRegs(uint8_t reg, uint8_t *buf, uint8_t n);
  bool readControl1(uint8_t *control1);
  bool readControl2(uint8_t *control2);
  bool writeControl1(uint8_t control1);
  bool releaseClock(uint8_t runningControl1);
  bool prepareTimeWrite(uint8_t *runningControl1);
  bool hasCurrentFormat();
  bool storeCurrentFormat();
  bool invalidateCurrentFormat();
  bool readEpochInternal(time_t *epoch, bool requireCurrentFormat);
  bool configureWakeInterrupt(bool alarmEnabled);
  bool verifyWakeInterruptConfig(bool alarmEnabled);
  bool verifyWakeAlarm(time_t epoch);
};

// ============================================================================
//  FileStore — system plikow MP3 (SPIFFS/LittleFS/FFat), montowany leniwie.
// ============================================================================
class FileStore {
public:
  bool ensureMounted();
  fs::FS     *fs() const   { return fs_; }
  const char *name() const { return fsName_; }
  bool        mounted() const { return mounted_ && fs_; }

private:
  bool        mounted_     = false;
  fs::FS     *fs_          = nullptr;
  const char *fsName_      = "brak";
  bool        formatTried_ = false;
  bool recoverSoundFilesOnce();
  void recoverSoundFiles();
};

// ============================================================================
//  [7] BatteryManager — ADC, krzywa OCV->SoC, estymator, ochrona ogniwa
// ============================================================================
class BatteryManager {
public:
  explicit BatteryManager(App &app) : app_(app) {}

  bool init();
  void update(bool force);
  uint8_t percent() const    { return g_persist.batteryPercent; }
  float   voltage() const    { return g_persist.batteryVoltage; }
  bool    hasReading() const { return g_persist.batteryHasReading; }
#if ENABLE_LOW_BATTERY_PROTECT
  bool isLow() const;
  bool isCritical() const;
#endif

private:
  App &app_;
  adc_oneshot_unit_handle_t adc_  = nullptr;
  adc_cali_handle_t         cali_ = nullptr;
  bool     ready_     = false;
  bool     caliReady_ = false;
  uint32_t nextReadMs_ = 0;

  static uint8_t percentFromVoltage(float v);
  uint8_t stabilizePercent(uint8_t measured);
  void    maybeWriteDailySummary(time_t now);
};

// ============================================================================
//  [6a] AudioManager — odtwarzanie MP3 (ES8311 + I2S) oraz petla "dzwonienia"
// ============================================================================
class AudioManager {
public:
  explicit AudioManager(App &app) : app_(app) {}

  bool ensureCodec();
  void shutdownCodec();
  void codecPowerDown();        // ES8311 w stan niskiego poboru (setup)
  bool startSound(uint8_t soundIndex, uint8_t volume);
  void stopPlayback(bool userStopped);
  void startRinging(int alarmIndex);
  void serviceAudio();
  void serviceRinging();

  bool isPlaying() const    { return playing_; }
  bool isRinging() const    { return ringing_; }
  bool codecActive() const  { return ready_; }    // kodek nie uspiony (idlePowerSave)

private:
  App &app_;
  void ensurePins();
  bool     pinsReady_ = false;
  bool     ready_     = false;     // audioReady (kodek skonfigurowany)
  bool     playing_   = false;
  uint32_t soundStartedMs_ = 0;
  // stan dzwonienia budzika
  bool     ringing_         = false;
  int      ringingAlarm_    = -1;
  uint32_t ringStartMs_     = 0;
  uint32_t ringLastRetryMs_ = 0;
  uint8_t  ringErrors_      = 0;
};

// ============================================================================
//  [10a] AlarmManager — model budzikow, harmonogram wyzwolen, dedup kolizji DST
// ============================================================================
class AlarmManager {
public:
  explicit AlarmManager(App &app) : app_(app) {}

  void setDefaults();
  void resetTriggerState();
  void sanitize(int index);
  void sanitizeAll();
  AlarmConfig       &config(int i)       { return alarms_[i]; }
  const AlarmConfig &config(int i) const { return alarms_[i]; }
  AlarmConfig       *raw()               { return alarms_; }     // NVS getBytes/putBytes
  size_t             rawSize() const     { return sizeof(alarms_); }

  bool anyEnabled() const;
  int  nextAlarmIndex(uint8_t *h, uint8_t *mi, uint8_t *s, char *label, size_t labelSize);
  void checkAlarms(uint32_t lateWindowSec = ALARM_LATE_WINDOW_SEC);

  // Uzywane przez PowerManager przy planowaniu snu.
  bool   anyDueNow(const struct tm &t);
  time_t nextEpoch(const struct tm &base, time_t now);

private:
  App        &app_;
  AlarmConfig alarms_[ALARM_COUNT];
};

// ============================================================================
//  [5] NtpService — WiFi + NTP, nieblokujaca maszyna stanow + harmonogram dobowy
// ============================================================================
enum class NtpSyncState : uint8_t { IDLE, WIFI_CONNECTING, WAITING_PACKET };

class NtpService {
public:
  explicit NtpService(App &app) : app_(app) {}

  void syncNtp(bool manual);
  bool service();                       // true gdy NTP wlasnie zatwierdzil nowy czas
  void checkSchedule();                 // dobowa synchronizacja 18:00 (+ dogonienie)
  void handleRtcWriteRetry();
  void stopWifiAndNtp();
  bool ensureWifi(uint32_t timeoutMs);  // blokujace laczenie (tylko panel WWW)

  void setRetries(uint8_t n)   { retriesLeft_ = n; }
  bool isBusy() const          { return state_ != NtpSyncState::IDLE; }
  bool rtcWritePending() const { return rtcWriteRetriesLeft_ != 0; }
  long secsSinceLastSync() const;

  const char *ssid() const { return ssid_; }
  const char *pass() const { return pass_; }   // tylko do zapisu NVS (NIE renderowane w HTML)
  void setCreds(const char *s, const char *p);
  void setSsid(const String &s);
  void setPass(const String &p);

private:
  App         &app_;
  NtpSyncState state_         = NtpSyncState::IDLE;
  bool         manualAttempt_ = false;
  bool         wifiHintActive_ = false;
  uint32_t     stateStartedMs_ = 0;
  struct timeval fallbackTime_ = {};    // snapshot czasu przed proba (rollback po bledzie)
  uint8_t      retriesLeft_ = 0;
  uint32_t     nextRetryMs_ = 0;
  uint32_t     backoffMs_   = NTP_BACKOFF_MIN_MS;
  uint32_t     wifiLostMs_  = 0;        // od kiedy WiFi zerwane w trakcie WAITING_PACKET
  uint8_t      rtcWriteRetriesLeft_ = 0;
  uint32_t     rtcWriteNextRetryMs_ = 0;
  char         ssid_[33] = "";
  char         pass_[65] = "";

  bool wifiBeginSmart();
  void wifiRetryFullScan();
  void wifiCacheHint();
  void stopSntpCallbacks();
  void restoreTimeAfterFailedNtp();
  void beginNtpRequest();
  void finishNtpAttempt(bool ok);
  bool handleNtpRetry();
  void scheduleRtcWriteRetry(uint8_t retries = RTC_WRITE_RETRIES);
#if ENABLE_RTC_AUTOTRIM
  void maybeAutotrimRtc(time_t syncedEpoch);
#endif
};

// ============================================================================
//  [8] DisplayManager (U8g2) — strony, ikony, dedup repaintu (clockSig)
// ============================================================================
class DisplayManager {
public:
  explicit DisplayManager(App &app) : app_(app) {}

  void setup(bool coldBoot);
  void update();                    // dispatcher updateDisplay
  void drawNightPage();             // wolane przez PowerManager przed deep-sleep
#if ENABLE_LOW_BATTERY_PROTECT
  void drawCriticalBatteryPage();   // jw. przed ochronnym snem
#endif

private:
  App     &app_;
  int      lastDrawMinute_ = -1;
  int      lastDrawDay_    = -1;
  uint32_t lastClockSig_   = 0xFFFFFFFFu;   // sentinel = wymus repaint

  void     st7305SetStaticLowPower(bool enable);
  void     drawPhotoGlyph(int x, int y, int glyph, bool colon);
  void     drawLargeTime(uint8_t hour, uint8_t minute, int y);
  void     drawBellIcon(int x, int y, bool active);
  void     drawBatteryIcon(int x, int y, uint8_t percent);
  void     drawCentered(const char *text, int y);
  uint32_t clockSig(const struct tm &t, int day);
  void     drawClockPage(const struct tm &t);
  void     drawPreviewPage();
  void     drawRingingPage();
};

// ============================================================================
//  [9] WebManager — panel konfiguracji, sync, pliki MP3, /info (WiFi na zadanie)
// ============================================================================
class WebManager {
public:
  explicit WebManager(App &app) : app_(app) {}

  bool startWebWindow();
  void service();              // obsluga w loop(): grace WiFi + handleClient + timeout
  bool isActive() const { return active_; }
  void forceStop();           // server.stop + active_=false (przy zasypianiu nocnym)

private:
  App     &app_;
  bool     active_         = false;
  bool     routesInstalled_ = false;
  uint32_t untilMs_        = 0;
  uint32_t wifiLostMs_     = 0;

  File     mp3UploadFile_;
  bool     mp3UploadAccepted_ = false;
  bool     mp3UploadHadData_  = false;
  String   mp3UploadPath_     = "";
  String   mp3UploadTempPath_ = "";

  void stopWindow();          // stopWebWindow
  void installRoutes();
  void redirectHome();        // 303 -> "/" (wspolne zakonczenie akcji POST/GET)
  void appendMp3Manager(String &html);
  void pageHeader(String &h);
  void handleRoot();
  void handleSave();
  void handleSync();
  void handleStop();
  void handleTest();
  void handleMp3UploadDone();
  void handleMp3UploadStream();
  void handleMp3Delete();
  void handleWebOff();
#if ENABLE_DIAGNOSTICS
  void handleInfo();
#endif
};

// ============================================================================
//  [11] PowerManager — tryb nocny (deep-sleep), light-sleep, ochronny sen
// ============================================================================
class PowerManager {
public:
  explicit PowerManager(App &app) : app_(app) {}

  void releaseNightHolds();
  void enterNightSleepIfNeeded(bool force);
  void idlePowerSave();
#if ENABLE_LOW_BATTERY_PROTECT
  void enterCriticalBatterySleepIfNeeded();
#endif

private:
  App     &app_;
  void     holdNightPins();
  void     shutdownNightPeripherals();
  time_t   nightEndEpoch(time_t now);
  uint64_t computeIdleSleepUs();
};

// ============================================================================
//  Przyciski (KEY = wycisz/panel WWW, PREVIEW = podglad budzika)
// ============================================================================
class Buttons {
public:
  explicit Buttons(App &app) : app_(app) {}

  void begin();                  // pinMode KEY/PREVIEW + stan poczatkowy
  void handleKey();
  void handleBoot();
  void showPreview();
  void primeKeyPress();          // setup: zacznij mierzyc nacisniecie KEY (bez wakeHold)
  void primeKeyPressFromWake();  // wybudzenie z light-sleep: jw. + utrzymaj wybudzenie
  void onPreviewWakeFromSleep(); // wybudzenie z light-sleep przyciskiem PREVIEW

  void holdWake()           { wakeHoldUntilMs_ = millis() + WAKE_HOLD_MS; }
  bool wakeHeld() const     { return (int32_t)(millis() - wakeHoldUntilMs_) < 0; }
  bool previewActive() const       { return previewActive_; }
  bool previewWithinWindow() const { return (int32_t)(millis() - previewUntilMs_) < 0; }
  void clearPreview()              { previewActive_ = false; }

private:
  App     &app_;
  bool     keyRaw_ = HIGH, keyStable_ = HIGH;
  uint32_t keyChangedMs_ = 0, keyPressMs_ = 0;
  bool     keyLongFired_ = false;
  bool     bootRaw_ = HIGH, bootStable_ = HIGH;
  uint32_t bootChangedMs_ = 0, bootPressedMs_ = 0;
  bool     previewActive_  = false;
  uint32_t previewUntilMs_ = 0;
  uint32_t wakeHoldUntilMs_ = 0;
};

// ============================================================================
//  [4] Settings — NVS (alarmy + WiFi) oraz diagnostyka resetu
// ============================================================================
class Settings {
public:
  explicit Settings(App &app) : app_(app) {}

  void load();
  void save();
#if ENABLE_RTC_AUTOTRIM
  void saveRtcOffset(int8_t lsb);   // tylko klucz "rtcOffLsb" (bez przepisywania reszty)
#endif
#if ENABLE_DIAGNOSTICS
  void     recordResetReason(esp_reset_reason_t r);
  uint32_t brownoutResets() const { return brownoutResets_; }
  uint32_t wdtResets() const      { return wdtResets_; }
  uint32_t panicResets() const    { return panicResets_; }
#endif

private:
  App &app_;
#if ENABLE_DIAGNOSTICS
  uint32_t brownoutResets_ = 0;
  uint32_t wdtResets_      = 0;
  uint32_t panicResets_    = 0;
#endif
};

// ============================================================================
//  App — koordynator. Trzyma wszystkie managery i stan UI; udostepnia rodzenstwo
//  jako mediator (kazdy manager dostaje tylko App&). Agreguje wspoldzielona
//  bramke "zajetosci" (dawniej powielana 3x w sleepach).
// ============================================================================
class App {
public:
  App()
    : rtc_(*this), battery_(*this), audio_(*this), alarm_(*this),
      ntp_(*this), display_(*this), web_(*this), power_(*this),
      buttons_(*this), settings_(*this) {}

  void setup();
  void loop();

  // --- mediator: dostep do rodzenstwa ---
  FileStore       &fs()       { return fs_; }
  RtcManager      &rtc()      { return rtc_; }
  BatteryManager  &battery()  { return battery_; }
  AudioManager    &audio()    { return audio_; }
  AlarmManager    &alarm()    { return alarm_; }
  NtpService      &ntp()      { return ntp_; }
  DisplayManager  &display()  { return display_; }
  WebManager      &web()      { return web_; }
  PowerManager    &power()    { return power_; }
  Buttons         &buttons()  { return buttons_; }
  Settings        &settings() { return settings_; }
  PersistentState &persist()  { return g_persist; }

  // --- stan UI (dawne globale displayDirty / statusText / ntpStatus) ---
  void markDirty()                   { dirty_ = true; }
  void clearDirty()                  { dirty_ = false; }
  bool dirty() const                 { return dirty_; }
  void setStatus(const String &s)    { statusText_ = s; }
  const String &statusText() const   { return statusText_; }
  void setNtpStatus(const String &s) { ntpStatus_ = s; }
  const String &ntpStatus() const    { return ntpStatus_; }

  // --- bramki wspoldzielone ---
  bool audioActive() const { return audio_.isPlaying() || audio_.isRinging(); }
  // Stan, w ktorym NIE wolno uspic na dlugo (suma warunkow z 3 dawnych miejsc).
  bool busy() const {
    return audioActive() || web_.isActive() || buttons_.previewActive() ||
           ntp_.isBusy() || ntp_.rtcWritePending();
  }

  uint32_t &lastAlarmCheckMs() { return lastAlarmCheckMs_; }

private:
  FileStore      fs_;
  RtcManager     rtc_;
  BatteryManager battery_;
  AudioManager   audio_;
  AlarmManager   alarm_;
  NtpService     ntp_;
  DisplayManager display_;
  WebManager     web_;
  PowerManager   power_;
  Buttons        buttons_;
  Settings       settings_;

  bool     dirty_      = true;
  String   statusText_ = "IDLE";
  String   ntpStatus_  = "never";
  uint32_t lastAlarmCheckMs_ = 0;   // loop(): poszerza okno spoznienia o przestoj petli
};

App app;   // jedyna instancja koordynatora

// ============================================================================
//  DEFINICJE METOD — RtcManager (PCF85063A). Logika 1:1 z wersji proceduralnej.
// ============================================================================
// Odblokowanie zawieszonej szyny I2C: slave (RTC/kodek) trzymajacy SDA nisko po
// resecie ESP w polowie transakcji zwalnia ja po maks. 9 impulsach SCL + STOP.
// Bez tego kazda sesja RTC pada az do odlaczenia zasilania. Zwraca true, gdy
// szyna byla zawieszona i zostala uwolniona.
static bool i2cBusRecover() {
  // Sonda BEZ pinMode: rekonfiguracja pinu I2C w core 3.x przechodzi przez
  // Peripheral Manager i deinicjalizuje caly sterownik Wire (wspoldzielony z
  // ES8311) — nawet przy zwyklym NACK. gpio_get_level czyta stan szyny nie
  // dotykajac konfiguracji (bufor wejsciowy SDA wlaczony przez sterownik I2C).
  if (gpio_get_level((gpio_num_t)I2C_SDA_PIN) == 1) return false;   // SDA wolne -> zwykly NACK
  Wire.end();                                           // jawny demontaz PRZED przejeciem pinow
  pinMode(I2C_SDA_PIN, INPUT_PULLUP);
  pinMode(I2C_SCL_PIN, OUTPUT_OPEN_DRAIN);
  digitalWrite(I2C_SCL_PIN, HIGH);
  for (uint8_t i = 0; i < 9 && digitalRead(I2C_SDA_PIN) == LOW; i++) {
    digitalWrite(I2C_SCL_PIN, LOW);  delayMicroseconds(50);
    digitalWrite(I2C_SCL_PIN, HIGH); delayMicroseconds(50);
  }
  pinMode(I2C_SDA_PIN, OUTPUT_OPEN_DRAIN);              // warunek STOP: SDA lo->hi przy SCL hi
  digitalWrite(I2C_SDA_PIN, LOW);  delayMicroseconds(50);
  digitalWrite(I2C_SDA_PIN, HIGH); delayMicroseconds(50);
  bool freed = digitalRead(I2C_SDA_PIN) == HIGH;
  Serial.printf("I2C: szyna zawieszona -> odzysk %s\n", freed ? "OK" : "NIEUDANY");
  return freed;
}

bool RtcManager::beginSession() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);
  Wire.beginTransmission(PCF85063A_ADDR);
  if (Wire.endTransmission() == 0) return true;
  if (!i2cBusRecover()) return false;                   // NACK bez zwisu SDA -> zwykly blad
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);                 // po odzysku pelna re-inicjalizacja
  Wire.setClock(400000);
  Wire.beginTransmission(PCF85063A_ADDR);
  return Wire.endTransmission() == 0;
}

bool RtcManager::writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(PCF85063A_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

// Odczyt n rejestrow od 'reg' jednym transferem. Wolajacy juz otworzyl sesje.
bool RtcManager::readRegs(uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(PCF85063A_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)PCF85063A_ADDR, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

bool RtcManager::readControl1(uint8_t *control1) {
  if (!control1 || !beginSession()) return false;
  return readRegs(0x00, control1, 1);
}
bool RtcManager::readControl2(uint8_t *control2) {
  if (!control2 || !beginSession()) return false;
  return readRegs(0x01, control2, 1);
}
bool RtcManager::writeControl1(uint8_t control1) { return writeReg(0x00, control1); }

bool RtcManager::releaseClock(uint8_t runningControl1) {
  for (uint8_t attempt = 0; attempt < 3; attempt++) {
    if (writeControl1(runningControl1)) return true;
    delay(2);
  }
  return false;
}

// Zatrzymaj licznik przed ustawieniem czasu. STOP zeruje glowna czesc preskalera
// -> pierwszy impuls sekundowy ma okreslona faze.
bool RtcManager::prepareTimeWrite(uint8_t *runningControl1) {
  if (!runningControl1) return false;
  uint8_t control1;
  if (!readControl1(&control1)) return false;
  *runningControl1 = control1 & (uint8_t)~((1U << 7) | (1U << 5) | (1U << 2) | (1U << 1));
  if (writeControl1(*runningControl1 | (1U << 5))) return true;
  releaseClock(*runningControl1);
  return false;
}

bool RtcManager::hasCurrentFormat() {
  uint8_t marker;
  return readRegs(0x03, &marker, 1) && marker == RTC_FORMAT_MARKER;
}
bool RtcManager::storeCurrentFormat()      { return writeReg(0x03, RTC_FORMAT_MARKER); }
bool RtcManager::invalidateCurrentFormat() { return writeReg(0x03, 0); }

// Odczyt 7 rejestrow czasu (0x04..0x0A) jednym transferem -> brak rwania na granicy
// sekundy. Rok 00..99 = 2000..2099.
bool RtcManager::readEpochInternal(time_t *epoch, bool requireCurrentFormat) {
  if (!epoch) return false;
  uint8_t control1;
  if (!readControl1(&control1)) return false;

  // Nie wznawiaj po cichu zatrzymanego/przestawionego RTC (staly czas != aktualny).
  const uint8_t invalidModeMask = (1U << 7) | (1U << 5) | (1U << 1);
  if ((control1 & invalidModeMask) != 0 ||
      (requireCurrentFormat && !hasCurrentFormat())) return false;

  uint8_t t[7];
  if (!readRegs(0x04, t, 7)) return false;
  uint8_t secReg = t[0], minReg = t[1], hourReg = t[2], dayReg = t[3];
  uint8_t monReg = t[5], yearReg = t[6];   // t[4] = dzien tygodnia (pomijamy)

  bool oscStopped = (secReg & 0x80) != 0;
  secReg &= 0x7F; minReg &= 0x7F; hourReg &= 0x3F; dayReg &= 0x3F; monReg &= 0x1F;

  if (oscStopped || !validBcd(secReg) || !validBcd(minReg) || !validBcd(hourReg) ||
      !validBcd(dayReg) || !validBcd(monReg) || !validBcd(yearReg)) return false;

  int s  = bcd2dec(secReg), mi = bcd2dec(minReg), h = bcd2dec(hourReg);
  int d  = bcd2dec(dayReg), mo = bcd2dec(monReg), y = bcd2dec(yearReg) + 2000;

  if (!validDateTime(y, mo, d, h, mi, s)) return false;
  *epoch = utcEpochFromCivil(y, mo, d, h, mi, s);
  return true;
}

bool RtcManager::readEpoch(time_t *epoch) { return readEpochInternal(epoch, true); }

bool RtcManager::readToSystem() {
  for (uint8_t attempt = 0; attempt < 3; attempt++) {
    time_t epoch;
    if (readEpoch(&epoch)) { setSystemEpochMidSecond(epoch); return true; }
    delay(2);
  }
  return false;
}

// Zapis czasu UTC do RTC z WYROWNANIEM do granicy sekundy. Po zwolnieniu STOP
// preskaler startuje od zera, wiec wpisana sekunda "trzyma sie" ~1.0 s -> faza
// przewrotu = chwila zwolnienia STOP. Pomijamy wyrownanie gdy gra audio/budzik
// (audioActive) — busy-wait do ~1 s przerwalby audio.loop().
bool RtcManager::writeRtc(time_t epoch, bool audioActive) {
  if (!isSupportedEpoch(epoch)) return false;
#if ENABLE_RTC_AUTOTRIM
  g_persist.rtcPhaseAligned = false;   // az do potwierdzonego, wyrownanego zapisu
#endif

  struct timeval tv;
  gettimeofday(&tv, nullptr);
  time_t writeEpoch = epoch;
  bool   alignToBoundary = false;
  if (!audioActive &&
      isSupportedEpoch(tv.tv_sec) && (tv.tv_sec == epoch || tv.tv_sec + 1 == epoch)) {
    writeEpoch = tv.tv_sec + 1;          // najblizsza nadchodzaca pelna sekunda
    alignToBoundary = true;
  }

  struct tm gm;
  gmtime_r(&writeEpoch, &gm);
  int year = gm.tm_year + 1900;

  // Najpierw uniewaznij dane: reset w dowolnym dalszym miejscu nie ujawni
  // niepelnego zapisu jako poprawnego czasu.
  if (!beginSession()) return false;
  if (!invalidateCurrentFormat()) return false;
  uint8_t runningControl1;
  if (!prepareTimeWrite(&runningControl1)) return false;
  uint8_t yearReg = (uint8_t)(year % 100);

  Wire.beginTransmission(PCF85063A_ADDR);
  Wire.write(0x04);
  Wire.write(dec2bcd(gm.tm_sec) & 0x7F);            // OS=0 -> oscylator OK
  Wire.write(dec2bcd(gm.tm_min) & 0x7F);
  Wire.write(dec2bcd(gm.tm_hour) & 0x3F);
  Wire.write(dec2bcd(gm.tm_mday) & 0x3F);
  Wire.write((uint8_t)(gm.tm_wday & 0x07));
  Wire.write(dec2bcd((uint8_t)(gm.tm_mon + 1)) & 0x1F);
  Wire.write(dec2bcd(yearReg));
  if (Wire.endTransmission() != 0) { releaseClock(runningControl1); return false; }

  // Odczekaj do granicy docelowej sekundy (czas I2C wliczony), potem zwolnij STOP.
  if (alignToBoundary) {
    struct timeval nowTv;
    for (;;) {
      gettimeofday(&nowTv, nullptr);
      long remainingUs = (long)(writeEpoch - nowTv.tv_sec) * 1000000L - nowTv.tv_usec;
      if (remainingUs <= 0) break;
      if (remainingUs > 2000) delay(1); else delayMicroseconds((uint32_t)remainingUs);
    }
  }
  if (!releaseClock(runningControl1)) return false;

  time_t verifiedEpoch;
  if (!readEpochInternal(&verifiedEpoch, false)) return false;
  long long diff = (long long)verifiedEpoch - (long long)writeEpoch;
  if (diff < -2 || diff > 2) return false;
  if (storeCurrentFormat() && hasCurrentFormat()) {
#if ENABLE_RTC_AUTOTRIM
    g_persist.rtcPhaseAligned = alignToBoundary;   // okno dryftu mierzalne fazowo
#endif
    return true;
  }
  invalidateCurrentFormat();
  return false;
}

// Kalibracja sprzetowa dryfu (rejestr 0x02) Z WERYFIKACJA odczytem zwrotnym.
// Wolana przy KAZDYM starcie: rejestr jest podtrzymywany bateryjnie, ale ponowny
// zapis+odczyt dowodzi, ze bajt wszedl. Cel zapisu = targetOffsetReg_ (start:
// RTC_OFFSET_VALUE; Settings::load moze nadpisac wartoscia z NVS).
bool RtcManager::ensureOffsetCalibration() {
  offsetVerified_ = false;
  for (uint8_t attempt = 0; attempt < RTC_OFFSET_RETRIES; attempt++) {
    if (!beginSession()) { delay(2); continue; }
    writeReg(RTC_OFFSET_REG, targetOffsetReg_);
    uint8_t raw;
    if (readRegs(RTC_OFFSET_REG, &raw, 1)) {
      offsetRaw_ = raw;
      offsetLsb_ = decodeRtcOffset(raw);
      offsetPpm_ = rtcOffsetPpm(offsetLsb_, (raw & 0x80) != 0);
      if (raw == targetOffsetReg_) { offsetVerified_ = true; return true; }
    }
    delay(2);
  }
  return false;
}

#if ENABLE_RTC_AUTOTRIM
bool RtcManager::applyOffsetLsb(int8_t lsb) {
  setTargetOffsetLsb(lsb);
  return ensureOffsetCalibration();
}

// Pomiar dryftu z rozdzielczoscia ~ms zamiast 1 s: czekaj (polling I2C) na
// przewrot sekundy RTC i zlap te chwile wg zegara systemowego. Wolane ZARAZ po
// pakiecie NTP (czas systemowy = wzorzec) i PRZED writeRtc (nadpisze faze).
// W przewrocie faza RTC = dokladnie 0.000 s, wiec epokaRTC*1e6 - czas_sys_us
// = dryft narosly od ostatniego wyrownanego zapisu. Blokuje do ~1.3 s — wolac
// tylko bez aktywnego audio (busy-wait zaglodzilby audio.loop()).
bool RtcManager::measureDriftUs(int64_t *driftUs) {
  if (!driftUs || !beginSession()) return false;
  uint8_t sec0, sec;
  if (!readRegs(0x04, &sec0, 1)) return false;
  sec0 &= 0x7F;
  uint32_t startMs = millis();
  do {
    if (millis() - startMs > RTC_BOUNDARY_POLL_TIMEOUT_MS) return false;   // brak przewrotu: oscylator stoi
    if (!readRegs(0x04, &sec, 1)) return false;
    sec &= 0x7F;
  } while (sec == sec0);
  struct timeval tv;
  gettimeofday(&tv, nullptr);              // chwila przewrotu wg czasu systemowego
  watchdogFeed();
  time_t rtcEpoch;
  if (!readEpochInternal(&rtcEpoch, true)) return false;
  *driftUs = (int64_t)rtcEpoch * 1000000LL -
             ((int64_t)tv.tv_sec * 1000000LL + (int64_t)tv.tv_usec);
  return true;
}
#endif

// INT sluzy wylacznie alarmowi wybudzajacemu. Wylacz korekcje, MI/HMI i countdown
// timer, skasuj flagi AF/TF, wylacz CLKOUT (mniejszy pobor RTC).
bool RtcManager::configureWakeInterrupt(bool alarmEnabled) {
  uint8_t control1;
  if (!readControl1(&control1)) return false;
  control1 &= (uint8_t)~(1U << 2);                   // CIE=0
  if (!writeReg(0x00, control1)) return false;
  if (!writeReg(0x11, 0x18)) return false;           // Timer_mode: TCF=1/60 Hz, TE=0, TIE=0
  if (!writeReg(0x10, 0)) return false;              // Timer_value
  uint8_t control2;
  if (!readControl2(&control2)) return false;
  control2 = (uint8_t)(0x07 | (alarmEnabled ? (1U << 7) : 0));   // COF=111 (CLKOUT off), AIE
  return writeReg(0x01, control2);
}

bool RtcManager::verifyWakeInterruptConfig(bool alarmEnabled) {
  uint8_t control1, control2;
  if (!readControl1(&control1) || !readControl2(&control2)) return false;
  if (!beginSession()) return false;
  uint8_t timer[2];
  if (!readRegs(0x10, timer, 2)) return false;
  uint8_t expectedControl2 = alarmEnabled ? (1U << 7) : 0;
  return (control1 & (1U << 2)) == 0 &&
         (control2 & 0xF8) == expectedControl2 &&
         timer[0] == 0 && timer[1] == 0x18;
}

bool RtcManager::verifyWakeAlarm(time_t epoch) {
  struct tm gm;
  gmtime_r(&epoch, &gm);
  if (!beginSession()) return false;
  uint8_t a[5];
  if (!readRegs(0x0B, a, 5)) return false;
  if (a[0] != (dec2bcd((uint8_t)gm.tm_sec) & 0x7F) ||
      a[1] != (dec2bcd((uint8_t)gm.tm_min) & 0x7F) ||
      a[2] != (dec2bcd((uint8_t)gm.tm_hour) & 0x3F) ||
      a[3] != (dec2bcd((uint8_t)gm.tm_mday) & 0x3F) ||
      a[4] != 0x80) return false;
  return verifyWakeInterruptConfig(true);
}

// Alarm PCF85063A pracuje w UTC (jak rejestry czasu). INT aktywne stanem niskim.
bool RtcManager::setWakeAlarm(time_t epoch) {
  time_t systemNow = time(nullptr);
  if (!isSupportedEpoch(epoch) || !isSupportedEpoch(systemNow)) return false;
  time_t rtcNow;
  if (!readEpoch(&rtcNow)) return false;     // odrzuca STOP/12h
  long long rtcSkew = (long long)rtcNow - (long long)systemNow;
  if (rtcSkew < -RTC_WAKE_MAX_SKEW_SEC || rtcSkew > RTC_WAKE_MAX_SKEW_SEC) return false;
  if (!configureWakeInterrupt(false)) return false;

  struct tm gm;
  gmtime_r(&epoch, &gm);
  if (!beginSession()) return false;
  Wire.beginTransmission(PCF85063A_ADDR);
  Wire.write(0x0B);                               // Second_alarm..
  Wire.write(dec2bcd((uint8_t)gm.tm_sec) & 0x7F); // AEN=0 -> aktywny
  Wire.write(dec2bcd((uint8_t)gm.tm_min) & 0x7F);
  Wire.write(dec2bcd((uint8_t)gm.tm_hour) & 0x3F);
  Wire.write(dec2bcd((uint8_t)gm.tm_mday) & 0x3F);
  Wire.write(0x80);                               // Weekday_alarm AEN=1 (wylaczony)
  if (Wire.endTransmission() != 0) return false;

  if (!configureWakeInterrupt(true)) return false;
  return verifyWakeAlarm(epoch);
}

bool RtcManager::disableWakeAlarm() {
  return configureWakeInterrupt(false) && verifyWakeInterruptConfig(false);
}

// ============================================================================
//  DEFINICJE METOD — FileStore (system plikow MP3)
// ============================================================================
bool FileStore::recoverSoundFilesOnce() {
  File root = fs_->open("/");
  if (!root) return false;
  for (File f = root.openNextFile(); f; f = root.openNextFile()) {
    bool isDir = f.isDirectory();
    String path = f.name();
    f.close();
    if (isDir) continue;
    if (!path.startsWith("/")) path = String("/") + path;
    String lower = path; lower.toLowerCase();
    if (lower.endsWith(".bak") && path.length() > 5) {
      String target = path.substring(0, path.length() - 4);
      bool changed = fs_->exists(target) ? fs_->remove(path) : fs_->rename(path, target);
      if (changed) { root.close(); return true; }
    } else if (lower.endsWith(".part") && path.length() > 6) {
      String target = path.substring(0, path.length() - 5);
      String backup = target + ".bak";
      if (fs_->exists(backup) && !fs_->exists(target)) continue;
      if (fs_->remove(path)) { root.close(); return true; }
    }
  }
  root.close();
  return false;
}

void FileStore::recoverSoundFiles() {
  if (!fs_) return;
  for (uint8_t pass = 0; pass < 16 && recoverSoundFilesOnce(); pass++) {}
}

bool FileStore::ensureMounted() {
  if (mounted_ && fs_) return true;
  struct { fs::FS *fs; const char *name; bool ok; } cands[] = {
    { &SPIFFS,   "SPIFFS",   SPIFFS.begin(false) },
    { &LittleFS, "LittleFS", false },
    { &FFat,     "FFat",     false },
  };
  if (!cands[0].ok) cands[1].ok = LittleFS.begin(false);
  if (!cands[0].ok && !cands[1].ok) cands[2].ok = FFat.begin(false);
  for (auto &c : cands) {
    if (c.ok) {
      mounted_ = true; fs_ = c.fs; fsName_ = c.name;
      recoverSoundFiles();
      Serial.printf("System plikow: %s\n", c.name);
      return true;
    }
  }
  // Fabrycznie czysty flash: jednorazowy format SPIFFS, by panel WWW mogl wgrac MP3.
  if (!formatTried_) {
    formatTried_ = true;
    Serial.println("Brak FS - formatuje SPIFFS...");
    if (SPIFFS.begin(true)) {
      mounted_ = true; fs_ = &SPIFFS; fsName_ = "SPIFFS";
      Serial.println("System plikow: SPIFFS (nowo sformatowany)");
      return true;
    }
  }
  mounted_ = false; fs_ = nullptr; fsName_ = "brak";
  Serial.println("Brak systemu plikow z MP3");
  return false;
}

// ============================================================================
//  DEFINICJE METOD — BatteryManager
// ============================================================================
bool BatteryManager::init() {
  if (ready_) return true;
  adc_oneshot_unit_init_cfg_t initCfg = {};
  initCfg.unit_id = ADC_UNIT_1;
  if (adc_oneshot_new_unit(&initCfg, &adc_) != ESP_OK) return false;

  adc_oneshot_chan_cfg_t chanCfg = {};
  chanCfg.bitwidth = ADC_BITWIDTH_12;
  chanCfg.atten = ADC_ATTEN_DB_12;
  if (adc_oneshot_config_channel(adc_, BATTERY_ADC_CHANNEL, &chanCfg) != ESP_OK) {
    adc_oneshot_del_unit(adc_); adc_ = nullptr; return false;
  }
  adc_cali_curve_fitting_config_t caliCfg = {};
  caliCfg.unit_id = ADC_UNIT_1;
  caliCfg.atten = ADC_ATTEN_DB_12;
  caliCfg.bitwidth = ADC_BITWIDTH_12;
  caliReady_ = adc_cali_create_scheme_curve_fitting(&caliCfg, &cali_) == ESP_OK;
  ready_ = true;
  return true;
}

// Krzywa OCV->SoC dla INR18650-35E (Li-ion NMC), 3.00 V=0%, 4.20 V=100%.
uint8_t BatteryManager::percentFromVoltage(float v) {
  static const struct { float v; uint8_t soc; } OCV[] = {
    { 3.00f,   0 }, { 3.10f,   2 }, { 3.20f,   4 }, { 3.30f,   7 },
    { 3.40f,  11 }, { 3.45f,  14 }, { 3.50f,  18 }, { 3.55f,  23 },
    { 3.60f,  29 }, { 3.65f,  36 }, { 3.70f,  43 }, { 3.73f,  47 },
    { 3.76f,  51 }, { 3.79f,  55 }, { 3.82f,  59 }, { 3.85f,  63 },
    { 3.88f,  68 }, { 3.91f,  71 }, { 3.95f,  76 }, { 4.00f,  80 },
    { 4.05f,  87 }, { 4.10f,  93 }, { 4.15f,  97 }, { 4.20f, 100 },
  };
  const uint8_t N = sizeof(OCV) / sizeof(OCV[0]);
  if (v <= OCV[0].v)   return 0;
  if (v >= OCV[N-1].v) return 100;
  for (uint8_t i = 1; i < N; i++) {
    if (v < OCV[i].v) {
      float frac = (v - OCV[i-1].v) / (OCV[i].v - OCV[i-1].v);
      return (uint8_t)(OCV[i-1].soc + frac * (OCV[i].soc - OCV[i-1].soc) + 0.5f);
    }
  }
  return 100;
}

uint8_t BatteryManager::stabilizePercent(uint8_t measured) {
  if (measured >= BATTERY_FULL_FLOOR) return 100;
  if (g_persist.batteryHasReading && g_persist.batteryPercent == 100 &&
      measured >= BATTERY_FULL_HOLD_FLOOR) return 100;
  return measured;
}

// Zwiezle podsumowanie zuzycia do NVS max raz/dobe (nadpisywane, malo flash).
void BatteryManager::maybeWriteDailySummary(time_t now) {
  struct tm t; localtime_r(&now, &t);
  int day = localDayKey(t);
  if (day == g_persist.lastSummaryDay) return;
  float perDay = drainMahPerDay(g_persist.drain);
  if (perDay < 0.0f) return;
  g_persist.lastSummaryDay = day;              // oznacz dzien NIM sprobujesz zapisu
  if (!prefs.begin(NVS_NS, false)) return;
  prefs.putUShort("drnSoc", g_persist.batteryPercent);
  prefs.putFloat("drnMahDay", perDay);
  prefs.putUInt("drnDay", (uint32_t)day);
  prefs.end();
  Serial.printf("Zuzycie: %.1f mAh/dzien, ~%.0f dni\n",
                perDay, drainDaysRemaining(g_persist.drain, g_persist.batteryPercent));
}

void BatteryManager::update(bool force) {
  if (!force && (int32_t)(millis() - nextReadMs_) < 0) return;

  // Pod obciazeniem (audio, dzwoniacy budzik, serwer web) napiecie zacisku siada
  // przez rezystancje wewnetrzna ogniwa -> pomiar zanizony. Odlozyc do bezczynnosci.
  if (!force && (app_.audioActive() || app_.web().isActive())) {
    nextReadMs_ = millis() + BATTERY_BUSY_RETRY_MS; return;
  }

  nextReadMs_ = millis() + BATTERY_READ_INTERVAL_MS;
  if (!init()) { nextReadMs_ = millis() + BATTERY_BUSY_RETRY_MS; return; }

  uint32_t sum = 0; uint8_t samples = 0;
  for (uint8_t i = 0; i < BATTERY_SAMPLE_COUNT; i++) {
    int raw = 0;
    if (adc_oneshot_read(adc_, BATTERY_ADC_CHANNEL, &raw) == ESP_OK) { sum += (uint32_t)raw; samples++; }
  }
  if (samples == 0) { nextReadMs_ = millis() + BATTERY_BUSY_RETRY_MS; return; }

  int raw = (int)((sum + samples / 2) / samples);
  int mv = 0;
  if (caliReady_) adc_cali_raw_to_voltage(cali_, raw, &mv);
  else mv = (raw * 3300) / 4095;

  float measured = (mv * 0.001f) * BATTERY_DIVIDER_RATIO * BATTERY_VOLTAGE_CALIBRATION;
  if (!g_persist.batteryHasReading || force) { g_persist.batteryVoltage = measured; g_persist.batteryHasReading = true; }
  else g_persist.batteryVoltage = g_persist.batteryVoltage * 0.80f + measured * 0.20f;   // EMA

  uint8_t old = g_persist.batteryPercent;
  g_persist.batteryPercent = stabilizePercent(percentFromVoltage(g_persist.batteryVoltage));
  time_t nowT = time(nullptr);
  if (isSupportedEpoch(nowT)) {
    drainOnSample(g_persist.drain, (int64_t)nowT, g_persist.batteryPercent);
    maybeWriteDailySummary(nowT);
#if ENABLE_DIAGNOSTICS
    if (g_persist.coldBootEpoch == 0) g_persist.coldBootEpoch = (int64_t)nowT;
#endif
  }
  if (force || old != g_persist.batteryPercent) app_.markDirty();
}

#if ENABLE_LOW_BATTERY_PROTECT
bool BatteryManager::isLow() const {
  return g_persist.batteryHasReading && g_persist.batteryPercent <= BATTERY_LOW_PERCENT;
}
// update() bramkuje pomiar pod obciazeniem, wiec batteryVoltage ~ OCV.
bool BatteryManager::isCritical() const {
  return g_persist.batteryHasReading &&
         (g_persist.batteryVoltage <= BATTERY_CRITICAL_VOLTAGE ||
          g_persist.batteryPercent <= BATTERY_CRITICAL_PERCENT);
}
#endif

// ============================================================================
//  DEFINICJE METOD — AudioManager (ES8311 + MP3 przez I2S, petla dzwonienia)
// ============================================================================
void AudioManager::ensurePins() {
  if (pinsReady_) return;
  Audio::audio_info_callback = audioInfo;
  audio.setPinout(I2S_BCLK_PIN, I2S_WS_PIN, I2S_DOUT_PIN, I2S_MCLK_PIN);
  audio.setOutput48KHz(true);
  audio.setVolume(DEFAULT_AUDIO_VOLUME);
  pinsReady_ = true;
}

bool AudioManager::ensureCodec() {
  if (ready_) return true;
  setCpuFrequencyMhz(AUDIO_CPU_MHZ);  // dekoder MP3 potrzebuje zapasu ponad dzienne 80 MHz
  pinMode(PA_ENABLE_PIN, OUTPUT);
  digitalWrite(PA_ENABLE_PIN, LOW);   // PA wlacza dopiero startSound() PO konfiguracji kodeka (bez trzasku)
  ensurePins();
  ready_ = codec.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000);
  if (ready_) { codec.setVolume(CODEC_VOLUME_PERCENT); codec.configBits16(); }
  else {
    Serial.println("ES8311 init failed");
    digitalWrite(PA_ENABLE_PIN, LOW);
    setCpuFrequencyMhz(DAY_CPU_MHZ);  // audio nie ruszy -> nie trzymaj podbitego CPU
  }
  return ready_;
}

void AudioManager::codecPowerDown() { codec.powerDown(); }

// Uspij kodek po odtwarzaniu — ES8311 jest na stalej szynie 3V3, wiec to najwiekszy
// odbiornik w spoczynku. startSound() go wskrzesza. Zdejmuje tez podbicie CPU
// (nocne sciezki i tak ustawiaja NIGHT_CPU_MHZ PO tym wywolaniu).
void AudioManager::shutdownCodec() {
  if (!ready_) { digitalWrite(PA_ENABLE_PIN, LOW); return; }
  audio.stopSong();
  codec.powerDown();
  digitalWrite(PA_ENABLE_PIN, LOW);
  ready_ = false;
  setCpuFrequencyMhz(DAY_CPU_MHZ);
}

bool AudioManager::startSound(uint8_t soundIndex, uint8_t volume) {
  stopPlayback(false);
  if (soundIndex >= SOUND_COUNT) soundIndex = 0;
  if (!app_.fs().ensureMounted() || !ensureCodec()) { app_.setStatus("blad FS/kodek"); app_.markDirty(); return false; }
  const char *path = SOUND_FILES[soundIndex];
  if (!app_.fs().fs()->exists(path)) { shutdownCodec(); app_.setStatus(String("brak ") + path); app_.markDirty(); return false; }

  digitalWrite(PA_ENABLE_PIN, HIGH);
  audio.stopSong();
  audio.setVolume(constrain(volume, 0, MAX_AUDIO_VOLUME));
  codec.setVolume(CODEC_VOLUME_PERCENT);
  if (!audio.connecttoFS(*app_.fs().fs(), path)) { shutdownCodec(); app_.setStatus(String("blad odtw. ") + path); app_.markDirty(); return false; }

  soundStartedMs_ = millis();
  playing_ = true;
  Serial.printf("Odtwarzam %s vol=%u\n", path, volume);
  app_.markDirty();
  return true;
}

void AudioManager::stopPlayback(bool userStopped) {
  if (userStopped) { ringing_ = false; ringingAlarm_ = -1; ringErrors_ = 0; }
  if (ready_) {
    audio.stopSong();
    audio.setVolume(0);
    codec.setVolume(0);
    digitalWrite(PA_ENABLE_PIN, LOW);
  }
  if (playing_) { app_.setStatus(userStopped ? "STOP" : "koniec"); playing_ = false; app_.markDirty(); }
}

void AudioManager::startRinging(int alarmIndex) {
  ringing_ = true;
  ringingAlarm_ = alarmIndex;
  ringStartMs_ = millis();
  ringLastRetryMs_ = millis();
  ringErrors_ = 0;
  app_.setStatus(String("BUDZI: ") + app_.alarm().config(alarmIndex).label);
  app_.markDirty();
  if (!startSound(app_.alarm().config(alarmIndex).sound, app_.alarm().config(alarmIndex).volume)) {
    ringErrors_ = 1;
    ringLastRetryMs_ = millis() - RING_RETRY_MS;
  }
}

void AudioManager::serviceAudio() {
  if (!playing_) return;
  if (!ready_) {
    playing_ = false;
    digitalWrite(PA_ENABLE_PIN, LOW);
    if (!ringing_) app_.setStatus("audio stop");
    app_.markDirty();
    return;
  }
  audio.loop();
  if (!audio.isRunning()) {
    if (ringing_) {
      // connecttoFS() zwraca true dla KAZDEGO istniejacego pliku; uszkodzony MP3
      // "konczy sie" po kilkudziesieciu ms. Bez zliczania budzik krecil sie cicho.
      playing_ = false;
      if (millis() - soundStartedMs_ < RING_MIN_PLAY_MS) {
        if (++ringErrors_ >= RING_MAX_ERRORS) { app_.setStatus("budzik: blad MP3"); ringing_ = false; ringingAlarm_ = -1; }
      } else ringErrors_ = 0;
      app_.markDirty();
      return;
    }
    stopPlayback(false);
  }
}

void AudioManager::serviceRinging() {
  if (!ringing_) return;
  if (millis() - ringStartMs_ > RING_TIMEOUT_MS) {
    app_.setStatus("budzik: timeout"); stopPlayback(false); ringing_ = false; ringingAlarm_ = -1; app_.markDirty(); return;
  }
  if (playing_) return;
  if (millis() - ringLastRetryMs_ < RING_RETRY_MS) return;
  ringLastRetryMs_ = millis();
  if (ringingAlarm_ >= 0 && startSound(app_.alarm().config(ringingAlarm_).sound, app_.alarm().config(ringingAlarm_).volume)) return;
  if (++ringErrors_ >= RING_MAX_ERRORS) { app_.setStatus("budzik: blad MP3"); ringing_ = false; ringingAlarm_ = -1; app_.markDirty(); }
}

// ============================================================================
//  DEFINICJE METOD — AlarmManager (model + harmonogram wyzwolen)
// ============================================================================
void AlarmManager::setDefaults() {
  for (int i = 0; i < ALARM_COUNT; i++) {
    alarms_[i].enabled = 0;
    alarms_[i].hour = 7; alarms_[i].minute = 0; alarms_[i].second = 0;
    alarms_[i].volume = DEFAULT_AUDIO_VOLUME;
    alarms_[i].sound = (uint8_t)i;
    snprintf(alarms_[i].label, sizeof(alarms_[i].label), "Budzik %d", i + 1);
  }
}

void AlarmManager::resetTriggerState() {
  for (int i = 0; i < ALARM_COUNT; i++) g_persist.lastTriggerDay[i] = -1;
}

void AlarmManager::sanitize(int index) {
  AlarmConfig &alarm = alarms_[index];
  alarm.enabled = alarm.enabled ? 1 : 0;
  if (alarm.hour > 23) alarm.hour = 7;
  if (alarm.minute > 59) alarm.minute = 0;
  if (alarm.second > 59) alarm.second = 0;
  if (alarm.volume > MAX_AUDIO_VOLUME) alarm.volume = DEFAULT_AUDIO_VOLUME;
  if (alarm.sound >= SOUND_COUNT) alarm.sound = (uint8_t)(index % SOUND_COUNT);
  alarm.label[sizeof(alarm.label) - 1] = '\0';
  if (alarm.label[0] == '\0')
    snprintf(alarm.label, sizeof(alarm.label), "Budzik %d", index + 1);
}

void AlarmManager::sanitizeAll() {
  for (int i = 0; i < ALARM_COUNT; i++) sanitize(i);
}

bool AlarmManager::anyEnabled() const {
  for (int i = 0; i < ALARM_COUNT; i++) if (alarms_[i].enabled) return true;
  return false;
}

// Najblizszy wlaczony budzik (dzis/jutro). Ranking po realnym epoch (mktime).
int AlarmManager::nextAlarmIndex(uint8_t *h, uint8_t *mi, uint8_t *s, char *label, size_t labelSize) {
  struct tm t;
  bool hasTime = getLocalTm(&t);
  time_t now = hasTime ? time(nullptr) : 0;
  long bestDelta = LONG_MAX;
  int best = -1;
  for (int i = 0; i < ALARM_COUNT; i++) {
    if (!alarms_[i].enabled) continue;
    long delta;
    if (hasTime) {
      time_t cand = epochForClock(t, alarms_[i].hour, alarms_[i].minute, alarms_[i].second, 0);
      if (cand <= now) cand = epochForClock(t, alarms_[i].hour, alarms_[i].minute, alarms_[i].second, 1);
      delta = (long)(cand - now);
    } else delta = i;
    if (delta < bestDelta) {
      bestDelta = delta; best = i;
      if (h) *h = alarms_[i].hour;
      if (mi) *mi = alarms_[i].minute;
      if (s) *s = alarms_[i].second;
      if (label && labelSize) { strncpy(label, alarms_[i].label, labelSize - 1); label[labelSize - 1] = 0; }
    }
  }
  return best;
}

void AlarmManager::checkAlarms(uint32_t lateWindowSec) {
  if (app_.audioActive()) return;
  time_t now = time(nullptr);
  struct tm t;
  if (!isSupportedEpoch(now)) return;
  localtime_r(&now, &t);
  for (int i = 0; i < ALARM_COUNT; i++) {
    if (!alarms_[i].enabled) continue;
    int occurrenceDay;
    if (alarmDueNow(t, now, alarms_[i].hour, alarms_[i].minute, alarms_[i].second, lateWindowSec, &occurrenceDay) &&
        g_persist.lastTriggerDay[i] != occurrenceDay) {
      g_persist.lastTriggerDay[i] = occurrenceDay;
      // Skonsumuj inne budziki wymagalne w tej samej chwili (kolizja godzin w noc
      // zmiany czasu: 02:30 i 03:30 = ten sam epoch) -> jedno dzwonienie.
      for (int j = 0; j < ALARM_COUNT; j++) {
        int otherDay;
        if (j != i && alarms_[j].enabled &&
            alarmDueNow(t, now, alarms_[j].hour, alarms_[j].minute, alarms_[j].second, lateWindowSec, &otherDay))
          g_persist.lastTriggerDay[j] = otherDay;
      }
      app_.audio().startRinging(i);
      return;
    }
  }
}

bool AlarmManager::anyDueNow(const struct tm &t) {
  time_t now = time(nullptr);
  for (int i = 0; i < ALARM_COUNT; i++)
    if (alarms_[i].enabled &&
        alarmDueNow(t, now, alarms_[i].hour, alarms_[i].minute, alarms_[i].second, ALARM_LATE_WINDOW_SEC, nullptr))
      return true;
  return false;
}

time_t AlarmManager::nextEpoch(const struct tm &base, time_t now) {
  time_t best = 0;
  for (int i = 0; i < ALARM_COUNT; i++) {
    if (!alarms_[i].enabled) continue;
    time_t cand = epochForClock(base, alarms_[i].hour, alarms_[i].minute, alarms_[i].second, 0);
    if (cand <= now) cand = epochForClock(base, alarms_[i].hour, alarms_[i].minute, alarms_[i].second, 1);
    if (best == 0 || cand < best) best = cand;
  }
  return best;
}

// ============================================================================
//  DEFINICJE METOD — NtpService (WiFi + NTP). Czas z RTC pierwszy; NTP NADPISUJE
//  tylko po REALNYM pakiecie (gate atomic). WiFi tylko na czas proby.
// ============================================================================
void NtpService::setCreds(const char *s, const char *p) {
  strncpy(ssid_, s ? s : "", sizeof(ssid_) - 1);
  strncpy(pass_, p ? p : "", sizeof(pass_) - 1);
  ssid_[sizeof(ssid_) - 1] = 0;
  pass_[sizeof(pass_) - 1] = 0;
}
void NtpService::setSsid(const String &s) { s.toCharArray(ssid_, sizeof(ssid_)); }
void NtpService::setPass(const String &p) { p.toCharArray(pass_, sizeof(pass_)); }

long NtpService::secsSinceLastSync() const {
  if (g_persist.lastNtpSyncEpoch <= 0) return -1;
  time_t now = time(nullptr);
  if (!isSupportedEpoch(now)) return -1;
  long d = (long)(now - (time_t)g_persist.lastNtpSyncEpoch);
  return d < 0 ? 0 : d;
}

void NtpService::stopSntpCallbacks() {
  g_ntpAcceptPacket.store(false, std::memory_order_release);
  if (esp_sntp_enabled()) esp_sntp_stop();
  sntp_set_time_sync_notification_cb(nullptr);
}

// Rollback po nieudanym NTP: przywroc czas sprzed proby (cala sekunda).
void NtpService::restoreTimeAfterFailedNtp() {
  if (isSupportedEpoch(fallbackTime_.tv_sec)) settimeofday(&fallbackTime_, nullptr);
  else setSystemEpoch(0);
}

void NtpService::stopWifiAndNtp() {
  bool wasAttempting = state_ != NtpSyncState::IDLE;
  if (state_ == NtpSyncState::WAITING_PACKET) restoreTimeAfterFailedNtp();
  stopSntpCallbacks();
  state_ = NtpSyncState::IDLE;
  manualAttempt_ = false;
  wifiHintActive_ = false;
  wifiLostMs_ = 0;
  g_ntpGotPacket.store(false, std::memory_order_release);
  // Przerwano probe, brak waznego czasu i brak zaplanowanego ponowienia -> zaplanuj
  // jedno (backoff), by zegar w koncu zdobyl czas mimo przerwania (np. zamkniecie WWW).
  if (wasAttempting && retriesLeft_ == 0 && !isSupportedEpoch(time(nullptr))) {
    retriesLeft_ = 1;
    nextRetryMs_ = millis() + backoffMs_;
  }
  WiFi.disconnect(false, false);
  WiFi.mode(WIFI_OFF);
}

bool NtpService::wifiBeginSmart() {
  WiFi.persistent(false);
#if FAST_NTP_RECONNECT
  if (g_persist.wifiHint) { WiFi.begin(ssid_, pass_, g_persist.wifiChannel, g_persist.wifiBssid); return true; }
#endif
  WiFi.begin(ssid_, pass_);
  return false;
}

void NtpService::wifiRetryFullScan() {
#if FAST_NTP_RECONNECT
  g_persist.wifiHint = false;
#endif
  WiFi.disconnect(false, false);
  WiFi.begin(ssid_, pass_);
}

void NtpService::wifiCacheHint() {
#if FAST_NTP_RECONNECT
  if (WiFi.status() == WL_CONNECTED) {
    g_persist.wifiChannel = WiFi.channel();
    const uint8_t *b = WiFi.BSSID();
    if (b) { for (int i = 0; i < 6; i++) g_persist.wifiBssid[i] = b[i]; g_persist.wifiHint = true; }
  }
#endif
}

// Blokujace laczenie WiFi (tylko sciezka panelu WWW; NTP laczy sie nieblokujaco).
bool NtpService::ensureWifi(uint32_t timeoutMs) {
  if (strlen(ssid_) == 0) return false;
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(true);                 // modem-sleep -> mniejszy prad STA
  if (WiFi.status() != WL_CONNECTED) {
    bool usedHint = wifiBeginSmart();
    bool retriedFullScan = false;
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
      if (usedHint && !retriedFullScan && millis() - start >= WIFI_HINT_FALLBACK_MS) {
        retriedFullScan = true; wifiRetryFullScan();
      }
      delay(100); yield();
      watchdogFeed();                  // blokuje do ~15 s — nie pozwol na falszywy reset
    }
  }
  bool ok = WiFi.status() == WL_CONNECTED;
  if (ok) wifiCacheHint(); else g_persist.wifiHint = false;
  return ok;
}

void NtpService::scheduleRtcWriteRetry(uint8_t retries) {
  rtcWriteRetriesLeft_ = retries;
  rtcWriteNextRetryMs_ = millis() + RTC_WRITE_RETRY_MS;
}

void NtpService::handleRtcWriteRetry() {
  if (rtcWriteRetriesLeft_ == 0 || (int32_t)(millis() - rtcWriteNextRetryMs_) < 0) return;
  if (app_.rtc().writeRtc(time(nullptr), app_.audioActive())) {
    rtcWriteRetriesLeft_ = 0;
    app_.setNtpStatus("NTP OK, RTC OK");
  } else if (--rtcWriteRetriesLeft_ > 0) {
    rtcWriteNextRetryMs_ = millis() + RTC_WRITE_RETRY_MS;
    app_.setNtpStatus("NTP OK, RTC retry");
  } else {
    app_.setNtpStatus("NTP OK, RTC ERR");
  }
  app_.markDirty();
}

void NtpService::finishNtpAttempt(bool ok) {
  bool wasManual = manualAttempt_;
  state_ = NtpSyncState::IDLE;
  manualAttempt_ = false;
  wifiHintActive_ = false;
  wifiLostMs_ = 0;
  stopSntpCallbacks();
  g_ntpGotPacket.store(false, std::memory_order_release);

  if (ok) {
    retriesLeft_ = 0;
    backoffMs_ = NTP_BACKOFF_MIN_MS;          // reset backoffu po sukcesie
  } else if (!wasManual && retriesLeft_ > 0) {
    nextRetryMs_ = millis() + NTP_QUICK_RETRY_MS;     // dokoncz serie szybka
  } else if (!isSupportedEpoch(time(nullptr))) {
    // Brak waznego czasu -> ponawiaj z BACKOFFEM (1 min -> 1 h). Dotyczy takze
    // proby RECZNEJ (manual ustawia retriesLeft_=0): zegar bez czasu nie moze
    // zostac bez zaplanowanego ponowienia tylko dlatego, ze klikniecie zawiodlo.
    retriesLeft_ = 1;
    nextRetryMs_ = millis() + backoffMs_;
    backoffMs_ = (backoffMs_ >= NTP_BACKOFF_MAX_MS / 2) ? NTP_BACKOFF_MAX_MS : backoffMs_ * 2;
  }
  if (!app_.web().isActive()) { WiFi.disconnect(false, false); WiFi.mode(WIFI_OFF); }
  app_.markDirty();
}

void NtpService::beginNtpRequest() {
  g_ntpAcceptPacket.store(false, std::memory_order_release);
  g_ntpGotPacket.store(false, std::memory_order_release);
  sntp_set_time_sync_notification_cb(onSntpSync);   // PRZED configTzTime
  sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED);
  gettimeofday(&fallbackTime_, nullptr);
  g_ntpAcceptPacket.store(true, std::memory_order_release);
  configTzTime(TZ_POLAND, NTP_SERVER_1, NTP_SERVER_2, NTP_SERVER_3);
  state_ = NtpSyncState::WAITING_PACKET;
  wifiLostMs_ = 0;                     // swiezy grace 8 s dla tej proby
  stateStartedMs_ = millis();
  app_.setNtpStatus("oczekiwanie na NTP");
  app_.markDirty();
}

void NtpService::syncNtp(bool manual) {
  if (state_ != NtpSyncState::IDLE) return;
  if (manual) retriesLeft_ = 0;
  manualAttempt_ = manual;
  app_.setNtpStatus(manual ? "manual: laczenie WiFi" : "auto: laczenie WiFi");
  app_.markDirty();

  if (strlen(ssid_) == 0) { app_.setNtpStatus("brak SSID WiFi"); finishNtpAttempt(false); return; }

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(true);
  if (WiFi.status() == WL_CONNECTED) {
    beginNtpRequest();
  } else {
    wifiHintActive_ = wifiBeginSmart();
    state_ = NtpSyncState::WIFI_CONNECTING;
    stateStartedMs_ = millis();
  }
}

// Wolane co petle. Zwraca true gdy NTP wlasnie zatwierdzil nowy czas (dla okna budzika).
bool NtpService::service() {
  if (state_ == NtpSyncState::WIFI_CONNECTING) {
    if (WiFi.status() == WL_CONNECTED) {
      wifiCacheHint();
      beginNtpRequest();
    } else if (wifiHintActive_ && millis() - stateStartedMs_ >= WIFI_HINT_FALLBACK_MS) {
      wifiHintActive_ = false;
      app_.setNtpStatus("WiFi: pelny skan");
      wifiRetryFullScan();
      stateStartedMs_ = millis();
      app_.markDirty();
    } else if (millis() - stateStartedMs_ >= NTP_WIFI_TIMEOUT_MS) {
      app_.setNtpStatus("blad WiFi");
      g_persist.wifiHint = false;
      finishNtpAttempt(false);
    }
    return false;
  }

  if (state_ != NtpSyncState::WAITING_PACKET) return false;

  if (g_ntpGotPacket.load(std::memory_order_acquire)) {
    stopSntpCallbacks();                       // zablokuj kolejny pakiet w trakcie zatwierdzania
    time_t syncedEpoch = time(nullptr);
    if (!isSupportedEpoch(syncedEpoch)) {
      restoreTimeAfterFailedNtp();
      app_.setNtpStatus("bledny czas NTP");
      finishNtpAttempt(false);
      return false;
    }
#if ENABLE_RTC_AUTOTRIM
    maybeAutotrimRtc(syncedEpoch);             // pomiar fazowy PRZED nadpisaniem RTC
    syncedEpoch = time(nullptr);               // pomiar mogl potrwac ~1.3 s -> swiezy epoch,
                                               // inaczej writeRtc nie wyrowna fazy
#endif
    struct tm local = {};
    localtime_r(&syncedEpoch, &local);

#if ENABLE_DIAGNOSTICS
    { // Dryft: porownaj STARY czas RTC (jeszcze nienadpisany) z czasem NTP.
      time_t oldRtc;
      if (app_.rtc().readEpoch(&oldRtc)) g_persist.lastRtcDriftSec = (int32_t)(oldRtc - syncedEpoch);
      g_persist.prevNtpSyncEpoch = g_persist.lastNtpSyncEpoch;
      g_persist.lastNtpSyncEpoch = (int64_t)syncedEpoch;
    }
#else
    g_persist.lastNtpSyncEpoch = (int64_t)syncedEpoch;
#endif

    bool rtcOk = app_.rtc().writeRtc(syncedEpoch, app_.audioActive());
    if (rtcOk) rtcWriteRetriesLeft_ = 0; else scheduleRtcWriteRetry();
    char buf[48];
    snprintf(buf, sizeof(buf), "OK %02d:%02d:%02d RTC:%s",
             local.tm_hour, local.tm_min, local.tm_sec, rtcOk ? "OK" : "RETRY");
    app_.setNtpStatus(buf);
    // Znacznik dobowego synchro 18:00 zalicza WYLACZNIE sync w oknie wieczornym
    // (>=18:00, przed noca). Poranne 06:00 ma wlasny znacznik ntpMorningDay.
    if (local.tm_hour >= NTP_HOUR && local.tm_hour < NIGHT_START_HOUR)
      g_persist.ntpStartedDay = localDayKey(local);
    finishNtpAttempt(true);
    return true;
  }

  // Zerwane WiFi w trakcie oczekiwania: pakiet juz nie nadejdzie. Krotki grace
  // (auto-reconnect czesto wraca po chwili), potem szybkie zakonczenie proby
  // zamiast palenia radia do pelnego NTP_PACKET_TIMEOUT_MS (50 s).
  if (WiFi.status() != WL_CONNECTED) {
    if (wifiLostMs_ == 0) {
      wifiLostMs_ = millis();
    } else if (millis() - wifiLostMs_ >= NTP_WIFI_LOST_GRACE_MS) {
      stopSntpCallbacks();
      restoreTimeAfterFailedNtp();
      app_.setNtpStatus("blad NTP (WiFi zerwane)");
      finishNtpAttempt(false);
      return false;
    }
  } else {
    wifiLostMs_ = 0;
  }

  if (millis() - stateStartedMs_ >= NTP_PACKET_TIMEOUT_MS) {
    stopSntpCallbacks();
    restoreTimeAfterFailedNtp();
    app_.setNtpStatus("blad NTP");
    finishNtpAttempt(false);
  }
  return false;
}

bool NtpService::handleNtpRetry() {
  if (state_ != NtpSyncState::IDLE) return true;
  if (retriesLeft_ == 0) return false;
  if ((int32_t)(millis() - nextRetryMs_) < 0) return true;
  --retriesLeft_;
  syncNtp(false);
  return true;
}

#if ENABLE_RTC_AUTOTRIM
// Autokalibracja offsetu: po kazdym pakiecie NTP zmierz dryft fazowo i — gdy
// okno NTP->NTP jest wiarygodne — przesun rejestr 0x02 tak, by wyzerowac ppm.
// Nowy offset trafia do NVS (przezywa utrate zasilania) i dziala od razu.
// Kwantyzacja 4.34 ppm/LSB zostawia dryf resztkowy do ~0.19 s/dobe.
void NtpService::maybeAutotrimRtc(time_t syncedEpoch) {
  if (app_.audioActive()) return;                 // pomiar blokuje ~1.3 s -> nie przy audio
  if (!g_persist.rtcPhaseAligned) return;         // faza startu okna nieznana -> ppm bez sensu
  if (g_persist.lastNtpSyncEpoch <= 0) return;
  int64_t windowSec = (int64_t)syncedEpoch - g_persist.lastNtpSyncEpoch;
  if (windowSec < AUTOTRIM_MIN_WINDOW_SEC || windowSec > AUTOTRIM_MAX_WINDOW_SEC) return;

  int64_t driftUs;
  if (!app_.rtc().measureDriftUs(&driftUs)) return;
  float ppm = (float)driftUs / (float)windowSec;  // us/s = ppm
  g_persist.lastDriftPpm = ppm;
  g_persist.lastDriftWindowSec = (int32_t)windowSec;
  if (fabsf(ppm) > AUTOTRIM_MAX_PLAUSIBLE_PPM) return;   // krysztal tak nie plywa -> zly pomiar

  int target = suggestedTotalOffsetLsb(ppm * PPM_SEC_PER_DAY, app_.rtc().offsetLsb());
  if (target == (int)app_.rtc().offsetLsb()) return;     // korekta ponizej 1 LSB
  if (app_.rtc().applyOffsetLsb((int8_t)target)) {
    g_persist.autotrimAdjustCount++;
    app_.settings().saveRtcOffset((int8_t)target);
    Serial.printf("RTC autotrim: %.2f ppm w %lld s -> offset LSB %d\n",
                  ppm, (long long)windowSec, target);
  }
}
#endif

void NtpService::checkSchedule() {
#if ENABLE_LOW_BATTERY_PROTECT
  // ECO: gdy mamy poprawny czas, NIE budz WiFi do auto-sync (najwiekszy pobor).
  // Pusty RTC -> pozwol sie synchronizowac mimo niskiej baterii (zegar musi miec czas).
  if (app_.battery().isLow() && isSupportedEpoch(time(nullptr))) return;
#endif
  if (handleNtpRetry()) return;
  struct tm t;
  if (!getLocalTm(&t)) return;
  int day = localDayKey(t);
  // Preferowane okno 18:00-18:05; pominiete -> dogon do poczatku nocy. ntpStartedDay
  // gwarantuje DOKLADNIE jeden sync/dobe.
  bool preferredWindow = (t.tm_hour == NTP_HOUR && t.tm_min <= NTP_MIN_WINDOW);
  bool catchupWindow   = (t.tm_hour >= NTP_HOUR && t.tm_hour < NIGHT_START_HOUR);
  if ((preferredWindow || catchupWindow) && g_persist.ntpStartedDay != day) {
    g_persist.ntpStartedDay = day;
    retriesLeft_ = 2;
    syncNtp(false);
  }
}

// ============================================================================
//  DEFINICJE METOD — DisplayManager (U8g2). Rysuje wprost na singletonie 'display'.
// ============================================================================
void DisplayManager::st7305SetStaticLowPower(bool enable) {
#if ST7305_STATIC_LPM
  (void)enable;   // zarezerwowane: HPM/LPM panelu — nieprzetestowana komenda
#else
  (void)enable;
#endif
}

// Ten egzemplarz ST7305 NIE przywraca obrazu po deep-sleep przez sam initInterface()
// -> pelny re-init (reset RST + begin) przy KAZDYM wybudzeniu. W dzien petla uzywa
// light-sleep (setup() sie nie powtarza) -> panel trzyma obraz -> brak mrugania.
void DisplayManager::setup(bool coldBoot) {
  (void)coldBoot;
  pinMode(RLCD_RST_PIN, OUTPUT);
  digitalWrite(RLCD_RST_PIN, HIGH);
  digitalWrite(RLCD_RST_PIN, LOW);
  delay(10);
  digitalWrite(RLCD_RST_PIN, HIGH);
  delay(10);
  // HW SPI: usuwa ~4 s opoznienia pelnego sendBuffer().
  SPI.begin(RLCD_SCK_PIN, -1, RLCD_MOSI_PIN, RLCD_CS_PIN);
  display.setBusClock(2000000UL);      // zalecenie sterownika U8g2 dla ST7305
  display.begin();
  display.setContrast(255);
  st7305SetStaticLowPower(false);
}

void DisplayManager::drawPhotoGlyph(int x, int y, int glyph, bool colon) {
  uint16_t p = colon ? PHOTO_COLON_OFFSET : pgm_read_word(&PHOTO_DIGIT_OFFSET[glyph]);
  for (int row = 0; row < PHOTO_DIGIT_H; row++) {
    uint8_t segs = pgm_read_byte(&PHOTO_FONT_RLE[p++]);
    for (uint8_t i = 0; i < segs; i++) {
      uint8_t start = pgm_read_byte(&PHOTO_FONT_RLE[p++]);
      uint8_t width = pgm_read_byte(&PHOTO_FONT_RLE[p++]);
      display.drawHLine(x + start, y + row, width);
    }
  }
  // Font (Impact) ma skrocona dolna stopke cyfry 4 — uzupelniamy ja tu (latka
  // zwiazana z PHOTO_DIGIT_W/H; zmiana fontu wymaga rewizji tych 2 prostokatow).
  if (!colon && glyph == 4) {
    display.drawBox(x + 39, y + 112, 41, 13);
    display.drawBox(x + 40, y + 125, 31, 25);
  }
}

void DisplayManager::drawLargeTime(uint8_t hour, uint8_t minute, int y) {
  int totalW = PHOTO_DIGIT_W * 4 + PHOTO_COLON_W + PHOTO_DIGIT_GAP * 4;
  int x = (LCD_WIDTH - totalW) / 2 + PHOTO_TIME_X_SHIFT;
  drawPhotoGlyph(x, y, hour / 10, false);   x += PHOTO_DIGIT_W + PHOTO_DIGIT_GAP;
  drawPhotoGlyph(x, y, hour % 10, false);   x += PHOTO_DIGIT_W + PHOTO_DIGIT_GAP;
  drawPhotoGlyph(x + PHOTO_COLON_X_SHIFT, y + PHOTO_COLON_Y_SHIFT, 0, true);
  x += PHOTO_COLON_W + PHOTO_DIGIT_GAP;
  drawPhotoGlyph(x, y, minute / 10, false); x += PHOTO_DIGIT_W + PHOTO_DIGIT_GAP;
  drawPhotoGlyph(x, y, minute % 10, false);
}

void DisplayManager::drawBellIcon(int x, int y, bool active) {
  display.setDrawColor(1);
  display.drawDisc(x + 12, y + 10, 8);
  display.drawBox(x + 5, y + 10, 15, 11);
  display.drawBox(x + 3, y + 20, 19, 4);
  display.drawDisc(x + 12, y + 25, 3);
  display.drawBox(x + 9, y + 3, 7, 4);
  if (!active) {
    display.drawLine(x + 1, y + 26, x + 25, y + 2);
    display.drawLine(x + 2, y + 26, x + 26, y + 2);
  }
}

void DisplayManager::drawBatteryIcon(int x, int y, uint8_t percent) {
  if (percent > 100) percent = 100;
  display.drawFrame(x, y, 28, 13);
  display.drawBox(x + 28, y + 4, 3, 5);
  int fill = map(percent, 0, 100, 0, 24);
  if (fill > 0) display.drawBox(x + 2, y + 2, fill, 9);
  char txt[8];
  snprintf(txt, sizeof(txt), "%u%%", percent);
  display.setFont(u8g2_font_7x14_tf);
  display.drawStr(x - 42, y + 12, txt);
}

void DisplayManager::drawCentered(const char *text, int y) {
  int w = display.getStrWidth(text);
  int x = (LCD_WIDTH - w) / 2;
  if (x < 0) x = 0;
  display.drawStr(x, y, text);
}

// Sygnatura WIDOCZNEJ tresci strony zegara (HH:MM, %aku, budzik, WWW). Pola
// niepokazywane (ntpStatus/statusText) NIE wchodza -> zero pelnych sendBuffer()
// identycznej klatki na panelu refleksyjnym (zero mrugniec).
uint32_t DisplayManager::clockSig(const struct tm &t, int day) {
  uint8_t nh = 0, nm = 0;
  int next = app_.alarm().nextAlarmIndex(&nh, &nm, nullptr, nullptr, 0);
  uint32_t h = 2166136261u;                           // FNV-1a
  auto mix = [&](uint32_t v) { h = (h ^ v) * 16777619u; };
  mix((uint32_t)t.tm_hour);
  mix((uint32_t)t.tm_min);
  mix((uint32_t)day);
  mix((uint32_t)app_.battery().percent());
  mix(app_.web().isActive() ? 1u : 0u);
  mix(app_.alarm().anyEnabled() ? 1u : 0u);
  mix(next >= 0 ? (uint32_t)(nh * 60 + nm + 1) : 0u);
  return h;
}

void DisplayManager::drawClockPage(const struct tm &t) {
  uint8_t nh = 0, nm = 0;
  int next = app_.alarm().nextAlarmIndex(&nh, &nm, nullptr, nullptr, 0);
  bool alarmOn = app_.alarm().anyEnabled();
  uint8_t pct = app_.battery().percent();
  bool webOn = app_.web().isActive();

  display.clearBuffer();
  display.setDrawColor(1);
  drawBatteryIcon(360, 8, pct);

  int y = (LCD_HEIGHT - PHOTO_DIGIT_H) / 2 + PHOTO_TIME_Y_SHIFT;
  drawLargeTime(t.tm_hour, t.tm_min, y);

  display.drawHLine(0, 264, LCD_WIDTH);
  drawBellIcon(10, 270, alarmOn);
  display.setFont(u8g2_font_7x14_tf);
  char bottom[80];
  if (next >= 0)
    snprintf(bottom, sizeof(bottom), "Budzik %02u:%02u %s  Aku %u%%  WWW:%s",
             nh, nm, alarmOn ? "ON" : "OFF", pct, webOn ? "ON" : "OFF");
  else
    snprintf(bottom, sizeof(bottom), "Budzik --:-- OFF  Aku %u%%  WWW:%s",
             pct, webOn ? "ON" : "OFF");
  display.drawStr(42, 287, bottom);
  display.sendBuffer();
}

void DisplayManager::drawPreviewPage() {
  uint8_t nh = 0, nm = 0, ns = 0;
  char label[sizeof(app_.alarm().config(0).label)] = "";
  int next = app_.alarm().nextAlarmIndex(&nh, &nm, &ns, label, sizeof(label));

  display.clearBuffer();
  display.setDrawColor(1);
  drawBatteryIcon(360, 8, app_.battery().percent());
  if (next >= 0) {
    drawBellIcon(10, 8, true);
    display.setFont(u8g2_font_9x15_tf);
    drawCentered("NAJBLIZSZY BUDZIK", 25);
    drawLargeTime(nh, nm, 60);
    display.drawHLine(0, 264, LCD_WIDTH);
    char detail[96];
    snprintf(detail, sizeof(detail), "%s  %02u:%02u:%02u  [%s]",
             label, nh, nm, ns, SOUND_NAMES[app_.alarm().config(next).sound % SOUND_COUNT]);
    drawCentered(detail, 287);
  } else {
    drawBellIcon(185, 35, false);
    display.setFont(u8g2_font_logisoso50_tf);
    drawCentered("--:--", 150);
    display.setFont(u8g2_font_9x15_tf);
    drawCentered("Brak aktywnego budzika", 215);
  }
  display.sendBuffer();
  lastClockSig_ = 0xFFFFFFFFu;   // powrot do zegara wymusi repaint
}

void DisplayManager::drawRingingPage() {
  display.clearBuffer();
  display.setDrawColor(1);
  drawBatteryIcon(360, 8, app_.battery().percent());
  drawBellIcon(185, 35, true);
  display.setFont(u8g2_font_logisoso50_tf);
  drawCentered("BUDZIK", 150);
  display.setFont(u8g2_font_9x15_tf);
  drawCentered(app_.statusText().c_str(), 210);
  drawCentered("Przycisk = wycisz", 245);
  display.sendBuffer();
  lastClockSig_ = 0xFFFFFFFFu;
}

void DisplayManager::drawNightPage() {
  display.clearBuffer();
  display.setDrawColor(1);
  display.setFont(u8g2_font_7x14_tf);
  drawCentered("TRYB NOCNY 23:59 - 06:00", 150);
  display.sendBuffer();
  app_.clearDirty();
  lastClockSig_ = 0xFFFFFFFFu;
}

#if ENABLE_LOW_BATTERY_PROTECT
void DisplayManager::drawCriticalBatteryPage() {
  display.clearBuffer();
  display.setDrawColor(1);
  drawBatteryIcon(360, 8, app_.battery().percent());
  display.setFont(u8g2_font_logisoso32_tf);
  drawCentered("ROZLADOWANY", 120);
  display.setFont(u8g2_font_9x15_tf);
  drawCentered("Podlacz ladowarke", 165);
  char b[40];
  snprintf(b, sizeof(b), "%u%%  %.2f V", app_.battery().percent(), app_.battery().voltage());
  drawCentered(b, 200);
  display.sendBuffer();
  lastClockSig_ = 0xFFFFFFFFu;
}
#endif

void DisplayManager::update() {
  if (app_.audioActive()) {                        // stan nadrzedny: dzwoni budzik
    if (app_.dirty()) { drawRingingPage(); app_.clearDirty(); }
    return;
  }
  if (app_.buttons().previewActive()) {            // stan nadrzedny: podglad budzika
    if (app_.buttons().previewWithinWindow()) {
      if (app_.dirty()) { drawPreviewPage(); app_.clearDirty(); }
      return;
    }
    app_.buttons().clearPreview();
    app_.markDirty();
  }

  struct tm t;
  if (!getLocalTm(&t)) {                            // brak czasu
    if (app_.dirty()) {
      display.clearBuffer();
      display.setDrawColor(1);
      display.setFont(u8g2_font_9x15_tf);
      drawCentered("Brak czasu RTC/NTP", 135);
      drawCentered(app_.ntpStatus().c_str(), 170);
      display.sendBuffer();
      app_.clearDirty();
      lastClockSig_ = 0xFFFFFFFFu;
    }
    return;
  }

  // Strona zegara — repaint TYLKO gdy zmienia sie WIDOCZNA tresc (clockSig).
  int day = localDayKey(t);
  uint32_t sig = clockSig(t, day);
  if (sig != lastClockSig_) {
    drawClockPage(t);
    lastDrawMinute_ = t.tm_min; lastDrawDay_ = day; lastClockSig_ = sig;
  }
  app_.clearDirty();
}

// ============================================================================
//  DEFINICJE METOD — WebManager (panel WWW)
// ============================================================================
void WebManager::redirectHome() {
  server.sendHeader("Location", "/");
  server.send(303, "text/plain", "");
}

void WebManager::appendMp3Manager(String &html) {
  html += F("<section><b>Pliki MP3</b>");
  if (!app_.fs().ensureMounted()) { html += F("<p>Brak zamontowanego systemu plikow.</p></section>"); return; }
  html += F("<p class='muted'>System plikow: "); html += htmlEscape(app_.fs().name());
  html += F("</p><form method='post' action='/mp3upload' enctype='multipart/form-data'>"
            "<div class='row'><input type='file' name='mp3' accept='.mp3,audio/mpeg' required>"
            "<button type='submit'>Dodaj / nadpisz MP3</button></div></form>");
  html += F("<table><tr><th>Plik</th><th>Rozmiar</th><th></th></tr>");
  bool any = false;
  File root = app_.fs().fs()->open("/");
  if (root) {
    for (File file = root.openNextFile(); file; file = root.openNextFile()) {
      if (!file.isDirectory()) {
        String path = file.name();
        if (!path.startsWith("/")) path = String("/") + path;
        String lower = path; lower.toLowerCase();
        if (lower.endsWith(".mp3")) {
          any = true;
          html += F("<tr><td>"); html += htmlEscape(path);
          html += F("</td><td>"); html += formatFileSize(file.size());
          html += F("</td><td><form method='post' action='/mp3delete' style='display:inline'>"
                    "<input type='hidden' name='file' value='"); html += htmlEscape(path);
          html += F("'><button type='submit'>Usun</button></form></td></tr>");
        }
      }
      file.close();
    }
    root.close();
  }
  if (!any) html += F("<tr><td colspan='3'>Brak plikow MP3</td></tr>");
  html += F("</table><p class='muted'>Nazwy: litery/cyfry oraz _ - ., maks. 25 znakow; budziki uzywaja /alarm1.mp3, /alarm2.mp3, /alarm3.mp3.</p></section>");
}

void WebManager::pageHeader(String &h) {
  h += F("<!doctype html><html><head><meta charset='utf-8'>"
         "<meta name='viewport' content='width=device-width,initial-scale=1'>"
         "<title>Zegar NTP</title><style>"
         "body{font-family:system-ui,Arial;margin:16px;background:#f5f5f5;color:#111}"
         "main{max-width:760px;margin:auto}section{background:#fff;border:1px solid #ddd;padding:12px;margin:12px 0}"
         "input,button,select{font:inherit;padding:6px;margin:3px}label{display:inline-block;margin:4px 8px 4px 0}"
         "table{width:100%;border-collapse:collapse}td,th{border-bottom:1px solid #ddd;padding:6px;text-align:left}"
         ".row{display:flex;gap:8px;flex-wrap:wrap;align-items:center}.muted{color:#666}"
         "</style></head><body><main><h1>Zegar NTP — 3 budziki</h1>"
         "<p class='muted'>" FW_NAME " v" FW_VERSION " &middot; build " FW_BUILD_DATE " " FW_BUILD_TIME "</p>");
}

void WebManager::handleRoot() {
  untilMs_ = millis() + WEB_WINDOW_MS;
  app_.battery().update(false);   // bez wymuszania: pod WiFi pomiar zanizony, OCV po zamknieciu WWW
  String html; html.reserve(14000);
  pageHeader(html);

  html += F("<section><b>Status</b><p>Czas: "); html += currentTimeText();
  html += F("<br>Budzik: ");     html += htmlEscape(app_.statusText());
  html += F("<br>NTP: ");        html += htmlEscape(app_.ntpStatus());
  html += F("<br>Pliki MP3: ");  html += htmlEscape(app_.fs().name());
  html += F("<br>Akumulator: "); html += app_.battery().percent();
  html += F("% (");              html += String(app_.battery().voltage(), 2);
  html += F(" V, ok. ");         html += String((uint32_t)app_.battery().percent() * BATTERY_CAPACITY_MAH / 100U);
  html += F(" mAh)<br>IP: ");    html += WiFi.localIP().toString();
  {
    float perDay = drainMahPerDay(app_.persist().drain);
    float daysLeft = drainDaysRemaining(app_.persist().drain, app_.battery().percent());
    html += F("<br>Zuzycie: ");
    if (perDay < 0)       html += F("zbieram dane (min ~12 h)");
    else if (perDay == 0) html += F("brak mierzalnego zuzycia");
    else { html += String(perDay, 1); html += F(" mAh/dzien, ~"); html += String(daysLeft, 0); html += F(" dni do rozladowania"); }
  }
  html += F("</p><p><a href='/sync'><button>Synchronizuj NTP</button></a>"
            "<a href='/stop'><button>Stop MP3</button></a>");
#if ENABLE_DIAGNOSTICS
  html += F("<a href='/info'><button>Informacje</button></a>");
#endif
  html += F("<a href='/off'><button>Wylacz panel WWW</button></a>");
  html += F("</p></section>");

  html += F("<form method='post' action='/save'>");
  // Hasla nie renderujemy (panel bez logowania; kazdy w LAN odczytalby je ze zrodla).
  html += F("<section><b>WiFi</b><div class='row'><label>SSID <input name='ssid' value='");
  html += htmlEscape(app_.ntp().ssid());
  html += F("'></label><label>Haslo <input name='pass' type='password' placeholder='(bez zmian)'>"
            "</label></div><p class='muted'>Puste pole hasla = haslo bez zmian.</p></section>");

  html += F("<section><b>Budziki</b><table>"
            "<tr><th>Wl.</th><th>Czas</th><th>Glos.</th><th>Dzwiek</th><th>Nazwa</th></tr>");
  for (int i = 0; i < ALARM_COUNT; i++) {
    AlarmConfig &a = app_.alarm().config(i);
    char timeBuf[16];
    snprintf(timeBuf, sizeof(timeBuf), "%02u:%02u:%02u", a.hour, a.minute, a.second);
    html += F("<tr><td><input type='checkbox' name='en"); html += i; html += F("' ");
    if (a.enabled) html += F("checked");
    html += F("></td><td><input type='time' step='1' name='time"); html += i;
    html += F("' value='"); html += timeBuf;
    html += F("'></td><td><input type='number' min='0' max='21' name='vol"); html += i;
    html += F("' value='"); html += a.volume;
    html += F("'></td><td><select name='snd"); html += i; html += F("'>");
    for (int sIdx = 0; sIdx < SOUND_COUNT; sIdx++) {
      html += F("<option value='"); html += sIdx; html += F("'");
      if (a.sound == sIdx) html += F(" selected");
      html += F(">"); html += SOUND_NAMES[sIdx]; html += F("</option>");
    }
    html += F("</select></td><td><input maxlength='15' name='label"); html += i;
    html += F("' value='"); html += htmlEscape(a.label);
    html += F("'></td></tr>");
  }
  html += F("</table><p class='muted'>Dzwiek 1/2/3 = /alarm1.mp3 /alarm2.mp3 /alarm3.mp3.</p></section>");
  html += F("<p><button type='submit'>Zapisz</button></p></form>");

  appendMp3Manager(html);

  html += F("<section><b>Test dzwiekow</b><div class='row'>");
  for (int sIdx = 0; sIdx < SOUND_COUNT; sIdx++) {
    html += F("<form method='post' action='/test' style='display:inline'>"
              "<input type='hidden' name='snd' value='"); html += sIdx;
    html += F("'><button type='submit'>Test "); html += SOUND_NAMES[sIdx]; html += F("</button></form>");
  }
  html += F("</div></section></main></body></html>");
  server.send(200, "text/html", html);
}

void WebManager::handleSave() {
  untilMs_ = millis() + WEB_WINDOW_MS;
  if (server.hasArg("ssid")) { String ssid = server.arg("ssid"); ssid.trim(); app_.ntp().setSsid(ssid); }
  if (server.hasArg("pass") && server.arg("pass").length() > 0) { String pass = server.arg("pass"); app_.ntp().setPass(pass); }

  for (int i = 0; i < ALARM_COUNT; i++) {
    String idx = String(i);
    AlarmConfig &a = app_.alarm().config(i);
    uint8_t prevEnabled = a.enabled;
    uint8_t prevH = a.hour, prevM = a.minute, prevS = a.second;
    a.enabled = server.hasArg(String("en") + idx) ? 1 : 0;
    uint8_t h, m, s;
    if (parseClock(server.arg(String("time") + idx), &h, &m, &s)) { a.hour = h; a.minute = m; a.second = s; }
    bool timeChanged = (a.hour != prevH || a.minute != prevM || a.second != prevS);
    bool reEnabled   = (a.enabled && !prevEnabled);
    if (timeChanged || reEnabled) app_.persist().lastTriggerDay[i] = -1;   // ponowne uzbrojenie: moze odpalic dzis
    String volumeArg = String("vol") + idx, soundArg = String("snd") + idx, labelArg = String("label") + idx;
    if (server.hasArg(volumeArg)) a.volume = constrain(server.arg(volumeArg).toInt(), 0, MAX_AUDIO_VOLUME);
    if (server.hasArg(soundArg))  a.sound  = constrain(server.arg(soundArg).toInt(), 0, SOUND_COUNT - 1);
    if (server.hasArg(labelArg)) {
      String label = server.arg(labelArg); label.trim();
      if (label.length() == 0) label = "Budzik " + String(i + 1);
      label.toCharArray(a.label, sizeof(a.label));
    }
    app_.alarm().sanitize(i);
  }
  app_.settings().save();
  app_.markDirty();
  redirectHome();
}

void WebManager::handleSync() { app_.ntp().syncNtp(true); redirectHome(); }
void WebManager::handleStop() { app_.audio().stopPlayback(true); redirectHome(); }

void WebManager::handleTest() {
  // Nie przerywaj dzwoniacego budzika testem (startSound nie kasuje ringing).
  if (!app_.audio().isRinging()) { uint8_t sIdx = constrain(server.arg("snd").toInt(), 0, SOUND_COUNT - 1); app_.audio().startSound(sIdx, DEFAULT_AUDIO_VOLUME); }
  redirectHome();
}

void WebManager::handleMp3UploadDone() { redirectHome(); }

void WebManager::handleMp3UploadStream() {
  untilMs_ = millis() + WEB_WINDOW_MS;
  watchdogFeed();   // callback per fragment: upload duzego MP3 blokuje loop() minutami
  HTTPUpload &upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    if (mp3UploadFile_) mp3UploadFile_.close();
    mp3UploadAccepted_ = false; mp3UploadHadData_ = false; mp3UploadPath_ = ""; mp3UploadTempPath_ = "";
    if (!normalizeMp3Path(upload.filename, mp3UploadPath_, false)) { app_.setStatus("zla nazwa MP3"); app_.markDirty(); return; }
    if (!app_.fs().ensureMounted()) { app_.setStatus("brak FS dla MP3"); app_.markDirty(); return; }
    if (app_.audioActive()) app_.audio().stopPlayback(true);
    mp3UploadTempPath_ = mp3UploadPath_ + ".part";
    if (app_.fs().fs()->exists(mp3UploadTempPath_)) app_.fs().fs()->remove(mp3UploadTempPath_);
    mp3UploadFile_ = app_.fs().fs()->open(mp3UploadTempPath_, FILE_WRITE);
    mp3UploadAccepted_ = (bool)mp3UploadFile_;
    app_.setStatus(mp3UploadAccepted_ ? String("upload ") + mp3UploadPath_ : "blad zapisu MP3");
    app_.markDirty();
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (mp3UploadAccepted_ && mp3UploadFile_) {
      size_t written = mp3UploadFile_.write(upload.buf, upload.currentSize);
      if (written != upload.currentSize) { mp3UploadAccepted_ = false; app_.setStatus("blad zapisu MP3"); }
      else if (upload.currentSize > 0) mp3UploadHadData_ = true;
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (mp3UploadFile_) mp3UploadFile_.close();
    if (mp3UploadAccepted_ && mp3UploadHadData_) {
      String backupPath = mp3UploadPath_ + ".bak";
      if (app_.fs().fs()->exists(backupPath)) app_.fs().fs()->remove(backupPath);
      bool hadOld = app_.fs().fs()->exists(mp3UploadPath_);
      bool oldMoved = !hadOld || app_.fs().fs()->rename(mp3UploadPath_, backupPath);
      bool newInstalled = oldMoved && app_.fs().fs()->rename(mp3UploadTempPath_, mp3UploadPath_);
      if (newInstalled) {
        if (app_.fs().fs()->exists(backupPath)) app_.fs().fs()->remove(backupPath);
        app_.setStatus(String("dodano ") + mp3UploadPath_);
      } else {
        if (oldMoved && hadOld) {
          if (app_.fs().fs()->exists(mp3UploadPath_)) app_.fs().fs()->remove(mp3UploadPath_);
          if (app_.fs().fs()->exists(backupPath)) app_.fs().fs()->rename(backupPath, mp3UploadPath_);
        }
        if (app_.fs().fs()->exists(mp3UploadTempPath_)) app_.fs().fs()->remove(mp3UploadTempPath_);
        app_.setStatus("blad instalacji MP3");
      }
    } else {
      if (mp3UploadTempPath_.length() && app_.fs().fs() && app_.fs().fs()->exists(mp3UploadTempPath_)) app_.fs().fs()->remove(mp3UploadTempPath_);
      if (app_.statusText() == String("upload ") + mp3UploadPath_) app_.setStatus("pusty MP3");
    }
    app_.markDirty();
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    if (mp3UploadFile_) mp3UploadFile_.close();
    if (mp3UploadTempPath_.length() && app_.fs().fs() && app_.fs().fs()->exists(mp3UploadTempPath_)) app_.fs().fs()->remove(mp3UploadTempPath_);
    app_.setStatus("upload przerwany");
    app_.markDirty();
  }
}

void WebManager::handleMp3Delete() {
  String path;
  if (!normalizeMp3Path(server.arg("file"), path, true)) app_.setStatus("zla nazwa MP3");
  else if (!app_.fs().ensureMounted()) app_.setStatus("brak FS dla MP3");
  else {
    if (app_.audioActive()) app_.audio().stopPlayback(true);
    app_.setStatus((app_.fs().fs()->exists(path) && app_.fs().fs()->remove(path)) ? String("usunieto ") + path : String("brak ") + path);
  }
  app_.markDirty();
  redirectHome();
}

#if ENABLE_DIAGNOSTICS
void WebManager::handleInfo() {
  untilMs_ = millis() + WEB_WINDOW_MS;
  app_.battery().update(false);
  String html; html.reserve(4500);
  pageHeader(html);
  html += F("<section><b>Informacje / diagnostyka</b><table>");
  auto row = [&](const char *k, const String &v) {
    html += F("<tr><td>"); html += k; html += F("</td><td>"); html += htmlEscape(v); html += F("</td></tr>");
  };
  row("Nazwa", FW_NAME);
  row("Wersja", String("v" FW_VERSION));
  row("Kompilacja", String(FW_BUILD_DATE) + " " + FW_BUILD_TIME);
  row("Czas lokalny", currentTimeText());
  {
    long s = app_.ntp().secsSinceLastSync();
    row("Od ostatniego NTP", s < 0 ? String("brak synchronizacji")
        : String(s / 3600) + " h " + String((s % 3600) / 60) + " min");
  }
  row("Przyczyna ostatniego resetu", resetReasonText(esp_reset_reason()));
  row("Rozruchy od zimnego startu", String(app_.persist().bootCount));
  {
    time_t now = time(nullptr);
    if (isSupportedEpoch(now) && app_.persist().coldBootEpoch > 0) {
      long s = (long)(now - (time_t)app_.persist().coldBootEpoch); if (s < 0) s = 0;
      char buf[48];
      snprintf(buf, sizeof(buf), "%ld dni %02ld:%02ld:%02ld", s / 86400, (s % 86400) / 3600, (s % 3600) / 60, s % 60);
      row("Czas pracy (od zimnego startu)", buf);
    } else row("Czas pracy (od zimnego startu)", "ustala sie");
  }
  row("Resety brownout / WDT / panic",
      String(app_.settings().brownoutResets()) + " / " + String(app_.settings().wdtResets()) + " / " + String(app_.settings().panicResets()));
  row("Heap wolny", String(ESP.getFreeHeap()) + " B");
  row("Heap min. od startu", String(ESP.getMinFreeHeap()) + " B");
  row("Heap najwiekszy blok", String(ESP.getMaxAllocHeap()) + " B");
  row("RTC offset (0x02)",
      "0x" + String(app_.rtc().offsetRaw(), HEX) + " (LSB " + String((int)app_.rtc().offsetLsb()) + ", " + String(app_.rtc().offsetPpm(), 1) + " ppm)");
  row("RTC offset status",
      app_.rtc().offsetVerified() ? String("OK (potwierdzony odczytem 0x02)")
        : (app_.rtc().offsetRaw() == 0xFF ? String("BLAD: brak odczytu rejestru 0x02")
             : String("NIEZWERYFIKOWANY: zapis 0x") + String(app_.rtc().targetOffsetRaw(), HEX) + " != odczyt 0x" + String(app_.rtc().offsetRaw(), HEX)));
  if (app_.persist().lastNtpSyncEpoch > 0 && app_.persist().prevNtpSyncEpoch > 0) {
    long  winSec = (long)(app_.persist().lastNtpSyncEpoch - app_.persist().prevNtpSyncEpoch);
    float winH   = winSec / 3600.0f;
    float driftPpm    = winSec > 0 ? (float)app_.persist().lastRtcDriftSec / winSec * 1e6f : 0.0f;
    float driftPerDay = driftPpm * PPM_SEC_PER_DAY;
    float uncPpm      = winSec > 0 ? 1.0f / winSec * 1e6f : 0.0f;
    int   suggest     = suggestedTotalOffsetLsb(driftPerDay, app_.rtc().offsetLsb());
    row("Dryft RTC (ostatni sync)",
        String(app_.persist().lastRtcDriftSec) + " s / " + String(winH, 1) + " h  (" + String(driftPpm, 1) + " +-" +
        String(uncPpm, 1) + " ppm, ~" + String(driftPerDay, 2) + " s/dobe)");
    row("Sugerowany offset RTC",
        "LSB " + String(suggest) + " -> 0x" + String(encodeRtcOffset(suggest, false), HEX) +
        " (z biezacego LSB " + String((int)app_.rtc().offsetLsb()) + ")");
  } else row("Dryft RTC (ostatni sync)", "brak danych (min. 2 synchronizacje)");
#if ENABLE_RTC_AUTOTRIM
  if (app_.persist().lastDriftWindowSec > 0)
    row("Autokalibracja RTC",
        String(app_.persist().lastDriftPpm, 2) + " ppm (pomiar fazowy ~ms, okno " +
        String(app_.persist().lastDriftWindowSec / 3600.0f, 1) + " h), korekt offsetu: " +
        String(app_.persist().autotrimAdjustCount));
  else
    row("Autokalibracja RTC", "czeka na pierwsze pelne okno NTP->NTP (min. 6 h)");
#else
  row("Dryft — rozdzielczosc", "1 s (PCF85063A bez rejestru pod-sekundowego; mierz kilka dob dla < 1 ppm)");
#endif
  row("Akumulator", String(app_.battery().percent()) + "% (" + String(app_.battery().voltage(), 2) + " V)");
#if ENABLE_LOW_BATTERY_PROTECT
  row("Ochrona ogniwa", app_.battery().isCritical() ? "KRYTYCZNY" : (app_.battery().isLow() ? "tryb oszczedny (bez auto-NTP)" : "OK"));
#endif
  if (WiFi.status() == WL_CONNECTED) { row("WiFi", WiFi.SSID() + "  RSSI " + String(WiFi.RSSI()) + " dBm"); row("IP", WiFi.localIP().toString()); }
  else row("WiFi", "rozlaczone");
  row("System plikow", app_.fs().name());
  html += F("</table><p><a href='/'><button>Powrot</button></a></p></section></main></body></html>");
  server.send(200, "text/html", html);
}
#endif

// Reczne wylaczenie panelu WWW: odsyla potwierdzenie, a samo zamkniecie wykonuje
// petla PO wyslaniu odpowiedzi (ustawiamy termin okna na "teraz").
void WebManager::handleWebOff() {
  String html; pageHeader(html);
  html += F("<section><b>Panel WWW wylaczony.</b>"
            "<p class='muted'>WiFi zostalo wylaczone dla oszczednosci baterii. "
            "Aby wlaczyc panel ponownie: przytrzymaj przycisk KEY na zegarze.</p>"
            "</section></main></body></html>");
  server.send(200, "text/html", html);
  untilMs_ = millis();   // petla wykryje uplyw okna i zamknie panel po flushu strony
}

void WebManager::installRoutes() {
  server.on("/", HTTP_GET, [this]{ handleRoot(); });
  server.on("/save", HTTP_POST, [this]{ handleSave(); });
  server.on("/sync", HTTP_GET, [this]{ handleSync(); });
  server.on("/stop", HTTP_GET, [this]{ handleStop(); });
  server.on("/test", HTTP_POST, [this]{ handleTest(); });
  server.on("/mp3upload", HTTP_POST, [this]{ handleMp3UploadDone(); }, [this]{ handleMp3UploadStream(); });
  server.on("/mp3delete", HTTP_POST, [this]{ handleMp3Delete(); });
  server.on("/off", HTTP_GET, [this]{ handleWebOff(); });
#if ENABLE_DIAGNOSTICS
  server.on("/info", HTTP_GET, [this]{ handleInfo(); });
#endif
}

bool WebManager::startWebWindow() {
  if (active_) {
    if (WiFi.status() == WL_CONNECTED) { untilMs_ = millis() + WEB_WINDOW_MS; app_.markDirty(); return true; }
    server.stop(); active_ = false;
  }
  if (!app_.ntp().ensureWifi(NTP_WIFI_TIMEOUT_MS)) { app_.setStatus("brak polaczenia WiFi"); app_.ntp().stopWifiAndNtp(); app_.markDirty(); return false; }
  if (!routesInstalled_) { installRoutes(); routesInstalled_ = true; }
  server.begin();
  active_ = true;
  untilMs_ = millis() + WEB_WINDOW_MS;
  app_.markDirty();
  Serial.print("WWW IP: "); Serial.println(WiFi.localIP());
  return true;
}

void WebManager::stopWindow() {
  server.stop();
  active_ = false;
  // Aktywna proba NTP (do ~50 s) sama dokonczy walidacje i wylaczy WiFi.
  if (!app_.ntp().isBusy()) app_.ntp().stopWifiAndNtp();
  app_.markDirty();
}

void WebManager::forceStop() {
  server.stop();
  active_ = false;
}

void WebManager::service() {
  if (WiFi.status() != WL_CONNECTED) {
    // Chwilowy zanik (roaming) nie zamyka okna od razu: auto-reconnect zwykle wraca.
    if (wifiLostMs_ == 0) wifiLostMs_ = millis();
    if (millis() - wifiLostMs_ > WEB_WIFI_GRACE_MS) { wifiLostMs_ = 0; app_.setStatus("WiFi rozlaczone"); stopWindow(); }
  } else {
    wifiLostMs_ = 0;
    server.handleClient();
    if ((int32_t)(millis() - untilMs_) >= 0) stopWindow();
  }
}

// ============================================================================
//  DEFINICJE METOD — PowerManager (tryb nocny / light-sleep / sen ochronny)
// ============================================================================
void PowerManager::releaseNightHolds() {
  gpio_deep_sleep_hold_dis();
  for (auto pin : HOLD_PINS) gpio_hold_dis(pin);
  gpio_hold_dis((gpio_num_t)PA_ENABLE_PIN);
  rtc_gpio_deinit((gpio_num_t)KEY_PIN);
  rtc_gpio_deinit((gpio_num_t)PREVIEW_BUTTON_PIN);
  rtc_gpio_deinit((gpio_num_t)RTC_INT_PIN);
}

void PowerManager::holdNightPins() {
  pinMode(RLCD_RST_PIN, OUTPUT);  digitalWrite(RLCD_RST_PIN, HIGH);
  pinMode(RLCD_CS_PIN, OUTPUT);   digitalWrite(RLCD_CS_PIN, HIGH);
  pinMode(RLCD_DC_PIN, OUTPUT);   digitalWrite(RLCD_DC_PIN, HIGH);
  pinMode(RLCD_SCK_PIN, OUTPUT);  digitalWrite(RLCD_SCK_PIN, LOW);
  pinMode(RLCD_MOSI_PIN, OUTPUT); digitalWrite(RLCD_MOSI_PIN, LOW);
  pinMode(PA_ENABLE_PIN, OUTPUT); digitalWrite(PA_ENABLE_PIN, LOW);
  for (auto pin : HOLD_PINS) gpio_hold_en(pin);
  gpio_hold_en((gpio_num_t)PA_ENABLE_PIN);
  gpio_deep_sleep_hold_en();
}

void PowerManager::shutdownNightPeripherals() {
  if (app_.web().isActive()) app_.web().forceStop();
  app_.ntp().stopWifiAndNtp();
  app_.audio().stopPlayback(true);
  app_.audio().shutdownCodec();
  pinMode(PA_ENABLE_PIN, OUTPUT);
  digitalWrite(PA_ENABLE_PIN, LOW);
}

time_t PowerManager::nightEndEpoch(time_t now) {
  if (!isSupportedEpoch(now)) return 0;
  struct tm t; localtime_r(&now, &t);
  time_t end = epochForClock(t, NIGHT_END_HOUR, NIGHT_END_MIN, 0, 0);
  if (end <= now) end = epochForClock(t, NIGHT_END_HOUR, NIGHT_END_MIN, 0, 1);
  return end;
}

#if ENABLE_LOW_BATTERY_PROTECT
// Ochronny deep-sleep przy krytycznym ogniwie: chroni Li-ion przed glebokim
// rozladowaniem i brownoutem. Budzi sie cyklicznie (timer) sprawdzic ladowanie;
// KEY pozwala wybudzic recznie. Deep-sleep resetuje chip -> setup() ponownie
// zmierzy ogniwo i wznowi prace lub usnie dalej.
void PowerManager::enterCriticalBatterySleepIfNeeded() {
  if (!app_.battery().isCritical()) return;
  if (app_.busy()) return;   // dzwoni budzik / audio / WWW / podglad / aktywny NTP / zapis RTC

  Serial.printf("Akumulator krytyczny (%u%%, %.2f V) -> ochronny sen %lu s\n",
                app_.battery().percent(), app_.battery().voltage(), (unsigned long)BATTERY_CRITICAL_RECHECK_SEC);
  app_.display().drawCriticalBatteryPage();
  delay(150);
  shutdownNightPeripherals();
  setCpuFrequencyMhz(NIGHT_CPU_MHZ);
  watchdogFeed();
  esp_sleep_enable_timer_wakeup((uint64_t)BATTERY_CRITICAL_RECHECK_SEC * 1000000ULL);
  esp_sleep_enable_ext1_wakeup((1ULL << KEY_PIN), ESP_EXT1_WAKEUP_ANY_LOW);
  rtc_gpio_pullup_en((gpio_num_t)KEY_PIN);
  rtc_gpio_pulldown_dis((gpio_num_t)KEY_PIN);
  holdNightPins();
  Serial.flush();
  esp_deep_sleep_start();
}
#endif

// Tryb nocny: deep-sleep do 06:00 / najblizszego budzika. Ekran zgaszony. Budzenie:
// alarm RTC (dokladny) + timer ESP (zabezpieczenie) + przyciski EXT1.
void PowerManager::enterNightSleepIfNeeded(bool force) {
  struct tm t;
  if (!getLocalTm(&t) || !isNightMode(t)) return;
  if (app_.alarm().anyDueNow(t)) return;
  if (!force && (app_.busy() || app_.buttons().wakeHeld())) return;   // utrzymaj wybudzenie po nacisnieciu

  time_t now = time(nullptr);
  time_t wakeAt = nightEndEpoch(now);
  time_t alarmAt = app_.alarm().nextEpoch(t, now);
  if (alarmAt > now && alarmAt < wakeAt) wakeAt = alarmAt;
  if (wakeAt <= now) wakeAt = now + 6 * 60 * 60;
  uint64_t sleepSec = (uint64_t)(wakeAt - now);
  if (sleepSec < NIGHT_MIN_SLEEP_SEC) { sleepSec = NIGHT_MIN_SLEEP_SEC; wakeAt = now + (time_t)sleepSec; }

  bool rtcAlarmOk = app_.rtc().setWakeAlarm(wakeAt);
  if (rtcAlarmOk && digitalRead(RTC_INT_PIN) != HIGH) { app_.rtc().disableWakeAlarm(); rtcAlarmOk = false; }
  // Bez dzialajacego alarmu RTC budzi tylko timer ESP na niedokladnym RC: spij w
  // odcinkach, by kazde wybudzenie zakotwiczylo czas z PCF85063A (setup) i dryft
  // pojedynczego odcinka byl << okno wylapania budzika.
  if (!rtcAlarmOk) {
    bool alarmIsNextEvent = (alarmAt > now && wakeAt == alarmAt);
    uint64_t cap = alarmIsNextEvent ? NIGHT_FALLBACK_ALARM_CHUNK_SEC : NIGHT_FALLBACK_CHUNK_SEC;
    if (sleepSec > cap) sleepSec = cap;
  }
  Serial.printf("Tryb nocny: sleep %llu s, RTC alarm: %s\n",
                (unsigned long long)sleepSec, rtcAlarmOk ? "OK" : "ERR");
  app_.display().drawNightPage();
  delay(150);
  shutdownNightPeripherals();
  setCpuFrequencyMhz(NIGHT_CPU_MHZ);
  esp_sleep_enable_timer_wakeup(sleepSec * 1000000ULL);
  uint64_t wakeMask = (1ULL << KEY_PIN) | (1ULL << PREVIEW_BUTTON_PIN);
  if (rtcAlarmOk) wakeMask |= (1ULL << RTC_INT_PIN);
  esp_sleep_enable_ext1_wakeup(wakeMask, ESP_EXT1_WAKEUP_ANY_LOW);
  rtc_gpio_pullup_en((gpio_num_t)KEY_PIN);  rtc_gpio_pulldown_dis((gpio_num_t)KEY_PIN);
  rtc_gpio_pullup_en((gpio_num_t)PREVIEW_BUTTON_PIN); rtc_gpio_pulldown_dis((gpio_num_t)PREVIEW_BUTTON_PIN);
  if (rtcAlarmOk) { rtc_gpio_pullup_en((gpio_num_t)RTC_INT_PIN); rtc_gpio_pulldown_dis((gpio_num_t)RTC_INT_PIN); }
  watchdogFeed();
  holdNightPins();
  Serial.flush();
  esp_deep_sleep_start();
}

// Dynamiczny light-sleep w dzien: spij do granicy nastepnej minuty lub do sekundy
// najblizszego budzika, max 60 s. RLCD trzyma obraz; przyciski wybudzaja przez EXT1.
// Light-sleep NIE rebootuje -> setup() sie nie powtarza -> brak mrugania.
uint64_t PowerManager::computeIdleSleepUs() {
  time_t now = time(nullptr);
  if (!isSupportedEpoch(now)) return IDLE_LIGHT_SLEEP_US;   // czas nieznany -> krotki sen
  struct tm t; localtime_r(&now, &t);
  uint64_t sleepSec = 60 - (uint64_t)t.tm_sec;             // do granicy minuty
  time_t alarmAt = app_.alarm().nextEpoch(t, now);
  if (alarmAt > now) { uint64_t toAlarm = (uint64_t)(alarmAt - now); if (toAlarm < sleepSec) sleepSec = toAlarm; }
  if (sleepSec > IDLE_LIGHT_SLEEP_MAX_SEC) sleepSec = IDLE_LIGHT_SLEEP_MAX_SEC;
  if (sleepSec < 1) sleepSec = 1;
  return sleepSec * 1000000ULL;
}

void PowerManager::idlePowerSave() {
  // Nie usypiaj na dlugo, gdy cos wymaga szybkiej obslugi (App::busy) lub trwa
  // okno utrzymania wybudzenia po nacisnieciu przycisku (Buttons::wakeHeld).
  if (app_.busy() || app_.buttons().wakeHeld()) { delay(5); return; }
  if (digitalRead(KEY_PIN) == LOW || digitalRead(PREVIEW_BUTTON_PIN) == LOW) { delay(5); return; }

  if (app_.audio().codecActive()) app_.audio().shutdownCodec();   // kodek niepotrzebny -> uspij ES8311

  esp_sleep_enable_timer_wakeup(computeIdleSleepUs());
  const uint64_t wakeMask = (1ULL << KEY_PIN) | (1ULL << PREVIEW_BUTTON_PIN);
  esp_err_t extWake = esp_sleep_enable_ext1_wakeup(wakeMask, ESP_EXT1_WAKEUP_ANY_LOW);
#if POWER_DOWN_FLASH_IN_SLEEP
  esp_sleep_pd_config(ESP_PD_DOMAIN_VDDSDIO, ESP_PD_OPTION_OFF);
#endif
  watchdogFeed();
  esp_light_sleep_start();
  watchdogFeed();
  // EXT1 zostawia RTC-hold na padach wybudzajacych — zwolnij od razu, inaczej
  // digitalRead zwraca zatrzasniety stan i przyciski sa martwe do restartu.
  rtc_gpio_hold_dis((gpio_num_t)KEY_PIN);
  rtc_gpio_hold_dis((gpio_num_t)PREVIEW_BUTTON_PIN);
  esp_sleep_wakeup_cause_t wakeCause = esp_sleep_get_wakeup_cause();
  uint64_t wakePins = wakeCause == ESP_SLEEP_WAKEUP_EXT1 ? esp_sleep_get_ext1_wakeup_status() : 0;
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
  if (extWake == ESP_OK) esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_EXT1);

  // Po light-sleep czas systemowy szedl z mniej dokladnego RTC slow clock —
  // zakotwicz ponownie z PCF85063A, by dryft nie przesuwal zmiany minuty.
  app_.rtc().readToSystem();

  if (wakeCause == ESP_SLEEP_WAKEUP_EXT1) {
    if (wakePins & (1ULL << KEY_PIN))            app_.buttons().primeKeyPressFromWake();
    if (wakePins & (1ULL << PREVIEW_BUTTON_PIN)) app_.buttons().onPreviewWakeFromSleep();
  }
}

// ============================================================================
//  DEFINICJE METOD — Buttons
// ============================================================================
void Buttons::begin() {
  pinMode(KEY_PIN, INPUT_PULLUP);
  pinMode(PREVIEW_BUTTON_PIN, INPUT_PULLUP);
  keyRaw_ = digitalRead(KEY_PIN);  keyStable_ = keyRaw_;  keyChangedMs_ = millis();
  bootRaw_ = digitalRead(PREVIEW_BUTTON_PIN); bootStable_ = bootRaw_; bootChangedMs_ = millis();
}

void Buttons::showPreview() {
  if (app_.audioActive()) return;
  previewActive_ = true;
  previewUntilMs_ = millis() + ALARM_PREVIEW_MS;
  holdWake();
  app_.markDirty();
}

void Buttons::primeKeyPress() {
  keyRaw_ = LOW; keyStable_ = LOW; keyChangedMs_ = millis();
  keyPressMs_ = millis(); keyLongFired_ = false;
}

void Buttons::primeKeyPressFromWake() {
  primeKeyPress();
  holdWake();
}

void Buttons::onPreviewWakeFromSleep() {
  bootRaw_ = LOW; bootStable_ = LOW; bootPressedMs_ = millis(); bootChangedMs_ = millis();
  showPreview();
}

void Buttons::handleKey() {
  bool raw = digitalRead(KEY_PIN);
  if (raw != keyRaw_) { keyRaw_ = raw; keyChangedMs_ = millis(); }
  if (millis() - keyChangedMs_ > BUTTON_DEBOUNCE_MS && raw != keyStable_) {
    keyStable_ = raw;
    if (keyStable_ == LOW) {                         // nacisniety
      keyPressMs_ = millis(); keyLongFired_ = false;
      holdWake();
    } else {                                         // zwolniony — akcja krotka
      if (!keyLongFired_) {                          // krotki KEY: wycisz budzik lub wlacz panel WWW
        if (app_.audioActive()) app_.audio().stopPlayback(true); // dzwoni budzik -> wycisz
        else app_.web().startWebWindow();                        // nie dzwoni -> wlacz panel WWW
      }
      keyPressMs_ = 0;
    }
  }
  // Dlugie przytrzymanie KEY: nic nie robi — ustawiamy tylko keyLongFired, by
  // zwolnienie po dlugim przytrzymaniu NIE wywolalo akcji krotkiej.
  if (keyStable_ == LOW && !keyLongFired_ && keyPressMs_ != 0 &&
      millis() - keyPressMs_ >= WEB_LONG_PRESS_MS) {
    keyLongFired_ = true;
  }
}

void Buttons::handleBoot() {
  bool raw = digitalRead(PREVIEW_BUTTON_PIN);
  if (raw != bootRaw_) { bootRaw_ = raw; bootChangedMs_ = millis(); }
  if (millis() - bootChangedMs_ > BUTTON_DEBOUNCE_MS && raw != bootStable_) {
    bootStable_ = raw;
    if (bootStable_ == LOW) { bootPressedMs_ = millis(); holdWake(); }
    else {
      uint32_t press = millis() - bootPressedMs_;
      if (bootPressedMs_ != 0 && press <= 1200) showPreview();
      bootPressedMs_ = 0;
    }
  }
}

// ============================================================================
//  DEFINICJE METOD — Settings (NVS + diagnostyka resetu)
// ============================================================================
void Settings::load() {
  app_.alarm().setDefaults();
  if (!prefs.begin(NVS_NS, false)) {
    app_.ntp().setCreds(DEFAULT_WIFI_SSID, DEFAULT_WIFI_PASS);
    return;
  }
  if (prefs.getBytesLength("alarms") == app_.alarm().rawSize())
    prefs.getBytes("alarms", app_.alarm().raw(), app_.alarm().rawSize());
  app_.alarm().sanitizeAll();
  String ssid = prefs.getString("ssid", DEFAULT_WIFI_SSID);
  String pass = prefs.getString("pass", DEFAULT_WIFI_PASS);
  if (ssid.length() == 0) { ssid = DEFAULT_WIFI_SSID; pass = DEFAULT_WIFI_PASS; }
  app_.ntp().setCreds(ssid.c_str(), pass.c_str());
#if ENABLE_RTC_AUTOTRIM
  // Wykalibrowany offset przezywa utrate zasilania w NVS; brak klucza = start
  // z fabrycznego RTC_OFFSET_VALUE. Wczytany PRZED ensureOffsetCalibration().
  app_.rtc().setTargetOffsetLsb((int8_t)prefs.getChar("rtcOffLsb", decodeRtcOffset(RTC_OFFSET_VALUE)));
#endif
  prefs.end();
}

#if ENABLE_RTC_AUTOTRIM
void Settings::saveRtcOffset(int8_t lsb) {
  if (!prefs.begin(NVS_NS, false)) return;
  prefs.putChar("rtcOffLsb", lsb);
  prefs.end();
}
#endif

void Settings::save() {
  if (!prefs.begin(NVS_NS, false)) return;
  prefs.putBytes("alarms", app_.alarm().raw(), app_.alarm().rawSize());
  prefs.putString("ssid", app_.ntp().ssid());
  prefs.putString("pass", app_.ntp().pass());
  prefs.end();
}

#if ENABLE_DIAGNOSTICS
// Wczytaj dozywotnie liczniki resetow i — jesli ten rozruch to brownout/panic/WDT
// — zwieksz odpowiedni. Zdarzenia rzadkie, wiec zapis do flash bezpieczny.
void Settings::recordResetReason(esp_reset_reason_t r) {
  if (!prefs.begin(NVS_NS, false)) return;
  brownoutResets_ = prefs.getUInt("rstBrown", 0);
  wdtResets_      = prefs.getUInt("rstWdt",   0);
  panicResets_    = prefs.getUInt("rstPanic", 0);
  if (r == ESP_RST_BROWNOUT)                                                   prefs.putUInt("rstBrown", ++brownoutResets_);
  else if (r == ESP_RST_TASK_WDT || r == ESP_RST_INT_WDT || r == ESP_RST_WDT) prefs.putUInt("rstWdt",   ++wdtResets_);
  else if (r == ESP_RST_PANIC)                                                 prefs.putUInt("rstPanic", ++panicResets_);
  prefs.end();
}
#endif

// ============================================================================
//  DEFINICJE METOD — App (orkiestracja setup/loop)
// ============================================================================
void App::setup() {
  // Przyczyne wybudzenia ustalamy NAJPIERW — decyduje o delay(300) (tylko zimny
  // start) oraz o akcji po wybudzeniu.
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  uint64_t ext1WakePins = (cause == ESP_SLEEP_WAKEUP_EXT1) ? esp_sleep_get_ext1_wakeup_status() : 0;
  bool coldBoot = (cause == ESP_SLEEP_WAKEUP_UNDEFINED);
  bool keyWake = coldBoot || cause == ESP_SLEEP_WAKEUP_EXT0 || ((ext1WakePins & (1ULL << KEY_PIN)) != 0);
  bool previewWake = (ext1WakePins & (1ULL << PREVIEW_BUTTON_PIN)) != 0;
  bool rtcWake = (ext1WakePins & (1ULL << RTC_INT_PIN)) != 0;
  bool userWake = keyWake || previewWake;
  bool scheduledWake = rtcWake || cause == ESP_SLEEP_WAKEUP_TIMER;

  Serial.begin(115200);
  if (coldBoot) delay(300);            // delay tylko na zimnym starcie (USB-CDC)
  setCpuFrequencyMhz(DAY_CPU_MHZ);
  watchdogSetup();
  power().releaseNightHolds();
  pinMode(PA_ENABLE_PIN, OUTPUT);
  digitalWrite(PA_ENABLE_PIN, LOW);
  display().setup(coldBoot);

  if (coldBoot) {                       // RTC RAM przezywa deep-sleep -> zeruj tylko na zimnym starcie
    alarm().resetTriggerState();
    g_persist.ntpStartedDay = -1;
    g_persist.ntpMorningDay = -1;
    g_persist.batteryHasReading = false; g_persist.batteryPercent = 0; g_persist.batteryVoltage = 0.0f;
    g_persist.drain.valid = false;
    g_persist.lastSummaryDay = -1;
    g_persist.wifiHint = false;
    g_persist.bootCount = 0;
    g_persist.coldBootEpoch = 0;
    g_persist.lastRtcDriftSec = 0;
    g_persist.lastNtpSyncEpoch = 0;
    g_persist.prevNtpSyncEpoch = 0;
#if ENABLE_RTC_AUTOTRIM
    g_persist.rtcPhaseAligned = false;
    g_persist.lastDriftPpm = 0.0f;
    g_persist.lastDriftWindowSec = 0;
    g_persist.autotrimAdjustCount = 0;
#endif
  }
  g_persist.bootCount++;

  setenv("TZ", TZ_POLAND, 1);
  tzset();

  pinMode(RTC_INT_PIN, INPUT_PULLUP);
  buttons().begin();

  settings().load();
#if ENABLE_DIAGNOSTICS
  settings().recordResetReason(esp_reset_reason());
#endif
  Serial.printf("\n=== %s v%s  (build %s %s) ===\n", FW_NAME, FW_VERSION, FW_BUILD_DATE, FW_BUILD_TIME);
  Serial.printf("Rozruch #%u, przyczyna resetu: %s\n", g_persist.bootCount, resetReasonText(esp_reset_reason()));
  battery().update(true);

  bool timeFromRtc = rtc().readToSystem();
  rtc().ensureOffsetCalibration();        // kalibracja dryfu (0x02) + odczyt zwrotny
  bool rtcAlarmCleared = rtc().disableWakeAlarm();
  setNtpStatus(timeFromRtc ? "czas z RTC" : "RTC pusty/brak");
  if (rtcWake && !rtcAlarmCleared) setNtpStatus("RTC alarm: blad kasowania");
#if ENABLE_DIAGNOSTICS
  { time_t t0 = time(nullptr);
    if (g_persist.coldBootEpoch == 0 && isSupportedEpoch(t0)) g_persist.coldBootEpoch = (int64_t)t0; }
#endif

  audio().codecPowerDown();               // ES8311 startuje w stanie niskiego poboru

  if (coldBoot) fs().ensureMounted();     // FS tylko do MP3 — montuj leniwie, krotsze wybudzenie
  alarm().checkAlarms(scheduledWake ? NTP_ALARM_CATCHUP_SEC : ALARM_LATE_WINDOW_SEC);
  if (!userWake && !audio().isRinging()) power().enterNightSleepIfNeeded(true);
#if ENABLE_LOW_BATTERY_PROTECT
  if (!userWake && !audio().isRinging()) power().enterCriticalBatterySleepIfNeeded();
#endif

  // WiFi domyslnie OFF. Sync NTP: zimny start, pusty RTC, lub poranne wybudzenie
  // o 06:00 (drugie dobowe synchro obok 18:00). Po zwyklym wybudzeniu wystarcza RTC.
  bool morningSync = false;
  { struct tm t;
    if (getLocalTm(&t) && t.tm_hour == NIGHT_END_HOUR && t.tm_min < NTP_MIN_WINDOW) {
      int day = localDayKey(t);
      if (g_persist.ntpMorningDay != day) { g_persist.ntpMorningDay = day; morningSync = true; }   // raz na dobe
    }
  }
  bool willSync = (coldBoot || !timeFromRtc || morningSync);
  if (!willSync) ntp().stopWifiAndNtp();
  if (userWake) buttons().holdWake();
  if (previewWake) buttons().showPreview();
  if (keyWake && !coldBoot) buttons().primeKeyPress();   // KEY z deep-sleep: zacznij mierzyc nacisniecie
  if (willSync) {
    ntp().setRetries(timeFromRtc ? 2 : 3);   // pusty RTC: kilka szybkich prob, potem backoff
    ntp().syncNtp(false);
  }

  markDirty();
}

void App::loop() {
  watchdogFeed();
  buttons().handleKey();
  buttons().handleBoot();
  power().enterNightSleepIfNeeded(false);

  if (web().isActive()) web().service();

  audio().serviceAudio();
  audio().serviceRinging();
  // Koniec dzwieku w czasie okna WWW/podgladu: idlePowerSave nie biega (busy),
  // wiec uspienie ES8311 i zdjecie podbicia CPU robimy tu — jedno miejsce dla
  // wszystkich sciezek konca odtwarzania. Przerwy miedzy powtorkami budzika
  // chroni audioActive() (ringing_ trzyma true).
  if (audio().codecActive() && !audioActive()) audio().shutdownCodec();
  battery().update(false);
#if ENABLE_LOW_BATTERY_PROTECT
  power().enterCriticalBatterySleepIfNeeded();
#endif
  bool ntpAdjustedTime = ntp().service();
  // Okno spoznienia poszerzone o realny przestoj petli: dlugi handleClient (upload
  // MP3) ani 60 s light-sleep nie moga zgubic budzika z terminem w trakcie przestoju.
  uint32_t loopGapSec = lastAlarmCheckMs_ ? (millis() - lastAlarmCheckMs_) / 1000UL : 0;
  lastAlarmCheckMs_ = millis();
  uint32_t lateWindowSec = ntpAdjustedTime ? NTP_ALARM_CATCHUP_SEC : ALARM_LATE_WINDOW_SEC;
  if (loopGapSec + 2 > lateWindowSec) lateWindowSec = loopGapSec + 2;
  alarm().checkAlarms(lateWindowSec);
  ntp().checkSchedule();
  ntp().handleRtcWriteRetry();
  display().update();
  power().idlePowerSave();
}

// ============================================================================
//  Punkty wejscia Arduino — cala logika w App.
// ============================================================================
void setup() { app.setup(); }
void loop()  { app.loop(); }
