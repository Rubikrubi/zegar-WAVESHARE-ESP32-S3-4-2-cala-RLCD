// ============================================================================
//  Zegar NTP RLCD v6.0 — 28.06.2026 — czysta architektura (rewrite od zera)
//  Sprzet: Waveshare ESP32-S3-RLCD-4.2 (ST7305 refleksyjny 400x300 landscape,
//          ES8311 audio codec, PCF85063A RTC). Zasilanie: 1x Samsung INR18650-35E.
//
//  Arduino IDE: plytka "ESP32S3 Dev Module", Flash 8MB, partycja "8M with spiffs".
//  Pliki obok .ino: photo_font.h  (+ MP3 budzikow na partycji danych: /alarm1..3.mp3)
//
//  Biblioteki (dokladne nazwy + wersje):
//    - U8g2                 by oliver    >= 2.35.x  (ST7305 400x300 HW-SPI)
//    - ESP32 Arduino core   3.x          (esp_task_wdt v3, esp_adc/*, esp_sntp)
//    - ESP32-audioI2S       by schreibfaul1  3.4.5  (MP3 -> ES8311 przez I2S)
//    Wbudowane w core: WiFi, WebServer, Preferences, Wire, SPI, FS/SPIFFS/LittleFS/FFat.
//
//  PRIORYTETY (w kolejnosci): 1) dokladnosc czasu, 2) czas pracy na baterii,
//  3) stabilnosc/odpornosc na bledy WiFi/NTP/RTC, 4) czytelnosc i rozwoj.
//
//  OGRANICZENIA DOKLADNOSCI (uczciwie — nie udajemy dokladnosci, ktorej sprzet
//  nie ma):
//    - NTP daje ms w chwili sync, ale RTC ustawiamy z dokladnoscia do GRANICY
//      SEKUNDY (writeRtc wyrownuje faze). Absolutny blad ~+-0.5..1 s przy sync.
//    - Miedzy synchronizacjami liczy PCF85063A. Po sprzetowej korekcji offsetu
//      (rejestr 0x02 = 0x7B, -5 LSB ~ -21.7 ppm) resztkowy dryf jest rzedu
//      <1 s/dobe (zalezny od temperatury — NIE deklarujemy sztywnej liczby).
//    - Ekran glowny pokazuje HH:MM (bez sekund), wiec dryf sub-minutowy jest
//      niewidoczny; przewrot minuty wyrownany do sekundy RTC przy kazdym wybudzeniu.
//    => Praktycznie zegar trzyma sie w granicach ~1 s przez cala dobe. Brak PPS
//       w tym sprzecie => sub-sekundowej dyscypliny nie symulujemy.
//
//  MODEL ENERGII (skrot): podswietlenia brak (panel refleksyjny) => 0 mA bazy.
//    DZIEN  = light-sleep do granicy minuty (panel trzyma obraz, setup() sie nie
//             powtarza -> brak mrugania), ~1 mA srednio.
//    NOC 23:59-06:00 = deep-sleep do 06:00 / najblizszego budzika, ekran zgaszony
//             (refleksyjny LCD bez podswietlenia i tak nieczytelny w ciemnosci).
//    ECO  <=15% = wstrzymanie auto-WiFi/NTP (najwiekszy pobor); zegar z RTC.
//    KRYT <=3.05 V = ochronny deep-sleep (ochrona ogniwa + FLASH przed brownoutem).
//    WiFi tylko na czas synchronizacji/panelu WWW, potem WIFI_OFF. BT nigdy nieuzywany.
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

// Deklaracje wyprzedzajace struktur uzywanych w sygnaturach funkcji PRZED ich
// pierwsza definicja (Arduino auto-generuje prototypy i wstawia je tuz przed
// pierwsza definicja funkcji). Bez nich kompilator zglasza "not declared".
struct AlarmConfig;
struct DrainState;

// ============================================================================
//  [0] WERSJA, BUILD INFO, FLAGI FUNKCJI
// ============================================================================
#define FW_NAME        "Zegar NTP RLCD"
#define FW_VERSION     "6.0"
#define FW_BUILD_DATE  __DATE__
#define FW_BUILD_TIME  __TIME__

// 1=wlaczone, 0=wylaczone (#if usuwa kod z kompilacji -> zero narzutu gdy off).
#define ENABLE_WATCHDOG            1   // sprzetowy reset po zawieszeniu petli (TWDT)
#define ENABLE_LOW_BATTERY_PROTECT 1   // ochrona ogniwa: degradacja + ochronny sen
#define ENABLE_DIAGNOSTICS         1   // strona /info + ekranowa DIAGNOSTYKA, liczniki resetow

// ============================================================================
//  Watchdog zadania (TWDT)
//  Timeout MUSI byc > najdluzszy light-sleep dzienny (60 s). W light-sleep licznik
//  TWDT zwykle stoi (APB bramkowany), ale margines gwarantuje brak falszywego
//  resetu niezaleznie od wersji rdzenia. Realne zawieszenie petli i tak zostaje
//  wylapane. Dlugie operacje blokujace (upload MP3, laczenie WiFi) jawnie karmia
//  watchdog. Deep-sleep RESETUJE chip -> TWDT startuje od nowa w setup().
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

// Detektor brownout: ESP32-S3 ma sprzetowy brownout-detector wlaczany przez rdzen
// (~2.51 V). CELOWO zostaje WLACZONY — przy zalamaniu napiecia natychmiast resetuje
// uklad, chroniac FLASH przed uszkodzeniem w trakcie zapisu. Progu nie zmieniamy
// (brak API w szkicu .ino). Programowa ochrona ogniwa usypia juz przy ~3.05 V,
// duzo powyzej progu sprzetowego. Kazdy reset brownout jest zliczany (/info).

// ============================================================================
//  power_runtime — estymator zuzycia baterii. Czysta arytmetyka na probkach SoC,
//  bez zaleznosci od sprzetu. Dane cosmetyczne dla /info i strony DIAGNOSTYKA.
// ============================================================================
#ifndef DRAIN_CAP_MAH
#define DRAIN_CAP_MAH        3400
#endif
#define DRAIN_CHARGE_HYST    5         // wzrost SoC% liczony jako "doladowano" -> reset kotwicy
#define DRAIN_MIN_WINDOW_SEC 43200     // min. 12 h danych (plaskie plateau Li-ion = szum ADC; dluzsze okno usrednia)
#define DRAIN_MIN_DSOC       2         // min. spadek SoC%, by estymata miala sens (ponizej = szum)

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
// ZNAK OFFSETU (datasheet NXP PCF85063A rev.7 sek. 8.2.3 + Linux rtc-pcf85063.c):
//   offset DODATNI spowalnia zegar (wydluza sekunde); offset UJEMNY przyspiesza.
//   Offset[LSB] = blad_ppm / 4.34;  1 ppm = 0.0864 s/dobe.
// Zmierzony surowy dryft -2 s/24h (krysztal ZA WOLNY ~-23.1 ppm). Korekcja:
//   -23.1/4.34 = -5.3 -> -5 LSB = -21.7 ppm; reszta ~-1.4 ppm ~ -0.12 s/24h.
//   -5 w U2 na 7 bitach, MODE=0 (bit7) = 0x7B. Weryfikacja odczytem zwrotnym.
#define RTC_OFFSET_REG            0x02
#define RTC_OFFSET_VALUE          0x7B    // -5 LSB, MODE=0 (-21.7 ppm; przyspiesza zegar)
#define RTC_OFFSET_LSB_PPM_MODE0  4.34f
#define RTC_OFFSET_LSB_PPM_MODE1  4.069f
#define RTC_OFFSET_RETRIES        4
#define PPM_SEC_PER_DAY           0.0864f

// --- Piny: wyswietlacz RLCD (HARDWARE SPI; SPI.begin w setupDisplay) ----------
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
//  LOW: degradacja — wstrzymaj auto-NTP/WiFi (najwiekszy pobor); zegar dalej chodzi.
//  CRITICAL: ochronny deep-sleep — nie rozladowac Li-ion ponizej bezpiecznego
//  progu i nie wywolac brownoutu. Progi mierzone w spoczynku (OCV).
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
constexpr uint32_t NTP_PACKET_TIMEOUT_MS      = 50000;  // lwIP SNTP probuje 3 serwery co ~15 s -> 50 s pokrywa wszystkie
constexpr uint32_t NTP_ALARM_CATCHUP_SEC      = 90;

// Backoff dla nieudanego auto-NTP (zamiast wiecznego retry co 5 min). Po serii
// szybkich ponowien (cold boot) przechodzi w backoff rosnacy 1 min -> 1 h, by
// przy trwalym braku WiFi NIE palic radia bez konca. Reset po sukcesie.
constexpr uint32_t NTP_QUICK_RETRY_MS         = 30000;        // 30 s — seria szybka po starcie
constexpr uint32_t NTP_BACKOFF_MIN_MS         = 60000;        // 1 min
constexpr uint32_t NTP_BACKOFF_MAX_MS         = 3600000;      // 1 h

constexpr int      NTP_HOUR                   = 18;  // codzienna synchronizacja 18:00-18:05 (+ dogonienie do 23:00)
constexpr int      NTP_MIN_WINDOW             = 5;
constexpr uint32_t RTC_WRITE_RETRY_MS         = 5000;
constexpr uint8_t  RTC_WRITE_RETRIES          = 12;
constexpr int      RTC_WAKE_MAX_SKEW_SEC      = 30;

constexpr uint32_t DAY_CPU_MHZ                = 160;
constexpr uint32_t NIGHT_CPU_MHZ              = 40;
#define POWER_DOWN_FLASH_IN_SLEEP 0   // 1=odlacz flash (VDD_SDIO) w light-sleep -> najwiekszy spadek pradu bazowego.
                                      //   NIETESTOWANE na tej plytce (na innej = reboot/niestabilnosc). Zostaje 0.
                                      //   Przyszla optymalizacja: zmierzyc na sprzecie zanim wlaczysz.
#define FAST_NTP_RECONNECT 1          // 1=uzyj zapamietanego kanalu+BSSID (krocej radia)
#define ST7305_STATIC_LPM 0           // 0=WYLACZONE. Tryb LPM panelu — nieprzetestowana komenda; nie wysylamy bez testu.

constexpr uint64_t IDLE_LIGHT_SLEEP_US        = 900000ULL;    // fallback gdy czas nieznany
constexpr uint64_t IDLE_LIGHT_SLEEP_MAX_SEC   = 60ULL;        // sen do granicy minuty -> 1 wybudzenie/min
constexpr int      NIGHT_START_HOUR           = 23;
constexpr int      NIGHT_START_MIN            = 59;
constexpr int      NIGHT_END_HOUR             = 6;
constexpr int      NIGHT_END_MIN              = 0;
constexpr uint32_t NIGHT_MIN_SLEEP_SEC        = 5;
constexpr uint32_t NIGHT_FALLBACK_CHUNK_SEC   = 3600;         // maks. odcinek snu gdy alarm RTC nie uzbroil sie (budzenie timerem RC)
constexpr uint32_t NIGHT_FALLBACK_ALARM_CHUNK_SEC = 300;      // jw. gdy najblizszy jest budzik: krocej -> dryft RC << okno wylapania

constexpr uint32_t BUTTON_DEBOUNCE_MS         = 40;
constexpr uint32_t WEB_LONG_PRESS_MS          = 1500;         // KEY przytrzymany >= tyle -> panel WWW
constexpr uint32_t ALARM_PREVIEW_MS           = 5000;
constexpr uint32_t WAKE_HOLD_MS               = 15000;        // po wybudzeniu przyciskiem w nocy: pokaz tresc przez 15 s zanim znow uspisz

// --- Stale konfiguracyjne (TZ, NTP, WiFi, dzwieki) ----------------------------
static const char *TZ_POLAND    = "CET-1CEST,M3.5.0/2,M10.5.0/3";
static const char *NTP_SERVER_1 = "pool.ntp.org";
static const char *NTP_SERVER_2 = "time.google.com";
static const char *NTP_SERVER_3 = "time.cloudflare.com";

// Dane WiFi wpisane na stale (jawne zyczenie uzytkownika). Nadpisywalne z panelu
// WWW (zapis w NVS). Panel nie ma logowania -> haslo nie jest renderowane w HTML.
static const char *DEFAULT_WIFI_SSID = "";
static const char *DEFAULT_WIFI_PASS = "";

static const char *SOUND_FILES[SOUND_COUNT] = { "/alarm1.mp3", "/alarm2.mp3", "/alarm3.mp3" };
static const char *SOUND_NAMES[SOUND_COUNT] = { "Dzwiek 1", "Dzwiek 2", "Dzwiek 3" };

// ============================================================================
//  Sterownik kodeka ES8311 (sekwencja rejestrow — port z es8311.c)
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

  // Pelne uspienie: wycisz DAC i wprowadz uklad w reset (bloki analogowe off) —
  // najwiekszy odbiornik w spoczynku. begin() niezawodnie go budzi.
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

// ============================================================================
//  [2] MODEL DANYCH, OBIEKTY GLOBALNE, STAN
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

// Ekran pokazuje strone ZEGARA; PODGLAD i DZWONI/NOC/KRYT to stany nadrzedne
// sterowane zdarzeniem. Strona DIAGNOSTYKA na ekranie usunieta — komplet informacji
// dostepny w panelu WWW (/info + strona glowna).

// RST = U8X8_PIN_NONE: U8g2 nigdy sam nie pulsuje resetu — sterujemy nim recznie
// w setupDisplay() przy KAZDYM wejsciu w setup (zimny start ORAZ deep-sleep, bo ten
// egzemplarz ST7305 nie trzyma obrazu przez deep-sleep). W dzien petla uzywa
// light-sleep -> setup() sie nie powtarza -> panel trzyma obraz -> brak mrugania.
static U8G2_ST7305_300X400_F_4W_HW_SPI display(
  U8G2_R1, RLCD_CS_PIN, RLCD_DC_PIN, U8X8_PIN_NONE);
static U8G2 *gfx = &display;

static WebServer   server(80);
static Preferences prefs;
static Audio       audio;
static ES8311Codec codec;

static adc_oneshot_unit_handle_t batteryAdc  = nullptr;
static adc_cali_handle_t         batteryCali = nullptr;

static AlarmConfig alarms[ALARM_COUNT];
static char wifiSsid[33] = "";
static char wifiPass[65] = "";

static bool     webActive = false;
static bool     webRoutesInstalled = false;
static uint32_t webUntilMs = 0;

static bool        fsMounted = false;
static fs::FS     *dataFS = nullptr;
static const char *dataFSName = "brak";
static File        mp3UploadFile;
static bool        mp3UploadAccepted = false;
static bool        mp3UploadHadData = false;
static String      mp3UploadPath = "";
static String      mp3UploadTempPath = "";

static bool   audioReady = false;
static bool   batteryReady = false;
static bool   batteryCaliReady = false;
static bool   displayDirty = true;

static bool   playing = false;
static String statusText = "IDLE";
static String ntpStatus = "never";

static uint32_t wakeHoldUntilMs = 0;       // po wybudzeniu przyciskiem: utrzymaj wybudzenie chwile (noc)

// RTC RAM (przezywa deep-sleep noc/krytyka; zerowane na cold boot).
static RTC_DATA_ATTR uint8_t batteryPercent;
static RTC_DATA_ATTR float   batteryVoltage;
static RTC_DATA_ATTR bool    batteryHasReading;
static RTC_DATA_ATTR DrainState g_drain;
static RTC_DATA_ATTR int        g_lastSummaryDay;
static RTC_DATA_ATTR uint8_t g_wifiBssid[6];
static RTC_DATA_ATTR int32_t g_wifiChannel;
static RTC_DATA_ATTR bool    g_wifiHint;

// Diagnostyka — RTC RAM (jeden cykl pracy ogniwa), zerowana na cold boot.
static RTC_DATA_ATTR uint16_t g_bootCount;
static RTC_DATA_ATTR int64_t  g_coldBootEpoch;
static RTC_DATA_ATTR int32_t  g_lastRtcDriftSec;
static RTC_DATA_ATTR int64_t  g_lastNtpSyncEpoch;
static RTC_DATA_ATTR int64_t  g_prevNtpSyncEpoch;
// Kalibracja offsetu RTC (ustawiane przy kazdym starcie w rtcEnsureOffsetCalibration).
static uint8_t g_rtcOffsetRaw      = 0xFF;
static int8_t  g_rtcOffsetLsb      = 0;
static float   g_rtcOffsetPpm      = 0.0f;
static bool    g_rtcOffsetVerified = false;
// Dozywotnie liczniki "zlych" resetow — w NVS (zdarzenia rzadkie).
static uint32_t g_brownoutResets = 0;
static uint32_t g_wdtResets      = 0;
static uint32_t g_panicResets    = 0;

// Stan "dzwonienia" budzika.
static bool     ringing = false;
static int      ringingAlarm = -1;
static uint32_t ringStartMs = 0;
static uint32_t ringLastRetryMs = 0;
static uint8_t  ringErrors = 0;
static uint32_t soundStartedMs = 0;

static int lastDrawMinute = -1;
static int lastDrawDay = -1;
static uint32_t lastClockSig = 0xFFFFFFFFu;   // sygnatura widocznej tresci (sentinel = wymus repaint)

// Dedup wyzwolen budzikow — przezywa deep-sleep, zerowany na cold boot.
static RTC_DATA_ATTR int lastTriggerDay[ALARM_COUNT];
// Znacznik dnia dobowej synchronizacji NTP 10:00 (RTC RAM -> wybudzenie z
// deep-sleep nie wywoluje powtornego sync tego samego dnia).
static RTC_DATA_ATTR int ntpStartedDay;
// Znacznik dnia porannego NTP po wybudzeniu o 06:00 (drugie dobowe synchro).
static RTC_DATA_ATTR int ntpMorningDay;

static uint8_t  ntpRetriesLeft = 0;
static uint32_t ntpNextRetryMs = 0;
static uint32_t ntpBackoffMs   = NTP_BACKOFF_MIN_MS;
static uint8_t  rtcWriteRetriesLeft = 0;
static uint32_t rtcWriteNextRetryMs = 0;
static uint32_t batteryNextReadMs = 0;

static bool     keyRaw = HIGH, keyStable = HIGH;
static uint32_t keyChangedMs = 0, keyPressMs = 0;
static bool     keyLongFired = false;
static bool     bootRaw = HIGH, bootStable = HIGH;
static uint32_t bootChangedMs = 0, bootPressedMs = 0;
static bool     previewActive = false;
static uint32_t previewUntilMs = 0;

static const gpio_num_t HOLD_PINS[] = {
  (gpio_num_t)RLCD_DC_PIN, (gpio_num_t)RLCD_CS_PIN, (gpio_num_t)RLCD_SCK_PIN,
  (gpio_num_t)RLCD_MOSI_PIN, (gpio_num_t)RLCD_RST_PIN
};

// Prototypy (funkcje uzywane przed definicja).
static void stopPlayback(bool userStopped);
static bool startSound(uint8_t soundIndex, uint8_t volume);
static void startRinging(int alarmIndex);
static bool ensureCodec();
static void shutdownCodec();
static void updateBattery(bool force);
static bool syncNtp(bool manual);
static bool serviceNtp();
static void checkAlarms(uint32_t lateWindowSec = ALARM_LATE_WINDOW_SEC);
static void enterNightSleepIfNeeded(bool force);
static void showPreview();
static bool startWebWindow();
static void restoreTimeAfterFailedNtp();
static void drawCriticalBatteryPage();
#if ENABLE_LOW_BATTERY_PROTECT
static bool batteryLow();
static bool batteryCritical();
static void enterCriticalBatterySleepIfNeeded();
#endif

// ============================================================================
//  [3] CZAS — konwersje cywilne i RTC PCF85063A
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

static bool rtcBeginSession() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);
  Wire.beginTransmission(PCF85063A_ADDR);
  return Wire.endTransmission() == 0;
}

static bool rtcWriteReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(PCF85063A_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

// Odczyt n rejestrow od 'reg' jednym transferem. Wolajacy juz otworzyl sesje.
static bool rtcReadRegs(uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(PCF85063A_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)PCF85063A_ADDR, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

static bool rtcReadControl1(uint8_t *control1) {
  if (!control1 || !rtcBeginSession()) return false;
  return rtcReadRegs(0x00, control1, 1);
}
static bool rtcReadControl2(uint8_t *control2) {
  if (!control2 || !rtcBeginSession()) return false;
  return rtcReadRegs(0x01, control2, 1);
}
static bool rtcWriteControl1(uint8_t control1) { return rtcWriteReg(0x00, control1); }

static bool rtcReleaseClock(uint8_t runningControl1) {
  for (uint8_t attempt = 0; attempt < 3; attempt++) {
    if (rtcWriteControl1(runningControl1)) return true;
    delay(2);
  }
  return false;
}

// Zatrzymaj licznik przed ustawieniem czasu. STOP zeruje glowna czesc preskalera
// -> pierwszy impuls sekundowy ma okreslona faze.
static bool rtcPrepareTimeWrite(uint8_t *runningControl1) {
  if (!runningControl1) return false;
  uint8_t control1;
  if (!rtcReadControl1(&control1)) return false;
  *runningControl1 = control1 & (uint8_t)~((1U << 7) | (1U << 5) | (1U << 2) | (1U << 1));
  if (rtcWriteControl1(*runningControl1 | (1U << 5))) return true;
  rtcReleaseClock(*runningControl1);
  return false;
}

static bool rtcHasCurrentFormat() {
  uint8_t marker;
  return rtcReadRegs(0x03, &marker, 1) && marker == RTC_FORMAT_MARKER;
}
static bool rtcStoreCurrentFormat()      { return rtcWriteReg(0x03, RTC_FORMAT_MARKER); }
static bool rtcInvalidateCurrentFormat() { return rtcWriteReg(0x03, 0); }

// Odczyt 7 rejestrow czasu (0x04..0x0A) jednym transferem -> brak rwania na granicy
// sekundy. Rok 00..99 = 2000..2099 (zgodnosc z liczeniem lat przestepnych w PCF85063A).
static bool readRtcEpochInternal(time_t *epoch, bool requireCurrentFormat) {
  if (!epoch) return false;
  uint8_t control1;
  if (!rtcReadControl1(&control1)) return false;

  // Nie wznawiaj po cichu zatrzymanego/przestawionego RTC (staly czas != aktualny).
  const uint8_t invalidModeMask = (1U << 7) | (1U << 5) | (1U << 1);
  if ((control1 & invalidModeMask) != 0 ||
      (requireCurrentFormat && !rtcHasCurrentFormat())) return false;

  uint8_t t[7];
  if (!rtcReadRegs(0x04, t, 7)) return false;
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

static bool readRtcEpoch(time_t *epoch) { return readRtcEpochInternal(epoch, true); }

static bool readRtcToSystem() {
  for (uint8_t attempt = 0; attempt < 3; attempt++) {
    time_t epoch;
    if (readRtcEpoch(&epoch)) { setSystemEpoch(epoch); return true; }
    delay(2);
  }
  return false;
}

// Zapis czasu UTC do RTC z WYROWNANIEM do granicy sekundy. Po zwolnieniu STOP
// preskaler startuje od zera, wiec wpisana sekunda "trzyma sie" ~1.0 s -> faza
// przewrotu = chwila zwolnienia STOP. Bez tego RTC bywa stale spozniony 0.5-1 s,
// co po godzinach wyglada jak dryft. Pomijamy wyrownanie gdy gra audio/budzik
// (busy-wait do ~1 s przerwalby audio.loop()).
static bool writeRtc(time_t epoch) {
  if (!isSupportedEpoch(epoch)) return false;

  struct timeval tv;
  gettimeofday(&tv, nullptr);
  time_t writeEpoch = epoch;
  bool   alignToBoundary = false;
  if (!playing && !ringing &&
      isSupportedEpoch(tv.tv_sec) && (tv.tv_sec == epoch || tv.tv_sec + 1 == epoch)) {
    writeEpoch = tv.tv_sec + 1;          // najblizsza nadchodzaca pelna sekunda
    alignToBoundary = true;
  }

  struct tm gm;
  gmtime_r(&writeEpoch, &gm);
  int year = gm.tm_year + 1900;

  // Najpierw uniewaznij dane: reset w dowolnym dalszym miejscu nie ujawni
  // niepelnego zapisu jako poprawnego czasu.
  if (!rtcBeginSession()) return false;
  if (!rtcInvalidateCurrentFormat()) return false;
  uint8_t runningControl1;
  if (!rtcPrepareTimeWrite(&runningControl1)) return false;
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
  if (Wire.endTransmission() != 0) { rtcReleaseClock(runningControl1); return false; }

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
  if (!rtcReleaseClock(runningControl1)) return false;

  time_t verifiedEpoch;
  if (!readRtcEpochInternal(&verifiedEpoch, false)) return false;
  long long diff = (long long)verifiedEpoch - (long long)writeEpoch;
  if (diff < -2 || diff > 2) return false;
  if (rtcStoreCurrentFormat() && rtcHasCurrentFormat()) return true;
  rtcInvalidateCurrentFormat();
  return false;
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
// Sugerowana DOCELOWA wartosc offsetu po zmierzeniu dryfu (do /info — recznie):
//   delta_LSB = round( (driftSecPerDay / 0.0864) / 4.34 ); zwraca currentLsb + delta.
static int suggestedTotalOffsetLsb(float driftSecPerDay, int currentLsb) {
  float lsbF  = (driftSecPerDay / PPM_SEC_PER_DAY) / RTC_OFFSET_LSB_PPM_MODE0;
  int   delta = (int)(lsbF + (lsbF >= 0 ? 0.5f : -0.5f));
  int   total = currentLsb + delta;
  if (total > 63)  total = 63;
  if (total < -64) total = -64;
  return total;
}

// Kalibracja sprzetowa dryfu (rejestr 0x02) Z WERYFIKACJA odczytem zwrotnym.
// Niezalezna od rejestrow czasu i markera (writeRtc ich nie rusza), wiec wpis
// trwa przez kolejne synchronizacje. Wolana przy KAZDYM starcie: rejestr jest
// podtrzymywany bateryjnie, ale ponowny zapis+odczyt dowodzi, ze bajt wszedl.
static bool rtcEnsureOffsetCalibration() {
  g_rtcOffsetVerified = false;
  for (uint8_t attempt = 0; attempt < RTC_OFFSET_RETRIES; attempt++) {
    if (!rtcBeginSession()) { delay(2); continue; }
    rtcWriteReg(RTC_OFFSET_REG, RTC_OFFSET_VALUE);
    uint8_t raw;
    if (rtcReadRegs(RTC_OFFSET_REG, &raw, 1)) {
      g_rtcOffsetRaw = raw;
      g_rtcOffsetLsb = decodeRtcOffset(raw);
      g_rtcOffsetPpm = rtcOffsetPpm(g_rtcOffsetLsb, (raw & 0x80) != 0);
      if (raw == RTC_OFFSET_VALUE) { g_rtcOffsetVerified = true; return true; }
    }
    delay(2);
  }
  return false;
}

// INT sluzy wylacznie alarmowi wybudzajacemu. Wylacz korekcje, MI/HMI i countdown
// timer, skasuj flagi AF/TF, wylacz CLKOUT (mniejszy pobor RTC).
static bool rtcConfigureWakeInterrupt(bool alarmEnabled) {
  uint8_t control1;
  if (!rtcReadControl1(&control1)) return false;
  control1 &= (uint8_t)~(1U << 2);                   // CIE=0
  if (!rtcWriteReg(0x00, control1)) return false;
  if (!rtcWriteReg(0x11, 0x18)) return false;        // Timer_mode: TCF=1/60 Hz, TE=0, TIE=0
  if (!rtcWriteReg(0x10, 0)) return false;           // Timer_value
  uint8_t control2;
  if (!rtcReadControl2(&control2)) return false;
  control2 = (uint8_t)(0x07 | (alarmEnabled ? (1U << 7) : 0));   // COF=111 (CLKOUT off), AIE
  return rtcWriteReg(0x01, control2);
}

static bool rtcVerifyWakeInterruptConfig(bool alarmEnabled) {
  uint8_t control1, control2;
  if (!rtcReadControl1(&control1) || !rtcReadControl2(&control2)) return false;
  if (!rtcBeginSession()) return false;
  uint8_t timer[2];
  if (!rtcReadRegs(0x10, timer, 2)) return false;
  uint8_t expectedControl2 = alarmEnabled ? (1U << 7) : 0;
  return (control1 & (1U << 2)) == 0 &&
         (control2 & 0xF8) == expectedControl2 &&
         timer[0] == 0 && timer[1] == 0x18;
}

static bool rtcVerifyWakeAlarm(time_t epoch) {
  struct tm gm;
  gmtime_r(&epoch, &gm);
  if (!rtcBeginSession()) return false;
  uint8_t a[5];
  if (!rtcReadRegs(0x0B, a, 5)) return false;
  if (a[0] != (dec2bcd((uint8_t)gm.tm_sec) & 0x7F) ||
      a[1] != (dec2bcd((uint8_t)gm.tm_min) & 0x7F) ||
      a[2] != (dec2bcd((uint8_t)gm.tm_hour) & 0x3F) ||
      a[3] != (dec2bcd((uint8_t)gm.tm_mday) & 0x3F) ||
      a[4] != 0x80) return false;
  return rtcVerifyWakeInterruptConfig(true);
}

// Alarm PCF85063A pracuje w UTC (jak rejestry czasu). INT aktywne stanem niskim,
// niskie do skasowania flagi AF.
static bool rtcSetWakeAlarm(time_t epoch) {
  time_t systemNow = time(nullptr);
  if (!isSupportedEpoch(epoch) || !isSupportedEpoch(systemNow)) return false;
  time_t rtcNow;
  if (!readRtcEpoch(&rtcNow)) return false;     // odrzuca STOP/12h
  long long rtcSkew = (long long)rtcNow - (long long)systemNow;
  if (rtcSkew < -RTC_WAKE_MAX_SKEW_SEC || rtcSkew > RTC_WAKE_MAX_SKEW_SEC) return false;
  if (!rtcConfigureWakeInterrupt(false)) return false;

  struct tm gm;
  gmtime_r(&epoch, &gm);
  if (!rtcBeginSession()) return false;
  Wire.beginTransmission(PCF85063A_ADDR);
  Wire.write(0x0B);                               // Second_alarm..
  Wire.write(dec2bcd((uint8_t)gm.tm_sec) & 0x7F); // AEN=0 -> aktywny
  Wire.write(dec2bcd((uint8_t)gm.tm_min) & 0x7F);
  Wire.write(dec2bcd((uint8_t)gm.tm_hour) & 0x3F);
  Wire.write(dec2bcd((uint8_t)gm.tm_mday) & 0x3F);
  Wire.write(0x80);                               // Weekday_alarm AEN=1 (wylaczony)
  if (Wire.endTransmission() != 0) return false;

  if (!rtcConfigureWakeInterrupt(true)) return false;
  return rtcVerifyWakeAlarm(epoch);
}

static bool rtcDisableWakeAlarm() {
  return rtcConfigureWakeInterrupt(false) && rtcVerifyWakeInterruptConfig(false);
}

// ============================================================================
//  [4] USTAWIENIA (NVS) + DIAGNOSTYKA RESETU
// ============================================================================
static const char *NVS_NS = "zegar7";   // przestrzen nazw Preferences

static void setDefaultAlarms() {
  for (int i = 0; i < ALARM_COUNT; i++) {
    alarms[i].enabled = 0;
    alarms[i].hour = 7; alarms[i].minute = 0; alarms[i].second = 0;
    alarms[i].volume = DEFAULT_AUDIO_VOLUME;
    alarms[i].sound = (uint8_t)i;
    snprintf(alarms[i].label, sizeof(alarms[i].label), "Budzik %d", i + 1);
  }
}

static void resetTriggerState() {
  for (int i = 0; i < ALARM_COUNT; i++) lastTriggerDay[i] = -1;
}

static void sanitizeAlarm(AlarmConfig &alarm, int index) {
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

static void sanitizeAlarms() {
  for (int i = 0; i < ALARM_COUNT; i++) sanitizeAlarm(alarms[i], i);
}

static void loadSettings() {
  setDefaultAlarms();
  if (!prefs.begin(NVS_NS, false)) {
    strncpy(wifiSsid, DEFAULT_WIFI_SSID, sizeof(wifiSsid) - 1);
    strncpy(wifiPass, DEFAULT_WIFI_PASS, sizeof(wifiPass) - 1);
    wifiSsid[sizeof(wifiSsid) - 1] = 0; wifiPass[sizeof(wifiPass) - 1] = 0;
    return;
  }
  if (prefs.getBytesLength("alarms") == sizeof(alarms))
    prefs.getBytes("alarms", alarms, sizeof(alarms));
  sanitizeAlarms();
  String ssid = prefs.getString("ssid", DEFAULT_WIFI_SSID);
  String pass = prefs.getString("pass", DEFAULT_WIFI_PASS);
  if (ssid.length() == 0) { ssid = DEFAULT_WIFI_SSID; pass = DEFAULT_WIFI_PASS; }
  ssid.toCharArray(wifiSsid, sizeof(wifiSsid));
  pass.toCharArray(wifiPass, sizeof(wifiPass));
  prefs.end();
}

static void saveSettings() {
  if (!prefs.begin(NVS_NS, false)) return;
  prefs.putBytes("alarms", alarms, sizeof(alarms));
  prefs.putString("ssid", wifiSsid);
  prefs.putString("pass", wifiPass);
  prefs.end();
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

// Wczytaj dozywotnie liczniki resetow i — jesli ten rozruch to brownout/panic/WDT
// — zwieksz odpowiedni. Zdarzenia rzadkie, wiec zapis do flash bezpieczny.
static void recordResetReason(esp_reset_reason_t r) {
  if (!prefs.begin(NVS_NS, false)) return;
  g_brownoutResets = prefs.getUInt("rstBrown", 0);
  g_wdtResets      = prefs.getUInt("rstWdt",   0);
  g_panicResets    = prefs.getUInt("rstPanic", 0);
  if (r == ESP_RST_BROWNOUT)                                          prefs.putUInt("rstBrown", ++g_brownoutResets);
  else if (r == ESP_RST_TASK_WDT || r == ESP_RST_INT_WDT || r == ESP_RST_WDT) prefs.putUInt("rstWdt", ++g_wdtResets);
  else if (r == ESP_RST_PANIC)                                       prefs.putUInt("rstPanic", ++g_panicResets);
  prefs.end();
}

// ============================================================================
//  [5] WiFi + NTP — nieblokujaca maszyna stanow
//  Czas ustawiany z RTC pierwszy; NTP NADPISUJE tylko po REALNYM pakiecie
//  (gate atomic odroznia odpowiedz serwera od czasu wstepnie ustawionego z RTC).
//  WiFi wlaczane tylko na czas proby, potem WIFI_OFF. Nieudany sync = rollback.
// ============================================================================
static std::atomic<bool> g_ntpGotPacket{false};
static std::atomic<bool> g_ntpAcceptPacket{false};
static void onSntpSync(struct timeval *tv) {
  if (tv && g_ntpAcceptPacket.load(std::memory_order_acquire))
    g_ntpGotPacket.store(true, std::memory_order_release);
}

enum NtpSyncState : uint8_t { NTP_SYNC_IDLE, NTP_SYNC_WIFI_CONNECTING, NTP_SYNC_WAITING_PACKET };
static NtpSyncState ntpSyncState = NTP_SYNC_IDLE;
static bool         ntpManualAttempt = false;
static bool         ntpWifiHintActive = false;
static uint32_t     ntpStateStartedMs = 0;
static struct timeval ntpFallbackTime = {};   // snapshot czasu przed proba (rollback po bledzie)

static void stopSntpCallbacks() {
  g_ntpAcceptPacket.store(false, std::memory_order_release);
  if (esp_sntp_enabled()) esp_sntp_stop();
  sntp_set_time_sync_notification_cb(nullptr);
}

// Rollback po nieudanym NTP: przywroc czas sprzed proby (cala sekunda). Cala proba
// trwa <=65 s -> nie korygujemy ulamka (zbedna zlozonosc; sync i tak wroci w dobe).
static void restoreTimeAfterFailedNtp() {
  if (isSupportedEpoch(ntpFallbackTime.tv_sec)) settimeofday(&ntpFallbackTime, nullptr);
  else setSystemEpoch(0);
}

static void stopWifiAndNtp() {
  bool wasAttempting = ntpSyncState != NTP_SYNC_IDLE;   // przerywamy trwajaca probe (WIFI_CONNECTING lub WAITING_PACKET)
  if (ntpSyncState == NTP_SYNC_WAITING_PACKET) restoreTimeAfterFailedNtp();
  stopSntpCallbacks();
  ntpSyncState = NTP_SYNC_IDLE;
  ntpManualAttempt = false;
  ntpWifiHintActive = false;
  g_ntpGotPacket.store(false, std::memory_order_release);
  // Przerwano probe, brak waznego czasu i brak zaplanowanego ponowienia -> zaplanuj
  // jedno (backoff), by zegar w koncu zdobyl czas mimo przerwania (np. zamkniecie WWW).
  if (wasAttempting && ntpRetriesLeft == 0 && !isSupportedEpoch(time(nullptr))) {
    ntpRetriesLeft = 1;
    ntpNextRetryMs = millis() + ntpBackoffMs;
  }
  WiFi.disconnect(false, false);
  WiFi.mode(WIFI_OFF);
}

static bool wifiBeginSmart() {
  WiFi.persistent(false);
#if FAST_NTP_RECONNECT
  if (g_wifiHint) { WiFi.begin(wifiSsid, wifiPass, g_wifiChannel, g_wifiBssid); return true; }
#endif
  WiFi.begin(wifiSsid, wifiPass);
  return false;
}

static void wifiRetryFullScan() {
#if FAST_NTP_RECONNECT
  g_wifiHint = false;
#endif
  WiFi.disconnect(false, false);
  WiFi.begin(wifiSsid, wifiPass);
}

static void wifiCacheHint() {
#if FAST_NTP_RECONNECT
  if (WiFi.status() == WL_CONNECTED) {
    g_wifiChannel = WiFi.channel();
    const uint8_t *b = WiFi.BSSID();
    if (b) { for (int i = 0; i < 6; i++) g_wifiBssid[i] = b[i]; g_wifiHint = true; }
  }
#endif
}

// Blokujace laczenie WiFi (tylko sciezka panelu WWW; NTP laczy sie nieblokujaco).
static bool ensureWifi(uint32_t timeoutMs) {
  if (strlen(wifiSsid) == 0) return false;
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
  if (ok) wifiCacheHint(); else g_wifiHint = false;
  return ok;
}

static void scheduleRtcWriteRetry(uint8_t retries = RTC_WRITE_RETRIES) {
  rtcWriteRetriesLeft = retries;
  rtcWriteNextRetryMs = millis() + RTC_WRITE_RETRY_MS;
}

static void handleRtcWriteRetry() {
  if (rtcWriteRetriesLeft == 0 || (int32_t)(millis() - rtcWriteNextRetryMs) < 0) return;
  if (writeRtc(time(nullptr))) {
    rtcWriteRetriesLeft = 0;
    ntpStatus = "NTP OK, RTC OK";
  } else if (--rtcWriteRetriesLeft > 0) {
    rtcWriteNextRetryMs = millis() + RTC_WRITE_RETRY_MS;
    ntpStatus = "NTP OK, RTC retry";
  } else {
    ntpStatus = "NTP OK, RTC ERR";
  }
  displayDirty = true;
}

static void finishNtpAttempt(bool ok) {
  bool wasManual = ntpManualAttempt;
  ntpSyncState = NTP_SYNC_IDLE;
  ntpManualAttempt = false;
  ntpWifiHintActive = false;
  stopSntpCallbacks();
  g_ntpGotPacket.store(false, std::memory_order_release);

  if (ok) {
    ntpRetriesLeft = 0;
    ntpBackoffMs = NTP_BACKOFF_MIN_MS;          // reset backoffu po sukcesie
  } else if (!wasManual) {
    if (ntpRetriesLeft > 0) {
      ntpNextRetryMs = millis() + NTP_QUICK_RETRY_MS;   // dokoncz serie szybka
    } else if (!isSupportedEpoch(time(nullptr))) {
      // Brak waznego czasu -> ponawiaj z BACKOFFEM (1 min -> 1 h), by przy trwalym
      // braku WiFi nie palic radia bez konca. Gdy czas wazny: brak retry, dogoni
      // nastepne okno dobowe.
      ntpRetriesLeft = 1;
      ntpNextRetryMs = millis() + ntpBackoffMs;
      ntpBackoffMs = (ntpBackoffMs >= NTP_BACKOFF_MAX_MS / 2) ? NTP_BACKOFF_MAX_MS : ntpBackoffMs * 2;
    }
  }
  if (!webActive) { WiFi.disconnect(false, false); WiFi.mode(WIFI_OFF); }
  displayDirty = true;
}

static void beginNtpRequest() {
  g_ntpAcceptPacket.store(false, std::memory_order_release);
  g_ntpGotPacket.store(false, std::memory_order_release);
  sntp_set_time_sync_notification_cb(onSntpSync);   // PRZED configTzTime
  sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED);
  gettimeofday(&ntpFallbackTime, nullptr);
  g_ntpAcceptPacket.store(true, std::memory_order_release);
  configTzTime(TZ_POLAND, NTP_SERVER_1, NTP_SERVER_2, NTP_SERVER_3);
  ntpSyncState = NTP_SYNC_WAITING_PACKET;
  ntpStateStartedMs = millis();
  ntpStatus = "oczekiwanie na NTP";
  displayDirty = true;
}

static bool syncNtp(bool manual) {
  if (ntpSyncState != NTP_SYNC_IDLE) return false;
  if (manual) ntpRetriesLeft = 0;
  ntpManualAttempt = manual;
  ntpStatus = manual ? "manual: laczenie WiFi" : "auto: laczenie WiFi";
  displayDirty = true;

  if (strlen(wifiSsid) == 0) { ntpStatus = "brak SSID WiFi"; finishNtpAttempt(false); return false; }

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(true);
  if (WiFi.status() == WL_CONNECTED) {
    beginNtpRequest();
  } else {
    ntpWifiHintActive = wifiBeginSmart();
    ntpSyncState = NTP_SYNC_WIFI_CONNECTING;
    ntpStateStartedMs = millis();
  }
  return true;
}

// Wolane co petle. Zwraca true gdy NTP wlasnie zatwierdzil nowy czas (dla okna
// budzika). Nigdy nie blokuje petli.
static bool serviceNtp() {
  if (ntpSyncState == NTP_SYNC_WIFI_CONNECTING) {
    if (WiFi.status() == WL_CONNECTED) {
      wifiCacheHint();
      beginNtpRequest();
    } else if (ntpWifiHintActive && millis() - ntpStateStartedMs >= WIFI_HINT_FALLBACK_MS) {
      ntpWifiHintActive = false;
      ntpStatus = "WiFi: pelny skan";
      wifiRetryFullScan();
      ntpStateStartedMs = millis();
      displayDirty = true;
    } else if (millis() - ntpStateStartedMs >= NTP_WIFI_TIMEOUT_MS) {
      ntpStatus = "blad WiFi";
      g_wifiHint = false;
      finishNtpAttempt(false);
    }
    return false;
  }

  if (ntpSyncState != NTP_SYNC_WAITING_PACKET) return false;

  if (g_ntpGotPacket.load(std::memory_order_acquire)) {
    stopSntpCallbacks();                       // zablokuj kolejny pakiet w trakcie zatwierdzania
    time_t syncedEpoch = time(nullptr);
    if (!isSupportedEpoch(syncedEpoch)) {
      restoreTimeAfterFailedNtp();
      ntpStatus = "bledny czas NTP";
      finishNtpAttempt(false);
      return false;
    }
    struct tm local = {};
    localtime_r(&syncedEpoch, &local);

#if ENABLE_DIAGNOSTICS
    { // Dryft: porownaj STARY czas RTC (jeszcze nienadpisany) z czasem NTP.
      time_t oldRtc;
      if (readRtcEpoch(&oldRtc)) g_lastRtcDriftSec = (int32_t)(oldRtc - syncedEpoch);
      g_prevNtpSyncEpoch = g_lastNtpSyncEpoch;
      g_lastNtpSyncEpoch = (int64_t)syncedEpoch;
    }
#else
    g_lastNtpSyncEpoch = (int64_t)syncedEpoch;
#endif

    bool rtcOk = writeRtc(syncedEpoch);
    if (rtcOk) rtcWriteRetriesLeft = 0; else scheduleRtcWriteRetry();
    char buf[48];
    snprintf(buf, sizeof(buf), "OK %02d:%02d:%02d RTC:%s",
             local.tm_hour, local.tm_min, local.tm_sec, rtcOk ? "OK" : "RETRY");
    ntpStatus = buf;
    ntpStartedDay = localDayKey(local);
    finishNtpAttempt(true);
    return true;
  }

  if (millis() - ntpStateStartedMs >= NTP_PACKET_TIMEOUT_MS) {
    stopSntpCallbacks();
    restoreTimeAfterFailedNtp();
    ntpStatus = "blad NTP";
    finishNtpAttempt(false);
  }
  return false;
}

static bool handleNtpRetry() {
  if (ntpSyncState != NTP_SYNC_IDLE) return true;
  if (ntpRetriesLeft == 0) return false;
  if ((int32_t)(millis() - ntpNextRetryMs) < 0) return true;
  --ntpRetriesLeft;
  syncNtp(false);
  return true;
}

// ============================================================================
//  System plikow (MP3 budzikow) — montowany leniwie
// ============================================================================
static bool recoverSoundFilesOnce() {
  File root = dataFS->open("/");
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
      bool changed = dataFS->exists(target) ? dataFS->remove(path) : dataFS->rename(path, target);
      if (changed) { root.close(); return true; }
    } else if (lower.endsWith(".part") && path.length() > 6) {
      String target = path.substring(0, path.length() - 5);
      String backup = target + ".bak";
      if (dataFS->exists(backup) && !dataFS->exists(target)) continue;
      if (dataFS->remove(path)) { root.close(); return true; }
    }
  }
  root.close();
  return false;
}

static void recoverSoundFiles() {
  if (!dataFS) return;
  for (uint8_t pass = 0; pass < 16 && recoverSoundFilesOnce(); pass++) {}
}

static bool ensureFsMounted() {
  if (fsMounted && dataFS) return true;
  struct { fs::FS *fs; const char *name; bool ok; } cands[] = {
    { &SPIFFS,   "SPIFFS",   SPIFFS.begin(false) },
    { &LittleFS, "LittleFS", false },
    { &FFat,     "FFat",     false },
  };
  if (!cands[0].ok) cands[1].ok = LittleFS.begin(false);
  if (!cands[0].ok && !cands[1].ok) cands[2].ok = FFat.begin(false);
  for (auto &c : cands) {
    if (c.ok) {
      fsMounted = true; dataFS = c.fs; dataFSName = c.name;
      recoverSoundFiles();
      Serial.printf("System plikow: %s\n", c.name);
      return true;
    }
  }
  // Fabrycznie czysty flash: MP3 mozna wgrac tylko przez WWW (wymaga FS) -> pat.
  // Jednorazowy format SPIFFS go przerywa; nic sie nie zamontowalo = brak danych.
  static bool formatTried = false;
  if (!formatTried) {
    formatTried = true;
    Serial.println("Brak FS - formatuje SPIFFS...");
    if (SPIFFS.begin(true)) {
      fsMounted = true; dataFS = &SPIFFS; dataFSName = "SPIFFS";
      Serial.println("System plikow: SPIFFS (nowo sformatowany)");
      return true;
    }
  }
  fsMounted = false; dataFS = nullptr; dataFSName = "brak";
  Serial.println("Brak systemu plikow z MP3");
  return false;
}

// ============================================================================
//  [6] AUDIO + BUDZIKI (ES8311 + MP3 przez I2S)
// ============================================================================
static void audioInfo(Audio::msg_t m) {
  Serial.printf("Audio %s: %s\n", m.s ? m.s : "", m.msg ? m.msg : "");
}

static bool audioPinsReady = false;

static void ensureAudioPins() {
  if (audioPinsReady) return;
  Audio::audio_info_callback = audioInfo;
  audio.setPinout(I2S_BCLK_PIN, I2S_WS_PIN, I2S_DOUT_PIN, I2S_MCLK_PIN);
  audio.setOutput48KHz(true);
  audio.setVolume(DEFAULT_AUDIO_VOLUME);
  audioPinsReady = true;
}

static bool ensureCodec() {
  if (audioReady) return true;
  pinMode(PA_ENABLE_PIN, OUTPUT);
  digitalWrite(PA_ENABLE_PIN, LOW);   // PA wlacza dopiero startSound() PO konfiguracji kodeka (bez trzasku)
  ensureAudioPins();
  audioReady = codec.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000);
  if (audioReady) { codec.setVolume(CODEC_VOLUME_PERCENT); codec.configBits16(); }
  else { Serial.println("ES8311 init failed"); digitalWrite(PA_ENABLE_PIN, LOW); }
  return audioReady;
}

// Uspij kodek po odtwarzaniu — ES8311 jest na stalej szynie 3V3 (snu CPU go nie
// wylacza), wiec to najwiekszy odbiornik w spoczynku. startSound() go wskrzesza.
static void shutdownCodec() {
  if (!audioReady) { digitalWrite(PA_ENABLE_PIN, LOW); return; }
  audio.stopSong();
  codec.powerDown();
  digitalWrite(PA_ENABLE_PIN, LOW);
  audioReady = false;
}

static bool startSound(uint8_t soundIndex, uint8_t volume) {
  stopPlayback(false);
  if (soundIndex >= SOUND_COUNT) soundIndex = 0;
  if (!ensureFsMounted() || !ensureCodec()) { statusText = "blad FS/kodek"; displayDirty = true; return false; }
  const char *path = SOUND_FILES[soundIndex];
  if (!dataFS->exists(path)) { shutdownCodec(); statusText = String("brak ") + path; displayDirty = true; return false; }

  digitalWrite(PA_ENABLE_PIN, HIGH);
  audio.stopSong();
  audio.setVolume(constrain(volume, 0, MAX_AUDIO_VOLUME));
  codec.setVolume(CODEC_VOLUME_PERCENT);
  if (!audio.connecttoFS(*dataFS, path)) { shutdownCodec(); statusText = String("blad odtw. ") + path; displayDirty = true; return false; }

  soundStartedMs = millis();
  playing = true;
  Serial.printf("Odtwarzam %s vol=%u\n", path, volume);
  displayDirty = true;
  return true;
}

static void stopPlayback(bool userStopped) {
  if (userStopped) { ringing = false; ringingAlarm = -1; ringErrors = 0; }
  if (audioReady) {
    audio.stopSong();
    audio.setVolume(0);
    codec.setVolume(0);
    digitalWrite(PA_ENABLE_PIN, LOW);
  }
  if (playing) { statusText = userStopped ? "STOP" : "koniec"; playing = false; displayDirty = true; }
}

static void startRinging(int alarmIndex) {
  ringing = true;
  ringingAlarm = alarmIndex;
  ringStartMs = millis();
  ringLastRetryMs = millis();
  ringErrors = 0;
  statusText = String("BUDZI: ") + alarms[alarmIndex].label;
  displayDirty = true;
  if (!startSound(alarms[alarmIndex].sound, alarms[alarmIndex].volume)) {
    ringErrors = 1;
    ringLastRetryMs = millis() - RING_RETRY_MS;
  }
}

static void serviceAudio() {
  if (!playing) return;
  if (!audioReady) {
    playing = false;
    digitalWrite(PA_ENABLE_PIN, LOW);
    if (!ringing) statusText = "audio stop";
    displayDirty = true;
    return;
  }
  audio.loop();
  if (!audio.isRunning()) {
    if (ringing) {
      // connecttoFS() zwraca true dla KAZDEGO istniejacego pliku; uszkodzony MP3
      // "konczy sie" po kilkudziesieciu ms. Bez zliczania budzik krecil sie cicho.
      playing = false;
      if (millis() - soundStartedMs < RING_MIN_PLAY_MS) {
        if (++ringErrors >= RING_MAX_ERRORS) { statusText = "budzik: blad MP3"; ringing = false; ringingAlarm = -1; }
      } else ringErrors = 0;
      displayDirty = true;
      return;
    }
    stopPlayback(false);
  }
}

static void serviceRinging() {
  if (!ringing) return;
  if (millis() - ringStartMs > RING_TIMEOUT_MS) {
    statusText = "budzik: timeout"; stopPlayback(false); ringing = false; ringingAlarm = -1; displayDirty = true; return;
  }
  if (playing) return;
  if (millis() - ringLastRetryMs < RING_RETRY_MS) return;
  ringLastRetryMs = millis();
  if (ringingAlarm >= 0 && startSound(alarms[ringingAlarm].sound, alarms[ringingAlarm].volume)) return;
  if (++ringErrors >= RING_MAX_ERRORS) { statusText = "budzik: blad MP3"; ringing = false; ringingAlarm = -1; displayDirty = true; }
}

// ============================================================================
//  [7] BATERIA — ADC, krzywa OCV->SoC, estymator, ochrona
// ============================================================================
static bool initBattery() {
  if (batteryReady) return true;
  adc_oneshot_unit_init_cfg_t initCfg = {};
  initCfg.unit_id = ADC_UNIT_1;
  if (adc_oneshot_new_unit(&initCfg, &batteryAdc) != ESP_OK) return false;

  adc_oneshot_chan_cfg_t chanCfg = {};
  chanCfg.bitwidth = ADC_BITWIDTH_12;
  chanCfg.atten = ADC_ATTEN_DB_12;
  if (adc_oneshot_config_channel(batteryAdc, BATTERY_ADC_CHANNEL, &chanCfg) != ESP_OK) {
    adc_oneshot_del_unit(batteryAdc); batteryAdc = nullptr; return false;
  }
  adc_cali_curve_fitting_config_t caliCfg = {};
  caliCfg.unit_id = ADC_UNIT_1;
  caliCfg.atten = ADC_ATTEN_DB_12;
  caliCfg.bitwidth = ADC_BITWIDTH_12;
  batteryCaliReady = adc_cali_create_scheme_curve_fitting(&caliCfg, &batteryCali) == ESP_OK;
  batteryReady = true;
  return true;
}

// Krzywa OCV->SoC dla INR18650-35E (Li-ion NMC), 3.00 V=0%, 4.20 V=100%.
// Rozladowanie NIELINIOWE: plaski srodek 3.70-3.91 V to ~43-71% SoC; konce strome.
// Interpolacja liniowa miedzy punktami. Koszt: kilka porownan co 10 min -> zero
// dodatkowego pradu (brak licznika kulombow / rezystora pomiarowego).
static uint8_t percentFromVoltage(float v) {
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

static uint8_t stabilizePercent(uint8_t measured) {
  if (measured >= BATTERY_FULL_FLOOR) return 100;
  if (batteryHasReading && batteryPercent == 100 && measured >= BATTERY_FULL_HOLD_FLOOR) return 100;
  return measured;
}

// Zwiezle podsumowanie zuzycia do NVS max raz/dobe (nadpisywane, malo flash).
static void maybeWriteDailySummary(time_t now) {
  struct tm t; localtime_r(&now, &t);
  int day = localDayKey(t);
  if (day == g_lastSummaryDay) return;
  float perDay = drainMahPerDay(g_drain);
  if (perDay < 0.0f) return;
  g_lastSummaryDay = day;                      // oznacz dzien NIM sprobujesz zapisu (martwy NVS nie ma byc ponawiany co petle)
  if (!prefs.begin(NVS_NS, false)) return;
  prefs.putUShort("drnSoc", batteryPercent);
  prefs.putFloat("drnMahDay", perDay);
  prefs.putUInt("drnDay", (uint32_t)day);
  prefs.end();
  Serial.printf("Zuzycie: %.1f mAh/dzien, ~%.0f dni\n", perDay, drainDaysRemaining(g_drain, batteryPercent));
}

static void updateBattery(bool force) {
  if (!force && (int32_t)(millis() - batteryNextReadMs) < 0) return;

  // Pod obciazeniem (audio, dzwoniacy budzik, serwer web) napiecie zacisku siada
  // przez rezystancje wewnetrzna ogniwa -> pomiar zanizony. Odlozyc do bezczynnosci.
  if (!force && (playing || ringing || webActive)) { batteryNextReadMs = millis() + BATTERY_BUSY_RETRY_MS; return; }

  batteryNextReadMs = millis() + BATTERY_READ_INTERVAL_MS;
  if (!initBattery()) { batteryNextReadMs = millis() + BATTERY_BUSY_RETRY_MS; return; }

  uint32_t sum = 0; uint8_t samples = 0;
  for (uint8_t i = 0; i < BATTERY_SAMPLE_COUNT; i++) {
    int raw = 0;
    if (adc_oneshot_read(batteryAdc, BATTERY_ADC_CHANNEL, &raw) == ESP_OK) { sum += (uint32_t)raw; samples++; }
  }
  if (samples == 0) { batteryNextReadMs = millis() + BATTERY_BUSY_RETRY_MS; return; }

  int raw = (int)((sum + samples / 2) / samples);
  int mv = 0;
  if (batteryCaliReady) adc_cali_raw_to_voltage(batteryCali, raw, &mv);
  else mv = (raw * 3300) / 4095;

  float measured = (mv * 0.001f) * BATTERY_DIVIDER_RATIO * BATTERY_VOLTAGE_CALIBRATION;
  if (!batteryHasReading || force) { batteryVoltage = measured; batteryHasReading = true; }
  else batteryVoltage = batteryVoltage * 0.80f + measured * 0.20f;   // EMA

  uint8_t old = batteryPercent;
  batteryPercent = stabilizePercent(percentFromVoltage(batteryVoltage));
  time_t nowT = time(nullptr);
  if (isSupportedEpoch(nowT)) {
    drainOnSample(g_drain, (int64_t)nowT, batteryPercent);
    maybeWriteDailySummary(nowT);
#if ENABLE_DIAGNOSTICS
    if (g_coldBootEpoch == 0) g_coldBootEpoch = (int64_t)nowT;
#endif
  }
  if (force || old != batteryPercent) displayDirty = true;
}

#if ENABLE_LOW_BATTERY_PROTECT
static bool batteryLow() { return batteryHasReading && batteryPercent <= BATTERY_LOW_PERCENT; }
// updateBattery() bramkuje pomiar pod obciazeniem, wiec batteryVoltage ~ OCV.
static bool batteryCritical() {
  return batteryHasReading &&
         (batteryVoltage <= BATTERY_CRITICAL_VOLTAGE || batteryPercent <= BATTERY_CRITICAL_PERCENT);
}
#endif

// ============================================================================
//  [8] RYSOWANIE (U8g2) — strony i ikony
// ============================================================================
static void drawPhotoGlyph(int x, int y, int glyph, bool colon) {
  uint16_t p = colon ? PHOTO_COLON_OFFSET : pgm_read_word(&PHOTO_DIGIT_OFFSET[glyph]);
  for (int row = 0; row < PHOTO_DIGIT_H; row++) {
    uint8_t segs = pgm_read_byte(&PHOTO_FONT_RLE[p++]);
    for (uint8_t i = 0; i < segs; i++) {
      uint8_t start = pgm_read_byte(&PHOTO_FONT_RLE[p++]);
      uint8_t width = pgm_read_byte(&PHOTO_FONT_RLE[p++]);
      gfx->drawHLine(x + start, y + row, width);
    }
  }
  // Font (Impact, generowany) ma skrocona dolna stopke cyfry 4 — uzupelniamy ja
  // tu. UWAGA: te 2 prostokaty sa zwiazane z PHOTO_DIGIT_W/H; zmiana fontu wymaga
  // ich rewizji. Regeneracja kompletnego glifu 4 (eliminacja tej latki) wymaga
  // narzedzia generujacego photo_font.h, ktorego tu nie ma -> latka zostaje,
  // zamknieta w jednym miejscu i opisana.
  if (!colon && glyph == 4) {
    gfx->drawBox(x + 39, y + 112, 41, 13);
    gfx->drawBox(x + 40, y + 125, 31, 25);
  }
}

static void drawLargeTime(uint8_t hour, uint8_t minute, int y) {
  int totalW = PHOTO_DIGIT_W * 4 + PHOTO_COLON_W + PHOTO_DIGIT_GAP * 4;
  int x = (LCD_WIDTH - totalW) / 2 + PHOTO_TIME_X_SHIFT;
  drawPhotoGlyph(x, y, hour / 10, false);   x += PHOTO_DIGIT_W + PHOTO_DIGIT_GAP;
  drawPhotoGlyph(x, y, hour % 10, false);   x += PHOTO_DIGIT_W + PHOTO_DIGIT_GAP;
  drawPhotoGlyph(x + PHOTO_COLON_X_SHIFT, y + PHOTO_COLON_Y_SHIFT, 0, true);
  x += PHOTO_COLON_W + PHOTO_DIGIT_GAP;
  drawPhotoGlyph(x, y, minute / 10, false); x += PHOTO_DIGIT_W + PHOTO_DIGIT_GAP;
  drawPhotoGlyph(x, y, minute % 10, false);
}

static void drawBellIcon(int x, int y, bool active) {
  gfx->setDrawColor(1);
  gfx->drawDisc(x + 12, y + 10, 8);
  gfx->drawBox(x + 5, y + 10, 15, 11);
  gfx->drawBox(x + 3, y + 20, 19, 4);
  gfx->drawDisc(x + 12, y + 25, 3);
  gfx->drawBox(x + 9, y + 3, 7, 4);
  if (!active) {
    gfx->drawLine(x + 1, y + 26, x + 25, y + 2);
    gfx->drawLine(x + 2, y + 26, x + 26, y + 2);
  }
}

static void drawBatteryIcon(int x, int y, uint8_t percent) {
  if (percent > 100) percent = 100;
  gfx->drawFrame(x, y, 28, 13);
  gfx->drawBox(x + 28, y + 4, 3, 5);
  int fill = map(percent, 0, 100, 0, 24);
  if (fill > 0) gfx->drawBox(x + 2, y + 2, fill, 9);
  char txt[8];
  snprintf(txt, sizeof(txt), "%u%%", percent);
  gfx->setFont(u8g2_font_7x14_tf);
  gfx->drawStr(x - 42, y + 12, txt);
}

static void drawCentered(const char *text, int y) {
  int w = gfx->getStrWidth(text);
  int x = (LCD_WIDTH - w) / 2;
  if (x < 0) x = 0;
  gfx->drawStr(x, y, text);
}

static bool anyAlarmEnabled() {
  for (int i = 0; i < ALARM_COUNT; i++) if (alarms[i].enabled) return true;
  return false;
}

// Najblizszy wlaczony budzik (dzis/jutro). Ranking po realnym epoch (mktime), nie
// po sekundach doby — inaczej w noc zmiany czasu kolejnosc klamie.
static int nextAlarmIndex(uint8_t *h, uint8_t *mi, uint8_t *s, char *label, size_t labelSize) {
  struct tm t;
  bool hasTime = getLocalTm(&t);
  time_t now = hasTime ? time(nullptr) : 0;
  long bestDelta = LONG_MAX;
  int best = -1;
  for (int i = 0; i < ALARM_COUNT; i++) {
    if (!alarms[i].enabled) continue;
    long delta;
    if (hasTime) {
      time_t cand = epochForClock(t, alarms[i].hour, alarms[i].minute, alarms[i].second, 0);
      if (cand <= now) cand = epochForClock(t, alarms[i].hour, alarms[i].minute, alarms[i].second, 1);
      delta = (long)(cand - now);
    } else delta = i;
    if (delta < bestDelta) {
      bestDelta = delta; best = i;
      if (h) *h = alarms[i].hour;
      if (mi) *mi = alarms[i].minute;
      if (s) *s = alarms[i].second;
      if (label && labelSize) { strncpy(label, alarms[i].label, labelSize - 1); label[labelSize - 1] = 0; }
    }
  }
  return best;
}

// Sygnatura WIDOCZNEJ tresci strony zegara (HH:MM, %aku, budzik, WWW). Pola
// niepokazywane (ntpStatus/statusText) NIE wchodza -> zero pelnych sendBuffer()
// identycznej klatki na panelu refleksyjnym (zero mrugniec).
static uint32_t clockSig(const struct tm &t, int day) {
  uint8_t nh = 0, nm = 0;
  int next = nextAlarmIndex(&nh, &nm, nullptr, nullptr, 0);
  uint32_t h = 2166136261u;                           // FNV-1a
  auto mix = [&](uint32_t v) { h = (h ^ v) * 16777619u; };
  mix((uint32_t)t.tm_hour);
  mix((uint32_t)t.tm_min);
  mix((uint32_t)day);
  mix((uint32_t)batteryPercent);
  mix(webActive ? 1u : 0u);
  mix(anyAlarmEnabled() ? 1u : 0u);
  mix(next >= 0 ? (uint32_t)(nh * 60 + nm + 1) : 0u);
  return h;
}

static void drawClockPage(const struct tm &t) {
  uint8_t nh = 0, nm = 0;
  int next = nextAlarmIndex(&nh, &nm, nullptr, nullptr, 0);
  bool alarmOn = anyAlarmEnabled();

  gfx->clearBuffer();
  gfx->setDrawColor(1);
  drawBatteryIcon(360, 8, batteryPercent);

  int y = (LCD_HEIGHT - PHOTO_DIGIT_H) / 2 + PHOTO_TIME_Y_SHIFT;
  drawLargeTime(t.tm_hour, t.tm_min, y);

  gfx->drawHLine(0, 264, LCD_WIDTH);
  drawBellIcon(10, 270, alarmOn);
  gfx->setFont(u8g2_font_7x14_tf);
  char bottom[80];
  if (next >= 0)
    snprintf(bottom, sizeof(bottom), "Budzik %02u:%02u %s  Aku %u%%  WWW:%s",
             nh, nm, alarmOn ? "ON" : "OFF", batteryPercent, webActive ? "ON" : "OFF");
  else
    snprintf(bottom, sizeof(bottom), "Budzik --:-- OFF  Aku %u%%  WWW:%s",
             batteryPercent, webActive ? "ON" : "OFF");
  gfx->drawStr(42, 287, bottom);
  gfx->sendBuffer();
}

// --- Licznik czasu od ostatniego NTP (uzywany przez panel WWW /info) ---
static long secsSinceNtp() {
  if (g_lastNtpSyncEpoch <= 0) return -1;
  time_t now = time(nullptr);
  if (!isSupportedEpoch(now)) return -1;
  long d = (long)(now - (time_t)g_lastNtpSyncEpoch);
  return d < 0 ? 0 : d;
}

static void drawPreviewPage() {
  uint8_t nh = 0, nm = 0, ns = 0;
  char label[sizeof(alarms[0].label)] = "";
  int next = nextAlarmIndex(&nh, &nm, &ns, label, sizeof(label));

  gfx->clearBuffer();
  gfx->setDrawColor(1);
  drawBatteryIcon(360, 8, batteryPercent);
  if (next >= 0) {
    drawBellIcon(10, 8, true);
    gfx->setFont(u8g2_font_9x15_tf);
    drawCentered("NAJBLIZSZY BUDZIK", 25);
    drawLargeTime(nh, nm, 60);
    gfx->drawHLine(0, 264, LCD_WIDTH);
    char detail[96];
    snprintf(detail, sizeof(detail), "%s  %02u:%02u:%02u  [%s]",
             label, nh, nm, ns, SOUND_NAMES[alarms[next].sound % SOUND_COUNT]);
    drawCentered(detail, 287);
  } else {
    drawBellIcon(185, 35, false);
    gfx->setFont(u8g2_font_logisoso50_tf);
    drawCentered("--:--", 150);
    gfx->setFont(u8g2_font_9x15_tf);
    drawCentered("Brak aktywnego budzika", 215);
  }
  gfx->sendBuffer();
  lastClockSig = 0xFFFFFFFFu;   // powrot do zegara wymusi repaint
}

static void drawRingingPage() {
  gfx->clearBuffer();
  gfx->setDrawColor(1);
  drawBatteryIcon(360, 8, batteryPercent);
  drawBellIcon(185, 35, true);
  gfx->setFont(u8g2_font_logisoso50_tf);
  drawCentered("BUDZIK", 150);
  gfx->setFont(u8g2_font_9x15_tf);
  drawCentered(statusText.c_str(), 210);
  drawCentered("Przycisk = wycisz", 245);
  gfx->sendBuffer();
  lastClockSig = 0xFFFFFFFFu;
}

static void drawNightPage() {
  gfx->clearBuffer();
  gfx->setDrawColor(1);
  gfx->setFont(u8g2_font_7x14_tf);
  drawCentered("TRYB NOCNY 23:59 - 06:00", 150);
  gfx->sendBuffer();
  displayDirty = false;
  lastClockSig = 0xFFFFFFFFu;
}

#if ENABLE_LOW_BATTERY_PROTECT
static void drawCriticalBatteryPage() {
  gfx->clearBuffer();
  gfx->setDrawColor(1);
  drawBatteryIcon(360, 8, batteryPercent);
  gfx->setFont(u8g2_font_logisoso32_tf);
  drawCentered("ROZLADOWANY", 120);
  gfx->setFont(u8g2_font_9x15_tf);
  drawCentered("Podlacz ladowarke", 165);
  char b[40];
  snprintf(b, sizeof(b), "%u%%  %.2f V", batteryPercent, batteryVoltage);
  drawCentered(b, 200);
  gfx->sendBuffer();
  lastClockSig = 0xFFFFFFFFu;
}
#endif

static void updateDisplay() {
  if (playing || ringing) {                      // stan nadrzedny: dzwoni budzik
    if (displayDirty) { drawRingingPage(); displayDirty = false; }
    return;
  }
  if (previewActive) {                           // stan nadrzedny: podglad budzika
    if ((int32_t)(millis() - previewUntilMs) < 0) {
      if (displayDirty) { drawPreviewPage(); displayDirty = false; }
      return;
    }
    previewActive = false;
    displayDirty = true;
  }

  struct tm t;
  if (!getLocalTm(&t)) {                          // brak czasu
    if (displayDirty) {
      gfx->clearBuffer();
      gfx->setDrawColor(1);
      gfx->setFont(u8g2_font_9x15_tf);
      drawCentered("Brak czasu RTC/NTP", 135);
      drawCentered(ntpStatus.c_str(), 170);
      gfx->sendBuffer();
      displayDirty = false;
      lastClockSig = 0xFFFFFFFFu;
    }
    return;
  }

  // Strona zegara — repaint TYLKO gdy zmienia sie WIDOCZNA tresc (clockSig).
  int day = localDayKey(t);
  uint32_t sig = clockSig(t, day);
  if (sig != lastClockSig) {
    drawClockPage(t);
    lastDrawMinute = t.tm_min; lastDrawDay = day; lastClockSig = sig;
  }
  displayDirty = false;
}

// ============================================================================
//  [9] PANEL WWW (konfiguracja, sync, pliki MP3, /info) — WiFi tylko na zadanie
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
  if (name.length() < 5 || name.length() > 25) return false;   // SPIFFS: pelna sciezka <=31 znakow, upload pisze "/<nazwa>.part"
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

static void appendMp3Manager(String &html) {
  html += F("<section><b>Pliki MP3</b>");
  if (!ensureFsMounted()) { html += F("<p>Brak zamontowanego systemu plikow.</p></section>"); return; }
  html += F("<p class='muted'>System plikow: "); html += htmlEscape(dataFSName);
  html += F("</p><form method='post' action='/mp3upload' enctype='multipart/form-data'>"
            "<div class='row'><input type='file' name='mp3' accept='.mp3,audio/mpeg' required>"
            "<button type='submit'>Dodaj / nadpisz MP3</button></div></form>");
  html += F("<table><tr><th>Plik</th><th>Rozmiar</th><th></th></tr>");
  bool any = false;
  File root = dataFS->open("/");
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

static void pageHeader(String &h) {
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

static void handleRoot() {
  webUntilMs = millis() + WEB_WINDOW_MS;
  updateBattery(false);   // bez wymuszania: pod WiFi pomiar zanizony, OCV po zamknieciu WWW
  String html; html.reserve(14000);
  pageHeader(html);

  html += F("<section><b>Status</b><p>Czas: "); html += currentTimeText();
  html += F("<br>Budzik: ");     html += htmlEscape(statusText);
  html += F("<br>NTP: ");        html += htmlEscape(ntpStatus);
  html += F("<br>Pliki MP3: ");  html += htmlEscape(dataFSName);
  html += F("<br>Akumulator: "); html += batteryPercent;
  html += F("% (");              html += String(batteryVoltage, 2);
  html += F(" V, ok. ");         html += String((uint32_t)batteryPercent * BATTERY_CAPACITY_MAH / 100U);
  html += F(" mAh)<br>IP: ");    html += WiFi.localIP().toString();
  {
    float perDay = drainMahPerDay(g_drain);
    float daysLeft = drainDaysRemaining(g_drain, batteryPercent);
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
  html += htmlEscape(wifiSsid);
  html += F("'></label><label>Haslo <input name='pass' type='password' placeholder='(bez zmian)'>"
            "</label></div><p class='muted'>Puste pole hasla = haslo bez zmian.</p></section>");

  html += F("<section><b>Budziki</b><table>"
            "<tr><th>Wl.</th><th>Czas</th><th>Glos.</th><th>Dzwiek</th><th>Nazwa</th></tr>");
  for (int i = 0; i < ALARM_COUNT; i++) {
    char timeBuf[16];
    snprintf(timeBuf, sizeof(timeBuf), "%02u:%02u:%02u", alarms[i].hour, alarms[i].minute, alarms[i].second);
    html += F("<tr><td><input type='checkbox' name='en"); html += i; html += F("' ");
    if (alarms[i].enabled) html += F("checked");
    html += F("></td><td><input type='time' step='1' name='time"); html += i;
    html += F("' value='"); html += timeBuf;
    html += F("'></td><td><input type='number' min='0' max='21' name='vol"); html += i;
    html += F("' value='"); html += alarms[i].volume;
    html += F("'></td><td><select name='snd"); html += i; html += F("'>");
    for (int sIdx = 0; sIdx < SOUND_COUNT; sIdx++) {
      html += F("<option value='"); html += sIdx; html += F("'");
      if (alarms[i].sound == sIdx) html += F(" selected");
      html += F(">"); html += SOUND_NAMES[sIdx]; html += F("</option>");
    }
    html += F("</select></td><td><input maxlength='15' name='label"); html += i;
    html += F("' value='"); html += htmlEscape(alarms[i].label);
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

static void handleSave() {
  webUntilMs = millis() + WEB_WINDOW_MS;
  if (server.hasArg("ssid")) { String ssid = server.arg("ssid"); ssid.trim(); ssid.toCharArray(wifiSsid, sizeof(wifiSsid)); }
  if (server.hasArg("pass") && server.arg("pass").length() > 0) { String pass = server.arg("pass"); pass.toCharArray(wifiPass, sizeof(wifiPass)); }

  for (int i = 0; i < ALARM_COUNT; i++) {
    String idx = String(i);
    uint8_t prevEnabled = alarms[i].enabled;
    uint8_t prevH = alarms[i].hour, prevM = alarms[i].minute, prevS = alarms[i].second;
    alarms[i].enabled = server.hasArg(String("en") + idx) ? 1 : 0;
    uint8_t h, m, s;
    if (parseClock(server.arg(String("time") + idx), &h, &m, &s)) { alarms[i].hour = h; alarms[i].minute = m; alarms[i].second = s; }
    bool timeChanged = (alarms[i].hour != prevH || alarms[i].minute != prevM || alarms[i].second != prevS);
    bool reEnabled   = (alarms[i].enabled && !prevEnabled);
    if (timeChanged || reEnabled) lastTriggerDay[i] = -1;   // ponowne uzbrojenie: moze odpalic dzis ponownie
    String volumeArg = String("vol") + idx, soundArg = String("snd") + idx, labelArg = String("label") + idx;
    if (server.hasArg(volumeArg)) alarms[i].volume = constrain(server.arg(volumeArg).toInt(), 0, MAX_AUDIO_VOLUME);
    if (server.hasArg(soundArg))  alarms[i].sound  = constrain(server.arg(soundArg).toInt(), 0, SOUND_COUNT - 1);
    if (server.hasArg(labelArg)) {
      String label = server.arg(labelArg); label.trim();
      if (label.length() == 0) label = "Budzik " + String(i + 1);
      label.toCharArray(alarms[i].label, sizeof(alarms[i].label));
    }
    sanitizeAlarm(alarms[i], i);
  }
  saveSettings();
  displayDirty = true;
  server.sendHeader("Location", "/");
  server.send(303, "text/plain", "");
}

static void handleSync() { syncNtp(true); server.sendHeader("Location", "/"); server.send(303, "text/plain", ""); }
static void handleStop() { stopPlayback(true); server.sendHeader("Location", "/"); server.send(303, "text/plain", ""); }

static void handleTest() {
  // Nie przerywaj dzwoniacego budzika testem (startSound nie kasuje ringing).
  if (!ringing) { uint8_t sIdx = constrain(server.arg("snd").toInt(), 0, SOUND_COUNT - 1); startSound(sIdx, DEFAULT_AUDIO_VOLUME); }
  server.sendHeader("Location", "/");
  server.send(303, "text/plain", "");
}

static void handleMp3UploadDone() { server.sendHeader("Location", "/"); server.send(303, "text/plain", ""); }

static void handleMp3UploadStream() {
  webUntilMs = millis() + WEB_WINDOW_MS;
  watchdogFeed();   // callback per fragment: upload duzego MP3 blokuje loop() minutami
  HTTPUpload &upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    if (mp3UploadFile) mp3UploadFile.close();
    mp3UploadAccepted = false; mp3UploadHadData = false; mp3UploadPath = ""; mp3UploadTempPath = "";
    if (!normalizeMp3Path(upload.filename, mp3UploadPath, false)) { statusText = "zla nazwa MP3"; displayDirty = true; return; }
    if (!ensureFsMounted()) { statusText = "brak FS dla MP3"; displayDirty = true; return; }
    if (playing || ringing) stopPlayback(true);
    mp3UploadTempPath = mp3UploadPath + ".part";
    if (dataFS->exists(mp3UploadTempPath)) dataFS->remove(mp3UploadTempPath);
    mp3UploadFile = dataFS->open(mp3UploadTempPath, FILE_WRITE);
    mp3UploadAccepted = (bool)mp3UploadFile;
    statusText = mp3UploadAccepted ? String("upload ") + mp3UploadPath : "blad zapisu MP3";
    displayDirty = true;
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (mp3UploadAccepted && mp3UploadFile) {
      size_t written = mp3UploadFile.write(upload.buf, upload.currentSize);
      if (written != upload.currentSize) { mp3UploadAccepted = false; statusText = "blad zapisu MP3"; }
      else if (upload.currentSize > 0) mp3UploadHadData = true;
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (mp3UploadFile) mp3UploadFile.close();
    if (mp3UploadAccepted && mp3UploadHadData) {
      String backupPath = mp3UploadPath + ".bak";
      if (dataFS->exists(backupPath)) dataFS->remove(backupPath);
      bool hadOld = dataFS->exists(mp3UploadPath);
      bool oldMoved = !hadOld || dataFS->rename(mp3UploadPath, backupPath);
      bool newInstalled = oldMoved && dataFS->rename(mp3UploadTempPath, mp3UploadPath);
      if (newInstalled) {
        if (dataFS->exists(backupPath)) dataFS->remove(backupPath);
        statusText = String("dodano ") + mp3UploadPath;
      } else {
        if (oldMoved && hadOld) {
          if (dataFS->exists(mp3UploadPath)) dataFS->remove(mp3UploadPath);
          if (dataFS->exists(backupPath)) dataFS->rename(backupPath, mp3UploadPath);
        }
        if (dataFS->exists(mp3UploadTempPath)) dataFS->remove(mp3UploadTempPath);
        statusText = "blad instalacji MP3";
      }
    } else {
      if (mp3UploadTempPath.length() && dataFS && dataFS->exists(mp3UploadTempPath)) dataFS->remove(mp3UploadTempPath);
      if (statusText == String("upload ") + mp3UploadPath) statusText = "pusty MP3";
    }
    displayDirty = true;
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    if (mp3UploadFile) mp3UploadFile.close();
    if (mp3UploadTempPath.length() && dataFS && dataFS->exists(mp3UploadTempPath)) dataFS->remove(mp3UploadTempPath);
    statusText = "upload przerwany";
    displayDirty = true;
  }
}

static void handleMp3Delete() {
  String path;
  if (!normalizeMp3Path(server.arg("file"), path, true)) statusText = "zla nazwa MP3";
  else if (!ensureFsMounted()) statusText = "brak FS dla MP3";
  else {
    if (playing || ringing) stopPlayback(true);
    statusText = (dataFS->exists(path) && dataFS->remove(path)) ? String("usunieto ") + path : String("brak ") + path;
  }
  displayDirty = true;
  server.sendHeader("Location", "/");
  server.send(303, "text/plain", "");
}

#if ENABLE_DIAGNOSTICS
static void handleInfo() {
  webUntilMs = millis() + WEB_WINDOW_MS;
  updateBattery(false);
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
    long s = secsSinceNtp();
    row("Od ostatniego NTP", s < 0 ? String("brak synchronizacji")
        : String(s / 3600) + " h " + String((s % 3600) / 60) + " min");
  }
  row("Przyczyna ostatniego resetu", resetReasonText(esp_reset_reason()));
  row("Rozruchy od zimnego startu", String(g_bootCount));
  {
    time_t now = time(nullptr);
    if (isSupportedEpoch(now) && g_coldBootEpoch > 0) {
      long s = (long)(now - (time_t)g_coldBootEpoch); if (s < 0) s = 0;
      char buf[48];
      snprintf(buf, sizeof(buf), "%ld dni %02ld:%02ld:%02ld", s / 86400, (s % 86400) / 3600, (s % 3600) / 60, s % 60);
      row("Czas pracy (od zimnego startu)", buf);
    } else row("Czas pracy (od zimnego startu)", "ustala sie");
  }
  row("Resety brownout / WDT / panic",
      String(g_brownoutResets) + " / " + String(g_wdtResets) + " / " + String(g_panicResets));
  row("Heap wolny", String(ESP.getFreeHeap()) + " B");
  row("Heap min. od startu", String(ESP.getMinFreeHeap()) + " B");
  row("Heap najwiekszy blok", String(ESP.getMaxAllocHeap()) + " B");
  row("RTC offset (0x02)",
      "0x" + String(g_rtcOffsetRaw, HEX) + " (LSB " + String((int)g_rtcOffsetLsb) + ", " + String(g_rtcOffsetPpm, 1) + " ppm)");
  row("RTC offset status",
      g_rtcOffsetVerified ? String("OK (potwierdzony odczytem 0x02)")
        : (g_rtcOffsetRaw == 0xFF ? String("BLAD: brak odczytu rejestru 0x02")
             : String("NIEZWERYFIKOWANY: zapis 0x") + String(RTC_OFFSET_VALUE, HEX) + " != odczyt 0x" + String(g_rtcOffsetRaw, HEX)));
  if (g_lastNtpSyncEpoch > 0 && g_prevNtpSyncEpoch > 0) {
    long  winSec = (long)(g_lastNtpSyncEpoch - g_prevNtpSyncEpoch);
    float winH   = winSec / 3600.0f;
    // Dryft z rejestrow czasu PCF85063A ma rozdzielczosc 1 s -> niepewnosc +-1 s/okno.
    float driftPpm    = winSec > 0 ? (float)g_lastRtcDriftSec / winSec * 1e6f : 0.0f;
    float driftPerDay = driftPpm * PPM_SEC_PER_DAY;
    float uncPpm      = winSec > 0 ? 1.0f / winSec * 1e6f : 0.0f;
    int   suggest     = suggestedTotalOffsetLsb(driftPerDay, g_rtcOffsetLsb);
    row("Dryft RTC (ostatni sync)",
        String(g_lastRtcDriftSec) + " s / " + String(winH, 1) + " h  (" + String(driftPpm, 1) + " +-" +
        String(uncPpm, 1) + " ppm, ~" + String(driftPerDay, 2) + " s/dobe)");
    row("Sugerowany offset RTC",
        "LSB " + String(suggest) + " -> 0x" + String(encodeRtcOffset(suggest, false), HEX) +
        " (z biezacego LSB " + String((int)g_rtcOffsetLsb) + ")");
  } else row("Dryft RTC (ostatni sync)", "brak danych (min. 2 synchronizacje)");
  row("Dryft — rozdzielczosc", "1 s (PCF85063A bez rejestru pod-sekundowego; mierz kilka dob dla < 1 ppm)");
  row("Akumulator", String(batteryPercent) + "% (" + String(batteryVoltage, 2) + " V)");
#if ENABLE_LOW_BATTERY_PROTECT
  row("Ochrona ogniwa", batteryCritical() ? "KRYTYCZNY" : (batteryLow() ? "tryb oszczedny (bez auto-NTP)" : "OK"));
#endif
  if (WiFi.status() == WL_CONNECTED) { row("WiFi", WiFi.SSID() + "  RSSI " + String(WiFi.RSSI()) + " dBm"); row("IP", WiFi.localIP().toString()); }
  else row("WiFi", "rozlaczone");
  row("System plikow", dataFSName);
  html += F("</table><p><a href='/'><button>Powrot</button></a></p></section></main></body></html>");
  server.send(200, "text/html", html);
}
#endif

// Reczne wylaczenie panelu WWW: odsyla potwierdzenie, a samo zamkniecie (server.stop
// + WiFi OFF) wykonuje petla PO wyslaniu odpowiedzi — ustawiamy termin okna na "teraz",
// wiec po handleClient() warunek uplywu okna zamknie panel (jak auto-timeout 5 min).
static void handleWebOff() {
  String html; pageHeader(html);
  html += F("<section><b>Panel WWW wylaczony.</b>"
            "<p class='muted'>WiFi zostalo wylaczone dla oszczednosci baterii. "
            "Aby wlaczyc panel ponownie: przytrzymaj przycisk KEY na zegarze.</p>"
            "</section></main></body></html>");
  server.send(200, "text/html", html);
  webUntilMs = millis();   // petla wykryje uplyw okna i zamknie panel po flushu strony
}

static bool startWebWindow() {
  if (webActive) {
    if (WiFi.status() == WL_CONNECTED) { webUntilMs = millis() + WEB_WINDOW_MS; displayDirty = true; return true; }
    server.stop(); webActive = false;
  }
  if (!ensureWifi(NTP_WIFI_TIMEOUT_MS)) { statusText = "brak polaczenia WiFi"; stopWifiAndNtp(); displayDirty = true; return false; }
  if (!webRoutesInstalled) {
    server.on("/", HTTP_GET, handleRoot);
    server.on("/save", HTTP_POST, handleSave);
    server.on("/sync", HTTP_GET, handleSync);
    server.on("/stop", HTTP_GET, handleStop);
    server.on("/test", HTTP_POST, handleTest);
    server.on("/mp3upload", HTTP_POST, handleMp3UploadDone, handleMp3UploadStream);
    server.on("/mp3delete", HTTP_POST, handleMp3Delete);
    server.on("/off", HTTP_GET, handleWebOff);
#if ENABLE_DIAGNOSTICS
    server.on("/info", HTTP_GET, handleInfo);
#endif
    webRoutesInstalled = true;
  }
  server.begin();
  webActive = true;
  webUntilMs = millis() + WEB_WINDOW_MS;
  displayDirty = true;
  Serial.print("WWW IP: "); Serial.println(WiFi.localIP());
  return true;
}

static void stopWebWindow() {
  server.stop();
  webActive = false;
  // Aktywna proba NTP (do ~50 s) sama dokonczy walidacje i wylaczy WiFi.
  if (ntpSyncState == NTP_SYNC_IDLE) stopWifiAndNtp();
  displayDirty = true;
}

// ============================================================================
//  [10] BUDZIKI + HARMONOGRAM NTP (jedna synchronizacja na dobe)
// ============================================================================
static void checkAlarms(uint32_t lateWindowSec) {
  if (playing || ringing) return;
  time_t now = time(nullptr);
  struct tm t;
  if (!isSupportedEpoch(now)) return;
  localtime_r(&now, &t);
  for (int i = 0; i < ALARM_COUNT; i++) {
    if (!alarms[i].enabled) continue;
    int occurrenceDay;
    if (alarmDueNow(t, now, alarms[i].hour, alarms[i].minute, alarms[i].second, lateWindowSec, &occurrenceDay) &&
        lastTriggerDay[i] != occurrenceDay) {
      lastTriggerDay[i] = occurrenceDay;
      // Skonsumuj inne budziki wymagalne w tej samej chwili (kolizja godzin w noc
      // zmiany czasu: 02:30 i 03:30 = ten sam epoch) -> jedno dzwonienie.
      for (int j = 0; j < ALARM_COUNT; j++) {
        int otherDay;
        if (j != i && alarms[j].enabled &&
            alarmDueNow(t, now, alarms[j].hour, alarms[j].minute, alarms[j].second, lateWindowSec, &otherDay))
          lastTriggerDay[j] = otherDay;
      }
      startRinging(i);
      return;
    }
  }
}

static void checkNtpSchedule() {
#if ENABLE_LOW_BATTERY_PROTECT
  // ECO: gdy mamy poprawny czas, NIE budz WiFi do auto-sync (najwiekszy pobor).
  // Pusty RTC -> pozwol sie synchronizowac mimo niskiej baterii (zegar musi miec czas).
  if (batteryLow() && isSupportedEpoch(time(nullptr))) return;
#endif
  if (handleNtpRetry()) return;
  struct tm t;
  if (!getLocalTm(&t)) return;
  int day = localDayKey(t);
  // Preferowane okno 18:00-18:05; pominiete (np. dlugi upload) -> dogon o dowolnej
  // porze az do poczatku nocy. ntpStartedDay gwarantuje DOKLADNIE jeden sync/dobe.
  bool preferredWindow = (t.tm_hour == NTP_HOUR && t.tm_min <= NTP_MIN_WINDOW);
  bool catchupWindow   = (t.tm_hour >= NTP_HOUR && t.tm_hour < NIGHT_START_HOUR);
  if ((preferredWindow || catchupWindow) && ntpStartedDay != day) {
    ntpStartedDay = day;
    ntpRetriesLeft = 2;
    syncNtp(false);
  }
}

// ============================================================================
//  [11] ZARZADZANIE ENERGIA — tryb nocny (deep-sleep), light-sleep, ochronny sen
// ============================================================================
static bool anyAlarmDueNow(const struct tm &t) {
  time_t now = time(nullptr);
  for (int i = 0; i < ALARM_COUNT; i++)
    if (alarms[i].enabled &&
        alarmDueNow(t, now, alarms[i].hour, alarms[i].minute, alarms[i].second, ALARM_LATE_WINDOW_SEC, nullptr))
      return true;
  return false;
}

static time_t nightEndEpoch(time_t now) {
  if (!isSupportedEpoch(now)) return 0;
  struct tm t; localtime_r(&now, &t);
  time_t end = epochForClock(t, NIGHT_END_HOUR, NIGHT_END_MIN, 0, 0);
  if (end <= now) end = epochForClock(t, NIGHT_END_HOUR, NIGHT_END_MIN, 0, 1);
  return end;
}

static time_t nextAlarmEpoch(const struct tm &base, time_t now) {
  time_t best = 0;
  for (int i = 0; i < ALARM_COUNT; i++) {
    if (!alarms[i].enabled) continue;
    time_t cand = epochForClock(base, alarms[i].hour, alarms[i].minute, alarms[i].second, 0);
    if (cand <= now) cand = epochForClock(base, alarms[i].hour, alarms[i].minute, alarms[i].second, 1);
    if (best == 0 || cand < best) best = cand;
  }
  return best;
}

static void releaseNightHolds() {
  gpio_deep_sleep_hold_dis();
  for (auto pin : HOLD_PINS) gpio_hold_dis(pin);
  gpio_hold_dis((gpio_num_t)PA_ENABLE_PIN);
  rtc_gpio_deinit((gpio_num_t)KEY_PIN);
  rtc_gpio_deinit((gpio_num_t)PREVIEW_BUTTON_PIN);
  rtc_gpio_deinit((gpio_num_t)RTC_INT_PIN);
}

static void holdNightPins() {
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

static void shutdownNightPeripherals() {
  if (webActive) { server.stop(); webActive = false; }
  stopWifiAndNtp();
  stopPlayback(true);
  shutdownCodec();
  pinMode(PA_ENABLE_PIN, OUTPUT);
  digitalWrite(PA_ENABLE_PIN, LOW);
}

#if ENABLE_LOW_BATTERY_PROTECT
// Ochronny deep-sleep przy krytycznym ogniwie: chroni Li-ion przed glebokim
// rozladowaniem i brownoutem. Budzi sie cyklicznie (timer) sprawdzic ladowanie;
// KEY pozwala wybudzic recznie. Pomijane przy dzwoniacym budziku / audio / WWW /
// podgladzie / aktywnym NTP/zapisie RTC. Deep-sleep resetuje chip -> setup()
// ponownie zmierzy ogniwo i wznowi prace lub usnie dalej.
static void enterCriticalBatterySleepIfNeeded() {
  if (!batteryCritical()) return;
  if (playing || ringing || webActive || previewActive ||
      ntpSyncState != NTP_SYNC_IDLE || rtcWriteRetriesLeft != 0) return;

  Serial.printf("Akumulator krytyczny (%u%%, %.2f V) -> ochronny sen %lu s\n",
                batteryPercent, batteryVoltage, (unsigned long)BATTERY_CRITICAL_RECHECK_SEC);
  drawCriticalBatteryPage();
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

// Tryb nocny: deep-sleep do 06:00 / najblizszego budzika. Ekran zgaszony (panel
// refleksyjny w ciemnosci i tak nieczytelny). Budzenie: alarm RTC (dokladny) +
// timer ESP (zabezpieczenie) + przyciski EXT1.
static void enterNightSleepIfNeeded(bool force) {
  struct tm t;
  if (!getLocalTm(&t) || !isNightMode(t)) return;
  if (anyAlarmDueNow(t)) return;
  if (!force && (webActive || playing || ringing || previewActive ||
                 ntpSyncState != NTP_SYNC_IDLE || rtcWriteRetriesLeft != 0 ||
                 (int32_t)(millis() - wakeHoldUntilMs) < 0)) return;   // utrzymaj wybudzenie po nacisnieciu

  time_t now = time(nullptr);
  time_t wakeAt = nightEndEpoch(now);
  time_t alarmAt = nextAlarmEpoch(t, now);
  if (alarmAt > now && alarmAt < wakeAt) wakeAt = alarmAt;
  if (wakeAt <= now) wakeAt = now + 6 * 60 * 60;
  uint64_t sleepSec = (uint64_t)(wakeAt - now);
  if (sleepSec < NIGHT_MIN_SLEEP_SEC) { sleepSec = NIGHT_MIN_SLEEP_SEC; wakeAt = now + (time_t)sleepSec; }

  bool rtcAlarmOk = rtcSetWakeAlarm(wakeAt);
  if (rtcAlarmOk && digitalRead(RTC_INT_PIN) != HIGH) { rtcDisableWakeAlarm(); rtcAlarmOk = false; }
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
  drawNightPage();
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

// Dynamiczny light-sleep w dzien: spij do granicy nastepnej minuty (ekran HH:MM
// zmienia sie na czas) lub do sekundy najblizszego budzika, max 60 s. RLCD trzyma
// obraz; przyciski wybudzaja przez EXT1. Light-sleep NIE rebootuje -> setup() sie
// nie powtarza -> brak mrugania.
static uint64_t computeIdleSleepUs() {
  time_t now = time(nullptr);
  if (!isSupportedEpoch(now)) return IDLE_LIGHT_SLEEP_US;   // czas nieznany -> krotki sen
  struct tm t; localtime_r(&now, &t);
  uint64_t sleepSec = 60 - (uint64_t)t.tm_sec;             // do granicy minuty
  time_t alarmAt = nextAlarmEpoch(t, now);
  if (alarmAt > now) { uint64_t toAlarm = (uint64_t)(alarmAt - now); if (toAlarm < sleepSec) sleepSec = toAlarm; }
  if (sleepSec > IDLE_LIGHT_SLEEP_MAX_SEC) sleepSec = IDLE_LIGHT_SLEEP_MAX_SEC;
  if (sleepSec < 1) sleepSec = 1;
  return sleepSec * 1000000ULL;
}

static void idlePowerSave() {
  // Nie usypiaj na dlugo, gdy cos wymaga szybkiej obslugi: WWW, audio/dzwonienie,
  // podglad, trwajacy NTP, zaplanowany zapis RTC, lub nacisniety przycisk.
  // wakeHoldUntilMs: po aktywnosci przyciskiem trzymaj wybudzenie, by maszyna stanu
  // KEY zdazyla wykryc ZWOLNIENIE (debounce) i wykonac akcje krotka (wlacz panel WWW
  // / wycisz). Bez tego po szybkim tapnieciu uklad zasypial przed obsluga zwolnienia
  // -> panel WWW wstawal dopiero po ktoryms kolejnym wybudzeniu.
  if (webActive || playing || ringing || previewActive ||
      ntpSyncState != NTP_SYNC_IDLE || rtcWriteRetriesLeft != 0 ||
      (int32_t)(millis() - wakeHoldUntilMs) < 0) { delay(5); return; }
  if (digitalRead(KEY_PIN) == LOW || digitalRead(PREVIEW_BUTTON_PIN) == LOW) { delay(5); return; }

  if (audioReady) shutdownCodec();   // kodek niepotrzebny w spoczynku -> uspij ES8311

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
  readRtcToSystem();

  if (wakeCause == ESP_SLEEP_WAKEUP_EXT1) {
    if (wakePins & (1ULL << KEY_PIN)) {
      // Zacznij mierzyc nacisniecie: krotkie -> wycisz budzik / wlacz panel WWW; dlugie -> nic.
      keyRaw = LOW; keyStable = LOW; keyChangedMs = millis();
      keyPressMs = millis(); keyLongFired = false;
      wakeHoldUntilMs = millis() + WAKE_HOLD_MS;
    }
    if (wakePins & (1ULL << PREVIEW_BUTTON_PIN)) {
      bootRaw = LOW; bootStable = LOW; bootPressedMs = millis(); bootChangedMs = millis();
      showPreview();
    }
  }
}

// ============================================================================
//  Przyciski
// ============================================================================
static void showPreview() {
  if (playing || ringing) return;
  previewActive = true;
  previewUntilMs = millis() + ALARM_PREVIEW_MS;
  wakeHoldUntilMs = millis() + WAKE_HOLD_MS;
  displayDirty = true;
}

static void handleKey() {
  bool raw = digitalRead(KEY_PIN);
  if (raw != keyRaw) { keyRaw = raw; keyChangedMs = millis(); }
  if (millis() - keyChangedMs > BUTTON_DEBOUNCE_MS && raw != keyStable) {
    keyStable = raw;
    if (keyStable == LOW) {                         // nacisniety
      keyPressMs = millis(); keyLongFired = false;
      wakeHoldUntilMs = millis() + WAKE_HOLD_MS;
    } else {                                        // zwolniony — akcja krotka
      if (!keyLongFired) {                          // krotki KEY: wycisz budzik lub wlacz panel WWW
        if (playing || ringing) stopPlayback(true); // dzwoni budzik -> wycisz
        else startWebWindow();                       // nie dzwoni -> wlacz panel WWW
      }
      keyPressMs = 0;
    }
  }
  // Dlugie przytrzymanie KEY: nic nie robi — ustawiamy tylko keyLongFired, by
  // zwolnienie po dlugim przytrzymaniu NIE wywolalo akcji krotkiej.
  if (keyStable == LOW && !keyLongFired && keyPressMs != 0 &&
      millis() - keyPressMs >= WEB_LONG_PRESS_MS) {
    keyLongFired = true;
  }
}

static void handleBoot() {
  bool raw = digitalRead(PREVIEW_BUTTON_PIN);
  if (raw != bootRaw) { bootRaw = raw; bootChangedMs = millis(); }
  if (millis() - bootChangedMs > BUTTON_DEBOUNCE_MS && raw != bootStable) {
    bootStable = raw;
    if (bootStable == LOW) { bootPressedMs = millis(); wakeHoldUntilMs = millis() + WAKE_HOLD_MS; }
    else {
      uint32_t press = millis() - bootPressedMs;
      if (bootPressedMs != 0 && press <= 1200) showPreview();
      bootPressedMs = 0;
    }
  }
}

// ============================================================================
//  Inicjalizacja wyswietlacza
//  Ten egzemplarz ST7305 NIE przywraca obrazu po deep-sleep przez sam
//  initInterface() -> pelny re-init (reset RST + begin) przy KAZDYM wybudzeniu.
//  W dzien petla uzywa light-sleep (setup() sie nie powtarza), wiec panel nie jest
//  resetowany i trzyma obraz -> brak mrugania.
// ============================================================================
static inline void st7305SetStaticLowPower(bool enable) {
#if ST7305_STATIC_LPM
  (void)enable;   // zarezerwowane: HPM/LPM panelu — nieprzetestowana komenda, nie wysylamy bez testu
#else
  (void)enable;
#endif
}

static void setupDisplay(bool coldBoot) {
  (void)coldBoot;
  pinMode(RLCD_RST_PIN, OUTPUT);
  digitalWrite(RLCD_RST_PIN, HIGH);
  digitalWrite(RLCD_RST_PIN, LOW);
  delay(10);
  digitalWrite(RLCD_RST_PIN, HIGH);
  delay(10);
  // HW SPI: usuwa ~4 s opoznienia pelnego sendBuffer() (programowy SPI przesuwal zmiane minuty).
  SPI.begin(RLCD_SCK_PIN, -1, RLCD_MOSI_PIN, RLCD_CS_PIN);
  display.setBusClock(2000000UL);      // zalecenie sterownika U8g2 dla ST7305
  display.begin();
  gfx = &display;
  gfx->setContrast(255);
  st7305SetStaticLowPower(false);
}

// ============================================================================
//  setup / loop
// ============================================================================
void setup() {
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
  releaseNightHolds();
  pinMode(PA_ENABLE_PIN, OUTPUT);
  digitalWrite(PA_ENABLE_PIN, LOW);
  setupDisplay(coldBoot);

  if (coldBoot) {                       // RTC RAM przezywa deep-sleep -> zeruj tylko na zimnym starcie
    resetTriggerState();
    ntpStartedDay = -1;
    ntpMorningDay = -1;
    batteryHasReading = false; batteryPercent = 0; batteryVoltage = 0.0f;
    g_drain.valid = false;
    g_lastSummaryDay = -1;
    g_wifiHint = false;
    g_bootCount = 0;
    g_coldBootEpoch = 0;
    g_lastRtcDriftSec = 0;
    g_lastNtpSyncEpoch = 0;
    g_prevNtpSyncEpoch = 0;
  }
  g_bootCount++;

  setenv("TZ", TZ_POLAND, 1);
  tzset();

  pinMode(KEY_PIN, INPUT_PULLUP);
  pinMode(PREVIEW_BUTTON_PIN, INPUT_PULLUP);
  pinMode(RTC_INT_PIN, INPUT_PULLUP);
  keyRaw = digitalRead(KEY_PIN);  keyStable = keyRaw;  keyChangedMs = millis();
  bootRaw = digitalRead(PREVIEW_BUTTON_PIN); bootStable = bootRaw; bootChangedMs = millis();

  loadSettings();
#if ENABLE_DIAGNOSTICS
  recordResetReason(esp_reset_reason());
#endif
  Serial.printf("\n=== %s v%s  (build %s %s) ===\n", FW_NAME, FW_VERSION, FW_BUILD_DATE, FW_BUILD_TIME);
  Serial.printf("Rozruch #%u, przyczyna resetu: %s\n", g_bootCount, resetReasonText(esp_reset_reason()));
  updateBattery(true);

  bool timeFromRtc = readRtcToSystem();
  rtcEnsureOffsetCalibration();           // kalibracja dryfu (0x02) + odczyt zwrotny
  bool rtcAlarmCleared = rtcDisableWakeAlarm();
  ntpStatus = timeFromRtc ? "czas z RTC" : "RTC pusty/brak";
  if (rtcWake && !rtcAlarmCleared) ntpStatus = "RTC alarm: blad kasowania";
#if ENABLE_DIAGNOSTICS
  { time_t t0 = time(nullptr);
    if (g_coldBootEpoch == 0 && isSupportedEpoch(t0)) g_coldBootEpoch = (int64_t)t0; }
#endif

  codec.powerDown();                      // ES8311 startuje w stanie niskiego poboru

  if (coldBoot) ensureFsMounted();        // FS tylko do MP3 — montuj leniwie, krotsze wybudzenie
  checkAlarms(scheduledWake ? NTP_ALARM_CATCHUP_SEC : ALARM_LATE_WINDOW_SEC);
  if (!userWake && !ringing) enterNightSleepIfNeeded(true);
#if ENABLE_LOW_BATTERY_PROTECT
  if (!userWake && !ringing) enterCriticalBatterySleepIfNeeded();
#endif

  // WiFi domyslnie OFF. Sync NTP: zimny start, pusty RTC, lub poranne wybudzenie
  // o 06:00 (drugie dobowe synchro obok 18:00). Po zwyklym wybudzeniu wystarcza RTC.
  bool morningSync = false;
  { struct tm t;
    if (getLocalTm(&t) && t.tm_hour == NIGHT_END_HOUR && t.tm_min < NTP_MIN_WINDOW) {
      int day = localDayKey(t);
      if (ntpMorningDay != day) { ntpMorningDay = day; morningSync = true; }   // raz na dobe
    }
  }
  bool willSync = (coldBoot || !timeFromRtc || morningSync);
  if (!willSync) stopWifiAndNtp();
  if (userWake) wakeHoldUntilMs = millis() + WAKE_HOLD_MS;
  if (previewWake) showPreview();
  if (keyWake && !coldBoot) {             // KEY z deep-sleep: zacznij mierzyc nacisniecie (krotkie -> panel WWW)
    keyRaw = LOW; keyStable = LOW; keyChangedMs = millis();
    keyPressMs = millis(); keyLongFired = false;
  }
  if (willSync) {
    ntpRetriesLeft = timeFromRtc ? 2 : 3;   // pusty RTC: kilka szybkich prob, potem backoff
    syncNtp(false);
  }

  displayDirty = true;
}

void loop() {
  watchdogFeed();
  handleKey();
  handleBoot();
  enterNightSleepIfNeeded(false);

  if (webActive) {
    static uint32_t wifiLostMs = 0;
    if (WiFi.status() != WL_CONNECTED) {
      // Chwilowy zanik (roaming) nie zamyka okna od razu: auto-reconnect zwykle wraca.
      if (wifiLostMs == 0) wifiLostMs = millis();
      if (millis() - wifiLostMs > WEB_WIFI_GRACE_MS) { wifiLostMs = 0; statusText = "WiFi rozlaczone"; stopWebWindow(); }
    } else {
      wifiLostMs = 0;
      server.handleClient();
      if ((int32_t)(millis() - webUntilMs) >= 0) stopWebWindow();
    }
  }

  serviceAudio();
  serviceRinging();
  updateBattery(false);
#if ENABLE_LOW_BATTERY_PROTECT
  enterCriticalBatterySleepIfNeeded();
#endif
  bool ntpAdjustedTime = serviceNtp();
  // Okno spoznienia poszerzone o realny przestoj petli: dlugi handleClient (upload
  // MP3) ani 60 s light-sleep nie moga zgubic budzika z terminem w trakcie przestoju.
  static uint32_t lastAlarmCheckMs = 0;
  uint32_t loopGapSec = lastAlarmCheckMs ? (millis() - lastAlarmCheckMs) / 1000UL : 0;
  lastAlarmCheckMs = millis();
  uint32_t lateWindowSec = ntpAdjustedTime ? NTP_ALARM_CATCHUP_SEC : ALARM_LATE_WINDOW_SEC;
  if (loopGapSec + 2 > lateWindowSec) lateWindowSec = loopGapSec + 2;
  checkAlarms(lateWindowSec);
  checkNtpSchedule();
  handleRtcWriteRetry();
  updateDisplay();
  idlePowerSave();
}
