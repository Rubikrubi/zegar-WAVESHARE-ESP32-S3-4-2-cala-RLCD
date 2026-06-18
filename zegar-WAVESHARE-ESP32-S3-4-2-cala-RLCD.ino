
//  Zegar NTP v C.0.2
//  Sprzet: Waveshare ESP32-S3-LCD-4.2 (RLCD ST7305 400x300, ES8311, PCF85063A).
//  Arduino IDE: ESP32S3 Dev Module, Flash 8MB, partycja "8M with spiffs".
//  Wymagane pliki obok .ino: photo_font.h
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

#include "photo_font.h"

// ---------------------------------------------------------------------------
//  power_runtime — estymator zuzycia baterii (dawniej osobny power_runtime.h).
//  Wtopiony tutaj: deliverable = jeden plik .ino + osobny photo_font.h.
//  Czysta arytmetyka na probkach SoC, bez zaleznosci od Arduino/ESP.
// ---------------------------------------------------------------------------
// Deklaracje wyprzedzajace. Arduino IDE/arduino-cli auto-generuje prototypy
// funkcji i wstawia je PRZED definicjami struktur (m.in. sanitizeAlarm(AlarmConfig&)
// oraz drain*(DrainState&)). Bez nich kompilator zglasza
// "'AlarmConfig' was not declared in this scope". NIE usuwac.
struct AlarmConfig;
struct DrainState;

#ifndef DRAIN_CAP_MAH
#define DRAIN_CAP_MAH        3400
#endif
#define DRAIN_CHARGE_HYST    5        // wzrost SoC% liczony jako "doladowano" -> reset kotwicy
#define DRAIN_MIN_WINDOW_SEC 21600    // min. 6 h danych przed estymacja

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
  if (dSoc <= 0) return 0.0f;
  return (dSoc / 100.0f) * (float)DRAIN_CAP_MAH / (dt / 86400.0f);
}

// Dni do rozladowania przy biezacym tempie. <0 => nieznane.
static inline float drainDaysRemaining(const DrainState &d, uint8_t curSoc) {
  float perDay = drainMahPerDay(d);
  if (perDay <= 0.0f) return -1.0f;
  return (curSoc / 100.0f) * (float)DRAIN_CAP_MAH / perDay;
}

// ---------------------------------------------------------------------------
//  Adresy I2C
// ---------------------------------------------------------------------------
#define ES8311_ADDR        0x18
#define PCF85063A_ADDR     0x51
#define RTC_FORMAT_MARKER  0xA5

// ---------------------------------------------------------------------------
//  Piny — wyswietlacz RLCD (programowe SPI)
// ---------------------------------------------------------------------------
#define RLCD_SCK_PIN   11
#define RLCD_MOSI_PIN  12
#define RLCD_DC_PIN    5
#define RLCD_CS_PIN    40
#define RLCD_RST_PIN   41
#define LCD_WIDTH      400
#define LCD_HEIGHT     300

// ---------------------------------------------------------------------------
//  Piny — I2C (kodek + RTC), audio I2S, przyciski, akumulator
// ---------------------------------------------------------------------------
#define I2C_SDA_PIN    13
#define I2C_SCL_PIN    14
#define RTC_INT_PIN    15

#define I2S_MCLK_PIN   16
#define I2S_BCLK_PIN   9
#define I2S_WS_PIN     45
#define I2S_DOUT_PIN   8
#define PA_ENABLE_PIN  46

#define KEY_PIN        18    // krotki przycisk: start/odswiezenie panelu WWW (stop sam po 5 min od ostatniej aktywnosci), wyciszenie budzika
#define PREVIEW_BUTTON_PIN 0  // krotki przycisk: podglad najblizszego budzika

#define BATTERY_ADC_CHANNEL          ADC_CHANNEL_3
#define BATTERY_DIVIDER_RATIO        3.0f
#define BATTERY_VOLTAGE_CALIBRATION  1.017f
#define BATTERY_CAPACITY_MAH         3400    // Samsung INR18650-35E (3400 mAh)
#define BATTERY_SAMPLE_COUNT         16
#define BATTERY_EMPTY_VOLTAGE        3.00f
#define BATTERY_FULL_VOLTAGE         4.20f
#define BATTERY_FULL_FLOOR           99
#define BATTERY_FULL_HOLD_FLOOR      98
#define BATTERY_READ_INTERVAL_MS     (10UL * 60UL * 1000UL)
#define BATTERY_BUSY_RETRY_MS        (30UL * 1000UL)   // pod obciazeniem: ponow pomiar za 30 s zamiast czekac pelny interwal

// ---------------------------------------------------------------------------
//  Parametry logiki zegara / budzikow
//  Stale-wartosci jako `constexpr` (typowane, zasiegowe, widoczne w debuggerze).
//  Makrami zostaja tylko flagi uzywane w `#if` (preprocesor nie zna `constexpr`)
//  oraz IDLE_CPU_MHZ sparowane z flaga IDLE_CPU_DOWNCLOCK.
// ---------------------------------------------------------------------------
constexpr int      ALARM_COUNT                = 3;
constexpr int      SOUND_COUNT                = 3;
constexpr int      MAX_AUDIO_VOLUME           = 21;
constexpr uint8_t  DEFAULT_AUDIO_VOLUME       = 10;
constexpr uint8_t  CODEC_VOLUME_PERCENT       = 100;
constexpr uint32_t ALARM_LATE_WINDOW_SEC      = 20;  // budzik moze odpalic do 20 s po terminie (np. po wybudzeniu)
constexpr uint32_t RING_RETRY_MS              = 3000;  // odstep ponawiania odtwarzania w petli dzwonienia
constexpr uint32_t RING_MIN_PLAY_MS           = 750;   // odtwarzanie krotsze = blad pliku (uszkodzony MP3 "konczy sie" po kilkudziesieciu ms)
constexpr uint8_t  RING_MAX_ERRORS            = 10;
constexpr uint32_t RING_TIMEOUT_MS            = 15UL * 60UL * 1000UL;

constexpr uint32_t WEB_WINDOW_MS              = 5UL * 60UL * 1000UL;
constexpr uint32_t WEB_WIFI_GRACE_MS          = 10000;  // tyle czekaj na powrot WiFi (auto-reconnect) zanim okno WWW zostanie zamkniete
constexpr uint32_t NTP_RETRY_MS               = 5UL * 60UL * 1000UL;
constexpr uint32_t NTP_INVALID_TIME_RETRY_MS  = NTP_RETRY_MS;
constexpr uint32_t NTP_WIFI_TIMEOUT_MS        = 15000;
constexpr uint32_t NTP_PACKET_TIMEOUT_MS      = 50000;  // lwIP SNTP probuje serwery sekwencyjnie co 15 s (SNTP_RECV_TIMEOUT);
                                                        // 15 s ucinalo probe zanim serwer 2/3 dostal szanse -> 50 s pokrywa wszystkie trzy
constexpr uint32_t NTP_ALARM_CATCHUP_SEC      = 90;
constexpr int      NTP_HOUR                   = 10;  // codzienna synchronizacja w oknie 10:00–10:05
constexpr int      NTP_MIN_WINDOW             = 5;
constexpr uint32_t RTC_WRITE_RETRY_MS         = 5000;
constexpr uint8_t  RTC_WRITE_RETRIES          = 12;
constexpr int      RTC_WAKE_MAX_SKEW_SEC      = 30;

constexpr uint32_t DAY_CPU_MHZ                = 160;
#define POWER_DOWN_FLASH_IN_SLEEP 0   // 1=odlacz flash (VDD_SDIO) w light-sleep dziennym
                                      //   -> najwiekszy spadek pradu bazowego.
                                      //   Restart/niestabilnosc po wgraniu? Ustaw 0 i wgraj ponownie.
constexpr uint32_t NIGHT_CPU_MHZ              = 40;
#define FAST_NTP_RECONNECT 1   // 1=uzyj zapamietanego kanalu+BSSID (szybsze laczenie, krocej radia)
#define IDLE_CPU_DOWNCLOCK 0   // 1=80 MHz w bezczynnosci zegara (drobna oszczednosc).
                               //   OFF domyslnie: dotyka taktowania audio, brak testu.
#define IDLE_CPU_MHZ       80
#define ST7305_STATIC_LPM 0   // 0=WYLACZONE. Tryb niskiego poboru panelu przy statycznym
                              //   obrazie. Nieprzetestowana komenda panelu (jak zaklad o
                              //   deep-sleep) -> wlaczyc DOPIERO gdy mozliwy test sprzetowy.
constexpr uint64_t IDLE_LIGHT_SLEEP_US        = 900000ULL;  // fallback gdy czas nieznany (brak RTC/NTP)
constexpr uint64_t IDLE_LIGHT_SLEEP_MAX_SEC   = 60ULL;  // gorny limit dynamicznego light-sleep w dzien (sen do granicy minuty -> 1 wybudzenie/min)
constexpr int      NIGHT_START_HOUR           = 23;
constexpr int      NIGHT_START_MIN            = 59;
constexpr int      NIGHT_END_HOUR             = 6;
constexpr int      NIGHT_END_MIN              = 0;
constexpr uint32_t NIGHT_MIN_SLEEP_SEC        = 5;
constexpr uint32_t NIGHT_FALLBACK_CHUNK_SEC   = 3600;  // maks. odcinek snu nocnego, gdy alarm RTC nie uzbroil sie (budzenie tylko timerem RC)

constexpr uint32_t BUTTON_DEBOUNCE_MS         = 40;
constexpr uint32_t BOOT_SHORT_PRESS_MS        = 1200;
constexpr uint32_t ALARM_PREVIEW_MS           = 5000;

// ---------------------------------------------------------------------------
//  Stale konfiguracyjne
// ---------------------------------------------------------------------------
static const char *TZ_POLAND   = "CET-1CEST,M3.5.0/2,M10.5.0/3";
static const char *NTP_SERVER_1 = "pool.ntp.org";
static const char *NTP_SERVER_2 = "time.google.com";
static const char *NTP_SERVER_3 = "time.cloudflare.com";

static const char *DEFAULT_WIFI_SSID = "";
static const char *DEFAULT_WIFI_PASS = "";

// Pliki dzwiekow — indeks 0..2 odpowiada wyborowi w panelu WWW.
static const char *SOUND_FILES[SOUND_COUNT] = {
  "/alarm1.mp3",
  "/alarm2.mp3",
  "/alarm3.mp3"
};
static const char *SOUND_NAMES[SOUND_COUNT] = {
  "Dzwiek 1",
  "Dzwiek 2",
  "Dzwiek 3"
};

// ---------------------------------------------------------------------------
//  Sterownik kodeka ES8311 (sekwencja rejestrow — inicjalizacja sprzetowa)
// ---------------------------------------------------------------------------
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

  bool configBits16() {
    return write(0x09, 0x0C) && write(0x0A, 0x0C);
  }

  // Glosnosc wyjscia kodeka 0..100% (0 = mute). Mapowanie liniowe na rejestr 0x32
  // do 0xD7 (ok. +12 dB): glosniej niz 0xBF = 0 dB, ale z mniejszym ryzykiem
  // przesterowania niz przy maksymalnym cyfrowym wzmocnieniu 0xFF.
  bool setVolume(uint8_t percent) {
    if (percent > 100) percent = 100;
    uint8_t reg = (uint8_t)(((uint16_t)percent * 215u) / 100u);
    return write(0x32, reg);
  }

  // Pelne uspienie kodeka: wycisz DAC i wprowadz uklad w reset (bloki analogowe
  // off). To najwiekszy odbiornik w spoczynku, a begin() niezawodnie go budzi.
  bool powerDown() {
    write(0x32, 0x00);          // mute DAC
    return write(0x00, 0x1F);   // assert reset -> analog off (minimalny pobor)
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

// ---------------------------------------------------------------------------
//  Model danych
// ---------------------------------------------------------------------------
struct AlarmConfig {
  uint8_t enabled;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint8_t volume;     // 0..MAX_AUDIO_VOLUME
  uint8_t sound;      // indeks 0..SOUND_COUNT-1 → SOUND_FILES
  char    label[16];
};

// ---------------------------------------------------------------------------
//  Obiekty globalne
// ---------------------------------------------------------------------------
// RST = U8X8_PIN_NONE: U8g2 nigdy nie pulsuje resetu. Reset sprzetowy robimy
// recznie tylko na zimnym starcie (setupDisplay). Dzieki temu ciepłe wybudzenie z
// deep-sleep NIE resetuje panelu -> brak mrugania (panel trzyma obraz przez sen).
static U8G2_ST7305_300X400_F_4W_HW_SPI display(
  U8G2_R1, RLCD_CS_PIN, RLCD_DC_PIN, U8X8_PIN_NONE);
static U8G2 *gfx = &display;

static WebServer  server(80);
static Preferences prefs;
static Audio       audio;
static ES8311Codec codec;

static adc_oneshot_unit_handle_t batteryAdc  = nullptr;
static adc_cali_handle_t         batteryCali = nullptr;

// ---------------------------------------------------------------------------
//  Stan
// ---------------------------------------------------------------------------
static AlarmConfig alarms[ALARM_COUNT];
static char wifiSsid[33] = "";
static char wifiPass[65] = "";

static bool   webActive = false;
static bool   webRoutesInstalled = false;
static uint32_t webUntilMs = 0;

static bool      fsMounted = false;
static fs::FS   *dataFS = nullptr;
static const char *dataFSName = "brak";
static File      mp3UploadFile;
static bool      mp3UploadAccepted = false;
static bool      mp3UploadHadData = false;
static String    mp3UploadPath = "";
static String    mp3UploadTempPath = "";

static bool   audioReady = false;
static bool   batteryReady = false;
static bool   batteryCaliReady = false;
static bool   displayDirty = true;

static bool      playing = false;          // trwa odtwarzanie pliku MP3
static String    statusText = "IDLE";
static String    ntpStatus = "never";

// RTC RAM: przezywa deep-sleep. Inaczej reboot co minute zerowalby je i histereza
// "trzymaj 100% gdy pomiar >= 98%" (stabilizePercent) nigdy by nie dzialala -> ekran
// migalby 99<->100 przy pelnym ogniwie. Zerowane na cold boot.
static RTC_DATA_ATTR uint8_t batteryPercent;
static RTC_DATA_ATTR float   batteryVoltage;
static RTC_DATA_ATTR bool    batteryHasReading;

// Estymator zuzycia: probki w RTC RAM (przezywaja sen, znikaja po zaniku zasilania
// = dokladnie jeden cykl rozladowania). Zero zapisow do flash w trakcie pracy.
static RTC_DATA_ATTR DrainState g_drain;
static RTC_DATA_ATTR int        g_lastSummaryDay;
static RTC_DATA_ATTR uint8_t g_wifiBssid[6];
static RTC_DATA_ATTR int32_t g_wifiChannel;
static RTC_DATA_ATTR bool    g_wifiHint;

// Stan "dzwonienia" budzika (ponawianie az do wyciszenia / timeoutu)
static bool    ringing = false;
static int     ringingAlarm = -1;          // ktory budzik dzwoni (indeks)
static uint32_t ringStartMs = 0;
static uint32_t ringLastRetryMs = 0;
static uint8_t  ringErrors = 0;
static uint32_t soundStartedMs = 0;        // start ostatniego odtwarzania (wykrywanie plikow padajacych natychmiast)

static int lastDrawMinute = -1;
static int lastDrawDay = -1;

// Dedup wyzwolen — przezywa deep-sleep (RTC RAM), zerowany tylko na cold boot.
static RTC_DATA_ATTR int lastTriggerDay[ALARM_COUNT];

// RTC RAM: przezywa deep-sleep (reboot co minute kasowalby go i wymuszal
// powtorna synchronizacje NTP w kazdej minucie okna 10:00-10:05). Zerowany na cold boot.
static RTC_DATA_ATTR int ntpStartedDay;
// Osobny licznik dnia dla porannej re-synchronizacji (wybudzenie z trybu nocnego
// ~06:00). Oddzielny od ntpStartedDay, by sync 06:00 NIE blokowal planowego 10:00.
static RTC_DATA_ATTR int ntpMorningDay;
static uint8_t  ntpRetriesLeft = 0;
static uint32_t ntpNextRetryMs = 0;
static uint32_t ntpFailureRetryDelayMs = NTP_RETRY_MS;
static uint8_t  rtcWriteRetriesLeft = 0;
static uint32_t rtcWriteNextRetryMs = 0;
static uint32_t batteryNextReadMs = 0;

static bool     keyRaw = HIGH, keyStable = HIGH;
static uint32_t keyChangedMs = 0;
static bool     bootRaw = HIGH, bootStable = HIGH;
static uint32_t bootChangedMs = 0, bootPressedMs = 0;
static bool     previewActive = false;
static uint32_t previewUntilMs = 0;

static const gpio_num_t HOLD_PINS[] = {
  (gpio_num_t)RLCD_DC_PIN, (gpio_num_t)RLCD_CS_PIN, (gpio_num_t)RLCD_SCK_PIN,
  (gpio_num_t)RLCD_MOSI_PIN, (gpio_num_t)RLCD_RST_PIN
};

// Prototypy
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

// ============================================================================
//  Czas — konwersje cywilne i RTC PCF85063A
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
  base.tm_mday += addDays;     // mktime normalizuje przepelnienie dnia/miesiaca
  base.tm_isdst = -1;          // niech mktime sam ustali DST dla danej daty
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

static bool leapYear(int y) { return (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0); }

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

static bool rtcReadControl1(uint8_t *control1) {
  if (!control1 || !rtcBeginSession()) return false;

  Wire.beginTransmission(PCF85063A_ADDR);
  Wire.write(0x00);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)PCF85063A_ADDR, (uint8_t)1) != 1) return false;

  *control1 = Wire.read();
  return true;
}

static bool rtcReadControl2(uint8_t *control2) {
  if (!control2 || !rtcBeginSession()) return false;

  Wire.beginTransmission(PCF85063A_ADDR);
  Wire.write(0x01);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)PCF85063A_ADDR, (uint8_t)1) != 1) return false;

  *control2 = Wire.read();
  return true;
}

static bool rtcWriteControl1(uint8_t control1) {
  Wire.beginTransmission(PCF85063A_ADDR);
  Wire.write(0x00);
  Wire.write(control1);
  return Wire.endTransmission() == 0;
}

static bool rtcReleaseClock(uint8_t runningControl1) {
  for (uint8_t attempt = 0; attempt < 3; attempt++) {
    if (rtcWriteControl1(runningControl1)) return true;
    delay(2);
  }
  return false;
}

// Zatrzymaj licznik przed ustawieniem czasu. STOP zeruje glowna czesc
// preskalera, dzieki czemu pierwszy impuls sekundowy ma okreslona faze.
static bool rtcPrepareTimeWrite(uint8_t *runningControl1) {
  if (!runningControl1) return false;
  uint8_t control1;
  if (!rtcReadControl1(&control1)) return false;
  *runningControl1 = control1 &
                     (uint8_t)~((1U << 7) | (1U << 5) | (1U << 2) | (1U << 1));
  if (rtcWriteControl1(*runningControl1 | (1U << 5))) return true;
  rtcReleaseClock(*runningControl1);
  return false;
}

static bool rtcHasCurrentFormat() {
  Wire.beginTransmission(PCF85063A_ADDR);
  Wire.write(0x03);  // RAM_byte
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)PCF85063A_ADDR, (uint8_t)1) != 1) return false;
  return Wire.read() == RTC_FORMAT_MARKER;
}

static bool rtcStoreCurrentFormat() {
  Wire.beginTransmission(PCF85063A_ADDR);
  Wire.write(0x03);  // RAM_byte
  Wire.write(RTC_FORMAT_MARKER);
  return Wire.endTransmission() == 0;
}

static bool rtcInvalidateCurrentFormat() {
  Wire.beginTransmission(PCF85063A_ADDR);
  Wire.write(0x03);  // RAM_byte
  Wire.write((uint8_t)0);
  return Wire.endTransmission() == 0;
}

// Odczyt 7 rejestrow czasu (0x04..0x0A) jednym transferem → brak rwania na granicy sekundy.
// Rejestr roku 00..99 reprezentuje lata 2000..2099. Taka baza zachowuje
// zgodnosc sprzetowego obliczania lat przestepnych w PCF85063A.
static bool readRtcEpochInternal(time_t *epoch, bool requireCurrentFormat) {
  if (!epoch) return false;
  uint8_t control1;
  if (!rtcReadControl1(&control1)) return false;

  // Nie wznawiaj po cichu zatrzymanego lub przestawionego RTC. Przy aktywnym
  // markerze oznaczaloby to zaakceptowanie starego czasu jako aktualnego.
  const uint8_t invalidModeMask = (1U << 7) | (1U << 5) | (1U << 1);
  if ((control1 & invalidModeMask) != 0 ||
      (requireCurrentFormat && !rtcHasCurrentFormat())) return false;

  Wire.beginTransmission(PCF85063A_ADDR);
  Wire.write(0x04);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)PCF85063A_ADDR, (uint8_t)7) != 7) return false;

  uint8_t secReg  = Wire.read();
  uint8_t minReg  = Wire.read();
  uint8_t hourReg = Wire.read();
  uint8_t dayReg  = Wire.read();
  Wire.read();                      // dzien tygodnia — pomijamy
  uint8_t monReg  = Wire.read();
  uint8_t yearReg = Wire.read();

  bool oscStopped = (secReg & 0x80) != 0;
  secReg &= 0x7F; minReg &= 0x7F; hourReg &= 0x3F; dayReg &= 0x3F; monReg &= 0x1F;

  if (oscStopped || !validBcd(secReg) || !validBcd(minReg) || !validBcd(hourReg) ||
      !validBcd(dayReg) || !validBcd(monReg) || !validBcd(yearReg)) {
    return false;
  }

  int s  = bcd2dec(secReg);
  int mi = bcd2dec(minReg);
  int h  = bcd2dec(hourReg);
  int d  = bcd2dec(dayReg);
  int mo = bcd2dec(monReg);
  int y  = bcd2dec(yearReg) + 2000;

  if (!validDateTime(y, mo, d, h, mi, s)) return false;
  *epoch = utcEpochFromCivil(y, mo, d, h, mi, s);
  return true;
}

static bool readRtcEpoch(time_t *epoch) {
  return readRtcEpochInternal(epoch, true);
}

static bool readRtcToSystem() {
  for (uint8_t attempt = 0; attempt < 3; attempt++) {
    time_t epoch;
    if (readRtcEpoch(&epoch)) {
      setSystemEpoch(epoch);
      return true;
    }
    delay(2);
  }
  return false;
}

// Zapis lustrzany do readRtcToSystem: RTC przechowuje czas UTC i rok 00..99
// dla zakresu 2000..2099.
static bool writeRtc(time_t epoch) {
  if (!isSupportedEpoch(epoch)) return false;

  // Wyrownanie do granicy sekundy. Po zwolnieniu STOP preskaler PCF85063A startuje
  // od zera, wiec wpisana sekunda "trzyma sie" przez pelne ~1.0 s -> faza przewrotu
  // sekundy = chwila zwolnienia STOP. Dotychczas zapisywano uciety time_t (ulamek
  // sekundy z NTP gubiony) i zwalniano od razu po I2C -> RTC stale spozniony o
  // 0.5-1 s; po ~20 h do wybudzenia 06:00 skladalo sie to na ~3 s. Dlatego zapisz
  // NASTEPNA pelna sekunde i zwolnij STOP dokladnie, gdy zegar systemowy ja osiagnie.
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  time_t writeEpoch = epoch;
  bool   alignToBoundary = false;
  // Wyrownuj tylko gdy 'epoch' to biezacy czas systemowy (tak robia wszyscy wolajacy:
  // time(nullptr) / syncedEpoch). Inny epoch -> zapis 1:1, bez czekania na granice.
  if (isSupportedEpoch(tv.tv_sec) && (tv.tv_sec == epoch || tv.tv_sec + 1 == epoch)) {
    writeEpoch = tv.tv_sec + 1;          // najblizsza nadchodzaca pelna sekunda
    alignToBoundary = true;
  }

  struct tm gm;
  gmtime_r(&writeEpoch, &gm);
  int year = gm.tm_year + 1900;

  // Najpierw uniewaznij dane. Reset w dowolnym dalszym miejscu nie moze
  // ujawnic niepelnego zapisu jako poprawnego czasu.
  if (!rtcBeginSession()) return false;
  if (!rtcInvalidateCurrentFormat()) return false;
  uint8_t runningControl1;
  if (!rtcPrepareTimeWrite(&runningControl1)) return false;
  uint8_t yearReg = (uint8_t)(year % 100);

  Wire.beginTransmission(PCF85063A_ADDR);
  Wire.write(0x04);
  Wire.write(dec2bcd(gm.tm_sec) & 0x7F);            // OS=0 → oscylator OK
  Wire.write(dec2bcd(gm.tm_min) & 0x7F);
  Wire.write(dec2bcd(gm.tm_hour) & 0x3F);           // 24h
  Wire.write(dec2bcd(gm.tm_mday) & 0x3F);
  Wire.write((uint8_t)(gm.tm_wday & 0x07));
  Wire.write(dec2bcd((uint8_t)(gm.tm_mon + 1)) & 0x1F);
  Wire.write(dec2bcd(yearReg));
  bool registersWritten = Wire.endTransmission() == 0;
  if (!registersWritten) { rtcReleaseClock(runningControl1); return false; }

  // Odczekaj do granicy docelowej sekundy, potem zwolnij STOP. Czas zuzyty na I2C
  // jest wliczony (porownanie do tej samej bazy gettimeofday). Maks. czekanie ~1 s.
  if (alignToBoundary) {
    struct timeval nowTv;
    for (;;) {
      gettimeofday(&nowTv, nullptr);
      long remainingUs = (long)(writeEpoch - nowTv.tv_sec) * 1000000L - nowTv.tv_usec;
      if (remainingUs <= 0) break;
      if (remainingUs > 2000) delay(1); else delayMicroseconds((uint32_t)remainingUs);
    }
  }
  bool clockReleased = rtcReleaseClock(runningControl1);   // zwolnienie na granicy sekundy
  if (!clockReleased) return false;

  // Marker nadal jest niewazny. Najpierw sprawdz surowe rejestry, a dopiero
  // po poprawnej weryfikacji oznacz zapis jako gotowy do uzycia przy starcie.
  time_t verifiedEpoch;
  if (!readRtcEpochInternal(&verifiedEpoch, false)) return false;
  long long rtcDifference = (long long)verifiedEpoch - (long long)writeEpoch;
  if (rtcDifference < -2 || rtcDifference > 2) return false;
  if (rtcStoreCurrentFormat() && rtcHasCurrentFormat()) return true;
  rtcInvalidateCurrentFormat();
  return false;
}

// INT ma sluzyc wylacznie alarmowi wybudzajacemu. Wylacz korekcje, MI/HMI
// i countdown timer, a przy okazji skasuj pozostawione flagi AF/TF.
static bool rtcConfigureWakeInterrupt(bool alarmEnabled) {
  uint8_t control1;
  if (!rtcReadControl1(&control1)) return false;
  control1 &= (uint8_t)~(1U << 2);                   // CIE=0
  Wire.beginTransmission(PCF85063A_ADDR);
  Wire.write(0x00);
  Wire.write(control1);
  if (Wire.endTransmission() != 0) return false;

  // Najpierw zatrzymaj timer i jego przerwanie. PCF85063A wymaga TE=0 przed
  // zmiana Timer_value, inaczej pierwszy okres po zmianie jest nieokreslony.
  Wire.beginTransmission(PCF85063A_ADDR);
  Wire.write(0x11);                                  // Timer_mode
  Wire.write((uint8_t)0x18);                         // TCF=1/60 Hz, TE=0, TIE=0
  if (Wire.endTransmission() != 0) return false;

  Wire.beginTransmission(PCF85063A_ADDR);
  Wire.write(0x10);                                  // Timer_value
  Wire.write((uint8_t)0);
  if (Wire.endTransmission() != 0) return false;

  uint8_t control2;
  if (!rtcReadControl2(&control2)) return false;
  // COF=111 wylacza wyjscie CLKOUT (nieuzywane) -> mniejszy pobor RTC.
  control2 = (uint8_t)(0x07 | (alarmEnabled ? (1U << 7) : 0));
  Wire.beginTransmission(PCF85063A_ADDR);
  Wire.write(0x01);
  Wire.write(control2);                              // AF=TF=MI=HMI=0, CLKOUT off
  return Wire.endTransmission() == 0;
}

static bool rtcVerifyWakeInterruptConfig(bool alarmEnabled) {
  uint8_t control1, control2;
  if (!rtcReadControl1(&control1) || !rtcReadControl2(&control2)) return false;

  if (!rtcBeginSession()) return false;
  Wire.beginTransmission(PCF85063A_ADDR);
  Wire.write(0x10);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)PCF85063A_ADDR, (uint8_t)2) != 2) return false;
  uint8_t timerValue = Wire.read();
  uint8_t timerMode = Wire.read();

  uint8_t expectedControl2 = alarmEnabled ? (1U << 7) : 0;
  return (control1 & (1U << 2)) == 0 &&              // CIE=0
         (control2 & 0xF8) == expectedControl2 &&    // AIE zgodnie z zadaniem, flagi=0
         timerValue == 0 &&
         timerMode == 0x18;                          // TCF=1/60 Hz, TE=TIE=TI_TP=0
}

static bool rtcVerifyWakeAlarm(time_t epoch) {
  struct tm gm;
  gmtime_r(&epoch, &gm);

  if (!rtcBeginSession()) return false;
  Wire.beginTransmission(PCF85063A_ADDR);
  Wire.write(0x0B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)PCF85063A_ADDR, (uint8_t)5) != 5) return false;

  uint8_t alarmRegs[5];
  for (uint8_t i = 0; i < 5; i++) alarmRegs[i] = Wire.read();
  if (alarmRegs[0] != (dec2bcd((uint8_t)gm.tm_sec) & 0x7F) ||
      alarmRegs[1] != (dec2bcd((uint8_t)gm.tm_min) & 0x7F) ||
      alarmRegs[2] != (dec2bcd((uint8_t)gm.tm_hour) & 0x3F) ||
      alarmRegs[3] != (dec2bcd((uint8_t)gm.tm_mday) & 0x3F) ||
      alarmRegs[4] != 0x80) {
    return false;
  }

  return rtcVerifyWakeInterruptConfig(true);
}

// Alarm PCF85063A pracuje w UTC, tak samo jak rejestry czasu RTC.
// INT jest aktywne stanem niskim i pozostaje niskie do skasowania flagi AF.
static bool rtcSetWakeAlarm(time_t epoch) {
  time_t systemNow = time(nullptr);
  if (!isSupportedEpoch(epoch) || !isSupportedEpoch(systemNow)) return false;
  time_t rtcNow;
  // readRtcEpoch() odrzuca STOP, EXT_TEST i tryb 12 h. Nie uruchamiaj tutaj
  // zatrzymanego RTC, bo jego stary czas moglby zostac uznany za aktualny.
  if (!readRtcEpoch(&rtcNow)) return false;
  long long rtcSkew = (long long)rtcNow - (long long)systemNow;
  if (rtcSkew < -RTC_WAKE_MAX_SKEW_SEC || rtcSkew > RTC_WAKE_MAX_SKEW_SEC) return false;
  if (!rtcConfigureWakeInterrupt(false)) return false;

  struct tm gm;
  gmtime_r(&epoch, &gm);

  Wire.beginTransmission(PCF85063A_ADDR);
  Wire.write(0x0B);
  Wire.write(dec2bcd((uint8_t)gm.tm_sec) & 0x7F);
  Wire.write(dec2bcd((uint8_t)gm.tm_min) & 0x7F);
  Wire.write(dec2bcd((uint8_t)gm.tm_hour) & 0x3F);
  Wire.write(dec2bcd((uint8_t)gm.tm_mday) & 0x3F);
  Wire.write((uint8_t)0x80);                         // weekday alarm disabled
  if (Wire.endTransmission() != 0) return false;

  if (rtcConfigureWakeInterrupt(true) && rtcVerifyWakeAlarm(epoch)) return true;
  rtcConfigureWakeInterrupt(false);
  return false;
}

static bool rtcDisableWakeAlarm() {
  return rtcConfigureWakeInterrupt(false) &&
         rtcVerifyWakeInterruptConfig(false);
}

// ============================================================================
//  Ustawienia (NVS)
// ============================================================================
static void setDefaultAlarms() {
  for (int i = 0; i < ALARM_COUNT; i++) {
    alarms[i].enabled = 0;
    alarms[i].hour = 7;
    alarms[i].minute = 0;
    alarms[i].second = 0;
    alarms[i].volume = DEFAULT_AUDIO_VOLUME;
    alarms[i].sound = (uint8_t)i;          // domyslnie budzik i → dzwiek i
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
  if (alarm.label[0] == '\0') {
    snprintf(alarm.label, sizeof(alarm.label), "Budzik %d", index + 1);
  }
}

static void sanitizeAlarms() {
  for (int i = 0; i < ALARM_COUNT; i++) sanitizeAlarm(alarms[i], i);
}

static void loadSettings() {
  setDefaultAlarms();
  prefs.begin("zegar3", false);
  if (prefs.getBytesLength("alarms") == sizeof(alarms)) {
    prefs.getBytes("alarms", alarms, sizeof(alarms));
  }
  sanitizeAlarms();
  String ssid = prefs.getString("ssid", DEFAULT_WIFI_SSID);
  String pass = prefs.getString("pass", DEFAULT_WIFI_PASS);
  if (ssid.length() == 0) { ssid = DEFAULT_WIFI_SSID; pass = DEFAULT_WIFI_PASS; }
  ssid.toCharArray(wifiSsid, sizeof(wifiSsid));
  pass.toCharArray(wifiPass, sizeof(wifiPass));
  prefs.end();
}

static void saveSettings() {
  prefs.begin("zegar3", false);
  prefs.putBytes("alarms", alarms, sizeof(alarms));
  prefs.putString("ssid", wifiSsid);
  prefs.putString("pass", wifiPass);
  prefs.end();
}

// ============================================================================
//  WiFi + NTP
// ============================================================================
// Ustawiana w callbacku SNTP po realnym odebraniu czasu z serwera (nie z RTC).
static std::atomic<bool> g_ntpGotPacket{false};
static std::atomic<bool> g_ntpAcceptPacket{false};
static void onSntpSync(struct timeval *tv) {
  if (tv && g_ntpAcceptPacket.load(std::memory_order_acquire)) {
    g_ntpGotPacket.store(true, std::memory_order_release);
  }
}

enum NtpSyncState : uint8_t {
  NTP_SYNC_IDLE,
  NTP_SYNC_WIFI_CONNECTING,
  NTP_SYNC_WAITING_PACKET
};

static NtpSyncState ntpSyncState = NTP_SYNC_IDLE;
static bool         ntpManualAttempt = false;
static bool         ntpHadValidTimeBeforeAttempt = false;
static uint32_t     ntpStateStartedMs = 0;
static struct timeval ntpFallbackTime = {};
static uint32_t     ntpFallbackMs = 0;

static void restoreTimeAfterFailedNtp();

static void stopSntpCallbacks() {
  g_ntpAcceptPacket.store(false, std::memory_order_release);
  if (esp_sntp_enabled()) esp_sntp_stop();
  sntp_set_time_sync_notification_cb(nullptr);
}

static void stopWifiAndNtp() {
  bool canceledAttempt = ntpSyncState != NTP_SYNC_IDLE;
  stopSntpCallbacks();
  if (ntpSyncState == NTP_SYNC_WAITING_PACKET) restoreTimeAfterFailedNtp();
  ntpSyncState = NTP_SYNC_IDLE;
  ntpManualAttempt = false;
  g_ntpGotPacket.store(false, std::memory_order_release);
  if (canceledAttempt && ntpRetriesLeft == 0 && !isSupportedEpoch(time(nullptr))) {
    ntpRetriesLeft = 1;
    ntpNextRetryMs = millis() + NTP_INVALID_TIME_RETRY_MS;
  }
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
}

static void wifiBeginSmart() {
#if FAST_NTP_RECONNECT
  if (g_wifiHint) { WiFi.begin(wifiSsid, wifiPass, g_wifiChannel, g_wifiBssid); return; }
#endif
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

static bool ensureWifi(uint32_t timeoutMs) {
  if (strlen(wifiSsid) == 0) return false;
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(true);
  if (WiFi.status() != WL_CONNECTED) {
    wifiBeginSmart();
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
      delay(100);
      yield();
    }
  }
  bool ok = WiFi.status() == WL_CONNECTED;
  if (ok) wifiCacheHint(); else g_wifiHint = false;   // fallback: nastepna proba pelny skan
  return ok;
}

static void scheduleRtcWriteRetry(uint8_t retries = RTC_WRITE_RETRIES) {
  rtcWriteRetriesLeft = retries;
  rtcWriteNextRetryMs = millis() + RTC_WRITE_RETRY_MS;
}

static void handleRtcWriteRetry() {
  if (rtcWriteRetriesLeft == 0 ||
      (int32_t)(millis() - rtcWriteNextRetryMs) < 0) return;

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

static void scheduleNtpRetry(uint8_t retries, uint32_t delayMs = NTP_RETRY_MS) {
  ntpRetriesLeft = retries;
  ntpNextRetryMs = millis() + delayMs;
}

static void restoreTimeAfterFailedNtp() {
  if (isSupportedEpoch(ntpFallbackTime.tv_sec)) {
    uint64_t elapsedUs = (uint64_t)(millis() - ntpFallbackMs) * 1000ULL;
    struct timeval restored = ntpFallbackTime;
    restored.tv_sec += (time_t)(elapsedUs / 1000000ULL);
    restored.tv_usec += (suseconds_t)(elapsedUs % 1000000ULL);
    if (restored.tv_usec >= 1000000) {
      restored.tv_sec++;
      restored.tv_usec -= 1000000;
    }
    settimeofday(&restored, nullptr);
  } else {
    setSystemEpoch(0);
  }
}

static void finishNtpAttempt(bool ok) {
  bool wasManual = ntpManualAttempt;
  ntpSyncState = NTP_SYNC_IDLE;
  ntpManualAttempt = false;
  stopSntpCallbacks();
  g_ntpGotPacket.store(false, std::memory_order_release);

  if (ok) {
    ntpRetriesLeft = 0;
  } else {
    if (!wasManual && ntpRetriesLeft > 0) {
      // ntpFailureRetryDelayMs obowiazuje przez CALA serie ponowien (np. 6x30 s po
      // zimnym starcie bez RTC). Wczesniejszy reset na NTP_RETRY_MS po pierwszym
      // ponowieniu rozciagal odzysk czasu po awarii zasilania do ~26 minut.
      ntpNextRetryMs = millis() + ntpFailureRetryDelayMs;
    } else if (!ntpHadValidTimeBeforeAttempt || !isSupportedEpoch(time(nullptr))) {
      // Bez poprawnego RTC nie ma lokalnej daty, wiec ponawiaj az NTP zadziala.
      scheduleNtpRetry(1, NTP_INVALID_TIME_RETRY_MS);
    }
  }

  if (!webActive) {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
  }
  displayDirty = true;
}

static void beginNtpRequest() {
  // Callback odroznia realny pakiet NTP od czasu ustawionego wczesniej z RTC.
  g_ntpAcceptPacket.store(false, std::memory_order_release);
  g_ntpGotPacket.store(false, std::memory_order_release);
  sntp_set_time_sync_notification_cb(onSntpSync);   // przed configTzTime
  sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED);
  gettimeofday(&ntpFallbackTime, nullptr);
  ntpFallbackMs = millis();
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
  ntpHadValidTimeBeforeAttempt = isSupportedEpoch(time(nullptr));
  ntpStatus = manual ? "manual: laczenie WiFi" : "auto: laczenie WiFi";
  displayDirty = true;

  if (strlen(wifiSsid) == 0) {
    ntpStatus = "brak SSID WiFi";
    finishNtpAttempt(false);
    return false;
  }

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(true);
  if (WiFi.status() == WL_CONNECTED) {
    beginNtpRequest();
  } else {
    wifiBeginSmart();
    ntpSyncState = NTP_SYNC_WIFI_CONNECTING;
    ntpStateStartedMs = millis();
  }
  return true;
}

static bool serviceNtp() {
  if (ntpSyncState == NTP_SYNC_WIFI_CONNECTING) {
    if (WiFi.status() == WL_CONNECTED) {
      wifiCacheHint();
      beginNtpRequest();
    } else if (millis() - ntpStateStartedMs >= NTP_WIFI_TIMEOUT_MS) {
      ntpStatus = "blad WiFi";
      g_wifiHint = false;
      finishNtpAttempt(false);
    }
    return false;
  }

  if (ntpSyncState != NTP_SYNC_WAITING_PACKET) return false;

  if (g_ntpGotPacket.load(std::memory_order_acquire)) {
    // Po pierwszej odpowiedzi nie pozwalaj kolejnemu pakietowi zmienic czasu
    // pomiedzy jego odczytem a zatwierdzeniem tego samego czasu w RTC.
    stopSntpCallbacks();
    time_t syncedEpoch = time(nullptr);
    struct tm local = {};
    if (!isSupportedEpoch(syncedEpoch)) {
      restoreTimeAfterFailedNtp();
      ntpStatus = "bledny czas NTP";
      finishNtpAttempt(false);
      return false;
    }
    localtime_r(&syncedEpoch, &local);

    bool rtcOk = writeRtc(syncedEpoch);
    if (rtcOk) rtcWriteRetriesLeft = 0;
    else scheduleRtcWriteRetry();
    char buf[48];
    snprintf(buf, sizeof(buf), "OK %02d:%02d:%02d RTC:%s",
             local.tm_hour, local.tm_min, local.tm_sec, rtcOk ? "OK" : "RETRY");
    ntpStatus = buf;
    ntpStartedDay = localDayKey(local);
    finishNtpAttempt(true);
    return true;
  }

  if (millis() - ntpStateStartedMs >= NTP_PACKET_TIMEOUT_MS) {
    // Zatrzymaj SNTP i cofnij ewentualna spozniona/niepelna zmiane czasu.
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
//  System plikow z dzwiekami
// ============================================================================
// Dokoncz przerwana zanikiem zasilania podmiane pliku dzwiekowego. Skan
// calego katalogu obejmuje takze MP3 wgrane przez WWW, nie tylko
// /alarm1-3.mp3: *.bak bez pliku docelowego wraca na swoje miejsce,
// osierocone *.bak/*.part sa usuwane.
// Jeden przebieg; true po pierwszej zmianie w FS. Iterowanie katalogu w
// trakcie kasowania/przenoszenia bywa zawodne (SPIFFS), wiec po kazdej
// zmianie skan startuje od poczatku.
static bool recoverSoundFilesOnce() {
  File root = dataFS->open("/");
  if (!root) return false;
  for (File f = root.openNextFile(); f; f = root.openNextFile()) {
    bool isDir = f.isDirectory();
    String path = f.name();
    f.close();
    if (isDir) continue;
    if (!path.startsWith("/")) path = String("/") + path;
    String lower = path;
    lower.toLowerCase();

    if (lower.endsWith(".bak") && path.length() > 5) {
      String target = path.substring(0, path.length() - 4);
      bool changed = dataFS->exists(target) ? dataFS->remove(path)
                                            : dataFS->rename(path, target);
      if (changed) { root.close(); return true; }
    } else if (lower.endsWith(".part") && path.length() > 6) {
      String target = path.substring(0, path.length() - 5);
      String backup = target + ".bak";
      // Najpierw galaz .bak musi odtworzyc plik docelowy (kolejny przebieg);
      // dopiero wtedy .part jest bezpiecznie zbedny.
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
  // Fabrycznie czysty flash / po erase_flash: partycja danych niesformatowana,
  // a MP3 mozna wgrac tylko przez WWW, ktore wymaga FS -> pat bez wyjscia.
  // Jednorazowy format SPIFFS go przerywa; nic sie nie zamontowalo, wiec nie
  // ma danych do stracenia.
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
//  Audio
// ============================================================================
static void audioInfo(Audio::msg_t m) {
  Serial.printf("Audio %s: %s\n", m.s ? m.s : "", m.msg ? m.msg : "");
}

static bool audioPinsReady = false;

// Konfiguracja I2S wykonywana tylko raz. Przy ponownym budzeniu kodeka NIE
// przeinicjowujemy sterownika I2S — odswiezamy jedynie ES8311 przez codec.begin().
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
  // Wzmacniacz wlacza dopiero startSound() PO pelnej konfiguracji kodeka.
  // PA aktywny podczas resetu ES8311 = trzask w glosniku przed kazdym dzwonieniem.
  digitalWrite(PA_ENABLE_PIN, LOW);

  ensureAudioPins();

  audioReady = codec.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000);
  if (audioReady) {
    codec.setVolume(CODEC_VOLUME_PERCENT);
    codec.configBits16();
  } else {
    Serial.println("ES8311 init failed");
    digitalWrite(PA_ENABLE_PIN, LOW);
  }
  return audioReady;
}

// Uspij kodek po zakonczeniu odtwarzania/dzwonienia. ES8311 jest zasilany ze
// stalej szyny 3V3 (snu CPU go nie wylacza), wiec to najwiekszy odbiornik w
// spoczynku. Kolejny dzwiek odtworzy startSound() -> ensureCodec() (pelna
// reinicjalizacja kodeka), wiec zadna funkcja nie znika.
static void shutdownCodec() {
  if (!audioReady) {
    digitalWrite(PA_ENABLE_PIN, LOW);
    return;
  }
  audio.stopSong();
  codec.powerDown();
  digitalWrite(PA_ENABLE_PIN, LOW);
  audioReady = false;
}

// Odtworz wskazany plik dzwiekowy jednokrotnie.
static bool startSound(uint8_t soundIndex, uint8_t volume) {
  stopPlayback(false);
  if (soundIndex >= SOUND_COUNT) soundIndex = 0;
  if (!ensureFsMounted() || !ensureCodec()) {
    statusText = "blad FS/kodek";
    displayDirty = true;
    return false;
  }

  const char *path = SOUND_FILES[soundIndex];
  if (!dataFS->exists(path)) {
    digitalWrite(PA_ENABLE_PIN, LOW);   // nie zostawiaj PA wlaczonego po nieudanym starcie (syk + pobor przez cale okno WWW)
    statusText = String("brak ") + path;
    displayDirty = true;
    return false;
  }

  digitalWrite(PA_ENABLE_PIN, HIGH);
  audio.stopSong();
  audio.setVolume(constrain(volume, 0, MAX_AUDIO_VOLUME));
  codec.setVolume(CODEC_VOLUME_PERCENT);
  if (!audio.connecttoFS(*dataFS, path)) {
    digitalWrite(PA_ENABLE_PIN, LOW);
    statusText = String("blad odtw. ") + path;
    displayDirty = true;
    return false;
  }

  soundStartedMs = millis();
  playing = true;
  Serial.printf("Odtwarzam %s vol=%u\n", path, volume);
  displayDirty = true;
  return true;
}

static void stopPlayback(bool userStopped) {
  if (userStopped) {
    ringing = false;
    ringingAlarm = -1;
    ringErrors = 0;
  }
  if (audioReady) {
    audio.stopSong();
    audio.setVolume(0);
    codec.setVolume(0);
    digitalWrite(PA_ENABLE_PIN, LOW);
  }
  if (playing) {
    statusText = userStopped ? "STOP" : "koniec";
    playing = false;
    displayDirty = true;
  }
}

// Wejscie w stan dzwonienia: ponawiaj odtwarzanie wybranego dzwieku az do wyciszenia.
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
  if (!playing || !audioReady) return;
  audio.loop();
  if (!audio.isRunning()) {
    if (ringing) {
      // Petla dzwonienia obsluzy ponowne odtworzenie w serviceRinging().
      // connecttoFS() zwraca true dla KAZDEGO istniejacego pliku, a uszkodzony
      // MP3 "konczy sie" po kilkudziesieciu ms. Bez zliczania takich prob budzik
      // krecil sie bezglosnie przez cale RING_TIMEOUT_MS (15 min ciszy).
      playing = false;
      if (millis() - soundStartedMs < RING_MIN_PLAY_MS) {
        if (++ringErrors >= RING_MAX_ERRORS) {
          statusText = "budzik: blad MP3";
          ringing = false;
          ringingAlarm = -1;
        }
      } else {
        ringErrors = 0;
      }
      displayDirty = true;
      return;
    }
    stopPlayback(false);
  }
}

static void serviceRinging() {
  if (!ringing) return;

  if (millis() - ringStartMs > RING_TIMEOUT_MS) {
    statusText = "budzik: timeout";
    stopPlayback(false);
    ringing = false;
    ringingAlarm = -1;
    displayDirty = true;
    return;
  }
  if (playing) return;
  if (millis() - ringLastRetryMs < RING_RETRY_MS) return;

  ringLastRetryMs = millis();
  if (ringingAlarm >= 0 &&
      startSound(alarms[ringingAlarm].sound, alarms[ringingAlarm].volume)) {
    // ringErrors zeruje dopiero serviceAudio() po odtworzeniu >= RING_MIN_PLAY_MS;
    // sukces connecttoFS() niczego nie dowodzi (uszkodzony plik tez "startuje").
    return;
  }
  if (++ringErrors >= RING_MAX_ERRORS) {
    statusText = "budzik: blad MP3";
    ringing = false;
    ringingAlarm = -1;
    displayDirty = true;
  }
}

// ============================================================================
//  Akumulator
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
    adc_oneshot_del_unit(batteryAdc);   // bez tego uchwyt jednostki ADC wycieka przy kazdej probie
    batteryAdc = nullptr;
    return false;
  }

  adc_cali_curve_fitting_config_t caliCfg = {};
  caliCfg.unit_id = ADC_UNIT_1;
  caliCfg.atten = ADC_ATTEN_DB_12;
  caliCfg.bitwidth = ADC_BITWIDTH_12;
  batteryCaliReady = adc_cali_create_scheme_curve_fitting(&caliCfg, &batteryCali) == ESP_OK;
  batteryReady = true;
  return true;
}

// Krzywa OCV->SoC dla Samsung INR18650-35E (Li-ion NMC), znormalizowana do
// ramki firmware: 3.00 V = 0% (BATTERY_EMPTY_VOLTAGE), 4.20 V = 100% (BATTERY_FULL_VOLTAGE).
// Rozladowanie Li-ion jest NIELINIOWE: plaski srodek 3.70-3.91 V obejmuje az ~43-71% SoC,
// a konce sa strome. Liniowe mapowanie napiecia zawyzalo srodek i zanizalo okolice pelna.
// Interpolacja liniowa miedzy punktami tabeli. Koszt: kilka porownan co 10 min -> zero
// dodatkowego poboru pradu (brak licznika kulombow, brak rezystora pomiarowego).
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

// Zapis zwiezkiego podsumowania zuzycia do NVS najwyzej raz na dobe (nadpisywany,
// ograniczone zuzycie flash). Przezywa wymiane ogniwa.
static void maybeWriteDailySummary(time_t now) {
  struct tm t; localtime_r(&now, &t);
  int day = localDayKey(t);
  if (day == g_lastSummaryDay) return;
  float perDay = drainMahPerDay(g_drain);
  if (perDay < 0.0f) return;                  // brak sensownej estymaty
  g_lastSummaryDay = day;
  prefs.begin("zegar3", false);
  prefs.putUShort("drnSoc", batteryPercent);
  prefs.putFloat("drnMahDay", perDay);
  prefs.putUInt("drnDay", (uint32_t)day);
  prefs.end();
  Serial.printf("Zuzycie: %.1f mAh/dzien, ~%.0f dni\n",
                perDay, drainDaysRemaining(g_drain, batteryPercent));
}

static void updateBattery(bool force) {
  if (!force && (int32_t)(millis() - batteryNextReadMs) < 0) return;

  // Idle-gating: pod obciazeniem (odtwarzanie MP3, dzwoniacy budzik, aktywny serwer web)
  // napiecie zacisku siada przez rezystancje wewnetrzna ogniwa (~50 mOhm) -> pomiar
  // zanizony, krzywa OCV->SoC zaklada napiecie spoczynkowe. Odlozyc pomiar do bezczynnosci,
  // ponawiajac szybko (BATTERY_BUSY_RETRY_MS) zamiast kasowac caly interwal.
  // force pomija bramke: cold boot / wybudzenie potrzebuja wartosci od razu.
  if (!force && (playing || ringing || webActive)) {
    batteryNextReadMs = millis() + BATTERY_BUSY_RETRY_MS;
    return;
  }

  batteryNextReadMs = millis() + BATTERY_READ_INTERVAL_MS;
  if (!initBattery()) {
    batteryNextReadMs = millis() + BATTERY_BUSY_RETRY_MS;
    return;
  }

  uint32_t sum = 0;
  uint8_t samples = 0;
  for (uint8_t i = 0; i < BATTERY_SAMPLE_COUNT; i++) {
    int raw = 0;
    if (adc_oneshot_read(batteryAdc, BATTERY_ADC_CHANNEL, &raw) == ESP_OK) {
      sum += (uint32_t)raw;
      samples++;
    }
  }
  if (samples == 0) {
    batteryNextReadMs = millis() + BATTERY_BUSY_RETRY_MS;
    return;
  }

  int raw = (int)((sum + samples / 2) / samples);
  int mv = 0;
  if (batteryCaliReady) adc_cali_raw_to_voltage(batteryCali, raw, &mv);
  else mv = (raw * 3300) / 4095;

  float measured = (mv * 0.001f) * BATTERY_DIVIDER_RATIO * BATTERY_VOLTAGE_CALIBRATION;
  if (!batteryHasReading || force) { batteryVoltage = measured; batteryHasReading = true; }
  else batteryVoltage = batteryVoltage * 0.80f + measured * 0.20f;

  uint8_t old = batteryPercent;
  batteryPercent = stabilizePercent(percentFromVoltage(batteryVoltage));
  time_t nowT = time(nullptr);
  if (isSupportedEpoch(nowT)) {
    drainOnSample(g_drain, (int64_t)nowT, batteryPercent);
    maybeWriteDailySummary(nowT);
  }
  if (force || old != batteryPercent) displayDirty = true;
}

// ============================================================================
//  Rysowanie (U8g2)
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
  if (!colon && glyph == 4) {          // pelna stopka cyfry 4 (font ma ja skrocona)
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

// Najblizszy wlaczony budzik (dzis lub jutro). Zwraca indeks lub -1.
static int nextAlarmIndex(uint8_t *h, uint8_t *mi, uint8_t *s, char *label, size_t labelSize) {
  struct tm t;
  bool hasTime = getLocalTm(&t);
  time_t now = hasTime ? time(nullptr) : 0;
  // Ranking po rzeczywistym epoch (mktime), nie po sekundach doby: w noc zmiany
  // czasu kolejnosc wg godzin scianowych klamie (02:45 dzwoni PO 03:15), a ekran
  // pokazywalby inny budzik niz ten, ktory faktycznie odpali pierwszy.
  long bestDelta = LONG_MAX;
  int best = -1;
  for (int i = 0; i < ALARM_COUNT; i++) {
    if (!alarms[i].enabled) continue;
    long delta;
    if (hasTime) {
      time_t cand = epochForClock(t, alarms[i].hour, alarms[i].minute, alarms[i].second, 0);
      if (cand <= now) cand = epochForClock(t, alarms[i].hour, alarms[i].minute, alarms[i].second, 1);
      delta = (long)(cand - now);
    } else {
      delta = i;
    }
    if (delta < bestDelta) {
      bestDelta = delta;
      best = i;
      if (h) *h = alarms[i].hour;
      if (mi) *mi = alarms[i].minute;
      if (s) *s = alarms[i].second;
      if (label && labelSize) { strncpy(label, alarms[i].label, labelSize - 1); label[labelSize - 1] = 0; }
    }
  }
  return best;
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
  if (next >= 0) {
    snprintf(bottom, sizeof(bottom), "Budzik %02u:%02u %s  Aku %u%%  WWW:%s",
             nh, nm, alarmOn ? "ON" : "OFF", batteryPercent, webActive ? "ON" : "OFF");
  } else {
    snprintf(bottom, sizeof(bottom), "Budzik --:-- OFF  Aku %u%%  WWW:%s",
             batteryPercent, webActive ? "ON" : "OFF");
  }
  gfx->drawStr(42, 287, bottom);
  gfx->sendBuffer();
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
}

static void drawNightPage() {
  gfx->clearBuffer();
  gfx->setDrawColor(1);
  gfx->setFont(u8g2_font_7x14_tf);
  drawCentered("TRYB NOCNY 23:59 - 06:00", 150);
  gfx->sendBuffer();
  displayDirty = false;
}

static void updateDisplay() {
  if (playing || ringing) {
    if (displayDirty) { drawRingingPage(); displayDirty = false; }
    return;
  }
  if (previewActive) {
    if ((int32_t)(millis() - previewUntilMs) < 0) {
      if (displayDirty) { drawPreviewPage(); displayDirty = false; }
      return;
    }
    previewActive = false;
    displayDirty = true;
  }

  struct tm t;
  if (!getLocalTm(&t)) {
    if (displayDirty) {
      gfx->clearBuffer();
      gfx->setDrawColor(1);
      gfx->setFont(u8g2_font_9x15_tf);
      drawCentered("Brak czasu RTC/NTP", 135);
      drawCentered(ntpStatus.c_str(), 170);
      gfx->sendBuffer();
      displayDirty = false;
    }
    return;
  }

  int day = localDayKey(t);
  if (displayDirty || t.tm_min != lastDrawMinute || day != lastDrawDay) {
    drawClockPage(t);
    lastDrawMinute = t.tm_min;
    lastDrawDay = day;
    displayDirty = false;
  }
}

// ============================================================================
//  Panel WWW
// ============================================================================
static String htmlEscape(const String &v) {
  String out;
  out.reserve(v.length() + 8);
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
  int hh = 0, mm = 0, ss = 0;
  int consumed = 0;
  int n = sscanf(v.c_str(), "%d:%d:%d%n", &hh, &mm, &ss, &consumed);
  if (n == 2) {
    ss = 0;
    consumed = 0;
    if (sscanf(v.c_str(), "%d:%d%n", &hh, &mm, &consumed) != 2) return false;
  }
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

// keepCase: kasowanie istniejacych plikow musi uzyc dokladnie tej nazwy, ktora
// pokazuje lista WWW (SPIFFS/LittleFS rozroznia wielkosc liter; plik /Alarm.MP3
// wgrany narzedziem PC bylby nieusuwalny po zlowercase'owaniu). Upload nadal
// normalizuje nazwy do malych liter.
static bool normalizeMp3Path(String name, String &path, bool keepCase) {
  name.trim();
  name.replace("\\", "/");
  int slash = name.lastIndexOf('/');
  if (slash >= 0) name = name.substring(slash + 1);
  name.trim();
  // SPIFFS ogranicza pelna sciezke do 31 znakow, a upload pisze "/<nazwa>.part"
  // (1 + nazwa + 5). Dluzsze nazwy przechodzily walidacje i padaly dopiero na
  // open() z mylacym "blad zapisu MP3".
  if (name.length() < 5 || name.length() > 25) return false;

  String lower = name;
  lower.toLowerCase();
  if (!lower.endsWith(".mp3")) return false;
  if (!keepCase) name = lower;

  for (size_t i = 0; i < name.length(); i++) {
    if (!isMp3NameChar(name[i])) return false;
  }

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
  if (!ensureFsMounted()) {
    html += F("<p>Brak zamontowanego systemu plikow.</p></section>");
    return;
  }

  html += F("<p class='muted'>System plikow: ");
  html += htmlEscape(dataFSName);
  html += F("</p><form method='post' action='/mp3upload' enctype='multipart/form-data'>"
            "<div class='row'><input type='file' name='mp3' accept='.mp3,audio/mpeg' required>"
            "<button type='submit'>Dodaj / nadpisz MP3</button></div></form>");

  html += F("<table><tr><th>Plik</th><th>Rozmiar</th><th></th></tr>");
  bool any = false;
  File root = dataFS->open("/");
  if (root) {
    File file = root.openNextFile();
    while (file) {
      if (!file.isDirectory()) {
        String path = file.name();
        if (!path.startsWith("/")) path = String("/") + path;
        String lower = path;
        lower.toLowerCase();
        if (lower.endsWith(".mp3")) {
          any = true;
          html += F("<tr><td>");
          html += htmlEscape(path);
          html += F("</td><td>");
          html += formatFileSize(file.size());
          html += F("</td><td><form method='post' action='/mp3delete' style='display:inline'>"
                    "<input type='hidden' name='file' value='");
          html += htmlEscape(path);
          html += F("'><button type='submit'>Usun</button></form></td></tr>");
        }
      }
      file.close();
      file = root.openNextFile();
    }
    root.close();
  }
  if (!any) html += F("<tr><td colspan='3'>Brak plikow MP3</td></tr>");
  html += F("</table><p class='muted'>Nazwy plikow: litery/cyfry oraz _ - ., maks. 25 znakow; budziki domyslnie uzywaja /alarm1.mp3, /alarm2.mp3 i /alarm3.mp3.</p></section>");
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
         "</style></head><body><main><h1>Zegar NTP — 3 budziki</h1>");
}

static void handleRoot() {
  webUntilMs = millis() + WEB_WINDOW_MS;
  // Nie wymuszaj pomiaru podczas pracy WiFi. OCV mierzone pod obciazeniem
  // zaniza procent akumulatora; updateBattery() wykona pomiar po zamknieciu WWW.
  updateBattery(false);
  String html;
  html.reserve(14000);
  pageHeader(html);

  html += F("<section><b>Status</b><p>Czas: ");
  html += currentTimeText();
  html += F("<br>Budzik: ");        html += htmlEscape(statusText);
  html += F("<br>NTP: ");           html += htmlEscape(ntpStatus);
  html += F("<br>Pliki MP3: ");     html += htmlEscape(dataFSName);
  html += F("<br>Akumulator: ");    html += batteryPercent;
  html += F("% (");                 html += String(batteryVoltage, 2);
  html += F(" V, ok. ");            html += String((uint32_t)batteryPercent * BATTERY_CAPACITY_MAH / 100U);
  html += F(" mAh)<br>IP: ");       html += WiFi.localIP().toString();
  {
    float perDay = drainMahPerDay(g_drain);
    float daysLeft = drainDaysRemaining(g_drain, batteryPercent);
    html += F("<br>Zuzycie: ");
    if (perDay < 0)        html += F("zbieram dane (min ~6 h)");
    else if (perDay == 0)  html += F("brak mierzalnego zuzycia");
    else {
      html += String(perDay, 1); html += F(" mAh/dzien, ~");
      html += String(daysLeft, 0); html += F(" dni do rozladowania");
    }
  }
  html += F("</p><p><a href='/sync'><button>Synchronizuj NTP</button></a>"
            "<a href='/stop'><button>Stop MP3</button></a></p></section>");

  html += F("<form method='post' action='/save'>");

  // Hasla nie wpisujemy w HTML: kazdy w LAN moglby je odczytac ze zrodla
  // strony (panel nie ma logowania). Puste pole przy zapisie = bez zmian.
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
  html += F("</table><p class='muted'>Dzwiek 1/2/3 = pliki /alarm1.mp3 /alarm2.mp3 /alarm3.mp3 na partycji danych.</p></section>");

  html += F("<p><button type='submit'>Zapisz</button></p></form>");

  appendMp3Manager(html);

  html += F("<section><b>Test dzwiekow</b><div class='row'>");
  for (int sIdx = 0; sIdx < SOUND_COUNT; sIdx++) {
    html += F("<form method='post' action='/test' style='display:inline'>"
              "<input type='hidden' name='snd' value='"); html += sIdx;
    html += F("'><button type='submit'>Test "); html += SOUND_NAMES[sIdx];
    html += F("</button></form>");
  }
  html += F("</div></section>");

  html += F("</main></body></html>");
  server.send(200, "text/html", html);
}

static void handleSave() {
  webUntilMs = millis() + WEB_WINDOW_MS;
  if (server.hasArg("ssid")) {
    String ssid = server.arg("ssid"); ssid.trim();
    ssid.toCharArray(wifiSsid, sizeof(wifiSsid));
  }
  // Formularz nie odsyla biezacego hasla (pole w HTML jest puste), wiec
  // puste pole oznacza "zostaw stare", a nie "ustaw puste".
  if (server.hasArg("pass") && server.arg("pass").length() > 0) {
    String pass = server.arg("pass");
    pass.toCharArray(wifiPass, sizeof(wifiPass));
  }

  for (int i = 0; i < ALARM_COUNT; i++) {
    String idx = String(i);
    uint8_t prevEnabled = alarms[i].enabled;
    uint8_t prevH = alarms[i].hour, prevM = alarms[i].minute, prevS = alarms[i].second;

    alarms[i].enabled = server.hasArg(String("en") + idx) ? 1 : 0;
    uint8_t h, m, s;
    if (parseClock(server.arg(String("time") + idx), &h, &m, &s)) {
      alarms[i].hour = h; alarms[i].minute = m; alarms[i].second = s;
    }

    // Ponowne uzbrojenie: jesli zmieniono czas lub budzik wlaczono na nowo,
    // skasuj znacznik "juz dzis dzwonil", aby mogl odpalic ponownie tego dnia.
    bool timeChanged = (alarms[i].hour != prevH || alarms[i].minute != prevM ||
                        alarms[i].second != prevS);
    bool reEnabled   = (alarms[i].enabled && !prevEnabled);
    if (timeChanged || reEnabled) lastTriggerDay[i] = -1;

    String volumeArg = String("vol") + idx;
    String soundArg = String("snd") + idx;
    String labelArg = String("label") + idx;
    if (server.hasArg(volumeArg)) {
      alarms[i].volume = constrain(server.arg(volumeArg).toInt(), 0, MAX_AUDIO_VOLUME);
    }
    if (server.hasArg(soundArg)) {
      alarms[i].sound = constrain(server.arg(soundArg).toInt(), 0, SOUND_COUNT - 1);
    }
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

static void handleSync() {
  syncNtp(true);
  server.sendHeader("Location", "/");
  server.send(303, "text/plain", "");
}

static void handleStop() {
  stopPlayback(true);
  server.sendHeader("Location", "/");
  server.send(303, "text/plain", "");
}

static void handleTest() {
  uint8_t sIdx = constrain(server.arg("snd").toInt(), 0, SOUND_COUNT - 1);
  startSound(sIdx, DEFAULT_AUDIO_VOLUME);
  server.sendHeader("Location", "/");
  server.send(303, "text/plain", "");
}

static void handleMp3UploadDone() {
  server.sendHeader("Location", "/");
  server.send(303, "text/plain", "");
}

static void handleMp3UploadStream() {
  webUntilMs = millis() + WEB_WINDOW_MS;
  HTTPUpload &upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    if (mp3UploadFile) mp3UploadFile.close();
    mp3UploadAccepted = false;
    mp3UploadHadData = false;
    mp3UploadPath = "";
    mp3UploadTempPath = "";

    if (!normalizeMp3Path(upload.filename, mp3UploadPath, false)) {
      statusText = "zla nazwa MP3";
      displayDirty = true;
      return;
    }
    if (!ensureFsMounted()) {
      statusText = "brak FS dla MP3";
      displayDirty = true;
      return;
    }

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
      if (written != upload.currentSize) {
        mp3UploadAccepted = false;
        statusText = "blad zapisu MP3";
      } else if (upload.currentSize > 0) {
        mp3UploadHadData = true;
      }
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (mp3UploadFile) mp3UploadFile.close();
    if (mp3UploadAccepted && mp3UploadHadData) {
      String backupPath = mp3UploadPath + ".bak";
      if (dataFS->exists(backupPath)) dataFS->remove(backupPath);

      bool hadOld = dataFS->exists(mp3UploadPath);
      bool oldMoved = !hadOld ||
                      dataFS->rename(mp3UploadPath, backupPath);
      bool newInstalled = oldMoved &&
                          dataFS->rename(mp3UploadTempPath, mp3UploadPath);

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
      if (mp3UploadTempPath.length() && dataFS && dataFS->exists(mp3UploadTempPath)) {
        dataFS->remove(mp3UploadTempPath);
      }
      if (statusText == String("upload ") + mp3UploadPath) statusText = "pusty MP3";
    }
    displayDirty = true;
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    if (mp3UploadFile) mp3UploadFile.close();
    if (mp3UploadTempPath.length() && dataFS && dataFS->exists(mp3UploadTempPath)) {
      dataFS->remove(mp3UploadTempPath);
    }
    statusText = "upload przerwany";
    displayDirty = true;
  }
}

static void handleMp3Delete() {
  String path;
  if (!normalizeMp3Path(server.arg("file"), path, true)) {
    statusText = "zla nazwa MP3";
  } else if (!ensureFsMounted()) {
    statusText = "brak FS dla MP3";
  } else {
    if (playing || ringing) stopPlayback(true);
    statusText = (dataFS->exists(path) && dataFS->remove(path)) ?
                 String("usunieto ") + path : String("brak ") + path;
  }
  displayDirty = true;
  server.sendHeader("Location", "/");
  server.send(303, "text/plain", "");
}

static bool startWebWindow() {
  if (webActive) {
    if (WiFi.status() == WL_CONNECTED) {
      webUntilMs = millis() + WEB_WINDOW_MS;
      displayDirty = true;
      return true;
    }
    server.stop();
    webActive = false;
  }
  if (!ensureWifi(NTP_WIFI_TIMEOUT_MS)) {
    statusText = "brak polaczenia WiFi";
    stopWifiAndNtp();
    displayDirty = true;
    return false;
  }
  if (!webRoutesInstalled) {
    server.on("/", HTTP_GET, handleRoot);
    server.on("/save", HTTP_POST, handleSave);
    server.on("/sync", HTTP_GET, handleSync);
    server.on("/stop", HTTP_GET, handleStop);
    server.on("/test", HTTP_POST, handleTest);
    server.on("/mp3upload", HTTP_POST, handleMp3UploadDone, handleMp3UploadStream);
    server.on("/mp3delete", HTTP_POST, handleMp3Delete);
    webRoutesInstalled = true;
  }
  server.begin();
  webActive = true;
  webUntilMs = millis() + WEB_WINDOW_MS;
  displayDirty = true;
  Serial.print("WWW IP: ");
  Serial.println(WiFi.localIP());
  return true;
}

static void stopWebWindow() {
  server.stop();
  webActive = false;
  // Aktywna proba NTP trwa najwyzej kilkanascie sekund. Pozwol jej zakonczyc
  // walidacje i zapis RTC; finishNtpAttempt() sam wtedy wylaczy WiFi.
  if (ntpSyncState == NTP_SYNC_IDLE) stopWifiAndNtp();
  displayDirty = true;
}

// ============================================================================
//  Sprawdzanie budzikow i harmonogram NTP
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
    if (alarmDueNow(t, now, alarms[i].hour, alarms[i].minute, alarms[i].second,
                    lateWindowSec, &occurrenceDay) &&
        lastTriggerDay[i] != occurrenceDay) {
      lastTriggerDay[i] = occurrenceDay;
      // Skonsumuj pozostale budziki wymagalne w tej samej chwili (np. kolizja
      // godzin w noc wiosennej zmiany czasu: 02:30 i 03:30 to ten sam epoch).
      // Bez tego drugi z nich po wyciszeniu pierwszego mial juz przeterminowane
      // okno i przepadal bez sladu; swiadomie laczymy je w jedno dzwonienie.
      for (int j = 0; j < ALARM_COUNT; j++) {
        int otherDay;
        if (j != i && alarms[j].enabled &&
            alarmDueNow(t, now, alarms[j].hour, alarms[j].minute, alarms[j].second,
                        lateWindowSec, &otherDay)) {
          lastTriggerDay[j] = otherDay;
        }
      }
      startRinging(i);
      return;
    }
  }
}

static void checkNtpSchedule() {
  if (handleNtpRetry()) return;
  struct tm t;
  if (!getLocalTm(&t)) return;
  int day = localDayKey(t);
  if (t.tm_hour == NTP_HOUR && t.tm_min <= NTP_MIN_WINDOW && ntpStartedDay != day) {
    ntpStartedDay = day;
    ntpRetriesLeft = 2;
    ntpFailureRetryDelayMs = NTP_RETRY_MS;
    syncNtp(false);
  }
}

// ============================================================================
//  Tryb nocny (deep-sleep) i oszczedzanie energii w petli (light-sleep)
// ============================================================================
static bool anyAlarmDueNow(const struct tm &t) {
  time_t now = time(nullptr);
  for (int i = 0; i < ALARM_COUNT; i++) {
    if (alarms[i].enabled &&
        alarmDueNow(t, now, alarms[i].hour, alarms[i].minute, alarms[i].second,
                    ALARM_LATE_WINDOW_SEC, nullptr))
      return true;
  }
  return false;
}

static time_t nightEndEpoch(time_t now) {
  if (!isSupportedEpoch(now)) return 0;
  struct tm t;
  localtime_r(&now, &t);
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
  shutdownCodec();                 // wylacz ES8311 na czas snu nocnego
  pinMode(PA_ENABLE_PIN, OUTPUT);
  digitalWrite(PA_ENABLE_PIN, LOW);
}

static void enterNightSleepIfNeeded(bool force) {
  struct tm t;
  if (!getLocalTm(&t) || !isNightMode(t)) return;
  if (anyAlarmDueNow(t)) return;                       // budzik w oknie — nie spij
  if (!force && (webActive || playing || ringing || previewActive ||
                 ntpSyncState != NTP_SYNC_IDLE || rtcWriteRetriesLeft != 0)) return;

  time_t now = time(nullptr);
  time_t wakeAt = nightEndEpoch(now);
  time_t alarmAt = nextAlarmEpoch(t, now);
  if (alarmAt > now && alarmAt < wakeAt) wakeAt = alarmAt;

  if (wakeAt <= now) wakeAt = now + 6 * 60 * 60;
  uint64_t sleepSec = (uint64_t)(wakeAt - now);
  if (sleepSec < NIGHT_MIN_SLEEP_SEC) {
    sleepSec = NIGHT_MIN_SLEEP_SEC;
    wakeAt = now + (time_t)sleepSec;
  }

  bool rtcAlarmOk = rtcSetWakeAlarm(wakeAt);
  if (rtcAlarmOk && digitalRead(RTC_INT_PIN) != HIGH) {
    rtcDisableWakeAlarm();
    rtcAlarmOk = false;
  }
  // Bez dzialajacego alarmu RTC jedynym budzeniem jest timer ESP taktowany
  // wewnetrznym oscylatorem RC, ktorego dryft przez kilka godzin moze
  // przekroczyc okno NTP_ALARM_CATCHUP_SEC i cicho pominac budzik. Spij wtedy
  // w odcinkach: kazde wybudzenie kotwiczy czas z PCF85063A (readRtcToSystem
  // w setup) i liczy pozostaly sen od nowa, wiec dryft pojedynczego odcinka
  // jest duzo mniejszy niz okno wylapania budzika.
  if (!rtcAlarmOk && sleepSec > NIGHT_FALLBACK_CHUNK_SEC) sleepSec = NIGHT_FALLBACK_CHUNK_SEC;
  Serial.printf("Tryb nocny: sleep %llu s, RTC alarm: %s\n",
                (unsigned long long)sleepSec, rtcAlarmOk ? "OK" : "ERR");
  drawNightPage();
  delay(150);
  shutdownNightPeripherals();
  setCpuFrequencyMhz(NIGHT_CPU_MHZ);
  // Timer ESP32 pozostaje zabezpieczeniem, gdyby przerwanie RTC nie dotarlo.
  esp_sleep_enable_timer_wakeup(sleepSec * 1000000ULL);
  uint64_t wakeMask = (1ULL << KEY_PIN) | (1ULL << PREVIEW_BUTTON_PIN);
  if (rtcAlarmOk) wakeMask |= (1ULL << RTC_INT_PIN);
  esp_sleep_enable_ext1_wakeup(wakeMask, ESP_EXT1_WAKEUP_ANY_LOW);
  rtc_gpio_pullup_en((gpio_num_t)KEY_PIN);  rtc_gpio_pulldown_dis((gpio_num_t)KEY_PIN);
  rtc_gpio_pullup_en((gpio_num_t)PREVIEW_BUTTON_PIN); rtc_gpio_pulldown_dis((gpio_num_t)PREVIEW_BUTTON_PIN);
  if (rtcAlarmOk) {
    rtc_gpio_pullup_en((gpio_num_t)RTC_INT_PIN);
    rtc_gpio_pulldown_dis((gpio_num_t)RTC_INT_PIN);
  }
  holdNightPins();
  Serial.flush();
  esp_deep_sleep_start();
}

// Dynamiczny czas light-sleep: spij do najblizszego zdarzenia, lecz nie dluzej
// niz IDLE_LIGHT_SLEEP_MAX_SEC (60 s). Zdarzenia to:
//   - granica kolejnej minuty -> ekran HH:MM zmienia sie dokladnie na czas,
//   - sekunda najblizszego wlaczonego budzika -> zachowana dokladnosc sekundowa.
// RLCD trzyma ostatni obraz podczas snu, wiec ekran nie gasnie ani nie mruga.
// Przyciski wybudzaja niezaleznie przez EXT1, wiec dluzszy sen ich nie blokuje.
static uint64_t computeIdleSleepUs() {
  time_t now = time(nullptr);
  if (!isSupportedEpoch(now)) return IDLE_LIGHT_SLEEP_US;  // czas nieznany -> krotki sen

  struct tm t;
  localtime_r(&now, &t);

  // 1) Do granicy nastepnej minuty (1..60 s).
  uint64_t sleepSec = 60 - (uint64_t)t.tm_sec;

  // 2) Do sekundy najblizszego budzika, jesli wypada wczesniej.
  time_t alarmAt = nextAlarmEpoch(t, now);
  if (alarmAt > now) {
    uint64_t toAlarm = (uint64_t)(alarmAt - now);
    if (toAlarm < sleepSec) sleepSec = toAlarm;
  }

  // 3) Twardy limit 60 s = sen dokladnie do granicy minuty (bezpiecznik + okresowe
  //    zadania: bateria co 10 min, NTP 10:00, panel WWW). Zegar pokazuje HH:MM, wiec
  //    czesciej niz raz na minute budzic sie nie trzeba; budzik ma osobny, dokladny
  //    termin (pkt 2), a KEY/BOOT wybudzaja przez EXT1 niezaleznie od dlugosci snu.
  if (sleepSec > IDLE_LIGHT_SLEEP_MAX_SEC) sleepSec = IDLE_LIGHT_SLEEP_MAX_SEC;
  if (sleepSec < 1) sleepSec = 1;

  return sleepSec * 1000000ULL;
}

static void idlePowerSave() {
  // Nie usypiaj na dlugo, gdy cos wymaga szybkiej obslugi w petli: panel WWW,
  // odtwarzanie/dzwonienie, podglad budzika, trwajaca synchronizacja NTP lub
  // zaplanowane zapisy do RTC (ponawiane co kilka sekund).
  if (webActive || playing || ringing || previewActive ||
      ntpSyncState != NTP_SYNC_IDLE || rtcWriteRetriesLeft != 0) { delay(5); return; }
  if (digitalRead(KEY_PIN) == LOW || digitalRead(PREVIEW_BUTTON_PIN) == LOW) { delay(5); return; }

  if (audioReady) shutdownCodec();   // kodek niepotrzebny w spoczynku -> uspij ES8311

  // Dzienna sciezka oszczedzania: WYLACZNIE light-sleep do granicy nastepnej minuty
  // / sekundy budzika. Light-sleep zachowuje RAM i NIE rebootuje, wiec setup() i
  // setupDisplay() (reset RST + display.begin() -> czyszczenie panelu) NIE sa
  // wywolywane ponownie -> ekran odswiezany w miejscu przez sendBuffer() -> BRAK
  // mrugania. Ten sam mechanizm sprawia, ze tryb www on nigdy nie mruga.
  //
  // Swiadomy kompromis: light-sleep pobiera ~1 mA vs ~0.02 mA deep-sleep (krotsza
  // praca na baterii), ale ekran przestaje mrugac przy zmianie minuty oraz przy
  // KEY/BOOT (wybudzenia EXT1 obslugiwane nizej w miejscu, bez rebootu). Nocny
  // deep-sleep (enterNightSleepIfNeeded) pozostaje bez zmian.
  esp_sleep_enable_timer_wakeup(computeIdleSleepUs());
  const uint64_t wakeMask = (1ULL << KEY_PIN) | (1ULL << PREVIEW_BUTTON_PIN);
  esp_err_t extWake = esp_sleep_enable_ext1_wakeup(wakeMask, ESP_EXT1_WAKEUP_ANY_LOW);
#if POWER_DOWN_FLASH_IN_SLEEP
  esp_sleep_pd_config(ESP_PD_DOMAIN_VDDSDIO, ESP_PD_OPTION_OFF);
#endif
  esp_light_sleep_start();
  // ESP32-S3: EXT1 (esp_sleep_enable_ext1_wakeup) zostawia RTC-hold na padach
  // wybudzajacych. Bez zwolnienia KEY_PIN (GPIO18) i PREVIEW_BUTTON_PIN (GPIO0)
  // zostaja zatrzasniete na stanie ze snu -> digitalRead ponizej zwraca wartosc
  // zablokowana, przyciski martwe do restartu. Zwolnij od razu po wybudzeniu.
  rtc_gpio_hold_dis((gpio_num_t)KEY_PIN);
  rtc_gpio_hold_dis((gpio_num_t)PREVIEW_BUTTON_PIN);
  esp_sleep_wakeup_cause_t wakeCause = esp_sleep_get_wakeup_cause();
  uint64_t wakePins = wakeCause == ESP_SLEEP_WAKEUP_EXT1 ?
                      esp_sleep_get_ext1_wakeup_status() : 0;
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
  if (extWake == ESP_OK) esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_EXT1);

  // W light-sleep czas systemowy ESP32 korzysta z mniej dokladnego oscylatora
  // RTC slow clock. Po kazdym wybudzeniu zakotwicz go ponownie w PCF85063A,
  // aby dryft nie przesuwal zmiany minuty o kilka lub kilkadziesiat sekund.
  readRtcToSystem();

  if (wakeCause == ESP_SLEEP_WAKEUP_EXT1) {
    if (wakePins & (1ULL << KEY_PIN)) {
      keyRaw = digitalRead(KEY_PIN); keyStable = keyRaw; keyChangedMs = millis();
      startWebWindow();
    }
    if (wakePins & (1ULL << PREVIEW_BUTTON_PIN)) {
      bootRaw = LOW; bootStable = LOW; bootPressedMs = 0; bootChangedMs = millis();
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
  displayDirty = true;
}

static void handleKey() {
  bool raw = digitalRead(KEY_PIN);
  if (raw != keyRaw) { keyRaw = raw; keyChangedMs = millis(); }
  if (millis() - keyChangedMs > BUTTON_DEBOUNCE_MS && raw != keyStable) {
    keyStable = raw;
    if (keyStable == LOW) {
      if (playing || ringing) stopPlayback(true);   // wycisz budzik
      else startWebWindow();                         // otworz panel WWW
    }
  }
}

static void handleBoot() {
  bool raw = digitalRead(PREVIEW_BUTTON_PIN);
  if (raw != bootRaw) { bootRaw = raw; bootChangedMs = millis(); }
  if (millis() - bootChangedMs > BUTTON_DEBOUNCE_MS && raw != bootStable) {
    bootStable = raw;
    if (bootStable == LOW) {
      bootPressedMs = millis();
    } else {
      uint32_t press = millis() - bootPressedMs;
      if (bootPressedMs != 0 && press <= BOOT_SHORT_PRESS_MS) showPreview();
      bootPressedMs = 0;
    }
  }
}

// ============================================================================
//  Inicjalizacja wyswietlacza
// ============================================================================
//   Pierwotnie cieple wybudzenie uzywalo display.initInterface() (tylko piny SPI,
//   bez init-seq) zakladajac, ze panel trzyma obraz przez sen. Na uzywanym
//   egzemplarzu ST7305 to NIE dziala: po deep-sleep panel jest martwy i sam
//   initInterface() go nie przywraca -> w trybie www off ekran zostaje czarny.
//   Dlatego pelny re-init (reset RST + display.begin()) przy KAZDYM wybudzeniu.
// Zarezerwowane: ST7305 HPM/LPM. Domyslnie nieaktywne (patrz spec 5.4). Bez sprzetowego
// testu nie wysylamy niezweryfikowanych komend do panelu.
static inline void st7305SetStaticLowPower(bool enable) {
#if ST7305_STATIC_LPM
  (void)enable;   // celowo puste do czasu testu sprzetowego
#else
  (void)enable;
#endif
}

static void setupDisplay(bool coldBoot) {
  // Ten egzemplarz ST7305 NIE przywraca obrazu po deep-sleep przez sam
  // initInterface() -> w trybie www off (deep-sleep) ekran zostaje czarny.
  // Dlatego pelny re-init (reset + begin) przy KAZDYM wybudzeniu. Panel po snie
  // jest i tak czarny, wiec wzgledem stanu po snie nie ma dodatkowego mrugania.
  (void)coldBoot;
  pinMode(RLCD_RST_PIN, OUTPUT);
  digitalWrite(RLCD_RST_PIN, HIGH);
  digitalWrite(RLCD_RST_PIN, LOW);
  delay(10);
  digitalWrite(RLCD_RST_PIN, HIGH);
  delay(10);
  // Sprzetowe SPI usuwa ok. 4 s opoznienia pelnego sendBuffer(), ktore przy
  // programowym SPI przesuwalo widoczna zmiane minuty.
  SPI.begin(RLCD_SCK_PIN, -1, RLCD_MOSI_PIN, RLCD_CS_PIN);
  display.setBusClock(2000000UL);      // zalecenie sterownika U8g2 dla ST7305
  display.begin();                     // initDisplay + clearDisplay + powerSave(0): panel zawsze zapalony po wybudzeniu
  gfx = &display;
  gfx->setContrast(255);
}

// ============================================================================
//  setup / loop
// ============================================================================
void setup() {
  // Przyczyne wybudzenia ustalamy NAJPIERW - decyduje o delay(300) i sposobie
  // inicjalizacji ekranu (zimny start vs cieple wybudzenie z deep-sleep).
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  uint64_t ext1WakePins = (cause == ESP_SLEEP_WAKEUP_EXT1) ? esp_sleep_get_ext1_wakeup_status() : 0;
  bool coldBoot = (cause == ESP_SLEEP_WAKEUP_UNDEFINED);
  bool keyWake = coldBoot || cause == ESP_SLEEP_WAKEUP_EXT0 ||
                 ((ext1WakePins & (1ULL << KEY_PIN)) != 0);
  bool previewWake = (ext1WakePins & (1ULL << PREVIEW_BUTTON_PIN)) != 0;
  bool rtcWake = (ext1WakePins & (1ULL << RTC_INT_PIN)) != 0;
  bool userWake = keyWake || previewWake;
  bool scheduledWake = rtcWake || cause == ESP_SLEEP_WAKEUP_TIMER;

  Serial.begin(115200);
  if (coldBoot) delay(300);            // delay tylko na zimnym starcie (USB-CDC); wybudzenie co minute go pomija
  setCpuFrequencyMhz(DAY_CPU_MHZ);
  releaseNightHolds();
  pinMode(PA_ENABLE_PIN, OUTPUT);
  digitalWrite(PA_ENABLE_PIN, LOW);
  setupDisplay(coldBoot);              // zaraz po holdach: RST HIGH; cieplo -> bez resetu i bez clear -> bez mrugania

  if (coldBoot) {                       // tylko cold boot - RTC RAM przezywa deep-sleep
    resetTriggerState();
    ntpStartedDay = -1;
    ntpMorningDay = -1;
    batteryHasReading = false; batteryPercent = 0; batteryVoltage = 0.0f;
    g_drain.valid = false;
    g_lastSummaryDay = -1;
    g_wifiHint = false;
  }

  setenv("TZ", TZ_POLAND, 1);
  tzset();

  pinMode(KEY_PIN, INPUT_PULLUP);
  pinMode(PREVIEW_BUTTON_PIN, INPUT_PULLUP);
  pinMode(RTC_INT_PIN, INPUT_PULLUP);
  keyRaw = digitalRead(KEY_PIN);  keyStable = keyRaw;  keyChangedMs = millis();
  bootRaw = digitalRead(PREVIEW_BUTTON_PIN); bootStable = bootRaw; bootChangedMs = millis();

  loadSettings();
  updateBattery(true);               // ekran zainicjowany wczesniej w setupDisplay()

  bool timeFromRtc = readRtcToSystem();
  bool rtcAlarmCleared = rtcDisableWakeAlarm();
  ntpStatus = timeFromRtc ? "czas z RTC" : "RTC pusty/brak";
  if (rtcWake && !rtcAlarmCleared) ntpStatus = "RTC alarm: blad kasowania";

  codec.powerDown();   // ES8311 ma wystartowac w stanie niskiego poboru pradu

  if (coldBoot) ensureFsMounted();   // FS potrzebny tylko do MP3; cieplo montuj leniwie (startSound/web) -> krotszy wybudzenie
  checkAlarms(scheduledWake ? NTP_ALARM_CATCHUP_SEC : ALARM_LATE_WINDOW_SEC);
  if (!userWake && !ringing) enterNightSleepIfNeeded(true);

  if (keyWake && !ringing) startWebWindow();
  else stopWifiAndNtp();
  if (previewWake) showPreview();

  // Zimny start zawsze koryguje RTC z NTP. Po zwyklym wybudzeniu z deep-sleep
  // wystarcza czas podtrzymywany przez RTC.
  struct tm wakeTm;
  bool haveWakeTime = getLocalTm(&wakeTm);
  if (coldBoot || !timeFromRtc) {
    ntpRetriesLeft = timeFromRtc ? 2 : 6;
    ntpFailureRetryDelayMs = timeFromRtc ? NTP_RETRY_MS : 30000UL;
    syncNtp(false);
  } else if (scheduledWake && haveWakeTime && !isNightMode(wakeTm) &&
             ntpMorningDay != localDayKey(wakeTm)) {
    // Wyjatek: pierwsze planowe wybudzenie z nocnego deep-sleep do czesci dziennej
    // (~06:00). Po ~20 h od synchronizacji 10:00 sam RTC bywa spozniony > 1 s, a rano
    // uzytkownik patrzy na zegar -> odswiez czas z NTP raz dziennie. WiFi wstaje na
    // kilka s (koszt energii pomijalny). Gdy WiFi padnie: retry + zostaje czas z RTC.
    // Warunek !isNightMode lapie tez przeskoczone okno przy budzeniu timerem RC.
    ntpMorningDay = localDayKey(wakeTm);
    ntpRetriesLeft = 2;
    ntpFailureRetryDelayMs = NTP_RETRY_MS;
    syncNtp(false);
  }

  displayDirty = true;
}

static void applyCpuPolicy() {
#if IDLE_CPU_DOWNCLOCK
  static int curMhz = DAY_CPU_MHZ;
  bool needFast = webActive || playing || ringing || ntpSyncState != NTP_SYNC_IDLE;
  int want = needFast ? DAY_CPU_MHZ : IDLE_CPU_MHZ;
  if (want != curMhz) { setCpuFrequencyMhz(want); curMhz = want; }
#endif
}

void loop() {
  applyCpuPolicy();
  handleKey();
  handleBoot();
  enterNightSleepIfNeeded(false);

  if (webActive) {
    static uint32_t wifiLostMs = 0;
    if (WiFi.status() != WL_CONNECTED) {
      // Chwilowy zanik lacza (utrata beaconow, roaming AP) nie zamyka okna WWW
      // od razu: auto-reconnect zwykle wraca w kilka sekund, a natychmiastowe
      // WIFI_OFF by go ubilo i kasowalo sesje uzytkownika w srodku edycji.
      if (wifiLostMs == 0) wifiLostMs = millis();
      if (millis() - wifiLostMs > WEB_WIFI_GRACE_MS) {
        wifiLostMs = 0;
        statusText = "WiFi rozlaczone";
        stopWebWindow();
      }
    } else {
      wifiLostMs = 0;
      server.handleClient();
      if ((int32_t)(millis() - webUntilMs) >= 0) stopWebWindow();
    }
  }

  serviceAudio();
  serviceRinging();
  updateBattery(false);
  // SNTP ustawia czas w swoim zadaniu; najpierw zatwierdz lub cofnij korekte.
  bool ntpAdjustedTime = serviceNtp();
  // Okno spoznienia poszerzone o realny przestoj petli: dlugi server.handleClient()
  // (upload MP3 potrafi blokowac minuty) ani 60-sekundowy light-sleep nie moga
  // zgubic budzika, ktorego termin wypadl w trakcie przestoju.
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
