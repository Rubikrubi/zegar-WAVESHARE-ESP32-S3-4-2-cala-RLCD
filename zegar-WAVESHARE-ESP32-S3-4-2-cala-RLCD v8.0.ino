// ============================================================================
//  Zegar NTP RLCD v8.0 — wzorcowy refaktor architektury (funkcjonalnie = v7.0)
//
//  ZASADA NACZELNA: zachowanie identyczne z v7.0 co do bajta (NVS, HTML, rejestry,
//  kolejnosc operacji, timingi). Zmiany dotycza WYLACZNIE struktury kodu.
//
//  Zmiany v8.0 (architektura i jakosc):
//   [ARCHITEKTURA]
//    - lekki Dependency Injection: sprzet (Wire, panel, WebServer, Preferences,
//      dekoder Audio, kodek) wstrzykiwany referencjami w konstruktorach — klasy
//      logiki nie siegaja po globalne singletony; okablowanie widoczne w App,
//    - RtcController uniezalezniony od App (czysty sterownik: tylko TwoWire&),
//    - SettingsStore jedynym wlascicielem NVS (zapis dobowego podsumowania baterii
//      przeniesiony z BatteryMonitor); sesje Preferences pod straznikiem RAII
//      (NvsSession) — kazdy wczesny return domyka NVS,
//    - separacja UI/logiki: ClockView (model widoku) — render i anty-mruganie
//      pracuja na tych samych danych liczonych RAZ (v7.0 liczyl najblizszy budzik
//      dwukrotnie na repaint),
//    - DisplayUi::update() jako dispatcher po enum class UiPage (jawna hierarchia
//      stron: Ringing > Preview > NoTime > Clock),
//    - jedna klasa DebouncedButton (maszyna stanow debounce+pomiar nacisniecia)
//      zamiast dwoch recznych debouncerow; polityka krotki/dlugi w ButtonInput,
//    - wspolne zejscie do deep-sleep PowerPlanner::beginDeepSleep() (noc + ochrona
//      ogniwa) — sekwencja usypiania istnieje w JEDNYM egzemplarzu,
//    - klasyfikacja przyczyny wybudzenia w POD WakeInfo (jedno miejsce, testowalne).
//   [NOWOCZESNE C++ / CZYTELNOSC]
//    - enum class: NtpState, SyncTrigger, ButtonEvent, UiPage, StopCause, FsBackend,
//    - cala konfiguracja w przestrzeniach constexpr (pins/lcd/i2cbus/pcf85063/
//      es8311/rtccfg/batcfg/alarmcfg/ntpcfg/powercfg/btncfg/webcfg/fscfg/nvskeys)
//      — zero magicznych liczb w logice; piny jako gpio_num_t (bez rzutowan),
//    - rejestry i bity PCF85063A oraz ES8311 nazwane wg datasheetow,
//    - std::array dla tablic konfiguracyjnych i budzikow (+static_assert zgodnosci
//      rozmiaru bloba NVS "alarms" z v7.x),
//    - static_assert pilnuje trywialnosci PersistentState (RTC_DATA_ATTR) —
//      przypadkowy konstruktor kasowalby stan po kazdym wybudzeniu,
//    - estymator zuzycia jako metody struktury DrainEstimator (POD) — znika hack
//      forward-deklaracji pod auto-prototypy Arduino.
//   [MARTWY KOD — USUNIETE]
//    - no-op st7305SetStaticLowPower + flaga ST7305_STATIC_LPM, nieuzywany parametr
//      coldBoot w setupie ekranu, nieuzywana App& w RtcManager, martwe akcesory
//      (FileStore::mounted, App::lastAlarmCheckMs), nieuzywane stale BATTERY_
//      EMPTY/FULL_VOLTAGE, trwale martwa galaz POWER_DOWN_FLASH_IN_SLEEP
//      (ostrzezenie sprzetowe zostalo jako komentarz przy light-sleep).
//   [DEDUPLIKACJA]
//    - failNtpAttempt() (2 identyczne sciezki porazki NTP), wifiPowerOff(),
//      copyBounded() (3 warianty kopiowania poswiadczen), removeIfExists() +
//      discardUploadTemp() (7x exists/remove w uploadzie MP3), nextOccurrence()
//      (3x wzorzec "dzis albo jutro"), extendWindow() (6x przedluzenie okna WWW),
//      amplifierOff() (3x pinMode+LOW), sentinel repaintu w jednej stalej.
//   [POPRAWKI BLEDOW ODZIEDZICZONYCH PO v7.0 — jedyne swiadome zmiany zachowania]
//    1. rollback czasu po nieudanym NTP cofal zegar systemowy o czas trwania
//       proby (do ~50 s), mimo ze zaden pakiet nie zmodyfikowal zegara — teraz
//       przywracamy czas TYLKO gdy pakiet faktycznie wszedl (g_ntpGotPacket),
//    2. sweep "kolizji DST" w checkAlarms oznaczal jako wyzwolone WSZYSTKIE
//       budziki wymagalne w oknie spoznienia (przy oknie 90 s po korekcie NTP
//       drugi budzik znikal bez dzwieku) — teraz konsumowane sa wylacznie
//       budziki o IDENTYCZNEJ chwili wystapienia (prawdziwa kolizja),
//    3. budzik, ktorego termin wypadl PODCZAS dzwonienia innego, ginal po 20 s
//       okna — teraz trafia do kolejki i dzwoni zaraz po wyciszeniu/zakonczeniu,
//    4. poranna synchronizacja 06:00 zyla tylko w setup() — urzadzenie czuwajace
//       przez 06:00 (dzwoniacy budzik, otwarte WWW) gubilo ja na caly dzien —
//       przeniesiona do checkSchedule() (dziala tez w trybie oszczedzania:
//       obejmuje ja teraz bramka niskiej baterii, spojnie z oknem 18:00),
//    5. zimny start z wcisnietym KEY: zwolnienie po dowolnie dlugim przytrzymaniu
//       odpalalo akcje krotka (panel WWW) — begin() uzbraja pomiar nacisniecia,
//       wiec kontrakt "dlugi KEY = nic" obowiazuje takze od pierwszego zwolnienia,
//    6. busy-wait wyrownania fazy w zapisie RTC: arytmetyka w int64, watchdogFeed
//       w petli i twarde przerwanie zapisu przy rownoleglym skoku czasu (SNTP).
//   Pozostale inwarianty czasu/energii/RTC/NTP/sleep przeniesione 1:1 z v7.0
//   (opisane przy definicjach). Klucze NVS w namespace "zegar7" bez zmian —
//   budziki, WiFi i wykalibrowany offset RTC przezywaja aktualizacje firmware.
//
//  Historia v7.x (skrot): v7.0 autokalibracja offsetu RTC z pomiaru fazowego NTP
//  (dryf resztkowy ~±0.19 s/dobe), CPU 80 MHz w dzien / 160 MHz przy audio,
//  odzysk zawieszonej szyny I2C, NTP fail-fast po zerwaniu WiFi; v7.0 porzadki.
//
//  Sprzet: Waveshare ESP32-S3-RLCD-4.2 (ST7305 refleksyjny 400x300 landscape,
//          ES8311 audio codec, PCF85063A RTC). Zasilanie: 1x Samsung INR18650-35E.
//  Arduino IDE: plytka "ESP32S3 Dev Module", Flash 8MB, partycja "8M with spiffs".
//  Pliki obok .ino: photo_font.h  (+ MP3 budzikow na partycji danych: /alarm1..3.mp3)
//  Biblioteki: U8g2 (>=2.35), ESP32 core 3.x, ESP32-audioI2S 3.4.5.
// ============================================================================
// ============================================================================
//  [1] INCLUDES
// ============================================================================
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Wire.h>
#include <atomic>
#include <array>
#include <type_traits>
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

// ============================================================================
//  [2] WERSJA, BUILD INFO, FLAGI FUNKCJI
// ============================================================================
#define FW_NAME        "Zegar NTP RLCD"
#define FW_VERSION     "8.0"
#define FW_BUILD_DATE  __DATE__
#define FW_BUILD_TIME  __TIME__

// 1=wlaczone, 0=wylaczone (#if usuwa kod z kompilacji -> zero narzutu gdy off).
#define ENABLE_WATCHDOG            1   // sprzetowy reset po zawieszeniu petli (TWDT)
#define ENABLE_LOW_BATTERY_PROTECT 1   // ochrona ogniwa: degradacja + ochronny sen
#define ENABLE_DIAGNOSTICS         1   // strona /info + liczniki resetow
#define ENABLE_RTC_AUTOTRIM        1   // automatyczna kalibracja offsetu RTC z pomiaru NTP

// Szybki powrot do AP po znanym kanale+BSSID (krocej z wlaczonym radiem).
#define FAST_NTP_RECONNECT         1

// UWAGA (decyzja sprzetowa, NIE wlaczac ponownie): odciecie VDDSDIO w light-sleep
// (dawna flaga POWER_DOWN_FLASH_IN_SLEEP) jest NIEKOMPATYBILNE z ta plytka
// (flash/PSRAM na tej szynie -> reboot, cichy budzik). Kod usuniety celowo.

// ============================================================================
//  [3] KONFIGURACJA
//  Wszystkie stale projektu w nazwanych przestrzeniach constexpr. Zmiana tutaj
//  nie wymaga szukania magicznych liczb po pliku.
// ============================================================================

// --- Piny (Waveshare ESP32-S3-RLCD-4.2) ---------------------------------------
namespace pins {
  // Wyswietlacz RLCD (HARDWARE SPI; SPI.begin w DisplayUi::setup).
  constexpr gpio_num_t LCD_SCK    = GPIO_NUM_11;
  constexpr gpio_num_t LCD_MOSI   = GPIO_NUM_12;
  constexpr gpio_num_t LCD_DC     = GPIO_NUM_5;
  constexpr gpio_num_t LCD_CS     = GPIO_NUM_40;
  constexpr gpio_num_t LCD_RST    = GPIO_NUM_41;
  // I2C (kodek ES8311 + RTC PCF85063A na wspolnej szynie).
  constexpr gpio_num_t I2C_SDA    = GPIO_NUM_13;
  constexpr gpio_num_t I2C_SCL    = GPIO_NUM_14;
  constexpr gpio_num_t RTC_INT    = GPIO_NUM_15;  // wyjscie INT alarmu PCF85063A (aktywne LOW)
  // Audio I2S do ES8311 + wlacznik wzmacniacza.
  constexpr gpio_num_t I2S_MCLK   = GPIO_NUM_16;
  constexpr gpio_num_t I2S_BCLK   = GPIO_NUM_9;
  constexpr gpio_num_t I2S_WS     = GPIO_NUM_45;
  constexpr gpio_num_t I2S_DOUT   = GPIO_NUM_8;
  constexpr gpio_num_t PA_ENABLE  = GPIO_NUM_46;
  // Przyciski (aktywne LOW, wybudzanie EXT1).
  constexpr gpio_num_t KEY        = GPIO_NUM_18;  // krotko: wycisz budzik / panel WWW; dlugo: nic
  constexpr gpio_num_t PREVIEW    = GPIO_NUM_0;   // BOOT; krotko: podglad najblizszego budzika

  // Piny trzymane (gpio_hold) przez deep-sleep, by panel nie dostawal smieci.
  constexpr std::array<gpio_num_t, 5> DEEP_SLEEP_HOLD = {
    LCD_DC, LCD_CS, LCD_SCK, LCD_MOSI, LCD_RST
  };
}

// --- Wyswietlacz --------------------------------------------------------------
namespace lcd {
  constexpr int      WIDTH        = 400;
  constexpr int      HEIGHT       = 300;
  constexpr uint32_t SPI_BUS_HZ   = 2000000UL;   // zalecenie sterownika U8g2 dla ST7305
  constexpr uint8_t  CONTRAST     = 255;
}

// --- Szyna I2C ----------------------------------------------------------------
namespace i2cbus {
  constexpr uint32_t FREQ_HZ                = 400000;
  constexpr uint8_t  RECOVER_MAX_PULSES     = 9;    // maks. bitow trzymanych przez slave (spec I2C)
  constexpr uint32_t RECOVER_HALF_PERIOD_US = 50;   // pol okresu ~10 kHz przy recznym taktowaniu
}

// --- RTC PCF85063A: adres, rejestry, bity (datasheet NXP rev.7) -----------------
namespace pcf85063 {
  constexpr uint8_t I2C_ADDR         = 0x51;

  constexpr uint8_t REG_CONTROL1     = 0x00;
  constexpr uint8_t REG_CONTROL2     = 0x01;
  constexpr uint8_t REG_OFFSET       = 0x02;
  constexpr uint8_t REG_RAM_BYTE     = 0x03;   // wolny bajt RAM -> marker formatu czasu
  constexpr uint8_t REG_SECONDS      = 0x04;   // 0x04..0x0A: sec,min,hour,day,wday,month,year
  constexpr uint8_t REG_SECOND_ALARM = 0x0B;   // 0x0B..0x0F: sec,min,hour,day,wday alarm
  constexpr uint8_t REG_TIMER_VALUE  = 0x10;
  constexpr uint8_t REG_TIMER_MODE   = 0x11;

  constexpr uint8_t CTRL1_EXT_TEST   = 1u << 7;
  constexpr uint8_t CTRL1_STOP       = 1u << 5;  // zatrzymuje licznik + zeruje preskaler
  constexpr uint8_t CTRL1_CIE        = 1u << 2;  // przerwanie korekcji
  constexpr uint8_t CTRL1_12H        = 1u << 1;  // tryb 12h (uzywamy 24h)
  constexpr uint8_t CTRL2_AIE        = 1u << 7;  // przerwanie alarmu na pin INT
  constexpr uint8_t CTRL2_COF_OFF    = 0x07;     // CLKOUT wylaczony (mniejszy pobor)
  constexpr uint8_t SECONDS_OS       = 1u << 7;  // oscylator sie zatrzymal -> czas niewiarygodny
  constexpr uint8_t ALARM_AEN_OFF    = 0x80;     // AEN=1 w rejestrze alarmu = pole wylaczone
  constexpr uint8_t TIMER_MODE_OFF   = 0x18;     // TCF=1/60 Hz, TE=0, TIE=0 (timer nieuzywany)

  // Czas z zatrzymanego/testowego/12h RTC nie jest aktualny -> odrzucamy odczyt.
  constexpr uint8_t CTRL1_INVALID_MODE_MASK = CTRL1_EXT_TEST | CTRL1_STOP | CTRL1_12H;
  // prepareTimeWrite zeruje takze CIE (INT wylacznie dla alarmu wybudzajacego).
  constexpr uint8_t CTRL1_RUN_CLEAR_MASK    = CTRL1_EXT_TEST | CTRL1_STOP | CTRL1_CIE | CTRL1_12H;

  constexpr uint8_t FORMAT_MARKER    = 0xA5;     // w REG_RAM_BYTE: "czas zapisal ten firmware"
}

// --- Kalibracja dryftu krysztalu RTC (rejestr Offset 0x02 PCF85063A) -----------
//   offset DODATNI spowalnia zegar; UJEMNY przyspiesza. Offset[LSB]=blad_ppm/4.34.
//   Zmierzony surowy dryft -2 s/24h -> -5 LSB = -21.7 ppm. 0x7B = -5 LSB, MODE=0.
//   FACTORY_OFFSET_REG to wartosc STARTOWA; z ENABLE_RTC_AUTOTRIM biezacy offset
//   zyje w NVS ("rtcOffLsb") i jest korygowany automatycznie z pomiarow NTP.
namespace rtccfg {
  constexpr uint8_t  FACTORY_OFFSET_REG   = 0x7B;    // -5 LSB, MODE=0 (-21.7 ppm; przyspiesza)
  constexpr float    OFFSET_LSB_PPM_MODE0 = 4.34f;
  constexpr float    OFFSET_LSB_PPM_MODE1 = 4.069f;
  constexpr uint8_t  OFFSET_WRITE_RETRIES = 4;
  constexpr float    PPM_SEC_PER_DAY      = 0.0864f; // 1 ppm = 0.0864 s/dobe

  constexpr uint32_t WRITE_RETRY_MS       = 5000;    // ponowienie zapisu czasu po NTP
  constexpr uint8_t  WRITE_RETRIES        = 12;
  constexpr int      WRITE_VERIFY_TOLERANCE_SEC = 2; // odczyt zwrotny po zapisie czasu
  constexpr int64_t  ALIGN_ABORT_US       = 1500000; // wyrownanie fazy startuje z <=1 s;
                                                     // wiecej = rownolegly skok czasu -> przerwij zapis
  constexpr int      WAKE_MAX_SKEW_SEC    = 30;      // rozjazd RTC/system dyskwalifikuje alarm

#if ENABLE_RTC_AUTOTRIM
  // Pomiar fazowy (granica sekundy RTC vs czas NTP) ma rozdzielczosc ~ms, wiec juz
  // jedno okno 6 h daje ppm z niepewnoscia ~0.1 ppm — wystarcza do korekty o 1 LSB
  // (4.34 ppm). Kwantyzacja rejestru ogranicza dryf resztkowy do ~±2.2 ppm,
  // czyli ~±0.19 s/dobe — ponizej celu 1 s/dobe.
  constexpr int64_t  AUTOTRIM_MIN_WINDOW_SEC     = 6 * 3600;   // min. okno NTP->NTP
  constexpr int64_t  AUTOTRIM_MAX_WINDOW_SEC     = 7 * 86400;  // dluzsze = watpliwa kotwica
  constexpr float    AUTOTRIM_MAX_PLAUSIBLE_PPM  = 150.0f;     // wiecej = zly pomiar, nie kalibruj
  constexpr uint32_t BOUNDARY_POLL_TIMEOUT_MS    = 1300;       // >1 s: przewrot MUSI nadejsc
#endif
}

// --- Kodek audio ES8311 --------------------------------------------------------
namespace es8311 {
  constexpr uint8_t I2C_ADDR       = 0x18;
  constexpr uint8_t REG_RESET      = 0x00;  // reset / power-up sekwencja
  constexpr uint8_t REG_CLK_MGR1   = 0x01;
  constexpr uint8_t REG_SDP_IN     = 0x09;  // format danych I2S (DAC)
  constexpr uint8_t REG_SDP_OUT    = 0x0A;  // format danych I2S (ADC)
  constexpr uint8_t REG_DAC_VOLUME = 0x32;
  constexpr uint8_t DAC_VOLUME_MAX = 215;   // 0xD7 ~ +12 dB; mniejsze ryzyko przesteru niz 0xFF
}

// --- Akumulator (Samsung INR18650-35E) ------------------------------------------
#ifndef BATTERY_CAPACITY_MAH
#define BATTERY_CAPACITY_MAH 3400          // nadpisywalne z linii kompilacji (-D)
#endif
namespace batcfg {
  constexpr uint32_t     CAPACITY_MAH        = BATTERY_CAPACITY_MAH;
  constexpr adc_channel_t ADC_CHANNEL        = ADC_CHANNEL_3;   // = GPIO4 na ESP32-S3
  constexpr float        DIVIDER_RATIO       = 3.0f;
  constexpr float        VOLTAGE_CALIBRATION = 1.017f;
  constexpr uint8_t      SAMPLE_COUNT        = 16;
  constexpr float        EMA_KEEP            = 0.80f; // filtr napiecia: 80% starej + 20% nowej probki
  constexpr int          ADC_FALLBACK_FULL_MV = 3300; // przelicznik bez kalibracji eFuse
  constexpr int          ADC_MAX_RAW          = 4095;
  constexpr uint8_t      FULL_FLOOR          = 99;   // >=99% pokazuj 100%
  constexpr uint8_t      FULL_HOLD_FLOOR     = 98;   // trzymaj 100% az spadnie < 98%
  constexpr uint32_t     READ_INTERVAL_MS    = 10UL * 60UL * 1000UL;
  constexpr uint32_t     BUSY_RETRY_MS       = 30UL * 1000UL;

  // Ochrona niskiego stanu ogniwa (dwustopniowa).
  constexpr uint8_t      LOW_PERCENT          = 15;     // stopien 1: bez auto-NTP
  constexpr float        CRITICAL_VOLTAGE     = 3.05f;  // stopien 2: ochronny deep-sleep
  constexpr uint8_t      CRITICAL_PERCENT     = 3;
  constexpr uint32_t     CRITICAL_RECHECK_SEC = 30UL * 60UL;

  // Estymator zuzycia (czysta arytmetyka na probkach SoC).
  constexpr uint8_t      DRAIN_CHARGE_HYST     = 5;      // wzrost SoC% = "doladowano" -> reset kotwicy
  constexpr int64_t      DRAIN_MIN_WINDOW_SEC  = 43200;  // min. 12 h danych (plaskie plateau = szum ADC)
  constexpr int          DRAIN_MIN_DSOC        = 2;      // min. spadek SoC%, by estymata miala sens

  // Krzywa OCV->SoC dla INR18650-35E (Li-ion NMC), 3.00 V=0%, 4.20 V=100%.
  struct OcvPoint { float voltage; uint8_t soc; };
  constexpr std::array<OcvPoint, 24> OCV_CURVE = {{
    { 3.00f,   0 }, { 3.10f,   2 }, { 3.20f,   4 }, { 3.30f,   7 },
    { 3.40f,  11 }, { 3.45f,  14 }, { 3.50f,  18 }, { 3.55f,  23 },
    { 3.60f,  29 }, { 3.65f,  36 }, { 3.70f,  43 }, { 3.73f,  47 },
    { 3.76f,  51 }, { 3.79f,  55 }, { 3.82f,  59 }, { 3.85f,  63 },
    { 3.88f,  68 }, { 3.91f,  71 }, { 3.95f,  76 }, { 4.00f,  80 },
    { 4.05f,  87 }, { 4.10f,  93 }, { 4.15f,  97 }, { 4.20f, 100 },
  }};
}

// --- Budziki i audio ------------------------------------------------------------
namespace alarmcfg {
  constexpr int      ALARM_COUNT          = 3;
  constexpr int      SOUND_COUNT          = 3;
  constexpr int      MAX_VOLUME           = 21;    // natywny zakres audioI2S setVolume
  constexpr uint8_t  DEFAULT_VOLUME       = 10;
  constexpr uint8_t  CODEC_VOLUME_PERCENT = 100;
  constexpr uint32_t LATE_WINDOW_SEC      = 20;    // ile po terminie budzik jeszcze "wchodzi"
  constexpr uint32_t RING_RETRY_MS        = 3000;
  constexpr uint32_t RING_MIN_PLAY_MS     = 750;   // krocej = uszkodzony MP3 (patrz serviceAudio)
  constexpr uint8_t  RING_MAX_ERRORS      = 10;
  constexpr uint32_t RING_TIMEOUT_MS      = 15UL * 60UL * 1000UL;
  constexpr uint32_t LOOP_GAP_MARGIN_SEC  = 2;     // zapas okna spoznienia ponad przestoj petli

  constexpr std::array<const char *, SOUND_COUNT> SOUND_FILES = {
    "/alarm1.mp3", "/alarm2.mp3", "/alarm3.mp3"
  };
  constexpr std::array<const char *, SOUND_COUNT> SOUND_NAMES = {
    "Dzwiek 1", "Dzwiek 2", "Dzwiek 3"
  };
}

// --- WiFi / NTP -----------------------------------------------------------------
namespace ntpcfg {
  constexpr uint32_t WIFI_TIMEOUT_MS       = 15000;
  constexpr uint32_t PACKET_TIMEOUT_MS     = 50000;  // lwIP SNTP: 3 serwery co ~15 s -> 50 s pokrywa
  constexpr uint32_t WIFI_LOST_GRACE_MS    = 8000;   // zerwane WiFi w trakcie proby: tyle na powrot
  constexpr uint32_t WIFI_HINT_FALLBACK_MS = 5000;   // po tylu ms hint kanal+BSSID -> pelny skan
  constexpr uint32_t ALARM_CATCHUP_SEC     = 90;     // poszerzone okno budzika tuz po korekcie czasu

  constexpr uint32_t QUICK_RETRY_MS        = 30000;      // seria szybka po starcie
  constexpr uint32_t BACKOFF_MIN_MS        = 60000;      // 1 min
  constexpr uint32_t BACKOFF_MAX_MS        = 3600000;    // 1 h

  constexpr int      DAILY_SYNC_HOUR       = 18;  // codzienna synchronizacja 18:00-18:05 (+ dogonienie do 23:00)
  constexpr int      SYNC_MINUTE_WINDOW    = 5;

  constexpr const char *TZ_POLAND    = "CET-1CEST,M3.5.0/2,M10.5.0/3";
  constexpr const char *SERVER_1     = "pool.ntp.org";
  constexpr const char *SERVER_2     = "time.google.com";
  constexpr const char *SERVER_3     = "time.cloudflare.com";
  constexpr const char *DEFAULT_SSID = "";
  constexpr const char *DEFAULT_PASS = "";
}

// --- Energia: CPU, sen dzienny i nocny -------------------------------------------
// CPU: w dzien 80 MHz wystarcza (render RLCD ograniczony SPI 2 MHz, a SPI/I2C/WiFi
// taktowane z APB, ktore przy >=80 MHz CPU i tak chodzi 80 MHz). 160 MHz tylko na
// czas dekodowania MP3 (AudioPlayer podbija/odpuszcza).
namespace powercfg {
  constexpr uint32_t DAY_CPU_MHZ                  = 80;
  constexpr uint32_t AUDIO_CPU_MHZ                = 160;
  constexpr uint32_t PRE_SLEEP_CPU_MHZ            = 40;    // tuz przed deep-sleep

  constexpr uint64_t IDLE_LIGHT_SLEEP_US          = 900000ULL;  // fallback gdy czas nieznany
  constexpr uint64_t IDLE_LIGHT_SLEEP_MAX_SEC     = 60ULL;      // sen do granicy minuty -> 1 wybudzenie/min
  constexpr int      NIGHT_START_HOUR             = 23;
  constexpr int      NIGHT_START_MIN              = 59;
  constexpr int      NIGHT_END_HOUR               = 6;
  constexpr int      NIGHT_END_MIN                = 0;
  constexpr uint32_t NIGHT_MIN_SLEEP_SEC          = 5;
  constexpr uint32_t NIGHT_FALLBACK_CHUNK_SEC     = 3600;  // maks. odcinek snu bez alarmu RTC
  constexpr uint32_t NIGHT_FALLBACK_ALARM_CHUNK_SEC = 300; // jw. gdy najblizszy jest budzik: krocej
  constexpr uint32_t NIGHT_EMERGENCY_SLEEP_SEC    = 6UL * 3600UL; // cel snu gdy liczenie zawiodlo
  constexpr uint32_t DISPLAY_FLUSH_MS             = 150;   // panel konczy odswiezanie przed snem
}

// --- Przyciski --------------------------------------------------------------------
namespace btncfg {
  constexpr uint32_t DEBOUNCE_MS            = 40;
  constexpr uint32_t KEY_LONG_PRESS_MS      = 1500;  // KEY >= tyle = "dlugie": zadnej akcji,
                                                     // tylko tlumi akcje krotka przy zwolnieniu
  constexpr uint32_t BOOT_SHORT_PRESS_MAX_MS = 1200; // PREVIEW/BOOT <= tyle -> podglad budzika
  constexpr uint32_t PREVIEW_SHOW_MS        = 5000;
  constexpr uint32_t WAKE_HOLD_MS           = 15000; // po wybudzeniu przyciskiem: pokaz tresc 15 s
}

// --- Panel WWW ----------------------------------------------------------------------
namespace webcfg {
  constexpr uint32_t WINDOW_MS         = 5UL * 60UL * 1000UL;  // okno aktywnosci panelu
  constexpr uint32_t WIFI_GRACE_MS     = 10000;                // zanik WiFi: tyle na auto-reconnect
  constexpr uint16_t HTTP_PORT         = 80;
  constexpr size_t   HTML_ROOT_RESERVE = 14000;  // typowy rozmiar strony glownej (bez realokacji)
  constexpr size_t   HTML_INFO_RESERVE = 4500;
}

// --- System plikow MP3 ----------------------------------------------------------------
namespace fscfg {
  constexpr uint8_t RECOVER_MAX_PASSES = 16;  // limit przebiegow naprawy .bak/.part
  constexpr size_t  MP3_NAME_MIN       = 5;   // "x.mp3"
  constexpr size_t  MP3_NAME_MAX       = 25;  // SPIFFS: pelna sciezka <= 31 znakow
}

// --- Klucze NVS (Preferences) ---------------------------------------------------------
// KOMPATYBILNOSC: nazwy identyczne z v7.x — istniejace urzadzenie zachowa budziki,
// WiFi i wykalibrowany offset RTC po wgraniu v8.0.
namespace nvskeys {
  constexpr const char *NAMESPACE_NAME = "zegar7";
  constexpr const char *ALARMS         = "alarms";
  constexpr const char *WIFI_SSID      = "ssid";
  constexpr const char *WIFI_PASS      = "pass";
  constexpr const char *RTC_OFFSET_LSB = "rtcOffLsb";
  constexpr const char *DRAIN_SOC      = "drnSoc";
  constexpr const char *DRAIN_MAH_DAY  = "drnMahDay";
  constexpr const char *DRAIN_DAY      = "drnDay";
  constexpr const char *RESETS_BROWNOUT = "rstBrown";
  constexpr const char *RESETS_WDT      = "rstWdt";
  constexpr const char *RESETS_PANIC    = "rstPanic";
}

// ============================================================================
//  [4] TYPY WSPOLNE — enum class i male struktury
// ============================================================================

// Maszyna stanow synchronizacji NTP (NetworkTimeService).
enum class NtpState : uint8_t { Idle, WifiConnecting, WaitingForPacket };

// Kto zazadal synchronizacji: harmonogram (auto) czy uzytkownik (przycisk /sync).
enum class SyncTrigger : uint8_t { Scheduled, Manual };

// Zdarzenia z debouncera przycisku (DebouncedButton::poll).
enum class ButtonEvent : uint8_t { None, Pressed, Released };

// Strony ekranu w kolejnosci PRIORYTETOW (dzwonienie > podglad > brak czasu > zegar).
enum class UiPage : uint8_t { Ringing, Preview, NoTime, Clock };

// Kto zatrzymal odtwarzanie: uzytkownik (kasuje tez dzwonienie) czy automat.
enum class StopCause : uint8_t { User, Auto };

// Ktory system plikow z MP3 zamontowano.
enum class FsBackend : uint8_t { None, Spiffs, LittleFs, FFatFs };

// Sklasyfikowana przyczyna startu/wybudzenia — liczona RAZ, pierwsza operacja
// setup() (flagi sa kombinowalne, dlatego POD z polami, nie pojedynczy enum).
struct WakeInfo {
  bool coldBoot;      // zasilanie/reset (nie deep-sleep)
  bool keyWake;       // KEY (zimny start liczy sie jak wybudzenie uzytkownika)
  bool previewWake;   // przycisk PREVIEW/BOOT
  bool rtcWake;       // pin INT alarmu PCF85063A
  bool timerWake;     // timer ESP (zabezpieczenie nocne)

  bool userWake() const      { return keyWake || previewWake; }
  bool scheduledWake() const { return rtcWake || timerWake; }

  static WakeInfo classify() {
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    uint64_t ext1Pins = (cause == ESP_SLEEP_WAKEUP_EXT1) ? esp_sleep_get_ext1_wakeup_status() : 0;
    WakeInfo w;
    w.coldBoot    = (cause == ESP_SLEEP_WAKEUP_UNDEFINED);
    w.keyWake     = w.coldBoot || cause == ESP_SLEEP_WAKEUP_EXT0 ||
                    ((ext1Pins & (1ULL << pins::KEY)) != 0);
    w.previewWake = (ext1Pins & (1ULL << pins::PREVIEW)) != 0;
    w.rtcWake     = (ext1Pins & (1ULL << pins::RTC_INT)) != 0;
    w.timerWake   = (cause == ESP_SLEEP_WAKEUP_TIMER);
    return w;
  }
};

// Konfiguracja pojedynczego budzika. UWAGA: layout bajtowy identyczny z v7.x —
// blob NVS "alarms" (getBytes/putBytes) musi sie zgadzac po aktualizacji firmware.
struct AlarmConfig {
  uint8_t enabled;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint8_t volume;     // 0..alarmcfg::MAX_VOLUME
  uint8_t sound;      // indeks 0..SOUND_COUNT-1 -> SOUND_FILES
  char    label[16];
};
// Kontrakt binarny bloba NVS: getBytes/putBytes przyjmuje dane tylko przy zgodnym
// rozmiarze — zmiana layoutu = cicha utrata budzikow uzytkownika po aktualizacji.
static_assert(sizeof(AlarmConfig) == 22, "layout NVS 'alarms' (v7.x) nie do ruszenia");
static_assert(std::is_trivially_copyable<AlarmConfig>::value, "NVS blob musi byc memcpy-owalny");

// Wynik wyszukiwania najblizszego budzika (zamiast 5 parametrow wyjsciowych).
struct NextAlarmInfo {
  int     index;      // -1 = brak wlaczonego budzika
  uint8_t hour, minute, second;
  char    label[sizeof(AlarmConfig::label)];
  bool    valid() const { return index >= 0; }
};

// ============================================================================
//  [5] CZAS — konwersje cywilne (czyste funkcje, bez stanu)
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

// Unikalny klucz doby lokalnej (rok*400+dzien roku) — dedup zdarzen "raz na dobe".
static int localDayKey(const struct tm &t) {
  return (t.tm_year + 1900) * 400 + t.tm_yday;
}

static bool isNightMode(const struct tm &t) {
  int now = t.tm_hour * 60 + t.tm_min;
  int start = powercfg::NIGHT_START_HOUR * 60 + powercfg::NIGHT_START_MIN;
  int end   = powercfg::NIGHT_END_HOUR * 60 + powercfg::NIGHT_END_MIN;
  if (start < end) return now >= start && now < end;
  if (start > end) return now >= start || now < end;
  return false;
}

// Zakres epok akceptowany jako "prawdziwy czas" (odrzuca 1970 po resecie i smieci).
constexpr time_t MIN_VALID_EPOCH = 1700000000;   // 2023-11: wszystko wczesniej = smiec
constexpr int    VALID_YEAR_MIN  = 2024;
constexpr int    VALID_YEAR_MAX  = 2099;         // PCF85063A: rok dwucyfrowy 2000..2099

static bool isSupportedEpoch(time_t epoch) {
  if (epoch < MIN_VALID_EPOCH) return false;
  struct tm gm;
  gmtime_r(&epoch, &gm);
  int year = gm.tm_year + 1900;
  return year >= VALID_YEAR_MIN && year <= VALID_YEAR_MAX;
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

// Epoch dla godziny sciennej h:mi:s wzgledem doby 'base' (+addDays dni).
// mktime z tm_isdst=-1 -> poprawne przez noc zmiany czasu (doba != 86400 s).
static time_t epochForClock(struct tm base, int h, int mi, int s, int addDays) {
  base.tm_hour = h; base.tm_min = mi; base.tm_sec = s;
  base.tm_mday += addDays;     // mktime normalizuje przepelnienie
  base.tm_isdst = -1;          // mktime sam ustali DST
  return mktime(&base);
}

// Najblizsze PRZYSZLE wystapienie godziny sciennej h:mi:s (dzis lub jutro).
// Wspolny wzorzec dla budzikow i konca trybu nocnego.
static time_t nextOccurrence(const struct tm &base, int h, int mi, int s, time_t now) {
  time_t cand = epochForClock(base, h, mi, s, 0);
  if (cand <= now) cand = epochForClock(base, h, mi, s, 1);
  return cand;
}

// Czy budzik h:mi:s jest wymagalny TERAZ (termin <= now <= termin+lateWindowSec)?
// occurrenceDay dostaje klucz doby WYSTAPIENIA (nie doby biezacej) — dedup poprawny
// takze tuz po polnocy, gdy dzwoni budzik sprzed polnocy. dueEpoch (opcjonalny)
// dostaje dokladna chwile terminu — do wykrywania PRAWDZIWYCH kolizji budzikow.
static bool alarmDueNow(const struct tm &nowTm, time_t now, uint8_t h, uint8_t mi,
                        uint8_t s, uint32_t lateWindowSec, int *occurrenceDay,
                        time_t *dueEpoch) {
  time_t alarmAt = epochForClock(nowTm, h, mi, s, 0);
  if (alarmAt > now) alarmAt = epochForClock(nowTm, h, mi, s, -1);
  time_t delaySec = now - alarmAt;
  if (delaySec < 0 || (uint64_t)delaySec > lateWindowSec) return false;
  if (occurrenceDay) {
    struct tm alarmTm;
    localtime_r(&alarmAt, &alarmTm);
    *occurrenceDay = localDayKey(alarmTm);
  }
  if (dueEpoch) *dueEpoch = alarmAt;
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
  if (y < VALID_YEAR_MIN || y > VALID_YEAR_MAX || mo < 1 || mo > 12) return false;
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
  return (float)lsb * (mode1 ? rtccfg::OFFSET_LSB_PPM_MODE1 : rtccfg::OFFSET_LSB_PPM_MODE0);
}
// Sugerowana DOCELOWA wartosc offsetu po zmierzeniu dryfu (autotrim oraz /info).
static int suggestedTotalOffsetLsb(float driftSecPerDay, int currentLsb) {
  float lsbF  = (driftSecPerDay / rtccfg::PPM_SEC_PER_DAY) / rtccfg::OFFSET_LSB_PPM_MODE0;
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
//  [6] ESTYMATOR ZUZYCIA BATERII — czysta arytmetyka na probkach SoC.
//  Struktura z metodami zamiast wolnych funkcji: brak potrzeby forward-deklaracji
//  pod auto-prototypy Arduino, a stan i operacje mieszkaja razem.
//  POD bez konstruktorow/inicjalizatorow — zyje w RTC_DATA_ATTR (PersistentState).
// ============================================================================
struct DrainEstimator {
  bool     valid;
  int64_t  anchorEpoch;
  uint8_t  anchorSoc;
  int64_t  lastEpoch;
  uint8_t  lastSoc;

  void reset(int64_t epoch, uint8_t soc) {
    valid = true;
    anchorEpoch = epoch; anchorSoc = soc;
    lastEpoch   = epoch; lastSoc   = soc;
  }

  void onSample(int64_t epoch, uint8_t soc) {
    if (!valid) { reset(epoch, soc); return; }
    if (soc > anchorSoc + batcfg::DRAIN_CHARGE_HYST || soc > lastSoc + batcfg::DRAIN_CHARGE_HYST) {
      reset(epoch, soc); return;   // wykryto ladowanie -> restart okna
    }
    lastEpoch = epoch; lastSoc = soc;
  }

  // mAh/dobe. <0 => za malo danych; 0 => brak mierzalnego zuzycia.
  float mahPerDay() const {
    if (!valid) return -1.0f;
    int64_t dt = lastEpoch - anchorEpoch;
    if (dt < batcfg::DRAIN_MIN_WINDOW_SEC) return -1.0f;
    int dSoc = (int)anchorSoc - (int)lastSoc;
    if (dSoc < batcfg::DRAIN_MIN_DSOC) return 0.0f;
    return (dSoc / 100.0f) * (float)batcfg::CAPACITY_MAH / (dt / 86400.0f);
  }

  // Dni do rozladowania przy biezacym tempie. <0 => nieznane.
  float daysRemaining(uint8_t curSoc) const {
    float perDay = mahPerDay();
    if (perDay <= 0.0f) return -1.0f;
    return (curSoc / 100.0f) * (float)batcfg::CAPACITY_MAH / perDay;
  }
};

// ============================================================================
//  [7] STAN PRZEZYWAJACY DEEP-SLEEP (RTC_DATA_ATTR)
//  POD: brak konstruktorow — wybudzenie z deep-sleep NIE rusza wartosci, a zimny
//  start zeruje pola JAWNIE w App::setup() (czesc wymaga sentineli -1, nie zera).
// ============================================================================
struct PersistentState {
  uint8_t        batteryPercent;
  float          batteryVoltage;
  bool           batteryHasReading;
  DrainEstimator drain;
  int            lastSummaryDay;
  uint8_t        wifiBssid[6];
  int32_t        wifiChannel;
  bool           wifiHint;
  uint16_t       bootCount;
  int64_t        coldBootEpoch;
  int32_t        lastRtcDriftSec;
  int64_t        lastNtpSyncEpoch;
  int64_t        prevNtpSyncEpoch;
  int            lastTriggerDay[alarmcfg::ALARM_COUNT];  // dedup wyzwolen budzikow
  int            ntpStartedDay;                          // dobowa synchronizacja 18:00
  int            ntpMorningDay;                          // poranne NTP po wybudzeniu 06:00
#if ENABLE_RTC_AUTOTRIM
  bool           rtcPhaseAligned;      // ostatni zapis RTC wyrownal faze -> okno mierzalne
  float          lastDriftPpm;         // ostatni pomiar fazowy (diagnostyka /info)
  int32_t        lastDriftWindowSec;   // dlugosc okna tego pomiaru
  uint16_t       autotrimAdjustCount;  // ile razy offset skorygowano automatycznie
#endif
};
// Nietrywialny typ dostalby konstruktor statyczny, ktory po KAZDYM wybudzeniu z
// deep-sleep wyzerowalby RTC RAM — te asercje pilnuja, by nikt tego nie zepsul.
static_assert(std::is_trivial<PersistentState>::value &&
              std::is_standard_layout<PersistentState>::value,
              "PersistentState musi byc trywialnym PODem (RTC_DATA_ATTR)");
static RTC_DATA_ATTR PersistentState g_persist;

// ============================================================================
//  [8] WATCHDOG ZADANIA (TWDT). Timeout > najdluzszy light-sleep dzienny (60 s).
//  Deep-sleep RESETUJE chip -> TWDT startuje od nowa w setup().
// ============================================================================
constexpr uint32_t WDT_TIMEOUT_S = 90;
constexpr uint32_t COLD_BOOT_SERIAL_DELAY_MS = 300;   // USB-CDC potrzebuje chwili po zimnym starcie

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
//  [9] SINGLETONY STEROWNIKOW SPRZETU
//  Konstruowane przy starcie statycznym; same nie dotykaja sprzetu az do begin().
//  Klasy logiki NIE siegaja po nie bezposrednio — dostaja referencje w
//  konstruktorach (lekki DI: latwa podmiana, widoczne zaleznosci, testowalnosc).
// ============================================================================
using RlcdDisplay = U8G2_ST7305_300X400_F_4W_HW_SPI;

// RST = U8X8_PIN_NONE: U8g2 nigdy sam nie pulsuje resetu — robimy to recznie w
// DisplayUi::setup() przy KAZDYM wejsciu w setup (zimny start ORAZ deep-sleep).
static RlcdDisplay display(U8G2_R1, pins::LCD_CS, pins::LCD_DC, U8X8_PIN_NONE);
static WebServer   httpServer(webcfg::HTTP_PORT);
static Preferences preferences;
static Audio       audioOutput;

// Gate atomic odrozniajacy realna odpowiedz serwera NTP od czasu wstepnie
// ustawionego z RTC. Callback SNTP musi byc wolna funkcja C.
static std::atomic<bool> g_ntpGotPacket{false};
static std::atomic<bool> g_ntpAcceptPacket{false};
static void onSntpSync(struct timeval *tv) {
  if (tv && g_ntpAcceptPacket.load(std::memory_order_acquire))
    g_ntpGotPacket.store(true, std::memory_order_release);
}

static void audioLogCallback(Audio::msg_t m) {
  Serial.printf("Audio %s: %s\n", m.s ? m.s : "", m.msg ? m.msg : "");
}

// Twarde wylaczenie wzmacniacza (pin moze byc w dowolnym stanie po wybudzeniu).
static void amplifierOff() {
  pinMode(pins::PA_ENABLE, OUTPUT);
  digitalWrite(pins::PA_ENABLE, LOW);
}

// ============================================================================
//  [10] POMOCNICZE FUNKCJE PANELU WWW (czyste, bez stanu)
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

// "HH:MM" lub "HH:MM:SS" (dowolne biale znaki na koncu). false = odrzucone.
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
//  [11] Es8311Codec — sterownik kodeka (sekwencja rejestrow: port z es8311.c).
//  Samowystarczalny; szyne I2C dostaje w konstruktorze.
// ============================================================================
class Es8311Codec {
public:
  explicit Es8311Codec(TwoWire &bus) : wire_(bus) {}

  bool begin(int32_t sda, int32_t scl, uint32_t freq) {
    if (sda < 0 || scl < 0) return false;
    wire_.begin(sda, scl, freq);
    wire_.beginTransmission(es8311::I2C_ADDR);
    if (wire_.endTransmission() != 0) return false;

    // Sekwencja startowa 1:1 z referencyjnego es8311.c (esp_codec_dev):
    // pelny reset -> power-up -> zegary 48 kHz/MCLK=256*fs -> I2S 16 bit ->
    // power-up analogowy DAC -> unmute.
    bool ok = true;
    ok &= write(es8311::REG_RESET, 0x1F);
    delay(20);
    ok &= write(es8311::REG_RESET, 0x00);
    ok &= write(es8311::REG_RESET, 0x80);       // power-on, tryb slave
    ok &= write(es8311::REG_CLK_MGR1, 0x3F);    // wlacz wszystkie galezie zegarow
    ok &= write(0x06, read(0x06) & ~(1 << 5));  // BCLK invert off
    ok &= configSampleRate48k();
    ok &= configBits16();
    ok &= write(0x0D, 0x01);                    // power-up sekcji analogowej
    ok &= write(0x0E, 0x02);                    // wlacz DAC (PDN_DAC=0)
    ok &= write(0x12, 0x00);                    // wyjscie DAC nieodwracane
    ok &= write(0x13, 0x10);                    // HP switch on
    ok &= write(0x1C, 0x6A);                    // ustawienia ADC/eq zgodne z portem
    ok &= write(0x37, 0x08);                    // DAC ramp rate
    return ok;
  }

  bool configBits16() { return write(es8311::REG_SDP_IN, 0x0C) && write(es8311::REG_SDP_OUT, 0x0C); }

  // Glosnosc 0..100% -> rejestr DAC (0..DAC_VOLUME_MAX).
  bool setVolume(uint8_t percent) {
    if (percent > 100) percent = 100;
    uint8_t reg = (uint8_t)(((uint16_t)percent * es8311::DAC_VOLUME_MAX) / 100u);
    return write(es8311::REG_DAC_VOLUME, reg);
  }

  // Pelne uspienie: wycisz DAC i wprowadz uklad w reset (bloki analogowe off).
  bool powerDown() {
    write(es8311::REG_DAC_VOLUME, 0x00);
    return write(es8311::REG_RESET, 0x1F);
  }

private:
  TwoWire &wire_;

  bool write(uint8_t reg, uint8_t value) {
    wire_.beginTransmission(es8311::I2C_ADDR);
    wire_.write(reg);
    wire_.write(value);
    return wire_.endTransmission() == 0;
  }
  uint8_t read(uint8_t reg) {
    wire_.beginTransmission(es8311::I2C_ADDR);
    wire_.write(reg);
    wire_.endTransmission(false);
    wire_.requestFrom((uint16_t)es8311::I2C_ADDR, (uint8_t)1, true);
    return wire_.available() ? wire_.read() : 0;
  }
  bool configSampleRate48k() {
    bool ok = true;
    ok &= write(0x02, 0x00);                    // preskalery MCLK
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
//  [12] DebouncedButton — debouncer + pomiar czasu wcisniecia jako mala maszyna
//  stanow. Jedna klasa obsluguje oba przyciski (KEY i PREVIEW/BOOT); polityka
//  "co znaczy krotkie/dlugie" nalezy do ButtonInput, nie do debouncera.
// ============================================================================
class DebouncedButton {
public:
  explicit DebouncedButton(gpio_num_t pin) : pin_(pin) {}

  // pinMode + zdjecie stanu poczatkowego BEZ generowania zdarzenia Pressed
  // (przycisk trzymany podczas startu nie wywoluje akcji az do zwolnienia).
  // Gdy pin juz jest wcisniety, uzbrajamy pomiar czasu — zwolnienie dostanie
  // zmierzona dlugosc przytrzymania zamiast "nieznanej".
  void begin() {
    pinMode(pin_, INPUT_PULLUP);
    rawLevel_ = stableLevel_ = digitalRead(pin_);
    lastChangeMs_ = millis();
    pressStartMs_ = millis();
    pressStartKnown_ = (stableLevel_ == LOW);
  }

  // Wolane co obieg petli. Zwraca zdarzenie krawedziowe po debounce.
  ButtonEvent poll() {
    bool raw = digitalRead(pin_);
    uint32_t now = millis();
    if (raw != rawLevel_) { rawLevel_ = raw; lastChangeMs_ = now; }
    if (now - lastChangeMs_ > btncfg::DEBOUNCE_MS && raw != stableLevel_) {
      stableLevel_ = raw;
      if (stableLevel_ == LOW) {
        pressStartMs_ = now;
        pressStartKnown_ = true;
        return ButtonEvent::Pressed;
      }
      lastPressKnown_ = pressStartKnown_;
      lastPressDurationMs_ = pressStartKnown_ ? (now - pressStartMs_) : 0;
      pressStartKnown_ = false;
      return ButtonEvent::Released;
    }
    return ButtonEvent::None;
  }

  // Wybudzenie ze snu przyciskiem: pin JUZ jest wcisniety, a zbocza nie bylo —
  // ustaw stan "wcisniety od teraz", by zwolnienie zmierzylo czas przytrzymania.
  // (begin() robi to samo dla przycisku trzymanego od startu — fix v8.0: dlugie
  //  przytrzymanie od bootu nie odpala juz akcji krotkiej przy zwolnieniu.)
  void primePressed() {
    uint32_t now = millis();
    rawLevel_ = stableLevel_ = LOW;
    lastChangeMs_ = now;
    pressStartMs_ = now;
    pressStartKnown_ = true;
  }

  bool isDown() const { return stableLevel_ == LOW; }

  // Wazne tylko bezposrednio po ButtonEvent::Released:
  // known=false gdy poczatku nacisniecia nie widzielismy (np. trzymany od bootu).
  bool     lastPressKnown() const      { return lastPressKnown_; }
  uint32_t lastPressDurationMs() const { return lastPressDurationMs_; }

private:
  gpio_num_t pin_;
  bool     rawLevel_ = HIGH;
  bool     stableLevel_ = HIGH;
  uint32_t lastChangeMs_ = 0;
  uint32_t pressStartMs_ = 0;
  bool     pressStartKnown_ = false;
  bool     lastPressKnown_ = false;
  uint32_t lastPressDurationMs_ = 0;
};

// ============================================================================
//  Koordynator (forward) — klasy logiki dostaja App& jako mediator do rodzenstwa
//  plus bezposrednie referencje do sprzetu, ktorym same steruja.
// ============================================================================
class App;

// ============================================================================
//  [13] RtcController — PCF85063A: czas (z wyrownaniem fazy), offset, alarm
//  wybudzajacy. Zalezy WYLACZNIE od szyny I2C (bez App — czysty sterownik+polityka).
// ============================================================================
class RtcController {
public:
  explicit RtcController(TwoWire &bus) : wire_(bus) {}

  bool readToSystem();
  bool readEpoch(time_t *epoch);                    // requireCurrentFormat = true
  bool writeTime(time_t epoch, bool audioActive);   // audioActive: pomija wyrownanie fazy
  bool ensureOffsetCalibration();
  bool setWakeAlarm(time_t epoch);
  bool disableWakeAlarm();
#if ENABLE_RTC_AUTOTRIM
  void setTargetOffsetLsb(int8_t lsb) { targetOffsetReg_ = encodeRtcOffset(lsb, false); }
  bool applyOffsetLsb(int8_t lsb);                  // zapis+weryfikacja rejestru 0x02
  bool measureDriftUs(int64_t *driftUs);            // faza RTC vs czas systemowy (~ms)
#endif

  // Diagnostyka offsetu (do /info).
  uint8_t offsetRaw() const       { return offsetRaw_; }
  uint8_t targetOffsetRaw() const { return targetOffsetReg_; }
  int8_t  offsetLsb() const       { return offsetLsb_; }
  float   offsetPpm() const       { return offsetPpm_; }
  bool    offsetVerified() const  { return offsetVerified_; }

private:
  TwoWire &wire_;
  uint8_t targetOffsetReg_ = rtccfg::FACTORY_OFFSET_REG;  // docelowe 0x02 (NVS moze nadpisac)
  uint8_t offsetRaw_      = 0xFF;
  int8_t  offsetLsb_      = 0;
  float   offsetPpm_      = 0.0f;
  bool    offsetVerified_ = false;

  bool recoverStuckBus();
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
//  [14] SoundStore — system plikow MP3 (SPIFFS/LittleFS/FFat), montowany leniwie.
// ============================================================================
class SoundStore {
public:
  bool ensureMounted();
  fs::FS *fs() const { return fs_; }
  // Nazwy renderowane w panelu WWW i /info — musza zostac znakowo identyczne z v7.x.
  const char *name() const {
    switch (backend_) {
      case FsBackend::Spiffs:   return "SPIFFS";
      case FsBackend::LittleFs: return "LittleFS";
      case FsBackend::FFatFs:   return "FFat";
      default:                  return "brak";
    }
  }

private:
  FsBackend   backend_     = FsBackend::None;
  fs::FS     *fs_          = nullptr;
  bool        formatTried_ = false;
  bool recoverSoundFilesOnce();
  void recoverSoundFiles();
};

// ============================================================================
//  [15] BatteryMonitor — ADC, krzywa OCV->SoC, estymator zuzycia, ochrona ogniwa
// ============================================================================
class BatteryMonitor {
public:
  explicit BatteryMonitor(App &app) : app_(app) {}

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
  bool     ready_      = false;
  bool     caliReady_  = false;
  uint32_t nextReadMs_ = 0;

  static uint8_t percentFromVoltage(float v);
  uint8_t stabilizePercent(uint8_t measured) const;
  void    maybeWriteDailySummary(time_t now);
};

// ============================================================================
//  [16] AudioPlayer — odtwarzanie MP3 (ES8311 + I2S) oraz petla "dzwonienia".
//  Sprzet (dekoder audioI2S + kodek) wstrzykniety w konstruktorze.
// ============================================================================
class AudioPlayer {
public:
  AudioPlayer(App &app, Audio &decoder, Es8311Codec &codec)
    : app_(app), decoder_(decoder), codec_(codec) {}

  bool ensureCodec();
  void shutdownCodec();
  void codecPowerDown();        // ES8311 w stan niskiego poboru (setup)
  bool startSound(uint8_t soundIndex, uint8_t volume);
  void stopPlayback(StopCause cause);
  void startRinging(int alarmIndex);
  void serviceAudio();
  void serviceRinging();

  bool isPlaying() const   { return playing_; }
  bool isRinging() const   { return ringing_; }
  bool codecActive() const { return ready_; }    // kodek nie uspiony (idlePowerSave)

private:
  App         &app_;
  Audio       &decoder_;
  Es8311Codec &codec_;

  void ensurePins();
  bool     pinsReady_ = false;
  bool     ready_     = false;     // kodek skonfigurowany i zasilony
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
//  [17] AlarmScheduler — model budzikow, harmonogram wyzwolen, dedup kolizji DST
// ============================================================================
class AlarmScheduler {
public:
  explicit AlarmScheduler(App &app) : app_(app) {}

  void setDefaults();
  void resetTriggerState();
  void sanitize(int index);
  void sanitizeAll();
  AlarmConfig       &config(int i)       { return alarms_[i]; }
  const AlarmConfig &config(int i) const { return alarms_[i]; }
  void              *rawData()           { return alarms_.data(); }   // NVS getBytes/putBytes
  size_t             rawSize() const     { return sizeof(alarms_); }

  bool anyEnabled() const;
  NextAlarmInfo nextAlarm() const;
  void checkAlarms(uint32_t lateWindowSec = alarmcfg::LATE_WINDOW_SEC);

  // Uzywane przez PowerPlanner przy planowaniu snu.
  bool   anyDueNow(const struct tm &t) const;
  time_t nextEpoch(const struct tm &base, time_t now) const;

private:
  App &app_;
  std::array<AlarmConfig, alarmcfg::ALARM_COUNT> alarms_;
  int  pendingAlarm_ = -1;   // budzik wymagalny podczas dzwonienia innego (fix v8.0)

  void queueWhileRinging(const struct tm &t, time_t now, uint32_t lateWindowSec);
  bool firePendingIfAny();
  // rawSize() zasila kontrakt NVS — std::array nie moze dolozyc paddingu.
  static_assert(sizeof(std::array<AlarmConfig, alarmcfg::ALARM_COUNT>) ==
                alarmcfg::ALARM_COUNT * sizeof(AlarmConfig), "NVS blob 'alarms'");
};

// ============================================================================
//  [18] NetworkTimeService — WiFi + NTP: nieblokujaca maszyna stanow (NtpState)
//  + harmonogram dobowy. Czas z RTC pierwszy; NTP NADPISUJE tylko po REALNYM
//  pakiecie (gate atomic).
// ============================================================================
class NetworkTimeService {
public:
  explicit NetworkTimeService(App &app) : app_(app) {}

  void syncNow(SyncTrigger trigger);
  bool service();                       // true gdy NTP wlasnie zatwierdzil nowy czas
  void checkSchedule();                 // dobowa synchronizacja 18:00 (+ dogonienie)
  void handleRtcWriteRetry();
  void stopWifiAndNtp();
  bool ensureWifi(uint32_t timeoutMs);  // blokujace laczenie (tylko panel WWW)

  void setRetries(uint8_t n)   { retriesLeft_ = n; }
  bool isBusy() const          { return state_ != NtpState::Idle; }
  bool rtcWritePending() const { return rtcWriteRetriesLeft_ != 0; }
  long secsSinceLastSync() const;

  const char *ssid() const { return ssid_; }
  const char *pass() const { return pass_; }   // tylko do zapisu NVS (NIE renderowane w HTML)
  void setCreds(const char *s, const char *p);
  void setSsid(const String &s);
  void setPass(const String &p);

private:
  App           &app_;
  NtpState       state_          = NtpState::Idle;
  bool           manualAttempt_  = false;
  bool           wifiHintActive_ = false;
  uint32_t       stateStartedMs_ = 0;
  struct timeval fallbackTime_   = {};   // snapshot czasu przed proba (rollback po bledzie)
  uint8_t        retriesLeft_ = 0;
  uint32_t       nextRetryMs_ = 0;
  uint32_t       backoffMs_   = ntpcfg::BACKOFF_MIN_MS;
  uint32_t       wifiLostMs_  = 0;       // od kiedy WiFi zerwane w trakcie WaitingForPacket
  uint8_t        rtcWriteRetriesLeft_ = 0;
  uint32_t       rtcWriteNextRetryMs_ = 0;
  char           ssid_[33] = "";
  char           pass_[65] = "";

  static void wifiPowerOff() { WiFi.disconnect(false, false); WiFi.mode(WIFI_OFF); }
  static void copyBounded(char *dst, size_t dstSize, const char *src);
  bool wifiBeginSmart();
  void wifiRetryFullScan();
  void wifiCacheHint();
  void stopSntpCallbacks();
  void restoreTimeAfterFailedNtp();
  void beginNtpRequest();
  void failNtpAttempt(const char *status);   // wspolna sciezka porazki w WaitingForPacket
  void finishNtpAttempt(bool ok);
  bool handleNtpRetry();
  void scheduleRtcWriteRetry(uint8_t retries = rtccfg::WRITE_RETRIES);
#if ENABLE_RTC_AUTOTRIM
  void maybeAutotrimRtc(time_t syncedEpoch);
#endif
};

// ============================================================================
//  [19] DisplayUi — strony ekranu (U8g2) + dedup repaintu przez sygnature widoku.
//  ClockView to zdjecie WIDOCZNEJ tresci strony zegara: render i anty-mruganie
//  pracuja na tych samych danych, liczonych raz na obieg.
// ============================================================================
struct ClockView {
  uint8_t  hour, minute;
  int      dayKey;
  uint8_t  batteryPercent;
  bool     webActive;
  bool     anyAlarmEnabled;
  bool     hasNextAlarm;
  uint8_t  nextAlarmHour, nextAlarmMinute;

  // FNV-1a po widocznej tresci. Pola niepokazywane (ntpStatus/statusText) NIE
  // wchodza -> zero pelnych sendBuffer() identycznej klatki (zero mrugniec
  // panelu refleksyjnego).
  uint32_t signature() const {
    constexpr uint32_t FNV_OFFSET_BASIS = 2166136261u;
    constexpr uint32_t FNV_PRIME        = 16777619u;
    uint32_t h = FNV_OFFSET_BASIS;
    auto mix = [&](uint32_t v) { h = (h ^ v) * FNV_PRIME; };
    mix(hour);
    mix(minute);
    mix((uint32_t)dayKey);
    mix(batteryPercent);
    mix(webActive ? 1u : 0u);
    mix(anyAlarmEnabled ? 1u : 0u);
    mix(hasNextAlarm ? (uint32_t)(nextAlarmHour * 60 + nextAlarmMinute + 1) : 0u);
    return h;
  }
};

class DisplayUi {
public:
  DisplayUi(App &app, RlcdDisplay &panel) : app_(app), panel_(panel) {}

  void setup();                     // pelny re-init panelu (zimny start i po deep-sleep)
  void update();                    // dispatcher stron wg UiPage
  void drawNightPage();             // wolane przez PowerPlanner przed deep-sleep
#if ENABLE_LOW_BATTERY_PROTECT
  void drawCriticalBatteryPage();   // jw. przed ochronnym snem
#endif

private:
  App         &app_;
  RlcdDisplay &panel_;
  uint32_t     lastClockSig_ = FORCE_REPAINT_SIG;

  static constexpr uint32_t FORCE_REPAINT_SIG = 0xFFFFFFFFu;  // sentinel = wymus repaint

  // --- uklad strony zegara (wspolrzedne w px; panel 400x300 landscape) ---
  static constexpr int BATTERY_ICON_X   = 360;
  static constexpr int BATTERY_ICON_Y   = 8;
  static constexpr int FOOTER_LINE_Y    = 264;
  static constexpr int FOOTER_TEXT_Y    = 287;
  static constexpr int FOOTER_TEXT_X    = 42;
  static constexpr int FOOTER_BELL_X    = 10;
  static constexpr int FOOTER_BELL_Y    = 270;

  UiPage    resolvePage() const;
  ClockView makeClockView(const struct tm &t, int dayKey) const;
  void      renderTimePages();      // NoTime / Clock (wspolne dla update i wygasniecia podgladu)
  void      drawPhotoGlyph(int x, int y, int glyph, bool colon);
  void      drawLargeTime(uint8_t hour, uint8_t minute, int y);
  void      drawBellIcon(int x, int y, bool active);
  void      drawBatteryIcon(int x, int y, uint8_t percent);
  void      drawCentered(const char *text, int y);
  void      drawClockPage(const ClockView &view);
  void      drawPreviewPage();
  void      drawRingingPage();
  void      drawNoTimePage();
};

// ============================================================================
//  [20] WebPanel — panel konfiguracji, sync, pliki MP3, /info (WiFi na zadanie)
// ============================================================================
class WebPanel {
public:
  WebPanel(App &app, WebServer &server) : app_(app), server_(server) {}

  bool startWebWindow();
  void service();              // obsluga w loop(): grace WiFi + handleClient + timeout
  bool isActive() const { return active_; }
  void forceStop();            // server.stop + active_=false (przy zasypianiu nocnym)

private:
  App       &app_;
  WebServer &server_;
  bool       active_          = false;
  bool       routesInstalled_ = false;
  uint32_t   untilMs_         = 0;
  uint32_t   wifiLostMs_      = 0;

  File     mp3UploadFile_;
  bool     mp3UploadAccepted_ = false;
  bool     mp3UploadHadData_  = false;
  String   mp3UploadPath_     = "";
  String   mp3UploadTempPath_ = "";

  void stopWindow();
  void installRoutes();
  void redirectHome();        // 303 -> "/" (wspolne zakonczenie akcji POST/GET)
  void extendWindow()  { untilMs_ = millis() + webcfg::WINDOW_MS; }
  void appendMp3Manager(String &html);
  void pageHeader(String &h);
  void handleRoot();
  void handleSave();
  void handleSync();
  void handleStop();
  void handleTest();
  void handleMp3UploadDone();
  void handleMp3UploadStream();
  void discardUploadTemp();   // sprzatanie .part po pustym/przerwanym uploadzie
  void handleMp3Delete();
  void handleWebOff();
#if ENABLE_DIAGNOSTICS
  void handleInfo();
#endif
};

// ============================================================================
//  [21] PowerPlanner — tryb nocny (deep-sleep), light-sleep dzienny, ochronny sen
// ============================================================================
class PowerPlanner {
public:
  explicit PowerPlanner(App &app) : app_(app) {}

  void releaseNightHolds();
  void enterNightSleepIfNeeded(bool force);
  void idlePowerSave();
#if ENABLE_LOW_BATTERY_PROTECT
  void enterCriticalBatterySleepIfNeeded();
#endif

private:
  App &app_;
  void     holdNightPins();
  void     shutdownNightPeripherals();
  // Wspolne zejscie do deep-sleep (noc + ochrona ogniwa): peryferia off, CPU w dol,
  // timer + EXT1 z pull-upami, hold pinow, flush. NIE WRACA.
  [[noreturn]] void beginDeepSleep(uint64_t sleepUs, uint64_t ext1Mask);
  time_t   nightEndEpoch(time_t now);
  uint64_t computeIdleSleepUs();
};

// ============================================================================
//  [22] ButtonInput — polityka przyciskow (co znaczy krotkie/dlugie nacisniecie)
//  + stan podgladu budzika i okna utrzymania wybudzenia.
// ============================================================================
class ButtonInput {
public:
  explicit ButtonInput(App &app)
    : app_(app), keyButton_(pins::KEY), previewButton_(pins::PREVIEW) {}

  void begin();
  void pollKey();
  void pollPreview();
  void showPreview();
  void primeKeyFromSetup();           // KEY z deep-sleep: zacznij mierzyc nacisniecie (bez wakeHold)
  void primeKeyFromLightSleepWake();  // wybudzenie z light-sleep: jw. + utrzymaj wybudzenie
  void onPreviewWakeFromLightSleep(); // wybudzenie z light-sleep przyciskiem PREVIEW

  void holdWake()           { wakeHoldUntilMs_ = millis() + btncfg::WAKE_HOLD_MS; }
  bool wakeHeld() const     { return (int32_t)(millis() - wakeHoldUntilMs_) < 0; }
  bool previewActive() const       { return previewActive_; }
  bool previewWithinWindow() const { return (int32_t)(millis() - previewUntilMs_) < 0; }
  void clearPreview()              { previewActive_ = false; }

private:
  App            &app_;
  DebouncedButton keyButton_;
  DebouncedButton previewButton_;
  bool            previewActive_  = false;
  uint32_t        previewUntilMs_ = 0;
  uint32_t        wakeHoldUntilMs_ = 0;
};

// ============================================================================
//  [23] SettingsStore — NVS (budziki + WiFi + offset RTC + liczniki resetow).
//  JEDYNA klasa dotykajaca Preferences: pelna kontrola nad kluczami i zgodnoscia.
// ============================================================================
// Straznik RAII sesji Preferences: kazde wyjscie z funkcji (takze wczesny return)
// domyka NVS. Zero heapu, zero wyjatkow.
class NvsSession {
public:
  explicit NvsSession(Preferences &prefs)
    : prefs_(prefs), open_(prefs.begin(nvskeys::NAMESPACE_NAME, false)) {}
  ~NvsSession() { if (open_) prefs_.end(); }
  explicit operator bool() const { return open_; }
  NvsSession(const NvsSession &) = delete;
  NvsSession &operator=(const NvsSession &) = delete;

private:
  Preferences &prefs_;
  bool         open_;
};

class SettingsStore {
public:
  SettingsStore(App &app, Preferences &prefs) : app_(app), prefs_(prefs) {}

  void load();
  void save();
  void saveDrainSummary(uint8_t soc, float mahPerDay, int day);
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
  App         &app_;
  Preferences &prefs_;
#if ENABLE_DIAGNOSTICS
  uint32_t brownoutResets_ = 0;
  uint32_t wdtResets_      = 0;
  uint32_t panicResets_    = 0;
#endif
};

// ============================================================================
//  [24] App — koordynator (mediator). Trzyma wszystkie moduly i stan UI; sprzet
//  dostaje w konstruktorze i rozdaje modulom przez referencje (lekki DI: kazdy
//  modul widzi tylko to, czym steruje, a rodzenstwo osiaga przez App&).
// ============================================================================
class App {
public:
  App(TwoWire &i2c, RlcdDisplay &panel, WebServer &server, Preferences &prefs, Audio &decoder)
    : rtc_(i2c), codec_(i2c),
      battery_(*this), audio_(*this, decoder, codec_), alarms_(*this),
      ntp_(*this), display_(*this, panel), web_(*this, server), power_(*this),
      buttons_(*this), settings_(*this, prefs) {}

  void setup();
  void loop();

  // --- mediator: dostep do rodzenstwa ---
  SoundStore         &soundStore() { return soundStore_; }
  RtcController      &rtc()        { return rtc_; }
  BatteryMonitor     &battery()    { return battery_; }
  AudioPlayer        &audio()      { return audio_; }
  AlarmScheduler     &alarms()     { return alarms_; }
  NetworkTimeService &ntp()        { return ntp_; }
  DisplayUi          &displayUi()  { return display_; }
  WebPanel           &web()        { return web_; }
  PowerPlanner       &power()      { return power_; }
  ButtonInput        &buttons()    { return buttons_; }
  SettingsStore      &settings()   { return settings_; }
  PersistentState    &persist()    { return g_persist; }

  // --- stan UI (flaga repaintu + teksty statusow) ---
  void markDirty()                   { dirty_ = true; }
  void clearDirty()                  { dirty_ = false; }
  bool dirty() const                 { return dirty_; }
  void setStatus(const String &s)    { statusText_ = s; }
  const String &statusText() const   { return statusText_; }
  void setNtpStatus(const String &s) { ntpStatus_ = s; }
  const String &ntpStatus() const    { return ntpStatus_; }

  // --- bramki wspoldzielone ---
  bool audioActive() const { return audio_.isPlaying() || audio_.isRinging(); }
  // Stan, w ktorym NIE wolno uspic na dlugo (jedna definicja dla wszystkich snow).
  bool busy() const {
    return audioActive() || web_.isActive() || buttons_.previewActive() ||
           ntp_.isBusy() || ntp_.rtcWritePending();
  }

private:
  SoundStore         soundStore_;
  RtcController      rtc_;
  Es8311Codec        codec_;
  BatteryMonitor     battery_;
  AudioPlayer        audio_;
  AlarmScheduler     alarms_;
  NetworkTimeService ntp_;
  DisplayUi          display_;
  WebPanel           web_;
  PowerPlanner       power_;
  ButtonInput        buttons_;
  SettingsStore      settings_;

  bool     dirty_      = true;
  String   statusText_ = "IDLE";
  String   ntpStatus_  = "never";
  uint32_t lastAlarmCheckMs_ = 0;   // loop(): poszerza okno spoznienia o przestoj petli
};

// Jedyna instancja koordynatora — tu widac cale okablowanie sprzetu.
static App app(Wire, display, httpServer, preferences, audioOutput);

// ============================================================================
//  IMPLEMENTACJE — RtcController (PCF85063A)
// ============================================================================
// Odblokowanie zawieszonej szyny I2C: slave (RTC/kodek) trzymajacy SDA nisko po
// resecie ESP w polowie transakcji zwalnia ja po maks. 9 impulsach SCL + STOP.
// Bez tego kazda sesja RTC pada az do odlaczenia zasilania. Zwraca true, gdy
// szyna byla zawieszona i zostala uwolniona.
bool RtcController::recoverStuckBus() {
  // Sonda BEZ pinMode: rekonfiguracja pinu I2C w core 3.x przechodzi przez
  // Peripheral Manager i deinicjalizuje caly sterownik Wire (wspoldzielony z
  // ES8311) — nawet przy zwyklym NACK. gpio_get_level czyta stan szyny nie
  // dotykajac konfiguracji (bufor wejsciowy SDA wlaczony przez sterownik I2C).
  if (gpio_get_level(pins::I2C_SDA) == 1) return false;   // SDA wolne -> zwykly NACK
  wire_.end();                                            // jawny demontaz PRZED przejeciem pinow
  pinMode(pins::I2C_SDA, INPUT_PULLUP);
  pinMode(pins::I2C_SCL, OUTPUT_OPEN_DRAIN);
  digitalWrite(pins::I2C_SCL, HIGH);
  for (uint8_t i = 0; i < i2cbus::RECOVER_MAX_PULSES && digitalRead(pins::I2C_SDA) == LOW; i++) {
    digitalWrite(pins::I2C_SCL, LOW);  delayMicroseconds(i2cbus::RECOVER_HALF_PERIOD_US);
    digitalWrite(pins::I2C_SCL, HIGH); delayMicroseconds(i2cbus::RECOVER_HALF_PERIOD_US);
  }
  pinMode(pins::I2C_SDA, OUTPUT_OPEN_DRAIN);              // warunek STOP: SDA lo->hi przy SCL hi
  digitalWrite(pins::I2C_SDA, LOW);  delayMicroseconds(i2cbus::RECOVER_HALF_PERIOD_US);
  digitalWrite(pins::I2C_SDA, HIGH); delayMicroseconds(i2cbus::RECOVER_HALF_PERIOD_US);
  bool freed = digitalRead(pins::I2C_SDA) == HIGH;
  Serial.printf("I2C: szyna zawieszona -> odzysk %s\n", freed ? "OK" : "NIEUDANY");
  return freed;
}

bool RtcController::beginSession() {
  wire_.begin(pins::I2C_SDA, pins::I2C_SCL);
  wire_.setClock(i2cbus::FREQ_HZ);
  wire_.beginTransmission(pcf85063::I2C_ADDR);
  if (wire_.endTransmission() == 0) return true;
  if (!recoverStuckBus()) return false;                   // NACK bez zwisu SDA -> zwykly blad
  wire_.begin(pins::I2C_SDA, pins::I2C_SCL);              // po odzysku pelna re-inicjalizacja
  wire_.setClock(i2cbus::FREQ_HZ);
  wire_.beginTransmission(pcf85063::I2C_ADDR);
  return wire_.endTransmission() == 0;
}

bool RtcController::writeReg(uint8_t reg, uint8_t value) {
  wire_.beginTransmission(pcf85063::I2C_ADDR);
  wire_.write(reg);
  wire_.write(value);
  return wire_.endTransmission() == 0;
}

// Odczyt n rejestrow od 'reg' jednym transferem. Wolajacy juz otworzyl sesje.
bool RtcController::readRegs(uint8_t reg, uint8_t *buf, uint8_t n) {
  wire_.beginTransmission(pcf85063::I2C_ADDR);
  wire_.write(reg);
  if (wire_.endTransmission(false) != 0) return false;
  if (wire_.requestFrom((uint8_t)pcf85063::I2C_ADDR, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = wire_.read();
  return true;
}

bool RtcController::readControl1(uint8_t *control1) {
  if (!control1 || !beginSession()) return false;
  return readRegs(pcf85063::REG_CONTROL1, control1, 1);
}
bool RtcController::readControl2(uint8_t *control2) {
  if (!control2 || !beginSession()) return false;
  return readRegs(pcf85063::REG_CONTROL2, control2, 1);
}
bool RtcController::writeControl1(uint8_t control1) {
  return writeReg(pcf85063::REG_CONTROL1, control1);
}

bool RtcController::releaseClock(uint8_t runningControl1) {
  for (uint8_t attempt = 0; attempt < 3; attempt++) {
    if (writeControl1(runningControl1)) return true;
    delay(2);
  }
  return false;
}

// Zatrzymaj licznik przed ustawieniem czasu. STOP zeruje glowna czesc preskalera
// -> pierwszy impuls sekundowy ma okreslona faze.
bool RtcController::prepareTimeWrite(uint8_t *runningControl1) {
  if (!runningControl1) return false;
  uint8_t control1;
  if (!readControl1(&control1)) return false;
  *runningControl1 = control1 & (uint8_t)~pcf85063::CTRL1_RUN_CLEAR_MASK;
  if (writeControl1(*runningControl1 | pcf85063::CTRL1_STOP)) return true;
  releaseClock(*runningControl1);
  return false;
}

// Marker w bajcie RAM RTC: "czas zapisal ten firmware w znanym formacie".
// Brak markera = RTC ustawiony przez cos innego / niepelny zapis -> nie ufamy.
bool RtcController::hasCurrentFormat() {
  uint8_t marker;
  return readRegs(pcf85063::REG_RAM_BYTE, &marker, 1) && marker == pcf85063::FORMAT_MARKER;
}
bool RtcController::storeCurrentFormat()      { return writeReg(pcf85063::REG_RAM_BYTE, pcf85063::FORMAT_MARKER); }
bool RtcController::invalidateCurrentFormat() { return writeReg(pcf85063::REG_RAM_BYTE, 0); }

// Odczyt 7 rejestrow czasu (0x04..0x0A) jednym transferem -> brak rwania na granicy
// sekundy. Rok 00..99 = 2000..2099.
bool RtcController::readEpochInternal(time_t *epoch, bool requireCurrentFormat) {
  if (!epoch) return false;
  uint8_t control1;
  if (!readControl1(&control1)) return false;

  // Nie wznawiaj po cichu zatrzymanego/przestawionego RTC (staly czas != aktualny).
  if ((control1 & pcf85063::CTRL1_INVALID_MODE_MASK) != 0 ||
      (requireCurrentFormat && !hasCurrentFormat())) return false;

  uint8_t t[7];
  if (!readRegs(pcf85063::REG_SECONDS, t, 7)) return false;
  uint8_t secReg = t[0], minReg = t[1], hourReg = t[2], dayReg = t[3];
  uint8_t monReg = t[5], yearReg = t[6];   // t[4] = dzien tygodnia (pomijamy)

  bool oscStopped = (secReg & pcf85063::SECONDS_OS) != 0;
  secReg &= 0x7F; minReg &= 0x7F; hourReg &= 0x3F; dayReg &= 0x3F; monReg &= 0x1F;

  if (oscStopped || !validBcd(secReg) || !validBcd(minReg) || !validBcd(hourReg) ||
      !validBcd(dayReg) || !validBcd(monReg) || !validBcd(yearReg)) return false;

  int s  = bcd2dec(secReg), mi = bcd2dec(minReg), h = bcd2dec(hourReg);
  int d  = bcd2dec(dayReg), mo = bcd2dec(monReg), y = bcd2dec(yearReg) + 2000;

  if (!validDateTime(y, mo, d, h, mi, s)) return false;
  *epoch = utcEpochFromCivil(y, mo, d, h, mi, s);
  return true;
}

bool RtcController::readEpoch(time_t *epoch) { return readEpochInternal(epoch, true); }

bool RtcController::readToSystem() {
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
bool RtcController::writeTime(time_t epoch, bool audioActive) {
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

  wire_.beginTransmission(pcf85063::I2C_ADDR);
  wire_.write(pcf85063::REG_SECONDS);
  wire_.write(dec2bcd(gm.tm_sec) & 0x7F);            // OS=0 -> oscylator OK
  wire_.write(dec2bcd(gm.tm_min) & 0x7F);
  wire_.write(dec2bcd(gm.tm_hour) & 0x3F);
  wire_.write(dec2bcd(gm.tm_mday) & 0x3F);
  wire_.write((uint8_t)(gm.tm_wday & 0x07));
  wire_.write(dec2bcd((uint8_t)(gm.tm_mon + 1)) & 0x1F);
  wire_.write(dec2bcd(yearReg));
  if (wire_.endTransmission() != 0) { releaseClock(runningControl1); return false; }

  // Odczekaj do granicy docelowej sekundy (czas I2C wliczony), potem zwolnij STOP.
  if (alignToBoundary) {
    struct timeval nowTv;
    for (;;) {
      gettimeofday(&nowTv, nullptr);
      int64_t remainingUs = ((int64_t)writeEpoch - nowTv.tv_sec) * 1000000LL - nowTv.tv_usec;
      if (remainingUs <= 0) break;
      // Startowalismy z <=1 s zapasu; wiecej oznacza, ze rownolegly settimeofday
      // (np. pakiet SNTP w tasku lwIP) skoczyl czasem — kotwica stracila sens.
      // Przerwij CALY zapis: marker formatu juz uniewazniony, retry ponowi zapis
      // swiezym czasem (fix v8.0: wczesniej petla mogla wisiec do resetu TWDT).
      if (remainingUs > rtccfg::ALIGN_ABORT_US) { releaseClock(runningControl1); return false; }
      watchdogFeed();
      if (remainingUs > 2000) delay(1); else delayMicroseconds((uint32_t)remainingUs);
    }
  }
  if (!releaseClock(runningControl1)) return false;

  time_t verifiedEpoch;
  if (!readEpochInternal(&verifiedEpoch, false)) return false;
  long long diff = (long long)verifiedEpoch - (long long)writeEpoch;
  if (diff < -rtccfg::WRITE_VERIFY_TOLERANCE_SEC || diff > rtccfg::WRITE_VERIFY_TOLERANCE_SEC) return false;
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
// FACTORY_OFFSET_REG; SettingsStore::load moze nadpisac wartoscia z NVS).
bool RtcController::ensureOffsetCalibration() {
  offsetVerified_ = false;
  for (uint8_t attempt = 0; attempt < rtccfg::OFFSET_WRITE_RETRIES; attempt++) {
    if (!beginSession()) { delay(2); continue; }
    writeReg(pcf85063::REG_OFFSET, targetOffsetReg_);
    uint8_t raw;
    if (readRegs(pcf85063::REG_OFFSET, &raw, 1)) {
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
bool RtcController::applyOffsetLsb(int8_t lsb) {
  setTargetOffsetLsb(lsb);
  return ensureOffsetCalibration();
}

// Pomiar dryftu z rozdzielczoscia ~ms zamiast 1 s: czekaj (polling I2C) na
// przewrot sekundy RTC i zlap te chwile wg zegara systemowego. Wolane ZARAZ po
// pakiecie NTP (czas systemowy = wzorzec) i PRZED writeTime (nadpisze faze).
// W przewrocie faza RTC = dokladnie 0.000 s, wiec epokaRTC*1e6 - czas_sys_us
// = dryft narosly od ostatniego wyrownanego zapisu. Blokuje do ~1.3 s — wolac
// tylko bez aktywnego audio (busy-wait zaglodzilby audio.loop()).
bool RtcController::measureDriftUs(int64_t *driftUs) {
  if (!driftUs || !beginSession()) return false;
  uint8_t sec0, sec;
  if (!readRegs(pcf85063::REG_SECONDS, &sec0, 1)) return false;
  sec0 &= 0x7F;
  uint32_t startMs = millis();
  do {
    if (millis() - startMs > rtccfg::BOUNDARY_POLL_TIMEOUT_MS) return false;   // brak przewrotu: oscylator stoi
    if (!readRegs(pcf85063::REG_SECONDS, &sec, 1)) return false;
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

// INT sluzy wylacznie alarmowi wybudzajacemu. Wylacz korekcje (CIE), timer i
// CLKOUT (mniejszy pobor RTC); AIE tylko gdy alarm uzbrojony.
bool RtcController::configureWakeInterrupt(bool alarmEnabled) {
  uint8_t control1;
  if (!readControl1(&control1)) return false;
  control1 &= (uint8_t)~pcf85063::CTRL1_CIE;
  if (!writeReg(pcf85063::REG_CONTROL1, control1)) return false;
  if (!writeReg(pcf85063::REG_TIMER_MODE, pcf85063::TIMER_MODE_OFF)) return false;
  if (!writeReg(pcf85063::REG_TIMER_VALUE, 0)) return false;
  uint8_t control2;
  if (!readControl2(&control2)) return false;
  // Zapis control2 kasuje przy okazji flagi AF/TF (pola flagowe piszemy zerem).
  control2 = (uint8_t)(pcf85063::CTRL2_COF_OFF | (alarmEnabled ? pcf85063::CTRL2_AIE : 0));
  return writeReg(pcf85063::REG_CONTROL2, control2);
}

bool RtcController::verifyWakeInterruptConfig(bool alarmEnabled) {
  uint8_t control1, control2;
  if (!readControl1(&control1) || !readControl2(&control2)) return false;
  if (!beginSession()) return false;
  uint8_t timer[2];
  if (!readRegs(pcf85063::REG_TIMER_VALUE, timer, 2)) return false;
  uint8_t expectedControl2 = alarmEnabled ? pcf85063::CTRL2_AIE : 0;
  return (control1 & pcf85063::CTRL1_CIE) == 0 &&
         (control2 & 0xF8) == expectedControl2 &&
         timer[0] == 0 && timer[1] == pcf85063::TIMER_MODE_OFF;
}

bool RtcController::verifyWakeAlarm(time_t epoch) {
  struct tm gm;
  gmtime_r(&epoch, &gm);
  if (!beginSession()) return false;
  uint8_t a[5];
  if (!readRegs(pcf85063::REG_SECOND_ALARM, a, 5)) return false;
  if (a[0] != (dec2bcd((uint8_t)gm.tm_sec) & 0x7F) ||
      a[1] != (dec2bcd((uint8_t)gm.tm_min) & 0x7F) ||
      a[2] != (dec2bcd((uint8_t)gm.tm_hour) & 0x3F) ||
      a[3] != (dec2bcd((uint8_t)gm.tm_mday) & 0x3F) ||
      a[4] != pcf85063::ALARM_AEN_OFF) return false;
  return verifyWakeInterruptConfig(true);
}

// Alarm PCF85063A pracuje w UTC (jak rejestry czasu). INT aktywne stanem niskim.
// Odmowa gdy RTC i czas systemowy rozjechane > WAKE_MAX_SKEW_SEC — alarm liczony
// z czasu systemowego musi trafic w czas RTC.
bool RtcController::setWakeAlarm(time_t epoch) {
  time_t systemNow = time(nullptr);
  if (!isSupportedEpoch(epoch) || !isSupportedEpoch(systemNow)) return false;
  time_t rtcNow;
  if (!readEpoch(&rtcNow)) return false;     // odrzuca STOP/12h
  long long rtcSkew = (long long)rtcNow - (long long)systemNow;
  if (rtcSkew < -rtccfg::WAKE_MAX_SKEW_SEC || rtcSkew > rtccfg::WAKE_MAX_SKEW_SEC) return false;
  if (!configureWakeInterrupt(false)) return false;

  struct tm gm;
  gmtime_r(&epoch, &gm);
  if (!beginSession()) return false;
  wire_.beginTransmission(pcf85063::I2C_ADDR);
  wire_.write(pcf85063::REG_SECOND_ALARM);
  wire_.write(dec2bcd((uint8_t)gm.tm_sec) & 0x7F); // AEN=0 -> pole aktywne
  wire_.write(dec2bcd((uint8_t)gm.tm_min) & 0x7F);
  wire_.write(dec2bcd((uint8_t)gm.tm_hour) & 0x3F);
  wire_.write(dec2bcd((uint8_t)gm.tm_mday) & 0x3F);
  wire_.write(pcf85063::ALARM_AEN_OFF);            // Weekday_alarm wylaczony
  if (wire_.endTransmission() != 0) return false;

  if (!configureWakeInterrupt(true)) return false;
  return verifyWakeAlarm(epoch);
}

bool RtcController::disableWakeAlarm() {
  return configureWakeInterrupt(false) && verifyWakeInterruptConfig(false);
}

// ============================================================================
//  IMPLEMENTACJE — SoundStore (system plikow MP3)
// ============================================================================
// Dokoncz przerwana instalacje MP3 (patrz WebPanel::handleMp3UploadStream):
// osierocony .bak wraca na miejsce (lub znika, gdy cel istnieje), osierocony
// .part jest sprzatany. Jedno przejscie = jedna naprawa; wolane w petli.
bool SoundStore::recoverSoundFilesOnce() {
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

void SoundStore::recoverSoundFiles() {
  if (!fs_) return;
  for (uint8_t pass = 0; pass < fscfg::RECOVER_MAX_PASSES && recoverSoundFilesOnce(); pass++) {}
}

bool SoundStore::ensureMounted() {
  if (backend_ != FsBackend::None && fs_) return true;

  // Probuj po kolei znane systemy plikow (bez formatowania).
  struct Candidate { fs::FS *fs; FsBackend backend; bool ok; };
  Candidate candidates[] = {
    { &SPIFFS,   FsBackend::Spiffs,   SPIFFS.begin(false) },
    { &LittleFS, FsBackend::LittleFs, false },
    { &FFat,     FsBackend::FFatFs,   false },
  };
  if (!candidates[0].ok) candidates[1].ok = LittleFS.begin(false);
  if (!candidates[0].ok && !candidates[1].ok) candidates[2].ok = FFat.begin(false);
  for (auto &c : candidates) {
    if (c.ok) {
      backend_ = c.backend; fs_ = c.fs;
      recoverSoundFiles();
      Serial.printf("System plikow: %s\n", name());
      return true;
    }
  }
  // Fabrycznie czysty flash: jednorazowy format SPIFFS, by panel WWW mogl wgrac MP3.
  if (!formatTried_) {
    formatTried_ = true;
    Serial.println("Brak FS - formatuje SPIFFS...");
    if (SPIFFS.begin(true)) {
      backend_ = FsBackend::Spiffs; fs_ = &SPIFFS;
      Serial.println("System plikow: SPIFFS (nowo sformatowany)");
      return true;
    }
  }
  backend_ = FsBackend::None; fs_ = nullptr;
  Serial.println("Brak systemu plikow z MP3");
  return false;
}

// ============================================================================
//  IMPLEMENTACJE — BatteryMonitor
// ============================================================================
bool BatteryMonitor::init() {
  if (ready_) return true;
  adc_oneshot_unit_init_cfg_t initCfg = {};
  initCfg.unit_id = ADC_UNIT_1;
  if (adc_oneshot_new_unit(&initCfg, &adc_) != ESP_OK) return false;

  adc_oneshot_chan_cfg_t chanCfg = {};
  chanCfg.bitwidth = ADC_BITWIDTH_12;
  chanCfg.atten = ADC_ATTEN_DB_12;
  if (adc_oneshot_config_channel(adc_, batcfg::ADC_CHANNEL, &chanCfg) != ESP_OK) {
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

// Interpolacja liniowa po krzywej OCV->SoC (batcfg::OCV_CURVE).
uint8_t BatteryMonitor::percentFromVoltage(float v) {
  const auto &curve = batcfg::OCV_CURVE;
  if (v <= curve.front().voltage) return 0;
  if (v >= curve.back().voltage)  return 100;
  for (size_t i = 1; i < curve.size(); i++) {
    if (v < curve[i].voltage) {
      float frac = (v - curve[i-1].voltage) / (curve[i].voltage - curve[i-1].voltage);
      return (uint8_t)(curve[i-1].soc + frac * (curve[i].soc - curve[i-1].soc) + 0.5f);
    }
  }
  return 100;
}

// Histereza pelnego naladowania: >=99% pokazuj 100% i trzymaj, az spadnie <98%.
uint8_t BatteryMonitor::stabilizePercent(uint8_t measured) const {
  if (measured >= batcfg::FULL_FLOOR) return 100;
  if (g_persist.batteryHasReading && g_persist.batteryPercent == 100 &&
      measured >= batcfg::FULL_HOLD_FLOOR) return 100;
  return measured;
}

// Zwiezle podsumowanie zuzycia do NVS max raz/dobe (nadpisywane, malo flash).
void BatteryMonitor::maybeWriteDailySummary(time_t now) {
  struct tm t; localtime_r(&now, &t);
  int day = localDayKey(t);
  if (day == g_persist.lastSummaryDay) return;
  float perDay = g_persist.drain.mahPerDay();
  if (perDay < 0.0f) return;
  g_persist.lastSummaryDay = day;              // oznacz dzien NIM sprobujesz zapisu
  app_.settings().saveDrainSummary(g_persist.batteryPercent, perDay, day);
  Serial.printf("Zuzycie: %.1f mAh/dzien, ~%.0f dni\n",
                perDay, g_persist.drain.daysRemaining(g_persist.batteryPercent));
}

void BatteryMonitor::update(bool force) {
  if (!force && (int32_t)(millis() - nextReadMs_) < 0) return;

  // Pod obciazeniem (audio, dzwoniacy budzik, serwer web) napiecie zacisku siada
  // przez rezystancje wewnetrzna ogniwa -> pomiar zanizony. Odlozyc do bezczynnosci.
  if (!force && (app_.audioActive() || app_.web().isActive())) {
    nextReadMs_ = millis() + batcfg::BUSY_RETRY_MS; return;
  }

  nextReadMs_ = millis() + batcfg::READ_INTERVAL_MS;
  if (!init()) { nextReadMs_ = millis() + batcfg::BUSY_RETRY_MS; return; }

  uint32_t sum = 0; uint8_t samples = 0;
  for (uint8_t i = 0; i < batcfg::SAMPLE_COUNT; i++) {
    int raw = 0;
    if (adc_oneshot_read(adc_, batcfg::ADC_CHANNEL, &raw) == ESP_OK) { sum += (uint32_t)raw; samples++; }
  }
  if (samples == 0) { nextReadMs_ = millis() + batcfg::BUSY_RETRY_MS; return; }

  int raw = (int)((sum + samples / 2) / samples);
  int mv = 0;
  if (caliReady_) adc_cali_raw_to_voltage(cali_, raw, &mv);
  else mv = (raw * batcfg::ADC_FALLBACK_FULL_MV) / batcfg::ADC_MAX_RAW;   // bez kalibracji eFuse

  float measured = (mv * 0.001f) * batcfg::DIVIDER_RATIO * batcfg::VOLTAGE_CALIBRATION;
  if (!g_persist.batteryHasReading || force) { g_persist.batteryVoltage = measured; g_persist.batteryHasReading = true; }
  else g_persist.batteryVoltage = g_persist.batteryVoltage * batcfg::EMA_KEEP +
                                  measured * (1.0f - batcfg::EMA_KEEP);   // EMA

  uint8_t old = g_persist.batteryPercent;
  g_persist.batteryPercent = stabilizePercent(percentFromVoltage(g_persist.batteryVoltage));
  time_t nowT = time(nullptr);
  if (isSupportedEpoch(nowT)) {
    g_persist.drain.onSample((int64_t)nowT, g_persist.batteryPercent);
    maybeWriteDailySummary(nowT);
#if ENABLE_DIAGNOSTICS
    if (g_persist.coldBootEpoch == 0) g_persist.coldBootEpoch = (int64_t)nowT;
#endif
  }
  if (force || old != g_persist.batteryPercent) app_.markDirty();
}

#if ENABLE_LOW_BATTERY_PROTECT
bool BatteryMonitor::isLow() const {
  return g_persist.batteryHasReading && g_persist.batteryPercent <= batcfg::LOW_PERCENT;
}
// update() bramkuje pomiar pod obciazeniem, wiec batteryVoltage ~ OCV.
bool BatteryMonitor::isCritical() const {
  return g_persist.batteryHasReading &&
         (g_persist.batteryVoltage <= batcfg::CRITICAL_VOLTAGE ||
          g_persist.batteryPercent <= batcfg::CRITICAL_PERCENT);
}
#endif

// ============================================================================
//  IMPLEMENTACJE — AudioPlayer (ES8311 + MP3 przez I2S, petla dzwonienia)
// ============================================================================
void AudioPlayer::ensurePins() {
  if (pinsReady_) return;
  Audio::audio_info_callback = audioLogCallback;
  decoder_.setPinout(pins::I2S_BCLK, pins::I2S_WS, pins::I2S_DOUT, pins::I2S_MCLK);
  decoder_.setOutput48KHz(true);
  decoder_.setVolume(alarmcfg::DEFAULT_VOLUME);
  pinsReady_ = true;
}

bool AudioPlayer::ensureCodec() {
  if (ready_) return true;
  setCpuFrequencyMhz(powercfg::AUDIO_CPU_MHZ);  // dekoder MP3 potrzebuje zapasu ponad dzienne 80 MHz
  amplifierOff();   // PA wlacza dopiero startSound() PO konfiguracji kodeka (bez trzasku)
  ensurePins();
  ready_ = codec_.begin(pins::I2C_SDA, pins::I2C_SCL, i2cbus::FREQ_HZ);
  if (ready_) { codec_.setVolume(alarmcfg::CODEC_VOLUME_PERCENT); codec_.configBits16(); }
  else {
    Serial.println("ES8311 init failed");
    digitalWrite(pins::PA_ENABLE, LOW);
    setCpuFrequencyMhz(powercfg::DAY_CPU_MHZ);  // audio nie ruszy -> nie trzymaj podbitego CPU
  }
  return ready_;
}

void AudioPlayer::codecPowerDown() { codec_.powerDown(); }

// Uspij kodek po odtwarzaniu — ES8311 jest na stalej szynie 3V3, wiec to najwiekszy
// odbiornik w spoczynku. startSound() go wskrzesza. Zdejmuje tez podbicie CPU
// (nocne sciezki i tak ustawiaja PRE_SLEEP_CPU_MHZ PO tym wywolaniu).
void AudioPlayer::shutdownCodec() {
  if (!ready_) { digitalWrite(pins::PA_ENABLE, LOW); return; }
  decoder_.stopSong();
  codec_.powerDown();
  digitalWrite(pins::PA_ENABLE, LOW);
  ready_ = false;
  setCpuFrequencyMhz(powercfg::DAY_CPU_MHZ);
}

bool AudioPlayer::startSound(uint8_t soundIndex, uint8_t volume) {
  stopPlayback(StopCause::Auto);
  if (soundIndex >= alarmcfg::SOUND_COUNT) soundIndex = 0;

  // Wspolne zakonczenie bledu: kodek spac, status na ekran, repaint.
  auto fail = [&](const String &status, bool codecUp) {
    if (codecUp) shutdownCodec();
    app_.setStatus(status);
    app_.markDirty();
    return false;
  };

  if (!app_.soundStore().ensureMounted() || !ensureCodec()) return fail("blad FS/kodek", false);
  const char *path = alarmcfg::SOUND_FILES[soundIndex];
  if (!app_.soundStore().fs()->exists(path)) return fail(String("brak ") + path, true);

  digitalWrite(pins::PA_ENABLE, HIGH);
  decoder_.stopSong();
  decoder_.setVolume(constrain(volume, 0, alarmcfg::MAX_VOLUME));
  codec_.setVolume(alarmcfg::CODEC_VOLUME_PERCENT);
  if (!decoder_.connecttoFS(*app_.soundStore().fs(), path))
    return fail(String("blad odtw. ") + path, true);

  soundStartedMs_ = millis();
  playing_ = true;
  Serial.printf("Odtwarzam %s vol=%u\n", path, volume);
  app_.markDirty();
  return true;
}

void AudioPlayer::stopPlayback(StopCause cause) {
  bool userStopped = (cause == StopCause::User);
  if (userStopped) { ringing_ = false; ringingAlarm_ = -1; ringErrors_ = 0; }
  if (ready_) {
    decoder_.stopSong();
    decoder_.setVolume(0);
    codec_.setVolume(0);
    digitalWrite(pins::PA_ENABLE, LOW);
  }
  if (playing_) { app_.setStatus(userStopped ? "STOP" : "koniec"); playing_ = false; app_.markDirty(); }
}

void AudioPlayer::startRinging(int alarmIndex) {
  ringing_ = true;
  ringingAlarm_ = alarmIndex;
  ringStartMs_ = millis();
  ringLastRetryMs_ = millis();
  ringErrors_ = 0;
  const AlarmConfig &alarm = app_.alarms().config(alarmIndex);
  app_.setStatus(String("BUDZI: ") + alarm.label);
  app_.markDirty();
  if (!startSound(alarm.sound, alarm.volume)) {
    ringErrors_ = 1;
    ringLastRetryMs_ = millis() - alarmcfg::RING_RETRY_MS;
  }
}

void AudioPlayer::serviceAudio() {
  if (!playing_) return;
  if (!ready_) {
    playing_ = false;
    digitalWrite(pins::PA_ENABLE, LOW);
    if (!ringing_) app_.setStatus("audio stop");
    app_.markDirty();
    return;
  }
  decoder_.loop();
  if (!decoder_.isRunning()) {
    if (ringing_) {
      // connecttoFS() zwraca true dla KAZDEGO istniejacego pliku; uszkodzony MP3
      // "konczy sie" po kilkudziesieciu ms. Bez zliczania budzik krecil sie cicho.
      playing_ = false;
      if (millis() - soundStartedMs_ < alarmcfg::RING_MIN_PLAY_MS) {
        if (++ringErrors_ >= alarmcfg::RING_MAX_ERRORS) {
          app_.setStatus("budzik: blad MP3"); ringing_ = false; ringingAlarm_ = -1;
        }
      } else ringErrors_ = 0;
      app_.markDirty();
      return;
    }
    stopPlayback(StopCause::Auto);
  }
}

void AudioPlayer::serviceRinging() {
  if (!ringing_) return;
  if (millis() - ringStartMs_ > alarmcfg::RING_TIMEOUT_MS) {
    app_.setStatus("budzik: timeout");
    stopPlayback(StopCause::Auto);
    ringing_ = false; ringingAlarm_ = -1;
    app_.markDirty();
    return;
  }
  if (playing_) return;
  if (millis() - ringLastRetryMs_ < alarmcfg::RING_RETRY_MS) return;
  ringLastRetryMs_ = millis();
  if (ringingAlarm_ >= 0) {
    const AlarmConfig &alarm = app_.alarms().config(ringingAlarm_);
    if (startSound(alarm.sound, alarm.volume)) return;
  }
  if (++ringErrors_ >= alarmcfg::RING_MAX_ERRORS) {
    app_.setStatus("budzik: blad MP3"); ringing_ = false; ringingAlarm_ = -1; app_.markDirty();
  }
}

// ============================================================================
//  IMPLEMENTACJE — AlarmScheduler (model + harmonogram wyzwolen)
// ============================================================================
void AlarmScheduler::setDefaults() {
  for (int i = 0; i < alarmcfg::ALARM_COUNT; i++) {
    alarms_[i].enabled = 0;
    alarms_[i].hour = 7; alarms_[i].minute = 0; alarms_[i].second = 0;
    alarms_[i].volume = alarmcfg::DEFAULT_VOLUME;
    alarms_[i].sound = (uint8_t)i;
    snprintf(alarms_[i].label, sizeof(alarms_[i].label), "Budzik %d", i + 1);
  }
}

void AlarmScheduler::resetTriggerState() {
  for (int i = 0; i < alarmcfg::ALARM_COUNT; i++) g_persist.lastTriggerDay[i] = -1;
}

// Napraw wpis po odczycie z NVS/formularza: kazde pole sprowadzone do zakresu.
void AlarmScheduler::sanitize(int index) {
  AlarmConfig &alarm = alarms_[index];
  alarm.enabled = alarm.enabled ? 1 : 0;
  if (alarm.hour > 23) alarm.hour = 7;
  if (alarm.minute > 59) alarm.minute = 0;
  if (alarm.second > 59) alarm.second = 0;
  if (alarm.volume > alarmcfg::MAX_VOLUME) alarm.volume = alarmcfg::DEFAULT_VOLUME;
  if (alarm.sound >= alarmcfg::SOUND_COUNT) alarm.sound = (uint8_t)(index % alarmcfg::SOUND_COUNT);
  alarm.label[sizeof(alarm.label) - 1] = '\0';
  if (alarm.label[0] == '\0')
    snprintf(alarm.label, sizeof(alarm.label), "Budzik %d", index + 1);
}

void AlarmScheduler::sanitizeAll() {
  for (int i = 0; i < alarmcfg::ALARM_COUNT; i++) sanitize(i);
}

bool AlarmScheduler::anyEnabled() const {
  for (const AlarmConfig &a : alarms_) if (a.enabled) return true;
  return false;
}

// Najblizszy wlaczony budzik (dzis/jutro). Ranking po realnym epoch (mktime).
// Bez waznego czasu zwraca pierwszy wlaczony (delta = indeks).
NextAlarmInfo AlarmScheduler::nextAlarm() const {
  NextAlarmInfo info;
  info.index = -1;
  info.hour = info.minute = info.second = 0;
  info.label[0] = '\0';

  struct tm t;
  bool hasTime = getLocalTm(&t);
  time_t now = hasTime ? time(nullptr) : 0;
  long bestDelta = LONG_MAX;
  for (int i = 0; i < alarmcfg::ALARM_COUNT; i++) {
    const AlarmConfig &a = alarms_[i];
    if (!a.enabled) continue;
    long delta;
    if (hasTime) delta = (long)(nextOccurrence(t, a.hour, a.minute, a.second, now) - now);
    else delta = i;
    if (delta < bestDelta) {
      bestDelta = delta;
      info.index = i;
      info.hour = a.hour; info.minute = a.minute; info.second = a.second;
      strncpy(info.label, a.label, sizeof(info.label) - 1);
      info.label[sizeof(info.label) - 1] = '\0';
    }
  }
  return info;
}

// Budzik wymagalny PODCZAS dzwonienia innego nie moze przepasc (fix v8.0):
// oznacz go jako wyzwolony (dedup) i zapamietaj — zadzwoni zaraz po zakonczeniu
// biezacego dzwonienia. Deep-sleep nie grozi utrata kolejki: dzwonienie trzyma
// App::busy(), wiec zaden sen nie przerywa tego stanu.
void AlarmScheduler::queueWhileRinging(const struct tm &t, time_t now, uint32_t lateWindowSec) {
  if (pendingAlarm_ >= 0) return;
  for (int i = 0; i < alarmcfg::ALARM_COUNT; i++) {
    if (!alarms_[i].enabled) continue;
    int occurrenceDay;
    if (alarmDueNow(t, now, alarms_[i].hour, alarms_[i].minute, alarms_[i].second,
                    lateWindowSec, &occurrenceDay, nullptr) &&
        g_persist.lastTriggerDay[i] != occurrenceDay) {
      g_persist.lastTriggerDay[i] = occurrenceDay;
      pendingAlarm_ = i;
      return;
    }
  }
}

bool AlarmScheduler::firePendingIfAny() {
  if (pendingAlarm_ < 0) return false;
  int index = pendingAlarm_;
  pendingAlarm_ = -1;
  if (!alarms_[index].enabled) return false;   // wylaczony w miedzyczasie przez WWW
  app_.audio().startRinging(index);
  return true;
}

void AlarmScheduler::checkAlarms(uint32_t lateWindowSec) {
  time_t now = time(nullptr);
  struct tm t;
  if (!isSupportedEpoch(now)) return;
  localtime_r(&now, &t);

  if (app_.audioActive()) {              // dzwoni: nie startuj drugiego dzwonienia,
    queueWhileRinging(t, now, lateWindowSec);   // ale nie gub terminow
    return;
  }
  if (firePendingIfAny()) return;

  for (int i = 0; i < alarmcfg::ALARM_COUNT; i++) {
    if (!alarms_[i].enabled) continue;
    int occurrenceDay;
    time_t dueEpoch;
    if (alarmDueNow(t, now, alarms_[i].hour, alarms_[i].minute, alarms_[i].second,
                    lateWindowSec, &occurrenceDay, &dueEpoch) &&
        g_persist.lastTriggerDay[i] != occurrenceDay) {
      g_persist.lastTriggerDay[i] = occurrenceDay;
      // Skonsumuj budziki wymagalne w TEJ SAMEJ CHWILI (kolizja godzin w noc
      // zmiany czasu: 02:30 i 03:30 = ten sam epoch) -> jedno dzwonienie.
      // Porownujemy dokladne epoki wystapien, NIE "due w oknie" — szerokie okno
      // (90 s po korekcie NTP) pozeraloby budziki o roznych terminach (fix v8.0).
      for (int j = 0; j < alarmcfg::ALARM_COUNT; j++) {
        int otherDay;
        time_t otherEpoch;
        if (j != i && alarms_[j].enabled &&
            alarmDueNow(t, now, alarms_[j].hour, alarms_[j].minute, alarms_[j].second,
                        lateWindowSec, &otherDay, &otherEpoch) &&
            otherEpoch == dueEpoch)
          g_persist.lastTriggerDay[j] = otherDay;
      }
      app_.audio().startRinging(i);
      return;
    }
  }
}

bool AlarmScheduler::anyDueNow(const struct tm &t) const {
  time_t now = time(nullptr);
  for (const AlarmConfig &a : alarms_)
    if (a.enabled &&
        alarmDueNow(t, now, a.hour, a.minute, a.second, alarmcfg::LATE_WINDOW_SEC, nullptr, nullptr))
      return true;
  return false;
}

time_t AlarmScheduler::nextEpoch(const struct tm &base, time_t now) const {
  time_t best = 0;
  for (const AlarmConfig &a : alarms_) {
    if (!a.enabled) continue;
    time_t cand = nextOccurrence(base, a.hour, a.minute, a.second, now);
    if (best == 0 || cand < best) best = cand;
  }
  return best;
}

// ============================================================================
//  IMPLEMENTACJE — NetworkTimeService (WiFi + NTP)
//  Czas z RTC pierwszy; NTP NADPISUJE tylko po REALNYM pakiecie (gate atomic).
//  WiFi wlaczone tylko na czas proby.
// ============================================================================
// Jedna semantyka bezpiecznej kopii do bufora (obciecie + terminator).
void NetworkTimeService::copyBounded(char *dst, size_t dstSize, const char *src) {
  strncpy(dst, src ? src : "", dstSize - 1);
  dst[dstSize - 1] = '\0';
}
void NetworkTimeService::setCreds(const char *s, const char *p) {
  copyBounded(ssid_, sizeof(ssid_), s);
  copyBounded(pass_, sizeof(pass_), p);
}
void NetworkTimeService::setSsid(const String &s) { copyBounded(ssid_, sizeof(ssid_), s.c_str()); }
void NetworkTimeService::setPass(const String &p) { copyBounded(pass_, sizeof(pass_), p.c_str()); }

long NetworkTimeService::secsSinceLastSync() const {
  if (g_persist.lastNtpSyncEpoch <= 0) return -1;
  time_t now = time(nullptr);
  if (!isSupportedEpoch(now)) return -1;
  long d = (long)(now - (time_t)g_persist.lastNtpSyncEpoch);
  return d < 0 ? 0 : d;
}

void NetworkTimeService::stopSntpCallbacks() {
  g_ntpAcceptPacket.store(false, std::memory_order_release);
  if (esp_sntp_enabled()) esp_sntp_stop();
  sntp_set_time_sync_notification_cb(nullptr);
}

// Rollback po nieudanym NTP: przywroc czas sprzed proby — TYLKO gdy pakiet
// faktycznie zmodyfikowal zegar (g_ntpGotPacket). Nieudana proba bez pakietu
// (timeout, zerwane WiFi) nie ruszyla czasu systemowego, a bezwarunkowy rollback
// cofalby zegar o czas trwania proby, do ~50 s na kazde niepowodzenie (fix v8.0).
void NetworkTimeService::restoreTimeAfterFailedNtp() {
  if (!g_ntpGotPacket.load(std::memory_order_acquire)) return;
  if (isSupportedEpoch(fallbackTime_.tv_sec)) settimeofday(&fallbackTime_, nullptr);
  else setSystemEpoch(0);
}

void NetworkTimeService::stopWifiAndNtp() {
  bool wasAttempting = state_ != NtpState::Idle;
  if (state_ == NtpState::WaitingForPacket) restoreTimeAfterFailedNtp();
  stopSntpCallbacks();
  state_ = NtpState::Idle;
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
  wifiPowerOff();
}

bool NetworkTimeService::wifiBeginSmart() {
  WiFi.persistent(false);
#if FAST_NTP_RECONNECT
  if (g_persist.wifiHint) {
    WiFi.begin(ssid_, pass_, g_persist.wifiChannel, g_persist.wifiBssid);
    return true;
  }
#endif
  WiFi.begin(ssid_, pass_);
  return false;
}

void NetworkTimeService::wifiRetryFullScan() {
#if FAST_NTP_RECONNECT
  g_persist.wifiHint = false;
#endif
  WiFi.disconnect(false, false);
  WiFi.begin(ssid_, pass_);
}

void NetworkTimeService::wifiCacheHint() {
#if FAST_NTP_RECONNECT
  if (WiFi.status() == WL_CONNECTED) {
    g_persist.wifiChannel = WiFi.channel();
    const uint8_t *b = WiFi.BSSID();
    if (b) { for (int i = 0; i < 6; i++) g_persist.wifiBssid[i] = b[i]; g_persist.wifiHint = true; }
  }
#endif
}

// Blokujace laczenie WiFi (tylko sciezka panelu WWW; NTP laczy sie nieblokujaco).
bool NetworkTimeService::ensureWifi(uint32_t timeoutMs) {
  if (strlen(ssid_) == 0) return false;
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(true);                 // modem-sleep -> mniejszy prad STA
  if (WiFi.status() != WL_CONNECTED) {
    bool usedHint = wifiBeginSmart();
    bool retriedFullScan = false;
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
      if (usedHint && !retriedFullScan && millis() - start >= ntpcfg::WIFI_HINT_FALLBACK_MS) {
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

void NetworkTimeService::scheduleRtcWriteRetry(uint8_t retries) {
  rtcWriteRetriesLeft_ = retries;
  rtcWriteNextRetryMs_ = millis() + rtccfg::WRITE_RETRY_MS;
}

void NetworkTimeService::handleRtcWriteRetry() {
  if (rtcWriteRetriesLeft_ == 0 || (int32_t)(millis() - rtcWriteNextRetryMs_) < 0) return;
  if (app_.rtc().writeTime(time(nullptr), app_.audioActive())) {
    rtcWriteRetriesLeft_ = 0;
    app_.setNtpStatus("NTP OK, RTC OK");
  } else if (--rtcWriteRetriesLeft_ > 0) {
    rtcWriteNextRetryMs_ = millis() + rtccfg::WRITE_RETRY_MS;
    app_.setNtpStatus("NTP OK, RTC retry");
  } else {
    app_.setNtpStatus("NTP OK, RTC ERR");
  }
  app_.markDirty();
}

void NetworkTimeService::finishNtpAttempt(bool ok) {
  bool wasManual = manualAttempt_;
  state_ = NtpState::Idle;
  manualAttempt_ = false;
  wifiHintActive_ = false;
  wifiLostMs_ = 0;
  stopSntpCallbacks();
  g_ntpGotPacket.store(false, std::memory_order_release);

  if (ok) {
    retriesLeft_ = 0;
    backoffMs_ = ntpcfg::BACKOFF_MIN_MS;              // reset backoffu po sukcesie
  } else if (!wasManual && retriesLeft_ > 0) {
    nextRetryMs_ = millis() + ntpcfg::QUICK_RETRY_MS; // dokoncz serie szybka
  } else if (!isSupportedEpoch(time(nullptr))) {
    // Brak waznego czasu -> ponawiaj z BACKOFFEM (1 min -> 1 h). Dotyczy takze
    // proby RECZNEJ (manual ustawia retriesLeft_=0): zegar bez czasu nie moze
    // zostac bez zaplanowanego ponowienia tylko dlatego, ze klikniecie zawiodlo.
    retriesLeft_ = 1;
    nextRetryMs_ = millis() + backoffMs_;
    backoffMs_ = (backoffMs_ >= ntpcfg::BACKOFF_MAX_MS / 2) ? ntpcfg::BACKOFF_MAX_MS : backoffMs_ * 2;
  }
  if (!app_.web().isActive()) wifiPowerOff();
  app_.markDirty();
}

// Wspolna sciezka porazki w WaitingForPacket: NAJPIERW odetnij SNTP (spozniony
// pakiet nie moze nadpisac przywracanego czasu), potem rollback i zamkniecie proby.
void NetworkTimeService::failNtpAttempt(const char *status) {
  stopSntpCallbacks();
  restoreTimeAfterFailedNtp();
  app_.setNtpStatus(status);
  finishNtpAttempt(false);
}

void NetworkTimeService::beginNtpRequest() {
  g_ntpAcceptPacket.store(false, std::memory_order_release);
  g_ntpGotPacket.store(false, std::memory_order_release);
  sntp_set_time_sync_notification_cb(onSntpSync);   // PRZED configTzTime
  sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED);
  gettimeofday(&fallbackTime_, nullptr);
  g_ntpAcceptPacket.store(true, std::memory_order_release);
  configTzTime(ntpcfg::TZ_POLAND, ntpcfg::SERVER_1, ntpcfg::SERVER_2, ntpcfg::SERVER_3);
  state_ = NtpState::WaitingForPacket;
  wifiLostMs_ = 0;                     // swiezy grace 8 s dla tej proby
  stateStartedMs_ = millis();
  app_.setNtpStatus("oczekiwanie na NTP");
  app_.markDirty();
}

void NetworkTimeService::syncNow(SyncTrigger trigger) {
  if (state_ != NtpState::Idle) return;
  bool manual = (trigger == SyncTrigger::Manual);
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
    state_ = NtpState::WifiConnecting;
    stateStartedMs_ = millis();
  }
}

// Wolane co petle. Zwraca true gdy NTP wlasnie zatwierdzil nowy czas (dla okna budzika).
bool NetworkTimeService::service() {
  if (state_ == NtpState::WifiConnecting) {
    if (WiFi.status() == WL_CONNECTED) {
      wifiCacheHint();
      beginNtpRequest();
    } else if (wifiHintActive_ && millis() - stateStartedMs_ >= ntpcfg::WIFI_HINT_FALLBACK_MS) {
      wifiHintActive_ = false;
      app_.setNtpStatus("WiFi: pelny skan");
      wifiRetryFullScan();
      stateStartedMs_ = millis();
      app_.markDirty();
    } else if (millis() - stateStartedMs_ >= ntpcfg::WIFI_TIMEOUT_MS) {
      app_.setNtpStatus("blad WiFi");
      g_persist.wifiHint = false;
      finishNtpAttempt(false);
    }
    return false;
  }

  if (state_ != NtpState::WaitingForPacket) return false;

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
                                               // inaczej writeTime nie wyrowna fazy
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

    bool rtcOk = app_.rtc().writeTime(syncedEpoch, app_.audioActive());
    if (rtcOk) rtcWriteRetriesLeft_ = 0; else scheduleRtcWriteRetry();
    char buf[48];
    snprintf(buf, sizeof(buf), "OK %02d:%02d:%02d RTC:%s",
             local.tm_hour, local.tm_min, local.tm_sec, rtcOk ? "OK" : "RETRY");
    app_.setNtpStatus(buf);
    // Znacznik dobowego synchro 18:00 zalicza WYLACZNIE sync w oknie wieczornym
    // (>=18:00, przed noca). Poranne 06:00 ma wlasny znacznik ntpMorningDay —
    // wspoldzielenie znacznika blokowaloby wieczorna synchronizacje (fix z v6.x).
    if (local.tm_hour >= ntpcfg::DAILY_SYNC_HOUR && local.tm_hour < powercfg::NIGHT_START_HOUR)
      g_persist.ntpStartedDay = localDayKey(local);
    // Udany sync w oknie porannym konsumuje ntpMorningDay: sync odpalony z setup
    // (zimny start / pusty RTC o 06:0x) nie zostanie zdublowany przez checkSchedule;
    // nieudany NIE konsumuje — checkSchedule ma prawo sprobowac ponownie.
    if (local.tm_hour == powercfg::NIGHT_END_HOUR && local.tm_min < ntpcfg::SYNC_MINUTE_WINDOW)
      g_persist.ntpMorningDay = localDayKey(local);
    finishNtpAttempt(true);
    return true;
  }

  // Zerwane WiFi w trakcie oczekiwania: pakiet juz nie nadejdzie. Krotki grace
  // (auto-reconnect czesto wraca po chwili), potem szybkie zakonczenie proby
  // zamiast palenia radia do pelnego PACKET_TIMEOUT_MS (50 s).
  if (WiFi.status() != WL_CONNECTED) {
    if (wifiLostMs_ == 0) {
      wifiLostMs_ = millis();
    } else if (millis() - wifiLostMs_ >= ntpcfg::WIFI_LOST_GRACE_MS) {
      failNtpAttempt("blad NTP (WiFi zerwane)");
      return false;
    }
  } else {
    wifiLostMs_ = 0;
  }

  if (millis() - stateStartedMs_ >= ntpcfg::PACKET_TIMEOUT_MS) {
    failNtpAttempt("blad NTP");
  }
  return false;
}

bool NetworkTimeService::handleNtpRetry() {
  if (state_ != NtpState::Idle) return true;
  if (retriesLeft_ == 0) return false;
  if ((int32_t)(millis() - nextRetryMs_) < 0) return true;
  --retriesLeft_;
  syncNow(SyncTrigger::Scheduled);
  return true;
}

#if ENABLE_RTC_AUTOTRIM
// Autokalibracja offsetu: po kazdym pakiecie NTP zmierz dryft fazowo i — gdy
// okno NTP->NTP jest wiarygodne — przesun rejestr 0x02 tak, by wyzerowac ppm.
// Nowy offset trafia do NVS (przezywa utrate zasilania) i dziala od razu.
// Kwantyzacja 4.34 ppm/LSB zostawia dryf resztkowy do ~0.19 s/dobe.
void NetworkTimeService::maybeAutotrimRtc(time_t syncedEpoch) {
  if (app_.audioActive()) return;                 // pomiar blokuje ~1.3 s -> nie przy audio
  if (!g_persist.rtcPhaseAligned) return;         // faza startu okna nieznana -> ppm bez sensu
  if (g_persist.lastNtpSyncEpoch <= 0) return;
  int64_t windowSec = (int64_t)syncedEpoch - g_persist.lastNtpSyncEpoch;
  if (windowSec < rtccfg::AUTOTRIM_MIN_WINDOW_SEC || windowSec > rtccfg::AUTOTRIM_MAX_WINDOW_SEC) return;

  int64_t driftUs;
  if (!app_.rtc().measureDriftUs(&driftUs)) return;
  float ppm = (float)driftUs / (float)windowSec;  // us/s = ppm
  g_persist.lastDriftPpm = ppm;
  g_persist.lastDriftWindowSec = (int32_t)windowSec;
  if (fabsf(ppm) > rtccfg::AUTOTRIM_MAX_PLAUSIBLE_PPM) return;   // krysztal tak nie plywa -> zly pomiar

  int target = suggestedTotalOffsetLsb(ppm * rtccfg::PPM_SEC_PER_DAY, app_.rtc().offsetLsb());
  if (target == (int)app_.rtc().offsetLsb()) return;             // korekta ponizej 1 LSB
  if (app_.rtc().applyOffsetLsb((int8_t)target)) {
    g_persist.autotrimAdjustCount++;
    app_.settings().saveRtcOffset((int8_t)target);
    Serial.printf("RTC autotrim: %.2f ppm w %lld s -> offset LSB %d\n",
                  ppm, (long long)windowSec, target);
  }
}
#endif

void NetworkTimeService::checkSchedule() {
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
  bool preferredWindow = (t.tm_hour == ntpcfg::DAILY_SYNC_HOUR && t.tm_min <= ntpcfg::SYNC_MINUTE_WINDOW);
  bool catchupWindow   = (t.tm_hour >= ntpcfg::DAILY_SYNC_HOUR && t.tm_hour < powercfg::NIGHT_START_HOUR);
  if ((preferredWindow || catchupWindow) && g_persist.ntpStartedDay != day) {
    g_persist.ntpStartedDay = day;
    retriesLeft_ = 2;
    syncNow(SyncTrigger::Scheduled);
    return;
  }
  // Poranne NTP 06:00-06:05 (wlasny znacznik ntpMorningDay — NIE konsumuje
  // straznika okna wieczornego). W v7.0 to okno zylo tylko w setup(): urzadzenie
  // czuwajace przez 06:00 (dzwoniacy budzik, otwarte WWW) gubilo poranny sync na
  // caly dzien (fix v8.0). Po wybudzeniu z nocy pierwszy obieg loop() trafia tu
  // natychmiast, wiec timing wzgledem v7.0 bez zmian.
  if (t.tm_hour == powercfg::NIGHT_END_HOUR && t.tm_min < ntpcfg::SYNC_MINUTE_WINDOW &&
      g_persist.ntpMorningDay != day) {
    g_persist.ntpMorningDay = day;
    retriesLeft_ = 2;
    syncNow(SyncTrigger::Scheduled);
  }
}

// ============================================================================
//  IMPLEMENTACJE — DisplayUi (U8g2, panel ST7305)
// ============================================================================
// Ten egzemplarz ST7305 NIE przywraca obrazu po deep-sleep przez sam initInterface()
// -> pelny re-init (reset RST + begin) przy KAZDYM wejsciu w setup (zimny start
// ORAZ po deep-sleep). W dzien petla uzywa light-sleep (setup() sie nie powtarza)
// -> panel trzyma obraz -> brak mrugania. Tryb LPM panelu jest nieprzetestowany
// i celowo NIE jest wysylany.
void DisplayUi::setup() {
  constexpr uint32_t RESET_PULSE_MS = 10;
  pinMode(pins::LCD_RST, OUTPUT);
  digitalWrite(pins::LCD_RST, HIGH);
  digitalWrite(pins::LCD_RST, LOW);
  delay(RESET_PULSE_MS);
  digitalWrite(pins::LCD_RST, HIGH);
  delay(RESET_PULSE_MS);
  // HW SPI: usuwa ~4 s opoznienia pelnego sendBuffer().
  SPI.begin(pins::LCD_SCK, -1, pins::LCD_MOSI, pins::LCD_CS);
  panel_.setBusClock(lcd::SPI_BUS_HZ);
  panel_.begin();
  panel_.setContrast(lcd::CONTRAST);
}

void DisplayUi::drawPhotoGlyph(int x, int y, int glyph, bool colon) {
  uint16_t p = colon ? PHOTO_COLON_OFFSET : pgm_read_word(&PHOTO_DIGIT_OFFSET[glyph]);
  for (int row = 0; row < PHOTO_DIGIT_H; row++) {
    uint8_t segs = pgm_read_byte(&PHOTO_FONT_RLE[p++]);
    for (uint8_t i = 0; i < segs; i++) {
      uint8_t start = pgm_read_byte(&PHOTO_FONT_RLE[p++]);
      uint8_t width = pgm_read_byte(&PHOTO_FONT_RLE[p++]);
      panel_.drawHLine(x + start, y + row, width);
    }
  }
  // Font (Impact) ma skrocona dolna stopke cyfry 4 — uzupelniamy ja tutaj.
  // Wspolrzedne sprzezone z PHOTO_DIGIT_W/H; zmiana fontu = rewizja tych prostokatow.
  if (!colon && glyph == 4) {
    constexpr int DIGIT4_PATCH1_X = 39, DIGIT4_PATCH1_Y = 112, DIGIT4_PATCH1_W = 41, DIGIT4_PATCH1_H = 13;
    constexpr int DIGIT4_PATCH2_X = 40, DIGIT4_PATCH2_Y = 125, DIGIT4_PATCH2_W = 31, DIGIT4_PATCH2_H = 25;
    panel_.drawBox(x + DIGIT4_PATCH1_X, y + DIGIT4_PATCH1_Y, DIGIT4_PATCH1_W, DIGIT4_PATCH1_H);
    panel_.drawBox(x + DIGIT4_PATCH2_X, y + DIGIT4_PATCH2_Y, DIGIT4_PATCH2_W, DIGIT4_PATCH2_H);
  }
}

void DisplayUi::drawLargeTime(uint8_t hour, uint8_t minute, int y) {
  int totalW = PHOTO_DIGIT_W * 4 + PHOTO_COLON_W + PHOTO_DIGIT_GAP * 4;
  int x = (lcd::WIDTH - totalW) / 2 + PHOTO_TIME_X_SHIFT;
  drawPhotoGlyph(x, y, hour / 10, false);   x += PHOTO_DIGIT_W + PHOTO_DIGIT_GAP;
  drawPhotoGlyph(x, y, hour % 10, false);   x += PHOTO_DIGIT_W + PHOTO_DIGIT_GAP;
  drawPhotoGlyph(x + PHOTO_COLON_X_SHIFT, y + PHOTO_COLON_Y_SHIFT, 0, true);
  x += PHOTO_COLON_W + PHOTO_DIGIT_GAP;
  drawPhotoGlyph(x, y, minute / 10, false); x += PHOTO_DIGIT_W + PHOTO_DIGIT_GAP;
  drawPhotoGlyph(x, y, minute % 10, false);
}

void DisplayUi::drawBellIcon(int x, int y, bool active) {
  panel_.setDrawColor(1);
  panel_.drawDisc(x + 12, y + 10, 8);
  panel_.drawBox(x + 5, y + 10, 15, 11);
  panel_.drawBox(x + 3, y + 20, 19, 4);
  panel_.drawDisc(x + 12, y + 25, 3);
  panel_.drawBox(x + 9, y + 3, 7, 4);
  if (!active) {                                   // przekreslenie: budziki wylaczone
    panel_.drawLine(x + 1, y + 26, x + 25, y + 2);
    panel_.drawLine(x + 2, y + 26, x + 26, y + 2);
  }
}

void DisplayUi::drawBatteryIcon(int x, int y, uint8_t percent) {
  if (percent > 100) percent = 100;
  panel_.drawFrame(x, y, 28, 13);                  // korpus 28x13 + nosek
  panel_.drawBox(x + 28, y + 4, 3, 5);
  int fill = map(percent, 0, 100, 0, 24);
  if (fill > 0) panel_.drawBox(x + 2, y + 2, fill, 9);
  char txt[8];
  snprintf(txt, sizeof(txt), "%u%%", percent);
  panel_.setFont(u8g2_font_7x14_tf);
  panel_.drawStr(x - 42, y + 12, txt);             // etykieta % na lewo od ikony
}

void DisplayUi::drawCentered(const char *text, int y) {
  int w = panel_.getStrWidth(text);
  int x = (lcd::WIDTH - w) / 2;
  if (x < 0) x = 0;
  panel_.drawStr(x, y, text);
}

UiPage DisplayUi::resolvePage() const {
  if (app_.audioActive()) return UiPage::Ringing;              // stan nadrzedny: dzwoni budzik
  if (app_.buttons().previewActive()) return UiPage::Preview;  // wygasniecie rozstrzyga update()
  struct tm t;
  if (!getLocalTm(&t)) return UiPage::NoTime;
  return UiPage::Clock;
}

ClockView DisplayUi::makeClockView(const struct tm &t, int dayKey) const {
  NextAlarmInfo next = app_.alarms().nextAlarm();
  ClockView v;
  v.hour            = (uint8_t)t.tm_hour;
  v.minute          = (uint8_t)t.tm_min;
  v.dayKey          = dayKey;
  v.batteryPercent  = app_.battery().percent();
  v.webActive       = app_.web().isActive();
  v.anyAlarmEnabled = app_.alarms().anyEnabled();
  v.hasNextAlarm    = next.valid();
  v.nextAlarmHour   = next.hour;
  v.nextAlarmMinute = next.minute;
  return v;
}

void DisplayUi::drawClockPage(const ClockView &view) {
  panel_.clearBuffer();
  panel_.setDrawColor(1);
  drawBatteryIcon(BATTERY_ICON_X, BATTERY_ICON_Y, view.batteryPercent);

  int y = (lcd::HEIGHT - PHOTO_DIGIT_H) / 2 + PHOTO_TIME_Y_SHIFT;
  drawLargeTime(view.hour, view.minute, y);

  panel_.drawHLine(0, FOOTER_LINE_Y, lcd::WIDTH);
  drawBellIcon(FOOTER_BELL_X, FOOTER_BELL_Y, view.anyAlarmEnabled);
  panel_.setFont(u8g2_font_7x14_tf);
  char bottom[80];
  if (view.hasNextAlarm)
    snprintf(bottom, sizeof(bottom), "Budzik %02u:%02u %s  Aku %u%%  WWW:%s",
             view.nextAlarmHour, view.nextAlarmMinute, view.anyAlarmEnabled ? "ON" : "OFF",
             view.batteryPercent, view.webActive ? "ON" : "OFF");
  else
    snprintf(bottom, sizeof(bottom), "Budzik --:-- OFF  Aku %u%%  WWW:%s",
             view.batteryPercent, view.webActive ? "ON" : "OFF");
  panel_.drawStr(FOOTER_TEXT_X, FOOTER_TEXT_Y, bottom);
  panel_.sendBuffer();
}

void DisplayUi::drawPreviewPage() {
  NextAlarmInfo next = app_.alarms().nextAlarm();

  panel_.clearBuffer();
  panel_.setDrawColor(1);
  drawBatteryIcon(BATTERY_ICON_X, BATTERY_ICON_Y, app_.battery().percent());
  if (next.valid()) {
    drawBellIcon(FOOTER_BELL_X, 8, true);
    panel_.setFont(u8g2_font_9x15_tf);
    drawCentered("NAJBLIZSZY BUDZIK", 25);
    drawLargeTime(next.hour, next.minute, 60);
    panel_.drawHLine(0, FOOTER_LINE_Y, lcd::WIDTH);
    char detail[96];
    snprintf(detail, sizeof(detail), "%s  %02u:%02u:%02u  [%s]",
             next.label, next.hour, next.minute, next.second,
             alarmcfg::SOUND_NAMES[app_.alarms().config(next.index).sound % alarmcfg::SOUND_COUNT]);
    drawCentered(detail, FOOTER_TEXT_Y);
  } else {
    drawBellIcon(185, 35, false);
    panel_.setFont(u8g2_font_logisoso50_tf);
    drawCentered("--:--", 150);
    panel_.setFont(u8g2_font_9x15_tf);
    drawCentered("Brak aktywnego budzika", 215);
  }
  panel_.sendBuffer();
  lastClockSig_ = FORCE_REPAINT_SIG;   // powrot do zegara wymusi repaint
}

void DisplayUi::drawRingingPage() {
  panel_.clearBuffer();
  panel_.setDrawColor(1);
  drawBatteryIcon(BATTERY_ICON_X, BATTERY_ICON_Y, app_.battery().percent());
  drawBellIcon(185, 35, true);
  panel_.setFont(u8g2_font_logisoso50_tf);
  drawCentered("BUDZIK", 150);
  panel_.setFont(u8g2_font_9x15_tf);
  drawCentered(app_.statusText().c_str(), 210);
  drawCentered("Przycisk = wycisz", 245);
  panel_.sendBuffer();
  lastClockSig_ = FORCE_REPAINT_SIG;
}

void DisplayUi::drawNightPage() {
  panel_.clearBuffer();
  panel_.setDrawColor(1);
  panel_.setFont(u8g2_font_7x14_tf);
  char text[40];
  snprintf(text, sizeof(text), "TRYB NOCNY %02d:%02d - %02d:%02d",
           powercfg::NIGHT_START_HOUR, powercfg::NIGHT_START_MIN,
           powercfg::NIGHT_END_HOUR, powercfg::NIGHT_END_MIN);
  drawCentered(text, 150);
  panel_.sendBuffer();
  app_.clearDirty();
  lastClockSig_ = FORCE_REPAINT_SIG;
}

#if ENABLE_LOW_BATTERY_PROTECT
void DisplayUi::drawCriticalBatteryPage() {
  panel_.clearBuffer();
  panel_.setDrawColor(1);
  drawBatteryIcon(BATTERY_ICON_X, BATTERY_ICON_Y, app_.battery().percent());
  panel_.setFont(u8g2_font_logisoso32_tf);
  drawCentered("ROZLADOWANY", 120);
  panel_.setFont(u8g2_font_9x15_tf);
  drawCentered("Podlacz ladowarke", 165);
  char b[40];
  snprintf(b, sizeof(b), "%u%%  %.2f V", app_.battery().percent(), app_.battery().voltage());
  drawCentered(b, 200);
  panel_.sendBuffer();
  lastClockSig_ = FORCE_REPAINT_SIG;
}
#endif

void DisplayUi::drawNoTimePage() {
  panel_.clearBuffer();
  panel_.setDrawColor(1);
  panel_.setFont(u8g2_font_9x15_tf);
  drawCentered("Brak czasu RTC/NTP", 135);
  drawCentered(app_.ntpStatus().c_str(), 170);
  panel_.sendBuffer();
  lastClockSig_ = FORCE_REPAINT_SIG;
}

// Strony NoTime/Clock — wspolne zakonczenie update() oraz wygasniecia podgladu.
void DisplayUi::renderTimePages() {
  struct tm t;
  if (!getLocalTm(&t)) {
    if (app_.dirty()) {
      drawNoTimePage();
      app_.clearDirty();
    }
    return;
  }
  // Strona zegara — repaint TYLKO gdy zmienia sie WIDOCZNA tresc (sygnatura widoku).
  int dayKey = localDayKey(t);
  ClockView view = makeClockView(t, dayKey);
  uint32_t sig = view.signature();
  if (sig != lastClockSig_) {
    drawClockPage(view);
    lastClockSig_ = sig;
  }
  app_.clearDirty();
}

void DisplayUi::update() {
  switch (resolvePage()) {
    case UiPage::Ringing:
      if (app_.dirty()) { drawRingingPage(); app_.clearDirty(); }
      return;

    case UiPage::Preview:
      if (app_.buttons().previewWithinWindow()) {
        if (app_.dirty()) { drawPreviewPage(); app_.clearDirty(); }
        return;
      }
      app_.buttons().clearPreview();   // okno podgladu uplynelo -> wroc na strony czasu
      app_.markDirty();
      break;

    case UiPage::NoTime:
    case UiPage::Clock:
      break;
  }
  renderTimePages();
}

// ============================================================================
//  IMPLEMENTACJE — WebPanel (panel konfiguracyjny WWW)
// ============================================================================
static bool removeIfExists(fs::FS &fs, const String &path) {
  return fs.exists(path) && fs.remove(path);
}

void WebPanel::redirectHome() {
  server_.sendHeader("Location", "/");
  server_.send(303, "text/plain", "");
}

void WebPanel::appendMp3Manager(String &html) {
  html += F("<section><b>Pliki MP3</b>");
  if (!app_.soundStore().ensureMounted()) { html += F("<p>Brak zamontowanego systemu plikow.</p></section>"); return; }
  html += F("<p class='muted'>System plikow: "); html += htmlEscape(app_.soundStore().name());
  html += F("</p><form method='post' action='/mp3upload' enctype='multipart/form-data'>"
            "<div class='row'><input type='file' name='mp3' accept='.mp3,audio/mpeg' required>"
            "<button type='submit'>Dodaj / nadpisz MP3</button></div></form>");
  html += F("<table><tr><th>Plik</th><th>Rozmiar</th><th></th></tr>");
  bool any = false;
  File root = app_.soundStore().fs()->open("/");
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

void WebPanel::pageHeader(String &h) {
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

void WebPanel::handleRoot() {
  extendWindow();
  app_.battery().update(false);   // bez wymuszania: pod WiFi pomiar zanizony, OCV po zamknieciu WWW
  String html; html.reserve(webcfg::HTML_ROOT_RESERVE);
  pageHeader(html);

  html += F("<section><b>Status</b><p>Czas: "); html += currentTimeText();
  html += F("<br>Budzik: ");     html += htmlEscape(app_.statusText());
  html += F("<br>NTP: ");        html += htmlEscape(app_.ntpStatus());
  html += F("<br>Pliki MP3: ");  html += htmlEscape(app_.soundStore().name());
  html += F("<br>Akumulator: "); html += app_.battery().percent();
  html += F("% (");              html += String(app_.battery().voltage(), 2);
  html += F(" V, ok. ");         html += String((uint32_t)app_.battery().percent() * batcfg::CAPACITY_MAH / 100U);
  html += F(" mAh)<br>IP: ");    html += WiFi.localIP().toString();
  {
    float perDay = app_.persist().drain.mahPerDay();
    float daysLeft = app_.persist().drain.daysRemaining(app_.battery().percent());
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
  for (int i = 0; i < alarmcfg::ALARM_COUNT; i++) {
    AlarmConfig &a = app_.alarms().config(i);
    char timeBuf[16];
    snprintf(timeBuf, sizeof(timeBuf), "%02u:%02u:%02u", a.hour, a.minute, a.second);
    html += F("<tr><td><input type='checkbox' name='en"); html += i; html += F("' ");
    if (a.enabled) html += F("checked");
    html += F("></td><td><input type='time' step='1' name='time"); html += i;
    html += F("' value='"); html += timeBuf;
    html += F("'></td><td><input type='number' min='0' max='21' name='vol"); html += i;
    html += F("' value='"); html += a.volume;
    html += F("'></td><td><select name='snd"); html += i; html += F("'>");
    for (int sIdx = 0; sIdx < alarmcfg::SOUND_COUNT; sIdx++) {
      html += F("<option value='"); html += sIdx; html += F("'");
      if (a.sound == sIdx) html += F(" selected");
      html += F(">"); html += alarmcfg::SOUND_NAMES[sIdx]; html += F("</option>");
    }
    html += F("</select></td><td><input maxlength='15' name='label"); html += i;
    html += F("' value='"); html += htmlEscape(a.label);
    html += F("'></td></tr>");
  }
  html += F("</table><p class='muted'>Dzwiek 1/2/3 = /alarm1.mp3 /alarm2.mp3 /alarm3.mp3.</p></section>");
  html += F("<p><button type='submit'>Zapisz</button></p></form>");

  appendMp3Manager(html);

  html += F("<section><b>Test dzwiekow</b><div class='row'>");
  for (int sIdx = 0; sIdx < alarmcfg::SOUND_COUNT; sIdx++) {
    html += F("<form method='post' action='/test' style='display:inline'>"
              "<input type='hidden' name='snd' value='"); html += sIdx;
    html += F("'><button type='submit'>Test "); html += alarmcfg::SOUND_NAMES[sIdx]; html += F("</button></form>");
  }
  html += F("</div></section></main></body></html>");
  server_.send(200, "text/html", html);
}

void WebPanel::handleSave() {
  extendWindow();
  if (server_.hasArg("ssid")) { String ssid = server_.arg("ssid"); ssid.trim(); app_.ntp().setSsid(ssid); }
  if (server_.hasArg("pass") && server_.arg("pass").length() > 0) app_.ntp().setPass(server_.arg("pass"));

  for (int i = 0; i < alarmcfg::ALARM_COUNT; i++) {
    String idx = String(i);
    AlarmConfig &a = app_.alarms().config(i);
    uint8_t prevEnabled = a.enabled;
    uint8_t prevH = a.hour, prevM = a.minute, prevS = a.second;
    a.enabled = server_.hasArg(String("en") + idx) ? 1 : 0;
    uint8_t h, m, s;
    if (parseClock(server_.arg(String("time") + idx), &h, &m, &s)) { a.hour = h; a.minute = m; a.second = s; }
    bool timeChanged = (a.hour != prevH || a.minute != prevM || a.second != prevS);
    bool reEnabled   = (a.enabled && !prevEnabled);
    if (timeChanged || reEnabled) app_.persist().lastTriggerDay[i] = -1;   // ponowne uzbrojenie: moze odpalic dzis
    String volumeArg = String("vol") + idx, soundArg = String("snd") + idx, labelArg = String("label") + idx;
    if (server_.hasArg(volumeArg)) a.volume = constrain(server_.arg(volumeArg).toInt(), 0, alarmcfg::MAX_VOLUME);
    if (server_.hasArg(soundArg))  a.sound  = constrain(server_.arg(soundArg).toInt(), 0, alarmcfg::SOUND_COUNT - 1);
    if (server_.hasArg(labelArg)) {
      String label = server_.arg(labelArg); label.trim();
      if (label.length() == 0) label = "Budzik " + String(i + 1);
      label.toCharArray(a.label, sizeof(a.label));
    }
    app_.alarms().sanitize(i);
  }
  app_.settings().save();
  app_.markDirty();
  redirectHome();
}

void WebPanel::handleSync() { app_.ntp().syncNow(SyncTrigger::Manual); redirectHome(); }
void WebPanel::handleStop() { app_.audio().stopPlayback(StopCause::User); redirectHome(); }

void WebPanel::handleTest() {
  // Nie przerywaj dzwoniacego budzika testem (startSound nie kasuje dzwonienia).
  if (!app_.audio().isRinging()) {
    uint8_t sIdx = constrain(server_.arg("snd").toInt(), 0, alarmcfg::SOUND_COUNT - 1);
    app_.audio().startSound(sIdx, alarmcfg::DEFAULT_VOLUME);
  }
  redirectHome();
}

void WebPanel::handleMp3UploadDone() { redirectHome(); }

// Sprzatanie po nieudanym/przerwanym uploadzie: zamknij plik i usun .part.
void WebPanel::discardUploadTemp() {
  if (mp3UploadFile_) mp3UploadFile_.close();
  if (mp3UploadTempPath_.length() && app_.soundStore().fs())
    removeIfExists(*app_.soundStore().fs(), mp3UploadTempPath_);
}

// Upload MP3 jest ATOMOWY: dane plyna do <plik>.part, instalacja to sekwencja
// rename stary->.bak, rename .part->final, usun .bak — kazde przerwanie zostawia
// stan naprawialny przez SoundStore::recoverSoundFiles przy nastepnym montowaniu.
void WebPanel::handleMp3UploadStream() {
  extendWindow();
  watchdogFeed();   // callback per fragment: upload duzego MP3 blokuje loop() minutami
  HTTPUpload &upload = server_.upload();
  if (upload.status == UPLOAD_FILE_START) {
    if (mp3UploadFile_) mp3UploadFile_.close();
    mp3UploadAccepted_ = false; mp3UploadHadData_ = false; mp3UploadPath_ = ""; mp3UploadTempPath_ = "";
    if (!normalizeMp3Path(upload.filename, mp3UploadPath_, false)) { app_.setStatus("zla nazwa MP3"); app_.markDirty(); return; }
    if (!app_.soundStore().ensureMounted()) { app_.setStatus("brak FS dla MP3"); app_.markDirty(); return; }
    if (app_.audioActive()) app_.audio().stopPlayback(StopCause::User);
    fs::FS &fs = *app_.soundStore().fs();
    mp3UploadTempPath_ = mp3UploadPath_ + ".part";
    removeIfExists(fs, mp3UploadTempPath_);
    mp3UploadFile_ = fs.open(mp3UploadTempPath_, FILE_WRITE);
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
      fs::FS &fs = *app_.soundStore().fs();
      String backupPath = mp3UploadPath_ + ".bak";
      removeIfExists(fs, backupPath);
      bool hadOld = fs.exists(mp3UploadPath_);
      bool oldMoved = !hadOld || fs.rename(mp3UploadPath_, backupPath);
      bool newInstalled = oldMoved && fs.rename(mp3UploadTempPath_, mp3UploadPath_);
      if (newInstalled) {
        removeIfExists(fs, backupPath);
        app_.setStatus(String("dodano ") + mp3UploadPath_);
      } else {
        if (oldMoved && hadOld) {                      // rollback: przywroc stary plik
          removeIfExists(fs, mp3UploadPath_);
          if (fs.exists(backupPath)) fs.rename(backupPath, mp3UploadPath_);
        }
        removeIfExists(fs, mp3UploadTempPath_);
        app_.setStatus("blad instalacji MP3");
      }
    } else {
      discardUploadTemp();
      if (app_.statusText() == String("upload ") + mp3UploadPath_) app_.setStatus("pusty MP3");
    }
    app_.markDirty();
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    discardUploadTemp();
    app_.setStatus("upload przerwany");
    app_.markDirty();
  }
}

void WebPanel::handleMp3Delete() {
  String path;
  if (!normalizeMp3Path(server_.arg("file"), path, true)) app_.setStatus("zla nazwa MP3");
  else if (!app_.soundStore().ensureMounted()) app_.setStatus("brak FS dla MP3");
  else {
    if (app_.audioActive()) app_.audio().stopPlayback(StopCause::User);
    app_.setStatus(removeIfExists(*app_.soundStore().fs(), path)
                     ? String("usunieto ") + path : String("brak ") + path);
  }
  app_.markDirty();
  redirectHome();
}

#if ENABLE_DIAGNOSTICS
void WebPanel::handleInfo() {
  extendWindow();
  app_.battery().update(false);
  String html; html.reserve(webcfg::HTML_INFO_RESERVE);
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
    float driftPerDay = driftPpm * rtccfg::PPM_SEC_PER_DAY;
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
  row("System plikow", app_.soundStore().name());
  html += F("</table><p><a href='/'><button>Powrot</button></a></p></section></main></body></html>");
  server_.send(200, "text/html", html);
}
#endif

// Reczne wylaczenie panelu WWW: odsyla potwierdzenie, a samo zamkniecie wykonuje
// petla PO wyslaniu odpowiedzi (ustawiamy termin okna na "teraz").
void WebPanel::handleWebOff() {
  String html; pageHeader(html);
  html += F("<section><b>Panel WWW wylaczony.</b>"
            "<p class='muted'>WiFi zostalo wylaczone dla oszczednosci baterii. "
            "Aby wlaczyc panel ponownie: przytrzymaj przycisk KEY na zegarze.</p>"
            "</section></main></body></html>");
  server_.send(200, "text/html", html);
  untilMs_ = millis();   // petla wykryje uplyw okna i zamknie panel po flushu strony
}

void WebPanel::installRoutes() {
  server_.on("/", HTTP_GET, [this]{ handleRoot(); });
  server_.on("/save", HTTP_POST, [this]{ handleSave(); });
  server_.on("/sync", HTTP_GET, [this]{ handleSync(); });
  server_.on("/stop", HTTP_GET, [this]{ handleStop(); });
  server_.on("/test", HTTP_POST, [this]{ handleTest(); });
  server_.on("/mp3upload", HTTP_POST, [this]{ handleMp3UploadDone(); }, [this]{ handleMp3UploadStream(); });
  server_.on("/mp3delete", HTTP_POST, [this]{ handleMp3Delete(); });
  server_.on("/off", HTTP_GET, [this]{ handleWebOff(); });
#if ENABLE_DIAGNOSTICS
  server_.on("/info", HTTP_GET, [this]{ handleInfo(); });
#endif
}

bool WebPanel::startWebWindow() {
  if (active_) {
    if (WiFi.status() == WL_CONNECTED) { extendWindow(); app_.markDirty(); return true; }
    server_.stop(); active_ = false;   // okno "aktywne" bez WiFi -> restart od zera
  }
  if (!app_.ntp().ensureWifi(ntpcfg::WIFI_TIMEOUT_MS)) {
    app_.setStatus("brak polaczenia WiFi");
    app_.ntp().stopWifiAndNtp();
    app_.markDirty();
    return false;
  }
  if (!routesInstalled_) { installRoutes(); routesInstalled_ = true; }
  server_.begin();
  active_ = true;
  extendWindow();
  app_.markDirty();
  Serial.print("WWW IP: "); Serial.println(WiFi.localIP());
  return true;
}

void WebPanel::stopWindow() {
  server_.stop();
  active_ = false;
  // Aktywna proba NTP (do ~50 s) sama dokonczy walidacje i wylaczy WiFi.
  if (!app_.ntp().isBusy()) app_.ntp().stopWifiAndNtp();
  app_.markDirty();
}

void WebPanel::forceStop() {
  server_.stop();
  active_ = false;
}

void WebPanel::service() {
  if (WiFi.status() != WL_CONNECTED) {
    // Chwilowy zanik (roaming) nie zamyka okna od razu: auto-reconnect zwykle wraca.
    if (wifiLostMs_ == 0) wifiLostMs_ = millis();
    if (millis() - wifiLostMs_ > webcfg::WIFI_GRACE_MS) {
      wifiLostMs_ = 0;
      app_.setStatus("WiFi rozlaczone");
      stopWindow();
    }
  } else {
    wifiLostMs_ = 0;
    server_.handleClient();
    if ((int32_t)(millis() - untilMs_) >= 0) stopWindow();
  }
}

// ============================================================================
//  IMPLEMENTACJE — PowerPlanner (tryb nocny / light-sleep / sen ochronny)
// ============================================================================
void PowerPlanner::releaseNightHolds() {
  gpio_deep_sleep_hold_dis();
  for (gpio_num_t pin : pins::DEEP_SLEEP_HOLD) gpio_hold_dis(pin);
  gpio_hold_dis(pins::PA_ENABLE);
  rtc_gpio_deinit(pins::KEY);
  rtc_gpio_deinit(pins::PREVIEW);
  rtc_gpio_deinit(pins::RTC_INT);
}

// Zatrzasnij piny panelu i wzmacniacza w bezpiecznym stanie na czas deep-sleep
// (bez holdu piny plywaja i panel dostaje smieci / PA strzela).
void PowerPlanner::holdNightPins() {
  pinMode(pins::LCD_RST, OUTPUT);  digitalWrite(pins::LCD_RST, HIGH);
  pinMode(pins::LCD_CS, OUTPUT);   digitalWrite(pins::LCD_CS, HIGH);
  pinMode(pins::LCD_DC, OUTPUT);   digitalWrite(pins::LCD_DC, HIGH);
  pinMode(pins::LCD_SCK, OUTPUT);  digitalWrite(pins::LCD_SCK, LOW);
  pinMode(pins::LCD_MOSI, OUTPUT); digitalWrite(pins::LCD_MOSI, LOW);
  pinMode(pins::PA_ENABLE, OUTPUT); digitalWrite(pins::PA_ENABLE, LOW);
  for (gpio_num_t pin : pins::DEEP_SLEEP_HOLD) gpio_hold_en(pin);
  gpio_hold_en(pins::PA_ENABLE);
  gpio_deep_sleep_hold_en();
}

void PowerPlanner::shutdownNightPeripherals() {
  if (app_.web().isActive()) app_.web().forceStop();
  app_.ntp().stopWifiAndNtp();
  app_.audio().stopPlayback(StopCause::User);
  app_.audio().shutdownCodec();
  amplifierOff();
}

// Wspolne zejscie do deep-sleep (noc + ochrona ogniwa). Wolajacy narysowal juz
// swoja strone i policzyl maske EXT1. Kolejnosc 1:1 z v7.x: flush ekranu ->
// peryferia off -> CPU w dol -> zrodla wybudzen + pull-upy -> hold pinow -> sen.
void PowerPlanner::beginDeepSleep(uint64_t sleepUs, uint64_t ext1Mask) {
  delay(powercfg::DISPLAY_FLUSH_MS);        // panel konczy odswiezanie ostatniej strony
  shutdownNightPeripherals();
  setCpuFrequencyMhz(powercfg::PRE_SLEEP_CPU_MHZ);
  esp_sleep_enable_timer_wakeup(sleepUs);
  esp_sleep_enable_ext1_wakeup(ext1Mask, ESP_EXT1_WAKEUP_ANY_LOW);
  for (gpio_num_t pin : { pins::KEY, pins::PREVIEW, pins::RTC_INT }) {
    if (ext1Mask & (1ULL << pin)) {
      rtc_gpio_pullup_en(pin);
      rtc_gpio_pulldown_dis(pin);
    }
  }
  watchdogFeed();
  holdNightPins();
  Serial.flush();
  esp_deep_sleep_start();
}

time_t PowerPlanner::nightEndEpoch(time_t now) {
  if (!isSupportedEpoch(now)) return 0;
  struct tm t; localtime_r(&now, &t);
  return nextOccurrence(t, powercfg::NIGHT_END_HOUR, powercfg::NIGHT_END_MIN, 0, now);
}

#if ENABLE_LOW_BATTERY_PROTECT
// Ochronny deep-sleep przy krytycznym ogniwie: chroni Li-ion przed glebokim
// rozladowaniem i brownoutem. Budzi sie cyklicznie (timer) sprawdzic ladowanie;
// KEY pozwala wybudzic recznie. Deep-sleep resetuje chip -> setup() ponownie
// zmierzy ogniwo i wznowi prace lub usnie dalej.
void PowerPlanner::enterCriticalBatterySleepIfNeeded() {
  if (!app_.battery().isCritical()) return;
  if (app_.busy()) return;   // dzwoni budzik / audio / WWW / podglad / aktywny NTP / zapis RTC

  Serial.printf("Akumulator krytyczny (%u%%, %.2f V) -> ochronny sen %lu s\n",
                app_.battery().percent(), app_.battery().voltage(),
                (unsigned long)batcfg::CRITICAL_RECHECK_SEC);
  app_.displayUi().drawCriticalBatteryPage();
  beginDeepSleep((uint64_t)batcfg::CRITICAL_RECHECK_SEC * 1000000ULL, 1ULL << pins::KEY);
}
#endif

// Tryb nocny: deep-sleep do 06:00 / najblizszego budzika. Ekran zgaszony. Budzenie:
// alarm RTC (dokladny) + timer ESP (zabezpieczenie) + przyciski EXT1.
void PowerPlanner::enterNightSleepIfNeeded(bool force) {
  struct tm t;
  if (!getLocalTm(&t) || !isNightMode(t)) return;
  if (app_.alarms().anyDueNow(t)) return;
  if (!force && (app_.busy() || app_.buttons().wakeHeld())) return;   // utrzymaj wybudzenie po nacisnieciu

  time_t now = time(nullptr);
  time_t wakeAt = nightEndEpoch(now);
  time_t alarmAt = app_.alarms().nextEpoch(t, now);
  if (alarmAt > now && alarmAt < wakeAt) wakeAt = alarmAt;
  if (wakeAt <= now) wakeAt = now + (time_t)powercfg::NIGHT_EMERGENCY_SLEEP_SEC;
  uint64_t sleepSec = (uint64_t)(wakeAt - now);
  if (sleepSec < powercfg::NIGHT_MIN_SLEEP_SEC) {
    sleepSec = powercfg::NIGHT_MIN_SLEEP_SEC;
    wakeAt = now + (time_t)sleepSec;
  }

  bool rtcAlarmOk = app_.rtc().setWakeAlarm(wakeAt);
  // INT juz w stanie LOW = alarm odpaliłby natychmiast lub pin jest wadliwy ->
  // nie ufaj mu tej nocy.
  if (rtcAlarmOk && digitalRead(pins::RTC_INT) != HIGH) {
    app_.rtc().disableWakeAlarm();
    rtcAlarmOk = false;
  }
  // Bez dzialajacego alarmu RTC budzi tylko timer ESP na niedokladnym RC: spij w
  // odcinkach, by kazde wybudzenie zakotwiczylo czas z PCF85063A (setup) i dryft
  // pojedynczego odcinka byl << okno wylapania budzika.
  if (!rtcAlarmOk) {
    bool alarmIsNextEvent = (alarmAt > now && wakeAt == alarmAt);
    uint64_t cap = alarmIsNextEvent ? powercfg::NIGHT_FALLBACK_ALARM_CHUNK_SEC
                                    : powercfg::NIGHT_FALLBACK_CHUNK_SEC;
    if (sleepSec > cap) sleepSec = cap;
  }
  Serial.printf("Tryb nocny: sleep %llu s, RTC alarm: %s\n",
                (unsigned long long)sleepSec, rtcAlarmOk ? "OK" : "ERR");
  app_.displayUi().drawNightPage();
  uint64_t wakeMask = (1ULL << pins::KEY) | (1ULL << pins::PREVIEW);
  if (rtcAlarmOk) wakeMask |= (1ULL << pins::RTC_INT);
  beginDeepSleep(sleepSec * 1000000ULL, wakeMask);
}

// Dynamiczny light-sleep w dzien: spij do granicy nastepnej minuty lub do sekundy
// najblizszego budzika, max 60 s. RLCD trzyma obraz; przyciski wybudzaja przez EXT1.
// Light-sleep NIE rebootuje -> setup() sie nie powtarza -> brak mrugania.
uint64_t PowerPlanner::computeIdleSleepUs() {
  time_t now = time(nullptr);
  if (!isSupportedEpoch(now)) return powercfg::IDLE_LIGHT_SLEEP_US;   // czas nieznany -> krotki sen
  struct tm t; localtime_r(&now, &t);
  uint64_t sleepSec = 60 - (uint64_t)t.tm_sec;             // do granicy minuty
  time_t alarmAt = app_.alarms().nextEpoch(t, now);
  if (alarmAt > now) {
    uint64_t toAlarm = (uint64_t)(alarmAt - now);
    if (toAlarm < sleepSec) sleepSec = toAlarm;
  }
  if (sleepSec > powercfg::IDLE_LIGHT_SLEEP_MAX_SEC) sleepSec = powercfg::IDLE_LIGHT_SLEEP_MAX_SEC;
  if (sleepSec < 1) sleepSec = 1;
  return sleepSec * 1000000ULL;
}

void PowerPlanner::idlePowerSave() {
  // Nie usypiaj na dlugo, gdy cos wymaga szybkiej obslugi (App::busy) lub trwa
  // okno utrzymania wybudzenia po nacisnieciu przycisku (ButtonInput::wakeHeld).
  if (app_.busy() || app_.buttons().wakeHeld()) { delay(5); return; }
  if (digitalRead(pins::KEY) == LOW || digitalRead(pins::PREVIEW) == LOW) { delay(5); return; }

  if (app_.audio().codecActive()) app_.audio().shutdownCodec();   // kodek niepotrzebny -> uspij ES8311

  esp_sleep_enable_timer_wakeup(computeIdleSleepUs());
  const uint64_t wakeMask = (1ULL << pins::KEY) | (1ULL << pins::PREVIEW);
  esp_err_t extWake = esp_sleep_enable_ext1_wakeup(wakeMask, ESP_EXT1_WAKEUP_ANY_LOW);
  // UWAGA SPRZETOWA: NIE dodawac tu esp_sleep_pd_config(ESP_PD_DOMAIN_VDDSDIO, OFF)
  // — flash wisi na tej szynie; odciecie konczylo sie rebootem i cichym budzikiem.
  watchdogFeed();
  esp_light_sleep_start();
  watchdogFeed();
  // EXT1 zostawia RTC-hold na padach wybudzajacych — zwolnij od razu, inaczej
  // digitalRead zwraca zatrzasniety stan i przyciski sa martwe do restartu.
  rtc_gpio_hold_dis(pins::KEY);
  rtc_gpio_hold_dis(pins::PREVIEW);
  esp_sleep_wakeup_cause_t wakeCause = esp_sleep_get_wakeup_cause();
  uint64_t wakePins = wakeCause == ESP_SLEEP_WAKEUP_EXT1 ? esp_sleep_get_ext1_wakeup_status() : 0;
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
  if (extWake == ESP_OK) esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_EXT1);

  // Po light-sleep czas systemowy szedl z mniej dokladnego RTC slow clock —
  // zakotwicz ponownie z PCF85063A, by dryft nie przesuwal zmiany minuty.
  app_.rtc().readToSystem();

  if (wakeCause == ESP_SLEEP_WAKEUP_EXT1) {
    if (wakePins & (1ULL << pins::KEY))     app_.buttons().primeKeyFromLightSleepWake();
    if (wakePins & (1ULL << pins::PREVIEW)) app_.buttons().onPreviewWakeFromLightSleep();
  }
}

// ============================================================================
//  IMPLEMENTACJE — ButtonInput (polityka przyciskow)
// ============================================================================
void ButtonInput::begin() {
  keyButton_.begin();
  previewButton_.begin();
}

void ButtonInput::pollKey() {
  switch (keyButton_.poll()) {
    case ButtonEvent::Pressed:
      holdWake();
      break;
    case ButtonEvent::Released:
      // Zwolnienie po dlugim (>= KEY_LONG_PRESS_MS) przytrzymaniu: celowo BEZ akcji
      // (dlugie przytrzymanie tylko tlumi akcje krotka). begin()/primePressed()
      // zawsze uzbrajaja pomiar, wiec kontrakt obowiazuje takze dla KEY trzymanego
      // od zimnego startu (fix v8.0 — v7.0 odpalal wtedy panel WWW).
      if (keyButton_.lastPressKnown() &&
          keyButton_.lastPressDurationMs() < btncfg::KEY_LONG_PRESS_MS) {
        if (app_.audioActive()) app_.audio().stopPlayback(StopCause::User);  // dzwoni -> wycisz
        else app_.web().startWebWindow();                                    // nie dzwoni -> panel WWW
      }
      break;
    case ButtonEvent::None:
      break;
  }
}

void ButtonInput::pollPreview() {
  switch (previewButton_.poll()) {
    case ButtonEvent::Pressed:
      holdWake();
      break;
    case ButtonEvent::Released:
      // Tylko krotki, zmierzony klik pokazuje podglad; dluzsze przytrzymanie
      // (> BOOT_SHORT_PRESS_MAX_MS) celowo bez akcji.
      if (previewButton_.lastPressKnown() &&
          previewButton_.lastPressDurationMs() <= btncfg::BOOT_SHORT_PRESS_MAX_MS) {
        showPreview();
      }
      break;
    case ButtonEvent::None:
      break;
  }
}

void ButtonInput::showPreview() {
  if (app_.audioActive()) return;   // dzwonienie ma pierwszenstwo przed podgladem
  previewActive_ = true;
  previewUntilMs_ = millis() + btncfg::PREVIEW_SHOW_MS;
  holdWake();
  app_.markDirty();
}

void ButtonInput::primeKeyFromSetup() {
  keyButton_.primePressed();
}

void ButtonInput::primeKeyFromLightSleepWake() {
  keyButton_.primePressed();
  holdWake();
}

void ButtonInput::onPreviewWakeFromLightSleep() {
  previewButton_.primePressed();
  showPreview();
}

// ============================================================================
//  IMPLEMENTACJE — SettingsStore (NVS + diagnostyka resetu)
// ============================================================================
void SettingsStore::load() {
  app_.alarms().setDefaults();
  NvsSession session(prefs_);
  if (!session) {
    app_.ntp().setCreds(ntpcfg::DEFAULT_SSID, ntpcfg::DEFAULT_PASS);
    return;
  }
  if (prefs_.getBytesLength(nvskeys::ALARMS) == app_.alarms().rawSize())
    prefs_.getBytes(nvskeys::ALARMS, app_.alarms().rawData(), app_.alarms().rawSize());
  app_.alarms().sanitizeAll();
  String ssid = prefs_.getString(nvskeys::WIFI_SSID, ntpcfg::DEFAULT_SSID);
  String pass = prefs_.getString(nvskeys::WIFI_PASS, ntpcfg::DEFAULT_PASS);
  if (ssid.length() == 0) { ssid = ntpcfg::DEFAULT_SSID; pass = ntpcfg::DEFAULT_PASS; }
  app_.ntp().setCreds(ssid.c_str(), pass.c_str());
#if ENABLE_RTC_AUTOTRIM
  // Wykalibrowany offset przezywa utrate zasilania w NVS; brak klucza = start
  // z fabrycznego FACTORY_OFFSET_REG. Wczytany PRZED ensureOffsetCalibration().
  app_.rtc().setTargetOffsetLsb(
    (int8_t)prefs_.getChar(nvskeys::RTC_OFFSET_LSB, decodeRtcOffset(rtccfg::FACTORY_OFFSET_REG)));
#endif
}

void SettingsStore::save() {
  NvsSession session(prefs_);
  if (!session) return;
  prefs_.putBytes(nvskeys::ALARMS, app_.alarms().rawData(), app_.alarms().rawSize());
  prefs_.putString(nvskeys::WIFI_SSID, app_.ntp().ssid());
  prefs_.putString(nvskeys::WIFI_PASS, app_.ntp().pass());
}

// Zwiezle podsumowanie zuzycia baterii, nadpisywane max raz/dobe (malo flash).
void SettingsStore::saveDrainSummary(uint8_t soc, float mahPerDay, int day) {
  NvsSession session(prefs_);
  if (!session) return;
  prefs_.putUShort(nvskeys::DRAIN_SOC, soc);
  prefs_.putFloat(nvskeys::DRAIN_MAH_DAY, mahPerDay);
  prefs_.putUInt(nvskeys::DRAIN_DAY, (uint32_t)day);
}

#if ENABLE_RTC_AUTOTRIM
void SettingsStore::saveRtcOffset(int8_t lsb) {
  NvsSession session(prefs_);
  if (!session) return;
  prefs_.putChar(nvskeys::RTC_OFFSET_LSB, lsb);
}
#endif

#if ENABLE_DIAGNOSTICS
// Wczytaj dozywotnie liczniki resetow i — jesli ten rozruch to brownout/panic/WDT
// — zwieksz odpowiedni. Zdarzenia rzadkie, wiec zapis do flash bezpieczny.
void SettingsStore::recordResetReason(esp_reset_reason_t r) {
  NvsSession session(prefs_);
  if (!session) return;
  brownoutResets_ = prefs_.getUInt(nvskeys::RESETS_BROWNOUT, 0);
  wdtResets_      = prefs_.getUInt(nvskeys::RESETS_WDT,      0);
  panicResets_    = prefs_.getUInt(nvskeys::RESETS_PANIC,    0);
  if (r == ESP_RST_BROWNOUT)                                                  prefs_.putUInt(nvskeys::RESETS_BROWNOUT, ++brownoutResets_);
  else if (r == ESP_RST_TASK_WDT || r == ESP_RST_INT_WDT || r == ESP_RST_WDT) prefs_.putUInt(nvskeys::RESETS_WDT,      ++wdtResets_);
  else if (r == ESP_RST_PANIC)                                                prefs_.putUInt(nvskeys::RESETS_PANIC,    ++panicResets_);
}
#endif

// ============================================================================
//  IMPLEMENTACJE — App (orkiestracja setup/loop)
// ============================================================================
void App::setup() {
  // Przyczyne wybudzenia klasyfikujemy NAJPIERW — decyduje o zerowaniu stanu,
  // opoznieniu USB-CDC i akcjach po wybudzeniu.
  const WakeInfo wake = WakeInfo::classify();

  Serial.begin(115200);
  if (wake.coldBoot) delay(COLD_BOOT_SERIAL_DELAY_MS);   // tylko zimny start (USB-CDC)
  setCpuFrequencyMhz(powercfg::DAY_CPU_MHZ);
  watchdogSetup();
  power_.releaseNightHolds();     // MUSI poprzedzac dotykanie pinow LCD/PA (holdy z nocy)
  amplifierOff();
  display_.setup();

  if (wake.coldBoot) {            // RTC RAM przezywa deep-sleep -> zeruj tylko na zimnym starcie
    alarms_.resetTriggerState();
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

  setenv("TZ", ntpcfg::TZ_POLAND, 1);
  tzset();

  pinMode(pins::RTC_INT, INPUT_PULLUP);
  buttons_.begin();

  settings_.load();               // wczytuje m.in. offset RTC z NVS (PRZED kalibracja!)
#if ENABLE_DIAGNOSTICS
  settings_.recordResetReason(esp_reset_reason());
#endif
  Serial.printf("\n=== %s v%s  (build %s %s) ===\n", FW_NAME, FW_VERSION, FW_BUILD_DATE, FW_BUILD_TIME);
  Serial.printf("Rozruch #%u, przyczyna resetu: %s\n", g_persist.bootCount, resetReasonText(esp_reset_reason()));
  battery_.update(true);

  bool timeFromRtc = rtc_.readToSystem();
  rtc_.ensureOffsetCalibration();        // kalibracja dryfu (0x02) + odczyt zwrotny
  bool rtcAlarmCleared = rtc_.disableWakeAlarm();
  setNtpStatus(timeFromRtc ? "czas z RTC" : "RTC pusty/brak");
  if (wake.rtcWake && !rtcAlarmCleared) setNtpStatus("RTC alarm: blad kasowania");
#if ENABLE_DIAGNOSTICS
  { time_t t0 = time(nullptr);
    if (g_persist.coldBootEpoch == 0 && isSupportedEpoch(t0)) g_persist.coldBootEpoch = (int64_t)t0; }
#endif

  audio_.codecPowerDown();               // ES8311 startuje w stanie niskiego poboru

  if (wake.coldBoot) soundStore_.ensureMounted();   // FS tylko do MP3 — montuj leniwie
  alarms_.checkAlarms(wake.scheduledWake() ? ntpcfg::ALARM_CATCHUP_SEC : alarmcfg::LATE_WINDOW_SEC);
  if (!wake.userWake() && !audio_.isRinging()) power_.enterNightSleepIfNeeded(true);
#if ENABLE_LOW_BATTERY_PROTECT
  if (!wake.userWake() && !audio_.isRinging()) power_.enterCriticalBatterySleepIfNeeded();
#endif

  // WiFi domyslnie OFF. Sync NTP z setup: zimny start lub pusty RTC. Poranne
  // okno 06:00 obsluguje NetworkTimeService::checkSchedule() w loop() (fix v8.0:
  // dziala takze, gdy urzadzenie czuwalo przez 06:00 i setup sie nie wykonal).
  bool willSync = (wake.coldBoot || !timeFromRtc);
  if (!willSync) ntp_.stopWifiAndNtp();
  if (wake.userWake()) buttons_.holdWake();
  if (wake.previewWake) buttons_.showPreview();
  if (wake.keyWake && !wake.coldBoot) buttons_.primeKeyFromSetup();   // KEY z deep-sleep: mierz nacisniecie
  if (willSync) {
    ntp_.setRetries(timeFromRtc ? 2 : 3);   // pusty RTC: kilka szybkich prob, potem backoff
    ntp_.syncNow(SyncTrigger::Scheduled);
  }

  markDirty();
}

void App::loop() {
  watchdogFeed();
  buttons_.pollKey();
  buttons_.pollPreview();
  power_.enterNightSleepIfNeeded(false);

  if (web_.isActive()) web_.service();

  audio_.serviceAudio();
  audio_.serviceRinging();
  // Koniec dzwieku w czasie okna WWW/podgladu: idlePowerSave nie biega (busy),
  // wiec uspienie ES8311 i zdjecie podbicia CPU robimy tu — jedno miejsce dla
  // wszystkich sciezek konca odtwarzania. Przerwy miedzy powtorkami budzika
  // chroni audioActive() (ringing_ trzyma true).
  if (audio_.codecActive() && !audioActive()) audio_.shutdownCodec();
  battery_.update(false);
#if ENABLE_LOW_BATTERY_PROTECT
  power_.enterCriticalBatterySleepIfNeeded();
#endif
  bool ntpAdjustedTime = ntp_.service();
  // Okno spoznienia poszerzone o realny przestoj petli: dlugi handleClient (upload
  // MP3) ani 60 s light-sleep nie moga zgubic budzika z terminem w trakcie przestoju.
  uint32_t loopGapSec = lastAlarmCheckMs_ ? (millis() - lastAlarmCheckMs_) / 1000UL : 0;
  lastAlarmCheckMs_ = millis();
  uint32_t lateWindowSec = ntpAdjustedTime ? ntpcfg::ALARM_CATCHUP_SEC : alarmcfg::LATE_WINDOW_SEC;
  if (loopGapSec + alarmcfg::LOOP_GAP_MARGIN_SEC > lateWindowSec)
    lateWindowSec = loopGapSec + alarmcfg::LOOP_GAP_MARGIN_SEC;
  alarms_.checkAlarms(lateWindowSec);
  ntp_.checkSchedule();
  ntp_.handleRtcWriteRetry();
  display_.update();
  power_.idlePowerSave();
}

// ============================================================================
//  Punkty wejscia Arduino — cala logika w App.
// ============================================================================
void setup() { app.setup(); }
void loop()  { app.loop(); }
