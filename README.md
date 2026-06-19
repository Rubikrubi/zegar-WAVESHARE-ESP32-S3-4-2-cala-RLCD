# Zegar NTP na WAVESHARE ESP32-S3 4.2" RLCD

Projekt zegara NTP dla płytki **WAVESHARE ESP32-S3 z wyświetlaczem 4.2" RLCD**.
Kod obsługuje synchronizację czasu przez Internet, RTC **PCF85063A**, budziki MP3, panel WWW oraz tryby oszczędzania energii.

![Zegar NTP na Waveshare ESP32-S3 RLCD](foto%201.png)

![Widok projektu](foto%202.png)

## Najważniejsze funkcje

* synchronizacja czasu przez NTP,
* podtrzymanie czasu przez RTC PCF85063A,
* obsługa budzików MP3,
* panel WWW do konfiguracji,
* zapis ustawień w pamięci ESP32,
* pomiar napięcia akumulatora,
* tryb dzienny z oszczędzaniem energii,
* tryb nocny / deep sleep,
* wyświetlanie dużego zegara na ekranie RLCD,
* obsługa plików dźwiękowych z folderu `data`.

## Sprzęt

Projekt przygotowany dla:

* WAVESHARE ESP32-S3 z wyświetlaczem 4.2" RLCD,
* wyświetlacz RLCD/ST7305,
* RTC PCF85063A,
* zasilanie z USB lub akumulatora Li-Ion,
* głośnik / moduł audio zgodny z konfiguracją w kodzie.

## Pliki w repozytorium

```text
zegar-WAVESHARE-ESP32-S3-4-2-cala-RLCD-main/
├── assets/
│   └── zegar-ntp-github.mp4
├── data/
│   ├── alarm1.mp3
│   ├── alarm2.mp3
│   └── alarm3.mp3
├── .gitignore
├── LICENSE
├── README.md
├── foto 1.png
├── foto 2.png
├── photo_font.h
└── zegar-WAVESHARE-ESP32-S3-4-2-cala-RLCD.ino
```

## Wymagane biblioteki

Biblioteki zależą od wersji kodu i konfiguracji środowiska Arduino IDE.
Sprawdź sekcję `#include` w pliku `.ino`.

W projekcie mogą być używane między innymi:

* Arduino core for ESP32,
* WiFi,
* WebServer,
* Preferences,
* Wire / I2C,
* biblioteka RTC dla PCF85063A,
* biblioteki obsługi wyświetlacza RLCD/ST7305,
* biblioteki obsługi MP3/audio.

## Wgrywanie projektu

1. Pobierz repozytorium z GitHuba.
2. Otwórz plik `.ino` w Arduino IDE.
3. Wybierz odpowiednią płytkę ESP32-S3.
4. Ustaw właściwy port COM.
5. Zainstaluj wymagane biblioteki.
6. Wgraj kod do płytki.
7. Jeżeli używasz plików MP3 z folderu `data`, wgraj je do pamięci urządzenia zgodnie z używanym systemem plików, np. LittleFS albo SPIFFS.

## Folder `data`

W folderze `data` znajdują się pliki dźwiękowe używane przez budzik.

Przykład:

```text
data/
├── alarm1.mp3
├── alarm2.mp3
└── alarm3.mp3
```

## Tryby pracy

Projekt może działać w kilku trybach:

* normalny tryb zegara,
* synchronizacja czasu przez NTP,
* praca z czasem podtrzymywanym przez RTC,
* tryb oszczędzania energii,
* tryb nocny / deep sleep.

---

## Film demonstracyjny – głośność budzika

Poniższy film prezentuje działanie funkcji budzika w projekcie zegara NTP na płytce **Waveshare ESP32-S3 4.2" RLCD**. Nagranie pokazuje, jak głośno dzwoni alarm oraz jak urządzenie zachowuje się podczas odtwarzania sygnału dźwiękowego.





https://github.com/user-attachments/assets/10ea0199-e07e-42e1-8aef-4cdc399ce904





**Uwaga:** rzeczywista głośność może zależeć od zastosowanego głośnika, ustawień poziomu dźwięku oraz warunków nagrania.

---

## AI assistance

Część kodu, refaktoryzacja oraz dokumentacja projektu były przygotowywane z pomocą modelu AI Claude Code  
Ostateczne sprawdzenie, testy na urządzeniu oraz publikację wykonał autor repozytorium.

---

## Autor

Projekt: **Zegar NTP na WAVESHARE ESP32-S3 4.2" RLCD**<br>
Autor:    **Rubikrubi**<br>
GitHub: [github.com/Rubikrubi](https://github.com/Rubikrubi)



---

## Licencja

Projekt udostępniony na licencji MIT.
Szczegóły znajdują się w pliku `LICENSE`.
