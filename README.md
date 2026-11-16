# Digitronové hodiny – 6 digitronů

Projekt na EasyEDA (schéma a plošňák): <https://oshwlab.com/tomkubak/6nixie>

### Seznam součástek

|  | Ks | Poznámka |
|---|---|---|
| R 0805 18k | 6× | |
| R 0805 10k | 4× | |
| R 0805 2k2 | 4× | |
| C 1206 100nF/25V | 2x | |
| BC817 SOT23 | 4× | |
| HV5222 | 2× | AliExpress |
| step-up 5V-12V | 1× | [AliExpress](https://bit.ly/4iRAGcT) |
| step-up 12V-220V | 1× | [AliExpress](https://bit.ly/48WsiUZ) – pozn.: žlutý drát na GPIO 4 (SHDN) |
| ESP32-C3 supermini | 1× | |

Volitelně pro provoz bez internetu:

|  | Ks |
|---|---|
| DS3231 | 1× |
| 4k7 | 2× |
| CR2032 | |

---

AI slop: ↓

Firmware pro digitronové (nixie) hodiny se šesti digitrony (HH:MM:SS), dvěma drivery **HV5222**
a deskou **ESP32-C3 SuperMini**. Ovládání je přes webovou stránku v telefonu, čas z internetu (NTP)
a volitelně z obvodu RTC DS3231.

## Co umí

- **Přesný čas** – NTP na pozadí, časové pásmo s automatickým letním/zimním časem,
  volitelně RTC DS3231 (drží čas i bez internetu a bez proudu). Bez RTC čas jen z NTP
  nebo ručně z telefonu.
- **Webové rozhraní** – vše se nastavuje z prohlížeče, funguje i jako aplikace na ploše telefonu.
- **Wi-Fi s návratem** – když domácí síť nejde, hodiny vytvoří vlastní síť `hodiny`
  (captive portál – stránka se v telefonu otevře sama).
- **Jas a noční režim** – jas přes PWM, noční jas nebo úplné vypnutí VN zdroje.
- **Ochrana katod** – 10 efektů protočení číslic (každou minutu / 10 min / hodinu)
  a denní hloubkové čištění.
- **Datum** – v sekundách 30–33 volitelně DD MM RR.
- **Aktualizace přes Wi-Fi** – webem (`/update`) nebo z PlatformIO (ArduinoOTA), průběh na digitronech.
- **Nastavitelné zapojení desky** – piny CLK / DIN / OE / STR, drivery paralelně nebo do série,
  a **průvodce, který zapojení sám najde** (viz níže).

## Stažení firmwaru

Hotové soubory jsou v [Releases](../../releases):

| Soubor | K čemu |
|---|---|
| `hodiny6_vX.Y.Z_ota.bin` | aktualizace hodin, které už běží (web `/update` nebo PlatformIO) |
| `hodiny6_vX.Y.Z_full.bin` | první nahrání do čisté desky přes USB (adresa `0x0`) |

### První nahrání přes USB

Nejjednodušší je [ESP Web Flasher](https://espressif.github.io/esptool-js/) v Chrome / Edge:
připoj SuperMini kabelem, *Connect*, soubor `_full.bin` na adresu **0x0**, *Program*.

Nebo z příkazové řádky:

```
esptool.py --chip esp32c3 write_flash 0x0 hodiny6_vX.Y.Z_full.bin
```

(Když se deska nepřipojí: drž tlačítko **BOOT**, krátce stiskni **RESET**, pusť BOOT.)

### Aktualizace

Otevři `http://hodiny.local/update` (nebo IP adresu hodin + `/update`) a nahraj `_ota.bin`.
Nastavení i Wi-Fi zůstanou.

## První spuštění

1. Hodiny po startu vytvoří otevřenou Wi-Fi **`hodiny`**. Připoj se k ní telefonem,
   stránka se otevře sama (jinak `http://192.168.4.1`).
2. V kartě **WiFi** vyber domácí síť a zadej heslo. Hodiny se restartují a připojí.
3. Po připojení ukážou na digitronech svou IP adresu. Stránka pak běží na
   `http://hodiny.local` nebo na té IP adrese.

## Výchozí nastavení

Platí po prvním nahrání `_full.bin` i po továrním nastavení (fw 2.6.1):

| Oblast | Výchozí hodnota |
|---|---|
| Wi-Fi | žádná uložená → otevřená síť `hodiny`, stránka na `http://192.168.4.1` |
| Název v síti | `hodiny` (`http://hodiny.local`) |
| NTP | `cz.pool.ntp.org`, `tik.cesnet.cz`, synchronizace každých 6 h |
| Časové pásmo | Praha (SEČ / SELČ, automatický letní čas) |
| Jas | 60 % |
| Úvodní nula / datum | nula se zobrazuje, datum vypnuté |
| Noční režim | vypnutý (po zapnutí 23:00–6:00, jas 10 %, VN zůstává zapnutý) |
| Rychlé protočení číslic | každou minutu, náhodný efekt |
| Denní hloubkové čištění | zapnuté, začátek ve 3 h, délka 5 min |
| Přenos do registrů | délka hodinového pulzu 8 µs, zhasnout během přenosu zapnuto |
| Vysílací výkon Wi-Fi | 15 dBm |
| RTC DS3231 | zapnuté |
| Heslo pro aktualizace | žádné |

## Zapojení

Výchozí zapojení (platí i po továrním nastavení):

| Signál | GPIO | Poznámka |
|---|---|---|
| Drivery | – | výchozí do série (DIN → DOUT → DIN), první driver za ESP řídí digitrony 1–3 |
| CLK | 1 | přes invertující tranzistor |
| DIN | 3 | vstup prvního driveru |
| OE (jas) | 0 | PWM, přes invertující tranzistor |
| STR (latch) | 2 | aktivní v LOW (volitelný, jde i „nezapojeno“) |
| VN zdroj | 4 | SHDN step-upu 12V-220V (žlutý drát), zapnutý při LOW |
| RTC SDA / SCL | 8 / 9 | DS3231 (pevné; bez RTC volné pro displej) |

Piny displeje i VN zdroje jdou změnit v kartě **Zapojení desky** (GPIO 0–7, 10, 20, 21; bez RTC i 8 a 9).
Pin VN zdroje průvodce hledáním zapojení nikdy nezkouší.

### Drivery HV5222

- **Paralelně** – společný CLK, každý driver má vlastní DIN (DIN1 = digitrony 1–3, DIN2 = 4–6).
- **Do série** – DIN → 1. driver → jeho DOUT → DIN 2. driveru; posílá se 64 bitů.
  Volba *První driver za ESP řídí* určuje, který driver je blíž k ESP
  (když ukazují digitrony 456123 místo 123456, přepni ji).
- **STR** – když je zapojený, bity se nasunou a jeden pulz je převezme na výstupy najednou
  (nic neproblikne). Polarita *STR aktivní v HIGH / LOW* podle toho, jestli vede přes tranzistor.

### Test digitronů

**Zapojení desky → Test digitronů**:

- **Všechny číslice 0–9** – všechny digitrony ukazují postupně stejnou číslici.
- **Výstupy driverů** – svítí vždy jen jeden výstup HV5222 (0–31 driver digitronů 1–3, 32–63 driver 4–6).
  Stránka ukáže číslo výstupu a kterou číslici na něm firmware čeká – tak se najde, kam který výstup vede.

Pauza / Zpět / Další jde i ručně, test sám skončí po 30 minutách.

### Průvodce „Hledat zapojení“

Když nevíš, na kterých pinech co je: **Zapojení desky → Hledat zapojení**. Hodiny postupně zkoušejí
piny a ty u každého kroku stiskneš tlačítko, jakmile se na digitronech něco stane:

1. **CLK** – rozsvítí se nějaké číslice,
2. **DIN1** – vlevo 1 2 3 (do série celé 1 2 3 4 5 6),
3. **DIN2** – vpravo 4 5 6 (jen paralelně),
4. **OE** – displej bliká.

**Hledat STR** (až když CLK, DIN a OE fungují) zkouší každý pin s oběma polaritami: dokud displej
bliká, není to ono; správný STR = 1 2 3 4 5 6 svítí klidně.

Během hledání jsou vybrané piny výstupy – odškrtni ty, na kterých je zapojené něco jiného.
Průvodce počítá s invertujícími tranzistory na CLK, DIN i OE.

## Sestavení ze zdrojáků

[PlatformIO](https://platformio.org/) (VS Code nebo příkazová řádka):

```
pio run                      # sestavení
pio run -t upload            # nahrání přes USB
pio run -e ota -t upload     # nahrání přes Wi-Fi (hodiny.local)
```

Ověřeno s Arduino core 2.0.17 (platform espressif32 6.x), ESPAsyncWebServer 3.12, AsyncTCP 3.5,
ElegantOTA 3.1.7, RTClib 2.1.4. Na Macu s Apple Silicon potřebuje toolchain Rosettu
(`softwareupdate --install-rosetta`).

Zdrojáky: `src/main.cpp` (firmware), `src/web_ui.h` (webová stránka), `src/icons.h` (ikony).

## API

Stránka používá jednoduché JSON API – `GET /api/status`, `GET/POST /api/config`, `POST /api/settime`,
`/api/ntpsync`, `/api/clean`, `/api/test`, `/api/pinscan`, `/api/wifi`, `GET /api/scan`, `/api/restart`, `/api/factory`.
