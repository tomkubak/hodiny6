# Digitronové hodiny – 6 digitronů

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

## Zapojení

Výchozí zapojení (platí i po továrním nastavení):

| Signál | GPIO | Poznámka |
|---|---|---|
| CLK | 1 | přes invertující tranzistor |
| DIN1 | 3 | driver digitronů 1–3 |
| DIN2 | 2 | driver digitronů 4–6 (jen paralelně) |
| OE (jas) | 0 | PWM, přes invertující tranzistor |
| STR (latch) | – | volitelný, výchozí nezapojený |
| VN zdroj | 7 | HIGH = zapnutý (pevný) |
| RTC SDA / SCL | 8 / 9 | DS3231 (pevné; bez RTC volné pro displej) |

Piny displeje jdou změnit v kartě **Zapojení desky** (GPIO 0–6, 10, 20, 21; bez RTC i 8 a 9).

### Drivery HV5222

- **Paralelně** – společný CLK, každý driver má vlastní DIN (DIN1 = digitrony 1–3, DIN2 = 4–6).
- **Do série** – DIN → 1. driver → jeho DOUT → DIN 2. driveru; posílá se 64 bitů.
  Volba *První driver za ESP řídí* určuje, který driver je blíž k ESP
  (když ukazují digitrony 456123 místo 123456, přepni ji).
- **STR** – když je zapojený, bity se nasunou a jeden pulz je převezme na výstupy najednou
  (nic neproblikne). Polarita *STR aktivní v HIGH / LOW* podle toho, jestli vede přes tranzistor.

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
`/api/ntpsync`, `/api/clean`, `/api/pinscan`, `/api/wifi`, `GET /api/scan`, `/api/restart`, `/api/factory`.
