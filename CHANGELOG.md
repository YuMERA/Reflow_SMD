# Changelog

## [1.1] - 2026-11-14

### Bezbednost
- **Ispravljeno:** kada se termopar otkači, MAX6675 vraća `NAN`, PID izlaz postaje `NAN`, a poređenje
  u PWM logici je zbog toga uključivalo grejač **trajno na 100%**. Sada svako neispravno očitavanje
  odmah gasi SSR, a posle 3 uzastopna proces staje sa greškom `SENSOR ERROR`.
- Novo: apsolutna granica temperature 270 °C (`OVER TEMP!`), proverava se i kada proces ne radi.
- Novo: provera rasta temperature (`NO HEAT RISE`): grejač na ≥90% snage, a temperatura ne poraste 3 °C za 90 s.
- Novo: ekran greške sa alarmom; greška se potvrđuje tasterom tek kada je stanje ponovo ispravno.
- START je blokiran ako senzor ne radi (i pri paljenju uređaja).
- SSR se uključuje samo preko funkcije `setHeater()`, koja proverava proces, grešku i senzor.
- SSR pin se postavlja na LOW kao prva stvar u `setup()`.

### Ispravke
- PWM prozor se resetuje na START. Ranije je posle dužeg čekanja grejač kasnio desetine sekundi
  dok `windowStartTime` ne "sustigne" `millis()`.
- PID se prebacuje u `MANUAL` na STOP i u `AUTOMATIC` na START, pa integral iz prethodnog ciklusa
  više ne izaziva skok snage.
- PID računa jednom po novom očitavanju temperature (`SetSampleTime`). Ranije je računao na svakih 100 ms,
  a temperatura se menjala svakih 250 ms, pa je D-član dobijao lažne skokove.
- MAX6675 se pri paljenju čita tek posle 500 ms (vreme prve konverzije).
- Poruka o grešci senzora pri paljenju se više ne ispisuje u jednom redu.
- `#include "mera_logo.h"` umesto `<mera_logo.h>`.

### Izmene
- Serijski izlaz je CSV (`time_s,temp_c,setpoint_c,output_pct`) jednom u sekundi, umesto dvostrukog
  ispisa u svakom prolazu petlje.
- Ekran se osvežava 5 puta u sekundi, umesto u svakom prolazu petlje.
- Kod je podeljen u funkcije (`readTemperature`, `handleEncoder`, `handleButton`, `driveHeater`, ...).
- PID parametri su nepromenjeni.

## [1.0] - 2025-07-01

- Prva verzija: PID regulacija konstantne temperature, gain scheduling za Ki, OLED meni (SET / BEEP / START-STOP).
- Sačuvana u folderu `v1.0/` i kao git tag `v1.0`.
