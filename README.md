# ALDE-3030-Aventa-gen1-
Control Alde 3030 and Aventa
# Alde 3030 Plus + Truma Aventa – MQTT-Integration

Steuerung und Überwachung der Alde Compact 3030 Plus und der Truma Aventa
über MQTT. Ein Raspberry Pi am Fahrzeug ist mit dem Yellow CI-Bus (Panel)
und dem Red Bus (via T stück) verbunden. Bitte bei Fragen einfach eine Nachricht oder Issues hinterlassen! 
Testlauf lief über 2 Monate fehlerfrei, Garantie natürlich keine. Getestet mit ALDE 3030 Compact, Aventa Comfort Gen1. 


Home Assistant kann im Fahrzeug oder remote laufen. Node-RED auf einem
Victron Cerbo oder jeder andere MQTT-Client sind ebenfalls nutzbar.
Voraussetzung: IP-Netzwerk und MQTT-Broker.

Dashboards (Home Assistant und Node-RED) folgen in einem zweiten Schritt.


---

## Funktionen

### Alde (Yellow CI-Bus)
- Raum- und Außentemperatur, Sollwert
- Gas, Elektroleistung (0–3 kW), Warmwasser (off / on / boost)
- Klima-Modus: alde auto / manual
- Display ein/aus, System aus
- Pumpe, Gasventil, AC-Eingang, Fehlerstatus
- Schutz gegen Legionellen-Sperre: nur bei eingeschaltetem Display.
  Bei Warmwasser on/boost, Warmwassertemperatur unter 43 °C und
  höchstens 1 kW Elektroleistung wird Gas eingeschaltet. Ohne diese
  Absicherung kann das Panel den Betrieb verweigern, bis vor Ort
  am Display bestätigt wird.

### Red Bus
- Aventa: Betriebsart, Lüfterstufe, Solltemperatur. Aventa Vent Mode's Modus Night etc. hatte ich nicht auf dem Radar, habe Sie einfach stehen lassen, wahrscheinlich sind diese bei anderen AVENTA'S vorhanden. Low wird immer erscheinen, das ist normal bei auch bei OFF! Aventa Aircon Mode hier sind alle vorhanden bei der Aventa Comfort Gen1. mit anderen Aventas wurde der Code nicht getestet, wird aber funktionieren!  
- Glykol- und Warmwassertemperatur
- External Start (externer Eingang): Zustand wird vom Bus gelesen
  und als MQTT-Binary-Sensor veröffentlicht
- Bei Panel-Modus alde auto wird vor Aventa-Befehlen zuerst manual
  gesetzt, anschließend gesendet und bei Bedarf wiederholt

### External-Start-Relais
Das Skript kann ein Relais-Topic bedienen (z. B. Victron Cerbo), damit
Heat, Gas oder Display den External Start hardwareseitig setzen.
Ohne Relais bleibt `EXTERNAL_START_TOPIC = None`.

---

## Architektur

| Skript | Bus | Schnittstelle | Aufgabe |
|--------|-----|---------------|---------|
| `alde_mqtt.py` | Yellow CI-Bus | Hardware-UART `/dev/ttyAMA0` | Panel steuern und Status lesen |
| `alde_red_bus_mqtt.py` | Red Bus | pigpio GPIO RX und TX | Aventa steuern, Temperaturen und External Start lesen |

Beide Dienste laufen unabhängig und erscheinen in Home Assistant unter
einem gemeinsamen Gerät. Im MQTT!

---

## Hardware

| Komponente | Anmerkung |
|------------|-----------|
| Raspberry Pi | z. B. Zero 2 W, Hardware-UART und GPIO |Buck Converter (12v auf 5v für den PI)
| 2× LIN-Transceiver | getestet: Soldered NCV7329 Breakout (Slave). Andere LIN-Transceiver sind kompatibel, Versorgungsspannung beobachten, zwingend 3.3V, wenn High!, sonst zerstört man den PI |
| RJ12 Kabel 2 Stück | Yellow- und Red-Buchse am Panel |

### Soldered NCV7329 – Anschlüsse

| Pin | Funktion |
|-----|----------|
| GND | Masse, gemeinsam mit dem Pi |
| VCC | Versorgung (Fahrzeug-12 V bzw. laut Modul) von da zwingend über ein buck-converter zum PI 5v! Nie direkt 12v an den Pi, würde er sofort zerstört!|
| LIN | Busleitung, RJ12 Pin 4 |
| EN | Enable, **fest auf 3,3 V** |
| TXD | Senden (Pi → Bus) |
| RXD | Empfangen (Bus → Pi) |

### Yellow
- Pi UART TX → Modul TXD  
- Pi UART RX ← Modul RXD  
- GND gemeinsam  
- EN → 3,3 V  
- LIN → Yellow RJ12 Pin 4  
- Baudrate 19200

### Red
- GPIO TX (Skript: `TX_PIN`) → Modul TXD  
- GPIO RX (Skript: `GPIO_PIN`) ← Modul RXD  
- GND, EN (3,3 V), VCC, LIN → Red RJ12 Pin 4  
- Baudrate 9600  

Pins im Skript an die Verdrahtung anpassen.

## Mitwirken

Issues und Pull Requests sind willkommen.
---

## Software

1.PI Software OS lite, läuft auch mit anderen Software, falls man ein Desktop möchte. 
2. Hardware-UART freigeben, Serial-Konsole deaktivieren  
3. Abhängigkeiten: `paho-mqtt`, `pigpio` / `pigpiod`  
4. MQTT-Zugang in beiden Skripten setzen, richtige PIN eintragen, vor dem Anschluss bitte prüfen ob die TX und RX Pins, die man auswählt, auf HIGH sind beim Start, sonst zieht der der PI den BUS auf Masse schon beim Start=Fehler  
5. Bei Bedarf `EXTERNAL_START_TOPIC` setzen, sonst `None`

```bash
python3 alde_mqtt.py
python3 alde_red_bus_mqtt.py

## Credits

Basiert auf dem Projekt von davidotson (Alde 3030 MQTT Bridge).
Stark erweitert: Red Bus / Aventa, Display, External Start, Manual-Schutz
und Absicherung gegen Panel-Sperren.
