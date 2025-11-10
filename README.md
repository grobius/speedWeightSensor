ESP32-C3 Skivstångssensor (Hastighet & Acceleration)

Detta projekt använder ett ESP32-C3-kort med en 0,42″ OLED-display och en BNO085 IMU (SPI) för att mäta hastighet och acceleration på en skivstång vid styrketräning.
Koden integrerar filtrerad vertikal acceleration för att uppskatta stångens hastighet, detekterar STILLASTÅENDE/RÖRLIGT beteende och visar:

Aktuell hastighet (m/s)

Maximal acceleration (amax)

Status: STILL / RÖRLIG

Hårdvara

ESP32-C3 + 0,42" OLED (I2C: SDA=5, SCL=6)

BNO085 IMU (SPI: SCK=3, MOSI=1, MISO=4, CS=7, INT=10, RST=2)

Funktioner

Vertikal hastighetsberäkning med driftskontroll

Justerbara tröskelvärden för rörelsedetektering

Tydlig 3-radig OLED-layout

Icke-blockerande uppdateringar av IMU och display


