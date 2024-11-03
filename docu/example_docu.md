# Autonom Fahrendes Auto

In diesem Projekt beschäftigen wir uns mit der Herangehensweise und den Herausforderungen des autonomen Fahrens. Dabei wird ein Auto gebaut, dass einen Parcours von Start bis Ziel selbstständig durchfährt ohne menschliches Eingreifen.

## Fahrzeug Aufbau

Beim Aufbau müssen das Chassis, Motoren + Reifen, Motortreiber, Arduino inklusive Sensorshield, Sensoren und Stromversorgung entsprechend verkabelt werden.

Hier soll ein Bild eures skizzierter Schaltplans und ein Foto des tatsächlichen Aufbaus hinein:
<!--- ![Bild mit eurer Skizze sowie Foto des Aufbaus](./pics/symbolabbildung-skizze-foto.png) -->
<img src="./pics/symbolabbildung-skizze-foto.png" alt="Bild mit eurer Skizze sowie Foto des Aufbaus" width="600"/>

## Sensorerfassung

Bei den Sharp-Sensoren handelt es sich um Infrarotsensoren nach dem ... Prinzip ...

...

Folgende Tabelle beinhaltet die Messung des Front-Sensors:

| Länge | ADC Wert | Umkehrwert | 1 / (l+k) | m*ADC+d |
|-------|----------|------------|-----------|---------|
| 20    | 510      | 0,00196    | 0,03571   | 0,03277 |
| 30    | 405      | 0,00247    | 0,02632   | 0,02632 |
| ...   | ...      | .......    | .......   | ....... |
| 150   | 95       | 0,01053    | 0,00725   | 0,00725 |

Daraus ergeben sich folgende Parameter ...

Dies resultiert in folgendem Code zur richtigen Kalibrierung des Sensors ...

```c
// read IR sensor data
ir_sensor_front_raw = analogRead(IR_SENSOR_FRONT);
ir_sensor_front_new = (uint16_t) (16256.4 / (ir_sensor_front_raw + 22.8)) - 8;

if(ir_sensor_front_new > 150)
	ir_sensor_front_new = 151;
else if(ir_sensor_front_new < 20)
       	ir_sensor_front_new = 19;
```

## Fahrstrategie

### Seitenregelung

### Mittenregelung

## Optimierungen

## Gesammelte Erfahrungen
