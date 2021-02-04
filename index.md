## LQrssi
Feb 2021

![LQrssi logo](https://github.com/strgaltdel/3in1/img/LQrssi.png)


### Einführung

"LQrssi" ist ein Sensor zur Überwachung der Empfangsqualität via SPort
Er basiert auf den LinkQuality Sensor von RealTadango und erweitert dessen Telemetriesensoren.

Basierend auf der Arduino  Platform können RC Hobbyisten folgende Werte von Modellen per Telemetrie uebertragen:

- 5100 (5110)   der klassische Tadango LQ Sensor (Lost Frame Indikator)
- 5102 (5112)   die Summe an Lost Frames seit Einschalten
- 5104 (5114)   die Summe an Failsafe Frames seit Einschalten
-      (5116)   den RSSI Wert von Empfängern, die diesen als Kanalwert auf den SBUS ausgeben können


  
### Hardware
  
  * µController
	* Arduino nano / mini 

  * SBUS Inverter

  Pinning:
  SBUS  >> Arduino Rx
  SPort >> Arduino Pin 4
  
  
  






