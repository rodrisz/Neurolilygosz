Proyecto Neurowatch — T-Watch 2020 V2 con MindWave Mobile

Programa principal: src/Neurowatch/main.cpp
Plataforma: TTGO T-Watch 2020 V2 (ESP32)
Upload port: COM13
Libreria hardware: TTGO_TWatch_Library-master (en lib/)

Hardware del reloj:
- Pantalla TFT 240x240 touch
- DRV2605L: motor haptico, comunicacion I2C, controlado via watch->drv
- AXP202: gestion de energia (bateria, carga) via watch->power
- BMA423: acelerometro 3 ejes (pasos, inclinacion) via watch->bma
- PCF8563: reloj RTC
- GPS integrado
- Slot microSD

Motor DRV2605L:
- NO usar pinMode/digitalWrite — usa watch->drv->setWaveform() y watch->drv->go()
- Efecto 52: Pulsing Strong 1 100% — para umbral de meditacion alcanzado
- Efecto 15: 750ms Alert 100% — para meditacion = 100
- Para vibrar: setWaveform(0, efecto), setWaveform(1, 0), go()
- Para detener: stop()

Neurowatch — descripcion:
Sistema de neurofeedback con diadema MindWave Mobile (Bluetooth SPP).
Muestra en pantalla: atencion, meditacion, indices IFN y RC, bandas EEG.
Menu con botones 70/80/90 para umbral de vibracion.
Vibra con motor haptico cuando meditacion >= umbral elegido.
Meditacion = 100 vibra 3 segundos intermitente cada 300ms (sin delay, usa millis).

Bluetooth: MindWave conecta por SPP a SerialBT, protocolo ThinkGear ASIC.
Pantalla conexion: muestra NEURO-CRAWLER y estado de conexion BT.
Panel principal: atencion, meditacion, IFN, RC, bandas EEG, boton Menu.
Pantalla menu: botones umbral 70/80/90, estado bateria (accesible solo desde panel principal).

Gapaxionsz es un sisitema creado por RodriSZ, con este sistema se busca entrenar los espacios de vacio mental entre pensamiento y pensamiento, utilizando ejercicios de Mindfulness meditacion, respiración, bodymindfulness, meditación en movimiento, cambios de estado variando la tasa de pensamientos en un intervalo de tiempo, Gapaxion se podría decir que es la brecha entre pensamiento y pensamiento en donde podemos realizar una accion, sea accion mental o fisica, movimiento o estrucutra mental de sanacion, como los engranajes de observador. 
