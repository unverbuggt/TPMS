# TPMS External Tire Pressure Monitoring System 433 MHz (CC1101) / Bluetooth BLE (ESP32)

Two pressure sensors (direct external tpms) have been investigated. One model transmits via 433 MHz, the other via Bluetooth BLE.

## 433 MHz
Model https://aliexpress.com/item/1005004439177734.html

Signals are received with rtl_sdr and SDR dongle as protocol 201 "tpms_truck":
https://github.com/merbanan/rtl_433/blob/master/src/devices/tpms_truck.c

Previous work has been anticipated:

https://www.youtube.com/watch?app=desktop&v=stQPjNI7_DA

https://www.hackster.io/jsmsolns/arduino-tpms-tyre-pressure-display-b6e544

Goal was to receive data with ESP8266 and CC1101 and based on RadioLib:

https://jgromes.github.io/RadioLib/

Bitrate of 19200 baud and sync word 0x001a have shown to work fine. Sensors could NOT be activated with trigger tool OEC-T5 EL-50448. Once pressurized they transmit when pressure changes and about once an hour with constant non-zero pressure. Rotation of a bicycle wheel could not trigger transmission.

With RadioLib godmode has been enabled in order to address CC1101 registers directly, since the library does not allow for sync = 0x00.

## Bluetooth BLE
Model https://aliexpress.com/item/1005004504977890.html

Signals are received with SYTPMS app.

Previous work has been anticipated:

https://www.instructables.com/BLE-Direct-Tire-Pressure-Monitoring-System-TPMS-Di/

https://github.com/ra6070/BLE-TPMS

https://forum.arduino.cc/t/arduino-ble-tpms-capteur-pression-pneus/592030/60

https://raspberrypi.stackexchange.com/questions/76959/how-to-decode-tpms-sensor-data-through-rpi3-bluetooth

With BLE scanner name "BR", service "0x27a5" (pressure psi) and manufacturer data have been received as advertising data. The 7 byte manufacturer data seem to contain sensor readings with format SSBBTTPPPPCCCC:

|field|content|
|---:|-------|
|SS|status|
|BB|battery 1/10 V|
|TT|temperature degC|
|PPPP|pressure 1/10 psi|
|CCCC|checksum|

Status byte SS: ARSB2H1y

|bit|content|
|--:|-------|
|A|alarm zero pressure|
|R|rotating|
|S|standing still for about 15min|
|B|begin rotating|
|2|decreasing pressure below 20.7 psi|
|H|rising pressure|
|1|decreasing pressure above 20.7 psi|
|y|unknown|

Once pressurized they transmit when pressure changes and any couple of minutes. Rotation of a bicycle wheel corresponding to about 4 to 8 km/h speed triggered more frequent transmissions.

Advertising Data Payload: Short UUID (16 bit) "0x27a5" (byte 1 to 4), name "BR" (byte 5 to 8), and manufacturer data including checksum (byte 9 to 17) as shown above.

Sensor data could be received with ESP32 and BLE library.
