# esp-multical21
ESP8266 decrypts wireless MBus frames from a Multical21 water meter

A CC1101 868 MHz modul is connected via SPI to the ESP8266 an configured to receive Wireless MBus frames. The Multical21 is sending every 16 seconds wireless MBus frames (Mode C1, frame type B). The frames are encrypted and the ESP decrypts them with AES-128-CTR. The meter information (total counter, target counter, medium temperature, ambient temperature, alalm flags (BURST, LEAK, DRY, REVERSE) are sent via MQTT to a smarthomeNG/smartVISU service (running on a raspberry).
