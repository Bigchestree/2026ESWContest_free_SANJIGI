#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>

#define LORA_NSS   41
#define LORA_BUSY  40
#define LORA_NRST  42
#define LORA_DIO1  39

#define LORA_SCK    7
#define LORA_MISO   8
#define LORA_MOSI   9

SPIClass loraSPI(FSPI);

SX1262 radio = new Module(
    LORA_NSS,
    LORA_DIO1,
    LORA_NRST,
    LORA_BUSY,
    loraSPI
);

volatile bool receivedFlag = false;

void setFlag() {
    receivedFlag = true;
}

void setup() {

    Serial.begin(115200);
    delay(3000);

    Serial.println("MASTER TEST START");

    loraSPI.begin(
        LORA_SCK,
        LORA_MISO,
        LORA_MOSI,
        LORA_NSS
    );

    int state = radio.begin(
        923.0,
        125.0,
        9,
        7,
        0x12,
        10,
        8,
        1.8,
        false
    );

    Serial.print("LoRa begin = ");
    Serial.println(state);

    if (state != RADIOLIB_ERR_NONE) {

        Serial.println("LORA INIT FAIL");
        return;
    }

    radio.setDio2AsRfSwitch(true);

    radio.setPacketReceivedAction(setFlag);

    state = radio.startReceive();

    Serial.print("RX START = ");
    Serial.println(state);

    Serial.println("WAITING...");
}

void loop() {

    if (!receivedFlag) {
        return;
    }

    receivedFlag = false;

    String msg;

    int state = radio.readData(msg);

    Serial.println();
    Serial.println("----------------");

    Serial.print("RX STATE : ");
    Serial.println(state);

    Serial.print("DATA     : ");
    Serial.println(msg);

    Serial.print("RSSI     : ");
    Serial.println(radio.getRSSI());

    Serial.print("SNR      : ");
    Serial.println(radio.getSNR());

    Serial.println("----------------");

    radio.startReceive();
}