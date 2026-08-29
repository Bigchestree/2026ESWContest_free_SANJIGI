#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// =====================================================
// LoRa PIN
// =====================================================
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

// =====================================================
// LoRa RX FLAG
// =====================================================
volatile bool receivedFlag = false;

void setFlag() {
    receivedFlag = true;
}

// =====================================================
// BLE UUID
// 기존 HTML과 동일
// =====================================================
#define SERVICE_UUID   "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_CMD_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHAR_DATA_UUID "a3c17822-1d5b-4176-a447-0624916a0487"

BLECharacteristic* pDataChar = nullptr;

bool deviceConnected = false;

// =====================================================
// BLE SERVER CALLBACK
// =====================================================
class ServerCallbacks : public BLEServerCallbacks {

    void onConnect(BLEServer* pServer) override {

        deviceConnected = true;

        Serial.println("[BLE] CONNECTED");
    }

    void onDisconnect(BLEServer* pServer) override {

        deviceConnected = false;

        Serial.println("[BLE] DISCONNECTED");

        BLEDevice::startAdvertising();
    }
};

// =====================================================
// BLE COMMAND CALLBACK
// 아직 MEASURE 기능은 넣지 않음
// 수신 여부만 확인
// =====================================================
class CommandCallbacks : public BLECharacteristicCallbacks {

    void onWrite(BLECharacteristic* pChar) override {

        String value =
            pChar->getValue().c_str();

        Serial.print("[BLE CMD] ");
        Serial.println(value);
    }
};

// =====================================================
// BLE Notify
// =====================================================
void sendBle(String msg) {

    if (
        deviceConnected &&
        pDataChar != nullptr
    ) {

        pDataChar->setValue(
            msg.c_str()
        );

        pDataChar->notify();

        Serial.print("[BLE TX] ");
        Serial.println(msg);
    }
}

// =====================================================
// SETUP
// =====================================================
void setup() {

    Serial.begin(115200);
    delay(3000);

    Serial.println();
    Serial.println("=========================");
    Serial.println("MASTER LoRa + BLE TEST");
    Serial.println("=========================");

    // =================================================
    // SPI
    // =================================================
    loraSPI.begin(
        LORA_SCK,
        LORA_MISO,
        LORA_MOSI,
        LORA_NSS
    );

    Serial.println("[SPI] OK");

    // =================================================
    // LoRa
    // =================================================
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

    Serial.print("[LoRa begin] ");
    Serial.println(state);

    if (
        state !=
        RADIOLIB_ERR_NONE
    ) {

        Serial.println(
            "[FAIL] LoRa Init"
        );

        return;
    }

    radio.setDio2AsRfSwitch(
        true
    );

    radio.setPacketReceivedAction(
        setFlag
    );

    state =
        radio.startReceive();

    Serial.print(
        "[RX START] "
    );

    Serial.println(
        state
    );

    // =================================================
    // BLE
    // =================================================
    Serial.println(
        "[BLE] START"
    );

    BLEDevice::init(
        "Master_Rescue_Node"
    );

    BLEServer* pServer =
        BLEDevice::createServer();

    pServer->setCallbacks(
        new ServerCallbacks()
    );

    BLEService* pService =
        pServer->createService(
            SERVICE_UUID
        );

    // BLE COMMAND
    BLECharacteristic* pCmdChar =
        pService->createCharacteristic(
            CHAR_CMD_UUID,
            BLECharacteristic::PROPERTY_WRITE
        );

    pCmdChar->setCallbacks(
        new CommandCallbacks()
    );

    // BLE DATA
    pDataChar =
        pService->createCharacteristic(
            CHAR_DATA_UUID,
            BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_NOTIFY
        );

    pDataChar->addDescriptor(
        new BLE2902()
    );

    pService->start();

    BLEAdvertising* pAdvertising =
        BLEDevice::getAdvertising();

    pAdvertising->addServiceUUID(
        SERVICE_UUID
    );

    pAdvertising->start();

    Serial.println(
        "[BLE] READY"
    );

    Serial.println(
        "WAITING FOR TARGET..."
    );
}

// =====================================================
// LOOP
// =====================================================
void loop() {

    if (receivedFlag) {

        receivedFlag = false;

        String msg;

        int state =
            radio.readData(
                msg
            );

        if (
            state ==
            RADIOLIB_ERR_NONE
        ) {

            float rssi =
                radio.getRSSI();

            float snr =
                radio.getSNR();

            Serial.println();
            Serial.println(
                "------------------------"
            );

            Serial.print(
                "DATA : "
            );

            Serial.println(
                msg
            );

            Serial.print(
                "RSSI : "
            );

            Serial.println(
                rssi
            );

            Serial.print(
                "SNR  : "
            );

            Serial.println(
                snr
            );

            Serial.println(
                "------------------------"
            );

            // BLE로 그대로 전달
            sendBle(msg);

        } else {

            Serial.print(
                "[RX ERROR] "
            );

            Serial.println(
                state
            );
        }

        // 다음 LoRa 패킷 대기
        int rxState =
            radio.startReceive();

        if (
            rxState !=
            RADIOLIB_ERR_NONE
        ) {

            Serial.print(
                "[RX RESTART FAIL] "
            );

            Serial.println(
                rxState
            );
        }
    }
}