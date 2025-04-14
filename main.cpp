// Single Channel LoRa Gateway for SX1276
// Adapted from Thomas Telkamp's original version

#include <string>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
#include <iostream>
#include <cstdlib>
#include <sys/time.h>
#include <cstring>
#include <sys/ioctl.h>
#include <net/if.h>

#include "base64.h"

#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <mosquitto.h>


typedef bool boolean;
typedef unsigned char my_byte; // Renamed from byte

static const int CHANNEL = 0;

my_byte currentMode = 0x81;

char message[256];
char b64[256];
int receivedbytes;
bool sx1272 = true;

struct mosquitto *mosq = nullptr;

struct sockaddr_in si_other;
int s, slen = sizeof(si_other);
struct ifreq ifr;

uint32_t cp_nb_rx_rcv;
uint32_t cp_nb_rx_ok;
uint32_t cp_nb_rx_bad;
uint32_t cp_nb_rx_nocrc;
uint32_t cp_up_pkt_fwd;

enum sf_t { SF7 = 7, SF8, SF9, SF10, SF11, SF12};

int ssPin = 6;
int dio0 = 7;
int RST = 0;
sf_t sf = SF7;
uint32_t freq = 868100000;
float lat = 0.0;
float lon = 0.0;
int alt = 0;

static char platform[24] = "Single Channel Gateway";
static char email[40] = "";
static char description[64] = "";

#define SERVER1 "54.72.145.119"
#define PORT 1700

#define REG_FIFO 0x00
#define REG_FIFO_ADDR_PTR 0x0D
#define REG_FIFO_TX_BASE_AD 0x0E
#define REG_FIFO_RX_BASE_AD 0x0F
#define REG_RX_NB_BYTES 0x13
#define REG_OPMODE 0x01
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS 0x12
#define REG_DIO_MAPPING_1 0x40
#define REG_DIO_MAPPING_2 0x41
#define REG_MODEM_CONFIG 0x1D
#define REG_MODEM_CONFIG2 0x1E
#define REG_MODEM_CONFIG3 0x26
#define REG_SYMB_TIMEOUT_LSB 0x1F
#define REG_PKT_SNR_VALUE 0x19
#define REG_PAYLOAD_LENGTH 0x22
#define REG_IRQ_FLAGS_MASK 0x11
#define REG_MAX_PAYLOAD_LENGTH 0x23
#define REG_HOP_PERIOD 0x24
#define REG_SYNC_WORD 0x39
#define REG_VERSION 0x42
#define REG_FRF_MSB 0x06
#define REG_FRF_MID 0x07
#define REG_FRF_LSB 0x08
#define REG_LNA 0x0C

#define SX72_MODE_RX_CONTINUOS 0x85
#define SX72_MODE_TX 0x83
#define SX72_MODE_SLEEP 0x80
#define SX72_MODE_STANDBY 0x81

#define PAYLOAD_LENGTH 0x40

#define LNA_MAX_GAIN 0x23
#define LNA_OFF_GAIN 0x00
#define LNA_LOW_GAIN 0x20

#define TX_BUFF_SIZE 2048
#define STATUS_SIZE 1024
#define PROTOCOL_VERSION 1
#define PKT_PUSH_DATA 0
#define PKT_PUSH_ACK 1
#define PKT_PULL_DATA 2
#define PKT_PULL_RESP 3
#define PKT_PULL_ACK 4

void die(const char *s) {
    perror(s);
    exit(1);
}

void selectreceiver() {
    digitalWrite(ssPin, LOW);
}

void unselectreceiver() {
    digitalWrite(ssPin, HIGH);
}

my_byte readRegister(my_byte addr) {
    unsigned char spibuf[2];
    selectreceiver();
    spibuf[0] = addr & 0x7F;
    spibuf[1] = 0x00;
    wiringPiSPIDataRW(CHANNEL, spibuf, 2);
    unselectreceiver();
    return spibuf[1];
}

void writeRegister(my_byte addr, my_byte value) {
    unsigned char spibuf[2];
    spibuf[0] = addr | 0x80;
    spibuf[1] = value;
    selectreceiver();
    wiringPiSPIDataRW(CHANNEL, spibuf, 2);
    unselectreceiver();
}

boolean receivePkt(char *payload) {
    writeRegister(REG_IRQ_FLAGS, 0x40);
    int irqflags = readRegister(REG_IRQ_FLAGS);
    cp_nb_rx_rcv++;
    if ((irqflags & 0x20) == 0x20) {
        printf("CRC error\n");
        writeRegister(REG_IRQ_FLAGS, 0x20);
        return false;
    } else {
        cp_nb_rx_ok++;
        my_byte currentAddr = readRegister(REG_FIFO_RX_CURRENT_ADDR);
        my_byte receivedCount = readRegister(REG_RX_NB_BYTES);
        receivedbytes = receivedCount;
        writeRegister(REG_FIFO_ADDR_PTR, currentAddr);
        for (int i = 0; i < receivedCount; i++) {
            payload[i] = (char)readRegister(REG_FIFO);
        }
    }
    return true;
}

void SetupLoRa() {
    digitalWrite(RST, HIGH); delay(100);
    digitalWrite(RST, LOW); delay(100);

    my_byte version = readRegister(REG_VERSION);
    if (version == 0x22) {
        printf("SX1272 detected, starting.\n");
        sx1272 = true;
    } else {
        digitalWrite(RST, LOW); delay(100);
        digitalWrite(RST, HIGH); delay(100);
        version = readRegister(REG_VERSION);
        if (version == 0x12) {
            printf("SX1276 detected, starting.\n");
            sx1272 = false;
        } else {
            printf("Unrecognized transceiver.\n");
            exit(1);
        }
    }

    writeRegister(REG_OPMODE, SX72_MODE_SLEEP);

    uint64_t frf = ((uint64_t)freq << 19) / 32000000;
    writeRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
    writeRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
    writeRegister(REG_FRF_LSB, (uint8_t)(frf >> 0));

    writeRegister(REG_SYNC_WORD, 0x34);

    if (sx1272) {
        writeRegister(REG_MODEM_CONFIG, (sf == SF11 || sf == SF12) ? 0x0B : 0x0A);
        writeRegister(REG_MODEM_CONFIG2, (sf << 4) | 0x04);
    } else {
        writeRegister(REG_MODEM_CONFIG3, (sf == SF11 || sf == SF12) ? 0x0C : 0x04);
        writeRegister(REG_MODEM_CONFIG, 0x72);
        writeRegister(REG_MODEM_CONFIG2, (sf << 4) | 0x04);
    }

    writeRegister(REG_SYMB_TIMEOUT_LSB, (sf >= SF10) ? 0x05 : 0x08);
    writeRegister(REG_MAX_PAYLOAD_LENGTH, 0x80);
    writeRegister(REG_PAYLOAD_LENGTH, PAYLOAD_LENGTH);
    writeRegister(REG_HOP_PERIOD, 0xFF);
    writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_BASE_AD));
    writeRegister(REG_LNA, LNA_MAX_GAIN);
    writeRegister(REG_OPMODE, SX72_MODE_RX_CONTINUOS);
}

void mqtt_publish(const char* topic, const char* payload) {
    mosquitto_publish(mosq, nullptr, topic, strlen(payload), payload, 0, false);
}

int main() {
    wiringPiSetup();
    pinMode(ssPin, OUTPUT);
    pinMode(dio0, INPUT);
    pinMode(RST, OUTPUT);
    wiringPiSPISetup(CHANNEL, 500000);
    SetupLoRa();
    printf("Gateway ready on SX1276 with SF%i at %.1f MHz\n", sf, freq / 1e6);
    mosquitto_lib_init();
    mosq = mosquitto_new(nullptr, true, nullptr);
    if (!mosq || mosquitto_connect(mosq, "localhost", 1883, 60)) {
        fprintf(stderr, "Failed to connect to MQTT broker.\n");
        return 1;
    }
    if (mosquitto_connect(mosq, "localhost", 1883, 60) != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "Could not connect to MQTT broker.\n");
        return 1;
    }
    while (true) {
        if (digitalRead(dio0) == 1) {
            if (receivePkt(message)) {
                printf("Received: %.*s\n", receivedbytes, message);
                message[receivedbytes] = '\0';
                char* token = strtok(message, ",");
                if (token != nullptr) {
                  mqtt_publish("lora/temperature", token);
                  token = strtok(nullptr, ",");
                  if (token != nullptr) {
                    mqtt_publish("lora/humidity", token);
                    }
                }
            }
        }
        mosquitto_loop(mosq, 0, 1);
        delay(10);
    }
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    return 0;
}