/*********************************************************************************
 * Program for creating wireless serial bridge between 2 ESP32
 * 
 * In project 4 ESP32 boards will be used: one Mother and 3 Children
 * 
 * Only 1 Child (PADA) is turned on, ready to communicate with Mother (PA).
 * Mother chooses which Child to talk to by grounding (or pulling up?)
 * specific pin
 * 
 * Boards:
 * 1) MOTHER
 * 2) CHILD1
 * 3) CHILD2
 * 4) CHILD3
 * 
 * If you want Mother to send to:
 *      CHILD1, then ground GPIO5
 *      CHILD2, then ground GPIO6
 *      CHILD3, then ground GPIO7
*********************************************************************************/

#include "Arduino.h"
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

/** Uncomment only one! */
// #define MOTHER
#define CHILD1
// #define CHILD2
// #define CHILD3

/** if I am the Mother */
#ifdef MOTHER
    #define CHILD1_PIN 5
    #define CHILD2_PIN 6
    #define CHILD3_PIN 7
    #define CHILD1_RECVR_MAC {0x94, 0xB9, 0x7E, 0xFA, 0xD0, 0x10}
    #define CHILD2_RECVR_MAC {0xff, 0xff, 0xff, 0xff, 0xff, 0x00}
    #define CHILD3_RECVR_MAC {0xff, 0xff, 0xff, 0xff, 0xff, 0x00}
    const uint8_t child1_address[] = CHILD1_RECVR_MAC;
    const uint8_t child2_address[] = CHILD2_RECVR_MAC;
    const uint8_t child3_address[] = CHILD3_RECVR_MAC;
#endif

/** if I am one of the Chilren */
#ifndef MOTHER
    #define MOTHER_RECVR_MAC {0x94, 0xB9, 0x7E, 0xFA, 0xD0, 0x10}
    const uint8_t mother_address[] = MOTHER_RECVR_MAC;
#endif

#define SHOW_MAC_ADDRESS

#define WIFI_CHAN  13 // 12-13 only legal in US in lower power mode, do not use 14

/** Perhaps I can live baudrate at 9600 for slick printing but
 * when running simulation I should use higher speed in order to 
 * meet minimum requirements when it comes to amount of data ArduPilot
 * sends each second (at least 2kB ~ 20'000 baud rate)
 */
#define BAUD_RATE  115200
// #define BAUD_RATE  9600  #up to 1200Bps

#define TX_PIN     1 // default UART0 is pin 1 (shared by USB)
#define RX_PIN     3 // default UART0 is pin 3 (shared by USB)
#define SER_PARAMS SERIAL_8N1 // SERIAL_8N1: start/stop bits, no parity

#define BUFFER_SIZE 250 // max of 250 bytes

// wait for double the time between bytes at this serial baud rate before sending a packet
// this *should* allow for complete packet forming when using packetized serial comms
const uint32_t timeout_micros = (int)(1.0 / BAUD_RATE * 1E6) * 20;

uint8_t buf_recv[BUFFER_SIZE];
uint8_t buf_send[BUFFER_SIZE];
uint8_t buf_size = 0;
uint32_t send_timeout = 0;

esp_now_peer_info_t peerInfo;  // scope workaround for arduino-esp32 v2.0.1

static uint8_t broadcastAddress[6] = {};

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    #ifdef BLINK_ON_RECV
    digitalWrite(LED_BUILTIN, HIGH);
    #endif

    memcpy(&buf_recv, incomingData, sizeof(buf_recv));
    Serial.write(buf_recv, len);
    
    #ifdef BLINK_ON_RECV
    digitalWrite(LED_BUILTIN, LOW);
    #endif
    
    #ifdef DEBUG
    Serial.print("\n Bytes received: ");
    Serial.println(len);
    #endif
}


void setup()
{
    /* I am the mother*/
    #ifdef MOTHER
    pinMode(5, INPUT_PULLUP);
    pinMode(6, INPUT_PULLUP);
    pinMode(7, INPUT_PULLUP);

    if (digitalRead(CHILD1_PIN) == LOW)
    {   // child one has been chosen
        memcpy(broadcastAddress, child1_address, sizeof(child1_address));
    }
    else if (digitalRead(CHILD2_PIN) == LOW)
    {   // child two has been chosen
        memcpy(broadcastAddress, child2_address, sizeof(child2_address));
    }
    else if (digitalRead(CHILD3_PIN) == LOW)
    {   // child two has been chosen
        memcpy(broadcastAddress, child3_address, sizeof(child3_address));
    }
    else
    {
        Serial.println("Error: None of the pins is grounded!");
    }
    #endif

    /* I am one of the Children*/
    #ifndef MOTHER
    memcpy(broadcastAddress, mother_address, sizeof(mother_address));
    #endif

    Serial.begin(BAUD_RATE, SER_PARAMS, RX_PIN, TX_PIN);
    Serial.println(send_timeout);
    WiFi.mode(WIFI_STA);

    #ifdef SHOW_MAC_ADDRESS
    Serial.print("ESP32 MAC Address: ");
    Serial.println(WiFi.macAddress());
    #endif

        if (esp_wifi_set_channel(WIFI_CHAN, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
        #ifdef DEBUG
        Serial.println("Error changing WiFi channel");
        #endif
        return;
    }

    if (esp_now_init() != ESP_OK) {
        #ifdef DEBUG
        Serial.println("Error initializing ESP-NOW");
        #endif
        return;
    }

    #if defined(DEBUG) || defined(BLINK_ON_SEND_SUCCESS)
    esp_now_register_send_cb(OnDataSent);
    #endif
    
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = WIFI_CHAN;  
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        #ifdef DEBUG
        Serial.println("Failed to add peer");
        #endif
        return;
    }

    esp_now_register_recv_cb(OnDataRecv);
}

void loop()
{
    // read up to BUFFER_SIZE from serial port
    if (Serial.available())
    {
        while (Serial.available() && buf_size < BUFFER_SIZE)
        {
            buf_send[buf_size] = Serial.read();
            send_timeout = micros() + timeout_micros;
            buf_size++;
        }
    }

    // send buffer contents when full or timeout has elapsed
    if (buf_size == BUFFER_SIZE || (buf_size > 0 && micros() >= send_timeout))
    {
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &buf_send, buf_size);
        buf_size = 0;
    }
}
