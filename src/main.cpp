/*********************************************************************************
 * Program for creating wireless serial bridge between 2 ESP32
 * 
 * In project 4 ESP32 boards will be used: one Mother and 3 Children
 * 
 * Only 1 Child is turned on, ready to communicate with Mother.
 * Mother chooses which Child to talk to by grounding (or pulling up?)
 * specific pin
 * 
 * FOLLOW 2 STEPS BELOW!
 * Select if you are the Mother or the Child!
 * If you are mother just uncomment `#define MOTHER` and give value 1 or 2 (does not rly matter here which one)
 * If you are child, uncomment `#define CHILD` and give value  1, 2 or 3 (depends what's written on the board)
 *  and select which mother is in use (in second step) (also, look at the mother's board to know the number)
*********************************************************************************/

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "mac_addresses.h"

/********************************** STEP 1/2 **********************************/
// #define MOTHER 1
#define CHILD 2
/******************************************************************************/
#ifdef CHILD 
#define I_AM_CHILD
/********************************** STEP 2/2 **********************************/
#define MOTHER_USED 1
/******************************************************************************/
#endif

#ifdef MOTHER 
#define I_AM_MOTHER
#endif

// #define SHOW_MAC_ADDRESS
// #define DEBUG // for additional serial messages (may interfere with other messages)


/** if I am the Mother */
#ifdef I_AM_MOTHER
        #define CHILD1_PIN 19
        #define CHILD2_PIN 18
        #define CHILD3_PIN 5
        const uint8_t child1_address[] = MAC_CHILD1;
        const uint8_t child2_address[] = MAC_CHILD2;
        const uint8_t child3_address[] = MAC_CHILD3;
#endif

/** if I am one of the Children */
#ifdef I_AM_CHILD
    #if MOTHER_USED == 1
        #define MOTHER_RECVR_MAC MAC_MOTHER1
        const uint8_t mother_address[] = MOTHER_RECVR_MAC;
    #elif MOTHER_USED == 2
        #define MOTHER_RECVR_MAC MAC_MOTHER2
        const uint8_t mother_address[] = MOTHER_RECVR_MAC;
    #endif
#endif





#define WIFI_CHAN  13 // 12-13 only legal in US in lower power mode, do not use 14

/** Perhaps I can live baudrate at 9600 for slick printing but
 * when running simulation I should use higher speed in order to 
 * meet minimum requirements when it comes to amount of data ArduPilot
 * sends each second (at least 2kB ~ 20'000 baud rate)
 */
#define BAUD_RATE  115200
// #define BAUD_RATE  9600  //#up to 1200Bps
#define TX_PIN     1 // default UART0 is pin 1 (shared by USB)
#define RX_PIN     3 // default UART0 is pin 3 (shared by USB)
#define SER_PARAMS SERIAL_8N1 // SERIAL_8N1: start/stop bits, no parity

#define BUFFER_SIZE 250 // max of 250 bytes




uint8_t broadcastAddress[6];

// wait for double the time between bytes at this serial baud rate before sending a packet
// this *should* allow for complete packet forming when using packetized serial comms
const uint32_t timeout_micros = (int)(1.0 / BAUD_RATE * 1E6) * 20;

uint8_t buf_recv[BUFFER_SIZE];
uint8_t buf_send[BUFFER_SIZE];
uint8_t buf_size = 0;
uint32_t send_timeout = 0;

esp_now_peer_info_t peerInfo;  // scope workaround for arduino-esp32 v2.0.1


void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len)
{
    memcpy(&buf_recv, incomingData, sizeof(buf_recv));
    Serial.write(buf_recv, len);
    #ifdef DEBUG
    Serial.print("\n Bytes received: ");
    Serial.println(len);
    #endif
}
 
void setup()
{
    Serial.begin(BAUD_RATE, SER_PARAMS, RX_PIN, TX_PIN);
    Serial.println(send_timeout);
    WiFi.mode(WIFI_STA);

    #ifdef SHOW_MAC_ADDRESS
    /* Show MAC Adress */
    Serial.print("ESP32 MAC Address: ");
    Serial.println(WiFi.macAddress());
    #endif

    #if defined(I_AM_MOTHER)
    pinMode(CHILD1_PIN, INPUT_PULLUP);
    pinMode(CHILD2_PIN, INPUT_PULLUP);
    pinMode(CHILD3_PIN, INPUT_PULLUP);
    if (digitalRead(CHILD1_PIN) == LOW)
    {   // child one has been chosen
        memcpy(broadcastAddress, child1_address, sizeof(child1_address));
        #ifdef DEBUG
        Serial.println("Added peer: CHILD1");
        #endif
    }
    else if (digitalRead(CHILD2_PIN) == LOW)
    {   // child two has been chosen
        memcpy(broadcastAddress, child2_address, sizeof(child2_address));
        #ifdef DEBUG
        Serial.println("Added peer: CHILD2");
        #endif
    }
    else if (digitalRead(CHILD3_PIN) == LOW)
    {   // child two has been chosen
        memcpy(broadcastAddress, child3_address, sizeof(child3_address));
        #ifdef DEBUG
        Serial.println("Added peer: CHILD3");
        #endif
    }
    else
    {
        Serial.println("Error: None of the pins is grounded!");
    }
    #elif defined(I_AM_CHILD)
    memcpy(broadcastAddress, mother_address, sizeof(mother_address));
    #endif

    if (esp_wifi_set_channel(WIFI_CHAN, WIFI_SECOND_CHAN_NONE) != ESP_OK)
    {
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