/*********************************************************************************
 * Program for creating wireless serial bridge between 2 ESP32
 * 
 * In project 4 ESP32 boards will be used: one Mother and 3 Children
 * 
 * Only 1 Child is turned on, ready to communicate with Mother.
 * Mother chooses which Child to talk to by grounding (or pulling up?)
 * specific pin
*********************************************************************************/

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
/** 
 * BOARD1 is the one with white paper attached
 * BOARD2 looks better and has small blue dot on an antenna mounted on the board
*/



#define BOARD2 // BOARD1 or BOARD2
#define SHOW_MAC_ADDRESS

#ifdef BOARD1
#define RECVR_MAC {0x3C, 0x71, 0xBF, 0x4B, 0xF2, 0x1C}  // replace with your board's address
//#define BLINK_ON_SEND
//#define BLINK_ON_SEND_SUCCESS
#define BLINK_ON_RECV
#else
#define RECVR_MAC {0x94, 0xB9, 0x7E, 0xFA, 0xD0, 0x10}  // replace with your board's address
//#define BLINK_ON_SEND
#define BLINK_ON_SEND_SUCCESS
//#define BLINK_ON_RECV
#endif

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
// #define DEBUG // for additional serial messages (may interfere with other messages)

#ifndef LED_BUILTIN
#define LED_BUILTIN 2  // some boards don't have an LED or have it connected elsewhere
#endif

const uint8_t broadcastAddress[] = RECVR_MAC;
// wait for double the time between bytes at this serial baud rate before sending a packet
// this *should* allow for complete packet forming when using packetized serial comms
const uint32_t timeout_micros = (int)(1.0 / BAUD_RATE * 1E6) * 20;

uint8_t buf_recv[BUFFER_SIZE];
uint8_t buf_send[BUFFER_SIZE];
uint8_t buf_size = 0;
uint32_t send_timeout = 0;

esp_now_peer_info_t peerInfo;  // scope workaround for arduino-esp32 v2.0.1

#if defined(DEBUG) || defined(BLINK_ON_SEND_SUCCESS)
uint8_t led_status = 0;
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  #ifdef DEBUG
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("Send success");
  } else {
  Serial.println("Send failed");
  }
  #endif

  #ifdef BLINK_ON_SEND_SUCCESS
  if (status == ESP_NOW_SEND_SUCCESS) {
    led_status = ~led_status;
    // this function happens too fast to register a blink
    // instead, we latch on/off as data is successfully sent
    digitalWrite(LED_BUILTIN, led_status);
    return;
  }
  // turn off the LED if send fails
  led_status = 0;
  digitalWrite(LED_BUILTIN, led_status);
  #endif
}
#endif

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
 
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
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
  
  // esp_now_peer_info_t peerInfo;  // scope workaround for arduino-esp32 v2.0.1
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

void loop() {

  // read up to BUFFER_SIZE from serial port
  if (Serial.available()) {
    while (Serial.available() && buf_size < BUFFER_SIZE) {
      buf_send[buf_size] = Serial.read();
      send_timeout = micros() + timeout_micros;
      buf_size++;
    }
  }

  // send buffer contents when full or timeout has elapsed
  if (buf_size == BUFFER_SIZE || (buf_size > 0 && micros() >= send_timeout)) {
    #ifdef BLINK_ON_SEND
    digitalWrite(LED_BUILTIN, HIGH);
    #endif
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &buf_send, buf_size);
    buf_size = 0;
    #ifdef DEBUG
    if (result == ESP_OK) {
      Serial.println("Sent!");
    }
    else {
      Serial.println("Send error");
    }
    #endif
    #ifdef BLINK_ON_SEND
    digitalWrite(LED_BUILTIN, LOW);
    #endif
  }

}
