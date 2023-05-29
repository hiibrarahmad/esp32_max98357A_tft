# 1 "C:\\Users\\FA19-B~1.CUI\\AppData\\Local\\Temp\\tmp_hqgsh74"
#include <Arduino.h>
# 1 "E:/fyp/esp32_max98357A_tft/src/main.ino"
#include <SPI.h>
#include <ArduinoWebsockets.h>
#include <WiFi.h>
#include <TJpg_Decoder.h>
#include <TFT_eSPI.h>
#include <ezButton.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2s.h"

const char* ssid = "ibrar";
const char* password = "ibrarahmad";

using namespace websockets;

WebsocketsServer server;
WebsocketsClient client;

SemaphoreHandle_t connection_established = NULL;
SemaphoreHandle_t button_semaphore = NULL;
TaskHandle_t ws_comm_task = NULL;
TaskHandle_t button_task = NULL;



unsigned long last_update_sent = 0;
bool isAudioPlaying = false;

TFT_eSPI tft = TFT_eSPI();
# 42 "E:/fyp/esp32_max98357A_tft/src/main.ino"
void ws_comm(void* arg);
void button_task_func(void* arg);
bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap);
void setup();
void loop();
#line 50 "E:/fyp/esp32_max98357A_tft/src/main.ino"
bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap)
{

  if (y >= tft.height()) return 0;


  tft.pushImage(x, y, w, h, bitmap);


  return 1;
}

ezButton button(26);

bool buttonPressed = false;
unsigned long lastButtonPressTime = 0;
const unsigned long buttonCooldownTime = 5000;
# 86 "E:/fyp/esp32_max98357A_tft/src/main.ino"
void setup() {
  Serial.begin(115200);
  delay(1000);
# 99 "E:/fyp/esp32_max98357A_tft/src/main.ino"
  tft.begin();
  tft.setRotation(3);
  tft.setTextColor(0xFFFF, 0x0000);
  tft.fillScreen(TFT_RED);
  tft.setSwapBytes(true);


  TJpgDec.setJpgScale(1);


  TJpgDec.setCallback(tft_output);

  Serial.println();
  Serial.println("Setting AP...");
  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP Address: ");
  Serial.println(IP);

  server.listen(8888);

  button.setDebounceTime(50);

  connection_established = xSemaphoreCreateBinary();
  button_semaphore = xSemaphoreCreateBinary();

  xTaskCreatePinnedToCore(
   ws_comm,
   "websocket server",
   4096, NULL,
   5,
   &ws_comm_task,
   0);


  xTaskCreatePinnedToCore(
    button_task_func,
    "button task",
    4096,
    NULL,
    5,
    &button_task,
    0);


}

void loop() {
  if (server.poll()) {
    client = server.accept();
    xSemaphoreGive(connection_established);
  }


}

void button_task_func(void* arg) {
  while (1) {
     Serial.println("button task core");
    Serial.println(xPortGetCoreID());
    button.loop();

    if (button.isPressed() && !buttonPressed && (millis() - lastButtonPressTime >= buttonCooldownTime)) {
      buttonPressed = true;
      lastButtonPressTime = millis();


      if (client.available()) {
        client.send("Button pressed!");
        Serial.println("Button pressed");


        tft.setCursor(0, 0);
        tft.println("Button pressed");
      }

      xSemaphoreGive(button_semaphore);
    }

    if (buttonPressed && (millis() - lastButtonPressTime >= buttonCooldownTime)) {
      buttonPressed = false;
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}
# 219 "E:/fyp/esp32_max98357A_tft/src/main.ino"
void ws_comm(void* arg) {
  while (1) {
    if (xSemaphoreTake(connection_established, portMAX_DELAY) == pdTRUE) {
      while (client.available()) {
        WebsocketsMessage msg = client.readBlocking();

        Serial.println("pic task core");
        Serial.println(xPortGetCoreID());

        uint32_t t = millis();


        uint16_t w = 0, h = 0;
        TJpgDec.getJpgSize(&w, &h, (const uint8_t*)msg.c_str(), msg.length());
        Serial.print("Width = ");
        Serial.print(w);
        Serial.print(", height = ");
        Serial.println(h);


        TJpgDec.drawJpg(0, 0, (const uint8_t*)msg.c_str(), msg.length());


        t = millis() - t;
        Serial.print(t);
        Serial.println(" ms");


        if (xSemaphoreTake(button_semaphore, 0) == pdTRUE) {
          client.send("Button pressed!");
          Serial.println("Button pressed");
        }

        vTaskDelay(pdMS_TO_TICKS(20));
      }
    }
  }
}