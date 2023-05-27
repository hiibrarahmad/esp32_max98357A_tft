#include <SPI.h>
#include <ArduinoWebsockets.h>
#include <WiFi.h>
#include <TJpg_Decoder.h>
#include <TFT_eSPI.h>
#include <ezButton.h>
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

TFT_eSPI tft = TFT_eSPI(); // Invoke custom library

#define I2S_WS_TX  12
#define I2S_SCK_TX 13
#define I2S_DATA_OUT_TX  15

#define I2S_PORT I2S_NUM_0
#define I2S_SAMPLE_RATE   (16000)
#define I2S_SAMPLE_BITS   (32)
#define UPDATE_INTERVAL   (500)

ezButton button(26); // Pin for the button

bool buttonPressed = false;
unsigned long lastButtonPressTime = 0;
const unsigned long buttonCooldownTime = 5000; // Cooldown time in milliseconds

bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap)
{
  // Stop further decoding as the image is running off the bottom of the screen
  if (y >= tft.height()) return 0;

  // This function will clip the image block rendering automatically at the TFT boundaries
  tft.pushImage(x, y, w, h, bitmap);

  // Return 1 to decode the next block
  return 1;
}

void ws_comm(void* arg);
void button_task_func(void* arg);
void i2s_write_from_client();
void i2sInit();
static void server_task(void *arg);

void setup() {
  Serial.begin(115200);
  delay(1000);

  tft.begin();
  tft.setRotation(3);
  tft.setTextColor(0xFFFF, 0x0000);
  tft.fillScreen(TFT_RED);
  tft.setSwapBytes(true); // We need to swap the colour bytes (endianess)

  // The jpeg image can be scaled by a factor of 1, 2, 4, or 8
  TJpgDec.setJpgScale(1);

  // The decoder must be given the exact name of the rendering function above
  TJpgDec.setCallback(tft_output);

  Serial.println();
  Serial.println("Setting AP...");
  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP Address : ");
  Serial.println(IP);

  server.listen(8888);

  button.setDebounceTime(50); // Set debounce time to 50 milliseconds

  connection_established = xSemaphoreCreateBinary();
  button_semaphore = xSemaphoreCreateBinary();

  xTaskCreatePinnedToCore(ws_comm, "websocket server", 4096, NULL, 5, &ws_comm_task, 1);
  xTaskCreatePinnedToCore(button_task_func, "button task", 2048, NULL, 5, &button_task, 1);
}

void loop() {
  if (server.poll()) {
    client = server.accept();
    xSemaphoreGive(connection_established);
  }
  i2s_write_from_client();
}

void button_task_func(void* arg) {
  while (1) {
    button.loop(); // Call the button's loop function to update its state

    if (button.isPressed() && !buttonPressed && (millis() - lastButtonPressTime >= buttonCooldownTime)) {
      buttonPressed = true;
      lastButtonPressTime = millis();

      // Send message to the client
      if (client.available()) {
        client.send("Button pressed!");
        Serial.println("Button pressed");

        // Display "Button pressed" on the TFT display
        tft.setCursor(0, 0);
        tft.println("Button pressed");
      }

      xSemaphoreGive(button_semaphore);
    }

    if (buttonPressed && (millis() - lastButtonPressTime >= buttonCooldownTime)) {
      buttonPressed = false;
    }

    vTaskDelay(pdMS_TO_TICKS(20)); // Adjust delay as needed
  }
}

void ws_comm(void* arg) {
  while (1) {
    if (xSemaphoreTake(connection_established, portMAX_DELAY) == pdTRUE) {
      while (client.available()) {
        WebsocketsMessage msg = client.readBlocking();

        // Decode and display the image
        uint32_t t = millis();

        // Get the width and height in pixels of the JPEG if you wish
        uint16_t w = 0, h = 0;
        TJpgDec.getJpgSize(&w, &h, (const uint8_t*)msg.c_str(), msg.length());
        Serial.print("Width = ");
        Serial.print(w);
        Serial.print(", height = ");
        Serial.println(h);

        // Draw the image, top left at 0,0
        TJpgDec.drawJpg(0, 0, (const uint8_t*)msg.c_str(), msg.length());

        // How much time did rendering take
        t = millis() - t;
        Serial.print(t);
        Serial.println(" ms");

        // Send message to the client
        if (xSemaphoreTake(button_semaphore, 0) == pdTRUE) {
          client.send("Button pressed!");
          Serial.println("Button pressed");
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // Adjust delay as needed
      }
    }
  }
}

void i2s_write_from_client()
{
  if (client.available()) {
    last_update_sent = millis();
    client.poll();
    WebsocketsMessage msg = client.readBlocking();
    char *buf_ptr = (char*)msg.c_str();
    size_t bytes_written = static_cast<size_t>(msg.length());
    i2s_write(I2S_PORT, buf_ptr, bytes_written, &bytes_written, portMAX_DELAY);

    int32_t sample = (int32_t)buf_ptr[3] << 24;
    sample |= (int32_t)buf_ptr[2] << 16;
    sample |= (int32_t)buf_ptr[1] << 8;
    sample |= (int32_t)buf_ptr[0];
  }
}

void i2sInit() {
  i2s_driver_install(I2S_PORT, &i2s_config_tx, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config_tx);
}

static void server_task(void *arg)
{
  while (1) {
    if (server.poll()) {
      client = server.accept();
      i2sInit();
      last_update_sent = millis();
      isConnected = true;
      Serial.println("socket connected");
    }

    if (isConnected && millis() - last_update_sent > UPDATE_INTERVAL) {
      Serial.println("Client disconnected!");
      isConnected = false;
      i2s_driver_uninstall(I2S_PORT);
    }
    vTaskDelay(100);
  }
}
