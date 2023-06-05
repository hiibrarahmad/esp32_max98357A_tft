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
#include "soc/rtc_wdt.h"

const char* ssid = "ibrar";
const char* password = "ibrarahmad";

const char* sta_ssid = "your_router_ssid";
const char* sta_password = "your_router_password";

const IPAddress staticIP(192, 168, 4, 1); // Static IP address for AP mode
const IPAddress gateway(192, 168, 1, 1);
const IPAddress subnet(255, 255, 255, 0);

using namespace websockets;

WebsocketsServer server;
WebsocketsClient client;

SemaphoreHandle_t connection_established = NULL;
SemaphoreHandle_t button1_semaphore = NULL;
SemaphoreHandle_t audio_semaphore = NULL;
TaskHandle_t ws_comm_task = NULL;
TaskHandle_t button_task = NULL;
TaskHandle_t audio_task = NULL;

unsigned long last_update_sent = 0;
bool isConnected;
bool isAudioPlaying = false;

TFT_eSPI tft = TFT_eSPI(); // Invoke custom library

#define I2S_WS_TX  12
#define I2S_SCK_TX 13
#define I2S_DATA_OUT_TX  15

#define I2S_PORT I2S_NUM_0
#define I2S_SAMPLE_RATE   (16000)
#define I2S_SAMPLE_BITS   (32)
#define UPDATE_INTERVAL   (500)

ezButton button(35);
ezButton button1(26); // Switch case button
bool buttonPressed = false;
unsigned long lastButtonPressTime = 0;
const unsigned long buttonCooldownTime = 5000; // Cooldown time in milliseconds
bool button1Pressed = false;
unsigned long lastButton1PressTime = 0;
const unsigned long button1CooldownTime = 5000; // Cooldown time in milliseconds

int button1PressCount = 0;

const i2s_config_t i2s_config_tx = {
  .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_TX),
  .sample_rate = I2S_SAMPLE_RATE,
  .bits_per_sample = i2s_bits_per_sample_t(I2S_SAMPLE_BITS),
  .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
  .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
  .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
  .dma_buf_count = 32,
  .dma_buf_len = 64
};

const i2s_pin_config_t pin_config_tx = {
  .bck_io_num = I2S_SCK_TX, // LRC
  .ws_io_num = I2S_WS_TX,   //blck
  .data_out_num = I2S_DATA_OUT_TX, //din
  .data_in_num = I2S_PIN_NO_CHANGE
};

WebsocketsMessage audioDatarecieveddd;

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
  WiFi.softAPConfig(staticIP, gateway, subnet);
  WiFi.softAP(ssid, password);

  IPAddress AP_IP = WiFi.softAPIP();
  Serial.print("AP IP Address: ");
  Serial.println(AP_IP);

  Serial.println();
  Serial.println("Connecting to WiFi...");
  WiFi.begin(sta_ssid, sta_password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("STA IP Address: ");
  Serial.println(WiFi.localIP());

  server.listen(8888);

  button.setDebounceTime(50); // Set debounce time to 50 milliseconds
  button1.setDebounceTime(50); // Set debounce time to 50 milliseconds

  last_update_sent = millis();
  i2sInit();
  isConnected = true;
  Serial.println("Socket connected");

  connection_established = xSemaphoreCreateBinary();
  button1_semaphore = xSemaphoreCreateBinary();
  audio_semaphore = xSemaphoreCreateBinary();

  xTaskCreate(ws_comm, "websocket server", 8192, NULL, 5, &ws_comm_task);
  xTaskCreate(button_task_func, "button task", 2048, NULL, 5, &button_task);
  // xTaskCreate(audio_task_func, "audio task", 4096, NULL, 5, &audio_task); // Create audio task
}

bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap)
{
  // Stop further decoding as the image is running off the bottom of the screen
  if (y >= tft.height()) return 0;

  // This function will clip the image block rendering automatically at the TFT boundaries
  tft.pushImage(x, y, w, h, bitmap);

  // Return 1 to decode the next block
  return 1;
}

void loop() {
  if (server.poll()) {
    client = server.accept();
    xSemaphoreGive(connection_established);
    i2sInit();
  }
}

void button_task_func(void* arg) {
  while (1) {
    button.loop(); // Call the button's loop function to update its state
    button1.loop(); // Call the switch case button's loop function to update its state

    if (button.isPressed() && !buttonPressed && (millis() - lastButtonPressTime >= buttonCooldownTime)) {
      buttonPressed = true;
      lastButtonPressTime = millis();
      if(buttonPressed=true){
      sendButtonPress(); // Send button press data through websocket
    }
    }

    if (button1.isPressed() && !button1Pressed && (millis() - lastButton1PressTime >= button1CooldownTime)) {
      button1Pressed = true;
      lastButton1PressTime = millis();

      button1PressCount++;
      if (button1PressCount > 1) {
        button1PressCount = 0;
      }
      xSemaphoreGive(button1_semaphore); // Notify the switch case button task
      
    }

    if (buttonPressed && (millis() - lastButtonPressTime >= buttonCooldownTime)) {
      buttonPressed = false;
    }

    if (button1Pressed && (millis() - lastButton1PressTime >= button1CooldownTime)) {
      button1Pressed = false;
    }

    vTaskDelay(pdMS_TO_TICKS(20)); // Adjust delay as needed
  }
}


void sendButtonPress() {
  if (client.available()) {
    client.send("Button pressed!");
    Serial.println("Button pressed");
    // Display "Button pressed" on the TFT display
    tft.setCursor(0, 0);
    tft.println("Button pressed");
    client.poll();  
    }

}


void ws_comm(void* arg) {
  while (1) {
    if (xSemaphoreTake(connection_established, portMAX_DELAY) == pdTRUE) {
      while (client.available()) {
        WebsocketsMessage msg = client.readBlocking();

        if (button1PressCount == 0) {
          // Handle case 0: Receive picture
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
        } else if (button1PressCount == 1) {
          // Handle case 1: Allow audio
          Serial.println("audio recieving");
          const char* buf_ptr = msg.c_str();
          size_t bytes_written = msg.length();
          i2s_write(I2S_PORT, buf_ptr, bytes_written, &bytes_written, portMAX_DELAY);

          if (strncmp(buf_ptr, "Start audio", 11) == 0) {
            isAudioPlaying = true;
          } else if (strncmp(buf_ptr, "Stop audio", 10) == 0) {
            isAudioPlaying = false;
          }
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // Adjust delay as needed
      }
    }
  }
}

void i2sInit() {
  if (client.available()) {
    isConnected = true;
    Serial.println("Client connected");
  }
  i2s_driver_install(I2S_PORT, &i2s_config_tx, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config_tx);
}
