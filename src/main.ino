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
//TaskHandle_t audio_task = NULL;  // Audio task handle


unsigned long last_update_sent = 0;
bool isAudioPlaying = false;

TFT_eSPI tft = TFT_eSPI(); // Invoke custom library

// #define I2S_WS_TX  12
// #define I2S_SCK_TX 13
// #define I2S_DATA_OUT_TX  15

// #define I2S_PORT I2S_NUM_0
// #define I2S_SAMPLE_RATE   (16000)
// #define I2S_SAMPLE_BITS   (32)
// #define UPDATE_INTERVAL   (500)


void ws_comm(void* arg);
void button_task_func(void* arg);
//void audio_task_func(void* arg);  // Audio task function
//void i2s_write_from_client();
//void i2sInit();
//void startAudioPlayback();
//void stopAudioPlayback();

bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap)
{
  // Stop further decoding as the image is running off the bottom of the screen
  if (y >= tft.height()) return 0;

  // This function will clip the image block rendering automatically at the TFT boundaries
  tft.pushImage(x, y, w, h, bitmap);

  // Return 1 to decode the next block
  return 1;
}

ezButton button(26); // Pin for the button

bool buttonPressed = false;
unsigned long lastButtonPressTime = 0;
const unsigned long buttonCooldownTime = 5000; // Cooldown time in milliseconds

// const i2s_config_t i2s_config_tx = {
//   .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_TX),
//   .sample_rate = 16000,
//   .bits_per_sample = i2s_bits_per_sample_t(32),
//   .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
//   .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
//   .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
//   .dma_buf_count = 32,
//   .dma_buf_len = 64
// };

// const i2s_pin_config_t pin_config_tx = {
//   .bck_io_num = 13, // LRC
//   .ws_io_num = 12,   // BCK
//   .data_out_num = 15, // DIN
//   .data_in_num = I2S_PIN_NO_CHANGE
// };

void setup() {
  Serial.begin(115200);
  delay(1000);

  // xTaskCreatePinnedToCore(
  //     audio_task_func, /* Function to implement the task */
  //     "audio task", /* Name of the task */
  //     10000,  /* Stack size in words */
  //     NULL,  /* Task input parameter */
  //     0,  /* Priority of the task */
  //     &audio_task,  /* Task handle. */
  //     0); /* Core where the task should run */

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
  Serial.print("AP IP Address: ");
  Serial.println(IP);

  server.listen(8888);

  button.setDebounceTime(50); // Set debounce time to 50 milliseconds

  connection_established = xSemaphoreCreateBinary();
  button_semaphore = xSemaphoreCreateBinary();
  //xTaskCreate(ws_comm, "websocket server", 4096, NULL, 5, &ws_comm_task);
  xTaskCreatePinnedToCore(
   ws_comm, 
   "websocket server",
   4096, NULL,
   5,
   &ws_comm_task,
   0);

  //xTaskCreate(button_task_func, "button task", 2048, NULL, 5, &button_task);
  xTaskCreatePinnedToCore(
    button_task_func, 
    "button task", 
    4096, 
    NULL, 
    5, 
    &button_task, 
    0);
 // xTaskCreate(audio_task_func, "audio task", 4096, NULL, 5, &audio_task);  // Create audio task
  
}

void loop() {
  if (server.poll()) {
    client = server.accept();
    xSemaphoreGive(connection_established);
  }

  //i2s_write_from_client();
}

void button_task_func(void* arg) {
  while (1) {
     Serial.println("button task core");
    Serial.println(xPortGetCoreID());
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

// Void Task1code( void * parameter) {
//   for(;;) {
//     Code for task 1 - infinite loop
//     (...)
//   }
// }

// void audio_task_func(void* arg) {
//   i2sInit();  // Initialize I2S configuration

//   while (1) {
//     if (xSemaphoreTake(connection_established, portMAX_DELAY) == pdTRUE) {
//       while (client.available()) {
//         WebsocketsMessage msg = client.readBlocking();

//         // Check if the message is "Start audio"
//         if (msg.data() == "Start audio") {
//           startAudioPlayback();
//           isAudioPlaying = true;
//         } else if (msg.data() == "Stop audio") {
//           if (isAudioPlaying) {
//             stopAudioPlayback();
//             isAudioPlaying = false;
//           }
//         }

//         vTaskDelay(pdMS_TO_TICKS(20)); // Adjust delay as needed
//       }
//     }
//   }
// }

void ws_comm(void* arg) {
  while (1) {
    if (xSemaphoreTake(connection_established, portMAX_DELAY) == pdTRUE) {
      while (client.available()) {
        WebsocketsMessage msg = client.readBlocking();
        
        Serial.println("pic task core");
        Serial.println(xPortGetCoreID());
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

// void i2sInit() {
//   i2s_driver_install(I2S_PORT, &i2s_config_tx, 0, NULL);
//   i2s_set_pin(I2S_PORT, &pin_config_tx);
// }

// void i2s_write_from_client() {
//   if (client.available()) {
//     client.onMessage([](WebsocketsMessage msg) {
//       Serial.println("Got Message: " + msg.data());
//       last_update_sent = millis();
//       client.poll();

//       // Check if the message is "Button pressed!"
//       if (msg.data() == "Start audio") {
//         // Handle audio playback in the audio task
//         //client.send("Start audio");
//       } else if (msg.data() == "Stop audio") {
//         // Handle audio stop in the audio task
//         //client.send("Stop audio");
//       }
//     });
//   }
// }

// void startAudioPlayback() {
//   // Code to start audio playback
//   Serial.println("Start audio playback");
//   // Example: playAudio();
// }

// void stopAudioPlayback() {
//   // Code to stop audio playback
//   Serial.println("Stop audio playback");
//   // Example: stopAudio();
// }
