/**
 * @file main.c
 * @brief UWB Anchor Application - ESP-IDF
 *
 * Direct port of main_anchor.ino to ESP-IDF.
 * Responds to ranging requests from Tags using double-sided ranging protocol.
 *
 * SPI Pin Configuration (board-specific):
 *   MOSI=12, MISO=11, SCK=10, CS=13, RST=7
 */

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "dwm3000.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "UWB_ANCHOR";

// ============================================================
// Pin Configuration (board-specific, equivalent to .ino defines)
// ============================================================
#define PIN_NUM_MISO 11
#define PIN_NUM_MOSI 12
#define PIN_NUM_CLK 10
#define PIN_NUM_CS 13
#define PIN_NUM_RST 7

// ============================================================
// Anchor Configuration (from .ino)
// ============================================================
#define ANCHOR_ID 1            // Set to 1 for Anchor 1, 2 for Anchor 2
#define RESPONSE_TIMEOUT_MS 10 // Maximum time to wait for a response
#define MAX_RETRIES 3

// ============================================================
// Global Variables (matching .ino globals)
// ============================================================
static dwm3000_t dwm3000_dev;

static int rx_status;
static int tx_status;

static int curr_stage = 0;

static int t_roundB = 0;
static int t_replyB = 0;

static long long rx = 0;
static long long tx = 0;

static unsigned long last_ranging_time = 0;
static int retry_count = 0;

// ============================================================
// Helper: millis() equivalent
// ============================================================
static unsigned long millis(void) {
  return (unsigned long)(esp_timer_get_time() / 1000);
}

// ============================================================
// resetRadio() — equivalent to .ino resetRadio()
// ============================================================
static void resetRadio(void) {
  ESP_LOGI(TAG, "[INFO] Performing radio reset...");
  dwm3000_soft_reset(&dwm3000_dev);
  vTaskDelay(pdMS_TO_TICKS(100));
  dwm3000_clear_status(&dwm3000_dev);
  dwm3000_configure_as_tx(&dwm3000_dev);
  dwm3000_standard_rx(&dwm3000_dev);
}

// ============================================================
// loop() — equivalent to .ino loop(), runs as FreeRTOS task
// ============================================================
static void anchor_loop_task(void *pvParameters) {
  while (1) {
    // --- Pre-switch check (from .ino line 1756-1766) ---
    if (dwm3000_received_frame_succ(&dwm3000_dev) == 1 &&
        dwm3000_ds_get_stage(&dwm3000_dev) == 1 &&
        dwm3000_get_destination_id(&dwm3000_dev) == ANCHOR_ID) {
      // Reset session if new ranging request arrives
      if (curr_stage != 0) {
        ESP_LOGI(TAG, "[INFO] New request - resetting session");
        curr_stage = 0;
        t_roundB = 0;
        t_replyB = 0;
      }
    }

    switch (curr_stage) {
    case 0: // Await ranging
      t_roundB = 0;
      t_replyB = 0;
      last_ranging_time = millis(); // Reset timeout timer

      if ((rx_status = dwm3000_received_frame_succ(
               &dwm3000_dev))) { // Check Poll Received
        dwm3000_clear_status(&dwm3000_dev);
        if (rx_status == 1) { // If frame reception was successful
          // Only respond if frame is addressed to us
          if (dwm3000_get_destination_id(&dwm3000_dev) == ANCHOR_ID) {
            if (dwm3000_ds_is_error_frame(&dwm3000_dev)) {
              ESP_LOGW(TAG, "[WARNING] Received error frame!");
              curr_stage = 0;
              dwm3000_standard_rx(&dwm3000_dev);
            } else if (dwm3000_ds_get_stage(&dwm3000_dev) != 1) {
              ESP_LOGW(TAG, "[WARNING] Unexpected stage: %d",
                       dwm3000_ds_get_stage(&dwm3000_dev));
              dwm3000_ds_send_error_frame(&dwm3000_dev);
              dwm3000_standard_rx(&dwm3000_dev);
              curr_stage = 0;
            } else {
              curr_stage = 1; // Move to send response
            }
          } else {
            // Not for us, go back to RX
            dwm3000_standard_rx(&dwm3000_dev);
          }
        } else {
          ESP_LOGE(TAG, "[ERROR] Receiver Error occurred!");
          dwm3000_clear_status(&dwm3000_dev);
        }
      } else if (millis() - last_ranging_time > RESPONSE_TIMEOUT_MS) {
        ESP_LOGW(TAG, "[WARNING] Timeout waiting for ranging request");
        if (++retry_count > MAX_RETRIES) {
          ESP_LOGE(TAG, "[ERROR] Max retries reached, resetting radio");
          resetRadio();
          retry_count = 0;
        }
        dwm3000_standard_rx(&dwm3000_dev); // Reset to listening mode
      }
      break;

    case 1: // Ranging received. Sending response
      dwm3000_ds_send_frame(&dwm3000_dev, 2); // Sends "Response" (Stage 2)

      rx = dwm3000_read_rx_timestamp(
          &dwm3000_dev); // Reads T2 (Time Poll Arrived)
      tx = dwm3000_read_tx_timestamp(
          &dwm3000_dev); // Reads T3 (Time Response Sent)

      t_replyB = tx - rx;           // Calculates t_replyB (T3 - T2)
      curr_stage = 2;               // Move to wait for Final
      last_ranging_time = millis(); // Reset timeout timer
      break;

    case 2: // Awaiting response
      if ((rx_status = dwm3000_received_frame_succ(
               &dwm3000_dev))) { // Check Final Received
        retry_count = 0;         // Reset on successful response
        dwm3000_clear_status(&dwm3000_dev);
        if (rx_status == 1) { // If frame reception was successful
          if (dwm3000_ds_is_error_frame(&dwm3000_dev)) {
            ESP_LOGW(TAG, "[WARNING] Received error frame!");
            curr_stage = 0;
            dwm3000_standard_rx(&dwm3000_dev);
          } else if (dwm3000_ds_get_stage(&dwm3000_dev) != 3) {
            ESP_LOGW(TAG, "[WARNING] Unexpected stage: %d",
                     dwm3000_ds_get_stage(&dwm3000_dev));
            dwm3000_ds_send_error_frame(&dwm3000_dev);
            dwm3000_standard_rx(&dwm3000_dev);
            curr_stage = 0;
          } else {
            curr_stage = 3; // Move to calculate and report
          }
        } else {
          ESP_LOGE(TAG, "[ERROR] Receiver Error occurred!");
          dwm3000_clear_status(&dwm3000_dev);
        }
      } else if (millis() - last_ranging_time > RESPONSE_TIMEOUT_MS) {
        ESP_LOGW(TAG, "[WARNING] Timeout waiting for second response");
        if (++retry_count > MAX_RETRIES) {
          ESP_LOGE(TAG, "[ERROR] Max retries reached, resetting radio");
          resetRadio();
          retry_count = 0;
        }
        curr_stage = 0;
        dwm3000_standard_rx(&dwm3000_dev);
      }
      break;

    case 3: // Second response received. Sending information frame
      rx = dwm3000_read_rx_timestamp(
          &dwm3000_dev);  // Reads T6 (Time Final Arrived)
      t_roundB = rx - tx; // Calculate Anchor's Round Trip: (Time Final Arrived
                          // - Time Response Sent)
      dwm3000_ds_send_rt_info(&dwm3000_dev, t_roundB,
                              t_replyB); // Sends DATA packet

      curr_stage = 0; // Reset for next ranging
      dwm3000_standard_rx(&dwm3000_dev);
      break;

    default:
      ESP_LOGE(TAG,
               "[ERROR] Entered unknown stage (%d). Reverting back to stage 0",
               curr_stage);
      curr_stage = 0;
      dwm3000_standard_rx(&dwm3000_dev);
      break;
    }

    // Must yield at least 1 tick to feed watchdog.
    // pdMS_TO_TICKS(1) = 0 at 100Hz tick rate, so use vTaskDelay(1) directly.
    vTaskDelay(1);
  }
}

// ============================================================
// app_main() — equivalent to .ino setup()
// ============================================================
void app_main(void) {
  // Serial.begin(115200) — not needed, ESP_LOG handles serial output

  // --- DWM3000.begin() equivalent: SPI bus init + device add ---
  spi_bus_config_t buscfg = {
      .miso_io_num = PIN_NUM_MISO,
      .mosi_io_num = PIN_NUM_MOSI,
      .sclk_io_num = PIN_NUM_CLK,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = 32,
  };
  ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

  esp_err_t ret =
      dwm3000_begin(&dwm3000_dev, SPI2_HOST, PIN_NUM_CS, PIN_NUM_RST);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize SPI for DWM3000");
    return;
  }

  // --- DWM3000.hardReset() ---
  dwm3000_hard_reset(&dwm3000_dev);
  vTaskDelay(pdMS_TO_TICKS(200)); // delay(200)

  // --- if (!DWM3000.checkSPI()) { ... while(1); } ---
  if (!dwm3000_check_spi(&dwm3000_dev)) {
    ESP_LOGE(TAG, "[ERROR] Could not establish SPI Connection to DWM3000!");
    while (1) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }

  // --- while (!DWM3000.checkForIDLE()) { ... } ---
  while (!dwm3000_check_idle(&dwm3000_dev)) {
    ESP_LOGE(TAG, "[ERROR] IDLE1 FAILED");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  // --- DWM3000.softReset(); delay(200); ---
  dwm3000_soft_reset(&dwm3000_dev);
  vTaskDelay(pdMS_TO_TICKS(200));

  // --- if (!DWM3000.checkForIDLE()) { ... while(1); } ---
  if (!dwm3000_check_idle(&dwm3000_dev)) {
    ESP_LOGE(TAG, "[ERROR] IDLE2 FAILED");
    while (1) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }

  // --- DWM3000.init() ---
  ret = dwm3000_chip_init(&dwm3000_dev);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize DWM3000");
    return;
  }

  // --- DWM3000.setupGPIO() ---
  dwm3000_setup_gpio(&dwm3000_dev);

  // --- DWM3000.setTXAntennaDelay(16350) ---
  dwm3000_set_antenna_delay(&dwm3000_dev, 16350);

  // --- DWM3000.setSenderID(ANCHOR_ID) ---
  dwm3000_set_sender_id(&dwm3000_dev, ANCHOR_ID);

  // --- Print info ---
  ESP_LOGI(TAG, "> ANCHOR %d - Ready for ranging <", ANCHOR_ID);
  ESP_LOGI(TAG, "Antenna delay set to: %d",
           dwm3000_get_antenna_delay(&dwm3000_dev));
  ESP_LOGI(TAG, "[INFO] Setup finished.");

  // --- DWM3000.configureAsTX(); DWM3000.clearSystemStatus();
  // DWM3000.standardRX(); ---
  dwm3000_configure_as_tx(&dwm3000_dev);
  dwm3000_clear_status(&dwm3000_dev);
  dwm3000_standard_rx(&dwm3000_dev);

  // --- Start loop() as a FreeRTOS task ---
  xTaskCreate(anchor_loop_task, "anchor_loop", 4096, NULL, 5, NULL);
}
