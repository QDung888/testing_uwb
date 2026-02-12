/**
 * @file main.c
 * @brief UWB Tag Application for ESP-IDF
 *
 * Faithful port of main_tag.ino to ESP-IDF.
 * Multi-anchor double-sided ranging, serial output only.
 */

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "dwm3000.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "UWB_TAG";

// ============================================================================
// SPI Pin Configuration for ESP32-S3
// CLK=GPIO10, MISO=GPIO11, MOSI=GPIO12, CSn=GPIO13
// ============================================================================
#define PIN_NUM_CLK 10
#define PIN_NUM_MISO 11
#define PIN_NUM_MOSI 12
#define PIN_NUM_CS 13
#define PIN_NUM_RST 7

// ============================================================================
// Scalable Anchor Configuration (matching .ino)
// ============================================================================
#define NUM_ANCHORS 1
#define TAG_ID 10
#define FIRST_ANCHOR_ID 1

// Ranging Configuration
#define FILTER_SIZE 30
#define MIN_DISTANCE 0
#define MAX_DISTANCE 1000.0f

// ============================================================================
// Data Structures (matching .ino AnchorData)
// ============================================================================
typedef struct {
  int anchor_id;

  // Timing measurements
  int t_roundA;
  int t_replyA;
  int64_t rx;
  int64_t tx;
  int clock_offset;

  // Distance measurements
  float distance;
  float distance_history[FILTER_SIZE];
  int history_index;
  float filtered_distance;

  // Signal quality metrics
  float signal_strength;
  float fp_signal_strength;
} anchor_data_t;

// ============================================================================
// Global Variables (matching .ino)
// ============================================================================
static dwm3000_t dwm_device;
static anchor_data_t anchors[NUM_ANCHORS];
static int current_anchor_index = 0;
static int curr_stage = 0;

// ============================================================================
// Anchor Management Functions (matching .ino)
// ============================================================================

static void initialize_anchors(void) {
  for (int i = 0; i < NUM_ANCHORS; i++) {
    memset(&anchors[i], 0, sizeof(anchor_data_t));
    anchors[i].anchor_id = FIRST_ANCHOR_ID + i;
  }
}

static anchor_data_t *get_current_anchor(void) {
  return &anchors[current_anchor_index];
}

static int get_current_anchor_id(void) {
  return anchors[current_anchor_index].anchor_id;
}

static void switch_to_next_anchor(void) {
  current_anchor_index = (current_anchor_index + 1) % NUM_ANCHORS;
}

// ============================================================================
// Distance Filtering Functions (matching .ino)
// ============================================================================

static bool is_valid_distance(float distance) {
  return (distance >= MIN_DISTANCE && distance <= MAX_DISTANCE);
}

static float calculate_median(float arr[], int size) {
  float temp[FILTER_SIZE];
  for (int i = 0; i < size; i++) {
    temp[i] = arr[i];
  }

  // Bubble sort (matching .ino)
  for (int i = 0; i < size - 1; i++) {
    for (int j = i + 1; j < size; j++) {
      if (temp[j] < temp[i]) {
        float t = temp[i];
        temp[i] = temp[j];
        temp[j] = t;
      }
    }
  }

  if (size % 2 == 0) {
    return (temp[size / 2 - 1] + temp[size / 2]) / 2.0f;
  } else {
    return temp[size / 2];
  }
}

static void update_filtered_distance(anchor_data_t *data) {
  data->distance_history[data->history_index] = data->distance;
  data->history_index = (data->history_index + 1) % FILTER_SIZE;

  float valid_distances[FILTER_SIZE];
  int valid_count = 0;

  for (int i = 0; i < FILTER_SIZE; i++) {
    if (is_valid_distance(data->distance_history[i])) {
      valid_distances[valid_count++] = data->distance_history[i];
    }
  }

  if (valid_count > 0) {
    data->filtered_distance = calculate_median(valid_distances, valid_count);
  } else {
    data->filtered_distance = 0;
  }
}

// ============================================================================
// Output Functions (matching .ino printAllDistances)
// ============================================================================

static void print_all_distances(void) {
  printf("Distances - ");
  for (int i = 0; i < NUM_ANCHORS; i++) {
    printf("A%d: ", anchors[i].anchor_id);
    if (anchors[i].filtered_distance > 0) {
      printf("%.2f cm", anchors[i].filtered_distance);
    } else {
      printf("INVALID");
    }
    if (i < NUM_ANCHORS - 1) {
      printf(" | ");
    }
  }
  printf("\n");
}

// ============================================================================
// SPI Initialization (replaces Arduino SPI.begin())
// ============================================================================

static void init_spi(void) {
  spi_bus_config_t buscfg = {
      .miso_io_num = PIN_NUM_MISO,
      .mosi_io_num = PIN_NUM_MOSI,
      .sclk_io_num = PIN_NUM_CLK,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = 32,
  };

  ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
  ESP_LOGI(TAG, "SPI bus initialized");
}

// ============================================================================
// Ranging Task (matching .ino loop())
// ============================================================================

static void ranging_task(void *pvParameters) {
  int rx_status;

  while (1) {
    anchor_data_t *current_anchor = get_current_anchor();
    int current_anchor_id = get_current_anchor_id();

    switch (curr_stage) {

    case 0: // Start ranging with current target (matching .ino case 0)
      current_anchor->t_roundA = 0;
      current_anchor->t_replyA = 0;

      dwm3000_set_destination_id(&dwm_device, current_anchor_id);
      dwm3000_ds_send_frame(&dwm_device, 1);
      current_anchor->tx = dwm3000_read_tx_timestamp(&dwm_device);
      curr_stage = 1;
      break;

    case 1: // Await first response (matching .ino case 1)
      rx_status = dwm3000_received_frame_succ(&dwm_device);
      if (rx_status) {
        dwm3000_clear_status(&dwm_device);
        if (rx_status == 1) {
          if (dwm3000_ds_is_error_frame(&dwm_device)) {
            printf("[WARNING] Error frame from Anchor %d! Signal strength: ",
                   current_anchor_id);
            printf("%.2f dBm\n", dwm3000_get_signal_strength(&dwm_device));
            curr_stage = 0;
          } else if (dwm3000_ds_get_stage(&dwm_device) != 2) {
            printf("[WARNING] Unexpected stage from Anchor %d: %d\n",
                   current_anchor_id, dwm3000_ds_get_stage(&dwm_device));
            dwm3000_ds_send_error_frame(&dwm_device);
            curr_stage = 0;
          } else {
            curr_stage = 2;
          }
        } else {
          printf("[ERROR] Receiver Error from Anchor %d\n", current_anchor_id);
          dwm3000_clear_status(&dwm_device);
        }
      }
      break;

    case 2: // Response received. Send Final (matching .ino case 2)
      current_anchor->rx = dwm3000_read_rx_timestamp(&dwm_device);
      dwm3000_ds_send_frame(&dwm_device, 3);
      current_anchor->t_roundA = (int)(current_anchor->rx - current_anchor->tx);
      current_anchor->tx = dwm3000_read_tx_timestamp(&dwm_device);
      current_anchor->t_replyA = (int)(current_anchor->tx - current_anchor->rx);

      curr_stage = 3;
      break;

    case 3: // Await second response (matching .ino case 3)
      rx_status = dwm3000_received_frame_succ(&dwm_device);
      if (rx_status) {
        dwm3000_clear_status(&dwm_device);
        if (rx_status == 1) {
          if (dwm3000_ds_is_error_frame(&dwm_device)) {
            printf("[WARNING] Error frame from Anchor %d\n", current_anchor_id);
            curr_stage = 0;
          } else {
            current_anchor->clock_offset =
                dwm3000_get_raw_clock_offset(&dwm_device);
            curr_stage = 4;
          }
        } else {
          printf("[ERROR] Receiver Error from Anchor %d\n", current_anchor_id);
          dwm3000_clear_status(&dwm_device);
        }
      }
      break;

    case 4: // Calculate results (matching .ino case 4)
    {
      int ranging_time = dwm3000_ds_process_rt_info(
          &dwm_device, current_anchor->t_roundA, current_anchor->t_replyA,
          dwm3000_read(&dwm_device, 0x12, 0x04),
          dwm3000_read(&dwm_device, 0x12, 0x08), current_anchor->clock_offset);

      current_anchor->distance = dwm3000_convert_to_cm(ranging_time);
      current_anchor->signal_strength =
          dwm3000_get_signal_strength(&dwm_device);
      current_anchor->fp_signal_strength =
          dwm3000_get_fp_signal_strength(&dwm_device);
      update_filtered_distance(current_anchor);
    }

      // Print raw + filtered for debugging
      printf("A%d: raw=%.2f cm, filtered=%.2f cm, RSSI=%.2f dBm\n",
             current_anchor->anchor_id, current_anchor->distance,
             current_anchor->filtered_distance,
             current_anchor->signal_strength);

      // Switch to next anchor
      switch_to_next_anchor();
      curr_stage = 0;
      break;

    default:
      printf("Entered stage (%d). Reverting back to stage 0\n", curr_stage);
      curr_stage = 0;
      break;
    }

    // Must yield at least 1 tick to feed watchdog.
    // pdMS_TO_TICKS(1) = 0 at 100Hz tick rate, so use vTaskDelay(1) directly.
    vTaskDelay(1);
  }
}

// ============================================================================
// Main Application (matching .ino setup(), without WiFi)
// ============================================================================

void app_main(void) {
  esp_err_t ret;

  // Initialize anchor array (matching .ino)
  initialize_anchors();

  printf("Initialized %d anchors:\n", NUM_ANCHORS);
  for (int i = 0; i < NUM_ANCHORS; i++) {
    printf("  Anchor %d - ID: %d\n", i, anchors[i].anchor_id);
  }

  // Initialize UWB (matching .ino setup() sequence)
  init_spi();

  // DWM3000.begin()
  ret = dwm3000_begin(&dwm_device, SPI2_HOST, PIN_NUM_CS, PIN_NUM_RST);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize SPI device");
    while (1)
      vTaskDelay(pdMS_TO_TICKS(1000));
  }

  // DWM3000.hardReset()
  dwm3000_hard_reset(&dwm_device);
  vTaskDelay(pdMS_TO_TICKS(200));

  // DWM3000.checkSPI()
  if (!dwm3000_check_spi(&dwm_device)) {
    ESP_LOGE(TAG, "[ERROR] Could not establish SPI Connection to DWM3000!");
    while (1)
      vTaskDelay(pdMS_TO_TICKS(1000));
  }

  // Wait IDLE
  while (!dwm3000_check_idle(&dwm_device)) {
    ESP_LOGE(TAG, "[ERROR] IDLE1 FAILED");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  // DWM3000.softReset()
  dwm3000_soft_reset(&dwm_device);
  vTaskDelay(pdMS_TO_TICKS(200));

  // Check IDLE
  if (!dwm3000_check_idle(&dwm_device)) {
    ESP_LOGE(TAG, "[ERROR] IDLE2 FAILED");
    while (1)
      vTaskDelay(pdMS_TO_TICKS(1000));
  }

  // DWM3000.init()
  ret = dwm3000_chip_init(&dwm_device);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Chip initialization failed");
    while (1)
      vTaskDelay(pdMS_TO_TICKS(1000));
  }

  dwm3000_setup_gpio(&dwm_device);
  dwm3000_set_antenna_delay(&dwm_device, 16350);
  dwm3000_set_sender_id(&dwm_device, TAG_ID);

  printf("> TAG - Three Anchor Ranging System <\n");
  printf("[INFO] Setup is finished.\n");
  printf("Antenna delay set to: %d\n", dwm3000_get_antenna_delay(&dwm_device));

  dwm3000_configure_as_tx(&dwm_device);
  dwm3000_clear_status(&dwm_device);

  // Start ranging task
  xTaskCreate(ranging_task, "ranging_task", 8192, NULL, 5, NULL);
}
