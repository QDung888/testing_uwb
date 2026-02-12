/**
 * @file dwm3000.c
 * @brief DWM3000 UWB Module Driver for ESP-IDF
 *
 * Ported from Arduino to ESP-IDF for ESP32
 */

#include "dwm3000.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

static const char *TAG = "DWM3000";

// Internal function prototypes
static uint32_t send_bytes(dwm3000_t *dev, uint8_t *data, int len, int rec_len);
static uint32_t read_or_write_full_address(dwm3000_t *dev, uint32_t base,
                                           uint32_t sub, uint32_t data,
                                           uint32_t data_len, bool is_write);
static void write_fast_command(dwm3000_t *dev, uint8_t cmd);
static void set_bit(dwm3000_t *dev, int reg_addr, int sub_addr, int shift,
                    bool value);
static void set_bit_high(dwm3000_t *dev, int reg_addr, int sub_addr, int shift);
static void clear_aon_config(dwm3000_t *dev);
static int check_for_dev_id(dwm3000_t *dev);
static unsigned int count_bits(unsigned int number);
static void write_sys_config(dwm3000_t *dev);

// ============================================================================
// SPI Communication Functions
// ============================================================================

/**
 * @brief Send bytes over SPI and optionally receive response
 */
static uint32_t send_bytes(dwm3000_t *dev, uint8_t *data, int len,
                           int rec_len) {
  esp_err_t ret;
  spi_transaction_t trans;
  memset(&trans, 0, sizeof(trans));

  if (rec_len > 0) {
    // Allocation for full-duplex transfer (Header + Data)
    // We send Header + Dummy bytes to clock out the response
    int total_len = len + rec_len;
    uint8_t *tx_buf = heap_caps_malloc(total_len, MALLOC_CAP_DMA);
    uint8_t *rx_buf = heap_caps_malloc(total_len, MALLOC_CAP_DMA);

    if (!tx_buf || !rx_buf) {
      ESP_LOGE(TAG, "Failed to allocate SPI buffers");
      if (tx_buf)
        free(tx_buf);
      if (rx_buf)
        free(rx_buf);
      return 0;
    }

    // Prepare TX buffer: Header then Zeros (Dummy)
    memcpy(tx_buf, data, len);
    memset(tx_buf + len, 0, rec_len);

    trans.length = total_len * 8;
    trans.tx_buffer = tx_buf;
    trans.rx_buffer = rx_buf;

    ret = spi_device_transmit(dev->spi, &trans);

    uint32_t val = 0;
    if (ret == ESP_OK) {
      // Extract meaningful data (after the header duration)
      // Check if rec_len fits in uint32_t output
      for (int i = 0; i < rec_len; i++) {
        // In DWM3000 SPI, data comes immediately? Or valid data is shifted?
        // Usually for register read: [Header] -> [Data]
        // So valid data starts at index 'len' in rx_buf
        val |= ((uint32_t)rx_buf[len + i]) << (8 * i);
      }
    } else {
      ESP_LOGE(TAG, "SPI transmit failed");
    }

    free(tx_buf);
    free(rx_buf);
    return val;
  } else {
    // Transmit only
    trans.length = len * 8;
    trans.tx_buffer = data;

    ret = spi_device_transmit(dev->spi, &trans);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "SPI transmit failed");
    }
    return 0;
  }
}

/**
 * @brief Read or write to a full register address
 */
static uint32_t read_or_write_full_address(dwm3000_t *dev, uint32_t base,
                                           uint32_t sub, uint32_t data,
                                           uint32_t data_len, bool is_write) {
  uint32_t header = 0x00;

  if (is_write) {
    header |= 0x80;
  }

  header |= ((base & 0x1F) << 1);

  if (sub > 0) {
    header |= 0x40;
    header = header << 8;
    header |= ((sub & 0x7F) << 2);
  }

  uint32_t header_size = (header > 0xFF) ? 2 : 1;
  uint32_t res = 0;

  if (!is_write) {
    // Read operation
    uint8_t header_arr[2];
    if (header_size == 1) {
      header_arr[0] = header & 0xFF;
    } else {
      header_arr[0] = (header >> 8) & 0xFF;
      header_arr[1] = header & 0xFF;
    }
    res = send_bytes(dev, header_arr, header_size, 4);
    return res;
  } else {
    // Write operation
    uint32_t payload_bytes = 0;

    if (data_len == 0) {
      if (data > 0) {
        uint32_t payload_bits = count_bits(data);
        payload_bytes = (payload_bits - (payload_bits % 8)) / 8;
        if ((payload_bits % 8) > 0) {
          payload_bytes++;
        }
      } else {
        payload_bytes = 1;
      }
    } else {
      payload_bytes = data_len;
    }

    size_t total_len = header_size + payload_bytes;
    uint8_t *payload = heap_caps_malloc(total_len, MALLOC_CAP_DMA);
    if (!payload) {
      ESP_LOGE(TAG, "Failed to allocate SPI TX buffer");
      return 0;
    }

    if (header_size == 1) {
      payload[0] = header & 0xFF;
    } else {
      payload[0] = (header >> 8) & 0xFF;
      payload[1] = header & 0xFF;
    }

    for (int i = 0; i < payload_bytes; i++) {
      payload[header_size + i] = (data >> (i * 8)) & 0xFF;
    }

    res = send_bytes(dev, payload, total_len, 0);
    free(payload);
    return res;
  }
}

/**
 * @brief Write fast command to DWM3000
 */
static void write_fast_command(dwm3000_t *dev, uint8_t cmd) {
  uint8_t *header = heap_caps_malloc(1, MALLOC_CAP_DMA);
  if (!header)
    return;

  *header = 0;
  *header = *header | 0x1;
  *header = *header | ((cmd & 0x1F) << 1);
  *header = *header | 0x80;

  send_bytes(dev, header, 1, 0);
  free(header);
}

/**
 * @brief Set a specific bit in a register
 */
static void set_bit(dwm3000_t *dev, int reg_addr, int sub_addr, int shift,
                    bool value) {
  uint8_t tmp_byte = dwm3000_read8bit(dev, reg_addr, sub_addr);
  if (value) {
    tmp_byte |= (1 << shift);
  } else {
    tmp_byte &= ~(1 << shift);
  }
  dwm3000_write(dev, reg_addr, sub_addr, tmp_byte, 1);
}

static void set_bit_high(dwm3000_t *dev, int reg_addr, int sub_addr,
                         int shift) {
  set_bit(dev, reg_addr, sub_addr, shift, true);
}

/**
 * @brief Count the number of bits in a number
 */
static unsigned int count_bits(unsigned int number) {
  if (number == 0)
    return 1;
  return (int)log2(number) + 1;
}

/**
 * @brief Check for correct device ID
 */
static int check_for_dev_id(dwm3000_t *dev) {
  uint32_t res = dwm3000_read(dev, GEN_CFG_AES_LOW_REG, NO_OFFSET);
  if (res != 0xDECA0302 && res != 0xDECA0312) {
    ESP_LOGE(TAG, "DEV_ID IS WRONG! Got: 0x%08lX", res);
    return 0;
  }
  ESP_LOGI(TAG, "Device ID verified: 0x%08lX", res);
  return 1;
}

/**
 * @brief Clear AON configuration
 */
static void clear_aon_config(dwm3000_t *dev) {
  dwm3000_write(dev, AON_REG, NO_OFFSET, 0x00, 2);
  dwm3000_write(dev, AON_REG, 0x14, 0x00, 1);
  dwm3000_write(dev, AON_REG, 0x04, 0x00, 1);
  dwm3000_write(dev, AON_REG, 0x04, 0x02, 0);
  vTaskDelay(pdMS_TO_TICKS(1));
}

// ============================================================================
// Public API Functions
// ============================================================================

uint32_t dwm3000_write(dwm3000_t *dev, int reg_base, int sub_addr,
                       uint32_t data, int data_len) {
  return read_or_write_full_address(dev, reg_base, sub_addr, data, data_len,
                                    true);
}

uint32_t dwm3000_read(dwm3000_t *dev, int reg_base, int sub_addr) {
  return read_or_write_full_address(dev, reg_base, sub_addr, 0, 0, false);
}

uint8_t dwm3000_read8bit(dwm3000_t *dev, int reg_base, int sub_addr) {
  return (uint8_t)(dwm3000_read(dev, reg_base, sub_addr) >> 24);
}

uint32_t dwm3000_read_otp(dwm3000_t *dev, uint8_t addr) {
  dwm3000_write(dev, OTP_IF_REG, 0x04, addr, 0);
  dwm3000_write(dev, OTP_IF_REG, 0x08, 0x02, 0);
  return dwm3000_read(dev, OTP_IF_REG, 0x10);
}

void dwm3000_hard_reset(dwm3000_t *dev) {
  gpio_set_direction(dev->rst_pin, GPIO_MODE_OUTPUT);
  gpio_set_level(dev->rst_pin, 0);
  vTaskDelay(pdMS_TO_TICKS(10));
  gpio_set_direction(dev->rst_pin, GPIO_MODE_INPUT);
  vTaskDelay(pdMS_TO_TICKS(10));
  ESP_LOGI(TAG, "Hard reset completed");
}

void dwm3000_soft_reset(dwm3000_t *dev) {
  clear_aon_config(dev);
  dwm3000_write(dev, PMSC_REG, 0x04, 0x1, 0);
  dwm3000_write(dev, PMSC_REG, 0x00, 0x00, 2);
  vTaskDelay(pdMS_TO_TICKS(100));
  dwm3000_write(dev, PMSC_REG, 0x00, 0xFFFF, 0);
  dwm3000_write(dev, PMSC_REG, 0x04, 0x00, 1);
  ESP_LOGI(TAG, "Soft reset completed");
}

bool dwm3000_check_idle(dwm3000_t *dev) {
  uint32_t status1 = dwm3000_read(dev, 0x0F, 0x30);
  uint32_t status2 = dwm3000_read(dev, 0x00, 0x44);

  bool idle1 = ((status1 >> 16) & PMSC_STATE_IDLE) == PMSC_STATE_IDLE;
  bool idle2 = ((status2 >> 16) & (SPIRDY_MASK | RCINIT_MASK)) ==
               (SPIRDY_MASK | RCINIT_MASK);

  return idle1 || idle2;
}

bool dwm3000_check_spi(dwm3000_t *dev) { return check_for_dev_id(dev); }

void dwm3000_clear_status(dwm3000_t *dev) {
  dwm3000_write(dev, GEN_CFG_AES_LOW_REG, 0x44, 0x3F7FFFFF, 0);
}

uint32_t dwm3000_read_sys_status(dwm3000_t *dev) {
  return dwm3000_read(dev, GEN_CFG_AES_LOW_REG, 0x44);
}

void dwm3000_set_antenna_delay(dwm3000_t *dev, int delay) {
  dev->antenna_delay = delay;
  dwm3000_write(dev, 0x01, 0x04, delay, 0);
}

int dwm3000_get_antenna_delay(dwm3000_t *dev) {
  return dwm3000_read(dev, 0x01, 0x04) & 0xFFFF;
}

void dwm3000_set_sender_id(dwm3000_t *dev, int id) { dev->sender_id = id; }

void dwm3000_set_destination_id(dwm3000_t *dev, int id) {
  dev->destination_id = id;
}

int dwm3000_get_sender_id(dwm3000_t *dev) {
  return dwm3000_read(dev, 0x12, 0x01) & 0xFF;
}

int dwm3000_get_destination_id(dwm3000_t *dev) {
  return dwm3000_read(dev, 0x12, 0x02) & 0xFF;
}

uint64_t dwm3000_read_rx_timestamp(dwm3000_t *dev) {
  uint32_t ts_low = dwm3000_read(dev, 0x0C, 0x00);
  uint64_t ts_high = dwm3000_read(dev, 0x0C, 0x04) & 0xFF;
  uint64_t rx_timestamp = (ts_high << 32) | ts_low;
  return rx_timestamp;
}

uint64_t dwm3000_read_tx_timestamp(dwm3000_t *dev) {
  uint64_t ts_low = dwm3000_read(dev, 0x00, 0x74);
  uint64_t ts_high = dwm3000_read(dev, 0x00, 0x78) & 0xFF;
  uint64_t tx_timestamp = (ts_high << 32) + ts_low;
  return tx_timestamp;
}

int dwm3000_received_frame_succ(dwm3000_t *dev) {
  uint32_t sys_stat = dwm3000_read(dev, GEN_CFG_AES_LOW_REG, 0x44);
  if ((sys_stat & SYS_STATUS_FRAME_RX_SUCC) > 0) {
    return 1;
  } else if ((sys_stat & SYS_STATUS_RX_ERR) > 0) {
    return 2;
  }
  return 0;
}

int dwm3000_sent_frame_succ(dwm3000_t *dev) {
  uint32_t sys_stat = dwm3000_read(dev, GEN_CFG_AES_LOW_REG, 0x44);
  if ((sys_stat & SYS_STATUS_FRAME_TX_SUCC) == SYS_STATUS_FRAME_TX_SUCC) {
    return 1;
  }
  return 0;
}

double dwm3000_get_signal_strength(dwm3000_t *dev) {
  int CIR_power = dwm3000_read(dev, 0x0C, 0x2C) & 0x1FF;
  int PAC_val = dwm3000_read(dev, 0x0C, 0x58) & 0xFFF;
  unsigned int DGC_decision = (dwm3000_read(dev, 0x03, 0x60) >> 28) & 0x7;
  double PRF_const = 121.7;

  return 10 * log10((CIR_power * (1 << 21)) / pow(PAC_val, 2)) +
         (6 * DGC_decision) - PRF_const;
}

double dwm3000_get_fp_signal_strength(dwm3000_t *dev) {
  float f1 = (dwm3000_read(dev, 0x0C, 0x30) & 0x3FFFFF) >> 2;
  float f2 = (dwm3000_read(dev, 0x0C, 0x34) & 0x3FFFFF) >> 2;
  float f3 = (dwm3000_read(dev, 0x0C, 0x38) & 0x3FFFFF) >> 2;
  int PAC_val = dwm3000_read(dev, 0x0C, 0x58) & 0xFFF;
  unsigned int DGC_decision = (dwm3000_read(dev, 0x03, 0x60) >> 28) & 0x7;
  double PRF_const = 121.7;

  return 10 * log10((pow(f1, 2) + pow(f2, 2) + pow(f3, 2)) / pow(PAC_val, 2)) +
         (6 * DGC_decision) - PRF_const;
}

int dwm3000_get_raw_clock_offset(dwm3000_t *dev) {
  int raw_offset = dwm3000_read(dev, 0x06, 0x29) & 0x1FFFFF;
  if (raw_offset & (1 << 20)) {
    raw_offset |= ~((1 << 21) - 1);
  }
  return raw_offset;
}

long double dwm3000_get_clock_offset(dwm3000_t *dev, int32_t ext_clock_offset) {
  if (dev->config.channel == CHANNEL_5) {
    return ext_clock_offset * CLOCK_OFFSET_CHAN_5_CONSTANT / 1000000;
  } else {
    return ext_clock_offset * CLOCK_OFFSET_CHAN_9_CONSTANT / 1000000;
  }
}

float dwm3000_get_temp_c(dwm3000_t *dev) {
  dwm3000_write(dev, 0x07, 0x34, 0x04, 0);
  dwm3000_write(dev, 0x08, 0x00, 0x01, 0);

  while (!(dwm3000_read(dev, 0x08, 0x04) & 0x01)) {
    vTaskDelay(1);
  }

  int res = dwm3000_read(dev, 0x08, 0x08);
  res = (res & 0xFF00) >> 8;
  int otp_temp = dwm3000_read_otp(dev, 0x09) & 0xFF;
  float tmp = (float)((res - otp_temp) * 1.05f) + 22.0f;
  dwm3000_write(dev, 0x08, 0x00, 0x00, 1);

  return tmp;
}

double dwm3000_convert_to_cm(int ps_units) {
  return (double)ps_units * PS_UNIT * SPEED_OF_LIGHT;
}

void dwm3000_set_mode(dwm3000_t *dev, int mode) {
  dwm3000_write(dev, 0x14, 0x00, mode & 0x7, 0);
}

void dwm3000_set_frame_length(dwm3000_t *dev, int frame_len) {
  frame_len = frame_len + FCS_LEN;
  uint32_t curr_cfg = dwm3000_read(dev, 0x00, 0x24);

  if (frame_len > 1023) {
    ESP_LOGE(TAG, "Frame length + FCS_LEN (2) is longer than 1023");
    return;
  }

  uint32_t tmp_cfg = (curr_cfg & 0xFFFFFC00) | frame_len;
  dwm3000_write(dev, GEN_CFG_AES_LOW_REG, 0x24, tmp_cfg, 0);
}

void dwm3000_standard_tx(dwm3000_t *dev) { write_fast_command(dev, 0x01); }

void dwm3000_standard_rx(dwm3000_t *dev) { write_fast_command(dev, 0x02); }

void dwm3000_tx_instant_rx(dwm3000_t *dev) { write_fast_command(dev, 0x0C); }

void dwm3000_setup_gpio(dwm3000_t *dev) {
  dwm3000_write(dev, 0x05, 0x08, 0xF0, 0);
}

void dwm3000_configure_as_tx(dwm3000_t *dev) {
  dwm3000_write(dev, RF_CONF_REG, 0x1C, 0x34, 0);
  dwm3000_write(dev, GEN_CFG_AES_HIGH_REG, 0x0C, 0xFDFDFDFD, 0);
}

// To be continued in next part...

// ============================================================================
// System Configuration
// ============================================================================

/**
 * @brief Write system configuration to DWM3000
 */
static void write_sys_config(dwm3000_t *dev) {
  dwm3000_config_t *config = &dev->config;

  int usr_cfg = (STDRD_SYS_CONFIG & 0xFFF) | (config->phr_mode << 3) |
                (config->phr_rate << 4);
  dwm3000_write(dev, GEN_CFG_AES_LOW_REG, 0x10, usr_cfg, 0);

  if (config->preamble_code > 24) {
    ESP_LOGE(TAG, "TX & RX Preamble Code higher than 24!");
  }

  int otp_write = 0x1400;
  if (config->preamble_length >= 256) {
    otp_write |= 0x04;
  }

  dwm3000_write(dev, OTP_IF_REG, 0x08, otp_write, 0);
  dwm3000_write(dev, DRX_REG, 0x00, 0x00, 1);
  dwm3000_write(dev, DRX_REG, 0x0, config->pac_size, 0);
  dwm3000_write(dev, STS_CFG_REG, 0x0, 64 / 8 - 1, 0);
  dwm3000_write(dev, GEN_CFG_AES_LOW_REG, 0x29, 0x00, 1);
  dwm3000_write(dev, DRX_REG, 0x0C, 0xAF5F584C, 0);

  int chan_ctrl_val = dwm3000_read(dev, GEN_CFG_AES_HIGH_REG, 0x14);
  chan_ctrl_val &= (~0x1FFF);
  chan_ctrl_val |= config->channel;
  chan_ctrl_val |= 0x1F00 & (config->preamble_code << 8);
  chan_ctrl_val |= 0xF8 & (config->preamble_code << 3);
  chan_ctrl_val |= 0x06 & (0x01 << 1);
  dwm3000_write(dev, GEN_CFG_AES_HIGH_REG, 0x14, chan_ctrl_val, 0);

  int tx_fctrl_val = dwm3000_read(dev, GEN_CFG_AES_LOW_REG, 0x24);
  tx_fctrl_val |= (config->preamble_length << 12);
  tx_fctrl_val |= (config->data_rate << 10);
  dwm3000_write(dev, GEN_CFG_AES_LOW_REG, 0x24, tx_fctrl_val, 0);
  dwm3000_write(dev, DRX_REG, 0x02, 0x81, 0);

  int rf_tx_ctrl_2 = 0x1C071134;
  int pll_conf = 0x0F3C;

  if (config->channel) {
    rf_tx_ctrl_2 &= ~0x00FFFF;
    rf_tx_ctrl_2 |= 0x000001;
    pll_conf &= 0x00FF;
    pll_conf |= 0x001F;
  }

  dwm3000_write(dev, RF_CONF_REG, 0x1C, rf_tx_ctrl_2, 0);
  dwm3000_write(dev, FS_CTRL_REG, 0x00, pll_conf, 0);
  dwm3000_write(dev, RF_CONF_REG, 0x51, 0x14, 0);
  dwm3000_write(dev, RF_CONF_REG, 0x1A, 0x0E, 0);
  dwm3000_write(dev, FS_CTRL_REG, 0x08, 0x81, 0);
  dwm3000_write(dev, GEN_CFG_AES_LOW_REG, 0x44, 0x02, 0);
  dwm3000_write(dev, PMSC_REG, 0x04, 0x300200, 0);
  dwm3000_write(dev, PMSC_REG, 0x08, 0x0138, 0);

  // Wait for PLL lock
  int success = 0;
  for (int i = 0; i < 100; i++) {
    if (dwm3000_read(dev, GEN_CFG_AES_LOW_REG, 0x0) & 0x2) {
      success = 1;
      break;
    }
  }

  if (!success) {
    ESP_LOGE(TAG, "Couldn't lock PLL Clock!");
  } else {
    ESP_LOGI(TAG, "PLL is now locked");
  }

  int otp_val = dwm3000_read(dev, OTP_IF_REG, 0x08);
  otp_val |= 0x40;
  if (config->channel)
    otp_val |= 0x2000;

  dwm3000_write(dev, OTP_IF_REG, 0x08, otp_val, 0);
  dwm3000_write(dev, RX_TUNE_REG, 0x19, 0xF0, 0);

  int ldo_ctrl_val = dwm3000_read(dev, RF_CONF_REG, 0x48);
  int tmp_ldo = (0x105 | 0x100 | 0x4 | 0x1);

  dwm3000_write(dev, RF_CONF_REG, 0x48, tmp_ldo, 0);
  dwm3000_write(dev, EXT_SYNC_REG, 0x0C, 0x020000, 0);
  dwm3000_read(dev, 0x04, 0x0C); // .ino: int l = read(0x04, 0x0C)
  vTaskDelay(pdMS_TO_TICKS(20));
  dwm3000_write(dev, EXT_SYNC_REG, 0x0C, 0x11, 0);

  // Wait for calibration
  int succ = 0;
  for (int i = 0; i < 100; i++) {
    if (dwm3000_read(dev, EXT_SYNC_REG, 0x20)) {
      succ = 1;
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  if (succ) {
    ESP_LOGI(TAG, "PGF calibration complete");
  } else {
    ESP_LOGE(TAG, "PGF calibration failed!");
  }

  dwm3000_write(dev, EXT_SYNC_REG, 0x0C, 0x00, 0);
  dwm3000_write(dev, EXT_SYNC_REG, 0x20, 0x01, 0);

  int rx_cal_res = dwm3000_read(dev, EXT_SYNC_REG, 0x14);
  if (rx_cal_res == 0x1fffffff) {
    ESP_LOGE(TAG, "PGF_CAL failed in stage I!");
  }
  rx_cal_res = dwm3000_read(dev, EXT_SYNC_REG, 0x1C);
  if (rx_cal_res == 0x1fffffff) {
    ESP_LOGE(TAG, "PGF_CAL failed in stage Q!");
  }

  dwm3000_write(dev, RF_CONF_REG, 0x48, ldo_ctrl_val, 0);
  dwm3000_write(dev, 0x0E, 0x02, 0x01, 0);
  dwm3000_set_antenna_delay(dev, dev->antenna_delay);
}

/**
 * @brief Initialize SPI connection to DWM3000 (equivalent to .ino begin())
 */
esp_err_t dwm3000_begin(dwm3000_t *dev, spi_host_device_t spi_host,
                        gpio_num_t cs_pin, gpio_num_t rst_pin) {
  vTaskDelay(pdMS_TO_TICKS(5));

  // Store pin configuration
  dev->cs_pin = cs_pin;
  dev->rst_pin = rst_pin;
  dev->antenna_delay = 16350; // Default antenna delay
  dev->sender_id = 0;
  dev->destination_id = 0;

  // Set default configuration (matching .ino config[])
  dev->config.channel = CHANNEL_5;
  dev->config.preamble_length = PREAMBLE_128;
  dev->config.preamble_code = 9;
  dev->config.pac_size = PAC8;
  dev->config.data_rate = DATARATE_6_8MB;
  dev->config.phr_mode = PHR_MODE_STANDARD;
  dev->config.phr_rate = PHR_RATE_850KB;

  // Configure SPI device (equivalent to spiSelect)
  spi_device_interface_config_t devcfg = {
      .clock_speed_hz = 10 * 1000 * 1000, // 10 MHz
      .mode = 0,                          // SPI mode 0
      .spics_io_num = cs_pin,
      .queue_size = 7,
      .flags = SPI_DEVICE_NO_DUMMY,
  };

  esp_err_t ret = spi_bus_add_device(spi_host, &devcfg, &dev->spi);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to add SPI device");
    return ret;
  }

  vTaskDelay(pdMS_TO_TICKS(5));

  ESP_LOGI(TAG, "[INFO] SPI ready");
  return ESP_OK;
}

/**
 * @brief Initialize the DWM3000 chip (equivalent to .ino init())
 */
esp_err_t dwm3000_chip_init(dwm3000_t *dev) {
  ESP_LOGI(TAG, "\n+++ DecaWave DWM3000 Test +++\n");

  // Check device ID
  if (!check_for_dev_id(dev)) {
    ESP_LOGE(TAG, "[ERROR] Dev ID is wrong! Aborting!");
    return ESP_FAIL;
  }

  set_bit_high(dev, GEN_CFG_AES_LOW_REG, 0x10, 4);

  while (!dwm3000_check_idle(dev)) {
    ESP_LOGW(TAG, "[WARNING] IDLE FAILED (stage 1)");
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  dwm3000_soft_reset(dev);
  vTaskDelay(pdMS_TO_TICKS(200));

  while (!dwm3000_check_idle(dev)) {
    ESP_LOGW(TAG, "[WARNING] IDLE FAILED (stage 2)");
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  // Read OTP values
  uint32_t ldo_low = dwm3000_read_otp(dev, 0x04);
  uint32_t ldo_high = dwm3000_read_otp(dev, 0x05);
  uint32_t bias_tune = dwm3000_read_otp(dev, 0xA);
  bias_tune = (bias_tune >> 16) & BIAS_CTRL_BIAS_MASK;

  if (ldo_low != 0 && ldo_high != 0 && bias_tune != 0) {
    dwm3000_write(dev, 0x11, 0x1F, bias_tune, 0);
    dwm3000_write(dev, 0x0B, 0x08, 0x0100, 0);
  }

  int xtrim_value = dwm3000_read_otp(dev, 0x1E);
  xtrim_value = xtrim_value == 0 ? 0x2E : xtrim_value;
  dwm3000_write(dev, FS_CTRL_REG, 0x14, xtrim_value, 0);

  write_sys_config(dev);

  dwm3000_write(dev, 0x00, 0x3C, 0xFFFFFFFF, 0); // Set Status Enable
  dwm3000_write(dev, 0x00, 0x40, 0xFFFF, 0);

  dwm3000_write(dev, 0x0A, 0x00, 0x000900, 3); // AON_DIG_CFG

  // Set RX and TX config
  dwm3000_write(dev, 0x3, 0x1C, 0x10000240, 0); // DGC_CFG0
  dwm3000_write(dev, 0x3, 0x20, 0x1B6DA489, 0); // DGC_CFG1
  dwm3000_write(dev, 0x3, 0x38, 0x0001C0FD, 0); // DGC_LUT_0
  dwm3000_write(dev, 0x3, 0x3C, 0x0001C43E, 0); // DGC_LUT_1
  dwm3000_write(dev, 0x3, 0x40, 0x0001C6BE, 0); // DGC_LUT_2
  dwm3000_write(dev, 0x3, 0x44, 0x0001C77E, 0); // DGC_LUT_3
  dwm3000_write(dev, 0x3, 0x48, 0x0001CF36, 0); // DGC_LUT_4
  dwm3000_write(dev, 0x3, 0x4C, 0x0001CFB5, 0); // DGC_LUT_5
  dwm3000_write(dev, 0x3, 0x50, 0x0001CFF5, 0); // DGC_LUT_6
  dwm3000_write(dev, 0x3, 0x18, 0xE5E5, 0);     // THR_64
  dwm3000_read(dev, 0x4, 0x20);                 // .ino: int f = read(0x4, 0x20)

  // SET PAC TO 32
  dwm3000_write(dev, 0x6, 0x0, 0x81101C, 0);

  dwm3000_write(dev, 0x07, 0x34, 0x4, 0);        // Enable temp sensor
  dwm3000_write(dev, 0x07, 0x48, 0x14, 0);       // LDO_RLOAD
  dwm3000_write(dev, 0x07, 0x1A, 0x0E, 0);       // RF_TX_CTRL_1
  dwm3000_write(dev, 0x07, 0x1C, 0x1C071134, 0); // RF_TX_CTRL_2 (channel 5)
  dwm3000_write(dev, 0x09, 0x00, 0x1F3C, 0);     // PLL_CFG (channel 5)
  dwm3000_write(dev, 0x09, 0x80, 0x81, 0);       // PLL_CAL

  dwm3000_write(dev, 0x11, 0x04, 0xB40200, 0);
  dwm3000_write(dev, 0x11, 0x08, 0x80030738, 0);

  ESP_LOGI(TAG, "[INFO] Initialization finished.\n");

  return ESP_OK;
}

// ============================================================================
// Double-Sided Ranging Functions
// ============================================================================

void dwm3000_ds_send_frame(dwm3000_t *dev, int stage) {
  dwm3000_set_mode(dev, 1);
  dwm3000_write(dev, 0x14, 0x01, dev->sender_id & 0xFF, 0);
  dwm3000_write(dev, 0x14, 0x02, dev->destination_id & 0xFF, 0);
  dwm3000_write(dev, 0x14, 0x03, stage & 0x7, 0);
  dwm3000_set_frame_length(dev, 4);

  dwm3000_tx_instant_rx(dev); // Await response

  bool error = true;
  for (int i = 0; i < 50; i++) {
    if (dwm3000_sent_frame_succ(dev)) {
      error = false;
      break;
    }
  }

  if (error) {
    ESP_LOGE(TAG, "Could not send frame successfully!");
  }
}

void dwm3000_ds_send_rt_info(dwm3000_t *dev, int t_roundB, int t_replyB) {
  dwm3000_set_mode(dev, 1);
  dwm3000_write(dev, 0x14, 0x01, dev->destination_id & 0xFF, 0);
  dwm3000_write(dev, 0x14, 0x02, dev->sender_id & 0xFF, 0);
  dwm3000_write(dev, 0x14, 0x03, 4, 0);
  dwm3000_write(dev, 0x14, 0x04, t_roundB, 0);
  dwm3000_write(dev, 0x14, 0x08, t_replyB, 0);
  dwm3000_set_frame_length(dev, 12);
  dwm3000_tx_instant_rx(dev);
}

int dwm3000_ds_process_rt_info(dwm3000_t *dev, int t_roundA, int t_replyA,
                               int t_roundB, int t_replyB, int clk_offset) {
  int reply_diff = t_replyA - t_replyB;
  long double clock_offset =
      t_replyA > t_replyB ? 1.0 + dwm3000_get_clock_offset(dev, clk_offset)
                          : 1.0 - dwm3000_get_clock_offset(dev, clk_offset);

  int first_rt = t_roundA - t_replyB;
  int second_rt = t_roundB - t_replyA;

  int combined_rt =
      (first_rt + second_rt - (reply_diff - (reply_diff * clock_offset))) / 2;

  return combined_rt / 2; // divided by 2 to get just one range
}

int dwm3000_ds_get_stage(dwm3000_t *dev) {
  return dwm3000_read(dev, 0x12, 0x03) & 0b111;
}

bool dwm3000_ds_is_error_frame(dwm3000_t *dev) {
  return ((dwm3000_read(dev, 0x12, 0x00) & 0x7) == 7);
}

void dwm3000_ds_send_error_frame(dwm3000_t *dev) {
  ESP_LOGW(TAG, "Error Frame sent. Reverting back to stage 0");
  dwm3000_set_mode(dev, 7);
  dwm3000_set_frame_length(dev, 3);
  dwm3000_standard_tx(dev);
}
