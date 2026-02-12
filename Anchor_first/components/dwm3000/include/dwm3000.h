#ifndef DWM3000_H
#define DWM3000_H

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// DWM3000 Register Definitions
#define GEN_CFG_AES_LOW_REG 0x00
#define GEN_CFG_AES_HIGH_REG 0x01
#define STS_CFG_REG 0x02
#define RX_TUNE_REG 0x03
#define EXT_SYNC_REG 0x04
#define GPIO_CTRL_REG 0x05
#define DRX_REG 0x06
#define RF_CONF_REG 0x07
#define RF_CAL_REG 0x08
#define FS_CTRL_REG 0x09
#define AON_REG 0x0A
#define OTP_IF_REG 0x0B
#define CIA_REG1 0x0C
#define CIA_REG2 0x0D
#define CIA_REG3 0x0E
#define DIG_DIAG_REG 0x0F
#define PMSC_REG 0x11
#define RX_BUFFER_0_REG 0x12
#define RX_BUFFER_1_REG 0x13
#define TX_BUFFER_REG 0x14
#define ACC_MEM_REG 0x15
#define SCRATCH_RAM_REG 0x16
#define AES_RAM_REG 0x17
#define SET_1_2_REG 0x18
#define INDIRECT_PTR_A_REG 0x1D
#define INDIRECT_PTR_B_REG 0x1E
#define IN_PTR_CFG_REG 0x1F

// System Configuration
#define STDRD_SYS_CONFIG 0x188
#define DTUNE0_CONFIG 0x0F
#define FCS_LEN 2
#define PMSC_STATE_IDLE 0x3

// System Status
#define SYS_STATUS_FRAME_RX_SUCC 0x2000
#define SYS_STATUS_RX_ERR 0x4279000
#define SYS_STATUS_FRAME_TX_SUCC 0x80

// Preamble Lengths
#define PREAMBLE_32 4
#define PREAMBLE_64 8
#define PREAMBLE_128 5
#define PREAMBLE_256 9
#define PREAMBLE_512 11
#define PREAMBLE_1024 2
#define PREAMBLE_2048 10
#define PREAMBLE_4096 3
#define PREAMBLE_1536 6

// Channel Configuration
#define CHANNEL_5 0x0
#define CHANNEL_9 0x1

// PAC Size
#define PAC4 0x03
#define PAC8 0x00
#define PAC16 0x01
#define PAC32 0x02

// Data Rate
#define DATARATE_6_8MB 0x1
#define DATARATE_850KB 0x0

// PHR Mode
#define PHR_MODE_STANDARD 0x0
#define PHR_MODE_LONG 0x1

// PHR Rate
#define PHR_RATE_6_8MB 0x1
#define PHR_RATE_850KB 0x0

// Masks
#define SPIRDY_MASK 0x80
#define RCINIT_MASK 0x100
#define BIAS_CTRL_BIAS_MASK 0x1F

// Constants
#define TRANSMIT_DELAY 0x3B9ACA00
#define TRANSMIT_DIFF 0x1FF
#define NS_UNIT 4.0064102564102564
#define PS_UNIT 15.6500400641025641
#define SPEED_OF_LIGHT 0.029979245800 // cm per picosecond
#define CLOCK_OFFSET_CHAN_5_CONSTANT -0.5731e-3f
#define CLOCK_OFFSET_CHAN_9_CONSTANT -0.1252e-3f
#define NO_OFFSET 0x0

/**
 * @brief DWM3000 configuration structure
 */
typedef struct {
  int channel;         // CHANNEL_5 or CHANNEL_9
  int preamble_length; // PREAMBLE_xxx
  int preamble_code;   // 9-12
  int pac_size;        // PAC4, PAC8, PAC16, PAC32
  int data_rate;       // DATARATE_6_8MB or DATARATE_850KB
  int phr_mode;        // PHR_MODE_STANDARD or PHR_MODE_LONG
  int phr_rate;        // PHR_RATE_6_8MB or PHR_RATE_850KB
} dwm3000_config_t;

/**
 * @brief DWM3000 SPI configuration
 */
typedef struct {
  spi_device_handle_t spi;
  gpio_num_t cs_pin;
  gpio_num_t rst_pin;
  int antenna_delay;
  int sender_id;
  int destination_id;
  dwm3000_config_t config;
} dwm3000_t;

/**
 * @brief Initialize SPI connection to DWM3000 (equivalent to .ino begin())
 *
 * @param dev Pointer to DWM3000 device structure
 * @param spi_host SPI host device (SPI2_HOST or SPI3_HOST)
 * @param cs_pin Chip select GPIO pin
 * @param rst_pin Reset GPIO pin
 * @return esp_err_t ESP_OK on success
 */
esp_err_t dwm3000_begin(dwm3000_t *dev, spi_host_device_t spi_host,
                        gpio_num_t cs_pin, gpio_num_t rst_pin);

/**
 * @brief Initialize the DWM3000 chip (equivalent to .ino init())
 *
 * Checks device ID, sets up configuration, OTP, PLL, calibration etc.
 * Must call dwm3000_begin() first.
 *
 * @param dev Pointer to DWM3000 device structure
 * @return esp_err_t ESP_OK on success
 */
esp_err_t dwm3000_chip_init(dwm3000_t *dev);

/**
 * @brief Hard reset the DWM3000 chip
 */
void dwm3000_hard_reset(dwm3000_t *dev);

/**
 * @brief Soft reset the DWM3000 chip
 */
void dwm3000_soft_reset(dwm3000_t *dev);

/**
 * @brief Write data to DWM3000 register
 *
 * @param dev Device handle
 * @param reg_base Base register address
 * @param sub_addr Sub-address
 * @param data Data to write
 * @param data_len Data length in bytes (0 for auto)
 * @return uint32_t Response data
 */
uint32_t dwm3000_write(dwm3000_t *dev, int reg_base, int sub_addr,
                       uint32_t data, int data_len);

/**
 * @brief Read data from DWM3000 register
 *
 * @param dev Device handle
 * @param reg_base Base register address
 * @param sub_addr Sub-address
 * @return uint32_t Read data
 */
uint32_t dwm3000_read(dwm3000_t *dev, int reg_base, int sub_addr);

/**
 * @brief Read 8-bit data from register
 */
uint8_t dwm3000_read8bit(dwm3000_t *dev, int reg_base, int sub_addr);

/**
 * @brief Read OTP memory
 */
uint32_t dwm3000_read_otp(dwm3000_t *dev, uint8_t addr);

/**
 * @brief Check if device is in IDLE state
 */
bool dwm3000_check_idle(dwm3000_t *dev);

/**
 * @brief Check SPI connection and device ID
 */
bool dwm3000_check_spi(dwm3000_t *dev);

/**
 * @brief Clear system status register
 */
void dwm3000_clear_status(dwm3000_t *dev);

/**
 * @brief Read system status register
 */
uint32_t dwm3000_read_sys_status(dwm3000_t *dev);

/**
 * @brief Set TX antenna delay
 */
void dwm3000_set_antenna_delay(dwm3000_t *dev, int delay);

/**
 * @brief Get TX antenna delay
 */
int dwm3000_get_antenna_delay(dwm3000_t *dev);

/**
 * @brief Set sender ID
 */
void dwm3000_set_sender_id(dwm3000_t *dev, int id);

/**
 * @brief Set destination ID
 */
void dwm3000_set_destination_id(dwm3000_t *dev, int id);

/**
 * @brief Get sender ID from received frame
 */
int dwm3000_get_sender_id(dwm3000_t *dev);

/**
 * @brief Get destination ID from received frame
 */
int dwm3000_get_destination_id(dwm3000_t *dev);

// Double-Sided Ranging Functions

/**
 * @brief Send ranging frame
 *
 * @param dev Device handle
 * @param stage Ranging stage (1-4)
 */
void dwm3000_ds_send_frame(dwm3000_t *dev, int stage);

/**
 * @brief Send round-trip timing information
 */
void dwm3000_ds_send_rt_info(dwm3000_t *dev, int t_roundB, int t_replyB);

/**
 * @brief Process round-trip timing information and calculate distance
 *
 * @return int Ranging time in DWM3000 units (~15.65ps)
 */
int dwm3000_ds_process_rt_info(dwm3000_t *dev, int t_roundA, int t_replyA,
                               int t_roundB, int t_replyB, int clock_offset);

/**
 * @brief Get stage from received frame
 */
int dwm3000_ds_get_stage(dwm3000_t *dev);

/**
 * @brief Check if received frame is an error frame
 */
bool dwm3000_ds_is_error_frame(dwm3000_t *dev);

/**
 * @brief Send error frame
 */
void dwm3000_ds_send_error_frame(dwm3000_t *dev);

// Timestamp Functions

/**
 * @brief Read RX timestamp
 */
uint64_t dwm3000_read_rx_timestamp(dwm3000_t *dev);

/**
 * @brief Read TX timestamp
 */
uint64_t dwm3000_read_tx_timestamp(dwm3000_t *dev);

// Status Check Functions

/**
 * @brief Check if frame was received successfully
 *
 * @return int 0=no frame, 1=success, 2=error
 */
int dwm3000_received_frame_succ(dwm3000_t *dev);

/**
 * @brief Check if frame was sent successfully
 *
 * @return int 1=success, 0=not sent yet
 */
int dwm3000_sent_frame_succ(dwm3000_t *dev);

// Radio Analytics

/**
 * @brief Get signal strength in dBm
 */
double dwm3000_get_signal_strength(dwm3000_t *dev);

/**
 * @brief Get first path signal strength in dBm
 */
double dwm3000_get_fp_signal_strength(dwm3000_t *dev);

/**
 * @brief Get clock offset
 */
long double dwm3000_get_clock_offset(dwm3000_t *dev, int32_t ext_clock_offset);

/**
 * @brief Get raw clock offset value
 */
int dwm3000_get_raw_clock_offset(dwm3000_t *dev);

/**
 * @brief Get temperature in Celsius
 */
float dwm3000_get_temp_c(dwm3000_t *dev);

// Conversion Functions

/**
 * @brief Convert DWM3000 time units to centimeters
 *
 * @param ps_units Time in DWM3000 picosecond units
 * @return double Distance in centimeters
 */
double dwm3000_convert_to_cm(int ps_units);

// Mode Control Functions

/**
 * @brief Set frame mode
 */
void dwm3000_set_mode(dwm3000_t *dev, int mode);

/**
 * @brief Set frame length
 */
void dwm3000_set_frame_length(dwm3000_t *dev, int frame_len);

/**
 * @brief Standard TX mode
 */
void dwm3000_standard_tx(dwm3000_t *dev);

/**
 * @brief Standard RX mode
 */
void dwm3000_standard_rx(dwm3000_t *dev);

/**
 * @brief TX then instant RX mode
 */
void dwm3000_tx_instant_rx(dwm3000_t *dev);

/**
 * @brief Setup GPIO pins
 */
void dwm3000_setup_gpio(dwm3000_t *dev);

/**
 * @brief Configure as TX device
 */
void dwm3000_configure_as_tx(dwm3000_t *dev);

#ifdef __cplusplus
}
#endif

#endif // DWM3000_H
