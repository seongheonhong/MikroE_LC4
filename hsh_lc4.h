/****************************************************************************
** Copyright (C) 2020 MikroElektronika d.o.o.
** Contact: https://www.mikroe.com/contact
**
** Permission is hereby granted, free of charge, to any person obtaining a copy
** of this software and associated documentation files (the "Software"), to deal
** in the Software without restriction, including without limitation the rights
** to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
** copies of the Software, and to permit persons to whom the Software is
** furnished to do so, subject to the following conditions:
** The above copyright notice and this permission notice shall be
** included in all copies or substantial portions of the Software.
**
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
** EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
** OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
** IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
** DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT
** OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
**  USE OR OTHER DEALINGS IN THE SOFTWARE.
****************************************************************************/

/****************************************************************************
** Modified by Seongheon Hong - BRL UROP Student (Feb 23, 2022)
** Compatible with Arduino IDE & Wire library
****************************************************************************/

#ifndef LOADCELL4_H
#define LOADCELL4_H

#ifdef __cplusplus
//extern "C"{
#endif

#include <Wire.h>

/**
 * @brief Load Cell 4 description setting.
 * @details Specified setting for description of Load Cell 4 Click driver.
 */
#define LOADCELL4_GET_RESULT_ERROR                                     0
#define LOADCELL4_GET_RESULT_OK                                        1
#define LOADCELL4_DATA_NO_DATA                                         0
#define LOADCELL4_DATA_OK                                              1
#define LOADCELL4_WEIGHT_100G                                        100
#define LOADCELL4_WEIGHT_500G                                        500
#define LOADCELL4_WEIGHT_1000G                                      1000
#define LOADCELL4_WEIGHT_5000G                                      5000
#define LOADCELL4_WEIGHT_10000G                                    10000
#define LOADCELL4_DEFAULT_WEIGHT_SCALE_COEFFICIENT        0.088495575221

/**
 * @brief Load Cell 4 eeprom setting.
 * @details Specified eeprom setting of Load Cell 4 Click driver.
 */
#define LOADCELL4_EEPROM_ID_0                                       0x00
#define LOADCELL4_EEPROM_CONFIG_1                                   0x01
#define LOADCELL4_EEPROM_CONFIG_2                                   0x02
#define LOADCELL4_EEPROM_OFFSET_BRIDGE                              0x03
#define LOADCELL4_EEPROM_GAIN_BRIDGE                                0x04
#define LOADCELL4_EEPROM_TEMP_COF_GAIN                              0x05
#define LOADCELL4_EEPROM_TEMP_COF_OFFSET                            0x06
#define LOADCELL4_EEPROM_2ND_ORDER_TEMP_COF_OFFSET                  0x07
#define LOADCELL4_EEPROM_2ND_ORDER_TEMP_COF_GAIN                    0x08
#define LOADCELL4_EEPROM_2ND_ORDER_BRIDGE                           0x09
#define LOADCELL4_EEPROM_OFFSET_TEMP                                0x0A
#define LOADCELL4_EEPROM_GAINT_TEMP                                 0x0B
#define LOADCELL4_EEPROM_2ND_ORDER_TEMP                             0x0C
#define LOADCELL4_EEPROM_TEMP_SETL                                  0x0D
#define LOADCELL4_EEPROM_ID_1                                       0x0E
#define LOADCELL4_EEPROM_BRIDGE_CONFIG                              0x0F
#define LOADCELL4_EEPROM_TEMP_CONFIG                                0x10
#define LOADCELL4_EEPROM_OSC_TRIM                                   0x11
#define LOADCELL4_EEPROM_SIGNATURE                                  0x12
#define LOADCELL4_EEPROM_ID_2                                       0x13
#define LOADCELL4_EEPROM_WRITE_CMD                                  0x40
#define LOADCELL4_EEPROM_STATUS_READ_READY                          0x5A

/**
 * @brief Load Cell 4 status.
 * @details Status settings of Load Cell 4 Click driver.
 */
#define LOADCELL4_STATUS_NORMAL                                     0x00
#define LOADCELL4_STATUS_CMD_MODE                                   0x01
#define LOADCELL4_STATUS_STALE_DATA                                 0x02
#define LOADCELL4_STATUS_ERROR                                      0x03
#define LOADCELL4_STATUS_BIT_MASK                                   0x03
#define LOADCELL4_BRIDGE_RES                                      0x3FFF

/**
 * @brief Load Cell 4 dummy.
 * @details Dummy data for Load Cell 4 Click driver.
 */
#define LOADCELL4_DUMMY                                           0x0000

/**
 * @brief Load Cell 4 power setting.
 * @details Specified power setting for Load Cell 4 Click driver.
 */
#define LOADCELL4_PWR_OFF                                           0x00
#define LOADCELL4_PWR_ON                                            0x01

/**
 * @brief Load Cell 4 zero weight.
 * @details Zero weight setting for Load Cell 4 Click driver.
 */
#define LOADCELL4_WEIGHT_ZERO                                      0.000

/**
 * @brief Load Cell 4 number of average values.
 * @details Number of average values used for calibrating 
 * for Load Cell 4 Click driver.
 */
#define LOADCELL4_NUMB_OF_SUM_AVG_20                               20.00
#define LOADCELL4_NUMB_OF_SUM_AVG_100                              100.0

/**
 * @brief Load Cell 4 command mode.
 * @details Command mode setting for Load Cell 4 Click driver.
 */
#define LOADCELL4_CMD_MODE_START                                    0xA0
#define LOADCELL4_CMD_MODE_STOP                                     0x80

/**
 * @brief Load Cell 4 device address setting.
 * @details Specified setting for device slave address selection of
 * Load Cell 4 Click driver.
 */
#define LOADCELL4_SET_DEV_ADDR  0x28

/**
 * @brief Load Cell 4 Click context object.
 * @details Context object definition of Load Cell 4 Click driver.
 */
typedef struct
{
    // Output pins
    uint8_t  en;       /**< Enable. */

    // Input pins
    uint8_t  int_pin;       /**< Interrupt. */

    // I2C slave address
    uint8_t slave_address;                           /**< Device slave address (used for I2C driver). */

} loadcell4_t;

/**
 * @brief Load Cell 4 Click configuration object.
 * @details Configuration object definition of Load Cell 4 Click driver.
 */
typedef struct
{
    uint8_t  scl;        /**< Clock pin descriptor for I2C driver. */
    uint8_t  sda;        /**< Bidirectional data pin descriptor for I2C driver. */

    uint8_t  en;         /**< Enable. */
    uint8_t  int_pin;       /**< Interrupt. */

    uint32_t  i2c_speed;                                /**< I2C serial speed. */
    uint8_t   i2c_address;                              /**< I2C slave address. */

} loadcell4_cfg_t;

/**
 * @brief Load Cell 4 Click data object.
 * @details Data object definition of Load Cell 4 Click driver.
 */
typedef struct
{
    float tare;
    uint8_t tare_ok;
    float weight_coeff_100g;
    uint8_t weight_data_100g_ok;
    float weight_coeff_500g;
    uint8_t weight_data_500g_ok;
    float weight_coeff_1000g;
    uint8_t weight_data_1000g_ok;
    float weight_coeff_5000g;
    uint8_t weight_data_5000g_ok;
    float weight_coeff_10000g;
    uint8_t weight_data_10000g_ok;
    
} loadcell4_data_t;

/**
 * @brief Load Cell 4 Click return value data.
 * @details Predefined enum values for driver return values.
 */
typedef enum
{
   LOADCELL4_OK = 0,
   LOADCELL4_ERROR = -1

} loadcell4_return_value_t;


void loadcell4_cfg_setup ( loadcell4_cfg_t *cfg, uint8_t intpin, uint8_t enpin );
uint8_t loadcell4_init ( loadcell4_t *ctx, loadcell4_cfg_t *cfg );
uint8_t loadcell4_default_cfg ( loadcell4_t *ctx );
uint8_t loadcell4_generic_write ( loadcell4_t *ctx, uint8_t reg, uint8_t *tx_buf, uint8_t tx_len );
uint8_t loadcell4_generic_read ( loadcell4_t *ctx, uint8_t reg, uint8_t *rx_buf, uint8_t rx_len );
void loadcell4_read_raw ( loadcell4_t *ctx, uint32_t *rx_data );
uint8_t loadcell4_read_data ( loadcell4_t *ctx, uint16_t *bridge_data, int16_t *temperature_data );
uint16_t loadcell4_read_bridge_data ( loadcell4_t *ctx );
void loadcell4_start_cmd_mode ( loadcell4_t *ctx );
void loadcell4_end_cmd_mode ( loadcell4_t *ctx );
void loadcell4_power_dev ( loadcell4_t *ctx, uint8_t power_state );
void loadcell4_write_eeprom ( loadcell4_t *ctx, uint8_t cmd_byte, uint16_t data_word );
uint16_t loadcell4_read_eeprom ( loadcell4_t *ctx, uint8_t cmd_byte );
uint8_t loadcell4_get_int ( loadcell4_t *ctx );
void loadcell4_tare ( loadcell4_t *ctx, loadcell4_data_t *cell_data );
void loadcell4_tare_skip ( loadcell4_t *ctx, loadcell4_data_t *cell_data );
uint8_t loadcell4_calibration ( loadcell4_t *ctx, uint16_t cal_val, loadcell4_data_t *cell_data );
uint8_t loadcell4_calibration_skip ( loadcell4_t *ctx, uint16_t cal_val, loadcell4_data_t *cell_data );
float loadcell4_get_weight ( loadcell4_t *ctx, loadcell4_data_t *cell_data );
float loadcell4_get_raw_weight_data ( loadcell4_t *ctx, loadcell4_data_t *cell_data, uint8_t shotCounts );

#ifdef __cplusplus
//}
#endif
#endif // LOADCELL4_H
