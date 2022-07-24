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
****************************************************************************/

/****************************************************************************
** Modified by Seongheon Hong - BRL UROP Student (Feb 23, 2022)
** Compatible with Arduino IDE & Wire library
****************************************************************************/

#ifndef LOADCELL4_HSH
#define LOADCELL4_HSH

#include <Wire.h>

/**
 * @brief Load Cell 4 description setting.
 * @details Specified setting for description of Load Cell 4 Click driver.
 */
#define LOADCELL4_GET_RESULT_ERROR 0
#define LOADCELL4_GET_RESULT_OK 1
#define LOADCELL4_DATA_NO_DATA 0
#define LOADCELL4_DATA_OK 1
#define LOADCELL4_WEIGHT_100G 100
#define LOADCELL4_WEIGHT_500G 500
#define LOADCELL4_WEIGHT_1000G 1000
#define LOADCELL4_WEIGHT_5000G 5000
#define LOADCELL4_WEIGHT_10000G 10000
#define LOADCELL4_DEFAULT_WEIGHT_SCALE_COEFFICIENT 0.088495575221

/**
 * @brief Load Cell 4 eeprom setting.
 * @details Specified eeprom setting of Load Cell 4 Click driver.
 */
#define LOADCELL4_EEPROM_ID_0 0x00
#define LOADCELL4_EEPROM_CONFIG_1 0x01
#define LOADCELL4_EEPROM_CONFIG_2 0x02
#define LOADCELL4_EEPROM_OFFSET_BRIDGE 0x03
#define LOADCELL4_EEPROM_GAIN_BRIDGE 0x04
#define LOADCELL4_EEPROM_TEMP_COF_GAIN 0x05
#define LOADCELL4_EEPROM_TEMP_COF_OFFSET 0x06
#define LOADCELL4_EEPROM_2ND_ORDER_TEMP_COF_OFFSET 0x07
#define LOADCELL4_EEPROM_2ND_ORDER_TEMP_COF_GAIN 0x08
#define LOADCELL4_EEPROM_2ND_ORDER_BRIDGE 0x09
#define LOADCELL4_EEPROM_OFFSET_TEMP 0x0A
#define LOADCELL4_EEPROM_GAINT_TEMP 0x0B
#define LOADCELL4_EEPROM_2ND_ORDER_TEMP 0x0C
#define LOADCELL4_EEPROM_TEMP_SETL 0x0D
#define LOADCELL4_EEPROM_ID_1 0x0E
#define LOADCELL4_EEPROM_BRIDGE_CONFIG 0x0F
#define LOADCELL4_EEPROM_TEMP_CONFIG 0x10
#define LOADCELL4_EEPROM_OSC_TRIM 0x11
#define LOADCELL4_EEPROM_SIGNATURE 0x12
#define LOADCELL4_EEPROM_ID_2 0x13
#define LOADCELL4_EEPROM_WRITE_CMD 0x40
#define LOADCELL4_EEPROM_STATUS_READ_READY 0x5A

/**
 * @brief Load Cell 4 status.
 * @details Status settings of Load Cell 4 Click driver.
 */
#define LOADCELL4_STATUS_NORMAL 0x00
#define LOADCELL4_STATUS_CMD_MODE 0x01
#define LOADCELL4_STATUS_STALE_DATA 0x02
#define LOADCELL4_STATUS_ERROR 0x03
#define LOADCELL4_STATUS_BIT_MASK 0x03
#define LOADCELL4_BRIDGE_RES 0x3FFF

/**
 * @brief Load Cell 4 dummy.
 * @details Dummy data for Load Cell 4 Click driver.
 */
#define LOADCELL4_DUMMY 0x0000

/**
 * @brief Load Cell 4 power setting.
 * @details Specified power setting for Load Cell 4 Click driver.
 */
#define LOADCELL4_PWR_OFF 0x00
#define LOADCELL4_PWR_ON 0x01

/**
 * @brief Load Cell 4 zero weight.
 * @details Zero weight setting for Load Cell 4 Click driver.
 */
#define LOADCELL4_WEIGHT_ZERO 0.000

/**
 * @brief Load Cell 4 number of average values.
 * @details Number of average values used for calibrating
 * for Load Cell 4 Click driver.
 */
#define LOADCELL4_NUMB_OF_SUM_AVG_20 20.00
#define LOADCELL4_NUMB_OF_SUM_AVG_100 100.0

/**
 * @brief Load Cell 4 command mode.
 * @details Command mode setting for Load Cell 4 Click driver.
 */
#define LOADCELL4_CMD_MODE_START 0xA0
#define LOADCELL4_CMD_MODE_STOP 0x80

/**
 * @brief Load Cell 4 device address setting.
 * @details Specified setting for device slave address selection of
 * Load Cell 4 Click driver.
 */
#define LOADCELL4_SET_DEV_ADDR 0x28

/**
 * @brief MikroE LoadCell4 Class for Arduino
 * @details Test @n Requirements: Wire Library 
 * @code
 *    //Example Code
 *    Wire.begin();
 *    LoadCell4_HSH lc4; 
 *    lc4.setWireObj(&Wire);
 *    lc4.cfg_setup(LC4_INTPIN[i], LC4_ENPIN[i], 100000, LC4_I2C_ADDRESS[i]);
 *    lc4.init();
 *    lc4.restartAndWait(1000);
 *    float raw_data = lc4.get_raw_weight_data(1);
 * @endcode
 * @author  Seongheon Hong
 * @version 1.0 
 * @date July 25th, 2022
 */
class LoadCell4_HSH {
    /**
     * @brief Load Cell 4 Click context object.
     * @details Context object definition of Load Cell 4 Click driver.
     */
    typedef struct {
        // Output pins
        uint8_t en; /**< Enable. */

        // Input pins
        uint8_t int_pin; /**< Interrupt. */

        // I2C slave address
        uint8_t slave_address; /**< Device slave address (used for I2C driver). */

    } loadcell4_t;

    /**
     * @brief Load Cell 4 Click configuration object.
     * @details Configuration object definition of Load Cell 4 Click driver.
     */
    typedef struct {
        uint8_t scl; /**< Clock pin descriptor for I2C driver. */
        uint8_t sda; /**< Bidirectional data pin descriptor for I2C driver. */

        uint8_t en;      /**< Enable. */
        uint8_t int_pin; /**< Interrupt. */

        uint32_t i2c_speed;  /**< I2C serial speed. */
        uint8_t i2c_address; /**< I2C slave address. */

    } loadcell4_cfg_t;

    /**
     * @brief Load Cell 4 Click data object.
     * @details Data object definition of Load Cell 4 Click driver.
     */
    typedef struct {
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
    typedef enum {
        LOADCELL4_OK = 0,
        LOADCELL4_ERROR = -1

    } loadcell4_return_value_t;

   public:
    LoadCell4_HSH();
    void setWireObj(TwoWire* wire_obj);
    void changeWireSpeed(unsigned int wireSpeed);
    void restartAndWait(unsigned int delayMillis);
    uint8_t init(void);
    void default_cfg_setup(uint8_t intpin, uint8_t enpin);
    void cfg_setup(uint8_t intpin, uint8_t enpin, uint32_t i2c_speed, uint8_t i2c_slave_address);
    float get_weight();
    void read_raw(void);
    uint8_t default_cfg(void);
    uint8_t generic_write(uint8_t reg, uint8_t* tx_buf, uint8_t tx_len);
    uint8_t generic_read(uint8_t reg, uint8_t* rx_buf, uint8_t rx_len);
    void read_raw(uint32_t* rx_data);
    uint8_t read_data(uint16_t* bridge_data, int16_t* temperature_data);
    uint16_t read_bridge_data(void);
    void start_cmd_mode(void);
    void end_cmd_mode(void);
    void power_dev(uint8_t power_state);
    void write_eeprom(uint8_t cmd_byte, uint16_t data_word);
    uint16_t read_eeprom(uint8_t cmd_byte);
    uint8_t get_int(void);
    void tare(void);
    void tare_skip(void);
    uint8_t calibration(uint16_t cal_val);
    uint8_t calibration_skip(uint16_t cal_val);
    float get_raw_weight_data(uint8_t shotCounts);

   private:
    loadcell4_t ctx;
    loadcell4_data_t cell_data;
    loadcell4_cfg_t cfg;
    TwoWire* I2CWire;
    /**
     * @brief Hardware reset function.
     * @details The function performs hardware reset with delay of 40 ms.
     */
    void dev_hw_reset(void);

    /**
     * @brief Measurement delay function.
     * @details The function performs the measurement delay of 1 ms.
     */
    void dev_measure_delay(void);

    /**
     * @brief Power ON/OFF delay function.
     * @details The function performs the power ON/OFF delay of 1 ms.
     */
    void dev_pwr_delay(void);

    /**
     * @brief Reset delay function.
     * @details The function performs the reset delay of 10 ms.
     */
    void dev_reset_delay(void);

    /**
     * @brief Generic write data function.
     * @param[in] cmd_byte : 8-bit command byte.
     * @param[in] write_word : 16-bit write data.
     */
    void dev_i2c_write(uint8_t cmd_byte, uint16_t write_word);

    /**
     * @brief Generic read data function.
     * @param[out] read_buf : Pointer to the memory location where data be stored.
     */
    void dev_i2c_read(uint8_t* read_buf);

    /**
     * @brief Read eeprom data function.
     * @param[in] cmd_byte : 8-bit command byte.
     * @param[out] read_word : Pointer to the memory location where data be stored.
     */
    uint8_t dev_i2c_read_eeprom(uint8_t cmd_data, uint16_t* read_word);
};
#endif