#include <Loadcell4_HSH.h>

// --------------------------------------------------------- PUBLIC FUNCTIONS
void LoadCell4_HSH::default_cfg_setup(uint8_t intpin, uint8_t enpin) {
    cfg_setup(intpin, enpin, 1000000, LOADCELL4_SET_DEV_ADDR);
}

void LoadCell4_HSH::cfg_setup(uint8_t intpin, uint8_t enpin, uint32_t i2c_speed, uint8_t i2c_slave_address) {
    cfg.en = enpin;
    cfg.int_pin = intpin;
    cfg.i2c_speed = i2c_speed;
    cfg.i2c_address = i2c_slave_address;
}

uint8_t LoadCell4_HSH::init() {
    ctx.slave_address = cfg.i2c_address;
    ctx.en = cfg.en;
    ctx.int_pin = cfg.int_pin;
    pinMode(ctx.int_pin, INPUT);
    pinMode(ctx.en, OUTPUT);
    return 1;
}

LoadCell4_HSH::LoadCell4_HSH() { this->I2CWire = &Wire; }

void LoadCell4_HSH::setWireObj(TwoWire *wire_obj) { this->I2CWire = wire_obj; }

void LoadCell4_HSH::changeWireSpeed(unsigned int wireSpeed) {
    this->I2CWire->end();
    this->I2CWire->setClock(wireSpeed);
    this->I2CWire->begin();
    this->dev_hw_reset();
}

void LoadCell4_HSH::restartAndWait(unsigned int delayMillis) {
    power_dev(LOADCELL4_PWR_OFF);
    delay(delayMillis);
    power_dev(LOADCELL4_PWR_ON);
    delay(delayMillis);
}

void LoadCell4_HSH::read_raw(uint32_t *rx_data) {
    uint8_t rx_buf[4];
    uint32_t tmp;

    dev_i2c_read(rx_buf);

    tmp = rx_buf[0];
    tmp <<= 8;
    tmp |= rx_buf[1];
    tmp <<= 8;
    tmp |= rx_buf[2];
    tmp <<= 8;
    tmp |= rx_buf[3];

    *rx_data = tmp;
}

uint8_t LoadCell4_HSH::read_data(uint16_t *bridge_data, int16_t *temperature_data) {
    uint32_t raw_data;
    uint8_t status_data;
    uint16_t temp_data;

    read_raw(&raw_data);

    status_data = raw_data >> 30;
    status_data &= LOADCELL4_STATUS_BIT_MASK;

    temp_data = raw_data >> 16;
    temp_data &= LOADCELL4_BRIDGE_RES;
    *bridge_data = temp_data;

    temp_data = raw_data;
    temp_data >>= 5;
    *temperature_data = temp_data;

    return status_data;
}

uint16_t LoadCell4_HSH::read_bridge_data() {
    uint32_t raw_data;
    uint8_t status_data;
    uint16_t bridge_data;

    status_data = LOADCELL4_STATUS_ERROR;

    while (status_data != LOADCELL4_STATUS_NORMAL) {
        read_raw(&raw_data);
        dev_measure_delay();

        status_data = raw_data >> 30;
        status_data &= LOADCELL4_STATUS_BIT_MASK;

        bridge_data = raw_data >> 16;
        bridge_data &= LOADCELL4_BRIDGE_RES;
    }

    return bridge_data;
}

void LoadCell4_HSH::start_cmd_mode() { dev_i2c_write(LOADCELL4_CMD_MODE_START, LOADCELL4_DUMMY); }

void LoadCell4_HSH::end_cmd_mode() { dev_i2c_write(LOADCELL4_CMD_MODE_STOP, LOADCELL4_DUMMY); }

void LoadCell4_HSH::power_dev(uint8_t power_state) {
    if (power_state == LOADCELL4_PWR_OFF) {
        digitalWrite(ctx.en, LOW);
    } else {
        digitalWrite(ctx.en, HIGH);
    }
    dev_pwr_delay();
}

void LoadCell4_HSH::write_eeprom(uint8_t cmd_byte, uint16_t data_word) {
    uint8_t eeprom_word = cmd_byte;
    eeprom_word |= LOADCELL4_EEPROM_WRITE_CMD;
    dev_i2c_write(eeprom_word, data_word);
    /*Wire.beginTransmission( ctx->slave_address );
    Wire.write( eeprom_word );
    Wire.write( data_word >> 8 ); Wire.write( data_word & 0xFF );
    Wire.endTransmission();
    */
}

uint16_t LoadCell4_HSH::read_eeprom(uint8_t cmd_byte) {
    uint8_t status_data = 0;
    uint16_t data_word;

    while (status_data != LOADCELL4_EEPROM_STATUS_READ_READY) {
        /*Wire.beginTransmission( ctx->slave_address );
        Wire.write( eeprom_word );
        Wire.write( 0 ); Wire.write( 0 );
        Wire.endTransmission();*/
        status_data = dev_i2c_read_eeprom(cmd_byte, &data_word);
        dev_measure_delay();
    }

    return data_word;
}

uint8_t LoadCell4_HSH::get_int() { return digitalRead(ctx.int_pin); }

void LoadCell4_HSH::tare() {
    uint16_t results;
    uint8_t n_cnt;
    uint32_t sum_val;
    float average_val;

    for (n_cnt = 0; n_cnt < 5; n_cnt++) {
        dev_hw_reset();

        while (get_int() != 1)
            ;

        results = read_bridge_data();

        dev_measure_delay();
    }

    sum_val = 0;

    for (n_cnt = 0; n_cnt < 100; n_cnt++) {
        dev_hw_reset();
        results = read_bridge_data();
        sum_val += results;
        dev_measure_delay();
    }

    average_val = (float)sum_val;
    average_val /= LOADCELL4_NUMB_OF_SUM_AVG_100;

    cell_data.tare = average_val;
    cell_data.tare_ok = LOADCELL4_DATA_OK;
    cell_data.weight_data_100g_ok = LOADCELL4_DATA_NO_DATA;
    cell_data.weight_data_500g_ok = LOADCELL4_DATA_NO_DATA;
    cell_data.weight_data_1000g_ok = LOADCELL4_DATA_NO_DATA;
    cell_data.weight_data_5000g_ok = LOADCELL4_DATA_NO_DATA;
    cell_data.weight_data_10000g_ok = LOADCELL4_DATA_NO_DATA;
}
void LoadCell4_HSH::tare_skip() {
    cell_data.tare = 0;
    cell_data.tare_ok = LOADCELL4_DATA_OK;
    cell_data.weight_data_100g_ok = LOADCELL4_DATA_NO_DATA;
    cell_data.weight_data_500g_ok = LOADCELL4_DATA_NO_DATA;
    cell_data.weight_data_1000g_ok = LOADCELL4_DATA_NO_DATA;
    cell_data.weight_data_5000g_ok = LOADCELL4_DATA_NO_DATA;
    cell_data.weight_data_10000g_ok = LOADCELL4_DATA_NO_DATA;
}
uint8_t LoadCell4_HSH::calibration_skip(uint16_t cal_val) {
    uint16_t results;
    uint8_t n_cnt;
    uint32_t sum_val;
    float average_val;
    float tare_val;
    float weight_val;
    uint8_t status;

    status = LOADCELL4_OK;

    tare_val = this->cell_data.tare;
    cell_data.weight_coeff_1000g = 1.0;
    cell_data.weight_data_1000g_ok = LOADCELL4_DATA_OK;
    return status;
}

uint8_t LoadCell4_HSH::calibration(uint16_t cal_val) {
    uint16_t results;
    uint8_t n_cnt;
    uint32_t sum_val;
    float average_val;
    float tare_val;
    float weight_val;
    uint8_t status;

    status = LOADCELL4_OK;

    tare_val = cell_data.tare;

    sum_val = 0;

    for (n_cnt = 0; n_cnt < 100; n_cnt++) {
        dev_hw_reset();

        while (get_int() != 1)
            ;

        results = read_bridge_data();

        sum_val += results;

        dev_measure_delay();
    }

    average_val = (float)sum_val;
    average_val /= LOADCELL4_NUMB_OF_SUM_AVG_100;

    weight_val = average_val - tare_val;

    switch (cal_val) {
        case LOADCELL4_WEIGHT_100G: {
            cell_data.weight_coeff_100g = (float)LOADCELL4_WEIGHT_100G / weight_val;
            cell_data.weight_data_100g_ok = LOADCELL4_DATA_OK;
            break;
        }
        case LOADCELL4_WEIGHT_500G: {
            cell_data.weight_coeff_500g = (float)LOADCELL4_WEIGHT_500G / weight_val;
            cell_data.weight_data_500g_ok = LOADCELL4_DATA_OK;
            break;
        }
        case LOADCELL4_WEIGHT_1000G: {
            cell_data.weight_coeff_1000g = (float)LOADCELL4_WEIGHT_1000G / weight_val;
            cell_data.weight_data_1000g_ok = LOADCELL4_DATA_OK;
            break;
        }
        case LOADCELL4_WEIGHT_5000G: {
            cell_data.weight_coeff_5000g = (float)LOADCELL4_WEIGHT_5000G / weight_val;
            cell_data.weight_data_5000g_ok = LOADCELL4_DATA_OK;
            break;
        }
        case LOADCELL4_WEIGHT_10000G: {
            cell_data.weight_coeff_10000g = (float)LOADCELL4_WEIGHT_10000G / weight_val;
            cell_data.weight_data_10000g_ok = LOADCELL4_DATA_OK;
            break;
        }
        default: {
            status = LOADCELL4_ERROR;
            cell_data.weight_data_100g_ok = LOADCELL4_DATA_NO_DATA;
            cell_data.weight_data_500g_ok = LOADCELL4_DATA_NO_DATA;
            cell_data.weight_data_1000g_ok = LOADCELL4_DATA_NO_DATA;
            cell_data.weight_data_5000g_ok = LOADCELL4_DATA_NO_DATA;
            cell_data.weight_data_10000g_ok = LOADCELL4_DATA_NO_DATA;
            break;
        }
    }

    return status;
}

float LoadCell4_HSH::get_weight() {
    uint16_t results;
    uint8_t n_cnt;
    uint32_t sum_val;
    float average_val;
    float tare_val;
    float weight_val;
    uint8_t status;

    status = LOADCELL4_GET_RESULT_OK;

    tare_val = cell_data.tare;

    sum_val = 0;

    for (n_cnt = 0; n_cnt < 100; n_cnt++) {
        dev_hw_reset();

        while (get_int() != 1)
            ;

        results = read_bridge_data();

        sum_val += results;

        dev_measure_delay();
    }

    average_val = (float)sum_val;
    average_val /= LOADCELL4_NUMB_OF_SUM_AVG_100;

    weight_val = average_val - tare_val;

    if (cell_data.weight_data_100g_ok == LOADCELL4_DATA_OK) {
        weight_val *= cell_data.weight_coeff_100g;
        dev_pwr_delay();
    } else if (cell_data.weight_data_500g_ok == LOADCELL4_DATA_OK) {
        weight_val *= cell_data.weight_coeff_500g;
        dev_pwr_delay();
    } else if (cell_data.weight_data_1000g_ok == LOADCELL4_DATA_OK) {
        weight_val *= cell_data.weight_coeff_1000g;
        dev_pwr_delay();
    } else if (cell_data.weight_data_5000g_ok == LOADCELL4_DATA_OK) {
        weight_val *= cell_data.weight_coeff_5000g;
        dev_pwr_delay();
    } else if (cell_data.weight_data_10000g_ok == LOADCELL4_DATA_OK) {
        weight_val *= cell_data.weight_coeff_10000g;
        dev_pwr_delay();
    }

    if (weight_val < 0) {
        weight_val = LOADCELL4_WEIGHT_ZERO;
    }

    return weight_val;
}
float LoadCell4_HSH::get_raw_weight_data(uint8_t shotCounts) {
    uint16_t results;
    uint8_t n_cnt;
    uint32_t sum_val;
    float average_val;
    float weight_val;
    uint8_t status;

    status = LOADCELL4_GET_RESULT_OK;
    sum_val = 0;
    if (shotCounts > 1) {
        for (n_cnt = 0; n_cnt < shotCounts; n_cnt++) {
            results = read_bridge_data();
            sum_val += results;
            dev_measure_delay();
        }

        average_val = (float)sum_val;
        average_val /= shotCounts;

        weight_val = average_val;
        // dev_pwr_delay( );
        if (weight_val < 0) {
            weight_val = LOADCELL4_WEIGHT_ZERO;
        }
        return average_val;
    }
    results = read_bridge_data();
    weight_val = (float)results;
    return weight_val;
}

// --------------------------------------------- PRIVATE FUNCTION DEFINITIONS
void LoadCell4_HSH::dev_hw_reset() {
    digitalWrite(ctx.en, LOW);
    dev_reset_delay();
    digitalWrite(ctx.en, HIGH);
    dev_reset_delay();
}

void LoadCell4_HSH::dev_measure_delay() { delayMicroseconds(10); }

void LoadCell4_HSH::dev_pwr_delay() { delayMicroseconds(500); }

void LoadCell4_HSH::dev_reset_delay() { delay(200); }

void LoadCell4_HSH::dev_i2c_write(uint8_t cmd_byte, uint16_t write_word) {
    uint8_t write_buf[3];

    write_buf[0] = cmd_byte;
    write_buf[1] = write_word >> 8;
    write_buf[2] = write_word;

    I2CWire->beginTransmission(ctx.slave_address);
    I2CWire->write(write_buf, 3);
    I2CWire->endTransmission();
    while (!get_int())
        ;
    Serial.printf("Write Command: %02X ", write_buf[0]);
    Serial.printf("%02X ", write_buf[1]);
    Serial.printf("%02X \n", write_buf[2]);
}

void LoadCell4_HSH::dev_i2c_read(uint8_t *read_buf) {
    Wire.requestFrom(ctx.slave_address, 4);
    for (uint8_t i = 0; I2CWire->available() && (i < 4); i++) {
        read_buf[i] = I2CWire->read();
        // Serial.printf("%02X ", read_buf[i]);
    }
    // Serial.print("\n");
}

uint8_t LoadCell4_HSH::dev_i2c_read_eeprom(uint8_t cmd_data, uint16_t *read_word) {
    uint8_t read_buf[3] = {0, 0, 0};
    uint16_t temp_data;

    read_buf[0] = cmd_data;
    I2CWire->beginTransmission(ctx.slave_address);
    I2CWire->write(read_buf, 3);
    I2CWire->endTransmission();
    delay(1);
    // Serial.printf("%02X ", read_buf[0]);
    // Serial.println("REPEATING");
    I2CWire->requestFrom(ctx.slave_address, 3);
    delay(1);
    Serial.print("Read Result: ");
    for (uint8_t i = 0; I2CWire->available() && (i < 3); i++) {
        read_buf[i] = I2CWire->read();
        Serial.printf("%02X ", read_buf[i]);
    }
    Serial.print("\n");

    temp_data = read_buf[1];
    temp_data <<= 8;
    temp_data |= read_buf[2];
    *read_word = temp_data;

    return read_buf[0];
}

// ------------------------------------------------------------------------- END