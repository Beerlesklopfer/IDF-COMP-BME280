#include "comp_bme280.h"

/*
22    OR    SCL
23    YE    SDA
*/

BME280_INTF_RET_TYPE comp_bme280_init(struct bme280_dev *dev, struct bme280_settings *settings){ 

#ifdef CONFIG_BME280_USE_I2C
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 2, 0)

	i2c_config_t i2c_bus_config = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = I2C_MASTER_SDA_IO,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_io_num = I2C_MASTER_SCL_IO,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = I2C_MASTER_FREQ_HZ,
		.clk_flags = 0, 
	};
	
	ESP_ERROR_CHECK( i2c_param_config(I2C_MASTER_INTERFACE_NUMBER, &i2c_bus_config) );
#  else
	i2c_master_bus_config_t i2c_bus_config = {
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.i2c_port = I2C_MASTER_INTERFACE_NUMBER,
		.sda_io_num = I2C_MASTER_SDA_IO,
		.scl_io_num = I2C_MASTER_SCL_IO,
		.flags.enable_internal_pullup = true,
		.glitch_ignore_cnt = 7,
	};
	ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &i2c_bus_handle))

	i2c_device_config_t device_config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = BME280_SENSOR_ADDR,
    .scl_speed_hz = I2C_MASTER_FREQ_HZ*1000,
	};
	ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &device_config, &i2c_device_handle));

#  endif // verion check

    ESP_ERROR_CHECK( i2c_driver_install(I2C_MASTER_INTERFACE_NUMBER, i2c_bus_config.mode, 
	                   I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0) );
#endif // use i2C

	dev->write      = comp_bme280_i2c_write;
	dev->read       = comp_bme280_i2c_read;
	dev->chip_id    = BME280_SENSOR_ADDR;
	dev->delay_us   = comp_bme280_delay_us;

    ESP_ERROR_CHECK( bme280_init(dev) );
	ESP_ERROR_CHECK( bme280_get_sensor_settings(settings, dev) );

    /* Configuring the over-sampling rate, filter coefficient and standby time */
    /* Overwrite the desired settings */
    settings->filter = BME280_FILTER_COEFF_2;

    /* Over-sampling rate for humidity, temperature and pressure */
    settings->osr_h = BME280_OVERSAMPLING_1X;
    settings->osr_p = BME280_OVERSAMPLING_1X;
    settings->osr_t = BME280_OVERSAMPLING_1X;

    /* Setting the standby time */
    settings->standby_time = BME280_STANDBY_TIME_0_5_MS;

    ESP_ERROR_CHECK( bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, settings, dev) );

    /* Always set the power mode after setting the configuration */
    ESP_ERROR_CHECK( bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, dev) );

    /* Calculate measurement time in microseconds */
    ESP_ERROR_CHECK( bme280_cal_meas_delay(&bme280_polling_period, settings) );
	
	return ESP_ERR_INVALID_STATE;
}

#ifdef CONFIG_BME280_USE_I2C
BME280_INTF_RET_TYPE comp_bme280_i2c_read(uint8_t reg_addr, uint8_t *reg_data, size_t size, void *intf_ptr){

	esp_err_t rslt = 0;
	ESP_LOGW("comp_bme280_i2c_read", "READING.");

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (BME280_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK);

	if (size > 1) {
		i2c_master_read(cmd, reg_data, size-1, I2C_MASTER_ACK);
	}

	i2c_master_read_byte(cmd, reg_data+size-1, I2C_MASTER_NACK);
	i2c_master_stop(cmd);

	rslt =  i2c_master_cmd_begin(I2C_MASTER_INTERFACE_NUMBER, cmd, 10/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	return rslt;
}
#endif

#ifdef CONFIG_BME280_USE_I2C
BME280_INTF_RET_TYPE comp_bme280_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, size_t size, void *intf_ptr){

	ESP_LOGW("comp_bme280_i2c_write", "WRITING.");

	esp_err_t rslt = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK);
	i2c_master_write_byte(cmd, reg_addr, I2C_MASTER_ACK);
	i2c_master_write(cmd, reg_data, size, I2C_MASTER_ACK);
	    i2c_master_stop(cmd);

    rslt = i2c_master_cmd_begin(I2C_MASTER_INTERFACE_NUMBER, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return rslt;}
#endif

#ifdef CONFIG_BME280_USE_SPI
BME280_INTF_RET_TYPE comp_bme280_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
#endif

#ifdef CONFIG_BME280_USE_SPI
BME280_INTF_RET_TYPE comp_bme280_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
#endif


void comp_bme280_delay_us(uint32_t period_us, void *intf_ptr){
	ESP_LOGW("comp_bme280_delay_us", "WAITING.");
	const TickType_t xDelay = period_us*1000 / portTICK_PERIOD_MS;
	vTaskDelay( xDelay );
}

BME280_INTF_RET_TYPE comp_bme280_get_data(struct bme280_dev *dev, struct bme280_data *data){
	
	BME280_INTF_RET_TYPE rslt = BME280_E_NULL_PTR;
    uint8_t status_reg;
	struct bme280_data comp_data[CONFIG_BME280_SAMPLE_COUNT];

	for( uint8_t i = 0; i<CONFIG_BME280_SAMPLE_COUNT; i++)
    {
        rslt = bme280_get_regs(BME280_REG_STATUS, &status_reg, 1, dev);

        if (status_reg & BME280_STATUS_MEAS_DONE)
        {
            /* Measurement time delay given to read sample */
            dev->delay_us(bme280_polling_period, dev->intf_ptr);

            /* Read compensated data */
            rslt = bme280_get_sensor_data(BME280_TEMP, &comp_data[i], dev);
            i++;
        }
#ifndef BME280_AVG_MEDIAN
		data->temperature += comp_data->temperature;
		data->pressure    += comp_data->pressure;
		data->humidity    += comp_data->humidity;
#endif
    }

#ifdef BME280_AVG_MEDIAN
    static struct bme280_data median(struct bme280_data *data){
        return -1;
    }
#else
		data->temperature = comp_data->temperature/CONFIG_BME280_SAMPLE_COUNT;
		data->pressure    = comp_data->pressure/CONFIG_BME280_SAMPLE_COUNT;
		data->humidity    = comp_data->humidity/CONFIG_BME280_SAMPLE_COUNT;
#endif

    return rslt;
}
