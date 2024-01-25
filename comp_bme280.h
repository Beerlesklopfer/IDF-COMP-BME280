#ifndef _MAIN_H
#define _MAIN_H

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>
#include "esp_log.h"
#include "esp_idf_version.h"

#ifdef CONFIG_BME280_USE_I2C
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 2, 0)
    #include "driver/i2c.h"
#else
#warning this component is suited only for versions prior 5.2.x. 
    #include "driver/i2c_master.h"
    i2c_master_bus_handle_t i2c_bus_handle;
	i2c_master_dev_handle_t i2c_device_handle;
#endif
#endif

#ifdef CONFIG_BME280_USE_SPI
#endif

#include "bme280.h"

#include "sdkconfig.h" // generated by "make menuconfig"

#ifdef CONFIG_BME280_USE_I2C
    #define I2C_MASTER_INTERFACE_NUMBER         CONFIG_I2C_MASTER_INTERFACE_NUMBER
    #define I2C_MASTER_SCL_IO                   CONFIG_I2C_MASTER_SCL           /*!< GPIO number used for I2C master clock */
    #define I2C_MASTER_SDA_IO                   CONFIG_I2C_MASTER_SDA           /*!< GPIO number used for I2C master data  */
    #define I2C_MASTER_NUM                      CONFIG_I2C_MASTER_PORT          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
    #define I2C_MASTER_FREQ_HZ                  CONFIG_I2C_MASTER_FREQ_KHZ*1000 /*!< I2C master clock frequency */
    #define I2C_MASTER_TX_BUF_DISABLE           0                               /*!< I2C master doesn't need buffer */
    #define I2C_MASTER_RX_BUF_DISABLE           0                               /*!< I2C master doesn't need buffer */
    #define I2C_MASTER_TIMEOUT_MS               1000
    #define I2C_MASTER_ACK                      CONFIG_I2C_MASTER_ACK
    #define BME280_SENSOR_ADDR                  CONFIG_BME280_SENSOR_ADDR
#endif

#ifdef CONFIG_BME280_USE_SPI
    #include "driver/spi.h"
    #error This code has not been implemented
#endif

#ifdef CONFIG_BME280_DOUBLE_ENABLE
    #define BME280_DOUBLE_ENABLE
#else
    #undef BME280_DOUBLE_ENABLE
#endif

/*!
 *  @brief sample time [ms].
 */
static uint32_t bme280_polling_period;

/*!
 *  @brief The driver.
 */
static struct bme280_dev bme280_dev;

/*!
 *  @brief Settings for this driver.
 */
static struct bme280_settings bme280_settings;

BME280_INTF_RET_TYPE comp_bme280_init(struct bme280_dev *dev, struct bme280_settings *settings);
/*!
 *  @brief Sampled values of sensor data.
 */
static struct bme280_data bme280_data;

#ifdef CONFIG_BME280_USE_I2C
/*!
 *  @brief Function for reading the sensor's registers through I2C bus.
 *
 * @param[in] reg_addr       : Register address from which data is read.
 * @param[out] reg_data      : Pointer to data buffer where read data is stored.
 * @param[in] length         : Number of bytes of data to be read.
 * @param[in, out] intf_ptr  : Void pointer that can enable the linking of descriptors
 *                             for interface related call backs.
 *
 *  @return Status of execution
 *
 *  @retval BME280_INTF_RET_SUCCESS -> Success.
 *  @retval != BME280_INTF_RET_SUCCESS -> Failure.
 *
 */
BME280_INTF_RET_TYPE comp_bme280_i2c_read(uint8_t reg_addr, uint8_t *reg_data, size_t length, void *intf_ptr);
#endif

#ifdef CONFIG_BME280_USE_I2C
/*!
 *  @brief Function for writing the sensor's registers through I2C bus.
 *
 * @param[in] reg_addr      : Register address to which the data is written.
 * @param[in] reg_data      : Pointer to data buffer in which data to be written
 *                            is stored.
 * @param[in] length        : Number of bytes of data to be written.
 * @param[in, out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                            for interface related call backs
 *
 *  @return Status of execution
 *
 *  @retval BME280_INTF_RET_SUCCESS -> Success.
 *  @retval != BME280_INTF_RET_SUCCESS -> Failure.
 *
 */
BME280_INTF_RET_TYPE comp_bme280_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, size_t size, void *intf_ptr);
#endif

#ifdef CONFIG_BME280_USE_SPI
/*!
 *  @brief Function for reading the sensor's registers through I2C bus.
 *
 * @param[in] reg_addr       : Register address from which data is read.
 * @param[out] reg_data      : Pointer to data buffer where read data is stored.
 * @param[in] length         : Number of bytes of data to be read.
 * @param[in, out] intf_ptr  : Void pointer that can enable the linking of descriptors
 *                             for interface related call backs.
 *
 *  @return Status of execution
 *
 *  @retval BME280_INTF_RET_SUCCESS -> Success.
 *  @retval != BME280_INTF_RET_SUCCESS -> Failure.
 *
 */
BME280_INTF_RET_TYPE comp_bme280_spi_read(spi_port_t reg_addr, uint8_t *reg_data, uint32_t size, void *intf_ptr);
#endif

#ifdef CONFIG_BME280_USE_SPI
/*!
 *  @brief Function for writing the sensor's registers through I2C bus.
 *
 * @param[in] reg_addr      : Register address to which the data is written.
 * @param[in] reg_data      : Pointer to data buffer in which data to be written
 *                            is stored.
 * @param[in] length        : Number of bytes of data to be written.
 * @param[in, out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                            for interface related call backs
 *
 *  @return Status of execution
 *
 *  @retval BME280_INTF_RET_SUCCESS -> Success.
 *  @retval != BME280_INTF_RET_SUCCESS -> Failure.
 *
 */
BME280_INTF_RET_TYPE comp_bme280_spi_write(spi_port_t reg_addr, const uint8_t *reg_data, uint32_t size, void *intf_ptr);
#endif

/*!
 *  @brief This function provides the delay for required time (Microsecond) as per the input provided in some of the
 *  APIs.
 *
 *  @param[in] period_us  : The required wait time in microsecond.
 *  @param[in] intf_ptr   : Interface pointer
 *
 *  @return void.
 */
void comp_bme280_delay_us(uint32_t period_us, void *intf_ptr);

/*!
 *  @brief This function is to select the interface between SPI and I2C.
 *
 *  @param[in] dev    : Structure instance of bme280_dev
 *  @param[in] intf   : Interface selection parameter
 *                          For I2C : BME280_I2C_INTF
 *                          For SPI : BME280_SPI_INTF
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval < 0 -> Failure
 */
BME280_INTF_RET_TYPE comp_bme280_get_data(struct bme280_dev *dev, struct bme280_data *data);

#ifdef __cplusplus
}

#endif /* End of CPP guard */

#endif /* _MAIN_H */