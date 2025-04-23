#include "classBmi160.h"

#include <esp_log.h>

static char *TAG = "BMI160";

constexpr float sensorTimeScale = 0.039f;

/**** PRIVATE FUNCTIONS ****/
int8_t bmi160_read_spi(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    spi_transaction_t spiTrans = {0};

    spiTrans.length = 8 * len;
    spiTrans.addr = reg_addr ;
    spiTrans.rxlength = 8 * len;
    spiTrans.rx_buffer = data;

    spi_device_acquire_bus(Bmi160::spiHandle, portMAX_DELAY);
    spi_device_polling_transmit(Bmi160::spiHandle, &spiTrans);
    spi_device_release_bus(Bmi160::spiHandle);
 
    return 0;
}

int8_t bmi160_write_spi(uint8_t dev_addr, uint8_t reg_addr, uint8_t *read_data, uint16_t len)
{
    spi_transaction_t spiTrans = {0};

    spiTrans.length = 8 * len;
    spiTrans.addr = reg_addr ;
    spiTrans.tx_buffer = read_data ;
 
    spi_device_acquire_bus(Bmi160::spiHandle, portMAX_DELAY);
    spi_device_polling_transmit(Bmi160::spiHandle, &spiTrans);
    spi_device_release_bus(Bmi160:: spiHandle);

    return 0;
}

void bmi_delay(uint32_t period)
{
    if (period < 10)
    {
        period = 10;
    }
    
    vTaskDelay(pdMS_TO_TICKS(period)); 

} 

/**** PUBLIC FUNCTIONS ****/
spi_device_handle_t Bmi160::spiHandle = {0};
 
Bmi160::Bmi160():
    dev({0}), spiBus({0}), spiIf({0})
{

}

uint8_t Bmi160::init(Bmi160SpiConfig spiConfig)
{
    spiBus.mosi_io_num = spiConfig.mosi;
    spiBus.miso_io_num = spiConfig.miso;
    spiBus.sclk_io_num = spiConfig.sclk;
    spiBus.quadhd_io_num = -1;
    spiBus.quadwp_io_num = -1;
    spiBus.max_transfer_sz = 4092;

    //ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &spiBus, SPI_DMA_DISABLED));
    if ( spi_bus_initialize(spiConfig.spidev, &spiBus, SPI_DMA_DISABLED) != ESP_OK )
    {
         ESP_LOGE(TAG, "SPI Bus cannot be initialized");

         return 1;
    }

    ESP_LOGE(TAG, "SPI Bus initialized correctly");

    spiIf.spics_io_num = spiConfig.cs;
    spiIf.clock_speed_hz = spiConfig.speed;
    spiIf.mode = 0;
    spiIf.queue_size = 10;
    spiIf.address_bits = 8;

    //ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &spiIf, &spiHandle));
    if ( spi_bus_add_device(spiConfig.spidev, &spiIf, &spiHandle)  != ESP_OK )
    {
         ESP_LOGE(TAG, "SPI Device cannot be added");

         return 2;
    }

    ESP_LOGE(TAG, "SPI Device added correctly");

    dev.intf = BMI160_SPI_INTF;
    dev.read = bmi160_read_spi;
    dev.write = bmi160_write_spi;
    dev.delay_ms = bmi_delay;

    return configure();
}

uint8_t Bmi160::init(Bmi160I2cConfig i2cConfig)
{
    return configure();
}

uint8_t Bmi160::configure()
{
    int8_t rslt = bmi160_init(&dev);

    if (rslt == BMI160_OK)
    {
        ESP_LOGE(TAG, "BMI160 initialization success !\n");
        ESP_LOGE(TAG, "Chip ID 0x%X\n", dev.chip_id);
    }
    else
    {
        ESP_LOGE(TAG, "BMI160 initialization failure !\n");
    }

    /* Select the Output data rate, range of accelerometer sensor */
    dev.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
    dev.accel_cfg.range = BMI160_ACCEL_RANGE_16G;
    dev.accel_cfg.bw = BMI160_ACCEL_BW_OSR4_AVG1;

    /* Select the power mode of accelerometer sensor */
    dev.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    /* Select the Output data rate, range of Gyroscope sensor */
    dev.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
    dev.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    dev.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

    accScale = 16.0f / float((1 << 15) - 1);
    gyrScale =  2000.0f / float((1 << 15) - 1);

    /* Select the power mode of Gyroscope sensor */
    dev.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

    /* Set the sensor configuration */
    rslt = bmi160_set_sens_conf(&dev);

    if (rslt != BMI160_OK)
    {
        ESP_LOGE(TAG, "Sensor configuration failed");
    }

    return rslt; 
}

uint8_t Bmi160::getRawData(bmi160_sensor_data &acc, bmi160_sensor_data &gyr)
{
    uint8_t ret = bmi160_get_sensor_data(BMI160_ACCEL_SEL | BMI160_GYRO_SEL | BMI160_TIME_SEL, &acc, &gyr, &dev);
    ESP_LOGE(TAG, "Acc: %6d, %6d, %6d %6lu\n", acc.x, acc.y, acc.z, acc.sensortime);

    return ret; 
}

 uint8_t Bmi160::getData(Data &acc, Data &gyr)
{
    bmi160_sensor_data rawAcc, rawGyr;
    uint8_t ret = getRawData(rawAcc, rawGyr);

    acc.x = rawAcc.x * accScale;
    acc.y = rawAcc.y * accScale;
    acc.z = rawAcc.z * accScale;
    acc.time = rawAcc.sensortime * sensorTimeScale;

    gyr.x = rawGyr.x * gyrScale; 
    gyr.y = rawGyr.y * gyrScale;  
    gyr.z = rawGyr.z * gyrScale; 
    gyr.time = rawGyr .sensortime * sensorTimeScale ;

    return ret;
}
