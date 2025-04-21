#include "bmi160.h"

#include <esp_log.h>

#include <driver/gpio.h>
#include <driver/spi_master.h>

const char TAG[] = "BMI_DRIVER";

void bmi_initSpi() 
{
    spi_bus_config_t spiBus = {0};
    spi_device_handle_t spiHandle = 0;
    spi_device_interface_config_t spiIf = {0};

    spiBus.mosi_io_num = GPIO_NUM_23;
    spiBus.miso_io_num = GPIO_NUM_19;
    spiBus.sclk_io_num = GPIO_NUM_18;

    //ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &spiBus, SPI_DMA_DISABLED));
    if ( spi_bus_initialize(SPI3_HOST, &spiBus, SPI_DMA_DISABLED) != ESP_OK )
    {
         ESP_LOGE(TAG, "SPI Bus cannot be initialized");

         return;
    }

    ESP_LOGE(TAG, "SPI Bus initialized correctly");

    spiIf.spics_io_num = GPIO_NUM_21;
    spiIf.clock_speed_hz = 10 * 1000 * 1000;
    spiIf.mode = 0;
    spiIf.queue_size = 10;
    spiIf.address_bits = 8;

    //ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &spiIf, &spiHandle));
    if ( spi_bus_add_device(SPI3_HOST, &spiIf, &spiHandle)  != ESP_OK )
    {
         ESP_LOGE(TAG, "SPI Device cannot be added");

         return;
    }

    ESP_LOGE(TAG, "SPI Device added correctly");

    uint8_t spiReadAddr = 0x7F;
    spi_transaction_t spiTrans = {0};

    uint8_t data;
    spiTrans.length = 8;
    spiTrans.addr = (0x7F | 0x80);
    spiTrans.rxlength = 8;
    spiTrans.rx_buffer = &data;
    //spiTrans.flags = SPI_TRANS_USE_RXDATA;

    spi_device_acquire_bus(spiHandle, portMAX_DELAY);
    if ( spi_device_polling_transmit(spiHandle, &spiTrans)  != ESP_OK )
    {
         ESP_LOGE(TAG, "SPI Transaction ERROR");

         return;
    }

    ESP_LOGE(TAG, "SPI Transaction done");

    spi_device_release_bus(spiHandle);

    ESP_LOGE(TAG, "Received value: %d", data);

    spi_device_acquire_bus(spiHandle, portMAX_DELAY);
    spiTrans.addr = (0x0 | 0x80);
    if ( spi_device_polling_transmit(spiHandle, &spiTrans)  != ESP_OK )
    {
         ESP_LOGE(TAG, "SPI Transaction ERROR");

         return;
    }

    ESP_LOGE(TAG, "SPI Transaction done");

    spi_device_release_bus(spiHandle);

    ESP_LOGE(TAG, "Received value: %x", data);

}

uint8_t bmi_getId()
{
    return 0;
}
