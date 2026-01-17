#pragma once
#ifndef ESP_HAL_H
#define ESP_HAL_H

#include <RadioLib.h>

#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_attr.h"

// ROM delay
#if CONFIG_IDF_TARGET_ESP32S3
  #include "esp32s3/rom/ets_sys.h"
#else
  #include "rom/ets_sys.h"
#endif

// Arduino-ish macros, but mapped to ESP-IDF enums where it matters
#define LOW     (0)
#define HIGH    (1)
#define INPUT   GPIO_MODE_INPUT
#define OUTPUT  GPIO_MODE_OUTPUT
#define RISING  GPIO_INTR_POSEDGE
#define FALLING GPIO_INTR_NEGEDGE

class EspHal : public RadioLibHal {
public:
  EspHal(int8_t sck, int8_t miso, int8_t mosi)
  : RadioLibHal(INPUT, OUTPUT, LOW, HIGH, RISING, FALLING),
    spiSCK(sck), spiMISO(miso), spiMOSI(mosi) {}

  void init() override {
    installIsrServiceOnce();
    spiBegin();
  }

  void term() override {
    spiEnd();
  }

  void pinMode(uint32_t pin, uint32_t mode) override {
    if(pin == RADIOLIB_NC) return;

    gpio_config_t conf = {};
    conf.pin_bit_mask = (1ULL << pin);
    conf.mode = (gpio_mode_t)mode;
    conf.pull_up_en = GPIO_PULLUP_DISABLE;
    conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&conf);
  }

  void digitalWrite(uint32_t pin, uint32_t value) override {
    if(pin == RADIOLIB_NC) return;
    gpio_set_level((gpio_num_t)pin, (int)value);
  }

  uint32_t digitalRead(uint32_t pin) override {
    if(pin == RADIOLIB_NC) return 0;
    return (uint32_t)gpio_get_level((gpio_num_t)pin);
  }

  void attachInterrupt(uint32_t interruptNum, void (*interruptCb)(void), uint32_t mode) override {
    if(interruptNum == RADIOLIB_NC) return;
    installIsrServiceOnce();

    gpio_set_intr_type((gpio_num_t)interruptNum, (gpio_int_type_t)(mode & 0x7));
    gpio_isr_handler_add((gpio_num_t)interruptNum, &isrTrampoline, (void*)interruptCb);
  }

  void detachInterrupt(uint32_t interruptNum) override {
    if(interruptNum == RADIOLIB_NC) return;
    gpio_isr_handler_remove((gpio_num_t)interruptNum);
    gpio_wakeup_disable((gpio_num_t)interruptNum);
    gpio_set_intr_type((gpio_num_t)interruptNum, GPIO_INTR_DISABLE);
  }

  void delay(unsigned long ms) override {
    vTaskDelay(ms / portTICK_PERIOD_MS);
  }

  void delayMicroseconds(unsigned long us) override {
    ets_delay_us(us);
  }

  unsigned long millis() override {
    return (unsigned long)(esp_timer_get_time() / 1000ULL);
  }

  unsigned long micros() override {
    return (unsigned long)(esp_timer_get_time());
  }

  long pulseIn(uint32_t pin, uint32_t state, unsigned long timeout) override {
    if(pin == RADIOLIB_NC) return 0;

    pinMode(pin, INPUT);
    uint32_t start = micros();
    uint32_t last  = micros();

    while(digitalRead(pin) == state) {
      if((micros() - last) > timeout) return 0;
    }
    return (long)(micros() - start);
  }

  // ---- SPI (ESP-IDF spi_master) ----
  void spiBegin() {
    if(spiDev) return;

    spi_bus_config_t buscfg = {};
    buscfg.miso_io_num = spiMISO;
    buscfg.mosi_io_num = spiMOSI;
    buscfg.sclk_io_num = spiSCK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.data4_io_num = -1;
    buscfg.data5_io_num = -1;
    buscfg.data6_io_num = -1;
    buscfg.data7_io_num = -1;

    // ESP32-S3: use SPI3_HOST by default (GP-SPI). ESP32 classic can use VSPI/HSPI.
    const spi_host_device_t host =
#if CONFIG_IDF_TARGET_ESP32
      VSPI_HOST;
#else
      SPI3_HOST;
#endif
    spiHost = host;

    // If you run into DMA-capable buffer issues, change SPI_DMA_CH_AUTO -> SPI_DMA_DISABLED.
    esp_err_t ret = spi_bus_initialize(spiHost, &buscfg, SPI_DMA_CH_AUTO);
    if(ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
      ESP_LOGE("EspHal", "spi_bus_initialize failed: %s", esp_err_to_name(ret));
      return;
    }

    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = 2 * 1000 * 1000;   // 2 MHz like your original code
    devcfg.mode = 0;
    devcfg.spics_io_num = -1;                  // CS handled by RadioLib via GPIO
    devcfg.queue_size = 1;

    ret = spi_bus_add_device(spiHost, &devcfg, &spiDev);
    if(ret != ESP_OK) {
      ESP_LOGE("EspHal", "spi_bus_add_device failed: %s", esp_err_to_name(ret));
      spiDev = nullptr;
    }
  }

  void spiBeginTransaction() {}
  void spiEndTransaction() {}

  uint8_t spiTransferByte(uint8_t b) {
    if(!spiDev) return 0;

    uint8_t rx = 0;
    spi_transaction_t t = {};
    t.length = 8;
    t.tx_buffer = &b;
    t.rx_buffer = &rx;

    esp_err_t ret = spi_device_polling_transmit(spiDev, &t);
    if(ret != ESP_OK) return 0;
    return rx;
  }

  void spiTransfer(uint8_t* out, size_t len, uint8_t* in) {
    if(!spiDev || !out || !in || len == 0) return;

    spi_transaction_t t = {};
    t.length = len * 8;
    t.tx_buffer = out;
    t.rx_buffer = in;

    spi_device_polling_transmit(spiDev, &t);
  }

  void spiEnd() {
    if(spiDev) {
      spi_bus_remove_device(spiDev);
      spiDev = nullptr;
    }
    // Only free the bus if you know nothing else uses it:
    // spi_bus_free(spiHost);
  }

private:
  int8_t spiSCK;
  int8_t spiMISO;
  int8_t spiMOSI;

  spi_host_device_t spiHost = (spi_host_device_t)0;
  spi_device_handle_t spiDev = nullptr;

  static void IRAM_ATTR isrTrampoline(void* arg) {
    auto cb = reinterpret_cast<void(*)(void)>(arg);
    if(cb) cb();
  }

  static void installIsrServiceOnce() {
    static bool installed = false;
    if(installed) return;
    esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if(err == ESP_OK || err == ESP_ERR_INVALID_STATE) installed = true;
  }
};

#endif
