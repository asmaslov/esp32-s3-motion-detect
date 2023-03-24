#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

const char *tag = "motion-detect";

#if defined(CONFIG_UART_NUM_0)
#define UART_NUM  UART_NUM_0
#elif defined(CONFIG_UART_NUM_1)
#define UART_NUM  UART_NUM_1
#endif

#define I2C_MASTER_NUM             I2C_NUM_0
#define I2C_MASTER_FREQ_HZ         400000
#define I2C_MASTER_TX_BUF_DISABLE  0
#define I2C_MASTER_RX_BUF_DISABLE  0
#define I2C_MASTER_TIMEOUT_MS      1000

typedef int16_t sample_t;

#define QMA7981_ADDR                   0x12
#define QMA7981_REG_CHIP_ID            0x00
#define QMA7981_REG_ACC_X_LSB          0x01
#define QMA7981_REG_ACC_X_LSB_NEWDATA  (1 << 0)
#define QMA7981_REG_ACC_X_MSB          0x02
#define QMA7981_REG_ACC_Y_LSB          0x03
#define QMA7981_REG_ACC_Y_LSB_NEWDATA  (1 << 0)
#define QMA7981_REG_ACC_Y_MSB          0x04
#define QMA7981_REG_ACC_Z_LSB          0x05
#define QMA7981_REG_ACC_Z_LSB_NEWDATA  (1 << 0)
#define QMA7981_REG_ACC_Z_MSB          0x06
#define QMA7981_REG_ACC_VAL(lsb, msb)  ((sample_t)(((uint16_t)msb << 8) | ((uint16_t)lsb & 0xFC)) >> 2)
#define QMA7981_REG_PM                 0x11
#define QMA7981_REG_PM_MODE            (1 << 7)
#define QMA7981_REG_S_RESET            0x36
#define QMA7981_REG_S_RESET_SOFTRESET  0xB6

#define TASK_GET_ACC_PERIOD_MS         10
#define TASK_GET_ACC_PRIORITY          (configMAX_PRIORITIES - 1)
#define TASK_SEND_ACC_UART_PERIOD_MS   CONFIG_DATA_PERIOD_MS
#define TASK_SEND_ACC_UART_PRIORITY    (configMAX_PRIORITIES - 1)
#define TASK_READ_UART_TIMEOUT_MS      100
#define TASK_READ_UART_PRIORITY        (configMAX_PRIORITIES - 2)
#define TASK_HEARTBEAT_PERIOD_MS       1000

/* I2C */

esp_err_t i2c_master_init(void) {
    esp_err_t ret;

    const i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = CONFIG_I2C_MASTER_SDA_PIN,
        .scl_io_num = CONFIG_I2C_MASTER_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    ESP_ERROR_CHECK(ret);

    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    ESP_ERROR_CHECK(ret);

    return ret;
}

esp_err_t qma7981_register_read(const char* tag, const uint8_t reg_addr, uint8_t *data, const size_t len) {
    esp_err_t ret;

    ret = i2c_master_write_read_device(I2C_MASTER_NUM, QMA7981_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
    ESP_ERROR_CHECK(ret);
    ESP_LOG_BUFFER_HEXDUMP(tag, data, len, ESP_LOG_DEBUG);

    return ret;
}

esp_err_t qma7981_register_write_byte(const char* tag, const uint8_t reg_addr, const uint8_t data) {
    esp_err_t ret;
    const uint8_t write_buf[2] = {reg_addr, data};

    ESP_LOG_BUFFER_HEXDUMP(tag, write_buf, 2, ESP_LOG_DEBUG);
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, QMA7981_ADDR, write_buf, 2, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
    ESP_ERROR_CHECK(ret);

    return ret;
}

esp_err_t qma7981_register_update_byte(const char* tag, const uint8_t reg_addr, const uint8_t data) {
    esp_err_t ret;
    uint8_t buf[2] = {reg_addr, 0};

    ret = i2c_master_write_read_device(I2C_MASTER_NUM, QMA7981_ADDR, &reg_addr, 1, &buf[1], 1, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
    ESP_ERROR_CHECK(ret);
    buf[1] &= ~data;
    buf[1] |= data;
    ESP_LOG_BUFFER_HEXDUMP(tag, buf, 2, ESP_LOG_DEBUG);
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, QMA7981_ADDR, buf, 2, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
    ESP_ERROR_CHECK(ret);

    return ret;
}

/* UART */

esp_err_t uart_init(void) {
    esp_err_t ret;
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ret = uart_driver_install(UART_NUM, CONFIG_UART_RX_BUFFER_SIZE, CONFIG_UART_TX_BUFFER_SIZE, 0, NULL, 0);
    ESP_ERROR_CHECK(ret);
    ret = uart_param_config(UART_NUM, &uart_config);
    ESP_ERROR_CHECK(ret);
    ret = uart_set_pin(UART_NUM, CONFIG_UART_TX_PIN, CONFIG_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    ESP_ERROR_CHECK(ret);

    return ret;
}

int uart_write(const char* tag, const char* data, size_t len) {
    const int txBytes = uart_write_bytes(UART_NUM, data, strlen(data));
    ESP_LOG_BUFFER_HEXDUMP(tag, data, txBytes, ESP_LOG_DEBUG);
    return txBytes;
}

int uart_read(const char* tag, char* data, size_t len, TickType_t timeout) {
    const int rxBytes = uart_read_bytes(UART_NUM, data, len, timeout);
    ESP_LOG_BUFFER_HEXDUMP(tag, data, rxBytes, ESP_LOG_DEBUG);
    return rxBytes;
}

/* LED */

uint32_t led_state;

esp_err_t led_init(void) {
    esp_err_t ret;

    ret = gpio_reset_pin(CONFIG_BLINK_GPIO_PIN);
    ESP_ERROR_CHECK(ret);
    ret = gpio_set_direction(CONFIG_BLINK_GPIO_PIN, GPIO_MODE_OUTPUT);
    ESP_ERROR_CHECK(ret);
    led_state = 1;

    return ret;
}

esp_err_t led_blink(void) {
    esp_err_t ret;

    led_state = !led_state;
    ret = gpio_set_level(CONFIG_BLINK_GPIO_PIN, led_state);

    return ret;
}

/* MAIN */

sample_t x, y, z;
SemaphoreHandle_t mutex;

void task_get_acc(void *arg) {
    uint8_t data[2];
    const char *tag = pcTaskGetName(xTaskGetCurrentTaskHandle());
    esp_log_level_set(tag, ESP_LOG_INFO);

    ESP_ERROR_CHECK(qma7981_register_read(tag, QMA7981_REG_CHIP_ID, data, 1));
    ESP_LOGI(tag, "QMA7981 ID = 0x%02X", data[0]);

    ESP_ERROR_CHECK(qma7981_register_update_byte(tag, QMA7981_REG_PM, QMA7981_REG_PM_MODE));

    while (1) {
        xSemaphoreTake(mutex, portMAX_DELAY);
        qma7981_register_read(tag, QMA7981_REG_ACC_X_LSB, data, 2);
        x = QMA7981_REG_ACC_VAL(data[0], data[1]);
        qma7981_register_read(tag, QMA7981_REG_ACC_Y_LSB, data, 2);
        y = QMA7981_REG_ACC_VAL(data[0], data[1]);
        qma7981_register_read(tag, QMA7981_REG_ACC_Z_LSB, data, 2);
        z = QMA7981_REG_ACC_VAL(data[0], data[1]);
        xSemaphoreGive(mutex);
        vTaskDelay(TASK_GET_ACC_PERIOD_MS / portTICK_PERIOD_MS);
    }
}

void task_send_acc_uart(void *arg) {
    char message[CONFIG_UART_TX_BUFFER_SIZE];
    const char *tag = pcTaskGetName(xTaskGetCurrentTaskHandle());
    esp_log_level_set(tag, ESP_LOG_INFO);

    while (1) {
        xSemaphoreTake(mutex, portMAX_DELAY);
        sprintf(message, "%d\t%d\t%d\n", x, y, z);
        xSemaphoreGive(mutex);
        uart_write(tag, message, strlen(message));
        vTaskDelay(TASK_SEND_ACC_UART_PERIOD_MS / portTICK_PERIOD_MS);
    }
}

void task_read_uart(void *arg)
{
    char* data = (char*)malloc(CONFIG_UART_RX_BUFFER_SIZE + 1);
    const char *tag = pcTaskGetName(xTaskGetCurrentTaskHandle());
    esp_log_level_set(tag, ESP_LOG_INFO);

    while (1) {
        const int rxBytes = uart_read(tag, data, CONFIG_UART_RX_BUFFER_SIZE, TASK_READ_UART_TIMEOUT_MS / portTICK_RATE_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGI(tag, "Read %d bytes: '%s'", rxBytes, data);
        }
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_ERROR_CHECK(uart_init());
    ESP_ERROR_CHECK(led_init());

    mutex = xSemaphoreCreateMutex();
    xTaskCreate(task_get_acc, "task_get_acc", 4096, NULL, TASK_GET_ACC_PRIORITY, NULL);
    xTaskCreate(task_send_acc_uart, "task_send_acc_uart", 4096, NULL, TASK_SEND_ACC_UART_PRIORITY, NULL);
    xTaskCreate(task_read_uart, "task_read_uart", 4096, NULL, TASK_READ_UART_PRIORITY, NULL);

    while (1) {
        led_blink();
        ESP_LOGI(tag, "Heartbeat");
        vTaskDelay(TASK_HEARTBEAT_PERIOD_MS / portTICK_PERIOD_MS);
    }
}
