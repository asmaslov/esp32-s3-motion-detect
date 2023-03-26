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
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"

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

#define TASK_GET_ACC_PERIOD_MS              10
#define TASK_GET_ACC_PRIORITY               (configMAX_PRIORITIES - 1)
#define TASK_SAVE_ACC_SEND_UART_PERIOD_MS   CONFIG_DATA_PERIOD_MS
#define TASK_SAVE_ACC_SEND_UART_PRIORITY    (configMAX_PRIORITIES - 1)
#define TASK_DETECT_PRIORITY                (configMAX_PRIORITIES - 2)
#define TASK_READ_UART_TIMEOUT_MS           100
#define TASK_READ_UART_PRIORITY             (configMAX_PRIORITIES - 2)
#define TASK_HEARTBEAT_PERIOD_MS            1000

#define DSP_INPUT_BLOCK_SIZE  60
#define DSP_INPUT_BLOCK_NUM  (EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE / DSP_INPUT_BLOCK_SIZE)

#define BLINK_OFF_PERIOD_MS          3000
#define BLINK_LIGHT_LOAD_PERIOD_MS   1000
#define BLINK_MEDIUM_LOAD_PERIOD_MS  500
#define BLINK_HEAVY_LOAD_PERIOD_MS   250

/* I2C */

esp_err_t i2c_master_init(void) {
    esp_err_t ret;

    const i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = CONFIG_I2C_MASTER_SDA_PIN,
        .scl_io_num = CONFIG_I2C_MASTER_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = I2C_MASTER_FREQ_HZ
        },
        .clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL
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
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_APB
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

static uint32_t led_state;

esp_err_t led_init(void) {
    esp_err_t ret;

    ret = gpio_reset_pin((gpio_num_t)CONFIG_BLINK_GPIO_PIN);
    ESP_ERROR_CHECK(ret);
    ret = gpio_set_direction((gpio_num_t)CONFIG_BLINK_GPIO_PIN, GPIO_MODE_OUTPUT);
    ESP_ERROR_CHECK(ret);
    led_state = 1;

    return ret;
}

esp_err_t led_blink(void) {
    esp_err_t ret;

    led_state = !led_state;
    ret = gpio_set_level((gpio_num_t)CONFIG_BLINK_GPIO_PIN, led_state);

    return ret;
}

/* MAIN */

static sample_t acc_x, acc_y, acc_z;
SemaphoreHandle_t mutex_acc, semaphore_save, semaphore_detect;
TickType_t blink_period[] = {BLINK_HEAVY_LOAD_PERIOD_MS, BLINK_LIGHT_LOAD_PERIOD_MS, BLINK_MEDIUM_LOAD_PERIOD_MS, BLINK_OFF_PERIOD_MS};
TickType_t current_blink_period;

void task_get_acc(void *arg) {
    uint8_t data[2];
    const char *tag = pcTaskGetName(xTaskGetCurrentTaskHandle());
    esp_log_level_set(tag, ESP_LOG_INFO);

    ESP_ERROR_CHECK(qma7981_register_read(tag, QMA7981_REG_CHIP_ID, data, 1));
    ESP_LOGI(tag, "QMA7981 ID = 0x%02X", data[0]);

    ESP_ERROR_CHECK(qma7981_register_update_byte(tag, QMA7981_REG_PM, QMA7981_REG_PM_MODE));

    while (1) {
        xSemaphoreTake(mutex_acc, portMAX_DELAY);
        qma7981_register_read(tag, QMA7981_REG_ACC_X_LSB, data, 2);
        acc_x = QMA7981_REG_ACC_VAL(data[0], data[1]);
        qma7981_register_read(tag, QMA7981_REG_ACC_Y_LSB, data, 2);
        acc_y = QMA7981_REG_ACC_VAL(data[0], data[1]);
        qma7981_register_read(tag, QMA7981_REG_ACC_Z_LSB, data, 2);
        acc_z = QMA7981_REG_ACC_VAL(data[0], data[1]);
        xSemaphoreGive(mutex_acc);
        vTaskDelay(TASK_GET_ACC_PERIOD_MS / portTICK_PERIOD_MS);
    }
}

#if defined(CONFIG_MODE_PREDICT)
float input_buf[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
size_t total_blocks, block_idx, data_idx;
#endif

void task_save_acc_send_uart(void *arg) {
    sample_t x, y, z;
    char message[CONFIG_UART_TX_BUFFER_SIZE];
    const char *tag = pcTaskGetName(xTaskGetCurrentTaskHandle());
    esp_log_level_set(tag, ESP_LOG_INFO);

#if defined(CONFIG_MODE_PREDICT)
    ESP_LOGI(tag, "Block update interval %d ms", TASK_GET_ACC_PERIOD_MS * DSP_INPUT_BLOCK_SIZE);
    xSemaphoreGive(semaphore_save);
#endif
    while (1) {
        xSemaphoreTake(mutex_acc, portMAX_DELAY);
        x = acc_x;
        y = acc_y;
        z = acc_z;
        xSemaphoreGive(mutex_acc);
        // String format for edge-impulse-data-forwarder
        sprintf(message, "%d\t%d\t%d\r\n", x, y, z);
#if defined(CONFIG_MODE_PREDICT)
        if (xSemaphoreTake(semaphore_save, 0) == pdTRUE) {
            input_buf[block_idx * DSP_INPUT_BLOCK_SIZE + data_idx + 0] = x;
            input_buf[block_idx * DSP_INPUT_BLOCK_SIZE + data_idx + 1] = y;
            input_buf[block_idx * DSP_INPUT_BLOCK_SIZE + data_idx + 2] = z;
            data_idx += EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME;
            if (DSP_INPUT_BLOCK_SIZE == data_idx) {
                data_idx = 0;
                block_idx++;
                if (DSP_INPUT_BLOCK_NUM == block_idx) {
                    block_idx = 0;
                }
                if (total_blocks < DSP_INPUT_BLOCK_NUM) {
                    total_blocks++;
                    xSemaphoreGive(semaphore_save);
                } else {
                    xSemaphoreGive(semaphore_detect);
                }
            } else {
                xSemaphoreGive(semaphore_save);
            }
        }
#elif defined(CONFIG_MODE_COLLECT)
        printf(message);
#endif
        uart_write(tag, message, strlen(message));
        vTaskDelay(TASK_SAVE_ACC_SEND_UART_PERIOD_MS / portTICK_PERIOD_MS);
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

#if defined(CONFIG_MODE_PREDICT)
static int get_signal_data(size_t offset, size_t length, float *out_ptr) {
    // Get maximum length of data till the end of the buffer
    size_t max_cont_len = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - (block_idx * DSP_INPUT_BLOCK_SIZE) - offset;
    size_t cont_len = (length < max_cont_len) ? length : max_cont_len;
    size_t i = 0;
    // Get all data of length size till the end of the buffer 
    while (i < cont_len) {
        out_ptr[i] = input_buf[block_idx * DSP_INPUT_BLOCK_SIZE + offset + i];
        i++;
    }
    size_t j = 0;
    // Get the rest of the data from the beggining of the buffer if there is some
    while (i < length) {
        out_ptr[i] = input_buf[j];
        i++;
        j++;
    }
    return EIDSP_OK;
}

void task_detect(void *arg)
{
    const char *tag = pcTaskGetName(xTaskGetCurrentTaskHandle());
    esp_log_level_set(tag, ESP_LOG_INFO);

    signal_t signal;
    ei_impulse_result_t result;
    EI_IMPULSE_ERROR ret;

    // Assign callback function to fill buffer used for preprocessing/inference
    signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
    signal.get_data = &get_signal_data;

    while (1) {
        xSemaphoreTake(semaphore_detect, portMAX_DELAY);

        // Perform DSP pre-processing and inference
        ret = run_classifier(&signal, &result, false);

        // Print return code and how long it took to perform inference
        if (ret != EI_IMPULSE_OK) {
            ESP_LOGE(tag,  "Classifier returned %d", ret);
        }
        ESP_LOGD(tag, "Timing: DSP %d ms, inference %d ms, anomaly %d ms\r\n", 
                 result.timing.dsp, result.timing.classification, result.timing.anomaly);

        size_t max_idx;
        float max = 0;
        printf("Predictions:\r\n");
        for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
            printf("  %s: ", ei_classifier_inferencing_categories[i]);
            printf("%.5f\r\n", result.classification[i].value);
            if (result.classification[i].value > max) {
                max = result.classification[i].value;
                max_idx = i;
            }
        }
        printf("Anomaly prediction: %.3f\r\n", result.anomaly);

        current_blink_period = blink_period[max_idx];

        xSemaphoreGive(semaphore_save);
    }
}
#endif

extern "C" void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_ERROR_CHECK(uart_init());
    ESP_ERROR_CHECK(led_init());

#if defined(CONFIG_MODE_COLLECT)
    current_blink_period = TASK_HEARTBEAT_PERIOD_MS;
#elif defined(CONFIG_MODE_PREDICT)
    current_blink_period = BLINK_OFF_PERIOD_MS;
#endif
    mutex_acc = xSemaphoreCreateMutex();
    semaphore_save = xSemaphoreCreateBinary();
    semaphore_detect = xSemaphoreCreateBinary();
    xTaskCreate(task_get_acc, "task_get_acc", 4096, NULL, TASK_GET_ACC_PRIORITY, NULL);
    xTaskCreate(task_save_acc_send_uart, "task_save_acc_send_uart", 4096, NULL, TASK_SAVE_ACC_SEND_UART_PRIORITY, NULL);
#if defined(CONFIG_MODE_PREDICT)
    xTaskCreate(task_detect, "task_detect", 4096, NULL, TASK_DETECT_PRIORITY, NULL);
#endif
    xTaskCreate(task_read_uart, "task_read_uart", 4096, NULL, TASK_READ_UART_PRIORITY, NULL);

    while (1) {
        led_blink();
        vTaskDelay(current_blink_period / 2 / portTICK_PERIOD_MS);
    }
}
