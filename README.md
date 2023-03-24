# ESP32-S3 Motion Detect

Based on ESP32-S3 development board with QMA7981.

## Overview

Accelerometer data is collected from QMA7981 and transmitted to UART.

## Pin Assignment:

**Note:** The following pin assignments are used by default, you can change these in the `menuconfig` .

| I2C              | SDA                | SCL                |
| ---------------- | ------------------ | ------------------ |
| ESP I2C Master   | I2C_MASTER_SDA_PIN | I2C_MASTER_SCL_PIN |
| QMA7981 Sensor   | SDA_PIN            | SCL_PIN            |

| UART             | RX          | TX          |
| ---------------- | ----------- | ----------- |
| UART_NUM_0       | UART_RX_PIN | UART_TX_PIN |

For the actual default value of `I2C_MASTER_SDA` and `I2C_MASTER_SCL` see `Project Configuration` in `menuconfig`.

**Note: ** There’s no need to add an external pull-up resistors for SDA/SCL pin, because the driver will enable the internal pull-up resistors.

## Build and Flash

Enter `idf.py -p PORT flash monitor` to build, flash and monitor the project.

(To exit the serial monitor, type ``Ctrl-]``.)

See the [Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for full steps to configure and use ESP-IDF to build projects.

## Example Output

```
I (214) task_get_acc: QMA7981 ID = 0xE7
I (214) motion-detect: Hearbeat 0
I (1214) motion-detect: Hearbeat 1
I (2214) motion-detect: Hearbeat 2
```
