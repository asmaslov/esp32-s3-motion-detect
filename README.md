# ESP32-S3 Motion Detect

Based on ESP32-S3 development board with QMA7981.

## Overview

Used [ESP32-S3-EYE v2.2](https://github.com/espressif/esp-who/blob/master/docs/en/get-started/ESP32-S3-EYE_Getting_Started_Guide.md) development kit without LCD module.

Used Edge Impulse model [industrial-motion-classifier](https://studio.edgeimpulse.com/public/201134/latest) trained to detect by vibration several washing mashine states: off, light-load, medium-load and high-load.

Accelerometer data is collected with QMA7981 and transmitted to UART.

Firmware has 2 operation modes. Mode can be selected in `Project Configuration` in `menuconfig`

### Data collection
Values are directly printed to the debug console for further transmitting to Edge Impulse platform.
Launch `edge-impulse-data-forwarder` for data collection.

#### Example output
```
I (240) task_get_acc: QMA7981 ID = 0xE7
10      -48     -4876
10      -48     -4876
-15     -59     -4834
```

### Prediction
Values are collected in circle buffer and prediction result are printed in the debug console.

#### Example output
```
I (241) task_save_acc_s: Block update interval 600 ms
I (241) task_get_acc: QMA7981 ID = 0xE7
Predictions:
  heavy-load: 0.00005
  light-load: 0.00120
  medium-load: 0.00277
  off: 0.99598
Anomaly prediction: -0.487
```

## Pin Assignment:

**Note:** The following pin assignments are used by default, you can change these in the `menuconfig` .

| I2C              | SDA                | SCL                |
| ---------------- | ------------------ | ------------------ |
| ESP I2C Master   | I2C_MASTER_SDA_PIN | I2C_MASTER_SCL_PIN |
| QMA7981 Sensor   | SDA_PIN            | SCL_PIN            |

| UART             | RX          | TX          |
| ---------------- | ----------- | ----------- |
| UART_NUM_0       | UART_RX_PIN | UART_TX_PIN |

For the actual default value of `I2C_MASTER_SDA`, `I2C_MASTER_SCL`, `UART_RX_PIN` and `UART_TX_PIN` see `Project Configuration` in `menuconfig`.

**Note:** There’s no need to add an external pull-up resistors for SDA/SCL pin, because the driver will enable the internal pull-up resistors.

## Build and Flash

Enter `idf.py -p PORT flash monitor` to build, flash and monitor the project.

(To exit the serial monitor, type ``Ctrl-]``.)

See the [Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for full steps to configure and use ESP-IDF to build projects.
