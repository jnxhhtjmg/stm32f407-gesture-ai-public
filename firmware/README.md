# Firmware

STM32F407VET6 firmware for the 6-class gesture demo.  FreeRTOS + LiteRT Micro
(formerly TFLite Micro), Keil MDK project.

> The trained model is **not included**.  `Core/Src/g_gesture_model_data.c` is a
> 16-byte placeholder and `gesture_preprocess.c` contains placeholder
> normalization/quantization values.  The project builds and boots, but logs
> `gesture inference init failed` until you train a model and update both files
> (see *Updating the model*).

## Hardware

| Function | Pin | Configuration |
| --- | --- | --- |
| MCU | STM32F407VET6 | 168 MHz, 512 KB Flash, 128 KB SRAM |
| IMU | MPU6050 | I2C2: PB10/SCL, PB11/SDA, 0x68, ±2 g / ±250 °/s |
| Console | USART1 | PA9/TX, PA10/RX, 115200 8N1 |
| OLED | SSD1306 | I2C1: PB8/SCL, PB9/SDA, 0x3C (optional) |
| Button | KEY0 | PE4, active low |
| Debug | SWD | PA13/SWDIO, PA14/SWCLK |

MPU6050 and OLED use 3.3 V.  Do not apply 5 V to the I2C pins.

## Build

Open `MDK-ARM/f407_gesture_ai.uvprojx` in Keil MDK (ARMCLANG V6.7) and build
the `f407_gesture_ai` target.  Command line:

```powershell
UV4.exe -b MDK-ARM\f407_gesture_ai.uvprojx -j0 -o build.log
```

Reference build with a 20 KB INT8 model: Code ~158 KB + RO ~32 KB, ZI ~48 KB
(RW 24 B).  The 64 KB CCM is not used.  With the placeholder model the sizes
are slightly smaller.

USART1 TX uses **DMA2 Stream 7 Channel 4** and a 4 KB ring buffer; USART1 RX
uses a byte interrupt with the streaming CRC16 parser.  `APP_UART_TX_DMA 1`
selects the DMA path; the polling path remains as a fallback.

## Flash

With OpenOCD and an ST-Link:

```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "program MDK-ARM/f407_gesture_ai/f407_gesture_ai.hex verify reset exit"
```

Keil's Flash Download also works when the ST-Link is connected to SWDIO/SWCLK/
GND/3V3.  If OpenOCD reports `init mode failed`, check the SWD wiring and common
ground before changing the firmware.

## Runtime behaviour

1. Boot: clock check, MPU6050 WHO_AM_I, gyro bias calibration, accelerometer
   baseline, inference init, optional OLED init.
2. Sampling: TIM6 interrupt at 100 Hz wakes `defaultTask`; the I2C read runs in
   task context.
3. Event: motion is detected when `|gyro| > 3 °/s` or the acceleration vector
   deviates more than 0.03 g from the boot baseline for 5 consecutive samples.
4. The next 200 samples (2 s) are buffered, then decimated 2:1 to 100 samples
   at 50 Hz and classified **once**.
5. The result is latched until the sensor is quiet for 300 ms (`|gyro| < 2 °/s`
   and acceleration deviation < 0.03 g); then `gesture,neutral,100` is emitted.
6. The return phase never triggers a second classification, so a left-and-back
   gesture cannot flip to `move_right` halfway through.

The model is trained by `../training/` on your own captures.  The
`APP_LIFT_AZ_RULE` / `APP_TILT_GZ_RULE` physics fallbacks remain in the source
but are compiled out (`..._RULE 0`); enable them only if your data needs a
domain-specific fallback.

The event thresholds and buffer size are in `Core/Inc/app_config.h`:

```c
#define APP_SEQUENCE_RAW_SAMPLES 200U
#define APP_SEQUENCE_DOWNSAMPLE  2U
#define APP_ONSET_GYRO_DPS       3
#define APP_ONSET_ACCEL_G        0.03f
#define APP_RELEASE_GYRO_DPS     2
#define APP_RELEASE_ACCEL_G      0.03f
```

## UART output

```text
f407_gesture_ai v0.3.0 started
clock,hse,168000000
MPU6050 WHO_AM_I=0x68
gyro calibration: keep sensor still
gyro_bias,<x>,<y>,<z>
gesture inference ready
oled ready
sampling,100
imu,<ax>,<ay>,<az>,<gx>,<gy>,<gz>   # every 100 ms while idle
infer_us,<us>                       # ~110 ms on the F407
gesture,<label>,<confidence>        # one line per gesture
gesture,neutral,100                 # release
capture,start / capture,stop        # button or protocol
timestamp,ax,ay,az,gx,gy,gz          # 100 Hz CSV during capture
```

The binary command channel is `AA 55 | FUNC | LEN | PAYLOAD | CRC16_L CRC16_H`
(CRC-16/MODBUS).  Supported commands: `REQ_STATUS` (0x81), `CTRL` (0x87:
capture start/stop, reset statistics).

## Updating the model

1. Train and export from `../training/` (see `training/README.md`).
2. Regenerate the C array:

```bash
python ../training/generate_model_c_array.py \
  ../training/runs/<name>/gesture_model_int8.tflite \
  ../training/runs/<name> \
  --header-dir Core/Inc --source-dir Core/Src
```

3. Copy `normalization_mean`, `normalization_std`, `INPUT_SCALE` and
   `INPUT_ZERO_POINT` from `report.json` into
   `Core/Src/gesture_preprocess.c`.
4. Rebuild.

`ACCEL_X_TRIM_G` is an optional static X-axis mount calibration.  The public
placeholder is `0.0f`; set it only if your own board needs it.
