# STM32 Multi-Sensor Logger (MPU6050 + BME280)

Firmware for an STM32 Nucleo-F446RE that reads acceleration from an MPU6050 and
temperature/pressure from a BME280 over a shared I2C bus, and streams CSV over
UART with user-controlled start/stop logging and automatic I2C re-initialization
after repeated read failures.

## Features

- Polling-based I2C reads: MPU6050 (accelerometer), BME280 (temperature, pressure)
- Start/stop logging on demand, via either:
  - **Enter** in a serial terminal
  - the onboard **User button (B1)**
- CSV output with a millisecond timestamp, ready for Excel, Python or MATLAB
- Per-sample validity flags (`AccelOK`, `BmeOK`) so a failed read is never
  silently hidden behind a stale value
- I2C re-initialization: after 5 consecutive failed cycles the firmware
  re-initializes the I2C peripheral and resumes without a manual reset
- Non-blocking failure handling: the recovery routine reports failure instead of
  calling the CubeMX `Error_Handler()`, so UART and the button stay responsive

## Hardware

| Component      | Details                                        |
|----------------|------------------------------------------------|
| MCU board      | STM32 Nucleo-F446RE                            |
| Accelerometer  | MPU6050 (I2C address `0x68`)                   |
| Temp/Pressure  | BME280 (I2C address `0x76`)                    |
| Interface      | I2C1, 100 kHz                                  |
| Serial console | USART2 (ST-LINK Virtual COM Port), 115200 8N1  |

<!-- Add a wiring photo or table here: SCL/SDA pins, pull-ups, 3V3/GND -->

## Output format

```
AccelX_g,AccelY_g,AccelZ_g,Temp_C,Press_hPa,AccelOK,BmeOK.
Delete the bullet beginning `Time_ms`: milliseconds since boot
```

- `Time_ms`: milliseconds since boot (`HAL_GetTick()`)
- `AccelX/Y/Z_g`: acceleration in g (9.81 m/s²)
- `Temp_C`: temperature in °C
- `Press_hPa`: pressure in hPa
- `AccelOK` / `BmeOK`: `1` if that sensor's read succeeded this cycle, `0` if it
  failed (the printed value is then the last known-good reading)

<!-- Paste 5-10 real sample rows here, e.g. in docs/sample.csv -->

## How it works

1. On boot, the MPU6050 is woken from sleep and the BME280 calibration constants
   are read once.
2. The firmware prints the CSV header and waits for a start signal.
3. **Enter** or **B1** toggles `streaming_active`. While active, the main loop
   polls both sensors about every 100 ms and prints one CSV row per cycle.
4. A failed I2C transaction keeps the last value and sets its flag to `0`.
   After 5 consecutive failed cycles the I2C peripheral is re-initialized
   (with a 500 ms cooldown between attempts).

### Limits of the recovery

This is a software-level recovery (peripheral re-init). If a slave holds SDA low
mid-transaction, the bus needs a GPIO clock-out sequence, which is not implemented.

## Build and flash

1. Open the `sensor_talking` project in STM32CubeIDE.
2. **Project → Build Project**.
3. Connect the Nucleo board over USB and click **Run** (or **Debug**).
4. Open a serial terminal (e.g. PuTTY) on the board's COM port: 115200 baud,
   8N1, no flow control.

## Debugging notes

- **Linker errors from non-existent HAL functions:** an early draft declared its
  own `HAL_I2C_Transmit`/`HAL_I2C_Receive`, which don't exist in the STM32 HAL.
  Fixed by using `HAL_I2C_Master_Transmit`/`HAL_I2C_Master_Receive`.
- **Silent output buffering:** `printf` output sat in an internal buffer and never
  reached the UART. Fixed with `setvbuf(stdout, NULL, _IONBF, 0)` and `fflush()`.
- **A recovery routine that made things worse:** the first version called the
  CubeMX-generated `MX_I2C1_Init()`, whose failure path is `Error_Handler()`
  (`__disable_irq(); while(1){}`). On a real bus glitch this froze the whole MCU,
  including the button interrupt. Fixed with a safe re-init that returns a status.
- **Pressure offset:** MEASURE THIS: state the offset, e.g. "+X hPa versus a
  weather-station reading", and say whether the reference was station pressure or
  sea-level-adjusted pressure.

## License

MIT, see LICENSE.
