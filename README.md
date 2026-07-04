\# STM32 Multi-Sensor Logger (MPU6050 + BME280)



Firmware for an STM32 Nucleo-F446RE that reads acceleration data from an MPU6050

and temperature/pressure from a BME280 over a shared I2C bus, streaming CSV data

over UART with user-controlled start/stop logging and automatic fault recovery.



\## Features



\- Polling-based I2C reads for MPU6050 (accelerometer) and BME280 (temperature/pressure)

\- Start/stop data logging on demand, via either:

&#x20; - Pressing \*\*Enter\*\* in a serial terminal

&#x20; - Pressing the onboard \*\*User button (B1)\*\*

\- CSV-formatted output, ready to import directly into Excel, Python, or MATLAB

\- Per-sample data-validity flags (`AccelOK`, `BmeOK`) so a failed sensor read is

&#x20; never silently hidden behind a stale value

\- Automatic I2C bus recovery: after several consecutive read failures, the

&#x20; firmware re-initializes the I2C peripheral and resumes — without needing a

&#x20; manual power cycle

\- Safe failure handling: the recovery routine reports failures instead of

&#x20; calling a blocking error handler, so the rest of the system (UART, button)

&#x20; stays responsive even if the sensor bus is having a bad day



\## Hardware



| Component        | Details                          |

|-------------------|-----------------------------------|

| MCU board          | STM32 Nucleo-F446RE               |

| Accelerometer      | MPU6050 (I2C address `0x68`)      |

| Temp/Pressure      | BME280 (I2C address `0x76`)       |

| Interface          | I2C1, 100 kHz                     |

| Serial console     | USART2 (ST-LINK Virtual COM Port), 115200 baud |



\## Output format



\- `AccelX/Y/Z\_g`: acceleration in units of g (9.81 m/s²)

\- `Temp\_C`: temperature in °C

\- `Press\_hPa`: barometric pressure in hPa

\- `AccelOK` / `BmeOK`: `1` if that sensor's I2C read succeeded this cycle, `0` if

&#x20; it failed (in which case the printed value is the last known-good reading)



\## How it works



1\. On boot, the MPU6050 is woken from sleep and the BME280's factory calibration

&#x20;  constants are read once.

2\. The firmware then idles, printing a CSV header and waiting for a start signal.

3\. Pressing \*\*Enter\*\* or the \*\*User button\*\* toggles a `streaming\_active` flag.

&#x20;  While active, the main loop polls both sensors roughly every 100 ms and

&#x20;  prints one CSV row per cycle.

4\. If an I2C transaction fails, the last known value is kept (flagged `0`

&#x20;  instead of `1`) rather than left undefined. After 5 consecutive failures,

&#x20;  the firmware automatically re-initializes the I2C peripheral and retries.



\## Build \& flash



1\. Open the `sensor\_talking` project in STM32CubeIDE.

2\. Build: \*\*Project → Build Project\*\*.

3\. Flash: connect the Nucleo board via USB and click \*\*Run\*\* (or \*\*Debug\*\*).

4\. Open a serial terminal (e.g. PuTTY) on the board's COM port at 115200 baud,

&#x20;  8N1, no flow control.



\## Debugging notes / lessons learned



This project involved diagnosing several real embedded issues along the way:



\- \*\*Linker errors from non-existent HAL functions\*\* — an early draft declared

&#x20; its own `HAL\_I2C\_Transmit`/`HAL\_I2C\_Receive` prototypes that don't actually

&#x20; exist in the STM32 HAL; fixed by switching to the real

&#x20; `HAL\_I2C\_Master\_Transmit`/`HAL\_I2C\_Master\_Receive`.

\- \*\*Silent output buffering\*\* — `printf` output was sitting in an internal

&#x20; buffer and never reaching the UART, since the retargeted stdout isn't

&#x20; recognized as a real terminal. Fixed with `setvbuf(stdout, NULL, \_IONBF, 0)`

&#x20; and explicit `fflush()` calls.

\- \*\*A recovery routine that made things worse\*\* — the first I2C recovery

&#x20; implementation called the CubeMX-generated `MX\_I2C1\_Init()`, which calls

&#x20; `Error\_Handler()` (`\_\_disable\_irq(); while(1){}`) on failure. During a real

&#x20; bus glitch this froze the entire MCU, including the button interrupt.

&#x20; Fixed by writing a safe re-init that reports failure instead of halting.

\- Pressure readings show a calibration offset, uncorrected





\## License



MIT — see \[LICENSE](LICENSE).

