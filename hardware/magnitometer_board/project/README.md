# AVR Magnetometer Logger with RTC, SD Card and UART Interface

A firmware project for the ATmega1284 MCU using:

- **RTC PCF8563T** over I2C for real-time clock and alarm interrupt
- **QMC5883L 3-axis magnetometer** over I2C with configurable options
- **SD Card** over SPI with FatFs for data logging
- **UART** (9600 baud) with RX/TX interrupt support for commands
- **Power saving** with `SLEEP_MODE_IDLE`, wakeup on INT0 and UART RX

## 🔧 Features

- RTC alarm every X minutes (`ALARM_INTERVAL_SEC`) triggers data logging
- Logs `unix_time,x,y,z,temp_mag,temp_mcu` to `log.txt`
- Supports the following UART commands:
  - `TIME` – return current Unix timestamp
  - `MEASURE` – perform one measurement and log
  - `LOG <unix> <n>` – (not yet implemented) return `n` log lines from time `<unix>`
  - `FORMAT_SD` – format the SD card and recreate `config.txt` and `log.txt`
  - `REBOOT` – software reset using WDT

## 📂 Project Structure

project/
├── main.c # Main logic
├── uart.c/.h # UART with RX/TX interrupt buffers
├── i2c.c/.h # Basic I2C master
├── pcf8563.c/.h # RTC PCF8563 driver
├── qmc5883l.c/.h # QMC5883L magnetometer driver
├── fatfs/ # FatFs source files
├── config.txt # Configuration file (generated on first boot)
└── log.txt # Log file (append-only)

## ⚙️ Configuration (`config.txt`)

File stored on SD card with settings:
ALARM_INTERVAL=120
MAG_ODR=50
MAG_RANGE=2
MAG_MODE=1


Values:
- `ALARM_INTERVAL`: seconds between automatic logs
- `MAG_ODR`: Output data rate (10/50/100/200 Hz)
- `MAG_RANGE`: Measurement range (2 or 8 Gauss)
- `MAG_MODE`: 0 = standby, 1 = continuous

## 🧪 Dependencies

- AVR-GCC (`avr-gcc`, `avr-libc`)
- `avrdude` for flashing
- FatFs (already included)

## 🔨 Build & Flash

1. Compile using `avr-gcc`:
   avr-gcc -mmcu=atmega1284p -Os -DF_CPU=16000000UL \
       main.c uart.c i2c.c pcf8563.c qmc5883l.c fatfs/ff.c fatfs/diskio.c -o firmware.elf
   avr-objcopy -O ihex firmware.elf firmware.hex
2.Flash to device (replace usbasp and port as needed):
	avrdude -c usbasp -p m1284p -U flash:w:firmware.hex
	
##	💡 Notes
INT0 (PD2) used for RTC alarm interrupt

UART wake-up is enabled via RX interrupt

Logging occurs every configured interval (ALARM_INTERVAL)

Files log.txt and config.txt are auto-created if missing

MCU enters Idle sleep mode when idle

##  🧠 License
MIT License or Public Domain. Use freely.