# Smart helmet — sensor interconnect (KiCad)

Schematic-only sheet derived from `include/config.h` (imu-firmware).

## Wiring diagrams

| Format | File |
|--------|------|
| **Excalidraw** (block diagram) | [smart-helmet-sensor-wiring.excalidraw](../docs/smart-helmet-sensor-wiring.excalidraw) |
| **PDF** (pin tables) | [smart-helmet-sensor-wiring.pdf](../docs/smart-helmet-sensor-wiring.pdf) |

Open the `.excalidraw` file at [excalidraw.com](https://excalidraw.com) or in VS Code with an Excalidraw extension.

Regenerate:

```bash
python3 hardware/scripts/generate_sensor_wiring_excalidraw.py
python3 hardware/scripts/generate_sensor_wiring_pdf.py
```

## Open (KiCad)

1. Install [KiCad](https://www.kicad.org/) 7 or 8.
2. Open `smart-helmet-sensors.kicad_pro`.
3. Schematic editor → sheet **Sensor connections**.

No PCB layout is included; assign footprints later if you build a board.

## Bus summary

| Net | ESP32 | Devices |
|-----|-------|---------|
| `SDA` | GPIO22 | ICM-20948, MAX30102, MAX30205 |
| `SCL` | GPIO21 | same I2C bus |
| `+3V3` / `GND` | 3V3, GND | all modules |
| `SPI_MOSI` | GPIO23 | OLED DIN |
| `SPI_SCK` | GPIO18 | OLED CLK |
| `OLED_DC` | GPIO2 | OLED DC |
| `OLED_CS` | GPIO5 | OLED CS |
| `OLED_RST` | GPIO4 | OLED RST |
| `MQ135_AO` | GPIO34 | MQ135 analog out |

## ICM-20948 straps (I2C mode)

- **AD0 → GND** → address `0x68`
- **NCS → 3.3 V** (disable SPI)
- Optional: 4.7 kΩ pull-ups on SDA/SCL to 3.3 V

Firmware also retries IMU at `0x69` (AD0 → 3.3 V).
