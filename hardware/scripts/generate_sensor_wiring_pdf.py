#!/usr/bin/env python3
"""Generate sensor → ESP32 wiring PDF from imu-firmware config."""

from pathlib import Path

try:
    from reportlab.lib import colors
    from reportlab.lib.enums import TA_CENTER, TA_LEFT
    from reportlab.lib.pagesizes import A4, landscape
    from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
    from reportlab.lib.units import mm
    from reportlab.platypus import (
        Paragraph,
        SimpleDocTemplate,
        Spacer,
        Table,
        TableStyle,
    )
except ImportError:
    raise SystemExit("Install reportlab: pip install reportlab")

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "hardware" / "docs" / "smart-helmet-sensor-wiring.pdf"

# Sensor pin label → ESP32 / net (from include/config.h + driver notes)
SHARED_I2C = [
    ["SDA", "GPIO22", "I2C data (all I2C devices)"],
    ["SCL", "GPIO21", "I2C clock (all I2C devices)"],
    ["VCC / VIN", "3.3 V", "ESP32 3V3 rail"],
    ["GND", "GND", "Common ground"],
]

ICM20948 = [
    ["SDA", "GPIO22", "Shared I2C bus"],
    ["SCL", "GPIO21", "Shared I2C bus"],
    ["VCC / VDD", "3.3 V", ""],
    ["GND", "GND", ""],
    ["AD0", "GND", "I2C address 0x68 (tie high for 0x69)"],
    ["NCS / CS", "3.3 V", "I2C mode — must be high, not SPI"],
    ["SDO / SA0", "—", "Leave default (I2C)"],
]

MAX30102 = [
    ["SDA", "GPIO22", "Shared I2C bus"],
    ["SCL", "GPIO21", "Shared I2C bus"],
    ["VIN", "3.3 V", ""],
    ["GND", "GND", ""],
    ["INT", "—", "Not used in firmware"],
]

MAX30205 = [
    ["SDA", "GPIO22", "Shared I2C bus"],
    ["SCL", "GPIO21", "Shared I2C bus"],
    ["VCC", "3.3 V", ""],
    ["GND", "GND", ""],
    ["A0 / A1 / A2", "—", "Straps set address 0x4F on your board"],
]

OLED = [
    ["DIN / MOSI", "GPIO23", "SPI data (VSPI)"],
    ["CLK / SCK", "GPIO18", "SPI clock"],
    ["DC", "GPIO2", "Data / command select"],
    ["CS", "GPIO5", "Chip select"],
    ["RST / RES", "GPIO4", "Reset"],
    ["VCC", "3.3 V", ""],
    ["GND", "GND", ""],
]

MQ135 = [
    ["AO (analog out)", "GPIO34", "ADC1 — air quality"],
    ["VCC", "3.3 V", ""],
    ["GND", "GND", ""],
    ["DO", "—", "Not used in firmware"],
]

MQ136 = [
    ["AO (analog out)", "GPIO35", "ADC1"],
    ["VCC", "3.3 V", ""],
    ["GND", "GND", ""],
    ["DO", "—", "Not used in firmware"],
]

MQ4 = [
    ["AO (analog out)", "GPIO32", "ADC1 — methane"],
    ["VCC", "3.3 V", ""],
    ["GND", "GND", ""],
    ["DO", "—", "Not used in firmware"],
]

MQ7 = [
    ["AO (analog out)", "GPIO33", "ADC1 — carbon monoxide"],
    ["VCC", "3.3 V", ""],
    ["GND", "GND", ""],
    ["DO", "—", "Not used in firmware"],
]

SUMMARY = [
    ["Net / function", "ESP32 pin", "Modules"],
    ["I2C SDA", "GPIO22", "ICM-20948, MAX30102, MAX30205"],
    ["I2C SCL", "GPIO21", "ICM-20948, MAX30102, MAX30205"],
    ["SPI MOSI", "GPIO23", "OLED DIN"],
    ["SPI SCK", "GPIO18", "OLED CLK"],
    ["OLED DC", "GPIO2", "OLED"],
    ["OLED CS", "GPIO5", "OLED"],
    ["OLED RST", "GPIO4", "OLED"],
    ["MQ-135 AO", "GPIO34", "Air quality"],
    ["MQ-136 AO", "GPIO35", "Air quality"],
    ["MQ-4 AO", "GPIO32", "Methane"],
    ["MQ-7 AO", "GPIO33", "Carbon monoxide"],
    ["+3.3 V", "3V3 pin", "All sensors"],
    ["Ground", "GND pin", "All sensors"],
]


def table(data, col_widths=None):
    t = Table(data, colWidths=col_widths, repeatRows=1)
    t.setStyle(
        TableStyle(
            [
                ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#1a1a2e")),
                ("TEXTCOLOR", (0, 0), (-1, 0), colors.white),
                ("FONTNAME", (0, 0), (-1, 0), "Helvetica-Bold"),
                ("FONTSIZE", (0, 0), (-1, 0), 9),
                ("FONTNAME", (0, 1), (-1, -1), "Helvetica"),
                ("FONTSIZE", (0, 1), (-1, -1), 8.5),
                ("ALIGN", (0, 0), (-1, -1), "LEFT"),
                ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
                ("GRID", (0, 0), (-1, -1), 0.4, colors.grey),
                ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.white, colors.HexColor("#f4f6fa")]),
                ("TOPPADDING", (0, 0), (-1, -1), 5),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 5),
                ("LEFTPADDING", (0, 0), (-1, -1), 6),
            ]
        )
    )
    return t


def section(title, rows, styles, w):
    story = []
    story.append(Paragraph(title, styles["H2"]))
    story.append(Spacer(1, 2 * mm))
    story.append(table(rows, w))
    story.append(Spacer(1, 5 * mm))
    return story


def main():
    OUT.parent.mkdir(parents=True, exist_ok=True)
    doc = SimpleDocTemplate(
        str(OUT),
        pagesize=landscape(A4),
        leftMargin=14 * mm,
        rightMargin=14 * mm,
        topMargin=12 * mm,
        bottomMargin=12 * mm,
        title="Smart Helmet Sensor Wiring",
    )
    styles = getSampleStyleSheet()
    styles.add(
        ParagraphStyle(
            name="H1",
            parent=styles["Heading1"],
            fontSize=16,
            textColor=colors.HexColor("#1a1a2e"),
            spaceAfter=4,
        )
    )
    styles.add(
        ParagraphStyle(
            name="H2",
            parent=styles["Heading2"],
            fontSize=11,
            textColor=colors.HexColor("#2d4a7a"),
            spaceBefore=2,
            spaceAfter=2,
        )
    )
    styles.add(
        ParagraphStyle(
            name="Sub",
            parent=styles["Normal"],
            fontSize=9,
            textColor=colors.grey,
        )
    )
    styles.add(
        ParagraphStyle(
            name="Note",
            parent=styles["Normal"],
            fontSize=8,
            textColor=colors.HexColor("#333333"),
        )
    )

    cw = [52 * mm, 38 * mm, 165 * mm]
    story = []
    story.append(Paragraph("Smart Helmet — Sensor to ESP32 Wiring", styles["H1"]))
    story.append(
        Paragraph(
            "ESP32 DOIT DevKit V1 · from <i>include/config.h</i> (imu-firmware) · "
            "I2C @ 100 kHz · schematic-only reference",
            styles["Sub"],
        )
    )
    story.append(Spacer(1, 4 * mm))

    story.append(Paragraph("Quick reference — all connections", styles["H2"]))
    story.append(Spacer(1, 2 * mm))
    story.append(table(SUMMARY, [48 * mm, 32 * mm, 175 * mm]))
    story.append(Spacer(1, 6 * mm))

    hdr = ["Sensor pin", "Connect to", "Notes"]
    story += section("Shared I2C bus (parallel wiring)", [hdr] + SHARED_I2C, styles, cw)
    story += section("ICM-20948 (9-axis IMU) — I2C 0x68", [hdr] + ICM20948, styles, cw)
    story += section("MAX30102 (heart rate / SpO₂) — I2C 0x57", [hdr] + MAX30102, styles, cw)
    story += section("CJMCU-30205 / MAX30205 (temperature) — I2C 0x4F", [hdr] + MAX30205, styles, cw)
    story += section("0.96″ SPI OLED (SH1106)", [hdr] + OLED, styles, cw)
    story += section("MQ-135 (air quality) — ADC1", [hdr] + MQ135, styles, cw)
    story += section("MQ-136 — ADC1", [hdr] + MQ136, styles, cw)
    story += section("MQ-4 (methane) — ADC1", [hdr] + MQ4, styles, cw)
    story += section("MQ-7 (CO) — ADC1", [hdr] + MQ7, styles, cw)

    story.append(
        Paragraph(
            "<b>Labelling tip:</b> On the bench, label harness wires "
            "<font face='Courier'>SDA→22</font>, <font face='Courier'>SCL→21</font>, "
            "<font face='Courier'>MOSI→23</font>, <font face='Courier'>SCK→18</font>, "
            "<font face='Courier'>MQ135→34</font>, <font face='Courier'>MQ136→35</font>, "
            "<font face='Courier'>MQ4→32</font>, <font face='Courier'>MQ7→33</font>. "
            "Firmware may swap-retry SDA/SCL if init fails; working config is GPIO22=SDA, GPIO21=SCL.",
            styles["Note"],
        )
    )

    doc.build(story)
    print(f"Wrote {OUT}")


if __name__ == "__main__":
    main()
