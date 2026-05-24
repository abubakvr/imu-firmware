#!/usr/bin/env python3
"""Generate Excalidraw sensor → ESP32 wiring diagram."""

import json
import random
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "hardware" / "docs" / "smart-helmet-sensor-wiring.excalidraw"

STROKE = "#1e1e1e"
ESP_BG = "#d0ebff"
I2C_BG = "#e7f5ff"
OLED_BG = "#fff3bf"
GAS_BG = "#d3f9d8"
ARROW = "#1971c2"
LABEL = "#495057"


def _id():
    return format(random.getrandbits(64), "x")[:16]


def _seed():
    return random.randint(1, 2**31 - 1)


def base(**kw):
    t = int(time.time() * 1000)
    el = {
        "angle": 0,
        "strokeColor": STROKE,
        "backgroundColor": "transparent",
        "fillStyle": "solid",
        "strokeWidth": 2,
        "strokeStyle": "solid",
        "roughness": 1,
        "opacity": 100,
        "groupIds": [],
        "frameId": None,
        "roundness": {"type": 3},
        "seed": _seed(),
        "version": 1,
        "versionNonce": _seed(),
        "isDeleted": False,
        "boundElements": [],
        "updated": t,
        "link": None,
        "locked": False,
    }
    el.update(kw)
    if "id" not in el:
        el["id"] = _id()
    return el


def rect(x, y, w, h, bg, label_lines, font=16):
    rid = _id()
    tid = _id()
    text = "\n".join(label_lines)
    r = base(
        id=rid,
        type="rectangle",
        x=x,
        y=y,
        width=w,
        height=h,
        backgroundColor=bg,
        boundElements=[{"type": "text", "id": tid}],
    )
    t = base(
        id=tid,
        type="text",
        x=x + 8,
        y=y + 8,
        width=w - 16,
        height=h - 16,
        text=text,
        originalText=text,
        fontSize=font,
        fontFamily=5,
        textAlign="left",
        verticalAlign="top",
        containerId=rid,
        autoResize=True,
        lineHeight=1.25,
    )
    return [r, t]


def label(x, y, text, size=14, color=LABEL):
    w = max(80, len(text) * 7)
    return base(
        type="text",
        x=x,
        y=y,
        width=w,
        height=24,
        text=text,
        originalText=text,
        fontSize=size,
        fontFamily=5,
        textAlign="left",
        verticalAlign="top",
        strokeColor=color,
        containerId=None,
        autoResize=True,
        lineHeight=1.25,
    )


def arrow(x1, y1, x2, y2, color=ARROW):
    dx, dy = x2 - x1, y2 - y1
    return base(
        type="arrow",
        x=x1,
        y=y1,
        width=abs(dx) or 1,
        height=abs(dy) or 1,
        strokeColor=color,
        strokeWidth=2,
        points=[[0, 0], [dx, dy]],
        lastCommittedPoint=None,
        startBinding=None,
        endBinding=None,
        startArrowhead=None,
        endArrowhead="arrow",
        roundness={"type": 2},
    )


def line_h(x, y, length, color=STROKE):
    return base(
        type="line",
        x=x,
        y=y,
        width=length,
        height=0,
        strokeColor=color,
        points=[[0, 0], [length, 0]],
        lastCommittedPoint=None,
        startBinding=None,
        endBinding=None,
        startArrowhead=None,
        endArrowhead=None,
    )


def main():
    elements = []
    # Title
    elements.append(
        label(
            40,
            20,
            "Smart Helmet — Sensor → ESP32 (from config.h)",
            size=22,
            color=STROKE,
        )
    )
    elements.append(
        label(
            40,
            52,
            "I2C bus: SDA=GPIO22, SCL=GPIO21 @ 100 kHz  ·  Label wires SDA→22, SCL→21, etc.",
            size=13,
            color=LABEL,
        )
    )

    # ESP32 block (center-right)
    ex, ey, ew, eh = 520, 140, 220, 280
    elements.extend(
        rect(
            ex,
            ey,
            ew,
            eh,
            ESP_BG,
            [
                "ESP32 DOIT DevKit V1",
                "",
                "GPIO22  ← I2C SDA",
                "GPIO21  ← I2C SCL",
                "GPIO23  ← SPI MOSI",
                "GPIO18  ← SPI SCK",
                "GPIO2   ← OLED DC",
                "GPIO5   ← OLED CS",
                "GPIO4   ← OLED RST",
                "GPIO32  ← MQ-4 AO",
                "GPIO33  ← MQ-7 AO",
                "GPIO34  ← MQ-135 AO",
                "GPIO35  ← MQ-136 AO",
                "",
                "3V3, GND → all modules",
            ],
            font=15,
        )
    )

    # I2C cluster (left)
    ix, iw = 40, 200
    elements.extend(
        rect(
            ix,
            100,
            iw,
            95,
            I2C_BG,
            [
                "ICM-20948 IMU",
                "SDA, SCL, VCC, GND",
                "AD0 → GND (addr 0x68)",
                "NCS → 3.3V (I2C mode)",
            ],
        )
    )
    elements.extend(
        rect(
            ix,
            210,
            iw,
            80,
            I2C_BG,
            ["MAX30102 HR/SpO₂", "SDA, SCL, VIN, GND", "I2C 0x57"],
        )
    )
    elements.extend(
        rect(
            ix,
            305,
            iw,
            80,
            I2C_BG,
            ["MAX30205 temp", "SDA, SCL, VCC, GND", "I2C 0x4F"],
        )
    )

    # OLED (top right of ESP)
    elements.extend(
        rect(
            780,
            100,
            190,
            150,
            OLED_BG,
            [
                "SPI OLED SH1106",
                "DIN  → GPIO23",
                "CLK  → GPIO18",
                "DC   → GPIO2",
                "CS   → GPIO5",
                "RST  → GPIO4",
                "VCC, GND",
            ],
        )
    )

    # MQ gas sensors (ADC1 — one AO pin each)
    mq_sensors = [
        ("MQ-135", 34, 380),
        ("MQ-136", 35, 530),
        ("MQ-4", 32, 680),
        ("MQ-7", 33, 830),
    ]
    for name, gpio, bx in mq_sensors:
        elements.extend(
            rect(
                bx,
                470,
                120,
                72,
                GAS_BG,
                [name, f"AO → GPIO{gpio}", "VCC → 3V3", "GND"],
                font=14,
            )
        )
        elements.append(arrow(bx + 60, 470, ex + ew // 2, ey + eh))
        elements.append(label(bx + 10, 448, f"ADC1 · GPIO{gpio}", size=11, color=ARROW))

    # I2C bus spine
    bus_x = 280
    elements.append(line_h(bus_x, 175, 220, "#868e96"))
    elements.append(line_h(bus_x, 250, 220, "#868e96"))
    elements.append(label(bus_x + 60, 158, "SDA / SCL bus", size=12))

    # Arrows: I2C modules → bus → ESP
    for sy in (147, 250, 345):
        elements.append(arrow(ix + iw, sy, bus_x, sy))
    elements.append(arrow(bus_x + 220, 210, ex, 200))
    elements.append(label(350, 188, "→ GPIO22 SDA", size=12, color=ARROW))
    elements.append(label(350, 228, "→ GPIO21 SCL", size=12, color=ARROW))

    # OLED → ESP SPI pins
    elements.append(arrow(780, 175, ex + ew, 200))
    elements.append(label(680, 155, "SPI", size=13, color=ARROW))

    # Power rails (dashed style via grey lines)
    elements.append(line_h(40, 560, 960, "#fab005"))
    elements.append(label(40, 542, "+3V3 rail (ESP 3V3 pin) → VCC on every module", size=12, color="#e67700"))
    elements.append(line_h(40, 590, 960, "#495057"))
    elements.append(label(40, 572, "GND rail (common)", size=12, color="#495057"))

    doc = {
        "type": "excalidraw",
        "version": 2,
        "source": "https://excalidraw.com",
        "elements": elements,
        "appState": {
            "gridSize": 20,
            "viewBackgroundColor": "#ffffff",
            "currentItemStrokeColor": STROKE,
            "currentItemBackgroundColor": "transparent",
            "currentItemFillStyle": "solid",
            "currentItemStrokeWidth": 2,
            "currentItemRoughness": 1,
            "currentItemOpacity": 100,
            "currentItemFontFamily": 5,
            "currentItemFontSize": 16,
            "scrollX": 0,
            "scrollY": 0,
            "zoom": {"value": 0.8},
        },
        "files": {},
    }

    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(doc, indent=2), encoding="utf-8")
    print(f"Wrote {OUT}")
    print("Open at https://excalidraw.com → Open → select this file")


if __name__ == "__main__":
    main()
