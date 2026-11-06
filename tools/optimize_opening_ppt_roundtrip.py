#!/usr/bin/env python3
"""Apply restrained native SVG polish to selected round-trip PPT pages."""
from pathlib import Path
import xml.etree.ElementTree as ET

NS = "http://www.w3.org/2000/svg"
ET.register_namespace("", NS)

LOCKED = {3, 7, 9}
ACCENT = "#00809C"
MUTED = "#D8E4E4"


def svg(tag: str, **attrs):
    return ET.Element(f"{{{NS}}}{tag}", attrs)


def add_polish(path: Path, page_no: int) -> None:
    if page_no in LOCKED:
        return
    tree = ET.parse(path)
    root = tree.getroot()

    # Keep the imported source objects intact; add only independent visual anchors.
    rail = svg(
        "rect",
        x="38",
        y="126",
        width="4",
        height="520",
        rx="2",
        fill=ACCENT,
        opacity="0.9",
        **{"data-pptx-object": "shape", "data-pptx-semantic-object": "decoration"},
    )
    root.append(rail)

    # Thin scientific-style footer rule and a progress marker improve page rhythm.
    rule = svg(
        "rect",
        x="58",
        y="678",
        width="1160",
        height="1.5",
        fill=MUTED,
        **{"data-pptx-object": "shape", "data-pptx-semantic-object": "decoration"},
    )
    root.append(rule)
    progress = svg(
        "rect",
        x="58",
        y="678",
        width=str(1160 * page_no / 16),
        height="1.5",
        fill=ACCENT,
        **{"data-pptx-object": "shape", "data-pptx-semantic-object": "decoration"},
    )
    root.append(progress)

    tree.write(path, encoding="utf-8", xml_declaration=False)


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("workspace", type=Path)
    args = parser.parse_args()
    folder = args.workspace / "authoring-svg-flat"
    for path in sorted(folder.glob("slide_*.svg")):
        page_no = int(path.stem.split("_")[1])
        add_polish(path, page_no)


if __name__ == "__main__":
    main()
