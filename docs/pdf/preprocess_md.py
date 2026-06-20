#!/usr/bin/env python3
"""Preprocess CALIBRATION_SOLVER.md for pandoc PDF build."""

import re
import sys
from pathlib import Path


def main() -> int:
    src = Path(sys.argv[1])
    dst = Path(sys.argv[2])
    text = src.read_text(encoding="utf-8")

    text = re.sub(r"\\\[", "$$", text)
    text = re.sub(r"\\\]", "$$", text)
    text = re.sub(r"\\\(", "$", text)
    text = re.sub(r"\\\)", "$", text)

    for a, b in [
        ("\u201e", '"'),
        ("\u201c", '"'),
    ]:
        text = text.replace(a, b)

    parts = text.split("$")
    for i, part in enumerate(parts):
        if i % 2 == 1:
            parts[i] = part.replace("°", r"^\circ")
        else:
            parts[i] = re.sub(r"(\d+)°", r"$\1^\\circ$", part)
    text = "$".join(parts)

    dst.write_text(text, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
