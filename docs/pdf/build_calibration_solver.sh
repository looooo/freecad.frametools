#!/usr/bin/env bash
# Build CALIBRATION_SOLVER.pdf from ../CALIBRATION_SOLVER.md (LaTeX math via pandoc).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
DOCS="$ROOT/docs"
PDF_DIR="$DOCS/pdf"
BUILD_MD="$PDF_DIR/.build.md"
OUT_PDF="$PDF_DIR/CALIBRATION_SOLVER.pdf"

python3 "$PDF_DIR/preprocess_md.py" "$DOCS/CALIBRATION_SOLVER.md" "$BUILD_MD"

pandoc "$BUILD_MD" \
  -o "$OUT_PDF" \
  --pdf-engine=pdflatex \
  -V documentclass=article \
  -V geometry:margin=2.5cm \
  -V lang=de \
  --include-in-header="$PDF_DIR/header.tex"

echo "Wrote $OUT_PDF"
