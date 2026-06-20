# Bild-Überlagerung und Maßkalibrierung

Kurzüberblick für den Image-Workflow im Frame-Tools-Workbench.

**Vollständige Dokumentation:** [IMAGE_TOOLS.md](IMAGE_TOOLS.md) — Objekte, Befehle, Mathematik, Interaktion und Implementierungsdetails.

**Kalibrierungs-Solver (Formeln):** [CALIBRATION_SOLVER.md](CALIBRATION_SOLVER.md)

**Kameramodell (Entwurf):** [CAMERA_MODEL.md](CAMERA_MODEL.md) — Gitter-Kalibrierung,
B-Spline-Entzerrung, Anwendung beim Foto-Laden.

---

## Workflow

```text
[Draft → Insert Image]
    → Feature-Paare (≥ 3)
    → Image Overlay
    → Referenzlinien mit Soll-Längen
    → Scale Solver
```

Optional vorab: **Aligned Image** (Coin-Darstellung). Overlay und Solver wandeln `ImagePlane` bei Bedarf automatisch.

---

## Befehle (Toolbar „Image“)

| Befehl | Funktion |
|--------|----------|
| Aligned Image | `ImagePlane` → `AlignedImage` |
| Feature Pair | Zwei Punkte → `FeaturePair` |
| Image Overlay | 2 Bilder + ≥ 3 Paare → Homographie |
| Image Calibration | Kalibrierungs-Objekt + Sketch (Sketcher) |
| Calibration Sketch | Sketch bearbeiten / neuen Sketch öffnen |
| Calibration Constraints | Soll-Längen und Winkel-Bedingungen |
| Calibration Solve | Bild kalibrieren, neuer Sketch |
| *Kameramodell kalibrieren* | *geplant:* Display-Muster 16×8, Foto, `CameraModel` — [CAMERA_MODEL.md §13](CAMERA_MODEL.md#13-geplanter-freecad-workflow-display-muster) |
| *Bild mit Entzerrung laden* | *geplant:* Foto + Entzerrung, rechteckiges `AlignedImage`, schwarze Ränder — [CAMERA_MODEL.md §13.5](CAMERA_MODEL.md#135-befehl-bild-mit-entzerrung-laden) |

---

## Objekte

- **AlignedImage** — Bild als UV-Quad mit `WarpMatrix`
- **FeaturePair** — `RefPoint` + `MovPoint`
- **ReferenceLine** — `Start`, `End`, `TargetLength`, `Image`

Details und Properties: [IMAGE_TOOLS.md#freecad-objekte](IMAGE_TOOLS.md#freecad-objekte).
