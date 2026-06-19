# Bild-Überlagerung und Maßkalibrierung

Implementierungsbeschreibung für das Frame-Tools-Workbench.

Ziel: Zwei Bilder anhand von Feature-Paaren ausrichten, danach über
Referenzlinien metrisch kalibrieren. Getrennte FreeCAD-Objekte und
eigenständige Befehle.

---

## Übersicht

```text
[Bilder in FreeCAD laden (Draft/Image)]
    → [Aligned Image: in Coin-Objekt umwandeln]
    → [Feature-Paare erstellen]
    → [Image Overlay: Bilder + Feature-Paare auswählen]
    → [Referenzlinien erstellen]
    → [Scale Solver: Bilder + Referenzlinien auswählen]
```

Image Overlay und Scale Solver wandeln `Image::ImagePlane` bei Bedarf
automatisch in `AlignedImage` um.

---

## FreeCAD-Objekte

### AlignedImage (`Part::FeaturePython`)

Texturiertes Bild als Parallelogramm über Coin (volle affine Darstellung
inkl. Scherung).

| Property | Typ | Beschreibung |
|----------|-----|--------------|
| `ImageFile` | File | Pfad zur Bilddatei |
| `Corner0` | Vector | Ursprung (UV 0,0) |
| `CornerX` | Vector | Ecke entlang U-Richtung (UV 1,0) |
| `CornerY` | Vector | Ecke entlang V-Richtung (UV 0,1) |
| `SourceImage` | Link | Ursprüngliches ImagePlane (optional) |

Die vierte Ecke ist `CornerX + CornerY - Corner0`. Der ViewProvider nutzt
`SoTexture2`, `SoCoordinate3`, `SoTextureCoordinate2` und `SoFaceSet`.

### FeaturePair (`Part::FeaturePython`)

Ein Objekt pro übereinstimmendem Punktpaar.

| Property | Typ | Beschreibung |
|----------|-----|--------------|
| `RefPoint` | Vector | Punkt auf Referenzbild |
| `MovPoint` | Vector | Entsprechender Punkt auf Bild 2 |

`execute()` erzeugt eine Verbindungslinie.

### ReferenceLine (`Part::FeaturePython`)

Ein Objekt pro Maßlinie.

| Property | Typ | Beschreibung |
|----------|-----|--------------|
| `Start` | Vector | Linienanfang |
| `End` | Vector | Linienende |
| `TargetLength` | Length | Soll-Länge |

`execute()` erzeugt eine `Part::Edge`.

---

## Befehle (Toolbar „Image")

| Befehl | Icon | Funktion |
|--------|------|----------|
| **Aligned Image** | `image_align.svg` | ImagePlane → AlignedImage (Coin) |
| **Feature Pair** | `feature_pair.svg` | Zwei Punkte wählen → `FeaturePair` |
| **Reference Line** | `reference_line.svg` | Soll-Länge + Linie → `ReferenceLine` |
| **Image Overlay** | `image_overlay.svg` | 2 Bilder + ≥3 Feature-Paare → affine Ausrichtung |
| **Scale Solver** | `scale_solver.svg` | ≥2 Referenzlinien → Skalierung/Scherung |

Bilder werden zunächst mit **Draft → Insert Image** geladen.

---

## Dateistruktur

```text
freecad/frametools/
  image_objects.py      # AlignedImage, FeaturePair, ReferenceLine + ViewProvider
  image_tools.py        # Mathe + Befehlslogik
  reference_line.ui     # Dialog für Referenzlinie (Soll-Länge)
  commands.py           # 5 Image-Commands
  init_gui.py           # Toolbar Image
```

---

## Nutzer-Workflow

1. Zwei Bilder mit **Draft → Insert Image** laden
2. **Aligned Image** — Bilder auswählen und umwandeln (optional; Overlay/Solver tun das automatisch)
3. **Feature Pair** — mindestens 3 Paare anklicken
4. Referenzbild, Bild 2 und alle Feature-Paare auswählen
5. **Image Overlay** — Bild 2 wird auf Referenz ausgerichtet
6. **Reference Line** — Linien mit bekannten Maßen (mindestens 2)
7. Bilder, Feature-Paare und Referenzlinien auswählen
8. **Scale Solver** — metrische Kalibrierung

---

## Mathematik

### Image Overlay — Affine Transformation (2×2 + Translation)

6 Parameter, mindestens 3 Feature-Paare. Lösung per `numpy.linalg.lstsq`.

```text
RefPoint = M @ MovPoint + t
```

Wird auf die drei Ecken von Bild 2 (`Corner0`, `CornerX`, `CornerY`) angewendet.

### Scale Solver — Skalierung + Scherung (symmetrisch, 3 Parameter)

```text
M = | sx  sh |
    | sh  sy |
```

Keine Rotation. Lösung per `scipy.optimize.least_squares` aus Referenzlinien.

Transformation in lokalen Bildkoordinaten (metrisch entlang U/V-Kanten),
dann zurück in Welt-Ecken.

---

## Auswahl-Regeln

### Image Overlay

- Genau **2** Bildobjekte (erstes = Referenz, zweites = Bild 2)
- Mindestens **3** `FeaturePair`-Objekte

### Scale Solver

- Mindestens **2** `ReferenceLine`-Objekte mit `TargetLength > 0`
- Bilder, Feature-Paare und Referenzlinien als Transformationsziele

---

## Abhängigkeiten

- `numpy`, `scipy`, `pivy` (Coin3D)
- Keine Draft-Abhängigkeit für Punktauswahl (eigener Klick-Handler)
