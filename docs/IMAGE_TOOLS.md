# Image Tools — Dokumentation

Implementierungs- und Nutzerbeschreibung für die Bild-Werkzeuge des Frame-Tools-Workbench.

**Ziel:** Zwei Bilder anhand von Feature-Paaren ausrichten, danach über Referenzlinien mit bekannten Maßen metrisch kalibrieren. Alle Hilfsgeometrie (Punkte, Linien) bleibt als eigene FreeCAD-Objekte erhalten und wird bei Transformationen konsistent mitbewegt.

**Quellcode:**

| Datei | Inhalt |
|-------|--------|
| `freecad/frametools/image_objects.py` | FreeCAD-Objekte und ViewProvider (Coin-Darstellung) |
| `freecad/frametools/image_tools.py` | Befehlslogik, Interaktion, Kalibrierungs-Workflow |
| `freecad/frametools/image_homography.py` | Homographie, UV-Koordinaten |
| `freecad/frametools/image_constraint_solver.py` | Kalibrierungs-Solver (Längen, Winkel, Optimierung) |
| `freecad/frametools/image_point_alignment.py` | Punkt-/Eck-Ausrichtung, Warp-Matrix |
| `freecad/frametools/reference_line.ui` | Dialog für Referenzlinien (Soll-Länge) |
| `freecad/frametools/commands.py` | Toolbar-Befehle |
| `freecad/frametools/init_gui.py` | Toolbar „Image“ |

---

## Inhaltsverzeichnis

1. [Gesamtworkflow](#gesamtworkflow)
2. [FreeCAD-Objekte](#freecad-objekte)
3. [Toolbar-Befehle](#toolbar-befehle)
4. [Interaktion und Punktauswahl](#interaktion-und-punktauswahl)
5. [Koordinaten und Bildabbildung](#koordinaten-und-bildabbildung)
6. [Image Overlay (Homographie)](#image-overlay-homographie)
7. [Scale Solver (Eckpunkt-Kalibrierung)](#scale-solver-eckpunkt-kalibrierung)
8. [Auswahl-Regeln](#auswahl-regeln)
9. [Abhängigkeiten](#abhängigkeiten)

Weitere Dokumente: [README.md](README.md), [CALIBRATION_SOLVER.md](CALIBRATION_SOLVER.md)
(mathematische Solver-Beschreibung), [CAMERA_MODEL.md](CAMERA_MODEL.md) (Entwurf:
Objektiv-Entzerrung aus Gitter-Foto, B-Spline, Coin-Darstellung).

---

## Gesamtworkflow

```text
[Draft → Insert Image: Bilder laden]
    → [Aligned Image]  Bild in Coin-Darstellung umwandeln (optional)
    → [Feature Pair]   mindestens 3 übereinstimmende Punktpaare
    → [Image Overlay]  Bild 2 projektiv auf Referenz ausrichten
    → [Reference Line] Maßlinien mit Soll-Längen zeichnen
    → [Scale Solver]   metrische Kalibrierung über Bildecken
```

**Image Overlay** und **Scale Solver** wandeln `Image::ImagePlane` bei Bedarf automatisch in `AlignedImage` um. Das Referenzbild (`AlignedImage`) bleibt während Overlay und Solver unverändert (Zustand wird vor der Operation gesichert und danach wiederhergestellt).

---

## FreeCAD-Objekte

### AlignedImage (`App::FeaturePython`)

Texturiertes Bild als perspektivisch korrektes Quad in der 3D-Ansicht. Die Geometrie ist ein UV-Einheitsquadrat `(0,0)…(1,1)`, das per `WarpMatrix` in die Welt transformiert wird.

| Property | Typ | Beschreibung |
|----------|-----|--------------|
| `ImageFile` | File | Pfad zur Bilddatei |
| `Corner0` | Vector | Weltposition der Ecke UV `(0, 0)` |
| `CornerX` | Vector | Weltposition der Ecke UV `(1, 0)` |
| `CornerY` | Vector | Weltposition der Ecke UV `(0, 1)` |
| `Corner1` | Vector | Weltposition der Ecke UV `(1, 1)` |
| `WarpMatrix` | Matrix | Projektive 4×4-Abbildung des UV-Quads in die Welt |
| `SourceImage` | Link | Ursprüngliches `Image::ImagePlane` (optional) |

**Darstellung (ViewProvider):** Coin-Szene mit `SoMatrixTransform` (`WarpMatrix`), `SoTexture2`, Einheitsquad `SoCoordinate3` + `SoFaceSet`. Die vier `Corner*`-Properties sind Cache für Picking und Werkzeuge; die sichtbare Geometrie folgt der Matrix.

**Erzeugung:** `create_aligned_image_from_plane()` kopiert die vier Ecken aus der `ImagePlane`-Placement-Geometrie, setzt `WarpMatrix` aus den Ecken (`_sync_warp_from_corners`) und versteckt optional die Quelle.

---

### FeaturePair (`App::FeaturePython`)

Ein übereinstimmendes Punktpaar zwischen Referenzbild und Bild 2.

| Property | Typ | Beschreibung |
|----------|-----|--------------|
| `RefPoint` | Vector | Punkt auf dem Referenzbild (Weltkoordinaten) |
| `MovPoint` | Vector | Entsprechender Punkt auf Bild 2 |
| `ShowMarkers` | Bool | Marker und Verbindungslinie anzeigen (default: `true`) |

`execute()` erzeugt keine Part-Geometrie — die Darstellung liegt vollständig im ViewProvider (Coin-Marker + Linie).

**Darstellung:** Blauer Marker (`RefPoint`), roter Marker (`MovPoint`), graue Verbindungslinie. Sichtbarkeit über `ShowMarkers` / ViewObject-Visibility.

---

### ReferenceLine (`App::FeaturePython`)

Eine Maßlinie mit bekannter Soll-Länge.

| Property | Typ | Beschreibung |
|----------|-----|--------------|
| `Image` | Link | Bildebene (`AlignedImage`), auf der Start/Ende liegen |
| `Start` | Vector | Linienanfang (Weltkoordinaten) |
| `End` | Vector | Linienende |
| `TargetLength` | Length | Soll-Länge in mm |
| `CurrentLength` | Length | Aktuelle 2D-Länge in der XY-Ebene (nur Anzeige, read-only) |

`execute()` erzeugt eine `Part::Edge` zwischen `Start` und `End` (sofern nicht degeneriert).

**Länge:** `reference_line_length_xy()` = euklidische Distanz nur in X/Y (Kalibrierung und Anzeige). `onChanged` für `Start`/`End` projiziert und snappt Punkte automatisch (`snap_reference_line_point`).

**Bearbeiten:** Doppelklick auf die Linie → `ReferenceLineEditor` (Endpunkte ziehen).

---

## Toolbar-Befehle

Alle Befehle liegen in der Toolbar **Image** (`init_gui.py`).

| Befehl | Icon | Funktion | Implementierung |
|--------|------|----------|-----------------|
| **Aligned Image** | `image_align.svg` | `ImagePlane` → `AlignedImage` | `convert_selected_to_aligned_images()` |
| **Feature Pair** | `feature_pair.svg` | Zwei Punkte klicken | `create_feature_pair()` |
| **Reference Line** | `reference_line.svg` | Maßlinie zeichnen | `create_reference_line()` → `ReferenceLineDialog` |
| **Image Overlay** | `image_overlay.svg` | Bilder überlagern | `overlay_images()` |
| **Scale Solver** | `scale_solver.svg` | Metrische Kalibrierung | `solve_reference_lines()` |

Bilder werden zunächst mit **Draft → Insert Image** (`Image::ImagePlane`) geladen.

---

### Aligned Image

**Nutzer:** Bildobjekt(e) auswählen → Befehl ausführen.

**Ablauf (`convert_selected_to_aligned_images`):**

1. Für jedes ausgewählte Bild (kein bereits vorhandenes `AlignedImage`):
2. `create_aligned_image_from_plane(plane)`:
   - Lokale Ecken `(-X/2,-Y/2)…` der ImagePlane mit `Placement` in Welt transformieren
   - Neues `AlignedImage` mit denselben Ecken und `ImageFile`
   - `WarpMatrix` aus Ecken berechnen
   - `SourceImage` verlinken, Original-ViewObject ausblenden
3. Dokument recomputen

Bereits umgewandelte Objekte werden übersprungen.

---

### Feature Pair

**Nutzer:** Ersten Punkt auf Referenzbild klicken, dann entsprechenden Punkt auf Bild 2.

**Ablauf (`create_feature_pair`):**

1. `_pick_point("Punkt auf Referenzbild wählen", …)` — Klick in 3D-Ansicht
2. `_pick_point("Entsprechenden Punkt auf Bild 2 wählen", …)`
3. `_create_feature_pair_object(ref, mov)` — neues `FeaturePair` mit beiden Punkten

**Klick-Handler (`_pick_point`):** Linksklick = Punkt setzen, Rechtsklick oder Escape = abbrechen. Kein Bild in der Auswahl → Rohpunkt aus `view.getPoint()`; mit Bild → Projektion auf Bildebene (siehe [Koordinaten](#koordinaten-und-bildabbildung)).

---

### Reference Line

**Nutzer:** Dialog öffnet sich mit Soll-Länge (mm). **Linie zeichnen** startet den interaktiven Modus.

**Ablauf (`ReferenceLineDialog` + `_pick_reference_line_endpoints`):**

1. Soll-Länge prüfen (`> 0`)
2. Bild aus aktueller Auswahl ermitteln (`_reference_image_from_selection`)
3. Zwei Klicks wie beim Feature Pair (`_pick_point`), mit Snap auf bestehende Endpunkte:
   - **1. Linksklick:** Linienanfang
   - **2. Linksklick:** Linienende → `ReferenceLine` wird erstellt
   - **Escape / Rechtsklick:** Abbrechen
4. Callback und Event-Cleanup laufen verzögert (`QTimer`), um Coin-Abstürze zu vermeiden

**Bearbeiten (`ReferenceLineEditor`):**

- Doppelklick auf Referenzlinie → Edit-Modus
- Start- (grün) oder End- (dunkelgrün) Handle im Pick-Radius anklicken und ziehen
- Snap wie beim Zeichnen; gelber Marker bei aktivem Snap
- Maus loslassen beendet den Drag; Escape beendet den Edit-Modus und committet die Transaction

---

### Image Overlay

**Nutzer:** Genau 2 Bilder + mindestens 3 Feature-Paare auswählen → Befehl ausführen.

**Ablauf (`overlay_images`):**

1. Validierung: 2 Bildobjekte, ≥ 3 `FeaturePair`
2. Referenz vs. Bild 2 per Punkt-Hit-Test (`_identify_ref_mov_images`): welches Bild enthält mehr `RefPoint`/`MovPoint`
3. Zustand des Referenz-`AlignedImage` sichern (`_snapshot_aligned_state`)
4. Bild 2 in `AlignedImage` umwandeln falls nötig (`ensure_aligned_image`)
5. Für jedes Feature-Paar:
   - UV von `MovPoint` auf Bild 2 (`_uv_on_image`)
   - Paar `((u,v), RefPoint)` für Homographie-Lösung
6. `H = compute_homography(uv_world_pairs)` — siehe [Homographie](#image-overlay-homographie)
7. `H` auf Bild 2 anwenden (`_set_aligned_homography`)
8. `MovPoint` aller Paare auf neue Weltposition setzen (gleiche UV, neue Homographie)
9. Referenzbild-Zustand wiederherstellen
10. RMS-Fehler der Homographie ausgeben

Das Referenzbild wird **nicht** transformiert.

---

### Scale Solver

**Nutzer:** Mindestens 1 Referenzlinie mit Soll-Länge + ein `AlignedImage` (typisch: zu kalibrierendes Bild) in der Auswahl. Feature-Paare und weitere Referenzlinien werden mittransformiert.

**Ablauf (`solve_reference_lines`):**

1. Referenzlinien aus Auswahl sammeln
2. Bilder identifizieren (wie Overlay, falls 2 Bilder + Paare vorhanden)
3. Referenz-`AlignedImage`-Zustand sichern
4. Ziel-`AlignedImage` sicherstellen (`ensure_aligned_image`)
5. Transformationsziele sammeln: kalibriertes Bild, Feature-Paare, Referenzlinien auf diesem Bild
6. `compute_calibration_corners(ref_lines, img_mov)` — Optimierung der vier Bildecken
7. UV-Snapshots der Ziele vor Transformation (`_snapshot_homography_targets`)
8. `apply_corner_calibration(img, corners_new, objects)` — Ecken setzen, Objekte per fixer UV neu platzieren
9. Referenzbild wiederherstellen
10. Debug-Tabelle Vor/Nach Transformation in der Konsole

Details: [Eckpunkt-Kalibrierung](#scale-solver-eckpunkt-kalibrierung).

---

## Interaktion und Punktauswahl

### Projektion auf die Bildebene

`project_point_to_image(point, img)`:

1. UV des Klickpunkts auf dem Bild-Quad (`_uv_on_image`)
2. Rückabbildung UV → Welt mit bilinearer Z-Interpolation (`_world_on_image_uv`)

Für `AlignedImage`: inverse Homographie aus `WarpMatrix`. Für `ImagePlane`: baryzentrische UV auf dem aus Placement abgeleiteten Quad.

### Snap (Referenzlinien)

Snap-Ziele (`_collect_snap_points`):

- `Start`/`End` aller Referenzlinien auf demselben `Image` (optional `exclude_line` beim Editieren)
- `RefPoint` und `MovPoint` aller Feature-Paare im Dokument

**Toleranz:** `max(5 mm, Pixel-Toleranz)` — Pixel-Toleranz = Weltabstand von 12 Pixeln am Cursor (`_REF_LINE_SNAP_PIXELS`). So bleibt Snap bei Zoom nutzbar.

**Exaktes Überlappen:** Innerhalb der Toleranz wird die exakte Koordinate des Snap-Ziels übernommen (identische Weltvektoren).

**Endpunkt-Welding (Solver):** Vor der Kalibrierung werden Endpunkte innerhalb von 5 mm zu gemeinsamen UV-Knoten verschmolzen (`_weld_reference_line_endpoint_uvs`) — wichtig für L-förmige Maßketten mit gemeinsamen Eckpunkten.

### Klassen für Interaktion

| Klasse | Zweck |
|--------|--------|
| `_ReferenceLineOverlay` | Coin-Overlay (Snap-Marker) |
| `_pick_reference_line_endpoints` | Zwei Klicks mit Snap (ohne Scenegraph-Overlay) |
| `ReferenceLineEditor` | Endpunkt-Bearbeitung per Drag |
| `_pick_point` | Einmaliger Punkt-Klick (Feature Pair) |

---

## Koordinaten und Bildabbildung

### UV-Koordinaten

Jedes Bild wird als Quad mit UV `(0,0)`, `(1,0)`, `(1,1)`, `(0,1)` modelliert. Textur und Picking nutzen dasselbe Einheitsquad.

**UV aus Weltpunkt (`_uv_on_image`):**

- `AlignedImage`: inverse 3×3-Homographie aus `WarpMatrix`
- `ImagePlane` / Fallback: baryzentrische Koordinaten auf zwei Dreieck-Hälften des Quads (`_uv_from_quad`), mit Fallback auf nächstes Dreieck

**Welt aus UV (`_world_on_image_uv`):**

```text
(x, y) = H @ [u, v, 1]   (Perspektiv-Division)
z      = bilinear(u, v) über die vier Ecken-Z-Werte
```

### WarpMatrix

`WarpMatrix` ist eine FreeCAD 4×4-Matrix, die das UV-Einheitsquad perspektivisch abbildet. Coin nutzt sie direkt als `SoMatrixTransform`. `_homography_to_warp_matrix` / `_homography_from_warp_matrix` konvertieren zwischen 3×3-Homographie und Matrix.

Ecken und Matrix werden synchron gehalten:

- Ecken geändert → `_sync_warp_from_corners`
- Homographie gesetzt → `_sync_corners_from_homography`

---

## Image Overlay (Homographie)

**Problem:** Bild 2 so ausrichten, dass seine Feature-Punkte (`MovPoint`) auf die Referenzpunkte (`RefPoint`) liegen — projektiv (8 Parameter, `h₃₃ = 1`).

**Eingabe:** Liste `((u, v), RefPoint)` — UV auf Bild 2, Zielweltpunkt auf Referenz.

**Lösung (`compute_homography`):** Lineares System `A·params = b` mit `numpy.linalg.lstsq`:

```text
x = (h₁₁·u + h₁₂·v + h₁₃) / (h₃₁·u + h₃₂·v + 1)
y = (h₂₁·u + h₂₂·v + h₂₃) / (h₃₁·u + h₃₂·v + 1)
```

| Feature-Paare | Verhalten |
|---------------|-----------|
| 3 | exakt, Minimum-Norm (unterbestimmt) |
| 4 | exakt, eindeutig (nicht degeneriert) |
| 5+ | Ausgleichsrechnung |

**Anwendung:** Homographie setzt die vier Ecken von Bild 2 (`Corner0`, `CornerX`, `Corner1`, `CornerY`) und `WarpMatrix`. Anschließend werden `MovPoint` der Feature-Paare auf die neuen Weltpositionen bei gleicher UV aktualisiert.

---

## Scale Solver (Eckpunkt-Kalibrierung)

**Problem:** Referenzlinien haben eine **Soll-Länge** in mm; die aktuelle 2D-Länge auf dem Bild stimmt noch nicht. Das Bild (und alle darauf liegenden Hilfsgeometrie) soll skaliert werden, ohne willkürliche Verzerrung.

**Ansatz:** Die vier XY-Positionen der Bildecken werden als Optimierungsvariablen bewegt (Z-Werte der Ecken bleiben fix). Daraus folgt eine Homographie `H`; Längen der Referenzlinien werden in UV-Raum berechnet (`_line_length_uv`).

**Mathematik und Solver-Details:** [CALIBRATION_SOLVER.md](CALIBRATION_SOLVER.md) (Formeln, Residuen, `uniform_scale` vs. `corners`).

**Geplant:** Vor der Homographie optionale **Objektiv-Entzerrung** aus einem
gespeicherten Kameramodell (Gitter-Kalibrierung, B-Spline δ(u,v)) —
siehe [CAMERA_MODEL.md](CAMERA_MODEL.md).

### Referenzlinien-Specs

Für jede Linie:

```text
(u₀, v₀) = UV von Start    (nach Welding gemeinsamer Endpunkte)
(u₁, v₁) = UV von End
target   = TargetLength
```

Welding: Endpunkte ≤ 5 mm auseinander → ein gemeinsamer UV-Knoten (Mittelwert).

### Anwendung der Lösung

1. UV-Snapshots aller zu transformierenden Objekte (`_snapshot_homography_targets`)
2. Neue Ecken setzen (`_restore_aligned_corners` + `_sync_warp_from_corners`)
3. Objekte per gespeicherter UV neu in Welt setzen (`apply_corner_calibration`):
   - `ReferenceLine`: `Start`/`End` via `_world_on_image_uv`
   - `FeaturePair`: nur `MovPoint` (Referenzpunkt liegt auf anderem Bild)

**Ergebnis:** Referenzlinien haben (idealerweise exakt) die Soll-Längen in der XY-Ebene; das Bild wurde metrisch kalibriert.

### Debug-Ausgabe

`_print_scale_solver_debug` gibt tabellarisch pro Linie aus:

- Vor Transformation: Ist-Länge, Soll, Modell-Länge (aus `H`), Delta
- Nach Transformation: Ist 2D-Länge, Soll, Delta

Zusätzlich: Verzerrungsenergie, starre Bewegung (Δ, θ), Eck-Verschiebung, Endpunkt-Welds.

---

## Auswahl-Regeln

### Image Overlay

| Anforderung | Wert |
|-------------|------|
| Bildobjekte | genau **2** (Referenz + Bild 2, automatische Erkennung) |
| Feature-Paare | mindestens **3** |

### Scale Solver

| Anforderung | Wert |
|-------------|------|
| Referenzlinien | mindestens **1** mit `TargetLength > 0` |
| AlignedImage | mindestens **1** in der Auswahl (kalibriertes Bild) |
| Optional | Feature-Paare und weitere Referenzlinien (werden mittransformiert) |

### Reference Line (Bild für Zeichnen)

- Bild aus aktueller Auswahl; bei mehreren Bildern wird ein `AlignedImage` bevorzugt
- Ohne Auswahl: keine Projektion/Snap auf Bildebene (Warnung in Konsole)

### Aligned Image

- Ein oder mehrere `Image::ImagePlane` oder bereits `AlignedImage` (übersprungen)

---

## Abhängigkeiten

| Paket | Verwendung |
|-------|------------|
| `numpy` | Homographie, Optimierung, Längenberechnung |
| `scipy` | `least_squares` für Scale Solver (Pflicht für Kalibrierung) |
| `pivy` (Coin3D) | Bilddarstellung, Marker, Interaktions-Overlay |
| `PySide` | Dialog `reference_line.ui` |

Keine Draft-Abhängigkeit für Punktauswahl — eigene Pivy-Event-Handler in `image_tools.py`.

---

## Konstanten (Referenz)

| Konstante | Wert | Bedeutung |
|-----------|------|-----------|
| `_REF_LINE_ENDPOINT_SNAP_MM` | 5,0 mm | Snap-/Weld-Toleranz für Endpunkte |
| `_REF_LINE_SNAP_PIXELS` | 12 px | Zusätzliche zoom-abhängige Snap-Toleranz |
| `_CALIB_LENGTH_TOLERANCE_MM` | 0,01 mm | „Exakt“-Schwelle für Längenfehler |
| `_CALIB_RIGID_TRANSLATION_TOLERANCE_MM` | 1,0 mm | Skalierung starre Translation im Residuum |
