# Fixpunkt-Bedingung — Umsetzungsplan

Stand: **implementiert** in `image_calibration_objects.py`, `image_tools.py`,
`image_constraint_solver.py`.

Erweiterung des Kalibrierungs-Workflows (`ImageCalibration` + Bedingungen-Dialog):
Sketch-Knoten **P0, P1, …** benennen und als **Fixpunkt** mit Soll-X/Y in mm
vorgeben. Damit lässt sich das Bild **global ausrichten**, ohne auf die weiche
Schwerpunkt-Strafe zu vertrauen.

Siehe auch [CALIBRATION_SOLVER.md](CALIBRATION_SOLVER.md) (Solver-Ist-Zustand),
[IMAGE_TOOLS.md](IMAGE_TOOLS.md) (UI/Workflow).

---

## 1. Ziel und Motivation

**Problem heute:** Längen- und Winkel-Bedingungen bestimmen Maßstab und Orientierung,
aber nicht zuverlässig die **globale Lage** des Bildes. Der Solver hält nur den
**Schwerpunkt der vier Bildecken** weich nahe der Startlage (`Δt/τ_t`) — kein
vom Nutzer gewählter Anker.

**Lösung:** Eine oder mehrere **Fixpunkt-Bedingungen**:

> Knoten *Pk* auf dem Bild soll nach der Kalibrierung bei `(x_soll, y_soll)` in
> Welt-XY liegen.

**Regel (Pflicht):** Sobald mindestens ein Fixpunkt gesetzt ist, entfällt die
Schwerpunkt-Nebenbedingung (`include_centroid=False`). Fixpunkt und Schwerpunkt
würden sonst dieselben Freiheitsgrade (Translation) unterschiedlich regeln.

---

## 2. Nutzer-Workflow (minimal)

1. Sketch mit Linien zeichnen (wie bisher).
2. Bedingungen-Dialog öffnen (Doppelklick `ImageCalibration`).
3. In der Szene erscheinen neben **L0, L1, …** auch **P0, P1, …** an den
   Sketch-Knoten (geschweißte Endpunkte).
4. Button **„Fixer Punkt“** → Zeile in der Tabelle:
   - **Punkt:** Combo `P0`, `P1`, …
   - **X mm**, **Y mm:** Soll-Position (Welt-XY)
5. Beim Anlegen: X/Y mit **aktueller Weltposition** des Knotens vorbelegen
   (typisch: Position halten). Nutzer kann Werte ändern für gezielte Ausrichtung
   (z. B. Sketch-Ursprung `(0, 0)`).
6. **Kalibrieren** — Solver erfüllt Längen/Winkel **und** Fixpunkte.

Kein Klick-Picking auf das Bild in v1 — Auswahl nur über benannte Punkte in der
Combo (einfacher, konsistent mit L0/L1).

---

## 3. Datenmodell

### 3.1 Punkte (parametrisch, analog zu `Lines`)

Neue Property am `ImageCalibration`-Objekt (Vorschlag):

| Property | Typ | Inhalt |
|----------|-----|--------|
| `Points` | `App::PropertyString` (JSON) | Liste der Sketch-Knoten in Bild-UV |

JSON-Schema (eine Zeile pro Knoten):

```json
[
  { "point": 0, "u": 0.12, "v": 0.34 },
  { "point": 1, "u": 0.56, "v": 0.34 }
]
```

- **`point`:** stabiler Index `0 … n-1` → Anzeige **P0, P1, …**
- **`u`, `v`:** Bild-UV nach Endpoint-Welding (siehe Abschnitt 4)
- Keine Soll-X/Y in `Points` — die gehören in die Constraints

Speicherung und Sync analog zu `Lines` / `_store_calibration_lines`:
Snapshot aus Sketch → bei Geometrie-Änderung neu berechnen, Indices möglichst
stabil halten.

### 3.2 Fixpunkt in `Constraints`

Erweiterung von `default_constraints()` in `image_calibration_objects.py`:

```json
{
  "lengths": [],
  "parallel": [],
  "perpendicular": [],
  "horizontal": [],
  "vertical": [],
  "fixed_points": []
}
```

Eintrag pro Fixpunkt:

```json
{ "point": 0, "target_x_mm": 100.0, "target_y_mm": 50.0 }
```

Legacy-JSON ohne `fixed_points` → leere Liste (abwärtskompatibel).

---

## 4. Punkte aus dem Sketch ableiten

### 4.1 Welding (bestehend)

Endpunkte nahe beieinander werden bereits zusammengeführt:
`_weld_sketch_line_uvs` in `image_tools.py` (Toleranz wie Linien:
`_REF_LINE_ENDPOINT_SNAP_MM` / `REF_LINE_ENDPOINT_SNAP_MM`).

Zwei Linienenden im selben Cluster → **ein** Punkt Pk, **eine** UV-Position.

### 4.2 Algorithmus (neu, klein)

Aus `lines_meta` nach Welding:

```text
1. Alle Endpunkte sammeln: (geo, "start"|"end") → (u, v, w_world)
2. Union-Find / Cluster wie _weld_sketch_line_uvs (bereits vorhanden)
3. Pro Cluster einen Eintrag in points_meta:
     point = 0..n-1
     u, v = Mittelwert im Cluster
     w = Mittelweltposition (für Anzeige / Default X,Y)
4. In ImageCalibration.Points persistieren (nur u, v + point)
```

Hilfsfunktionen (Vorschlag, `image_tools.py`):

- `_snapshot_sketch_points_uv(sketch, img)` → `points_meta`
- `_store_calibration_points(cal_obj, points_meta)`
- `_points_meta_for_calibration(cal_obj, sketch, img)` — analog `_lines_meta_for_calibration`
- `_point_by_index_from_points_meta(points_meta)` → `{0: {...}, 1: {...}}`

### 4.3 Szene-Beschriftung

`_CalibrationSketchLabelsOverlay` erweitern:

- Bisher: Label **Lk** an Linienmitte
- Neu: Label **Pk** an geschweißter Endpunkt-Position (Weltkoordinate aus
  `(u,v)` auf Bild oder aus Sketch-Endpunkt)

Kein separates FreeCAD-Objekt pro Punkt — nur Overlay + JSON (wie bei Linien).

---

## 5. Mathematik (Solver)

### 5.1 Residual

Für Fixpunkt mit gespeichertem `(u, v)` und Soll `(x_s, y_s)` in mm:

\[
r_x = \frac{x(u,v) - x_s}{\tau_p}, \qquad
r_y = \frac{y(u,v) - y_s}{\tau_p}
\]

mit `(x(u,v), y(u,v)) = H(u,v)` über `_apply_homography_uv` (`image_homography.py`).

**Konstante:** `\tau_p = 0.01` mm (wie `\tau_L` bei Längen) — neue Konstante
`_CALIB_POINT_TOLERANCE_MM`.

### 5.2 Einordnung im Residualvektor

| Term | Art | Wann |
|------|-----|------|
| Längen / Winkel | primär | immer (wenn gesetzt) |
| Fixpunkte | **primär** | wenn gesetzt (je 2 Gleichungen) |
| `E_angle` pro Ecke | Side-Term | rank < 6 |
| `Δt` Schwerpunkt | Side-Term | **nur wenn keine Fixpunkte** |

Fixpunkte in `_primary_calibration_residuals` und `_calibration_residuals`
**vor** den Side-Terms anhängen.

### 5.3 Schwerpunkt ausschalten

```python
def _has_fixed_point_constraints(constraints, point_by_index=None):
    items = (constraints or {}).get("fixed_points") or []
    if not point_by_index:
        return bool(items)
    return any(
        int(it["point"]) in point_by_index
        for it in items
        if "point" in it
    )

include_centroid = not _has_fixed_point_constraints(constraints, point_by_index)
```

Aufruf in `_solve_corner_calibration` bei `residual_fn` und in Meta:
`include_translation_side = (mode == "corners" and include_centroid)`.

### 5.4 DOF (kurz)

- 1 Fixpunkt ≈ 2 harte Gleichungen → Translation in XY fixiert (wenn unabhängig).
- Schwerpunkt-Strafe war bisher der weiche Ersatz (−2 eff. DOF). **Fixpunkt
  ersetzt sie** — nicht kombinieren.
- Rang-Bestimmung: Fixpunkt-Restfehler in `_primary_calibration_residuals`
  zählen mit (`n_primary` steigt um 2 pro gültigem Fixpunkt).

---

## 6. UI (Bedingungen-Dialog)

Datei: `ImageCalibrationConstraintsDialog` in `image_tools.py`.

### 6.1 Tabelle

Option **A (empfohlen, einfach):** Tabelle auf **4 Spalten** erweitern:

| Typ | Punkt / Kante | Wert A | Wert B |
|-----|---------------|--------|--------|
| Soll-Länge | L0 | 120.0 mm | — |
| Fixer Punkt | P0 | X: 0.0 mm | Y: 0.0 mm |
| Parallel | L0 | L1 | — |

- Spaltenüberschrift generisch: `["Typ", "Bezug", "Wert A", "Wert B"]`
- Längen: Spalte 3 leer oder „—“
- Fixpunkt: Spalte 2 = X-SpinBox, Spalte 3 = Y-SpinBox
- Parallel/Rechtwinklig: Spalte 2/3 = Kanten-Combos

### 6.2 Button

```python
("Fixer Punkt", self._add_fixed_point_row),
```

### 6.3 Handler

- `_point_combo(point_idx=None)` — analog `_geo_combo`, Labels `P0`, `P1`, …
- `_add_fixed_point_row(point_idx=None, x_mm=None, y_mm=None)`
- Default X/Y: aktuelle Weltposition `H(u,v)` oder gespeichertes `w` aus
  `points_meta` beim Öffnen der Zeile
- `_load_constraints` / `_collect_constraints`: `"fixed_points"` lesen/schreiben
- `_collect_constraints`: `"Fixer Punkt"` → `fixed_points.append({...})`

### 6.4 Konsole / Hinweistext

Im Dialog-Hint ergänzen:

> Fixer Punkt: Sketch-Knot Pk soll an Soll-X/Y (Welt-mm) liegen. Bei gesetzten
> Fixpunkten entfällt die Schwerpunkt-Nebenbedingung.

In `_print_calibration_constraint_report` (Solver):

```text
Fixpunkte:
  P0: Ist (x, y), Soll (xs, ys), Δ (+dx, +dy) mm
```

---

## 7. Dateien und Änderungen (Checkliste)

Reihenfolge für die Implementierung:

| # | Datei | Änderung |
|---|--------|----------|
| 1 | `image_calibration_objects.py` | `default_constraints`: `"fixed_points": []`; Property `Points`; parse/dump |
| 2 | `image_tools.py` | `_snapshot_sketch_points_uv`, `_store_calibration_points`, `_points_meta_for_calibration`; Overlay P-Labels; Dialog (Combo, Button, 4 Spalten) |
| 3 | `image_constraint_solver.py` | `_point_residuals_for_constraints`, `_has_fixed_point_constraints`, `_constraint_point_index`; Primary + full residuals; `include_centroid`; Report; `_normalize_constraints` / Remap für `point` |
| 4 | `tests/test_calibration_solver.py` | Tests: 1 Fixpunkt hält Position; Schwerpunkt aus wenn Fixpunkt; Residual bei Soll=Ist ≈ 0 |
| 5 | `docs/CALIBRATION_SOLVER.md` | Kurzer Verweis auf Fixpunkte (optional, nach Merge) |

**Nicht in v1:**

- Mehrere unabhängige Fixpunkt-Modi (Rotation explizit fixieren braucht oft 2 Punkte — kein Extra-UI nötig, Nutzer setzt 2 Zeilen)
- FreeCAD-Geometrie-Objekte pro Punkt
- Klick-Picking auf Bild
- Fixpunkt in Phase 1/2 (`uniform_scale` / `uv_scale`) — nur Phase 3 `corners`

---

## 8. Tests (Minimal)

In `tests/test_calibration_solver.py` (Headless, ohne GUI):

1. **`test_fixed_point_holds_target`**  
   Quad + eine Länge + Fixpunkt P0 mit Soll = Ist-Position an Start → nach Solve
   Abweichung `< τ_p`.

2. **`test_centroid_skipped_with_fixed_point`**  
   Mock/Meta: bei `fixed_points` non-empty → `_calibration_residuals(..., include_centroid=False)` bzw. letztes Residual-Element fehlt.

3. **`test_fixed_point_moves_quad`**  
   Soll-X/Y absichtlich versetzt → nach Solve liegt `H(u,v)` nahe Soll (Translation sichtbar).

4. **`test_welded_endpoints_one_point`**  
   Zwei Linien teilen Endpunkt → eine P-Liste mit n-1 Knoten; Constraint auf P0
   betrifft beide Linienenden.

Optional: Fixture-Eintrag in `tests/fixtures/align_image_test_1.json` mit
`"fixed_points": []` zur Dokumentation.

---

## 9. Randfälle

| Fall | Verhalten |
|------|-----------|
| Fixpunkt-Index ungültig (Sketch geändert) | Eintrag in `_collect_constraints` überspringen; Warnung in Konsole |
| Keine Soll-Längen | wie bisher: Solve abgebrochen („Mindestens eine Soll-Länge …“) |
| Fixpunkt + 0 Längen | nicht erlaubt (Längen weiter Pflicht) |
| Zwei Fixpunkte widersprüchlich | Solver: least_squares Kompromiss; Rang-Report zeigt Überbestimmtheit |
| `Points` leer (kein Sketch) | Dialog ohne P-Combo; Fixpunkt-Button deaktiviert oder Hinweis |

---

## 10. Akzeptanzkriterien

- [x] P0, P1, … in Sketch-Overlay sichtbar (wenn Bedingungen-Dialog offen)
- [x] Fixpunkt-Zeile: Punkt wählen, X/Y setzen, speichern (OK / Kalibrieren)
- [x] Nach Solve: Konsole zeigt Ist/Soll/Δ pro Fixpunkt
- [x] Mit ≥1 Fixpunkt: Diagnose „Schwerpunkt-Nebenbedingung: nein“
- [x] Bild lässt sich global verschieben, indem Soll-X/Y vom Ist abweichen
- [x] Bestehende Projekte ohne `fixed_points` / `Points` laden unverändert

---

## 11. Konstanten (Referenz)

| Symbol | Wert | Bedeutung |
|--------|------|-----------|
| `\tau_p` | 0,01 mm | Skalierung Fixpunkt-Residual (neu) |
| `\tau_L` | 0,01 mm | Längen (bestehend) |
| `\tau_t` | 1,0 mm | Schwerpunkt (nur ohne Fixpunkt) |
| Snap Endpunkte | 5,0 mm | Welding Linien/Punkte (bestehend) |
