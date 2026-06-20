# FrameTools Image Pipeline — Spezifikation

Stand: Juni 2026. Konsolidierte Dokumentation aller Erkenntnisse aus der
bisherigen Implementierung (`image_objects.py`, `image_homography.py`,
`image_constraint_solver.py`, `image_tools.py`, `image_calibration_objects.py`).

**Ziel dieses Dokuments:** Einheitliche Grundlage für eine **Neuimplementierung**
des Bild-Workflows — Workflow, Mathematik, Architektur und bewusste
Vereinfachungen in einem File.

**Aktueller Code:** funktionsfähig, aber historisch gewachsen (mehrere Solver-Pfade,
1412 Zeilen Constraint-Solver). Die Spezifikation beschreibt das **Soll-Verhalten**
und markiert, was man vereinfachen kann.

---

## Inhaltsverzeichnis

1. [Problem und Ziel](#1-problem-und-ziel)
2. [Architektur: zwei Ebenen](#2-architektur-zwei-ebenen)
3. [Koordinaten und Abbildung](#3-koordinaten-und-abbildung)
4. [FreeCAD-Objekte und Workflow](#4-freecad-objekte-und-workflow)
5. [Homographie (Overlay)](#5-homographie-overlay)
6. [Kalibrierungs-Solver](#6-kalibrierungs-solver)
7. [Winkelerhaltung \(E_{\mathrm{angle}}\)](#7-winkelerhaltung-e_mathrmangle)
8. [Freiheitsgrade, Rang, Bestimmtheit](#8-freiheitsgrade-rang-bestimmtheit)
9. [Kameramodell (geplant)](#9-kameramodell-geplant)
10. [Coin3D-Darstellung](#10-coin3d-darstellung)
11. [Erkenntnisse aus der Implementierung](#11-erkenntnisse-aus-der-implementierung)
12. [Vereinfachungen für Neuimplementierung](#12-vereinfachungen-für-neuimplementierung)
13. [Konstanten und Parameter](#13-konstanten-und-parameter)
14. [Tests und Validierung](#14-tests-und-validierung)
15. [Implementierungs-Roadmap](#15-implementierungs-roadmap)

---

## 1. Problem und Ziel

**Anwendungsfall:** Fotos von Bauplänen, Skizzen oder Werkstücken in FreeCAD
überlagern und **metrisch kalibrieren** (mm in der Werkstatt-XY-Ebene).

**Kernidee:** Jedes Foto wird als **UV-Quad** \([0,1]^2\) modelliert. Eine
**projektive Homographie** \(H\) bildet Bildkoordinaten in Welt-mm ab. Vier
Bildecken (oder `WarpMatrix`) parametrisieren \(H\).

**Kalibrierung:** Referenzlinien mit bekannter Soll-Länge und optional
Winkel-Bedingungen (horizontal, senkrecht, parallel, rechtwinklig) bestimmen
die Bildecken so, dass modellierte Längen und Winkel stimmen — ohne willkürliche
Scher-Verzerrung des Quads.

**Geplante Erweiterung:** Objektivverzerrung **einmal pro Kamera** aus einem
Display-Kalibriermuster; pro Foto bleiben Schieflage und Maßstab wie heute.

---

## 2. Architektur: zwei Ebenen

| Ebene | Inhalt | Gültigkeit | Parameter |
|-------|--------|------------|-----------|
| **Kameramodell** | Linsenentzerrung δ(u,v) | pro Kamera / Zoom | B-Spline 3×3 oder 4×4 |
| **AlignedImage** | Platzierung in der Werkstatt | pro Foto | Homographie \(H\), 4 Ecken |

**Pipeline (Zielzustand):**

```text
Rohpixel (u, v)
    → undistort(u, v)     [CameraModel, optional, fest]
    → H(u', v')           [4 Ecken / Längen-Solver, pro Bild]
    → Welt (X, Y, Z) mm
```

**Wichtig:** δ und \(H\) **sequentiell** schätzen, nicht gemeinsam — sonst tauschen
Spline und Homographie ihre Rollen (Perspektive „steckt“ in δ).

---

## 3. Koordinaten und Abbildung

### 3.1 UV auf dem Bild

| Ecke | UV | Property |
|------|-----|----------|
| Corner0 | (0, 0) | `Corner0` |
| CornerX | (1, 0) | `CornerX` |
| Corner1 | (1, 1) | `Corner1` |
| CornerY | (0, 1) | `CornerY` |

Textur, Picking und Solver nutzen dasselbe Einheitsquad.

### 3.2 Homographie

\[
\begin{pmatrix} x' \\ y' \\ w \end{pmatrix}
= H \begin{pmatrix} u \\ v \\ 1 \end{pmatrix},
\qquad
x = \frac{x'}{w},\; y = \frac{y'}{w},
\qquad H_{33} = 1.
\]

\(H\) ist aus vier Ecken **eindeutig** lösbar (8×8 lineares System).

**Z-Koordinate:** bilinear über die vier Eck-Z-Werte; während der
Kalibrierungsoptimierung **fix** (nur XY der Ecken variieren).

### 3.3 Welt ↔ UV

- **UV aus Weltpunkt:** inverse Homographie (AlignedImage) oder baryzentrisches
  Quad (ImagePlane).
- **Welt aus UV:** Homographie + bilineare Z-Interpolation.

### 3.4 WarpMatrix

FreeCAD 4×4-Matrix für Coin (`SoMatrixTransform`); Konvertierung zu/von 3×3-\(H\).
Ecken und Matrix synchron halten.

---

## 4. FreeCAD-Objekte und Workflow

### 4.1 Objekte

| Objekt | Rolle |
|--------|-------|
| `Image::ImagePlane` | Draft-Bild, Einstieg |
| `AlignedImage` | UV-Quad + Textur + `WarpMatrix` |
| `FeaturePair` | `RefPoint` + `MovPoint` für Overlay |
| `ReferenceLine` | Maßlinie: `Start`, `End`, `TargetLength`, `Image` |
| `ImageCalibration` | Sketch + JSON-Constraints + Solve |
| `CameraModel` *(geplant)* | δ(u,v), einmal pro Kamera |

### 4.2 Workflow Overlay (relativ)

```text
Draft → Insert Image
    → Feature Pair (≥ 3 Paare)
    → Image Overlay          # Bild 2 projektiv auf Referenz
```

Referenzbild bleibt unverändert; nur Bild 2 wird transformiert.

### 4.3 Workflow Kalibrierung (metrisch)

```text
Aligned Image (optional)
    → Image Calibration      # Objekt + Sketch
    → Referenzlinien / Sketch-Kanten
    → Calibration Constraints  # Soll-Längen, Winkel
    → Calibration Solve      # Bildecken optimieren, neuer Sketch
```

Alternativ (Legacy): `Reference Line` + `Scale Solver` ohne Sketch-Objekt.

**Nach dem Solve:** Sketch-Placement **nicht** optimieren; Geometrie aus
kalibrierten UV neu aufsetzen (`AlignedSketch`). Hilfsgeometrie (Referenzlinien,
Feature-Paare auf dem kalibrierten Bild) per **gespeicherter UV** mitbewegen.

### 4.4 Endpunkt-Welding

Referenzlinien-Endpunkte ≤ 5 mm auseinander → gemeinsamer UV-Knoten (Mittelwert).
Wichtig für L-förmige Maßketten.

### 4.5 Geplante Toolbar-Ergänzung

| Befehl | Funktion |
|--------|----------|
| Kameramodell kalibrieren | Display-Muster 16×8 → Foto → `CameraModel` |
| Bild mit Entzerrung laden | Foto + Model → rechteckiges `AlignedImage`, schwarze Ränder |

---

## 5. Homographie (Overlay)

**Problem:** Feature-Punkte auf Bild 2 sollen auf Referenzpunkte in der Welt liegen.

**Eingabe:** Liste `((u, v), Weltpunkt)` — UV auf Bild 2, Ziel auf Referenz.

**Lösung:** Lineares Ausgleichssystem `A·h = b` (`numpy.linalg.lstsq`).

| Feature-Paare | Verhalten |
|---------------|-----------|
| 3 | exakt, Minimum-Norm |
| 4 | exakt, eindeutig |
| 5+ | Ausgleich |

**Anwendung:** Ecken + `WarpMatrix` von Bild 2 setzen; `MovPoint` bei gleicher UV
neu projizieren.

---

## 6. Kalibrierungs-Solver

### 6.1 Problemstellung

**Gegeben:** Quad in Welt-XY, Referenzlinien mit UV-Endpunkten und Soll-Längen,
optional Winkel-Constraints aus Sketch.

**Gesucht:** Neue Bildeck-XY \(\mathbf{p} \in \mathbb{R}^8\), sodass

1. modellierte Längen = Soll-Werten,
2. Winkel-Bedingungen erfüllt,
3. bei Mehrdeutigkeit: Quad-Winkel erhalten, Schwerpunkt nahe Ausgangslage.

### 6.2 Längenbedingung

Modellierte Länge unter \(H\):

\[
\hat{L}_i = \left\|
\pi(H [u_0, v_0, 1]^\top) - \pi(H [u_1, v_1, 1]^\top)
\right\|_2.
\]

Restfehler: \(r_i^{\mathrm{len}} = \hat{L}_i - L_i^{\mathrm{soll}}\).

### 6.3 Winkel-Bedingungen

Einheits-Richtung der projizierten Linie in XY:

\[
\mathbf{d} = \frac{\pi(H [u_1,v_1,1]^\top) - \pi(H [u_0,v_0,1]^\top)}{\|\cdots\|}.
\]

| Bedingung | Restfehler |
|-----------|------------|
| Parallel (a, b) | \(\sin\angle(\mathbf{d}_a, \mathbf{d}_b)\) |
| Rechtwinklig | \(\mathbf{d}_a \cdot \mathbf{d}_b\) |
| Horizontal | \(\sin\angle(\mathbf{d}, \mathbf{r}_h)\) — Sketch-+X |
| Senkrecht | \(\sin\angle(\mathbf{d}, \mathbf{r}_v)\) — Sketch-+Y |

Sketch-Placement bleibt fix; Bedingungen beziehen sich auf Welt-XY relativ zum Sketch.

### 6.4 Zielfunktion (Modus Eckoptimierung)

\[
\min_{\mathbf{p}} \;\| \mathbf{r}(\mathbf{p}) \|_2^2
\]

mit `scipy.optimize.least_squares` (Trust-Region-Reflective).

**Residualvektor** (vereinfachte Notation):

\[
\mathbf{r} =
\begin{bmatrix}
\mathbf{r}^{\mathrm{len}} / \tau_L \\
w \cdot \mathbf{r}^{\mathrm{ang}} / \tau_a \\
\mathbf{1}_{\mathrm{rank} < 6}\, w \sqrt{E_{\mathrm{angle},k}} \;\text{(4 Ecken)} \\
\Delta t / \tau_t
\end{bmatrix}.
\]

Details zu Rang und \(E_{\mathrm{angle}}\): Abschnitte 7 und 8.

### 6.5 Warm-Start-Kaskade (aktueller Code)

Der Solver führt **immer** drei Phasen aus; Early-Exit nur ohne Winkel-Bedingungen:

```text
Phase 1: uniform_scale (1D, analytisch bei 1 Länge)
    → alle Ecken um Schwerpunkt gleich skalieren
Phase 2: uv_scale (2D, sx/sy)
    → unabhängige Skalierung entlang U/V-Kanten, Eckwinkel unverändert
Phase 3: least_squares auf 8 Eck-Koordinaten
    → voller Residualvektor
```

**Phase 1 — uniform_scale** bei einer Länge analytisch:

\[
s = \frac{L^{\mathrm{soll}}}{\hat{L}(H(\mathbf{p}_0))}, \qquad
\mathbf{c}_k' = \mathbf{g}_0 + s(\mathbf{c}_k - \mathbf{g}_0).
\]

**Phase 2 — uv_scale** mit Pivot \((u,v)=(0.5, 0.5)\):

\[
\mathbf{p}'(u,v) = \mathbf{g}_0 + s_x (u - 0.5)\,\mathbf{e}_u + s_y (v - 0.5)\,\mathbf{e}_v.
\]

Beide Phasen halten \(E_{\mathrm{angle}} = 0\). Phase 3 verfeinert und löst Fälle
mit Winkel-Constraints oder inkompatiblen Längen.

### 6.6 Typische Fälle

| Konfiguration | Erwartetes Verhalten |
|---------------|---------------------|
| 1 Soll-Länge, keine Winkel | Phase 1 reicht; \(E_{\mathrm{angle}} = 0\) |
| 2 Soll-Längen, keine Winkel | Phase 2 oft exakt; sonst Phase 3 |
| 1 Länge + Winkel | Phase 3; \(E_{\mathrm{angle}}\) bei Rang < 6 |
| Viele Längen + Winkel | überbestimmt; nur Längen/Winkel im Residual |

**Fallstrick:** Kaltstart in Phase 3 (8 DOF direkt) kann Scher-Minima finden —
Warm-Start über Phase 1/2 vermeidet das (`tests/plot_align_image_test_1_residuals.py`).

---

## 7. Winkelerhaltung \(E_{\mathrm{angle}}\)

### 7.1 Motivation

8 freie Eckkoordinaten erlauben **Scherung** — Winkel am Quad ändern sich, obwohl
Längen (und manchmal Winkel-Constraints) erfüllt sein können. Unter
**unterbestimmten** Systemen braucht man einen Tie-Breaker, der „natürliche“
Bildverformung bevorzugt.

### 7.2 Formulierung (finale Version)

An **jeder der vier Ecken** \(k\): Innenwinkel zwischen den beiden inzidenten
Kantenvektoren (nur XY) soll unverändert bleiben.

Am Ausgang \(\mathbf{p}_0\) und nach Optimierung \(\mathbf{p}\):

\[
\cos\alpha_k = \frac{\mathbf{e}_{k,1} \cdot \mathbf{e}_{k,2}}{\|\mathbf{e}_{k,1}\|\,\|\mathbf{e}_{k,2}\|},
\qquad
\sin\alpha_k = \frac{(\mathbf{e}_{k,1})_x (\mathbf{e}_{k,2})_y - (\mathbf{e}_{k,1})_y (\mathbf{e}_{k,2})_x}{\|\mathbf{e}_{k,1}\|\,\|\mathbf{e}_{k,2}\|}.
\]

Energie pro Ecke:

\[
E_{\mathrm{angle},k} = (\cos\beta_k - \cos\alpha_k)^2 + (\sin\beta_k - \sin\alpha_k)^2.
\]

Im Residual: \(\sqrt{E_{\mathrm{angle},k}}\) (gewichtet mit \(w\)).

### 7.3 Interpretation

| Verformung | \(E_{\mathrm{angle}}\) |
|------------|------------------------|
| Gleichmäßige Skalierung | 0 |
| Unterschiedliche Skalierung U/V (ohne Scherung) | 0 |
| Gemeinsame Rotation | 0 |
| Scherung / Perspektiv-Knick an mindestens einer Ecke | > 0 |

### 7.4 Abgelöste Formulierung

Früher: Energie auf **Kanten-Basis** (\(\mathbf{e}_u, \mathbf{e}_v\)) — misst
Anisotropie der Skalierung, nicht direkt Winkel. Die **Winkelerhaltung pro Ecke**
ist verständlicher, deckt dasselbe Ziel ab und korrespondiert mit der UV-Skalierung
in Phase 2.

---

## 8. Freiheitsgrade, Rang, Bestimmtheit

### 8.1 Effektive DOF

| | Anzahl |
|---|--------|
| Eckparameter (XY) | 8 |
| Schwerpunkt-Strafe \(\Delta t\) | −2 (Translation XY fixiert) |
| **Effektive DOF** | **6** |

\(\Delta t = \|\mathbf{g}_1 - \mathbf{g}_0\|\) mit Schwerpunkten der vier Ecken
vor/nach. Kabsch-Rotation nur zur Diagnose, nicht im Residual.

### 8.2 Rang-Entscheidung

Am Startpunkt \(\mathbf{p}_0\): Jacobian \(J = \partial \mathbf{r}^{\mathrm{prim}} / \partial \mathbf{p}\)
(nur Längen + Winkel), Rang per SVD.

| Rang vs. 6 | Bedeutung | \(E_{\mathrm{angle}}\) in Optimierung | \(\Delta t\) |
|------------|-----------|--------------------------------------|--------------|
| rank < 6 | unterbestimmt | **ja** | **ja** (immer in Phase 3) |
| rank ≥ 6 | bestimmt/überbestimmt | **nein** | **ja** |

**Bestimmtheit:**

- `unterbestimmt`: rank < 6
- `bestimmt`: rank = 6 und n_primary = 6
- `überbestimmt`: n_primary > 6

### 8.3 Obsolete Constraints (Erkenntnis)

Lokal linearisiert: Bedingung \(i\) ist redundant, wenn Zeile \(i\) von anderen
Zeilen in \(J\) abhängig ist (SVD-Rang pro Zeilen-Gruppe). Praktisch: zwei
parallele Längen auf derselben Kante, oder horizontal + senkrecht auf derselben
Linie. **Für v1:** Rang reicht; explizite Obsoleszenz-Erkennung optional.

---

## 9. Kameramodell (geplant)

### 9.1 Motivation

Homographie modelliert **Perspektive auf eine Ebene**, nicht:

- Objektivverzerrung (Radial, Weitwinkel),
- leichte Nicht-Planarität,
- lokale Abweichungen.

### 9.2 Kalibrierung am Display (16×8)

Statt gedrucktem Schachbrett: **128 helle Punkte** auf schwarzem Vollbild.

| Vorteil | |
|---------|---|
| Exakte Soll-Pixelkoordinaten | aus Muster-Metadaten |
| Kein Druckmaßstab | |
| 128 Punkte | ausreichend für B-Spline 4×4 + Regularisierung |

**Ablauf:**

```text
[Kameramodell kalibrieren]
    → Vollbild 16×8 Punkte
    → Nutzer fotografiert Bildschirm (gleiche Kamera/Zoom wie Arbeitsfotos)
    → Blob-Detektion + Raster-Sort (128 Punkte)
    → Schritt 1: H_grid (Perspektive Foto ↔ ideale Display-Ebene)
    → Schritt 2: B-Spline δ(u,v) auf Restfehler in Foto-UV
    → CameraModel speichern
```

**Punkt-Erkennung:** Graustufen → Threshold → Konturen → Schwerpunkt → Zeilen/Spalten
sortieren → Subpixel-Verfeinerung. OpenCV optional, nicht zwingend.

### 9.3 Datenmodell `CameraModel`

```text
CameraModel
  ImageSize              (Breite, Höhe px — Referenz für normiertes UV)
  PatternType            "display_dots_16x8"
  PatternMetadata        JSON: Display-Auflösung, Punkt-Soll-UV
  SplineControlPoints    δ(u,v) — B-Spline 3×3 oder 4×4
  CalibrationRMS         px
  CameraNote             Brennweite, Gerät (manuell)
```

**Nicht speichern:** \(H_{\mathrm{grid}}\) der Kalibrieraufnahme.

### 9.4 Anwendung auf Arbeitsfoto

```text
[Bild mit Entzerrung laden]
    → Foto + CameraModel
    → AlignedImage: rechteckiges Anzeige-Quad [0,1]²
    → pro Anzeige-Knoten (u_d, v_d):
         inverse_undistort → (u_r, v_r) → Textur-Sampling
         außerhalb Domain → schwarz
    → danach Ecken / ImageCalibration / Längen-Solver auf entzerrten UV
```

**Warum inverse Richtung:** δ: Roh → entzerrt. Coin fragt pro Anzeige-Pixel,
welche Texturkoordinate gelesen werden soll.

### 9.5 Stolpersteine

| Risiko | Maßnahme |
|--------|----------|
| Zoom/Brennweite geändert | neues CameraModel |
| EXIF-Rotation / Crop | ImageSize mitführen |
| H_grid + δ gemeinsam pro Foto | vermeiden |
| Gewelltes Gitter / Monitor | planare Platte bevorzugen |
| Zu viele Spline-DOF | 3×3 zuerst, Glattheitsstrafe |

---

## 10. Coin3D-Darstellung

### 10.1 Heute

- Ein Quad (4 Vertices), `SoTexture2`, `SoTextureCoordinate2` linear,
- `SoMatrixTransform` mit `WarpMatrix`.

### 10.2 Mit Kameramodell

**Empfohlen (v1):** tesselliertes Mesh 32×32 oder 64×64 über Anzeige-[0,1]²:

- Textur-UV pro Knoten via `inverse_undistort`,
- Weltposition aus Ecken-Interpolation / H,
- schwarze Dreiecke außerhalb gültiger Domain.

**Nicht v1:** Bitmap vorverzerren (speicherintensiv, unscharf beim Zoomen).

---

## 11. Erkenntnisse aus der Implementierung

### 11.1 Solver und Tests

1. **Tests prüfen oft den einfachen Pfad** (1 Länge → uniform_scale). Produktionsfälle
   mit 1 Länge + Winkel oder kaltem 8-DOF-Start zeigen Scherung — deshalb Warm-Start
   und \(E_{\mathrm{angle}}\)-Gating wichtig.

2. **Schwerpunkt-Strafe vs. uniform_scale:** Früher skalierte uniform_scale um c0 (UV-Ursprung),
   während \(\Delta t\) den Schwerpunkt bestraft — widersprüchlich. Lösung: Skalierung
   um **Schwerpunkt** \(\mathbf{g}_0\).

3. **Zwei Solver-Pfade** (`uniform_scale` vs. `corners`) erzeugen inkonsistente
   Meta-Daten und Verzweigungslogik. Mathematisch sind Phase 1/2 Teilmenge von
   „Eckwinkel-erhaltende Verformungen“.

4. **\(E_{\mathrm{angle}}\) nur bei rank < 6:** Ohne Gating dominiert bei
   überbestimmten Systemen ein unnötiger Tie-Breaker; mit Gating lösen primäre
   Constraints die 6 effektiven DOF allein.

5. **Winkel-Constraints sind stark gewichtet** (\(w = 25\), \(\tau_a = \sin 1°\)) —
   horizontal/rechtwinklig „gewinnt“ gegen kleine Längenabweichungen bei Konflikt.

6. **Sketch-Placement nicht optimieren** ist bewusst: Nutzer platziert Sketch;
   Bild passt sich an. Horizontal/Senkrecht beziehen sich auf Sketch-Achsen in Welt-XY.

### 11.2 Architektur

7. **UV-Snapshot bei Transformation:** Referenzlinien und Feature-Paare speichern UV
   vor Solve, projizieren nach Solve mit neuem \(H\) — konsistente Mitbewegung.

8. **Referenzbild schützen:** Overlay und Solver sichern/wiederherstellen den Zustand
   des Referenz-`AlignedImage`.

9. **Zwei Ebenen (Kamera vs. Bild)** reduziert Solver-Komplexität langfristig: δ frisst
   Objektivfehler; \(H\) + Längen nur noch Perspektive und Maßstab.

### 11.3 UI / Interaktion

10. **Snap:** max(5 mm, 12 px am Cursor) — zoom-unabhängig nutzbar.

11. **Coin-Abstürze:** Event-Cleanup per `QTimer` nach Referenzlinien-Picking.

12. **Display-Kalibrierung:** Moiré, PWM, Spiegelung — robustes Fit, große Punkte,
    längere Belichtung.

---

## 12. Vereinfachungen für Neuimplementierung

### 12.1 Ein Modul, eine Parametrisierung

**Statt** drei Solver-Modi (`uniform_scale`, `uv_scale`, `corners`) mit
`_can_use_*`-Weichen:

**Vorschlag:** Ein Optimierer mit **niedrigdimensionaler Parametrisierung** + Warm-Start:

| Parameter | Bedeutung | DOF |
|-----------|-----------|-----|
| \(s_x, s_y\) | Skalierung entlang U/V | 2 |
| \(\theta\) | Rotation um Schwerpunkt | 1 |
| \(t_x, t_y\) | Translation Schwerpunkt | 2 |
| \(h_{31}, h_{32}\) | Perspektive (optional) | 0–2 |

Phase 1/2 der aktuellen Kaskade werden zu **analytischen Initialwerten**
(\(s_x = s_y = s\) bei 1 Länge; \(s_x, s_y\) aus 2 Längen), danach ein
`least_squares`-Aufruf — entweder in 5–7 Parametern oder als Feinoptimierung
der 8 Ecken mit Warm-Start.

**Gewinn:** Keine duplizierte Residual-Logik, ein Report-Pfad, weniger Meta-Modi.

### 12.2 Side-Terms vereinheitlichen

| Term | Wann | Neuimplementierung |
|------|------|-------------------|
| Längen | immer | primär |
| Winkel-Constraints | wenn gesetzt | primär |
| \(\sqrt{E_{\mathrm{angle},k}}\) | rank < 6 | Side-Term (behalten) |
| \(\Delta t\) | rank < 6 | Side-Term (optional: auch an rank koppeln) |

Alternativ: \(\Delta t\) durch **explizite Nebenbedingung** \(\mathbf{g}_1 = \mathbf{g}_0\)
ersetzen (2 Gleichungen) — reduziert effektive DOF ohne weichen Term.

### 12.3 Kameramodell zuerst integrieren

Neuimplementierung sollte **undistort()** von Anfang an in der UV-Pipeline haben:

```python
def world_from_image_uv(u, v, corners, camera_model=None):
    if camera_model:
        u, v = undistort_uv(u, v, camera_model)
    return apply_homography(u, v, corners)
```

Dann muss \(H\) weniger Objektivfehler „schlucken“ → weniger Scher-Kompensation,
einfachere Kalibrierung.

### 12.4 Dateistruktur (Vorschlag)

| Modul | Verantwortung |
|-------|---------------|
| `image_coords.py` | UV, H, WarpMatrix, undistort |
| `image_solver.py` | Constraints, Residual, Optimierung |
| `image_objects.py` | FreeCAD-Objekte, ViewProvider |
| `image_commands.py` | UI, Picking, Workflow |
| `camera_model.py` | δ-Fit, Display-Muster, Blob-Detektion |

**Entfernen:** Vermischung von Solver und UI in `image_tools.py` (2000+ Zeilen).

### 12.5 Sketch-first Workflow

Primärer Pfad: `ImageCalibration` + Sketch-Constraints. Legacy
`ReferenceLine` + `Scale Solver` als dünner Wrapper — nicht zwei parallele Welten.

### 12.6 Was weglassen

- Separate `_can_use_uniform_scale_solver`-Abkürzung (durch Initialwert ersetzen)
- Alte `_distortion_energy` auf Kanten-Basis (nur Alias behalten falls Migration)
- Kaltstart Phase 3 ohne Warm-Start (immer Phase-1/2-Start)
- OpenCV-Zwang für Display-Kalibrierung

---

## 13. Konstanten und Parameter

| Symbol | Wert | Bedeutung |
|--------|------|-----------|
| \(\tau_L\) | 0,01 mm | Längen-Skalierung im Residual |
| \(\tau_a\) | \(\sin(1°)\) | Winkel-Skalierung |
| \(w\) | 25,0 | Gewicht Winkel-Constraints und \(E_{\mathrm{angle}}\) |
| \(\tau_t\) | 1,0 mm | Schwerpunkt-Strafe |
| Weld/Snap | 5,0 mm | Endpunkt-Zusammenführung |
| Snap px | 12 px | Zoom-abhängige Toleranz |
| `ftol`, `xtol`, `gtol` | \(10^{-6}\) | Optimierer |
| `max_nfev` | 250 | Optimierer |
| Jacobian-Rang | tol = max(10⁻¹⁰, 10⁻⁸ σ₁) | SVD |

---

## 14. Tests und Validierung

### 14.1 Unit-Tests (`tests/test_calibration_solver.py`)

| Testklasse | Prüft |
|------------|-------|
| `TestHomographyBasics` | H aus Ecken, Längen auf Rechteck |
| `TestTrapezoidToRectangle` | 2 Längen → Ziel-Längen |
| `TestSingleLengthScale` | 1 Länge → gleichmäßige Skalierung |
| `TestDistortionEnergyGating` | rank < 6 → \(E_{\mathrm{angle}}\) aktiv |
| Sketch-Constraints | horizontal, parallel, … |

### 14.2 Residual-Plots

`tests/plot_align_image_test_1_residuals.py` — Konvergenz Phase 1→2→3 vs.
Kaltstart; CSV/PNG unter `tests/output/`.

### 14.3 Kameramodell (geplant)

- Synthetisches verzerrtes Bild + bekanntes δ
- Roundtrip `undistort` / `inverse_undistort`
- Solver auf entzerrten UV

---

## 15. Implementierungs-Roadmap

### Phase A — Kern neu (ohne Kamera)

1. `image_coords.py` — H, UV, Welding
2. `image_solver.py` — einheitliche Parametrisierung + Warm-Start
3. Tests portieren / vereinfachen
4. `ImageCalibration`-Workflow anbinden

### Phase B — Kameramodell

5. Display-Muster 16×8 (Dialog/Vollbild)
6. Blob-Detektion + `fit_camera_model_from_points()`
7. `undistort_uv` / `inverse_undistort_uv`
8. Tesselliertes Coin-Mesh + schwarze Ränder

### Phase C — Aufräumen

9. Legacy Scale Solver als Wrapper
10. Doku in diesem File pflegen; alte Einzeldokumente archivieren

---

## Anhang: Quellcode-Referenz (Ist-Zustand)

| Konzept | Datei |
|---------|-------|
| Homographie | `image_homography.py` |
| Kalibrierungs-Solver | `image_constraint_solver.py` |
| FreeCAD-Objekte | `image_objects.py`, `image_calibration_objects.py` |
| Befehle / UI | `image_tools.py`, `commands.py` |
| Tests | `tests/test_calibration_solver.py`, `tests/helpers.py` |

---

## Anhang: Verwandte Dokumente (archiviert)

Einzelthemen wurden zuvor in separaten Files geführt; Inhalt ist hier
konsolidiert:

- `CALIBRATION_SOLVER.md` — Solver-Mathematik (Detail)
- `CAMERA_MODEL.md` — Kameramodell-Entwurf (Detail)
- `IMAGE_TOOLS.md` — UI und Objekte (Detail)
- `IMAGE_ALIGNMENT.md` — Kurzüberblick

Bei Widersprüchen gilt **dieses Dokument** (Spezifikation für Neuimplementierung).
