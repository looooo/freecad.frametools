# Kameramodell — Entwurf (Gitter-Kalibrierung + B-Spline-Entzerrung)

Stand: **Design-Dokument** (noch nicht implementiert). Beschreibt die geplante
Erweiterung des Image-Workflows in FrameTools.

**Aktuell implementiert:** projektive Homographie über vier Bildecken (`AlignedImage`,
`WarpMatrix`) und Längen-Kalibrierung — siehe [CALIBRATION_SOLVER.md](CALIBRATION_SOLVER.md).

**Siehe auch:** [IMAGE_TOOLS.md](IMAGE_TOOLS.md) (Anwendung), [IMAGE_ALIGNMENT.md](IMAGE_ALIGNMENT.md)
(Workflow), [README.md](README.md) (Doku-Übersicht).

**Geplanter UI-Workflow (Display-Muster 16×8, zwei Toolbar-Befehle):** Abschnitt [13](#13-geplanter-freecad-workflow-display-muster).

---

## 1. Motivation

Die heutige Abbildung modelliert das Foto als **Vier-Eck-Quad** mit **projektiver
Homographie** (Perspektive, Trapez). Das reicht für „Kamera schaut schräg auf eine
Ebene“, deckt aber **nicht** ab:

- **Objektivverzerrung** (Radial-/Tangentialfehler, Weitwinkel),
- leichte **Nicht-Planarität** des Trägers (gewelltes Papier),
- lokale Abweichungen, die eine reine Homographie nicht darstellen kann.

Ein **Kameramodell** soll die **Linsenentzerrung einmal** aus einem Gitter-Foto am
Rechner bestimmen, als Profil speichern und beim **Einladen jedes Fotos** automatisch
anwenden. **Schieflage und Maßstab in mm** bleiben **pro Bild** (Ecken / Längen wie
heute).

---

## 2. Zwei Ebenen: Kamera vs. Bild

| Ebene | Inhalt | Gültigkeit | Parameter (typisch) |
|-------|--------|------------|------------------------|
| **Kameramodell** | Entzerrung δ(u,v) in Bild-UV | pro Kamera / Brennweite | B-Spline 3×3 oder 4×4 KP |
| **AlignedImage** | Platzierung in der Werkstatt | pro Foto | Homographie H, 4 Ecken |

**Entkopplung:**

```
Rohpixel (u, v)
    → δ_lens  aus CameraModel     (fest, automatisch beim Laden)
    → entzerrtes UV
    → H       aus Ecken / Längen  (pro Bild, bestehender Solver)
    → Welt (X, Y) in mm
```

Was **nicht** ins Kameramodell gehört: die Homographie der **Gitter-Aufnahme** beim
Kalibrieren (Schieflage der Platte in diesem einen Foto). Nur **δ** wird übernommen.

---

## 3. Kalibrierung am Gitter (offline)

### 3.1 Eingabe

- Foto eines **planaren Gitters** (Schachbrett, Millimeterpapier, Kalibierplatte),
- bekannte **Soll-Geometrie** der Kreuzungspunkte in mm auf der Ebene (z. B.
  (0,0), (10,0), (20,0), …),
- optional **mehrere Gitter-Fotos** (verschiedene Winkel, gleiche Kamera/Brennweite)
  für robustere Schätzung von δ.

### 3.2 Schritt 1 — Schieflage (Homographie oder 3D-Lage)

Die **globale Perspektive** der Gitterebene in diesem Foto:

**Variante A — vier äußere Gitter-Ecken → Homographie H_grid**

Die Randpunkte der erkannten Punktewolke den bekannten mm-Ecken der Platte zuordnen.

**Variante B — alle Gitterpunkte → Ausgleichs-Homographie**

Robuster als nur vier Ecken; minimiert \(\sum_i \| H(u_i,v_i) - (X_i,Y_i) \|^2\).

**Variante C — 3D-Platzierung (R, t)**

Ebene Z = 0, `solvePnP` / Kameramodell. Für eine **plane** Platte äquivalent zur
Homographie; sinnvoll bei späterem vollständigen Pinhole-Modell (f, cx, cy, k₁…).

Für FrameTools reicht zunächst **Variante B**; **Variante A** als schnelle Vorschau.

### 3.3 Schritt 2 — Restfehler → B-Spline δ

Nach Schritt 1 Restfehler pro Gitterpunkt:

\[
\mathbf{r}_i = (X_i, Y_i) - H_{\mathrm{grid}}(u_i, v_i)
\]

**Wichtig:** δ wird als **Korrektur in Bild-Koordinaten** gespeichert, nicht als
Welt-mm-Rest nach H_grid. Ablauf zur Speicherung:

1. Restfehler \(\mathbf{r}_i\) in mm (oder normiert),
2. in **Bild-UV** \(\delta(u,v)\) umrechnen / direkt in (u,v) fiten,
3. B-Spline mit **Glattheitsstrafe** fitten.

Zielfunktion (schematisch):

\[
\min_\delta \sum_i \bigl\| (X_i,Y_i) - H_{\mathrm{grid}}(u_i,v_i)
  - \mathrm{lift}(\delta(u_i,v_i)) \bigr\|^2
  + \lambda_1 \|\delta\|^2 + \lambda_2 \|\nabla^2 \delta\|^2
\]

**Sequentiell fiten:** zuerst \(H_{\mathrm{grid}}\) fixieren, dann δ — sonst tauschen
Homographie und Spline die Rollen (Perspektive „steckt“ in δ).

### 3.4 B-Spline-Raster

Tensorprodukt-B-Spline über normiertes Bild \([0,1]^2\):

| Kontrollpunkt-Gitter | Freiheitsgrade (Δu, Δv) | Einsatz |
|----------------------|-------------------------|---------|
| 3×3 | 18 | milde Verzerrung |
| 4×4 | 32 | stärkere Verzerrung / großes Bildfeld |

Ein 7×7-Schachbrett (49 Ecken) liefert genug Constraints für 4×4 plus Regularisierung.

### 3.5 Optional: klassisches Pinhole-Modell

Alternativ oder zusätzlich OpenCV-typische Parameter (k₁, k₂, p₁, p₂, f, cx, cy).
Die B-Spline-Korrektur ist flexibler bei wenig Theorie; Pinhole-Parameter sind
kompakter und besser über viele Ansichten schätzbar. Beides kann langfristig
koexistieren (`CameraModelType`: `bspline` | `opencv`).

---

## 4. Datenmodell `CameraModel` (geplant)

FreeCAD-Objekt (doc-weit) oder importierbare JSON-Datei.

### 4.1 Properties (Vorschlag)

```text
CameraModel
  Label / Description
  ImageSize          (Breite, Höhe in Pixel — Referenz für normiertes UV)
  PrincipalPoint     (cx, cy) optional, sonst Bildmitte
  SplineDegreeU, SplineDegreeV   (z. B. 3)
  SplineControlPoints            (Liste von (u, v) → (Δu, Δv) in normierten Koords)
  CalibrationSource    (Pfad zum Gitter-Foto, optional)
  CalibrationRMS       (mm oder px, Qualitätshinweis)
  CameraNote           (Brennweite, Gerät, Zoom — manuell)
```

**Nicht speichern:** \(H_{\mathrm{grid}}\) der Kalibieraufnahme (posespezifisch).

### 4.2 Anwendung auf ein Foto

Funktion (Konzept):

```text
undistort_uv(u, v, camera_model) → (u', v')
```

Linear interpoliert über B-Spline-Basis; Domain [0,1]² relativ zur **Original-
Bildgröße** (Crop/EXIF-Rotation beachten).

---

## 5. Integration in `AlignedImage`

### 5.1 Neue Verknüpfung

```text
AlignedImage
  ImageFile          (wie heute)
  CameraModelLink    → CameraModel (optional)
  Corner0 … Corner1  (wie heute)
  WarpMatrix         (wie heute — projektive Platzierung in mm)
```

**Pipeline beim Anzeigen und beim Solver:**

1. Textur-UV (u, v) aus Rohbild,
2. falls `CameraModelLink`: (u', v') = undistort(u, v),
3. Homographie / Ecken: Weltposition aus (u', v') und `WarpMatrix`.

Der **Längen-Solver** ([CALIBRATION_SOLVER.md](CALIBRATION_SOLVER.md)) arbeitet
weiter auf den **entzerrten UV-Koordinaten** der Referenzlinien; δ ist **kein**
Optimierungsparameter in Phase 1–3.

### 5.2 Was pro Foto manuell bleibt

| Automatisch (CameraModel) | Pro Foto (wie heute) |
|---------------------------|----------------------|
| Linsenentzerrung | 4 Ecken in der Welt |
| gleichmäßigere Linien im UV | Soll-Längen / Sketch |
| | Verknüpfung ImageCalibration → Sketch |

Ohne Ecken/Längen ist **Maßstab und Orientierung** des Fotos in der Werkstatt unbekannt.

---

## 6. Darstellung in Coin3D

**Aktuell:** ein Quad (4 Vertices), `SoTexture2`, `SoTextureCoordinate2` linear,
`SoMatrixTransform` mit `WarpMatrix`.

**Geplant:** texturierte Fläche mit **entzerrter UV-Zuordnung**.

### 6.1 Variante A — tesselliertes Gitter (empfohlen für v1)

- Mesh z. B. 32×32 oder 64×64 über [0,1]²,
- pro Knoten: Textur-UV nach undistort(u,v), Weltposition aus Ecken-Interpolation
  oder später aus H,
- `SoCoordinate3` + `SoTextureCoordinate2` + `SoIndexedFaceSet`,
- `SoMatrixTransform` / `WarpMatrix` wie heute oder H direkt in Weltkoordinaten.

Vorteile: konsistent mit bestehendem FaceSet-Ansatz, volle Kontrolle, gut debuggbar.

### 6.2 Variante B — `SoNURBSurface`

- Kontrollpunkte in Welt-XY(Z), Texturkoordinaten an Knoten,
- konzeptionell passend zur B-Spline-Entzerrung,
- in Coin/Open Inventor aufwendiger (UV auf NURBS, Auflösung, Randverhalten).

### 6.3 Variante C — Bitmap vorverzerren

- beim Laden entzerrtes Raster erzeugen, dann Quad wie heute,
- einfach, aber speicherintensiv und unscharf beim Zoomen — für CAD eher nicht v1.

**Empfehlung:** v1 = **tesselliertes Gitter** in `ViewProviderAlignedImage`; NURBS
optional später.

---

## 7. Workflow (Gesamt)

```text
Einmalig (am Rechner):
  Gitter-Foto → Ecken detektieren
             → H_grid (Schieflage dieser Aufnahme)
             → δ(u,v) fitten, glatt
             → CameraModel speichern

Pro Arbeitsfoto:
  Foto laden → CameraModel verknüpfen → δ automatisch
            → AlignedImage / ImageCalibration
            → Ecken setzen + Längen-Solver (bestehend)
            → Sketch metrisch
```

Optional: mehrere Gitter-Fotos → ein gemeinsames δ (robuster Mittelwert / Bundle).

---

## 8. Bezug zum bestehenden Kalibrierungs-Solver

| Thema | Heute | Mit CameraModel |
|-------|--------|-----------------|
| Perspektive | 4 Ecken, H | unverändert |
| Objektiv | implizit in H / E_angle | explizit δ, vor Solver |
| Längen-Constraints | UV der Referenzlinien | UV **nach** undistort |
| E_angle / Δt | bei Rang < 6 / immer Δt | weiterhin; δ reduziert Bedarf an Scher-Kompensation |
| uniform / uv_scale | 1–2 Längen | unverändert, auf entzerrten UV |

**Reihenfolge im Code (Ziel):**

```text
world_from_image_uv(u, v):
    u2, v2 = undistort(u, v, camera_model)   # optional
    return apply_homography(u2, v2, corners)   # bestehend
```

---

## 9. Grenzen und Stolpersteine

| Risiko | Maßnahme |
|--------|----------|
| Brennweite / Zoom geändert | neues CameraModel |
| Crop, EXIF-Rotation | ImageSize und UV-Domain im Modell mitführen |
| H_grid und δ gemeinsam pro Foto neu fitten | vermeiden — nur δ global |
| Gewelltes Gitter | δ frisst Knick, nicht nur Objektiv — planare Platte |
| Zu viele Spline-DOF | 3×3 zuerst; Glattheitsstrafe; ggf. k₁,k₂ statt 4×4 |
| δ + H in einem Fit | sequentiell oder H fix beim δ-Fit |

---

## 10. Implementierungs-Roadmap (Vorschlag)

Siehe auch **Abschnitt 13.8** (UI mit Display-Muster und zwei Befehlen).

1. **`CameraModel`** — Property-Objekt, JSON Import/Export, undistort_uv().
2. **`AlignedImage.CameraModelLink`** — UV-Pipeline in `_world_on_image_uv` /
   `image_homography.py`.
3. **Gitter-Kalibrierungs-Wizard** — Eckenfindung (OpenCV optional), H_grid + δ-Fit,
   Speichern als CameraModel.
4. **ViewProvider** — tesselliertes Mesh statt 4-Punkt-Quad wenn CameraModel gesetzt.
5. **Tests** — synthetisches Gitter + künstliche Verzerrung, Roundtrip und Solver mit
   entzerrten UV.
6. Optional: Pinhole/OpenCV-Pfad, Mehrbild-Bundle.

---

## 11. Literatur / Referenzen

- OpenCV `calibrateCamera`, `findChessboardCorners` — Mehrbild-Objektivkalibrierung.
- Zhang, „A Flexible New Technique for Camera Calibration“ — Standard-Pipeline.
- Bestehende FrameTools-Homographie: `image_homography.py`, `compute_homography`.

---

## 12. Glossar

| Begriff | Bedeutung |
|---------|-----------|
| δ(u,v) | B-Spline-Entzerrung in Bild-UV (Kameramodell) |
| H | Projektive Abbildung UV → Welt-mm (pro AlignedImage) |
| H_grid | Homographie nur für Gitter-Kalibrierungsfoto (nicht speichern) |
| undistort | Anwendung von δ auf Roh-UV vor H |
| inverse_undistort | Roh-UV zum Textur-Sampling bei rechteckiger Anzeige |

---

## 13. Geplanter FreeCAD-Workflow (Display-Muster)

Dieser Abschnitt beschreibt den **vorgesehenen UI-Ablauf** in FrameTools: kein
gedrucktes Schachbrett nötig, stattdessen ein **Punktemuster auf dem Monitor**, das
der Nutzer mit der **Arbeitskamera** fotografiert.

### 13.1 Übersicht

Zwei neue Befehle in der Image-Toolbar (Namen vorläufig):

| Befehl | Funktion |
|--------|----------|
| **Kameramodell kalibrieren** | Muster 16×8 am Display anzeigen → Foto laden → δ berechnen → `CameraModel` im Dokument |
| **Bild mit Entzerrung laden** | Foto + verknüpftes `CameraModel` → `AlignedImage` mit B-Spline-UV, rechteckige Fläche, schwarze Ränder |

```text
[Kameramodell kalibrieren]
    → Vollbild-/Dialog: 16×8 Punkte (hell auf schwarz)
    → Nutzer fotografiert den Bildschirm
    → Foto in FreeCAD wählen
    → Punkte automatisch finden (128 Stück)
    → H_grid + δ(u,v) fitten → CameraModel speichern

[Bild mit Entzerrung laden]
    → CameraModel wählen (oder zuletzt verwendet)
    → Arbeitsfoto wählen
    → AlignedImage: rechteckiges Quad/Mesh in der Szene
    → Textur-Sampling über inverse Entzerrung
    → Bereich außerhalb gültiger Abbildung: schwarz
    → danach wie heute: Ecken / ImageCalibration / Längen
```

### 13.2 Display-Muster 16×8

**Geometrie:** regelmäßiges Raster **16 Spalten × 8 Zeilen** = **128 Kalibrierpunkte**.

| Parameter | Vorschlag |
|-----------|-----------|
| Darstellung | Vollbild schwarz, weiße/helle Kreise an Gitterpositionen |
| Rand | Randabstand (z. B. 5 %), damit äußere Punkte nicht am Monitorrand liegen |
| Metadaten (mit Muster speichern) | Auflösung in Pixel, Position/Größe des Muster-Rechtecks, Punktabstand in px |
| Optional | kurzer Text „FrameTools Kalibrierung — Foto aufnehmen“ |

**Warum Monitor statt Druck:** exakte Soll-Pixelkoordinaten bekannt, kein Druckmaßstab;
128 Punkte reichen für B-Spline 4×4 plus Regularisierung mit Reserve.

**Hinweise an den Nutzer (Dialog):**

- Kamera **dieselbe Brennweite/Zoom** wie bei späteren Arbeitsfotos,
- Bildschirm **frontal** anfahren (leichte Schräge ist OK — H_grid entfernt sie),
- **kein Blitz**, Reflexionen vermeiden,
- bei OLED/Laptop: Helligkeit hoch, ggf. längere Belichtung gegen Flackern.

**Stolpersteine Display-Kalibrierung:**

| Effekt | Folge |
|--------|--------|
| Moiré | Ecken ungenau → Subpixel-Fit + große Punkte im Muster |
| Refresh / PWM | Streifen im Foto → längere Belichtung, statisches Muster |
| Spiegelung | Ausreißer → robustes Fit, RAND-Maske |
| Monitor nicht planar | δ nimmt Knick mit auf — weniger ideal als Druckplatte |

### 13.3 Automatische Punkterkennung (128-Punkt-Muster)

Anders als Schachbrett-OpenCV: **Blob-Detektion** auf hellem Gitter:

1. Graustufen, Schwellwert / adaptive Threshold (helle Punkte),
2. Konturen → Schwerpunkt pro Blob,
3. **Raster sortieren:** Cluster nach y → Zeilen, innerhalb Zeile nach x → Spalten,
4. Plausibilität: genau 8×16 = 128 Punkte; sonst Fehlermeldung + manuelle Korrektur (später),
5. **Subpixel-Verfeinerung** (Schwerpunkt in Nachbarschaft oder `cornerSubPix`-ähnlich).

Soll-Koordinaten pro Punkt \(i\): normierte Display-UV \((u_i^{\mathrm{soll}}, v_i^{\mathrm{soll}})\)
aus gespeicherten Muster-Metadaten (nicht mm — die „Welt“ beim Kalibrieren ist die
**ideale Bildschirm-Ebene** in Pixel).

OpenCV `findChessboardCorners` ist **optional** falls das Muster stattdessen als
Schachbrett gerendert wird; für **Punktraster** reicht eigene Detektion (kein OpenCV
zwingend nötig).

### 13.4 Berechnung beim Kameramodell-Befehl

Reihenfolge (Entkopplung Schieflage / Verzerrung):

```
Detektierte (u, v)_raw im Foto     Soll (u, v)_display aus Muster-Metadaten
              │                              │
              └──────────┬───────────────────┘
                         ▼
              Schritt 1: H_grid (Ausgleich)
              Perspektive Foto ↔ ideale Display-Ebene
                         │
                         ▼
              Rest: (u,v)_display − H_grid(u,v)_raw
                         │
                         ▼
              Schritt 2: B-Spline δ(u,v) in Bild-UV
              (Korrektur im **Kamera-Fotoraster**, siehe unten)
                         │
                         ▼
              CameraModel im Dokument speichern
```

**Umrechnung δ:** Restfehler nach H_grid werden auf **normierte Foto-UV** \([0,1]^2\)
bezogen und als Verschiebung δ(u,v) gespeichert — dieselbe Domain wie bei späteren
Arbeitsfotos (**Referenz: Pixelkoordinaten des Kamera-Chips**, nicht Display-Pixel).

Referenz-Fotogröße (`ImageSize`) des Kalibrierbildes im `CameraModel` ablegen.

### 13.5 Befehl „Bild mit Entzerrung laden“

**Ziel:** Arbeitsfoto **rechteckig** in FreeCAD einbinden (wie gewohnt ein
achsenparalleles Bild-Quad), aber **inhaltlich entzerrt**; Randbereiche ohne gültige
Zuordnung **schwarz**.

#### Rechteckige Anzeige + B-Spline

Das Anzeige-Quad bleibt ein **Rechteck** in der Coin-Szene (UV-Anzeige-Raster
\([0,1]^2\)). Die **Textur** ist das **unveränderte Roh-JPEG** (rechteckig).

Für jeden Anzeige-Knoten \((u_d, v_d)\) auf dem Rechteck:

1. **Inverse Entzerrung:** finde Rohkoordinaten \((u_r, v_r)\), sodass  
   \(\text{undistort}(u_r, v_r) = (u_d, v_d)\)  
   (Newton / Lookup-Grid / festes iteratives Verfahren),
2. **Sampling:** Textur bei \((u_r, v_r)\),
3. **Schwarz**, wenn \((u_r, v_r) \notin [0,1]^2\) oder außerhalb des gültigen
   δ-Domain / Konvergenz der Inversion fehlschlägt.

```text
Anzeige-Rechteck (u_d, v_d)  ──inverse_undistort──►  (u_r, v_r)  ──► Textur
        │                                                    │
        └── außerhalb / ungültig ───────────────────────────► schwarz
```

**Warum inverse Richtung:** δ ist definiert als Roh → entzerrt. Coin fragt pro
**Anzeige-Pixel**, welche Texturkoordinate gelesen werden soll — das ist die **Inverse**.

Typisch entstehen **schwarze Ränder** (Letterboxing): nach Entzerrung füllt das
Kamerabild nicht mehr das volle Rechteck (Fischauge „zieht“ Ränder nach innen).

#### Coin-Umsetzung

- Tesselliertes Mesh (z. B. 64×64) über Anzeige-\([0,1]^2\),
- `SoTextureCoordinate2`: pro Vertex \((u_r, v_r)\) oder Sentinel → schwarze Farbe,
- `SoTexture2`: Rohbild,
- optional `SoMaterial` / zweite Ebene für explizit schwarze Dreiecke,
- Weltplatzierung weiter über `Corner0…Corner1` und `WarpMatrix` (bestehender Pfad).

**Wichtig:** Das Rechteck bezieht sich auf **entzerrten Bildinhalt**, nicht auf das
verzerrte Rohformat — Nutzer sieht ein „gerades“ Bild; schwarze Zonen markieren fehlende
Pixel nach Entzerrung.

### 13.6 FreeCAD-Objekte und Properties (Erweiterung)

```text
CameraModel
  PatternType              "display_dots_16x8"
  CalibrationImageSize     (width, height) px des Kalibrierfotos
  PatternMetadata          JSON: Display-Auflösung, Muster-Rect, Punktliste Soll-UV
  SplineControlPoints      δ(u,v) B-Spline (3×3 oder 4×4)
  CalibrationRMS           px
  CalibrationImage         Pfad Referenzfoto (optional)

AlignedImage  (Erweiterung)
  CameraModelLink          → CameraModel
  UndistortDisplay         True wenn über „Bild mit Entzerrung laden“
  … Corner0…Corner1, WarpMatrix wie heute
```

### 13.7 Ablauf im Gesamt-Workflow

```text
Einmalig:
  [Kameramodell kalibrieren] → CameraModel im Dokument

Pro Projekt / Foto:
  [Bild mit Entzerrung laden] → AlignedImage (entzerrt, rechteckig, schwarze Ränder)
  → optional ImageCalibration, Längen, Solve (auf entzerrten UV)

Overlay / Feature-Pairs:
  UV-Operationen nutzen undistort() konsistent (bestehende Pipeline erweitern)
```

### 13.8 Implementierungs-Roadmap (UI)

1. **`CameraModelCalibrate` Command** — Vollbild-Dialog mit 16×8-Muster + Metadaten-Export.
2. **Blob-Detektion + Grid-Sort** — 128 Punkte, Validierung, Vorschau im Dialog.
3. **`fit_camera_model_from_points()`** — H_grid, dann δ; speichern als `CameraModel`.
4. **`LoadUndistortedImage` Command** — Datei + Model → `AlignedImage`.
5. **`undistort_uv` / `inverse_undistort_uv`** in `image_homography.py` oder `camera_model.py`.
6. **ViewProvider** — tesselliertes Mesh, schwarze Ränder (UV außerhalb Domain).
7. **Tests** — synthetisches verzerrtes Bild + bekanntes δ, Roundtrip inverse.

Geplante Toolbar-Ergänzung: siehe [IMAGE_ALIGNMENT.md](IMAGE_ALIGNMENT.md).

---
