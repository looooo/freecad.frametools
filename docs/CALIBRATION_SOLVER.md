# Kalibrierungs-Solver — mathematische Beschreibung

Stand: Implementierung in `freecad/frametools/image_constraint_solver.py` und
`freecad/frametools/image_homography.py` (FreeCAD FrameTools Workbench).

Dieses Dokument beschreibt den **aktuellen** Zustand des Scale-/Kalibrierungs-Solvers:
Problemstellung, Koordinaten, Nebenbedingungen, Zielfunktion und die zwei
Lösungspfade (`uniform_scale` und `corners`).

**PDF:** `docs/pdf/build_calibration_solver.sh` → [pdf/CALIBRATION_SOLVER.pdf](pdf/CALIBRATION_SOLVER.pdf)

Siehe auch [README.md](README.md), [IMAGE_TOOLS.md](IMAGE_TOOLS.md) (Anwendung/UI) und
[CAMERA_MODEL.md](CAMERA_MODEL.md) (geplante Objektiv-Entzerrung vor der Homographie).

---

## 1. Problemstellung

Gegeben ist ein perspektivisch gerendertes Bild als **Vier-Eck-Quad** in der
Welt-XY-Ebene (Z der Ecken bleibt während der Optimierung fix). Auf dem Bild liegen
Referenzlinien mit bekannten UV-Endpunkten und einer **Soll-Länge** in Millimetern.
Optional sind **Winkel-Bedingungen** aus dem verknüpften Sketch aktiv
(horizontal, senkrecht, parallel, rechtwinklig).

Gesucht sind neue Bildeck-Positionen, sodass

1. die modellierten Längen der Referenzlinien den Soll-Werten entsprechen,
2. die Winkel-Bedingungen erfüllt sind (falls gesetzt),
3. bei mehrdeutigen Fällen die Verzerrung gering und die starre Verschiebung
   des Quads klein bleibt.

Das Sketch-Placement wird **nicht** optimiert; nur die vier Bildecken werden bewegt.
Die Sketch-Geometrie wird nach dem Solve aus den kalibrierten UV-Koordinaten neu
aufgesetzt (`AlignedSketch`).

---

## 2. Homographie und Bildecken

### 2.1 UV-Koordinaten

Jeder Punkt auf dem Bild trägt Parameter \((u, v) \in [0,1]^2\):

| Ecke | UV |
|------|-----|
| \(c_0\) (Corner0) | \((0, 0)\) |
| \(c_x\) (CornerX) | \((1, 0)\) |
| \(c_1\) (Corner1) | \((1, 1)\) |
| \(c_y\) (CornerY) | \((0, 1)\) |

Die Abbildung \((u, v) \mapsto (x, y)\) in der Welt-XY-Ebene erfolgt über eine
**projektive Homographie** \(H \in \mathbb{R}^{3 \times 3}\) mit \(H_{33} = 1\):

\[
\begin{pmatrix} x' \\ y' \\ w \end{pmatrix}
= H \begin{pmatrix} u \\ v \\ 1 \end{pmatrix},
\qquad
x = \frac{x'}{w},\; y = \frac{y'}{w}.
\]

Explizit:

\[
x(u,v) = \frac{H_{11} u + H_{12} v + H_{13}}{H_{31} u + H_{32} v + 1},
\qquad
y(u,v) = \frac{H_{21} u + H_{22} v + H_{23}}{H_{31} u + H_{32} v + 1}.
\]

\(H\) wird aus den vier Bildecken eindeutig bestimmt (`compute_homography` in
`image_homography.py`), indem die vier Paare \(((0,0), c_0), ((1,0), c_x), \ldots\)
in ein lineares System \(A \mathbf{h} = \mathbf{b}\) (8 Unbekannte) überführt werden.

### 2.2 Optimierungsvariablen

Im Modus **`corners`** sind die Unbekannten die acht XY-Koordinaten der Ecken:

\[
\mathbf{p} =
\bigl(x_0, y_0,\; x_x, y_x,\; x_1, y_1,\; x_y, y_y\bigr)^\top \in \mathbb{R}^8.
\]

Die Z-Koordinaten \((z_0, z_x, z_1, z_y)\) sind **fix** und stammen aus dem
Ausgangszustand \(\mathbf{p}_0\).

**Effektive Freiheitsgrade:** 8 Eckparameter minus 2 durch die
Schwerpunkt-Nebenbedingung (Abschnitt 6) → **6** relevante DOF für die
Entscheidung, ob \(E_{\mathrm{angle}}\) in die Optimierung eingeht.
\(\Delta t\) ist in Phase 3 (`corners`) immer im Residualvektor.

Aus \(\mathbf{p}\) und den fixen Z-Werten werden die Ecken-Vektoren
\(\mathbf{c}_0, \mathbf{c}_x, \mathbf{c}_1, \mathbf{c}_y \in \mathbb{R}^3\)
rekonstruiert und daraus \(H(\mathbf{p})\).

---

## 3. Referenzlinien und Längenbedingungen

### 3.1 Endpunkt-Welding

Bevor Längen-Specs erzeugt werden, werden Welt-Endpunkte, die näher als
\(\delta_{\mathrm{weld}} = 5\,\mathrm{mm}\) beieinander liegen, zu einem
gemeinsamen UV-Knoten zusammengeführt (Mittelwert der UV-Koordinaten).

### 3.2 Längen-Spec

Für jede Referenzlinie \(i\):

\[
\ell_i = \bigl((u_{0,i}, v_{0,i}),\ (u_{1,i}, v_{1,i}),\ L_i^{\mathrm{soll}}\bigr).
\]

Die **modellierte Länge** unter Homographie \(H\):

\[
\hat{L}_i(H)
= \left\|
\pi\!\left(H \begin{pmatrix} u_{0,i} \\ v_{0,i} \\ 1 \end{pmatrix}\right)
- \pi\!\left(H \begin{pmatrix} u_{1,i} \\ v_{1,i} \\ 1 \end{pmatrix}\right)
\right\|_2,
\]

wobei \(\pi(x', y', w)^\top = (x'/w,\; y'/w)^\top\) die Dehomogenisierung ist
(`_line_length_uv`).

**Längen-Restfehler:**

\[
r_i^{\mathrm{len}}(\mathbf{p})
= \hat{L}_i\!\bigl(H(\mathbf{p})\bigr) - L_i^{\mathrm{soll}}.
\]

---

## 4. Winkel-Bedingungen

Für eine Linie mit UV-Endpunkten wird die **Einheits-Richtung** in der XY-Ebene:

\[
\mathbf{d}(H;\, u_0, v_0, u_1, v_1)
= \frac{\pi(H [u_1, v_1, 1]^\top) - \pi(H [u_0, v_0, 1]^\top)}
       {\|\cdots\|_2}.
\]

### 4.1 Parallelität (zwei Linien \(a, b\))

\[
r_{a,b}^{\parallel} = \sin\angle(\mathbf{d}_a, \mathbf{d}_b)
= (\mathbf{d}_a)_x (\mathbf{d}_b)_y - (\mathbf{d}_a)_y (\mathbf{d}_b)_x.
\]

Null genau dann, wenn die Richtungen parallel oder antiparallel sind
(`_parallel_sin_xy`).

### 4.2 Rechtwinkligkeit

\[
r_{a,b}^{\perp} = \mathbf{d}_a \cdot \mathbf{d}_b.
\]

Null genau dann, wenn der Winkel \(90°\) oder \(270°\) ist.

### 4.3 Horizontal / Senkrecht (Sketch-Achsen)

Bei der Bildkalibrierung wird immer das verknüpfte Sketch übergeben. Dann gilt:

- **Horizontal:** Linie parallel zur Sketch-\(+X\)-Achse.
- **Senkrecht (vertical):** Linie parallel zur Sketch-\(+Y\)-Achse.

Sei \(\mathbf{r}_h, \mathbf{r}_v \in \mathbb{R}^2\) die normierten XY-Richtungen
der Sketch-Achsen (`_sketch_axis_directions_xy`). Für eine Linie mit Index \(g\):

\[
r_g^{\mathrm{horiz}} = \sin\angle\!\bigl(\mathbf{d}_g,\, \mathbf{r}_h\bigr),
\qquad
r_g^{\mathrm{vert}} = \sin\angle\!\bigl(\mathbf{d}_g,\, \mathbf{r}_v\bigr).
\]

Das Sketch-Placement selbst wird **nicht** verändert; die Bedingung bezieht sich
auf die Welt-XY-Richtung der projizierten Linie relativ zum festen Sketch.

---

## 5. Winkelerhaltung \(E_{\mathrm{angle}}\)

Nebenbedingung: an **jedem der vier Quad-Eckpunkte** \(c_0, c_x, c_1, c_y\)
soll der **Innenwinkel** zwischen den beiden anliegenden Kanten unverändert
bleiben (keine Scherung in der Bildebene).

### 5.1 Kantenvektoren pro Ecke

An jedem Eckpunkt werden die beiden inzidenten Kantenvektoren gebildet
(nur XY). Beispiel \(c_0\):

\[
\mathbf{e}_{u} = \mathbf{c}_x - \mathbf{c}_0,\qquad
\mathbf{e}_{v} = \mathbf{c}_y - \mathbf{c}_0.
\]

Analog an \(c_x\) (\(\mathbf{c}_0-\mathbf{c}_x\), \(\mathbf{c}_1-\mathbf{c}_x\)),
\(c_1\) und \(c_y\).

Am Ausgang \(\mathbf{p}_0\) und nach der Optimierung \(\mathbf{p}\) für Ecke \(k\):

\[
\cos\alpha_k = \frac{\mathbf{e}_{k,1} \cdot \mathbf{e}_{k,2}}{\|\mathbf{e}_{k,1}\|\,\|\mathbf{e}_{k,2}\|},
\qquad
\sin\alpha_k = \frac{(\mathbf{e}_{k,1})_x (\mathbf{e}_{k,2})_y - (\mathbf{e}_{k,1})_y (\mathbf{e}_{k,2})_x}{\|\mathbf{e}_{k,1}\|\,\|\mathbf{e}_{k,2}\|}.
\]

### 5.2 Energie pro Ecke und gesamt

\[
E_{\mathrm{angle},k}(\mathbf{p})
= (\cos\beta_k - \cos\alpha_k)^2 + (\sin\beta_k - \sin\alpha_k)^2,
\qquad
E_{\mathrm{angle}}(\mathbf{p}) = \sum_{k \in \{c_0,c_x,c_1,c_y\}} E_{\mathrm{angle},k}(\mathbf{p}).
\]

\(E_{\mathrm{angle}} = 0\) genau dann, wenn **alle vier** Eckwinkel gleich
geblieben sind.

**Interpretation:**

| Verformung | \(E_{\mathrm{angle}}\) |
|------------|-------------------------|
| Skalierung entlang U/V (\(s_x \neq s_y\) erlaubt) | \(0\) |
| Gemeinsame Rotation aller Ecken | \(0\) |
| Scherung / Perspektiv-Verzerrung an mindestens einer Ecke | \(> 0\) |

Im Residualvektor: je Ecke \(\sqrt{E_{\mathrm{angle},k}}\) (Funktionen
`_angle_preserving_energy_per_corner`, Summe in `_angle_preserving_energy`;
`distortion_energy` im Report ist ein Alias für die Summe).

---

## 6. Strafe für starre Translation (Nebenbedingung)

Zur Auswahl unter längen-kompatiblen Lösungen wird die Verschiebung des
Eck-Schwerpunkts bestraft. Das entspricht der Neigung, den Quad-Schwerpunkt
nahe der Ausgangslage zu halten, und **reduziert die effektiven Freiheitsgrade
um 2** (Translation in \(x\) und \(y\)). Zusammen mit den 8 Eckparametern
bleiben damit **6** unabhängige Freiheitsgrade für die Rang-Entscheidung
(Abschnitt 7.1).

Sei \(P_0, P_1 \in \mathbb{R}^{4 \times 2}\) die XY-Matrizen der vier Ecken
(vor/nach). Schwerpunkte:

\[
\mathbf{g}_0 = \frac{1}{4}\sum_k \mathbf{p}_{0,k}, \qquad
\mathbf{g}_1 = \frac{1}{4}\sum_k \mathbf{p}_{1,k}.
\]

**Translation (Skalar für Residuum):**

\[
\Delta t = \|\mathbf{g}_1 - \mathbf{g}_0\|_2.
\]

Zusätzlich wird für Diagnose die **Kabsch-Rotation** (ohne Skalierung) zwischen
den zentrierten Punktwolken berechnet; der Rotationswinkel erscheint im Report,
ist aber **nicht** Teil des Residualvektors.

---

## 7. Residualvektor und Zielfunktion (Modus `corners`)

Der nichtlineare Ausgleich minimiert

\[
\min_{\mathbf{p}} \;\| \mathbf{r}(\mathbf{p}) \|_2^2
\]

mit `scipy.optimize.least_squares` (Trust-Region-Reflective).

### 7.1 Zusammensetzung

Primäre Restfehler (Längen, Winkel) sind immer enthalten. Die Eck-Energien
\(w\sqrt{E_{\mathrm{angle},k}}\) werden **nur bei Unterbestimmung** angehängt
(Rang der primären Jacobian-Matrix \(< 6\)). Die Schwerpunkt-Strafe
\(\Delta t / \tau_t\) ist in Phase 3 **immer** enthalten.

\[
\mathbf{r}(\mathbf{p}) =
\begin{bmatrix}
\mathbf{r}^{\mathrm{len}} / \tau_L \\
w \cdot \mathbf{r}^{\mathrm{ang}} / \tau_a \\
\mathbf{1}_{\mathrm{rank} < 6}\,w \sqrt{E_{\mathrm{angle},c_0}(\mathbf{p})} \\
\mathbf{1}_{\mathrm{rank} < 6}\,w \sqrt{E_{\mathrm{angle},c_x}(\mathbf{p})} \\
\mathbf{1}_{\mathrm{rank} < 6}\,w \sqrt{E_{\mathrm{angle},c_1}(\mathbf{p})} \\
\mathbf{1}_{\mathrm{rank} < 6}\,w \sqrt{E_{\mathrm{angle},c_y}(\mathbf{p})} \\
\Delta t / \tau_t
\end{bmatrix}.
\]

**Rang-Bestimmung:** Am Startpunkt \(\mathbf{p}_0\) wird die Jacobian-Matrix
\(J = \partial \mathbf{r}^{\mathrm{prim}} / \partial \mathbf{p}\) numerisch
gebildet (\(\mathbf{r}^{\mathrm{prim}}\) = Längen- und Winkel-Restfehler ohne
Skalierung). Der Rang wird per SVD mit Toleranz
\(\max(10^{-10},\,10^{-8}\,\sigma_1)\) bestimmt. Bei \(\mathrm{rank} \geq 6\)
(gelöst oder überbestimmt in den **6 effektiven** Freiheitsgraden) entfällt
\(E_{\mathrm{angle}}\) in der Optimierung; \(\Delta t\) bleibt aktiv. Alle
Werte werden weiterhin im Report ausgewiesen.

| Symbol | Wert (Code) | Bedeutung |
|--------|-------------|-----------|
| \(\tau_L\) | `0.01` mm | Längen-Toleranz-Skalierung |
| \(\tau_a\) | \(\sin(1°)\) | Winkel-Toleranz (über Sinus) |
| \(w\) | `25.0` | Gewicht für \(\mathbf{r}^{\mathrm{ang}}/\tau_a\) und \(\sqrt{E_{\mathrm{angle},k}}\) |
| \(\tau_t\) | `1.0` mm | Translation-Toleranz |

Die Winkel-Restfehler \(\mathbf{r}^{\mathrm{ang}}\) werden nur angehängt, wenn
Sketch-Constraints und `line_by_geo` übergeben werden.

### 7.2 Numerische Parameter

| Parameter | Wert |
|-----------|------|
| `ftol`, `xtol`, `gtol` | \(10^{-6}\) |
| `max_nfev` | 250 |
| Verfeinerung (wenn \(\max_i |r_i^{\mathrm{len}}| \geq \tau_L\)) | `ftol` = \(10^{-9}\), max. 100 NFEV |

---

## 8. Spezialfall: Einheitliche Skalierung (Modus `uniform_scale`)

**Auslöser:** genau **eine** Soll-Länge und **keine** Winkel-Bedingungen
(`_can_use_uniform_scale_solver`).

Statt `least_squares` wird analytisch skaliert. Der Quad-Schwerpunkt \(\mathbf{g}_0\)
bleibt fix:

\[
\mathbf{c}_k' = \mathbf{g}_0 + s\,(\mathbf{c}_k - \mathbf{g}_0),
\qquad k \in \{0, x, 1, y\}.
\]

Skalenfaktor aus der einzigen Längenbedingung:

\[
s = \frac{L^{\mathrm{soll}}}{\hat{L}\!\left(H(\mathbf{p}_0);\, \ell\right)}.
\]

In diesem Modus ist \(E_{\mathrm{angle}} = 0\) (alle Eckwinkel unverändert).
Es gibt keinen Optimierungs-Residualvektor; `cost = 0`, `success = True`.

---

## 9. Zwei-Phasen-Lösung: UV-Skalierung + Ecken (Modus `corners` mit Warmstart)

**Auslöser:** genau **zwei** Soll-Längen und **keine** Winkel-Bedingungen
(`_can_use_uv_scale_warm_start`).

### Phase 1 — unabhängige Skalierung entlang U/V

Pivot im UV-Parameterraum \((u,v) = (0{,}5, 0{,}5)\) am Quad-Schwerpunkt
\(\mathbf{g}_0\). Mit \(\mathbf{e}_u = \mathbf{c}_x - \mathbf{c}_0\),
\(\mathbf{e}_v = \mathbf{c}_y - \mathbf{c}_0\) und Parametern \(s_x, s_y\):

\[
\mathbf{p}'(u,v) = \mathbf{g}_0 + s_x (u - 0{,}5)\,\mathbf{e}_u
  + s_y (v - 0{,}5)\,\mathbf{e}_v,
\qquad (u,v) \in \{(0,0), (1,0), (1,1), (0,1)\}.
\]

Kanten bleiben \(s_x \mathbf{e}_u\) und \(s_y \mathbf{e}_v\); Schwerpunkt und alle
Eckwinkel bleiben unverändert (\(E_{\mathrm{angle}} = 0\)). Es wird
`least_squares` nur auf \((s_x, s_y)\) mit den skalierten Längen-Restfehlern
\(\mathbf{r}^{\mathrm{len}} / \tau_L\) ausgeführt.

### Phase 2 — volle Eckoptimierung

Startpunkt \(\mathbf{p}_1 = \mathbf{p}(s_x, s_y)\) aus Phase 1; danach wie bisher
`least_squares` auf alle 8 Eck-Koordinaten mit vollem Residualvektor (Längen,
optional Winkel, \(\sqrt{E_{\mathrm{angle},k}}\), \(\Delta t\)).

Phase 2 dient der Verfeinerung und für Fälle, in denen Phase 1 die Längen noch
nicht exakt trifft; bei kompatiblen Zielen bleibt \(E_{\mathrm{angle}} \approx 0\).

Im Report: `uv_scale_warm_start`, `scale_sx`, `scale_sy`, `uv_scale_nfev`.

---

## 10. Entscheidungslogik (aktueller Solver)

```
compute_calibration_from_specs / compute_calibration_corners
        |
        v
   _solve_corner_calibration
        |
        +- Phase 1: uniform_scale (1D) — immer
        |      Abbruch wenn max |ΔL| < τ_L, E_angle ≈ 0 (nur ohne Winkel-Bedingungen)
        |
        +- Phase 2: uv_scale (2D, sx/sy) — immer auf Phase-1-Ergebnis
        |      Abbruch wenn max |ΔL| < τ_L, E_angle ≈ 0 (nur ohne Winkel-Bedingungen)
        |
        +- Phase 3: least_squares auf p ∈ R^8 (Ecken)
               Residuen: Längen + Winkel + w·√E_angle + [Δt]
```

Bei **Winkel-Bedingungen** laufen alle drei Phasen; Early-Exit nach Phase 1/2 entfällt
(Warmstart bis Phase 3).

---

## 11. Typische Constraint-Fälle

| Konfiguration | Erwartetes Verhalten (Stand heute) |
|---------------|-------------------------------------|
| 1 Soll-Länge | `uniform_scale`, \(E_{\mathrm{angle}} = 0\) |
| 2 Soll-Längen, keine Winkel | Kaskade 1D→2D; Abbruch in Phase 2 wenn exakt (`uv_scale`), sonst Phase 3 |
| 1 Länge + Winkel (z. B. horizontal) | `corners`, `least_squares`; Winkel gewichtet; \(E_{\mathrm{angle}}\) und \(\Delta t\) als Tie-Breaker |
| 2 Längen + Winkel | `corners` ab Ausgang (kein UV-Warmstart) |
| Viele Längen + Winkel (überbestimmt) | Längen- und Winkel-Terme dominieren; \(E_{\mathrm{angle}}\) und \(\Delta t\) entfallen bei Rang \(\geq 6\) |

Bei **unterbestimmten** Systemen ohne UV-Warmstart bestimmen
\(\sqrt{E_{\mathrm{angle},k}}\) und \(\Delta t\) die Lösung unter den
längen-treuen Alternativen — der reine 8-Parameter-Start kann in Scher-Minima
laufen (siehe Residual-Plot `tests/plot_align_image_test_1_residuals.py`).

---

## 12. Anwendung der Lösung (Kurzüberblick)

Nach dem Solve:

1. Neue Ecken \(\mathbf{c}_0', \ldots, \mathbf{c}_y'\) setzen und `WarpMatrix`
   aus \(H\) ableiten.
2. Gespeicherte UV aller abhängigen Objekte mit \(H'\) zurück in die Welt
   projizieren (`apply_corner_calibration`).
3. Sketch-Linien aus kalibrierten UV neu schreiben; Sketch-Placement unverändert.

---

## 13. Implementierungsreferenzen

| Konzept | Funktion | Datei |
|---------|----------|-------|
| Homographie aus Ecken | `compute_homography`, `_homography_from_corners` | `image_homography.py` |
| Länge in UV | `_line_length_uv` | `image_homography.py` |
| Richtung / Parallel-Sinus | `_direction_xy_from_uv_line`, `_parallel_sin_xy` | `image_homography.py` |
| Winkelerhaltung \(E_{\mathrm{angle}}\) (Summe) | `_angle_preserving_energy` | `image_constraint_solver.py` |
| Winkelerhaltung pro Ecke | `_angle_preserving_energy_per_corner` | `image_constraint_solver.py` |
| Residualvektor | `_calibration_residuals` | `image_constraint_solver.py` |
| Uniform-Scale | `_solve_uniform_scale_calibration` | `image_constraint_solver.py` |
| UV-Skalierung Phase 1 | `_solve_uv_scale_phase`, `_uv_scale_params` | `image_constraint_solver.py` |
| Allgemeiner Solve | `_solve_corner_calibration` | `image_constraint_solver.py` |
| Einstieg (Tests/UI) | `compute_calibration_from_specs` | `image_constraint_solver.py` |

---

## 14. Hinweise / offene Punkte

1. **Zwei Solver-Pfade:** `uniform_scale` und `corners` verwenden unterschiedliche
   Parametrisierungen; eine einheitliche niedrigdimensionale Parametrisierung
   (z. B. Skala/Rotation ohne Scherung) ist derzeit nicht implementiert.

2. **\(E_{\mathrm{angle}}\):** Null genau wenn der U/V-Winkel an \(c_0\) erhalten
   bleibt; erlaubt unterschiedliche Skalierung in U und V ohne Scherung.

3. **Sketch-Placement:** Horizontal/Senkrecht beziehen sich auf feste Sketch-Achsen;
   das Bild wird allein über Eckverschiebungen ausgerichtet.

4. **Perspektive:** \(H_{31}, H_{32}\) können von null verschieden sein; die
   Optimierung variiert Ecken so, dass sich die projektive Abbildung ändert — nicht
   nur eine affine Bildtransformation.
