# Kalibrierungs-Solver — mathematische Beschreibung

Stand: Implementierung in `freecad/frametools/image_constraint_solver.py` und
`freecad/frametools/image_homography.py` (FreeCAD FrameTools Workbench).

Dieses Dokument beschreibt den **aktuellen** Zustand des Scale-/Kalibrierungs-Solvers:
Problemstellung, Koordinaten, Nebenbedingungen, Zielfunktion und die zwei
Lösungspfade (`uniform_scale` und `corners`).

**PDF:** `docs/pdf/build_calibration_solver.sh` → [pdf/CALIBRATION_SOLVER.pdf](pdf/CALIBRATION_SOLVER.pdf)

Siehe auch [README.md](README.md) und [IMAGE_TOOLS.md](IMAGE_TOOLS.md) (Anwendung/UI).

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

## 5. Verzerrungsenergie \(E_{\mathrm{dist}}\)

Die Verzerrungsenergie misst, wie stark sich die **Kanten-Basis** des Quads
gegenüber dem Ausgangszustand verformt.

### 5.1 Kanten-Basis

\[
E(\mathbf{p}) =
\begin{bmatrix} \mathbf{e}_u & \mathbf{e}_v \end{bmatrix}
\in \mathbb{R}^{2 \times 2},
\qquad
\mathbf{e}_u = \mathbf{c}_x - \mathbf{c}_0,\;
\mathbf{e}_v = \mathbf{c}_y - \mathbf{c}_0
\]

(nur XY-Komponenten). Entsprechend \(E_0 = E(\mathbf{p}_0)\), \(E_1 = E(\mathbf{p})\).

### 5.2 Deformationsgradient in der Kanten-Basis

\[
F = E_1 \, E_0^{-1} \in \mathbb{R}^{2 \times 2}.
\]

\(F\) beschreibt die affine Verformung der UV-Kantenvektoren vom Ausgang zum
Zielzustand.

### 5.3 Normierte Energie

\[
\sigma = \sqrt{\det F}, \qquad F_n = \frac{F}{\sigma},
\]

\[
E_{\mathrm{dist}}(\mathbf{p}) = \| F_n - I \|_F^2
= \sum_{i,j} \bigl( (F_n)_{ij} - \delta_{ij} \bigr)^2.
\]

**Interpretation (Implementierung):**

| Eigenschaft von \(F\) | \(E_{\mathrm{dist}}\) |
|------------------------|------------------------|
| Reine gleichförmige Skalierung: \(F = s\, I\) | \(0\) |
| Unterschiedliche Skalierung in U/V oder Scherung | \(> 0\) |
| Rotation \(F = R\) (\(\det R = 1\), \(R \neq I\)) | \(> 0\) |

Die Energie bestraft damit Abweichung von **isotroper Skalierung in der
Kanten-Basis**, nicht allgemeine „Winkel-Erhaltung“ in der Ebene.

---

## 6. Strafe für starre Translation

Zur Auswahl unter längen-kompatiblen Lösungen wird die Verschiebung des
Eck-Schwerpunkts bestraft.

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

\[
\mathbf{r}(\mathbf{p}) =
\begin{bmatrix}
\mathbf{r}^{\mathrm{len}} / \tau_L \\
w_a \cdot \mathbf{r}^{\mathrm{ang}} / \tau_a \\
\sqrt{E_{\mathrm{dist}}(\mathbf{p})} \\
\Delta t / \tau_t
\end{bmatrix}.
\]

| Symbol | Wert (Code) | Bedeutung |
|--------|-------------|-----------|
| \(\tau_L\) | `0.01` mm | Längen-Toleranz-Skalierung |
| \(\tau_a\) | \(\sin(1°)\) | Winkel-Toleranz (über Sinus) |
| \(w_a\) | `25.0` | Gewicht der Winkel-Restfehler |
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

Statt `least_squares` wird analytisch skaliert. Der UV-Ursprung \(c_0\) bleibt fix:

\[
\mathbf{c}_k' = \mathbf{c}_0 + s\,(\mathbf{c}_k - \mathbf{c}_0),
\qquad k \in \{0, x, 1, y\}.
\]

Skalenfaktor aus der einzigen Längenbedingung:

\[
s = \frac{L^{\mathrm{soll}}}{\hat{L}\!\left(H(\mathbf{p}_0);\, \ell\right)}.
\]

In diesem Modus ist \(E_{\mathrm{dist}} = 0\) (reine Ähnlichkeit um \(c_0\)).
Es gibt keinen Optimierungs-Residualvektor; `cost = 0`, `success = True`.

---

## 9. Entscheidungslogik (aktueller Solver)

```
compute_calibration_from_specs / compute_calibration_corners
        |
        v
   _solve_corner_calibration
        |
        +- 1 Länge, keine Winkel? -> uniform_scale (analytisch)
        |
        +- sonst -> least_squares auf p in R^8
                      Residuen: Längen + Winkel + sqrt(E_dist) + Delta t
```

Es existieren damit **zwei** Lösungspfade mit unterschiedlicher Parametrisierung;
der allgemeine Pfad erlaubt beliebige Quad-Verformungen (inkl. Scherung).

---

## 10. Typische Constraint-Fälle

| Konfiguration | Erwartetes Verhalten (Stand heute) |
|---------------|-------------------------------------|
| 1 Soll-Länge | `uniform_scale`, \(E_{\mathrm{dist}} = 0\) |
| 1 Länge + Winkel (z. B. horizontal) | `corners`, `least_squares`; Winkel gewichtet; \(E_{\mathrm{dist}}\) und \(\Delta t\) als Tie-Breaker |
| 2 Längen auf U-/V-Kanten | `corners`; anisotrope Skalierung nötig → \(E_{\mathrm{dist}} > 0\) typisch |
| Viele Längen + Winkel (überbestimmt) | Längen- und Winkel-Terme dominieren; \(E_{\mathrm{dist}}\) und \(\Delta t\) sollten das Ergebnis kaum verschieben |

Bei **unterbestimmten** Systemen (wenige Gleichungen, viele Freiheitsgrade)
bestimmen \(\sqrt{E_{\mathrm{dist}}}\) und \(\Delta t\) die Lösung unter den
längen-treuen Alternativen mit.

---

## 11. Anwendung der Lösung (Kurzüberblick)

Nach dem Solve:

1. Neue Ecken \(\mathbf{c}_0', \ldots, \mathbf{c}_y'\) setzen und `WarpMatrix`
   aus \(H\) ableiten.
2. Gespeicherte UV aller abhängigen Objekte mit \(H'\) zurück in die Welt
   projizieren (`apply_corner_calibration`).
3. Sketch-Linien aus kalibrierten UV neu schreiben; Sketch-Placement unverändert.

---

## 12. Implementierungsreferenzen

| Konzept | Funktion | Datei |
|---------|----------|-------|
| Homographie aus Ecken | `compute_homography`, `_homography_from_corners` | `image_homography.py` |
| Länge in UV | `_line_length_uv` | `image_homography.py` |
| Richtung / Parallel-Sinus | `_direction_xy_from_uv_line`, `_parallel_sin_xy` | `image_homography.py` |
| Verzerrungsenergie | `_distortion_energy` | `image_constraint_solver.py` |
| Residualvektor | `_calibration_residuals` | `image_constraint_solver.py` |
| Uniform-Scale | `_solve_uniform_scale_calibration` | `image_constraint_solver.py` |
| Allgemeiner Solve | `_solve_corner_calibration` | `image_constraint_solver.py` |
| Einstieg (Tests/UI) | `compute_calibration_from_specs` | `image_constraint_solver.py` |

---

## 13. Hinweise / offene Punkte

1. **Zwei Solver-Pfade:** `uniform_scale` und `corners` verwenden unterschiedliche
   Parametrisierungen; eine einheitliche niedrigdimensionale Parametrisierung
   (z. B. Skala/Rotation ohne Scherung) ist derzeit nicht implementiert.

2. **\(E_{\mathrm{dist}}\) vs. Winkel-Erhaltung:** \(E_{\mathrm{dist}} = 0\) bedeutet
   isotrope Skalierung in der Kanten-Basis, nicht allgemeine Ähnlichkeitstransformation
   der Ebene (Rotation erzeugt \(E_{\mathrm{dist}} > 0\)).

3. **Sketch-Placement:** Horizontal/Senkrecht beziehen sich auf feste Sketch-Achsen;
   das Bild wird allein über Eckverschiebungen ausgerichtet.

4. **Perspektive:** \(H_{31}, H_{32}\) können von null verschieden sein; die
   Optimierung variiert Ecken so, dass sich die projektive Abbildung ändert — nicht
   nur eine affine Bildtransformation.
