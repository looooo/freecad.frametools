# Micro-Apps — Architektur-Entwurf (vorgemerkt)

Stand: **Design-Notiz**, noch **nicht umgesetzt**. FrameTools / Frame-and-Beams-Workbench
bleibt vorerst die **Testumgebung** mit klassischer Monolith-Workbench.

**Ziel (später):** Statt einer fest verdrahteten Workbench können Nutzer **Micro-Apps**
in eine **User-Workbench** laden (z. B. nur Image-Pipeline, oder Frame + Image).

Siehe auch: [IMAGE_PIPELINE_SPEC.md](IMAGE_PIPELINE_SPEC.md),
[README.md](README.md).

---

## Idee in einem Satz

**Eine dünne User-Workbench** lädt zur Laufzeit nur registrierte **Micro-Apps**
(je mit Manifest, Commands, optional Objekttypen); **Frame-Workbench** = Dev-Bundle
mit allen Apps voreingestellt.

---

## Micro-App (Konzept)

| Bestandteil | Inhalt |
|-------------|--------|
| `manifest.toml` | id, name, version, `depends`, commands, toolbars, types |
| `core/` | reines Python (Solver, Homographie) — **ohne** FreeCAD-Import |
| `fc/` | FeaturePython, ViewProvider, Commands, Task Panels |
| `register_app()` | Init.py: Typen, Importer |
| `register_gui(wb)` | Commands an `Gui.addCommand`, Toolbar-Einträge |

Command-IDs **namespaced** (z. B. `FT_Image_AlignedImage`), damit Apps nicht kollidieren.

---

## User-Workbench

- Ein Eintrag in `package.xml` (Addon Manager).
- Config z. B. `~/.FreeCAD/.../user_workbench.json` oder Dokument-Meta:

```json
{
  "workbench": "MyShop",
  "microapps": ["frametools.image", "frametools.frame_beam"]
}
```

- `microcore/registry.py`: Apps laden, `depends` topologisch sortieren, registrieren.

---

## Image-Pipeline als Micro-App(s)

Entlang [IMAGE_PIPELINE_SPEC.md](IMAGE_PIPELINE_SPEC.md):

| Modul | FreeCAD? |
|-------|----------|
| `image_pipeline/core/` — Homographie, Solver, Tests | nein |
| `image_pipeline/fc/` — AlignedImage, Calibration, Coin | ja |
| `image.camera` (später) — CameraModel, Wizard | ja, optional |

Stabile API im Core; Commands bleiben dünn.

---

## Migration (wenn es soweit ist)

1. `microcore` + Manifest-Format
2. Solver nach `core/` verschieben, Tests anpassen
3. Image-Commands/Objekte in erste Micro-App
4. Frame/Beam als zweite Micro-App
5. `init_gui.py` → Registry + Dev-Config (alle Apps)
6. Alte Imports als Re-Export-Shims eine Weile behalten

**Kein Big-Bang** — schrittweise aus dem bestehenden `freecad.frametools`-Paket.

---

## Bewusst offen

- Exaktes Config-Format (JSON vs. TOML vs. FreeCAD-Preferences)
- Ein vs. mehrere Addon-Pakete im Addon Manager
- UI zum Zusammenstellen der User-Workbench
- Verhalten bei fehlenden optionalen Dependencies (z. B. OpenCV)

---

## Status

| Item | Status |
|------|--------|
| Dokumentation | diese Datei |
| Implementierung | ** zurückgestellt** |
| Frame-Workbench | unverändert Test-/Dev-Umgebung |
