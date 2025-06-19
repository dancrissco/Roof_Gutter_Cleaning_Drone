# 🛠️ Roof Gutter Cleaning Drone Simulator

This project simulates an autonomous drone performing gutter cleaning over a round barn-style house using PyBullet. The simulation showcases drone takeoff, circular flight along the gutter perimeter, and return to start. A virtual camera is mounted for future computer vision tasks such as gutter following and dirt detection.

---

## 📂 Project Structure

```
.
├── quad_monobody.urdf           # Drone model with monocoque body and visual propellers
├── assets/
│   ├── house_model.urdf         # Assembled house URDF with platform, walls, roof, gutter
│   ├── part_1.stl               # Platform mesh
│   ├── part_1__2.stl            # Wall mesh
│   ├── part_1__3.stl            # Roof mesh
│   ├── part_1__4.stl            # Gutter mesh
├── autonomous_gutter_clean_camview.py  # Full PyBullet animation + live top camera view
```

---

## 🚁 Simulation Features

- [x] Modular round barn house model
- [x] Drone with integrated body and propellers
- [x] Circular flight path above roof perimeter
- [x] Smooth takeoff, path traversal, return, and landing
- [x] Live top-down drone-mounted camera view (OpenCV)
- [ ] (Planned) Dirty gutter detection using visual AI
- [ ] (Planned) Water spray mist effect + refill logic

---

## 🧱 House Color Scheme

| Part      | Material         | Color (RGBA)        |
|-----------|------------------|---------------------|
| Platform  | Concrete         | `0.7 0.7 0.7 1`     |
| Walls     | Reddish Brown    | `0.5 0.2 0.1 1`     |
| Roof      | Asphalt Tile     | `0.2 0.2 0.2 1`     |
| Gutter    | White Metal      | `0.95 0.95 0.95 1`  |

These are set in `house_model.urdf` using `<material>` blocks.

---

## 🖥️ Running the Simulation

### Requirements
- Python 3.10+
- PyBullet
- OpenCV (for live camera preview)

### Installation
```bash
pip install pybullet opencv-python
```

### Run the Simulation
```bash
python3 autonomous_gutter_clean_camview.py
```

The GUI will show the full scene, and an OpenCV window will pop up showing the drone's top-down camera feed.

---

## 📌 Notes
- STL files are assumed to be exported in millimeters and scaled with `0.001` in URDFs
- The drone camera is positioned above the body to provide a wide field of view
- Camera images are not saved, but can be extended with OpenCV for saving or processing

---

## 📸 Future Goals
- Add real misting spray particles
- Use virtual camera input for detecting gutter position or debris
- Enable vision-based path planning using Qwen-VL or CLIP

---

## 📧 Acknowledgments
Developed interactively using OpenAI ChatGPT (GPT-4), with user-contributed assets and custom modeling in OnShape.

Enjoy flying and cleaning! 🧽🚁🏠
