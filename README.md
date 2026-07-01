# 🚗 Self-Driving Car in Webots (Python)

A self-driving car simulation developed using **Webots** with a **Python-based controller**.  
The vehicle autonomously follows road lanes using a **PID controller** while detecting and avoiding obstacles using **LiDAR**.  

The system integrates multiple sensors to achieve reliable autonomous navigation in a simulated urban environment.

---

## Demo

> Add a GIF or video of the simulation here.

![Demo](docs/demo.gif)

---

## Features

- Autonomous lane following (Python controller)
- PID-based steering control (Python implementation)
- Camera-based lane detection
- LiDAR obstacle detection and avoidance
- Real-time steering adjustment
- GPS-based speed estimation
- Manual driving mode (keyboard)
- Automatic driving mode
- Steering smoothing and filtering

---

## Sensors

| Sensor | Purpose |
|--------|--------|
| Camera | Lane detection |
| Sick LMS 291 LiDAR | Obstacle detection |
| GPS | Speed estimation |
| Keyboard | Manual control |

---

## Control Architecture

```
Camera
  │
  ▼
Lane Detection (Python)
  │
  ▼
PID Controller (Python)
  │
  ▼
Steering Command
  ▲
  │
LiDAR ──► Obstacle Detection ──► Obstacle Avoidance (Python)
  │
  ▼
Final Steering Output

GPS ─────► Speed Monitoring (Python)
```

---

## PID Controller

The steering system is implemented in Python using a classic PID controller.

```
Steering = KP * Error
         + KI * Integral
         + KD * Derivative
```

### Gains

```python
KP = 0.25
KI = 0.006
KD = 2
```

### Features

- Integral anti-windup
- Error smoothing
- Steering stability filtering

---

## Lane Detection

Implemented in Python:

- Camera image processing
- Detection of lane color (yellow marking)
- Computation of lane center
- Conversion into steering angle

---

## Obstacle Avoidance

Implemented in Python using LiDAR data:

1. Read distance values from LiDAR
2. Detect obstacle position and angle
3. Estimate obstacle distance
4. Generate avoidance steering
5. Combine with PID output

---

## Project Structure

```
Self-Driving-Car-Webots/
│
├── controllers/
│   ├── my_ai_driver/
│   │     └── my_ai_driver.py   # Python controller
│
├── plugins/
├── worlds/
│   ├── city.wbt
│   └── city_net/
│
└── README.md
```

---

## Requirements

- Webots Simulator
- Python 3.x

---

## Running the Project

Clone the repository:

```bash
git clone https://github.com/NadeemMansour/Self-Driving-Car-Webots.git
```

Open the simulation:

```
worlds/city.wbt
```

Run Webots and start the controller.

---

## Controls

| Key | Action |
|-----|--------|
| ↑ | Increase speed |
| ↓ | Decrease speed |
| ← | Turn left |
| → | Turn right |
| A | Toggle Auto Drive |

---

## Future Improvements

- Deep learning-based lane detection (Python)
- Traffic sign recognition
- Traffic light detection
- Adaptive cruise control (ACC)
- Automatic emergency braking (AEB)
- Path planning (A*, RRT)
- Sensor fusion
- SLAM
- Reinforcement learning controller

---

## Author

**Nadeem Mansour**

Computer Engineering Student

GitHub: https://github.com/NadeemMansour

---

## License

This project is for educational and research purposes.
