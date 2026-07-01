# 🚗 Self-Driving Car in Webots

A self-driving car simulation developed using **Webots** and **Python**. The vehicle autonomously follows road lanes using a PID controller while detecting and avoiding obstacles with LiDAR. The project integrates multiple sensors to achieve reliable autonomous navigation in a simulated urban environment.

---

## Demo

> Add a GIF or video of the simulation here.

![Demo](docs/demo.gif)

---

## Features

- Autonomous lane following
- PID-based steering control
- Camera-based lane detection
- LiDAR obstacle detection
- Real-time obstacle avoidance
- GPS speed estimation
- Manual driving mode
- Automatic driving mode
- Smooth steering control
- Steering angle filtering

---

## Sensors

| Sensor | Purpose |
|---------|----------|
| Camera | Detect lane markings |
| Sick LMS 291 LiDAR | Detect obstacles in front of the vehicle |
| GPS | Measure vehicle speed |
| Keyboard | Manual driving |

---

## Control Architecture

```
                    Camera
                       │
                       ▼
              Lane Detection
                       │
                       ▼
                 PID Controller
                       │
                       ▼
                 Steering Command
                       ▲
                       │
LiDAR ──► Obstacle Detection ──► Obstacle Avoidance
                       │
                       ▼
                 Final Steering

GPS ─────────► Speed Monitoring
```

---

## PID Controller

The steering controller is based on a classic PID controller.

```
Steering = KP × Error
          + KI × Integral
          + KD × Derivative
```

Current gains:

```python
KP = 0.25
KI = 0.006
KD = 2
```

The controller includes:

- Integral reset
- Anti-windup
- Steering smoothing
- Angle filtering

---

## Lane Detection

The lane is detected by:

- Capturing images from the front camera
- Detecting the yellow lane marking
- Computing the lane center
- Converting the center into a steering angle

---

## Obstacle Avoidance

The LiDAR continuously scans the road ahead.

When an obstacle is detected:

1. Estimate its angle.
2. Estimate its distance.
3. Generate an avoidance steering angle.
4. Combine it with the PID steering command.

This allows the vehicle to avoid collisions while attempting to remain inside the lane.

---

## Project Structure

```
Self-Driving-Car-Webots
│
├── controllers
│   ├── my_ai_driver
│   │     └── my_ai_driver.py
│   ├── crossroads_traffic_lights
│   └── generic_traffic_light
│
├── plugins
│
├── worlds
│   ├── city.wbt
│   └── city_net
│
└── README.md
```

---

## Requirements

- Webots
- Python 3.x

---

## Running

Clone the repository

```bash
git clone https://github.com/NadeemMansour/Self-Driving-Car-Webots.git
```

Open

```
worlds/city.wbt
```

Run the simulation.

---

## Controls

| Key | Action |
|------|--------|
| ↑ | Increase speed |
| ↓ | Decrease speed |
| ← | Turn left |
| → | Turn right |
| A | Enable Auto Drive |

---

## Future Improvements

- Deep Learning lane detection
- Traffic sign recognition
- Traffic light recognition
- Adaptive Cruise Control (ACC)
- Automatic Emergency Braking (AEB)
- Lane Change Planning
- Path Planning (A*, RRT)
- Sensor Fusion
- SLAM
- Reinforcement Learning controller

---

## Author

**Nadeem Mansour**

Computer Engineering Student

GitHub:
https://github.com/NadeemMansour

---

## License

This project is released for educational and research purposes.
