# robot-nav

A Python robotics project for a GPIO-controlled mobile robot that can follow lines, detect intersections, estimate turns, build a grid map, save/load maps, plan shortest paths with Dijkstra's algorithm, and test ultrasonic proximity behaviors.

The code is organized as a small layered robot stack:

- **Hardware layer**: motors, IR line sensors, ADC, gyro, and ultrasonic sensors.
- **Behavior layer**: line following, intersection detection, end-of-line handling, pull-forward, and calibrated turning.
- **Tracking layer**: discrete 8-direction grid map, street status tracking, map save/load, visualization, and Dijkstra planning.
- **Brain layer**: interactive operator commands for manual driving, exploration, map saving/loading, and autonomous `goto` navigation.

## Repository Structure

```text
robot_nav/
├── brain.py              # Main operator-facing robot brain and command loop
├── behaviors.py          # High-level robot behaviors: line following, turns, pull-forward
├── tracking.py           # Grid map, pose tracking, Dijkstra path planning, visualization
├── drivesystem.py        # Motor control and drive action table
├── sensors.py            # IR line sensors, ADC, and gyroscope wrapper
├── proximitysensor.py    # Ultrasonic distance sensor interface
├── test_ultrasound.py    # Ultrasound, avoidance, and wall-following test modes
├── test_ADC.py           # ADC readout test script
├── calibrate_imu.py      # Gyro/ADC calibration helper
├── detectors.py          # Low-pass binary detectors and side estimator
└── constants.py          # GPIO pins, thresholds, calibration constants, filenames
```

## Main Features

### Line Following

The robot reads three IR sensors and maps sensor patterns to drive actions such as straight, hook left/right, and turn left/right. The behavior layer also includes filtered detectors for intersections and end-of-street events.

### Turn Estimation

Turns are estimated using a combination of:

1. a gyro-derived integrated angle, and
2. a calibrated turn-time table.

The final turn estimate is snapped to one of the legal grid angles: 45°, 90°, 135°, 180°, 270°, or 360°.

### Grid Mapping

The robot stores intersections on a discrete 2D grid. Each intersection tracks the state of outgoing streets in eight compass directions:

- `UNKNOWN`
- `NONEXISTENT`
- `UNEXPLORED`
- `DEADEND`
- `CONNECTED`

Maps can be saved and loaded using Python pickle files. The default filename is configured in `constants.py` as:

```python
map_filename = "mymap.pickle"
```

### Dijkstra Path Planning

After a map is built or loaded, the robot can plan a shortest path to a target grid coordinate using Dijkstra's algorithm. The `goto` command computes the route and the brain chooses the next turn/straight action toward the planned path.

### Ultrasonic Proximity Behaviors

The project supports three ultrasonic sensors: left, middle, and right. `test_ultrasound.py` includes modes for:

- live distance reading,
- herding-style obstacle avoidance,
- wall-following PWM tests,
- interactive testing.

## Hardware Requirements

This project is intended for a Raspberry Pi-style GPIO platform using `pigpio`.

Expected hardware includes:

- two DC motors driven by an H-bridge,
- three IR line sensors,
- one 8-bit ADC connected to the gyro,
- one analog gyroscope,
- three ultrasonic distance sensors,
- a robot chassis capable of differential drive.

The default GPIO pin assignments are in `constants.py`, `sensors.py`, and `drivesystem.py`. Check and update them before running on different hardware.

## Software Requirements

- Python 3
- `pigpio`
- optional: `matplotlib` for calibration plots

Install Python dependencies:

```bash
pip install pigpio matplotlib
```

On Raspberry Pi, install and start the pigpio daemon:

```bash
sudo apt update
sudo apt install pigpio python3-pigpio
sudo systemctl enable pigpiod
sudo systemctl start pigpiod
```

Check that the daemon is running:

```bash
sudo systemctl status pigpiod
```

## Quick Start

Clone the repository:

```bash
git clone <your-repo-url>
cd <your-repo-name>/W6
```

Start the main robot brain:

```bash
python3 brain.py
```

The program will ask whether to load a saved map. If no map is loaded, the robot will first follow the line until it reaches the first intersection and then ask for the starting map pose.

Example pose input:

```text
0 0 north
```

## Operator Commands

Inside `brain.py`, use the interactive prompt:

```text
Command [s]traight/[l]eft/[r]ight/[e]xplore/[g]oto/[w]rite/load/[q]uit:
```

Available commands:

| Command | Meaning |
|---|---|
| `s`, `straight`, or Enter | Follow the current street until an intersection or end-of-line |
| `l`, `left` | Turn left and update heading estimate |
| `r`, `right` | Turn right and update heading estimate |
| `e`, `explore`, `auto` | Automatically explore unfinished streets |
| `g`, `goto` | Plan a route to a target grid coordinate |
| `w`, `write`, `save` | Save the current map |
| `load`, `open` | Load a saved map |
| `q`, `quit`, `exit` | Stop and exit |

## Running Test Scripts

### Test ADC Readings

```bash
python3 test_ADC.py
```

This continuously prints the 8-bit ADC value.

### Calibrate Gyro / IMU Turn Reading

```bash
python3 calibrate_imu.py
```

This runs a turn calibration routine and can save a plot named:

```text
turn_adc_readings.png
```

### Test Ultrasonic Sensors

Interactive mode:

```bash
python3 test_ultrasound.py
```

Read-only mode:

```bash
python3 test_ultrasound.py read
```

Read for a fixed duration and save CSV data:

```bash
python3 test_ultrasound.py read --duration 10 --csv ultrasound_log.csv
```

Herding / obstacle-avoidance mode:

```bash
python3 test_ultrasound.py herd --duration 20
```

Wall-following PWM mode:

```bash
python3 test_ultrasound.py wall-pwm --side left --duration 20
```

## Tuning

Most robot-specific tuning values are centralized in `constants.py` and `drivesystem.py`.

Important parameters include:

- IR sensor GPIO pins,
- ultrasonic trigger/echo pins,
- PWM maximum and frequency,
- line detector time constants,
- side estimator thresholds,
- pull-forward duration,
- turn classification angles,
- turn-time calibration table,
- wall-following thresholds and PWM gains.

Motor action levels are defined in `DriveSystem.action_levels` inside `drivesystem.py`:

```python
DriveAction.STRAIGHT: (109, 111)
DriveAction.SPIN_LEFT: (-92, 95)
DriveAction.SPIN_RIGHT: (92, -95)
```

Tune these values slowly and test each motion on the floor before running autonomous behavior.

## Safety Notes

- Put the robot on blocks before first motor tests.
- Keep one hand near the power switch during calibration.
- Confirm motor direction before running `brain.py`.
- Confirm IR sensor polarity: this code assumes `BLACK = 1` and `WHITE = 0`.
- Stop the program with `Ctrl+C`; the behavior layer attempts to brake/stop motors during shutdown.

## Development Notes

This project currently uses direct Python modules rather than a package structure. Run scripts from inside the `W6/` directory so local imports resolve correctly:

```bash
cd W6
python3 brain.py
```

Generated files such as `__pycache__/`, `.pickle` map files, CSV logs, and calibration plots should usually be excluded from Git.

Suggested `.gitignore`:

```gitignore
__pycache__/
*.pyc
*.pyo
*.pickle
*.csv
*.png
.env
.venv/
```

## Suggested GitHub Description

> GPIO-based mobile robot navigation stack with line following, intersection mapping, turn estimation, ultrasonic sensing, autonomous exploration, and Dijkstra path planning.

## License

Add a license before publishing the repository. For coursework or research code, common choices are MIT, BSD-3-Clause, or Apache-2.0.
