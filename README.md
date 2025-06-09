# PID Controller Visualization

An interactive visualization tool for understanding PID (Proportional-Integral-Derivative) controller behavior. This tool allows you to experiment with different PID parameters and observe their effects on a second-order system response.

## Features

- Real-time visualization of PID controller response
- Interactive sliders for adjusting PID parameters (Kp, Ki, Kd)
- Visual representation of:
  - System output
  - Setpoint
  - Derivative (shown as tangent line)
  - Integral (shown as shaded area)
- Current values display showing:
  - Error
  - Integral value
  - Derivative value
  - P-term, I-term, and D-term contributions
- Play/pause functionality for animation
- Time slider for manual control

## Requirements

- Python 3.x
- NumPy
- Matplotlib

## Installation

1. Clone the repository:
```bash
git clone https://github.com/mikelikesrobots/pid-demo.git
cd pid-demo
```

2. (Optional) Create and activate a virtual environment:
```bash
# On Windows
python -m venv venv
venv\Scripts\activate

# On macOS/Linux
python3 -m venv venv
source venv/bin/activate
```

3. Install the required packages:
```bash
pip install -r requirements.txt
```

## Usage

Run the visualization:
```bash
python pid_demo.py
```

### Controls

- **Kp Slider**: Adjust the proportional gain
- **Ki Slider**: Adjust the integral gain
- **Kd Slider**: Adjust the derivative gain
- **Time Slider**: Manually control the simulation time
- **Play Button**: Start/stop the animation

### Understanding the Visualization

- **Output Line**: Shows the system response
- **Setpoint Line**: Shows the target value (dashed line)
- **Tangent Line**: Shows the derivative at the current point
- **Shaded Area**: Shows the integral accumulation
- **Value Display**: Shows current error, integral, and derivative values

## System Model

The visualization uses a second-order system model:
```
d²x/dt² = -x - 0.1*dx/dt + u
```

This represents a mass-spring-damper system where:
- x is the position
- dx/dt is the velocity
- u is the control input from the PID controller

## PID Controller

The PID controller calculates the control signal as:
```
u = Kp * error + Ki * ∫error dt + Kd * d(error)/dt
```

Where:
- Kp: Proportional gain
- Ki: Integral gain
- Kd: Derivative gain
- error: Difference between setpoint and current value

## License

MIT License
