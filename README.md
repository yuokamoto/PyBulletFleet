

# PyBullet Multi-Robot Simulation

This is a sample project for multi-robot and high-speed simulation using PyBullet.

## Main Features
- Simultaneous simulation of multiple robots (URDF-based)
- Pallet transport, cooperative operation, collision detection
- High-speed simulation (10x or more, physics disable supported)
- External data monitor (tkinter/console supported)
- URDF caching, batch generation, rendering control

## Setup
1. Install Python 3.8+
2. (Recommended) Create virtual environment: `python -m venv venv && source venv/bin/activate`
3. Install dependencies: `pip install -r requirements.txt`

## Example Usage
Update config/config.yaml as needed
```bash
python example/100robots_demo.py
```