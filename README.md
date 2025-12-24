# HighSpeedCollisionAvoidanceAirSimRL

## 📌 Overview
High-Speed collision avoidance based on RL in AirSim simulation & Unreal Engine


### Key Features
* **Simulation Environment:** Microsoft AirSim 
* **Algorithm:** PPO (Proximal Policy Optimization) via Stable Baselines 3
* **Observation Space:** Multi-modal fusion of Depth Images (120x120) and Vehicle Kinematics (Linear Velocity)
* **Action Space:** Continuous velocity control

## 🛠️ Prerequisites

The code has been tested on **Windows** with **Python 3.8+**.

### 1. AirSim Setup
* You need a running instance of **Microsoft AirSim**.
* Download the binary from [AirSim Releases](https://github.com/microsoft/AirSim/releases) or build your custom forest environment as described in the paper.
* Ensure your `settings.json` enables API control.


## 📂 File Structure

```text
.
├── config.yml                  # Configuration file for environment settings (Required)
├── drone_env_continuous_1d.py  # Custom Gym Environment wrapping AirSim API
├── ppo_drone_1d.py             # Main training script (PPO)
├── ppo_drone_policy_run1d.py   # Evaluation/Inference script
├── setup_path.py               # Path setup utility
└── README.md                   # Project documentation

### 2. Training

* 'python ppo_drone_1d.py'
* 
