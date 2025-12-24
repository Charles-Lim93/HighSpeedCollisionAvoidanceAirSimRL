# HighSpeedCollisionAvoidanceAirSimRL

## 📌 Overview
High-Speed collision avoidance based on RL in AirSim simulation & Unreal Engine

This repository contains the implementation of the reinforcement learning framework presented at the **AIAA SciTech 2025 Forum** [1]. An extended version of this work has been submitted to the *Journal of Aerospace Information Systems (JAIS)*.

> **Paper Link:** [AIAA SciTech 2025 Paper](https://arc.aiaa.org/doi/abs/10.2514/6.2025-2275)

### Key Features
* **Simulation Environment:** Microsoft AirSim 
* **Algorithm:** PPO (Proximal Policy Optimization) via Stable Baselines 3
* **Observation Space:** Multi-modal fusion of Depth Images (120x120) and Vehicle Kinematics (Linear Velocity)
* **Action Space:** Continuous velocity control

## 🛠️ Prerequisites
w
The code has been tested on **Windows 10** with **Python 3.8+**.

### 1. AirSim Setup
* You need a running instance of **Microsoft AirSim**.
* Download the binary from [AirSim Releases](https://github.com/microsoft/AirSim/releases) or build your custom forest environment as described in the paper.
* Ensure your `settings.json` enables API control.

### 2. Running
* 'python ppo_drone_1d.py' for data augmented framework.
* 'python ppo_drone_3d.py' for raw 3 vanilla time series RL framework.
* 'python ppo_drone_base.py' for nominal single-image RL framework.

### 3. Evaluation
* 'python ppo_drone_policy_run1d.py' for the data augmented framework.
* 'python ppo_drone_policy_run3d.py' for raw 3 vanilla time series RL framework.
* 'python ppo_drone_policy_run_based.py' for nominal single-image RL framework.



## 📂 File Structure

```text
.
├── config.yml                                # Configuration file for environment settings (Required)
├── ./airgym/envs/drone_env_continuous_1d.py  # Custom Gym Environment wrapping AirSim API
├── ppo_drone_1d.py                           # Main training script (PPO)
├── ppo_drone_policy_run1d.py                 # Evaluation/Inference script
├── setup_path.py                             # Path setup utility
└── README.md                                 # Project documentation


