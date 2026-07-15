# GridUAV — Neural Grid Code for UAV 3D Self Localization

## ⚠️ Active Research Repository

This repository is under active development. Code and structure will evolve as the project progresses.

---

## Context

This repository stores the code developed by Tom Schwierz for his master thesis project, exploring a biomimetic, grid cell inspired solution for IMU based 3D self localization in aerial robots. The project was supervised by Oscar Guerrero Rosado and Raimon Bullich Vilarrubias.

---

## Abstract

Traditional UAV localization systems are vulnerable to failure, sabotage, and complex indoor environments. This project explores a biomimetic alternative based on grid coding, a neural mechanism in the entorhinal cortex that supports path integration during navigation. We extend this 2D grid code into a 3D self localization model, combining multiple grid scales and three plane specific attractor sheets (XY, XZ, YZ) following the [Mixed Modular Grid Coding framework (Klukas et al., 2020)](https://journals.plos.org/ploscompbiol/article?id=10.1371/journal.pcbi.1007796). A Crazyflie quadrotor is simulated in Webots, and linear decoders (Ridge Regression and Recursive Least Squares) recover the drone's 3D displacement from the resulting grid cell activity, providing a robust IMU based solution for 3D self localization.

---

## 📁 Repository Structure

```text
GridUAV/
├── Data/                        # Recorded flight data (IMU signals, ground truth position)
│
├── Data_Analysis/
│   ├── Notebooks/               # Analysis notebooks (decoding performance, robustness tests, etc.)
│   └── Results/                 # Figures and result tables generated from the notebooks
│
├── Experiments/
│   ├── config.ini               # Experiment configuration (MMGC gains, noise levels, flight duration, etc.)
│   └── LaunchExp1.bat           # Launch script for running an experiment
│
├── RobotControl/
│   ├── 3DGridModel.py           # MMGC attractor network implementation
│   ├── Navigation.py            # UAV flight control and exploration logic
│   └── NeuralDecoders.py        # Offline Ridge Regression and Online RLS decoders
│
├── Webots/
│   └── SquareBox.wbt            # Webots world file (2x2x2 m cubic environment with Crazyflie UAV)
│
├── .gitignore
├── requirements.txt 			 # Python dependencies
└── README.md
```

---

## 🔽 Setup

### 1. Install Webots

This project uses Webots, an open source 3D robot simulator, to simulate the Crazyflie quadrotor and its environment.

1. Download and install Webots from the [official releases page](https://cyberbotics.com/).
2. Verify the installation by opening Webots and loading a sample world from `Webots/SquareBox.wbt`.

### 2. Set Environment Path Variables

The controller scripts in `RobotControl/` need to be able to locate the Webots installation. Set the following environment variables so Webots can find its Python bindings. Examples:

**Windows**
```bat
setx WEBOTS_HOME "C:\Program Files\Webots"
setx GRIDUAV "C:\Users\...\Repositories\GridUAV"
```

### 3. Create the Conda Environment

```bash
conda create -n griduav python=3.12.7
conda activate griduav
pip install -r requirements.txt
```

`requirements.txt` includes the core scientific stack (NumPy, SciPy, pandas, Matplotlib) along with any packages needed to interface with Webots controllers.

### 4. Run an Experiment

```bash
cd Experiments
./LaunchExp1.bat
```

Adjust `config.ini` to change the MMGC configuration (number of gains, gain step), injected activity noise, or flight duration before launching a new run.

---

## 📬 Contacts

For questions or collaborations, please contact:

📧 Tom Schwierz — Tom.Schwierz@outlook.de
📧 Oscar Guerrero Rosado — guerrerorosado.o@gmail.com
🌐 [oscarguerrerorosado.github.io](https://oscarguerrerorosado.github.io/)
