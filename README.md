## MRG Modimoop Sim Package

This ROS2 package includes the description and bringup for the MRG MODIMOOP SailBot.

## Usage

This package is intended to allow for simulation of the MRG MODIMOOP SailBot platform.


## Setup

<details>
  <summary>MacOS</summary>
  
  For MacOS, start with installing Docker Desktop (see instructions [here](https://docs.docker.com/desktop/setup/install/mac-install/)). Whenever you are developing this project, you should keep the Docker Desktop running in the background.
  1. Install [uv](https://docs.astral.sh/uv/getting-started/installation/):
```
curl -LsSf https://astral.sh/uv/install.sh | sh 
```
  2. Install qix:
```
uv tool install git+https://github.gatech.edu/ASDL-Robotics/qix.git
```
Note that Qix is in the ASDL-Robotics GitHub organization. Therefore, before working on this, you must request access to that organization using your Georgia Tech [GitHub Enterprise](https://github.gatech.edu/) account. You may have to set up authentication for your GitHub account. See this [page](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent?platform=mac) for more information about SSH key settup.

  3. Clone this repo somewhere onto your machine in an easy to navigate location. From a terminal, navigate to the folder above ```mrg_modimoop/``` and run:
```
qix stack install mrg_modimoop --novnc
```
</details>
<details>
  <summary>Windows</summary>

  For Windows, please install and set up WSL2 with Docker:
  1. Navigate to PowerShell and run:
```wsl --install```
  2. Then, launch Ubuntu by running ```wsl``` in PowerShell. Ensure the Docker engine is running, either inside WSL2 or with Docker Desktop with [WSL2 integration](https://docs.docker.com/desktop/features/wsl/)
  3. Finally, following the remaining guide in the Ubuntu setup below.
</details>
<details>
  <summary>Ubuntu / Linux</summary>

  1. Ensure Docker is installed and your user has permission to run Docker without sudo:
```sudo usermod -aG docker $USER && newgrp docker```
  2. Install uv and Qix:
```
curl -LsSf https://astral.sh/uv/install.sh | sh
uv tool install git+https://github.gatech.edu/ASDL-Robotics/qix.git
```
  3. Clone this ```mrg_modimoop``` repository:
```
git clone git@github.com:gt-marine-robotics-group/mrg_modimoop.git
```
  4. From one folder above ```mrg_modimoop```, install the Qix stack:
```
qix stack install mrg_modimoop --novnc
```
  
# Note 1:
If you have an Nvidia GPU, you can use the ```--gpu``` to enable GPU acceleration.
# Note 2:
If this is your first time setting up WSL or using your GitHub enterprise account, you will have to authenticate both your personal GitHub account (the account you are using to develop repositories in the gt-marine-robotics-group namespace) and your Georgia Tech GitHub Enterprise account (the account you need to use to develop and use tools in the ASDL-Robotics namespace). Please see this [page](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent?platform=linux) for more information in setting up SSH keys (I recommend setting these up and using them to pull, clone and push.)
</details>

## Running the Sim

To start the sim,
   1. Navigate in a terminal to one folder above the ```mrg_modimoop``` folder and run
```
qix stack enter mrg_modimoop
```
   2. Navigate to ```$HOME/colcon_ws/``` and run:
```bash
export GZ_SIM_RESOURCE_PATH=\
$GZ_SIM_RESOURCE_PATH:\
$HOME/colcon_ws/src/asv_wave_sim/gz-waves-models/models:\
$HOME/colcon_ws/src/asv_wave_sim/gz-waves-models/world_models:\
$HOME/colcon_ws/src/asv_wave_sim/gz-waves-models/worlds

export GZ_SIM_SYSTEM_PLUGIN_PATH=\
$GZ_SIM_SYSTEM_PLUGIN_PATH:\
$HOME/colcon_ws/install/lib

export GZ_GUI_PLUGIN_PATH=\
$GZ_GUI_PLUGIN_PATH:\
$HOME/colcon_ws/src/asv_wave_sim/gz-waves/src/gui/plugins/waves_control/build

export GZ_SIM_RESOURCE_PATH=\
$GZ_SIM_RESOURCE_PATH:\
$HOME/colcon_ws/src/mrg_modimoop/mrg_modimoop_description/world_models

export GZ_VERSION=harmonic

source /opt/ros/jazzy/setup.bash
```

   3. From the ```.../colcon_ws/``` directory, build and source:
```
colcon build --merge-install
source install/setup.bash
```

   4. Run the Sim and launch the Controls package in two different terminals or tmux:
```
ros2 launch mrg_modimoop_bringup sim.launch.py
ros2 launch mrg_modimoop_control control.launch.py
```
