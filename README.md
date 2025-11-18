## MRG Modimoop Sim Package

This ROS2 package includes the description and bringup for the MRG MODIMOOP SailBot.

## Usage

This package is to be used in tandem with ASDL's Lyoko ROS2 package for Gazebo integration, and the open source package asv_wave_sim.

The file structure should be:

workspace_name_ws/src
----> mrg_modimoop
----> asv_wave_sim
----> lyoko


## Setup

To set up the environment, first use Git and GH (for enterprise GitHub authentication) to clone the necessary repos

```bash
sudo apt update
sudo apt install gh
```

Next, to set up asv wave sim, run the following commands:

```bash
sudo apt-get update
sudo apt-get install libcgal-dev libfftw3-dev
```

and then

```bash
export GZ_SIM_RESOURCE_PATH=\
$GZ_SIM_RESOURCE_PATH:\
$HOME/gz_ws/src/asv_wave_sim/gz-waves-models/models:\
$HOME/gz_ws/src/asv_wave_sim/gz-waves-models/world_models:\
$HOME/gz_ws/src/asv_wave_sim/gz-waves-models/worlds

export GZ_SIM_SYSTEM_PLUGIN_PATH=\
$GZ_SIM_SYSTEM_PLUGIN_PATH:\
$HOME/gz_ws/install/lib

export GZ_GUI_PLUGIN_PATH=\
$GZ_GUI_PLUGIN_PATH:\
$HOME/gz_ws/src/asv_wave_sim/gz-waves/src/gui/plugins/waves_control/build
```

replacing gz_ws with your workspace name.

## Building

To build the ws, in the ws root directory, run

```bash
colcon build --merge-install
source install/setup.bash
```