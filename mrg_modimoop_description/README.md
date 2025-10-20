# MRG MODIMOOP Description

This ROS 2 package defines physical characteristics of MRG's MODIMOOP Half-scale sailbot model.

## Usage

The `*.urdf.xacro` files in the `urdf` directory are converted to proper `*.urdf` files upon building. They are found in the relevant `share` directory.


## Files

`config` - Configuration files for launch scripts

`launch` - Launch files for helpful tools

`models` - Subdirectories for each self-contained unit 

`urdf` - Xacro descriptions for some default combinations

# Acknowledgements

Note that this repository follows ROS 2 + Gazebo best practices as established at this [repository](https://github.com/gazebosim/ros_gz_project_template) which was presented at [ROSCON '22](https://vimeo.com/767127300).
