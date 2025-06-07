[![Python](https://img.shields.io/badge/python-3.10-blue.svg)](https://docs.python.org/3/whatsnew/3.10.html)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-orange.svg)](https://docs.ros.org/en/humble/index.html)
[![IsaacSim](https://img.shields.io/badge/IsaacSim-4.1.0-red.svg)](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html)
[![IsaacLab](https://img.shields.io/badge/IsaacLab-0.3.0-purple.svg)](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html)
[![Linux platform](https://img.shields.io/badge/platform-Ubuntu--22.04-green.svg)](https://releases.ubuntu.com/22.04/)

Welcome to the P2Dingo Repo, here you will find the source code for the autonomous SLAM quadruped first simulated on Isaac Sim with ros2 and then deployed on a Unitree Go2 EDU

## Requirements
1. Nvidia RTX 20 series or newer
2. Minimum 70gb of storage on Ubunutu 22.04
3. Ros2 Humble
4. IsaacSim 4.1
5. IsaacLab 0.3.1 (orbit)

## Installation Guide

**Step I:** Install Ubuntu 22.04 Native Install (docker untested)

**Step I.I:** Install Omniverse: https://developer.nvidia.com/omniverse?sortBy=developer_learning_library%2Fsort%2Ffeatured_in.omniverse%3Adesc%2Ctitle%3Aasc&hitsPerPage=6#section-getting-started

**Step II:** Install Ros2 Humble following this link: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

Add source `/opt/ros/humble/setup.bash` to your `~/.bashrc` to source ros2 in each terminal

**Step III:** Install MiniConda
```
mkdir -p ~/miniconda3
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh
bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3
rm ~/miniconda3/miniconda.sh
source ~/miniconda3/bin/activate
conda init --all
conda config --set auto_activate_base false
```
**Step IV:** Install IsaacLab 0.3.1
```
git clone https://github.com/isaac-sim/IsaacLab.git
cd IsaacLab
git checkout tags/v0.3.1 -b my-v0.3.1
```
Put these in your ~/.bashrc so that it sources on each terminal
```
export ISAACSIM_PATH="${HOME}/.local/share/ov/pkg/isaac-sim-2023.1.1"
export ISAACSIM_PYTHON_EXE="${ISAACSIM_PATH}/python.sh"
```
Add a sym link to connect to isaacsim `ln -s ${ISAACSIM_PATH} _isaac_sim` and run the conda setup
```
./orbit.sh --conda
conda activate orbit
sudo apt install cmake build-essential
```
Correct the rsl-rl dependancy
```
cd ~/IsaacLab/source/extensions/omni.isaac.orbit_tasks
code setup.py
```
Add this line to the extras require: `"rsl-rl": ["rsl-rl@git+https://github.com/leggedrobotics/rsl_rl.git@v2.0.1#egg=rsl-rl"],`
```
./orbit.sh --install
```
You may have to adjust the dependancy of streamsdk if errors appear:
```
code ~/IsaacLab/source/apps/orbit.python.kit
"omni.kit.streamsdk.plugins" = {version = "2.5.1", exact = true}
```
Open `FarmNoMDL.usd` from Isaac/envs in Isaac Sim and export to the same directory as flattened `flatter.usd`. This should Allow the launcher script to attach materials and props to the world.


## User Guide

**Step V:** Execute `./run_sim.sh` (without activated conda orbit env)



## Installation Guide (lightweight version, no imu)

**Step I:** Install Ubuntu 22.04 Native Install (docker untested)

**Step I.I:** Install Omniverse: https://developer.nvidia.com/omniverse?sortBy=developer_learning_library%2Fsort%2Ffeatured_in.omniverse%3Adesc%2Ctitle%3Aasc&hitsPerPage=6#section-getting-started

**Step II:** Install Ros2 Humble following this link: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

Add source `/opt/ros/humble/setup.bash` to your `~/.bashrc` to source ros2 in each terminal

**Step III:** Install MiniConda
```
mkdir -p ~/miniconda3
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh
bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3
rm ~/miniconda3/miniconda.sh
source ~/miniconda3/bin/activate
conda init --all
```

Now every terminal should initialise with (base)

**Step IV:**  Download Isaac Sim 4.2.0 from the Nvidia Docs under 'Download Archive': https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/download.html
```
mkdir ~/isaacsim
cd ~/Downloads
unzip "isaac-sim-standalone%404.2.0-rc.18%2Brelease.16044.3b2ed111.gl.linux-x86_64.release.zip" -d ~/isaacsim
cd ~/isaacsim
./isaac-sim.selector.sh
```

**Step V:** Install Isaac lab 1.4.1 (still in isaac_env)
```
conda create -n isaac_env python=3.10 -y
conda activate isaac_env
conda install -c conda-forge libstdcxx-ng
cd ~
git clone https://github.com/isaac-sim/IsaacLab.git
cd IsaacLab
git checkout tags/v1.4.1 -b v1.4.1
sudo apt update
sudo apt install -y cmake build-essential
```
  This version of IsaacLab depends on a version rsl-rl which has since been renamed. Lets fix this:
```
cd ~/IsaacLab/source/extensions/omni.isaac.lab_tasks
code setup.cfg
```
  Change the rsl-rl dependancy to what is shown below:
```
EXTRAS_REQUIRE = {
    "sb3": ["stable-baselines3>=2.1"],
    "skrl": ["skrl>=1.3.0"],
    "rl-games": ["rl-games==1.6.1", "gym"],  # rl-games still needs gym :(
    "rsl-rl": ["rsl-rl@git+https://github.com/leggedrobotics/rsl_rl.git@v2.0.1#egg=rsl-rl"],
    "robomimic": [],
}
```
create a symlink to the IsaacLab folder: 
```
  cd ~/IsaacLab
  ln -s /home/<your name>/isaacsim _isaac_sim
```
Now run the installation script
```
  cd ~/IsaacLab
  ./isaaclab.sh --install
```

**Step VI:**  Configure isaac_env to to recognise the Isaac Sim python libraries. Add the following at the end of your `~/.bashrc` file so that it automatically initialises in each terminal:
```
code ~/.bashrc
```
```
# Point at the tarball’s top‐level:
export ISAAC_PATH="$HOME/isaacsim"
# Tell SimulationApp where ‘apps’ lives:
export EXP_PATH="$ISAAC_PATH/apps"
# Tell Kit where its binary folder is:
export CARB_APP_PATH="$ISAAC_PATH/kit"
# When isaac_env is activated, add IsaacSim’s Python & library paths so that
# “import isaacsim” works without having to drop into a REPL.
ISAAC_PATH="$HOME/isaacsim"
# Append IsaacSim’s Kit and plugin libraries to LD_LIBRARY_PATH:
export LD_LIBRARY_PATH="$ISAAC_PATH/kit:$ISAAC_PATH/exts/omni.usd.schema.isaac/plugins/IsaacSensorSchema/lib:$ISAAC_PATH/exts/omni.usd.schema.isaac/plugins/RangeSensorSchema/lib:$ISAAC_PATH/exts/omni.isaac.lula/pip_prebundle:$ISAAC_PATH/exts/omni.exporter.urdf/pip_prebundle:$ISAAC_PATH/kit:$ISAAC_PATH/kit/kernel/plugins:$ISAAC_PATH/kit/libs/iray:$ISAAC_PATH/kit/plugins:$ISAAC_PATH/kit/plugins/bindings-python:$ISAAC_PATH/kit/plugins/carb_gfx:$ISAAC_PATH/kit/plugins/rtx:$ISAAC_PATH/kit/plugins/gpu.foundation:$LD_LIBRARY_PATH"
# Prepend IsaacSim’s Python site-packages and extension “pip_prebundle” folders so that
# “import isaacsim” and “import omni.isaac.*” work immediately:
export PYTHONPATH="$ISAAC_PATH/kit/python/lib/python3.10/site-packages:$ISAAC_PATH/python_packages:$ISAAC_PATH/exts/omni.isaac.kit:$ISAAC_PATH/kit/kernel/py:$ISAAC_PATH/kit/plugins/bindings-python:$ISAAC_PATH/exts/omni.isaac.lula/pip_prebundle:$ISAAC_PATH/exts/omni.exporter.urdf/pip_prebundle:$ISAAC_PATH/extscache/omni.kit.pip_archive-*/pip_prebundle:$ISAAC_PATH/exts/omni.isaac.core_archive/pip_prebundle:$ISAAC_PATH/exts/omni.isaac.ml_archive/pip_prebundle:$ISAAC_PATH/exts/omni.pip.compute/pip_prebundle:$ISAAC_PATH/exts/omni.pip.cloud/pip_prebundle:$PYTHONPATH"
```

**Step VII:**  Colcon build and related ros2 functions cannot access their libraries from within the isaac_env conda environment without running into errors. We will separate the conda initialisation such that any terminal created within the go2_ws file tree will open a fresh system terminal instead. Modify .bashrc with an if block:
```
# Only source ROS 2 Humble and activate Conda if $PWD is NOT under ~/P2Dingo/Isaac/go2_ws
if [[ ! "$PWD" == "$HOME/P2Dingo/Isaac/go2_ws"* ]]; then
    # 1) Source ROS 2 Humble so that 'ros2' and related tools work
    # 3) Initialize Conda (this block is autogenerated by 'conda init')
    __conda_setup="$('/home/lukas/miniconda3/bin/conda' 'shell.bash' 'hook' 2> /dev/null)"
    if [ $? -eq 0 ]; then
        eval "$__conda_setup"
    else
        if [ -f "/home/lukas/miniconda3/etc/profile.d/conda.sh" ]; then
            . "/home/lukas/miniconda3/etc/profile.d/conda.sh"
        else
            export PATH="/home/lukas/miniconda3/bin:$PATH"
        fi
    fi
    unset __conda_setup
    # <<< conda initialize <<<
    # 4) Activate your 'isaac_env' environment
    conda activate isaac_env
fi
```

**Step VIII:**  Connect to the system-wide Ros2 by adding these the /.bashrc above the if statement
```
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
source /home/lukas/P2Dingo/Isaac/go2_ws/install/setup.bash
```

**Step IX:**  If you have integrated graphics, force the dedicated gpu pipeline to handle applications
```
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __VK_LAYER_NV_optimus=1
export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json
```

## User guide

**Step I:** Open a fresh terminal in `~/P2Dingo/Isaac/go2_ws`
```
colcon build --symlink-install
```

**Step II:** Launch the Simulator
Open a terminal (anywhere outside go2_ws)
```
cd ~/P2Dingo/Isaac/isaac-go2-ros2
python isaac_go2_ros2.py
```

**Step III:** In the first terminal, do
```
ros2 run go2_control walk_forward
```

**Step IV:** To visualise in Rviz2, In another terminal run (with the correct path)
```
rviz2 -d /home/lukas/P2Dingo/Isaac/isaac-go2-ros2/rviz/go2.rviz
```


## Ros2 Topics

**Command and Control**  
- `/unitree_go2/cmd_vel`:  Topic to send velocity commands to the robot for motion control.

**Front Camera**  
- `/unitree_go2/front_cam/color_image`: Publishes RGB color images captured by the front camera.
- `/unitree_go2/front_cam/depth_image`: Publishes depth images from the front camera.
- `unitree_go2/front_cam/semantic_segmentation_image`: Publishes semantic segmentation images from the front camera.
- `/unitree_go2/front_cam/info`: Publishes camera information, including intrinsic parameters.

**LIDAR**  
- `/unitree_go2/lidar/point_cloud`:  Publishes a point cloud generated by the robot's LIDAR sensor.

**Odometry and Localization**  
- `/unitree_go2/odom`:  Publishes odometry data, including the robot's position, orientation, and velocity.
- `/unitree_go2/pose`:  Publishes the current pose of the robot in the world frame.
