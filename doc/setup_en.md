# Installation and G1 build

Training and ROS-facing deployment use different Python environments. Keep them
separate: the training example uses Python 3.11; ROS Humble Python modules require
Python 3.10. Existing compatible environments can be reused.

## Training environment

On the supported Ubuntu/NVIDIA setup, create and activate a Conda environment:

```bash
conda create -n unitree_rl_mjlab python=3.11
conda activate unitree_rl_mjlab
```

Use this checkout's `dev` branch for the G1 online workflow:

```bash
git clone --branch dev https://github.com/mmurooka/unitree_rl_mjlab.git
cd unitree_rl_mjlab
python -m pip install -e .
```

`setup.py` pins mjlab, MuJoCo, mujoco-warp and Warp. See the
[README](../README.md#1-train) for training/play commands.

## C++ simulator and G1 controller

Install the build dependencies and the official
[unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) / CycloneDDS
installation used by your machine:

```bash
sudo apt install build-essential cmake libyaml-cpp-dev libboost-all-dev \
  libeigen3-dev libspdlog-dev libfmt-dev libzmq3-dev libglfw3-dev
```

From this repository root, with ROS Humble installed:

```bash
source /opt/ros/humble/setup.bash
cmake -S simulate -B simulate/build
cmake --build simulate/build -j2
cmake -S deploy/robots/g1 -B deploy/robots/g1/build \
  -DG1_NAVIGATION_WITH_ROS2=ON
cmake --build deploy/robots/g1/build -j2
```

Check the simulator configure output for ROS sensor support. Camera recognition
needs those image/CameraInfo publishers even in GLIM-free mode. GLIM requires a
ROS-enabled controller; `G1_NAVIGATION_WITH_ROS2=OFF` is only appropriate when
that controller will use simulator localization. This flag does not configure
simulator sensor publishing or other ROS processes.

## Runtime Python and sensors

For the shared sender, recognition and viewer environment, follow
[TaskPromptRL's online setup](https://github.com/isri-aist/TaskPromptRL/blob/main/doc/online_motion_prompt_quickstart.md).
Recognition needs NumPy, PyYAML, OpenCV with ArUco, ROS Python and the official
[unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python).
If your CycloneDDS installation is under `/usr/local`, its build/runtime lookup
may require:

```bash
export CYCLONEDDS_HOME=/usr/local
export CMAKE_PREFIX_PATH=/usr/local:${CMAKE_PREFIX_PATH}
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}
```

GLIM and the real Livox driver are external installations. The repository reads
an explicitly supplied GLIM configuration; it does not install or start GLIM.
Continue with the [individual startup workflow](sim_real_workflow.md).
