# 安装与G1部署

训练与ROS部署请使用独立的Python环境：训练示例使用Python 3.11，
ROS Humble的Python接口使用Python 3.10。可以复用已有的兼容环境。

## 训练

```bash
conda create -n unitree_rl_mjlab python=3.11
conda activate unitree_rl_mjlab
git clone --branch dev https://github.com/mmurooka/unitree_rl_mjlab.git
cd unitree_rl_mjlab
python -m pip install -e .
```

依赖版本由 `setup.py` 指定。训练和回放命令见 [README](../README.md)。

## 仿真与部署

C++依赖、Unitree SDK、ROS构建选项及共享Python环境见
[安装与构建指南](setup_en.md)。按该指南从仓库根目录构建
`simulate/build/unitree_mujoco` 与 `deploy/robots/g1/build/g1_ctrl`。

逐个终端启动、Xbox操作、GLIM及无GLIM模式见
[sim/real启动流程](sim_real_workflow.md)。先进入Velocity并确认生效，再解除仿真悬挂。
GLIM使用外部安装与配置；定位源必须明确选择，实机不能选择模拟器真值。
