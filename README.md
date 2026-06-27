# Autonomous Robot Navigation in Crowded Warehouses

Navigation system for Nova Carter mobile robot in a busy warhouse environment. This system solves two problems, shortest way to the goal, while actively tracking worker's movement predicting their path and avoiding collisions adjusting it's route in real time. 

## Demo

---

## How it works

LiDAR scans go trough a preprocessing pipeline that filters out signals pointing the floor, robot's body and too far signals to be reliable. Everything else gets clustered to separate people from static objects.

Kalman filter tracker helps predicting future steps of each person by simple motion model. On a top of that sits a motion-state machine that decides which obstacle is treated as a moving/stationary. 

The planner in this project is MPC running at 10 Hz with 3s lookahead. Each cycle it solves an optimization problem finding fastest route without colliding with any of the people or stationary objects. Instead of taking into consideration only the current position of the person, it's shifted towards the prediction of their next steps. The robot should predict if the human would be an obstacle in the moment of crossing, or he would pass by at the time.

---

## Stack

| | |
|---|---|
| Robot | NVIDIA Nova Carter |
| Simulation | NVIDIA Isaac Sim 6.1.0 |
| Framework | ROS 2 Jazzy |
| Language | C++20, Python 3.12 |
| MPC solver | ACADOS — SQP / HPIPM |
| Symbolic math | CasADi |
| Point cloud | PCL |

---

## Getting started

**Dependencies**

```bash
sudo apt install ros-jazzy-pcl-ros ros-jazzy-pcl-conversions

# ACADOS
git clone https://github.com/acados/acados.git ~/acados
cd ~/acados && git submodule update --init --recursive
mkdir build && cd build
cmake .. -DACADOS_WITH_QPOASES=ON
make -j$(nproc) && cmake --install .

pip install ~/acados/interfaces/acados_template --break-system-packages
```

**Build**

```bash
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

**Run**

Start Isaac Sim with the warehouse scene, then:

```bash
ros2 launch human_tracker human_tracker.launch.py
```

Both nodes configure and activate automatically. Use the **2D Nav Goal** tool in RViz to set a destination.

**Regenerate MPC solver** (only needed after changing planner parameters)

```bash
ACADOS_SOURCE_DIR=~/acados \
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/acados/lib \
python3 scripts/generate_social_mpc.py

colcon build --packages-select social_mpc
```
