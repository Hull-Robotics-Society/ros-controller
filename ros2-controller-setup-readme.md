# Raspberry Pi 5 with ROS2 Docker Setup

## Overview
This project documents our journey of setting up **ROS2 (Robot Operating System 2)** on a **Raspberry Pi 5** with only 2GB of RAM. Due to hardware limitations and budget constraints, we opted for a lightweight Docker-based approach instead of installing Ubuntu 22.04 directly.

---

## Initial Plan
We initially intended to:
1. Install **Ubuntu 22.04** on the Raspberry Pi 5.
2. Run ROS2 natively on the operating system.

---

## What Actually Happened

SETUP_1
### **Power Supply Issue**
- The Raspberry Pi 5 was initially powered using a **Raspberry Pi 4 power supply**, which provided only about **50% of the required voltage**.
- Solution: We used a **Samsung S21 Ultra 5G brick** with a **Quest 3 generic charger cable**, which worked much better.

### **Memory Limitations**
- The Raspberry Pi 5 has only **2GB of RAM**, while Ubuntu 22.04 requires **4GB+**, making it impractical for our setup.
- Upgrading to an 8GB Pi5 was considered, but as students, the £80 cost was prohibitive.

### **The Docker Solution**
- After some research, we discovered:
  - A **Docker image** for ROS.
  - A **Docker container** specifically for **ROS2 Humble Hawksbill**.

---

## Final Approach

### Step 1: Install Raspberry Pi OS and Docker

# Install Docker using the official installation script
curl -sSL https://get.docker.com | sh

# Add the user to the Docker group
sudo usermod -aG docker $USER

# Enable and start Docker
sudo systemctl enable docker
sudo systemctl start docker

# Test Docker installation
sudo docker run hello-world


STEP_2
# Create a directory for the Docker setup
mkdir ~/ros2-docker
cd ~/ros2-docker

# Add your Dockerfile (use the one specific for the Pi5 container)

# Build the Docker image
docker build -t ros2-humble .

# Run the ROS2 Docker container
docker run -it --rm ros:humble


STEP_3
# Check the ROS2 distribution
echo $ROS_DISTRO
# Expected output: humble

# Start ROS2 daemons for networking
ros2 daemon start

# List available topics in ROS2
ros2 topic list


STEP_4
# Source ROS2 setup
source /opt/ros/humble/setup.bash

# Create a workspace and package
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python --license apache-2.0 --node-name my_node demo

# Build the package
cd ~/ros2_ws
colcon build

# Source the build
source install/local_setup.bash

# Run the package
ros2 run demo my_node
# Output: Hi from demo


Closing_Notes:
ROS2 Distribution: Humble Hawksbill
    • Docker Image: ros:humble
    • Follow the official ROS2 Wiki for detailed instructions on creating workspaces and packages.
