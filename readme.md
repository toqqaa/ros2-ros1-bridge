# ROS1-ROS2 Bridge Setup and Usage Guide

This README explains how to set up and use the ROS1-ROS2 bridge with a Docker container. The bridge enables seamless communication and message exchange between ROS1 and ROS2 systems.

---

## Building the Docker Image

1. Clone this repository and navigate to the directory containing the `Dockerfile`.
2. Build the Docker image:

   ```bash
   docker build -t ros1_ros2_bridge:latest .
   ```


## Running the Container

1. Make the script executable:

   ```bash
   chmod +x run-script.sh
   ```
2. Run the script:

   ```bash
   ./run-script.sh
   ```

---



## Setting Up the ROS1-ROS2 Bridge

#### Step 1: Start the ROS1 roscore

In  **Terminal 1** , start the ROS1 roscore:

```bash
source /opt/ros/noetic/setup.bash
roscore
```



#### Step 2: Start the Dynamic Bridge

In  **Terminal 2** , initiate the dynamic bridge, which identifies matching topics between ROS1 and ROS2 and bridges messages.

1.Source the ROS1 environment

```bash
source /opt/ros/noetic/setup.bash
```

2.Source the ROS2 environment

```bash
source ~/ros2_humble/install/setup.bash
```

3.Source the bridge workspace

```bash
source ~/bridge_ws/install/setup.bash
```

4.Export the ROS Master URI

```bash
export ROS_MASTER_URI=http://localhost:11311
```

5. Run the dynamic bridge

   ```bash
   ros2 run ros1_bridge dynamic_bridge
   ```


#### Step 3: Test Communication

1. **Create a ROS1 talker (publisher)**
   In  **Terminal 3** , run the following:

   ```bash
   source /opt/ros/noetic/setup.bash
   rosrun roscpp_tutorials talker
   ```

**2. Create a ROS2 listener (subscriber)**
In  **Terminal 4** , run the following:

```bash
source ~/ros2_humble/install/setup.bash
ros2 run demo_nodes_cpp listener
```
