# RViz Attitude Plugin 🧭

[![ROS 2](https://img.shields.io/badge/ROS2-%20Humble+-blue.svg)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)

A **RViz2 display plugin** that transforms complex **quaternion data** into intuitive visual feedback. Designed for roboticists who need to quickly understand and troubleshoot orientation data from **IMUs**, **sensor fusion algorithms**, and **navigation systems**.

> Quaternions are mathematically elegant but notoriously difficult to interpret at a glance.

This plugin bridges that gap with **aviation-style attitude indicators** and **real-time Euler angle conversion**, making it easy to debug sensor data, validate algorithms, and monitor robot behavior.

## 🎥 Demo

<div align="center">
  <img src="docs/demo.gif" alt="RViz Attitude Plugin Demo" width="800"/>
  <p><i>Real-time attitude visualization in RViz2</i></p>
</div>

## ✨ Features

- 🧭 **Artificial horizon & heading indicator** - Aviation-style gauges for immediate spatial awareness
- 🎯 **Euler angle readouts** - Real-time Roll, Pitch, Yaw values in degrees or radians
- 📡 **8+ message types** - IMU, Odometry, Pose, PoseWithCovariance, Quaternion messages
- 🎨 **Customizable overlay** - Adjustable position, size, and transparency
- ⚡ **Lightweight performance** - Efficient overlay rendering with minimal overhead

## 📋 Requirements

- **Ubuntu**: 22.04+
- **ROS 2**: Humble+

## 🚀 Installation

1. **Clone the repository** into your ROS2 workspace:
   ```bash
   cd ~/colcon_ws/src
   git clone https://github.com/amgaber95/rviz_attitude_plugin.git
   ```

2. **Install dependencies:**
   ```bash
   cd ~/colcon_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the package:**
   ```bash
   colcon build --packages-select rviz_attitude_plugin
   ```

4. **Source the workspace:**
   ```bash
   source ~/colcon_ws/install/setup.bash
   ```

## 📖 Usage

### Adding the Display to RViz2

1. **Launch RViz2:**
   ```bash
   rviz2
   ```

2. **Add the Attitude Display:**
   - Press `Ctrl+N` or click the **Add** button in the **Displays** panel
   - Navigate to `rviz_attitude_plugin` → `Attitude`
   - Click **OK**

3. **Configure the display:**
   - Select your orientation topic from the **Topic** dropdown in the display properties
   - Click the **🔄 Refresh** button to update the topics list if your topic doesn't appear
   - Adjust visual settings as needed


## 📦 Supported Message Types

The plugin automatically extracts quaternion data from these ROS2 message types:

| Message Type | Quaternion Source |
|--------------|-------------------|
| `geometry_msgs/msg/Quaternion` | Direct quaternion |
| `geometry_msgs/msg/QuaternionStamped` | `.quaternion` |
| `geometry_msgs/msg/Pose` | `.orientation` |
| `geometry_msgs/msg/PoseStamped` | `.pose.orientation` |
| `geometry_msgs/msg/PoseWithCovariance` | `.pose.orientation` |
| `geometry_msgs/msg/PoseWithCovarianceStamped` | `.pose.pose.orientation` |
| `sensor_msgs/msg/Imu` | `.orientation` |
| `nav_msgs/msg/Odometry` | `.pose.pose.orientation` |

> **Note:** The plugin uses TF2's `getRPY()` method for quaternion to Euler conversion, following the standard ROS convention (RPY with extrinsic XYZ rotation).

## 🐛 Troubleshooting

### Topic not appearing in the list?
- Click the **🔄 Refresh** button in the Topic property
- Ensure your node is publishing messages: `ros2 topic echo /your_topic`
- Verify the message type is supported (see table above)

### Display not showing?
- Check that **Show Overlay** is enabled in the properties
- Adjust **Overlay X** and **Overlay Y** positions if it's off-screen
- Verify your topic is publishing: `ros2 topic hz /your_topic`

### Orientation looks wrong?
- Confirm your quaternion follows ROS conventions (right-handed, Hamilton convention)
- Check if the data needs coordinate frame transformation
- Verify the message timestamp is recent (stale data may not update)

## 📄 License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

## 🤝 Contributing

Contributions are welcome. Please feel free to submit issues or pull requests.

## 👨‍💻 Maintainer

**Abdelrahman Mahmoud**

📧 Email: abdulrahman.mahmoud1995@gmail.com  
🐙 GitHub: [@amgaber95](https://github.com/amgaber95)
