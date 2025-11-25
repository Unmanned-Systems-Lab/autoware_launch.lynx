# lynx_vehicle_launch

This directory contains ROS 2 launch and description packages created from `/home/lynx/vehicle_with_sensor_6`.

- `lynx_vehicle_description`: installs the URDF, meshes, and configuration for the vehicle model.
- `lynx_vehicle_launch`: provides launch files for visualizing the model and spawning it into Gazebo.

Example usage:

```bash
ros2 launch lynx_vehicle_launch display.launch.xml
ros2 launch lynx_vehicle_launch gazebo.launch.xml
```
