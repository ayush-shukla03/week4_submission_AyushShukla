## 📦 Build and Setup Instructions

1. **Put this into any ROS 2 workspace**  
   (Example: `~/ros2_ws/src`)

   ```bash
   cd ~/ros2_ws/src
   # Copy your package folder here
   ```

2. **Build the workspace**

   ```bash
   cd ~/ros2_ws
   colcon build
   ```

3. **Source the workspace**

   ```bash
   cd week4_arm
   source install/setup.bash
   ```

4. **Make launch scripts executable**

   ```bash
   cd launch
   chmod +x *.py
   ```
