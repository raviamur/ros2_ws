# Vision-Based Obstacle Avoidance for Assistive Cart

This ROS 2 package enables real-time stereo vision-based obstacle avoidance for a motorized cart using ESP32 control.

## 🚀 Features

- Zone-based obstacle detection (Left, Center, Right) using disparity images
- Real-time blob detection and navigation commands: `LEFT`, `RIGHT`, `REVERSE`, `STOP`
- Distance estimation based on stereo camera disparity
- Serial communication bridge to ESP32 motor controller
- Live visual overlay with object distance estimates

## 🎯 Current Status

✅ Cart tested successfully at night with stable navigation  
✅ Ack-based command gating working  
❌ STOP threshold still under tuning  
⚠️ Needs improved lighting or IR pattern projection for daytime

## 🧠 Nodes

### `zone_disparity_nav_node.py`
- Subscribes to `/disparity` topic
- Accumulates L/C/R blobs
- Publishes `/nav_cmd` based on closest detected zone

### `zone_disparity_overlay_node.py`
- Subscribes to `/disparity` and `/left/image_raw`
- Projects 3D blob centroids onto image with bounding boxes
- Publishes `/overlay/image`

### `ros2_serial_bridge_node.py`
- Reads `/nav_cmd`
- Sends commands to ESP32 over serial only after `/esp_ack`
- Enforces directional control (reverse logic, speed limits)

## 📦 Launch

```bash
ros2 launch my_vision_package stereo_cart.launch.py
(This is yet to be edited, meanwhile, do ros2 my_vision_package disparity.launch.py
Then run the zone_disparity_nav_node.py from vscode arrow.Then run ros2 my_vision_package ros2_serial_bridge_node. To see the overlay alone without running the cart, do ros2 run zone_disparity_overlay_node
🧰 Calibration Files

Place these in your camera_info/ or root directory:

    left.yaml

    right.yaml

    stereo_calibration.yaml

🔦 Lighting Tips

    Use green laser mesh or IR projector at night

    Avoid strong sunlight reflections

    Projected patterns improve disparity in flat or textureless areas

📌 Version

Tag: v1.0-stable-cart
Date: May 4, 2025
🛠️ To Do (Next Phase)

    Improve STOP responsiveness

    Calibrate depth correction factor (k)

    Add wheel feedback from ESP32

    Build Docker image for deployment

    Add RViz/diagnostics launch for developers

🤝 Acknowledgments

This project was built for assistive mobility for individuals with cerebral palsy, using affordable and modular components.
