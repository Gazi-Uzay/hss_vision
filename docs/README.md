# hss_vision Package

## Purpose
The `hss_vision` package is responsible for all image processing and computer vision tasks within the HSS, including target detection, tracking, classification, and QR code recognition.

## Functionality
- **Camera Feed Processing:** Acquires and processes video frames from the `/camera/image_raw` topic.
- **Target Detection and Tracking:** Implements algorithms (e.g., OpenCV, YOLO, TensorRT) to detect and track moving targets within the camera's field of view at >= 15 FPS.
- **Target Information Extraction:** For each detected target, extracts key information as defined in `hss_interfaces/msg/Target`, including 3D position, confidence, color, and any decoded QR code data.
- **Target Data Publication:** Publishes an array of all detected targets to the `/vision/targets` topic (`hss_interfaces/msg/TargetArray`).
- **Debug Image Publication:** For validation and visualization, it publishes the processed video stream with detections overlaid to the `/vision/image_processed` topic (`sensor_msgs/msg/Image`).
- **Color Separation:** Identifies targets by color (e.g., red for enemy) to support the `AUTO_KILL_COLOR` mode.
- **QR Code Detection:** Detects and decodes QR codes to support the `QR_ENGAGE` mode.

## Nodes

### `vision_node.py`
- **Purpose:** The main node for processing camera frames and performing target detection.
- **Subscribes to:** `/camera/image_raw` (`sensor_msgs/msg/Image`)
- **Publishes to:** 
    - `/vision/targets` (`hss_interfaces/msg/TargetArray`)
    - `/vision/image_processed` (`sensor_msgs/msg/Image`)
- **Description:** This node receives raw image data, applies vision algorithms to detect targets, and publishes the results for the `hss_op_manager` to use. It also publishes a processed image stream for debugging and visualization in the ground station.

### `camera_publisher_node.py`
- **Purpose:** A simulated camera publisher for development and testing.
- **Publishes to:** `/camera/image_raw` (`sensor_msgs/msg/Image`)
- **Description:** This node generates a black screen with "SIMULATED CAMERA FEED" text and publishes it at 30 FPS. It allows for testing the vision pipeline without a physical camera attached.
