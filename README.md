# 🧭 AprilTag Multi-Tag Tracking & Map Frame Estimation

A standalone Python implementation for **multi-AprilTag detection**, **6DoF pose estimation**, and **map coordinate frame construction** using an Intel RealSense D435 RGB-D camera.

This project is based on and inspired by [Tinker-Twins/AprilTag-ROS-2](https://github.com/Tinker-Twins/AprilTag-ROS-2) — a ROS 2-based AprilTag detection package. This fork adapts their system into a **non-ROS, lightweight Python tool** with advanced multi-tag tracking and spatial mapping capabilities.

---

## 📸 Screenshots & Setup

### 🔧 Physical Setup
![Physical Setup](setup.png)
*Hardware configuration showing Intel RealSense D435 camera mounted on tripod with three AprilTags (ID: 0, 1, 2) mounted on a vertical board with grid paper for precise positioning*

---

### 🎯 Program: `my_camera_apriltag.py` - Multi-Tag Tracking & Map Frame
![AprilTag Map Frame Visualization](my_camera_apriltag.png)
*Real-time multi-tag tracking showing 3 AprilTags with map frame coordinate system. Features include:*
- *Live camera feed with detected tags (ID 0, 1, 2) highlighted with colored borders*
- *3D coordinate axes (RGB) overlaid on each tag showing orientation*
- *Real-time position data displayed: Tag 0 at origin (0.000, 0.000, 0.000), Tag 1 at (0.197, -0.002, -0.013), Tag 2 at (-0.002, -0.100, -0.003)*
- *Rotation angles (Roll, Pitch, Yaw) for each tag displayed in terminal*
- *Detection status: "Detected: 3/3 tags"*
- *Map frame established with Tag 0 as the origin*

![Map Frame Terminal Output](my_camera_apriltag.png)   

*Terminal output showing map frame status with final saved data including all tag positions and rotations. The program exports data to `apriltag_map.json` when exiting with 'Q'*

---

### 📊 Program: `record_calibration_data.py` - Hand-Eye Calibration
![Calibration Data Recording](record_calibration_data.png)
*Interactive hand-eye calibration data collection showing synchronized camera-tag poses and robot end-effector positions for computing camera-to-robot transformation*

---

### ✅ Program: `camera_position_validation.py` - Position Validator
![Position Validation Pass](camera-position_validator.png)
*Position validation showing ALL CHECKS PASSED with detailed error metrics:*
- *Camera position validated: [-0.0814, 0.0493, -0.5819] m with 14.41 mm error ✓*
- *Tag 0 [ORIGIN]: Perfect match at (0.000, 0.000, 0.000) m*
- *Tag 1: Actual [0.1978, -0.0019, -0.0141] vs Expected [0.1970, -0.0010, -0.0210] - Error: 7.50mm ✓*
- *Tag 2: Actual [-0.0016, -0.1010, -0.0040] vs Expected [-0.0010, -0.1010, -0.0040] - Error: 2.50mm ✓*
- *Green ✓ indicators show all positions within 15mm tolerance*
- *Real-time validation overlay with color-coded pass/fail status*

---

## 🚀 Key Features

### ✨ Advanced Capabilities
- **Multi-Tag Tracking**: Simultaneously detect and track multiple AprilTags (configurable IDs)
- **Map Frame Construction**: Establish a global coordinate system using one tag as the origin
- **Relative Position Calculation**: Compute 3D positions of all tags relative to the map origin
- **Temporal Filtering**: Smooth pose estimation with moving average filter (reduces jitter)
- **Pose Validation**: Automatic validation of detected poses for reliability
- **Real-time Visualization**: 
  - 3D coordinate axes for each tag
  - On-screen display of tag positions in map frame
  - Color-coded tags (origin vs. tracked tags)
- **Data Export**: Save final map data to JSON file with all tag positions and orientations

### 🎯 Technical Highlights
- Intel RealSense D435 RGB-D integration
- Image preprocessing for robust detection (histogram equalization, Gaussian blur)
- Configurable detection parameters for accuracy vs. speed trade-off
- Comprehensive error handling and validation
- Clean terminal output with periodic status updates

---

## 🆚 Key Differences from Original

| Feature | Original (ROS 2) | This Fork (Python Standalone) |
|---------|------------------|-------------------------------|
| **Dependencies** | ROS 2 + multiple packages | Pure Python (no ROS required) |
| **Multi-Tag Tracking** | Single tag focus | Multiple tags simultaneously |
| **Map Frame** | ❌ Not supported | ✅ Global coordinate system |
| **Relative Positioning** | ❌ Not supported | ✅ Tag-to-tag spatial relationships |
| **Temporal Filtering** | ❌ Not included | ✅ Pose smoothing built-in |
| **Data Export** | ROS messages | JSON file with full map data |
| **Setup Complexity** | High (ROS workspace) | Low (pip install only) |

---

## 📦 Requirements

### Hardware
- **Intel RealSense D435** RGB-D Camera
- **Robot Arm** (optional, for hand-eye calibration)
- **AprilTags** - Printed on flat, rigid surface (recommended: foam board or acrylic)

### Software
- **Python 3.8+**
- **Intel RealSense SDK 2.0** (install before running)

### Python Dependencies
```bash
pip install opencv-python pyrealsense2 numpy dt-apriltags scipy
```

> 💡 **Windows Users**: Download and install [Intel RealSense SDK 2.0](https://github.com/IntelRealSense/librealsense/releases) first.

---

## ⚙️ Configuration

Edit parameters at the top of `my_camera_apriltag.py`:

| Parameter | Description | Default |
|-----------|-------------|---------|
| `TAG_SIZE` | Physical size of AprilTag (meters) | `0.0625` (6.25 cm) |
| `FRAME_WIDTH`, `FRAME_HEIGHT` | Camera resolution | `640 × 480` |
| `FPS` | Frame rate | `30` |
| `TARGET_TAG_IDS` | List of tag IDs to track | `[0, 1, 2]` |
| `MAP_ORIGIN_TAG_ID` | Tag ID used as map origin | `0` |

### Detection Parameters
Fine-tune detection quality in the `Detector` initialization:

```python
detector = Detector(
    families="tag36h11",
    nthreads=4,
    quad_decimate=2.0,      # Lower = better accuracy, slower
    quad_sigma=0.8,         # Blur before detection
    refine_edges=1,         # Edge refinement
    decode_sharpening=0.25  # Sharpening factor
)
```

---

## 📂 Project Structure

```
AprilTag-ROS-2-camera/
├── scripts/
│   ├── my_camera_apriltag.py              # Main multi-tag tracking script
│   ├── record_calibration_data.py         # Hand-eye calibration data collection
│   ├── camera_position_validation.py      # Camera pose validation tool
│   └── ...
├── apriltag_map.json                      # Output: Map frame data
├── data/                                  # Calibration data (generated)
│   ├── camera_poses.npy
│   └── robot_poses.npy
└── README.md
```

### 🔧 Script Descriptions

#### 1️⃣ `my_camera_apriltag.py` - Multi-Tag Tracking & Mapping
The **primary script** for real-time AprilTag detection and map frame construction.

**Features:**
- Tracks multiple AprilTags simultaneously (configurable IDs)
- Establishes global coordinate system using one tag as origin
- Computes relative positions of all tags in map frame
- Real-time visualization with 3D axes and position overlay
- Exports map data to JSON file on exit

**Use Case:** Building a spatial map of AprilTag positions for navigation, localization, or scene understanding.

#### 2️⃣ `record_calibration_data.py` - Hand-Eye Calibration Tool
A **data collection script** for robot arm hand-eye calibration.

**Features:**
- Captures synchronized pairs of:
  - Camera-to-tag transformations (from AprilTag detection)
  - Robot base-to-end-effector poses
- Interactive capture mode (press Enter to sample)
- Saves calibration data as NumPy arrays for offline processing

**Use Case:** Determining the transformation between robot base and camera for eye-in-hand or eye-to-hand configurations.

#### 3️⃣ `camera_position_validation.py` - Camera Pose Validator
A **validation tool** that verifies camera and tag positions against ground truth measurements.

**Features:**
- Real-time comparison of detected vs. expected positions
- Validates camera position in map frame
- Checks all tag positions with configurable tolerance (default: 15mm)
- Visual pass/fail indicators (✓/✗) on-screen and in terminal
- Color-coded validation overlay (green=pass, red=fail)
- Detailed error metrics in millimeters
- Comprehensive final validation report

**Use Case:** Quality assurance for camera mounting, verifying physical setup accuracy, debugging positioning issues.

---

## ▶️ Usage

### 🗺️ Basic Multi-Tag Tracking
1. **Connect** your Intel RealSense D435 camera
2. **Navigate** to the scripts directory
   ```bash
   cd scripts
   ```
3. **Run** the main tracking script:
   ```bash
   python3 my_camera_apriltag.py
   ```
4. **Press Q** to exit and save map data to `apriltag_map.json`

### 🎯 Camera Position Validation

Verify that your camera and tags are correctly positioned:

```bash
cd scripts
python3 camera_position_validation.py
```

**Setup required:**
1. Edit the script to set expected positions:
   ```python
   # Expected tag positions (measured physically)
   EXPECTED_TAG_POSITIONS = {
       0: np.array([0.0, 0.0, 0.0]),      # Origin
       1: np.array([0.197, -0.001, -0.021]),  # Tag 1
       2: np.array([-0.001, -0.101, -0.004])  # Tag 2
   }
   
   # Expected camera position
   EXPECTED_CAMERA_POSITION = np.array([-0.088, 0.060, -0.589])
   
   # Tolerance (meters)
   POSITION_TOLERANCE = 0.015  # 15mm
   ```

2. Run the validator and observe:
   - **Green ✓**: Position is within tolerance
   - **Red ✗**: Position exceeds tolerance threshold
   - **Error metrics**: Shown in millimeters for easy interpretation

**Terminal Output Example:**
```
==================================================================
🎯 Camera Position Validation
==================================================================
Camera Position: [-0.0881,  0.0598, -0.5885] m
Expected:        [-0.0880,  0.0600, -0.5890] m
Error: 0.71 mm - ✓ PASS

==================================================================
Tag Position Validation
==================================================================
Tag 0:
  Actual:   [ 0.0000,  0.0000,  0.0000] m
  Expected: [ 0.0000,  0.0000,  0.0000] m
  Error: 0.00 mm - ✓ PASS

Tag 1:
  Actual:   [ 0.1971, -0.0015, -0.0213] m
  Expected: [ 0.1970, -0.0010, -0.0210] m
  Error: 0.58 mm - ✓ PASS

==================================================================
Overall: ✓✓✓ ALL CHECKS PASSED ✓✓✓
==================================================================
```

### 🤖 Hand-Eye Calibration Workflow

For robot arm integration, follow this two-step process:

#### Step 1: Collect Calibration Data
```bash
cd scripts
python3 record_calibration_data.py
```

**Interactive prompts:**
1. Position robot end-effector at different poses
2. Press **Enter** to capture camera-tag pose + robot pose
3. Repeat for 10-20 different poses (more = better accuracy)
4. Press **'q'** to finish and save data

**Output:** Creates `data/camera_poses.npy` and `data/robot_poses.npy`

#### Step 2: Compute Calibration (Placeholder)
```python
# Use your preferred hand-eye calibration solver
# Example with OpenCV:
import cv2
R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
    R_gripper2base, t_gripper2base,
    R_target2cam, t_target2cam,
    method=cv2.CALIB_HAND_EYE_TSAI
)
```

> **Note:** `record_calibration_data.py` requires integration with your robot's API to get end-effector poses. Update the `T_base_ee` placeholder with your robot's actual pose data.

### Terminal Output Example
```
📷 Camera Intrinsics:
   fx: 603.59, fy: 603.08, cx: 329.98, cy: 246.51
✅ AprilTag detector initialized! Press Q to quit
🗺️  Map Origin: Tag 0

============================================================
🗺️  Map Frame Status (Origin: Tag 0)
============================================================
Tag 0 [ORIGIN]: (0.000, 0.000, 0.000) m
Tag 1: ( 0.197, -0.002, -0.013) m
         Rotation: R=-2.6° P=9.1° Y=-0.5°
Tag 2: (-0.002, -0.100, -0.003) m
         Rotation: R=-2.5° P=-0.3° Y=0.3°
```

### On-Screen Display
The live video window shows:
- **Tag borders**: Green (origin) / Blue (tracked tags)
- **Tag IDs**: Yellow text overlay with ID number
- **3D Axes**: RGB lines showing orientation (X=Red, Y=Green, Z=Blue)
- **Map positions**: Real-time X/Y/Z coordinates displayed on screen
- **Detection status**: Shows "Detected: 3/3 tags" at bottom

---

## 💾 Output Data

When you exit (press Q), the program saves `apriltag_map.json`:

```json
{
  "metadata": {
    "map_origin_tag_id": 0,
    "tag_size": 0.0625,
    "timestamp": "2025-10-29 15:30:45",
    "camera_intrinsics": {
      "fx": 603.59, "fy": 603.08,
      "cx": 329.98, "cy": 246.51
    }
  },
  "tags": {
    "0": {
      "position": [0.0, 0.0, 0.0],
      "rotation_matrix": [[1,0,0],[0,1,0],[0,0,1]],
      "rotation_euler_deg": [0.0, 0.0, 0.0],
      "is_origin": true
    },
    "1": {
      "position": [0.1973, -0.0019, -0.0139],
      "rotation_matrix": [...],
      "rotation_euler_deg": [-2.65, 9.13, -0.49],
      "is_origin": false
    },
    "2": {
      "position": [-0.0020, -0.1003, -0.0033],
      "rotation_matrix": [...],
      "rotation_euler_deg": [-2.48, 0.26, 0.28],
      "is_origin": false
    }
  }
}
```

---

## 🎓 Use Cases

### 🗺️ Map Frame Construction
- **Robot Localization**: Use tags as landmarks for navigation
- **Multi-Robot Coordination**: Shared map frame for robot fleets
- **Scene Understanding**: Build spatial maps of environments

### 🤖 Hand-Eye Calibration
- **Eye-in-Hand**: Camera mounted on robot end-effector
- **Eye-to-Hand**: Camera fixed in workspace, observing robot
- **Robot-Camera Registration**: Align camera and robot coordinate systems

### ✅ Position Validation & Quality Assurance
- **Setup Verification**: Confirm physical installation matches design specs
- **Calibration Validation**: Verify camera mounting accuracy post-calibration
- **Troubleshooting**: Diagnose positioning errors in multi-tag systems
- **Quality Control**: Automated checks for production/research setups

### 🎯 Additional Applications
- **AR/VR Tracking**: Spatial tracking of multiple objects
- **Object Pose Estimation**: 6DoF tracking in cluttered scenes
- **Quality Inspection**: Precise part localization in manufacturing

---

## 🔧 Troubleshooting

### Camera Not Found
```bash
# Check if camera is detected
rs-enumerate-devices

# Test camera stream
realsense-viewer
```

### Low Detection Rate
- Increase lighting in the environment
- Reduce `quad_decimate` value (e.g., 1.5)
- Ensure tags are flat and clearly visible
- Check tag size matches `TAG_SIZE` parameter

### Jittery Pose Estimates
- Increase `TemporalFilter.window_size` (default: 5)
- Reduce camera motion during tracking
- Ensure stable mounting of tags

### Hand-Eye Calibration Issues
- **Robot poses not captured**: Update `T_base_ee` in `record_calibration_data.py` with your robot's API
- **Poor calibration accuracy**: Collect more samples (20+ recommended) with diverse robot poses
- **Import errors**: Ensure `my_camera_apriltag.py` is in the same directory or add to Python path

### Camera Position Validation Issues
- **All checks failing**: Verify you've updated `EXPECTED_TAG_POSITIONS` and `EXPECTED_CAMERA_POSITION` with your actual measurements
- **Inconsistent validation**: Increase `POSITION_TOLERANCE` if physical setup has measurement uncertainty
- **Camera position unknown**: Run `my_camera_apriltag.py` first to determine camera position, then use that value
- **Single tag fails**: Check if tag is properly mounted and flat - even small tilts affect position accuracy

---

## 🔄 Workflow Comparison

### Multi-Tag Tracking Workflow
```
1. Place multiple AprilTags in scene
2. Run my_camera_apriltag.py
3. System detects all tags and computes relative positions
4. View real-time map visualization
5. Exit (press Q) to save apriltag_map.json
```

### Camera Position Validation Workflow
```
1. Physically measure and record tag positions with ruler/caliper
2. Update EXPECTED_TAG_POSITIONS in camera_position_validation.py
3. Run camera_position_validation.py
4. System compares detected vs. expected positions
5. Review validation results (✓ pass / ✗ fail)
6. Adjust camera/tags if errors exceed tolerance
```

### Hand-Eye Calibration Workflow
```
1. Mount camera on robot or workspace
2. Place single AprilTag as calibration target
3. Run record_calibration_data.py
4. Move robot to multiple poses, capture at each position
5. Exit to save camera_poses.npy & robot_poses.npy
6. Process data with hand-eye calibration algorithm
7. Obtain camera-to-robot transformation matrix
```

---

## 🧪 Tested Environment

| Component | Version |
|-----------|---------|
| **Camera** | Intel RealSense D435 |
| **RealSense SDK** | 2.55+ |
| **Python** | 3.10 |
| **OpenCV** | 4.10+ |
| **dt-apriltags** | 1.0.4+ |
| **NumPy** | 1.24+ |
| **SciPy** | 1.11+ |

---

## 🙏 Credits

- **Original ROS 2 Framework**: [Tinker-Twins/AprilTag-ROS-2](https://github.com/Tinker-Twins/AprilTag-ROS-2)
- **AprilTag Library**: [AprilRobotics/apriltag](https://github.com/AprilRobotics/apriltag)
- **Python Wrapper**: [duckietown/lib-dt-apriltags](https://github.com/duckietown/lib-dt-apriltags)
- **Extended & Adapted by**: [@dddcccgw](https://github.com/dddcccgw)

---

## 📄 License

Released under the **MIT License**. Please also respect the licenses of:
- [Tinker-Twins/AprilTag-ROS-2](https://github.com/Tinker-Twins/AprilTag-ROS-2)
- [AprilTag original library](https://april.eecs.umich.edu/software/apriltag)

---

## 👨‍💻 Author

**David Chen**  
📧 gwchen24@gmail.com  
💼 [github.com/dddcccgw](https://github.com/dddcccgw)

---

## ⭐ Support This Project

If this project helps you, please star:
- ⭐ [Tinker-Twins/AprilTag-ROS-2](https://github.com/Tinker-Twins/AprilTag-ROS-2) (original)
- ⭐ [dddcccgw/AprilTag-ROS-2-camera](https://github.com/dddcccgw/AprilTag-ROS-2-camera) (this fork)

---

## 🛠️ Future Enhancements

### 🗺️ Multi-Tag Tracking
- [ ] Multi-camera support for larger coverage
- [ ] Bundle adjustment for improved accuracy
- [ ] Real-time 3D map visualization (Matplotlib/RViz)
- [ ] Tag persistence across frames (handle occlusions)
- [ ] IMU fusion for better tracking stability

### ✅ Position Validation
- [ ] Automatic tolerance calculation from measurements
- [ ] CSV export of validation results for record-keeping
- [ ] Historical validation tracking (compare runs over time)
- [ ] Support for dynamic tolerance based on distance

### 🤖 Hand-Eye Calibration
- [ ] Automated calibration solver integration (OpenCV/Easy-HandEye)
- [ ] Real-time calibration quality metrics
- [ ] Robot pose auto-capture with motion planning
- [ ] Support for multiple robot arms

### 🌐 Integration
- [ ] ROS 2 bridge for hybrid systems
- [ ] Web interface for remote monitoring
- [ ] REST API for external applications
- [ ] Docker containerization

**Contributions welcome!** Feel free to open issues or pull requests.
