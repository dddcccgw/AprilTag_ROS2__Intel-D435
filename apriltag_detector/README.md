# AprilTag Detector Package

一個 ROS 2 Python 套件，用於使用 Intel RealSense D435 攝像頭進行 AprilTag 檢測和攝像頭位置驗證。

## 功能

### 1. **apriltag_map** - AprilTag 地圖構建
- 檢測多個 AprilTag 並創建統一的地圖坐標系
- 將所有標籤位置轉換到一個統一的地圖框架中
- 支持臨時濾波以提高穩定性
- 將地圖數據保存為 JSON 格式

**運行方式：**
```bash
ros2 run apriltag_detector apriltag_map
```

### 地圖框架發布
- `apriltag_map` 會在 TF2 中發布 `map` 框架，child frame 為 `camera_link`，方便其他 ROS2 節點訂閱
- TF 發布頻率與相機幀率一致（預設 30 FPS）

### 2. **camera_position_validator** - 攝像頭位置驗證
- 驗證檢測到的攝像頭位置是否符合預期
- 支持誤差檢查和實時反饋
- 顯示攝像頭和標籤位置的驗證結果
- 公差可配置（默認 1.5cm）

**運行方式：**
# AprilTag Detector ROS 2 Package

This directory hosts `apriltag_detector`, a dedicated ROS 2 `ament_python` package that wraps AprilTag map construction, camera position validation, and calibration-data recording for Intel RealSense D435-based setups. Inspired by the broader AprilTag ROS community, the package now exposes ROS-native entry points, TF publishing, and Python console scripts.

## Key ROS components

- **`apriltag_map`** node: captures AprilTags, filters their poses, constructs a unified map frame, and publishes the transformation between the `map` frame and the camera.
- **`camera_validator`** node: validates that the camera pose inferred from tags matches an expected layout.
- **`record_calibration`** node: records paired camera and robot poses for hand-eye calibration pipelines.
- **TF publishing**: `apriltag_map` continuously broadcasts the `map` → `camera_link` transform at the detection frame rate, enabling other ROS 2 nodes to consume consistent pose data.

## Quick start (ROS 2 workspace)

```bash
# ensure workspace overlays are clean
cd ~/AprilTag_ROS2_intel-D435
colcon build --base-paths apriltag_detector --symlink-install
source install/setup.bash
```

Now run any of the package entry points:

```bash
ros2 run apriltag_detector apriltag_map
ros2 run apriltag_detector camera_validator
ros2 run apriltag_detector record_calibration
```

Each command relies on the `pyrealsense2`, `opencv-python`, `dt-apriltags`, and `scipy` stacks, so install them beforehand via `pip3 install -r requirements.txt` or the equivalent commands.

> The `apriltag_map` node publishes `TransformStamped` data for the `map` frame, with `camera_link` as the child frame, providing a solid base frame for downstream consumers.

## Recommended workflow

1. **Build the package** from the workspace root using `colcon`.
2. **Source the overlay** (`source install/setup.bash`) before invoking ROS 2 commands.
3. **Connect the RealSense D435**; without it the node will abort (`RuntimeError: No device connected`).
4. **Optionally use helper scripts** such as `test_package.py` to validate that Python entry points and console scripts are available.

## Entry points and scripts

The package exposes the following console scripts via `setup.py`:

| Script name | Module | Purpose |
|-------------|--------|---------|
| `apriltag_map` | `apriltag_detector.apriltag_map:main` | Map frame detection + TF publishing |
| `camera_validator` | `apriltag_detector.camera_position_validator:main` | Camera pose validation |
| `record_calibration` | `apriltag_detector.record_calibration_data:main` | Calibration data capture |

## Folder layout snapshot

```
apriltag_detector/
├── apriltag_detector/           # ROS package source with three main modules
│   ├── apriltag_map.py           # map frame detection + TF publisher
│   ├── camera_position_validator.py
│   └── record_calibration_data.py
├── package.xml                   # ROS 2 package manifest
├── setup.py                      # Python package + console scripts
├── setup.cfg
├── pyproject.toml
├── resource/
│   └── apriltag_detector         # ament index resource
├── QUICK_START.md
└── test_package.py               # helper to ensure imports & entry points
```

A screenshot of the organized layout can be stored at `docs/screenshots/folder-layout.png` for visual reference. Replace the placeholder image with an actual capture from your environment when available.

## Notes

- This package runs best on ROS 2 Humble or later, though the Python code is compatible with Python 3.10+.
- The TF broadcaster helps keep the detection frame consistent with other ROS tools in your stack.
- Inspired by earlier AprilTag ROS implementations, the current package keeps a ROS-native focus while providing data export and validation hooks.
    0: np.array([0.0, 0.0, 0.0]),
