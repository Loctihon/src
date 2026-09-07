# 📦 Tổng quan ROS2 Workspace — `do_an_ws/src`

> File tóm tắt toàn bộ workspace. Đọc file này là đủ để nắm được kiến trúc, mục đích, node/topic, cách chạy của từng package mà **không cần đọc lại toàn bộ source code**.
>
> **Kiến trúc robot:** Robot **2 bánh vi sai (diff drive)** dùng cho đồ án — chạy được cả **mô phỏng Gazebo** lẫn **xe thật** (ESP32/Arduino + RPLidar C1).
> **ROS distro:** Humble / Iron (dùng xacro, nav2, slam_toolbox, gazebo_ros2_control).

---

## 1. Sơ đồ cây workspace

```
src/
├── CONTROL WITH PLAYSTATION_SLAM/  # (Arduino .ino) FIRMWARE xe thật: ESP32 nhận tay cầm PS + Arduino Mega lái motor/đọc encoder [TỰ VIẾT]
├── hall_reader_cpp/                # (C++) Node đọc odometry từ Arduino Mega qua Serial -> odom + TF  [TỰ VIẾT]
├── my_robot_description/           # (Python) Package mô tả robot: URDF/xacro, launch, config, scripts, maps [TỰ VIẾT]
├── rf2o_laser_odometry/            # (C++) Thư viện ước lượng odometry từ scan laser (range-flow) [THƯ VIỆN NGOÀI]
└── sllidar_ros2/                   # (C++) Driver chính thức RPLidar (A1/A2/A3/C1/S1...) của Slamtec [THƯ VIỆN NGOÀI]
```

---

## 2. Package `hall_reader_cpp` — Node odometry C++ đọc từ Arduino/ESP32

| Thuộc tính | Giá trị |
|---|---|
| Loại build | `ament_cmake` |
| Executable | `arduino_odom_node` |
| Node name | `arduino_odom_node` |

**Vai trò:** Đọc dữ liệu xung encoder (pulse) gửi từ ESP32/Arduino qua cổng serial `/dev/ttyACM0`, tính toán **dead-reckoning odometry** bằng mô hình xe vi sai, rồi publish **Odometry + TF** cho Nav2/SLAM.

### Luồng hoạt động ([`arduino_odom_node.cpp`](hall_reader_cpp/src/arduino_odom_node.cpp))
1. Mở serial `/dev/ttyACM0` baud **115200** (cấu hình `termios` chuẩn, 8N1, raw).
2. Timer chạy **100 Hz** đọc từng dòng dữ liệu từ serial.
3. Mỗi dòng có định dạng **`<pulse_trái>,<pulse_phải>\n`** (2 số nguyên phân cách bằng dấu phẩy).
4. Tính **delta_pulse** giữa 2 lần đọc → quy đổi thành quãng đường di chuyển từng bánh.
5. Áp dụng **kinematics xe vi sai** (công thức trung bình + vi phân quay).
6. Publish **odom** và **TF `odom -> base_footprint`**.

### Thông số vật lý xe (khai báo **cứng** trong code, cần đo lại)
| Thông số | Giá trị | Ghi chú |
|---|---|---|
| `PULSES_PER_REV` | 330.0 | số xung encoder / 1 vòng |
| `WHEEL_RADIUS` | 0.065 m | bán kính bánh xe |
| `WHEEL_BASE` | 0.25 m | khoảng cách 2 bánh |
| `dist_per_pulse` | `(2π·R)/PULSES_PER_REV` | tự tính |

> ⚠️ **Lưu ý:** Các thông số này **hard-code** trong constructor (dòng 45–47), đồng thời trong [`my_robot.gazebo`/`diff_drive`] giá trị khác (wheel_radius 0.10795, wheel_separation 0.435). Xem phần 3.

### Topic / TF do node này publish
| Loại | Tên | Nội dung |
|---|---|---|
| Topic | `odom` (`nav_msgs/Odometry`) | pose `odom` + twist vận tốc |
| TF | `odom` → `base_footprint` | vị trí + hướng (RPY z = theta) |

**Executable build:** `add_executable(arduino_odom_node ...)` — xem [`CMakeLists.txt`](hall_reader_cpp/CMakeLists.txt:20).

---

## 3. Package `my_robot_description` — Mô tả robot + launch + điều khiển

| Thuộc tính | Giá trị |
|---|---|
| Loại build | `ament_python` |
| Vai trò | Chứa URDF/xacro, meshes STL, launch files, config (nav2/slam/controller), worlds Gazebo, maps, scripts điều khiển |

### 3.1 Cấu trúc file
```
my_robot_description/
├── urdf/          # my_robot.xacro (file gộp chính) + các xacro thành phần
├── meshes/        # base_link.stl, left/right_wheel_1.stl (CAD của xe thật)
├── launch/        # display, gazebo, manual_control, real_mapping, joystick
├── config/        # diff_drive_controller, nav2_params, xbox, laser_filter, display.rviz...
├── worlds/        # my_world.world (thế giới Gazebo với tường)
├── maps/          # my_map.yaml + my_map.pgm (bản đồ đã scan)
└── scripts/       # cmd_vel_to_serial.py, custom_joy_teleop.py
```

### 3.2 URDF — Robot (`urdf/my_robot.xacro`)

File chính include các file xacro khác:
[`my_robot.xacro`](my_robot_description/urdf/my_robot.xacro:4) → gộp `materials`, `my_robot.trans`, `my_robot.gazebo`, `inertial_macros`, `ros2_control`, `lidar`.

**Các link & joint:**
| Link | Mô tả | Mass (kg) |
|---|---|---|
| `base_footprint` | frame gốc (điểm chiếu xuống đất) | — |
| `base_link` | thân xe (mesh STL, scale 0.001) | 40 |
| `right_wheel_1` / `left_wheel_1` | bánh xe (mesh STL) | 4.5 mỗi bánh |

| Joint | Type | Parent → Child | Axis |
|---|---|---|---|
| `base_link_to_base_footprint_link` | fixed | base_footprint → base_link | — |
| `right_wheel_to_base_link` | **continuous** | base_link → right_wheel_1 | y (0 1 0) |
| `left_wheel_to_base_link` | **continuous** | base_link → left_wheel_1 | y (0 1 0) |

> ⚠️ Trong `my_robot.xacro` gốc không có camera sensor thật được include (chỉ có camera.xacro để tham khảo — parent là `chassis` không tồn tại, xem như **chưa dùng**). `camera.xacro` hiện là file "thừa"/đang phát triển.

**LiDAR:** [`lidar.xacro`](my_robot_description/urdf/lidar.xacro) định nghĩa joint `laser_joint` (fixed, base_link → laser_frame, vị trí x=-0.5 z=0.26, rpy quay 180°) và sensor ray trong Gazebo (270 samples, 200° FOV, range 0.3–20m, publish `~/out := scan`, frame `laser_frame`).

**Điều khiển Gazebo** ([`ros2_control.xacro`](my_robot_description/urdf/ros2_control.xacro)):
- Plugin `gazebo_ros2_control/GazeboSystem`, 2 joint bánh xe có `command_interface: velocity` + `state_interface: velocity/position`.
- Nạp config từ [`diff_drive_controller.yaml`](my_robot_description/config/diff_drive_controller.yaml).
- `my_robot.trans`: transmission EffortJointInterface (chuẩn ROS1 cũ, chỉ mang tính tham khảo).
- `my_robot.gazebo`: gán material, ma sát (base mu 0.2, **bánh xe mu 15** để bám).

**Inertial macros** ([`inertial_macros.xacro`](my_robot_description/urdf/inertial_macros.xacro)): macro tính inertia chuẩn cho `inertial_sphere`, `inertial_box`, `inertial_cylinder`.

### 3.3 Config quan trọng

#### `config/diff_drive_controller.yaml`
| Thông số | Giá trị |
|---|---|
| controller_manager `update_rate` | 50 |
| `publish_rate` | 50.0 |
| base/odom frame | `base_footprint` / `odom` |
| `enable_odom_tf` | true |
| `open_loop` | false |
| wheel_separation | **0.435 m** |
| wheel_radius | **0.10795 m** |

#### `config/nav2_params.yaml` (bộ tham số Nav2 chuẩn, cấu hình theo xe)
- **AMCL:** `base_footprint`/`odom`/`map`, model Differential, scan_topic `scan`.
- **DWB local planner** (`FollowPath`): `max_vel_x 0.26`, `max_vel_theta 1.0`, `acc_lim_x 2.5`.
- **Costmaps:** local 3×3m rolling window; robot_radius **0.22**; local dùng voxel+inflation, global dùng static+obstacle+inflation; inflation_radius 0.55, cost_scaling 3.0.
- **NavfnPlanner** global, SimpleSmoother, BT navigator, behavior server (spin/backup/...), velocity_smoother (max 0.26/1.0).
- **Tất cả `use_sim_time: True`.**

#### `config/mapepr_params_online_async.yaml`
Bộ tham số SLAM Toolbox (Ceres solver, chế độ mapping, base_frame `base_footprint`, scan `/scan`, resolution 0.05, loop closing bật...). Có cấu hình đầy đủ loop closure để scan map thật.

#### `config/laser_filter.yaml`
Filter `scan_to_scan_filter_chain` → `LaserScanAngularBoundsFilter` giới hạn góc ±90° (-1.5708 → 1.5708 rad).

#### `config/xbox.yaml`
Config `teleop_twist_joy`: axis_linear x=1, axis_angular yaw=0, scale 0.5, turbo button R1 (id 5) scale 1.0, `require_enable_button: false`.

#### `config/display.rviz`
File RViz dùng chung cho các launch.

### 3.4 Launch files (Cách chạy chính)

| Launch file | Mục đích |
|---|---|
| [`display.launch.py`](my_robot_description/launch/display.launch.py) | Chỉ xem robot: robot_state_publisher + joint_state_publisher(+gui) + rviz2 |
| [`gazebo.launch.py`](my_robot_description/launch/gazebo.launch.py) | **Mô phỏng đầy đủ**: Gazebo (world) + spawn robot + controller (diff_drive/joint_state) + rviz + **Nav2 bringup** (map my_map) (SLAM toolbox đang bị comment). `use_sim_time=true` |
| [`real_mapping.launch.py`](my_robot_description/launch/real_mapping.launch.py) | **Xe thật - scan bản đồ**: RPLidar node (serial /dev/ttyUSB0, baud 460800, frame laser_frame) + `arduino_odom_node` (hall_reader_cpp) + static TF base_link→laser_frame + laser_filter (comment) + **SLAM Toolbox** (mapping) + rviz. `use_sim_time=false` |
| [`joystick.launch.py`](my_robot_description/launch/joystick.launch.py) | joy_node + custom_joy_teleop → publish `/diff_drive_controller/cmd_vel_unstamped` (dùng cho **mô phỏng**) |
| [`manual_control.launch.py`](my_robot_description/launch/manual_control.launch.py) | joy_node + `cmd_vel_to_serial` (điều khiển **xe thật** qua Arduino bằng serial char) |

### 3.5 Scripts điều khiển

#### `scripts/cmd_vel_to_serial.py` (điều khiển xe thật bằng tay cầm)
- Sub `/joy` → mở serial `/dev/ttyACM0` baud **115200** → gửi **1 ký tự ASCII** xuống ESP32/Arduino.
- Bảng lệnh:
  | Phím | Lệnh gửi | Hành động |
  |---|---|---|
  | Axes[1] > 0.01 | `F` | tiến |
  | Axes[1] < -0.01 | `B` | lùi |
  | Axes[2] > 0.01 | `L` | trái |
  | Axes[2] < -0.01 | `R` | phải |
  | (thả cần / nút A=0) | `S` | dừng |
  | Button[0] | `T` | ... (tăng tốc tuỳ firmware) |
  | Button[1] | `S` | dừng |
  | Button[2]/[3]/[5]/[6]/[7]/[8] | `Y`/`X`/`E`/`Q`/`D`/`A` | các lệnh khác |
- Có hàm `watchdog_check()` để tự động dừng (`S`) nếu mất tín hiệu > 0.5s (đang bị comment timer).

> ⚠️ **Lưu ý giao thức 2 chiều:** Node này **gửi** ký tự lệnh qua `/dev/ttyACM0`, còn `arduino_odom_node` (C++) **đọc** pulse encoder cũng qua `/dev/ttyACM0`. → 2 node dùng chung 1 cổng serial. Phần firmware Arduino phải phân biệt lệnh nhận được & dòng dữ liệu gửi đi. Cần đảm bảo chỉ **1 node mở cổng** tại 1 thời điểm (thực tế đang có xung đột tiềm ẩn).

#### `scripts/custom_joy_teleop.py` (điều khiển mô phỏng)
- Sub `/joy` → publish `Twist` tới **`/diff_drive_controller/cmd_vel_unstamped`**.
- Axes[5] (RT): +1 → tăng scale 0.5, -1 → giảm. Cần analog trái: `linear.x = axes[1]·scale`, `angular.z = axes[0]·scale`.
- Không có watchdog.

### 3.6 Maps & Worlds
- [`maps/my_map.yaml`](my_robot_description/maps/my_map.yaml): image `my_map.pgm`, mode `trinary`, resolution **0.05**, origin `[-11.3, -9.67, 0]`, occupied 0.65 / free 0.25.
- [`worlds/my_world.world`](my_robot_description/worlds/my_world.world): thế giới Gazebo ~1390 dòng, gồm ground_plane + mô hình tường `my_world` (các box Wall_... tạo thành phòng để test SLAM/Nav2).

---

## 4. Firmware xe thật — `CONTROL WITH PLAYSTATION_SLAM` (ESP32 + Arduino Mega)

**Vai trò:** Toàn bộ phần mềm chạy trên vi điều khiển của xe thật, gồm **2 board**:
- **ESP32** (thư viện **Bluepad32**): nhận **tay cầm PlayStation** qua Bluetooth.
- **Arduino Mega**: điều khiển **2 động cơ DC** + đọc **encoder (cảm biến Hall)**.

2 board nối với nhau bằng **UART**; Mega nối với PC ROS2 bằng **USB Serial**. (Có sẵn bản build trong `ESP32/build/...`)

### Sơ đồ kết nối phần cứng
```
[Tay cầm PS] --Bluetooth--> [ESP32]
                                 │  UART Serial2 (9600 baud, TX2=17, RX2=16)
                                 ▼
                          [Arduino Mega Serial1 (RX1)]
                                 │
                                 ├── Điều khiển: DIR/BRAKE + DAC MCP4725 → 2 động cơ DC
                                 │
                                 └── USB Serial (115200) --> [PC ROS2: /dev/ttyACM0]  (gửi pulse encoder)
```

### 4.1 ESP32 — đọc tay cầm PS ([`ESP32.ino`](CONTROL WITH PLAYSTATION_SLAM/ESP32/ESP32.ino))
- `Serial2.begin(9600, SERIAL_8N1, 16, 17)`: gửi lệnh sang Arduino Mega (TX2 = chân 17, RX2 = chân 16).
- Map phím tay cầm → gửi **1 ký tự ASCII** xuống Mega qua Serial2 (mỗi lần kèm `delay(100)`):
  | Phím | Ký tự gửi |
  |---|---|
  | D-pad ↑ / ↓ / ← / → | `F` / `B` / `L` / `R` |
  | Nút **B** | `S` (phanh) |
  | Nút **X** | `X` (nhả phanh) |
  | Nút **Y** | `Y` (HUMAN MODE) |
  | Nút **A** | `T` (ROBOT MODE) |
  | Cò **LB** / **LT** | `Q` / `A` (tăng/giảm tốc phải) |
  | Cò **RB** / **RT** | `E` / `D` (tăng/giảm tốc trái) |

### 4.2 Arduino Mega — điều khiển động cơ & encoder ([`ARDUINO_MEGA.ino`](CONTROL WITH PLAYSTATION_SLAM/ARDUINO_MEGA/ARDUINO_MEGA.ino))
- `Serial.begin(115200)` → **gửi dữ liệu encoder lên PC ROS2**; `Serial1.begin(9600)` → **nhận lệnh từ ESP32**.
- **Chân & phần cứng:**
  | Chân / Module | Chức năng |
  |---|---|
  | Chân 3 / 2 | Hall L / R (ngắt `CHANGE`) |
  | Chân 4 / 5 | Brake L / R (phanh) |
  | Chân 6 / 7 | Dir L / R (đảo chiều) |
  | DAC MCP4725 `0x60` / `0x61` | Điều áp 2 động cơ (0–4095), không phải PWM |
- **Tốc độ** qua DAC: `current_speed = 1037` (ROBOT MODE), `current_speed_HM = 1200` (HUMAN MODE); vi chỉnh ±5 bằng lệnh `Q/A/E/D`.
- **Đổi chiều**: BRAKE → bật chân DIR → **delay 1500ms + 200ms** (bảo vệ mạch lái) → di chuyển.
- **2 chế độ**: `Y` = HUMAN MODE (tay cầm điều khiển), `T` = ROBOT MODE (tốc độ chuẩn để chạy SLAM).
- **Ngắt Hall** đếm `total_pulses_L/R`: **tăng khi DIR=0, giảm khi DIR=1** → biến đếm có thể **âm** khi lùi.
- **`DATA_ROS2()`**: cứ **50ms (~20Hz)** gửi 1 dòng `pulse_L,pulse_R\n` qua USB Serial lên PC.

> ✅ Đây chính là nguồn dữ liệu mà `arduino_odom_node` (phần 2) đọc. Vì `total_pulses_L/R` có thể **âm** khi lùi, node C++ dùng `std::stol` kiểu `long` để nhận giá trị âm là hợp lý.

### Bảng lệnh ký tự đầy đủ (phía firmware Mega nhận)
| Ký tự | Hành động |
|---|---|
| `F` / `B` / `L` / `R` | Tiến / Lùi / Cua trái / Cua phải |
| `S` | BRAKE (dừng) |
| `X` | RE_BRAKE (nhả phanh) |
| `Q` / `A` | Tăng / giảm tốc **phải** (±5 DAC) |
| `E` / `D` | Tăng / giảm tốc **trái** (±5 DAC) |
| `T` | ROBOT MODE (tốc độ chuẩn) |
| `Y` | HUMAN MODE (tốc độ nhanh) |

> ⚠️ **`T`/`Y` thực chất là chuyển chế độ ROBOT/HUMAN**, không phải "tăng/giảm tốc" như chú thích trong `cmd_vel_to_serial.py` (script map Button[0]→`T`, Button[2]→`Y` → khớp đúng là chuyển chế độ).

---

## 5. Package `rf2o_laser_odometry` — Laser Odometry (thư viện ngoài)

**Nguồn:** Thư viện từ bài báo ICRA 2016 "Planar Odometry from a Radial Laser Scanner. A Range Flow-based Approach" (MAPIR group, UMA).

**Vai trò:** Ước lượng odometry 2D chỉ dựa trên các scan laser liên tiếp (range-flow / dense scan alignment, không cần tìm correspondence) — dùng khi **wheel odometry không chính xác** (~0.9 ms/CPU core).

| Thuộc tính | Giá trị |
|---|---|
| Build | `ament_cmake`, C++14 |
| Executable | `rf2o_laser_odometry_node` |
| Node name | `rf2o_laser_odometry` |
| Phụ thuộc | Eigen3, Boost, tf2, nav_msgs... |

### Node & tham số ([`CLaserOdometry2DNode.cpp`](rf2o_laser_odometry/src/CLaserOdometry2DNode.cpp))
| Tham số | Default | Ý nghĩa |
|---|---|---|
| `laser_scan_topic` | `/scan` | sub scan laser |
| `odom_topic` | `/odom_rf2o` | publish odom |
| `base_frame_id` | `base_link` | child frame TF |
| `odom_frame_id` | `odom` | frame odom |
| `publish_tf` | true | có publish TF `odom→base_link` không |
| `init_pose_from_topic` | `/base_pose_ground_truth` | rỗng = khởi tạo từ (0,0) |
| `freq` | 10.0 (launch dùng 20) | tần số vòng lặp xử lý |

### Cách hoạt động
1. Sub scan (QoS best_effort, KeepLast 1).
2. Lần đầu: lấy TF `base_frame_id → frame scan` để đặt vị trí laser so với base (`setLaserPoseFromTf`), khởi tạo module.
3. Vòng lặp chính: cứ `freq` Hz gọi `odometryCalculation()` rồi `publish()`.
4. Publish Odometry + TF (nếu `publish_tf=true`). ⚠️ **Cảnh báo trong code:** chỉ nên 1 node publish TF `odom` — tránh xung đột với `arduino_odom_node` (cái này cũng publish `odom→base_footprint`).

> Trong [`real_mapping.launch.py`](my_robot_description/launch/real_mapping.launch.py:89) node này **đang bị comment** (dùng Arduino odometry thay thế). Có thể bật lại khi muốn dùng laser odometry.

**Thuật toán:** [`CLaserOdometry2D.cpp`](rf2o_laser_odometry/src/CLaserOdometry2D.cpp) (phần lõi, dense range-flow alignment).

---

## 6. Package `sllidar_ros2` — Driver RPLidar (thư viện ngoài)

**Nguồn:** Driver chính thức của **Slamtec** cho ROS2 (BSD license).

**Hỗ trợ các model:** RPLIDAR A1, A2, A3, **C1** (model đang dùng trong project), S1, S2, S2E, S3, T1.

| Thuộc tính | Giá trị |
|---|---|
| Build | `ament_cmake` (build cả SDK nhúng trong `sdk/`) |
| Executable | `sllidar_node` (driver chính), `sllidar_client` (test sub scan) |
| Node name | `sllidar_node` |

### Tham số node ([`sllidar_node.cpp`](sllidar_ros2/src/sllidar_node.cpp:67))
| Tham số | Default | Ghi chú |
|---|---|---|
| `channel_type` | serial | serial / tcp / udp |
| `serial_port` | `/dev/ttyUSB0` | cổng USB LiDAR |
| `serial_baudrate` | 1000000 | **C1 dùng 460800** (defaults 1M dành cho A-series) |
| `frame_id` | `laser_frame` | frame scan |
| `inverted` | false | đảo chiều dữ liệu |
| `angle_compensate` | false | bù góc (nên bật true) |
| `scan_mode` | (rỗng) | chọn mode scan nếu có |
| `scan_frequency` | 10 (serial) | tần số quét |

### Hoạt động & giao diện
1. Kết nối kênh (serial/tcp/udp) → đọc device info + check health.
2. Tạo **service** `stop_motor` / `start_motor` (`std_srvs/Empty`).
3. Start scan (dùng `scan_mode` nếu khai báo, không thì mode typical).
4. Loop: `grabScanDataHq` → sắp xếp tăng dần → nếu `angle_compensate` thì dựng mảng bù 1°/mẫu → **publish `sensor_msgs/LaserScan` lên topic `scan`** (QoS KeepLast 10).
5. Khi thoát: setMotorSpeed(0) + stop.

- [`sllidar_client.cpp`](sllidar_ros2/src/sllidar_client.cpp): node đơn giản sub `scan` và in từng điểm (angle-distance) ra console — dùng để test.

### Launch files (trong `launch/`)
- Mỗi model có 1 file launch riêng, VD: `sllidar_c1_launch.py`, `view_sllidar_a1_launch.py` (kèm rviz), v.v.
- [`sllidar_c1_launch.py`](sllidar_ros2/launch/sllidar_c1_launch.py): C1 với serial_port `/dev/ttyUSB0`, baud **460800**, frame_id `laser`, scan_mode `Standard`.

### udev rules ([`scripts/rplidar.rules`](sllidar_ros2/scripts/rplidar.rules))
```
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", SYMLINK+="rplidar"
```
Chạy `source scripts/create_udev_rules.sh` (hoặc `sudo chmod 777 /dev/ttyUSB0`) để cấp quyền đọc LiDAR.

---

## 7. 🔌 Sơ đồ luồng dữ liệu tổng thể

### A) Chế độ MÔ PHỎNG (Gazebo) — `gazebo.launch.py`
```
joy_node (tay cầm) → /joy
     → custom_joy_teleop → /diff_drive_controller/cmd_vel_unstamped
                                                  ↓
diff_drive_controller (gazebo_ros2_control) → joint velocity → Gazebo vật lý
                                                  ↓
joint_state_broadcaster → /joint_states → robot_state_publisher → TF
diff_drive_controller → /odom + TF(odom→base_footprint)
sensor ray Gazebo → /scan (laser_frame)
robot_state_publisher: TF base_footprint→base_link→laser_frame
Nav2 (map_server+amcl+planner+controller) chạy trên map my_map
```

### B) Chế độ XE THẬT — `real_mapping.launch.py`
```
sllidar_node (/dev/ttyUSB0, C1) ──→ /scan (laser_frame)
arduino_odom_node (/dev/ttyACM0, 115200) ──→ /odom + TF(odom→base_footprint)
        [đọc pulse từ Arduino Mega qua USB; Mega nhận lệnh điều khiển từ ESP32 qua Serial1]
static TF: base_link → laser_frame  (tf2 static_transform_publisher)
SLAM Toolbox (online_async, use_sim_time=false) ──→ /map (map->odom TF)
RViz hiển thị
```
Điều khiển xe thật có 2 đường đi, ĐỘC LẬP với nhau:
- **Tay cầm PS (chế độ HUMAN_MODE):** tay cầm → ESP32 (Bluepad32, Bluetooth) → UART 9600 → Mega Serial1 → motor. Không đi qua ROS2.
- **ROS2 joy (chế độ ROBOT_MODE):** `manual_control.launch.py` → joy_node → `/joy` → `cmd_vel_to_serial.py` → ký tự serial → (xem ⚠️ mục 8.8 — cổng đích đang nghi vấn).

### C) Các frame & topic chính
| TF tree | Ghi chú |
|---|---|
| `map` → `odom` | do SLAM Toolbox (mapping) / AMCL (Nav2) publish |
| `odom` → `base_footprint` | do `arduino_odom_node` HOẶC `diff_drive_controller` (Gazebo) |
| `base_footprint` → `base_link` | joint fixed, robot_state_publisher |
| `base_link` → `laser_frame` | fixed (URDF, x=-0.5) hoặc static TF xe thật (x=-0.46, z=0.15) |

| Topic quan trọng | Type |
|---|---|
| `/scan` | `sensor_msgs/LaserScan` |
| `/odom` | `nav_msgs/Odometry` |
| `/diff_drive_controller/cmd_vel_unstamped` | `geometry_msgs/Twist` |
| `/joy` | `sensor_msgs/Joy` |
| `/map`, `/map_updates` | SLAM toolbox |
| `/robot_description` | spawn Gazebo |

---

## 8. ⚠️ Điểm cần chú ý / có thể cần sửa

1. **2 node ROS2 cùng dùng chung cổng serial `/dev/ttyACM0`** (cổng USB của Arduino Mega):
   - `arduino_odom_node` (C++) — MỞ để ĐỌC pulse encoder, và `cmd_vel_to_serial.py` — MỞ để GHI lệnh. Nếu bật đồng thời dễ tranh chấp/treo cổng. (Chi tiết & hướng xử lý xem ⚠️ mục 8.8 bên dưới.)
2. **Thông số xe không thống nhất**:
   - `hall_reader_cpp`: wheel_radius 0.065, wheel_base 0.25, pulses 330 (hard-code, "đo lại" theo comment).
   - `diff_drive_controller.yaml` (Gazebo): wheel_radius 0.10795, wheel_separation 0.435.
   → Nếu dùng chung 1 xe thật thì 2 bộ số này phải khớp với nhau về thực tế.
3. **TF odom bị trùng nguồn publish:** Nếu bật cả `arduino_odom_node` lẫn rf2o (publish_tf=true) cùng lúc sẽ **tranh chấp TF `odom→...`**. Hiện rf2o đang comment trong real_mapping → OK.
4. **Frame đặt LiDAR khác nhau giữa mô phỏng và thật:** URDF `laser_joint` ở x=-0.5, z=0.26; còn static TF trong real_mapping là x=-0.46, z=0.15 → nên thống nhất.
5. **`use_sim_time`:** nav2_params và controller luôn bật `True`; khi chạy xe thật chỉ SLAM/rviz dùng `false`, nên kiểm tra node nào còn `use_sim_time=true` khi chạy thật (trong real_mapping không khởi động nav2 nên không ảnh hưởng).
6. **Camera.xacro** hiện không được include vào robot chính (parent `chassis` không tồn tại) — đang phát triển dở / tham khảo.
7. Trong `gazebo.launch.py` có nhiều dòng comment controller_manager/ros2_control_node cũ và dùng TimerAction trễ 4-7s cho spawner nhưng hiện ở LaunchDescription lại thêm thẳng không delay — phần này đang hơi lộn xộn, cần dọn nếu gặp lỗi timing.
8. **⚠️ Nghi vấn đường điều khiển ROS2 → xe thật (quan trọng nhất):**
   - Firmware hiện tại `ARDUINO_MEGA.ino` **chỉ đọc lệnh từ `Serial1`** (nối từ ESP32, 9600 baud). Cổng **USB Serial (`Serial`)** của Mega chỉ dùng để **GỬI pulse `L,R`** về PC cho `arduino_odom_node` đọc (115200).
   - Nhưng `cmd_vel_to_serial.py` lại **mở `/dev/ttyACM0` (USB) và GHI ký tự lệnh vào đó** → tức là gửi vào cổng USB mà Mega chỉ đọc data ra → **nhiều khả năng lệnh ROS2 KHÔNG tới được motor** với firmware hiện tại.
   - `cmd_vel_to_serial.py` có vẻ được viết cho **firmware cũ** (hồi Mega đọc lệnh ngay trên `Serial` USB). Nếu muốn điều khiển qua ROS2 thì cần: (a) sửa firmware để đọc lệnh chung trên `Serial`, HOẶC (b) chuyển `cmd_vel_to_serial.py` gửi xuống ESP32 (nối thêm UART PC→ESP32) để ESP32 relay xuống Mega.
   - Ngoài ra 2 node ROS2 hiện cùng dùng `/dev/ttyACM0` (đọc: `arduino_odom_node`, ghi: `cmd_vel_to_serial.py`) — dễ tranh chấp/treo cổng nếu bật đồng thời.
9. **Nút T/Y trên tay cầm là chuyển chế độ, không phải tăng/giảm tốc:**
   - Trong firmware Mega: `T` = ROBOT_MODE (HM=0, dùng `current_speed`=1037), `Y` = HUMAN_MODE (HM=1, dùng `current_speed_HM`=1200).
   - Trong `xbox.yaml`/`cmd_vel_to_serial.py`: Button[0]→`T`, Button[2]→`Y` (comment cũ ghi "tăng/giảm tốc" là **sai** so với firmware hiện tại).
10. **Encoder pulse có thể ÂM khi chạy lùi:** hàm ngắt `intrupt_L/R()` làm `pulse++` nếu DIR==0, ngược lại `pulse--`. `arduino_odom_node` dùng `std::stol` (long, có dấu) nên xử lý được — nhưng đừng dùng kiểu unsigned khi sửa code.
11. **Ghép nối kênh DAC trong firmware có vẻ đảo/ghi nhầm nhãn:**
    - `dac_R`(0x60) được gán biến `dac_out_L`, `dac_L`(0x61) gán `dac_out_R`; message setup cũng in nhầm "Khong tim thay ... Trai (0x60)" trong nhánh `dac_R`. Chỉ là nhãn tên — hoạt động vẫn ổn nếu đúng dây — nhưng khi sửa nên xác nhận lại thứ tự Trái/Phải theo thực tế.
12. **Chỉ có một "kênh lệnh" tại một thời điểm:** firmware Mega xử lý lệnh từng ký tự trong `loop()` (delay ~20ms + flush `Serial1`) — không nhận lệnh liên tục kiểu luồng vận tốc. Điều khiển kiểu PID/velocity theo ROS2 sẽ cần tần số gửi thấp & đổi firmware.
13. **Baud ESP32↔Mega là 9600** (Serial2 TX2=17/RX2=16 → Mega Serial1). Khi nối PC trực tiếp vào ESP32 để debug/test hãy để ý baud (PC↔Mega là 115200, PC↔ESP32 Bluepad32 debug cũng 115200).
14. **Kích thước tham chiếu:** `PULSES_PER_REV=330` trong `arduino_odom_node.cpp` là hard-code phải khớp với encoder hall thật (330 xung/rev/vòng motor). Kèm tỉ số giảm tốc hộp số mới ra số xung/vòng bánh xe — cần đo lại.

---

## 9. 🔨 Các lệnh build / chạy tham khảo

```bash
# Build toàn workspace (từ thư mục chứa src, VD: do_an_ws)
cd do_an_ws
colcon build --symlink-install
source install/setup.bash

# Xem robot trong rviz (không mô phỏng)
ros2 launch my_robot_description display.launch.py

# Mô phỏng Gazebo + Nav2
ros2 launch my_robot_description gazebo.launch.py

# Scan bản đồ với xe thật (RPLidar C1 + Arduino odom + SLAM toolbox)
ros2 launch my_robot_description real_mapping.launch.py

# Điều khiển bằng tay cầm
ros2 launch my_robot_description manual_control.launch.py   # xe thật (serial)
ros2 launch my_robot_description joystick.launch.py         # mô phỏng (cmd_vel)

# Chạy riêng driver LiDAR (test)
ros2 launch sllidar_ros2 sllidar_c1_launch.py
ros2 run sllidar_ros2 sllidar_client   # xem dữ liệu scan in ra console

# Chạy riêng odometry C++
ros2 run hall_reader_cpp arduino_odom_node

# Cấp quyền đọc LiDAR
sudo chmod 777 /dev/ttyUSB0
```

---

*File được tạo tự động bằng cách đọc toàn bộ workspace — mục đích tham khảo nhanh, không thay thế source gốc khi cần sửa code chi tiết.*
