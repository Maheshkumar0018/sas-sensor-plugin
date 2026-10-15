> **Reference only.** This is the original combined design. Theory (Section 2) is still valid.
> Scope, message definition, and processing parts are superseded by `docs/requirements.md` and
> `docs/decisions/0001`–`0006`. Do not implement Sections 6.3, 8, 9.2, 11.1–11.2 here.

# Synthetic Aperture Sonar (SAS) Plugin — Gazebo Classic 11 + ROS 2 Humble

Implementation guide for a physics-based SAS simulator. A Gazebo Classic sensor plugin generates
coherent, multi-element complex echoes. ROS 2 nodes form SAS images from those echoes.

Target stack: Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11 (`gazebo_ros_pkgs`), C++17, Python 3.10.

---

## 1. Feature List

| # | Feature | Where |
|---|---------|-------|
| F1 | LFM chirp transmitter (centre frequency, bandwidth, pulse length), complex baseband | `sas_sonar_plugin.cpp` |
| F2 | Multi-element along-track receiver array (N elements, spacing, element size) | `sas_sonar_plugin.cpp`, `sas_array.xacro` |
| F3 | Transmit and receive beam patterns (along-track and elevation `sinc` apertures) | `sas_sonar_plugin.cpp` |
| F4 | Scene geometry from GPU ray casting, surface reconstruction, acoustic shadows | `BuildScatterers()` |
| F5 | World-fixed coherent scatterer field with deterministic speckle (required for SAS focusing) | `BuildScatterers()` |
| F6 | Propagation: two-way spherical spreading, Thorp absorption, Lambertian backscatter | `Synthesize()` |
| F7 | Per-material reflectivity through SDF `<laser_retro>` | world file, `BuildScatterers()` |
| F8 | Continuous platform motion during propagation (no stop-and-hop), per element | `Synthesize()` |
| F9 | Raw echo output or range-compressed output, additive receiver noise | `Synthesize()` |
| F10 | Ground-truth pose and INS-drift navigation pose in every ping | `UpdateNavError()` |
| F11 | Kinematic AUV trajectory with motion errors (sway, heave, yaw) | `sas_trajectory_plugin.cpp` |
| F12 | ROS 2 interface: custom `sas_msgs/SasPing` message | `sas_msgs` |
| F13 | Time-domain backprojection imaging with motion compensation (CPU/NumPy or GPU/CuPy) | `sas_processing/backprojection_node.py` |
| F14 | Real-aperture sidescan waterfall for comparison | `sas_processing/waterfall_node.py` |
| F15 | Lockstep deterministic simulation and design checks (DPCA criterion, range ambiguity) | launch file, plugin |
| F16 | Validation procedure: point-spread function, motion-error tests | Section 11 |

---

## 2. SAS Theory Used by the Simulator

### 2.1 Signal model

Transmit pulse (complex baseband LFM):

$$p(t) = \exp\left(j\pi K \left(t - \tfrac{T_p}{2}\right)^2\right),\quad 0 \le t < T_p,\quad K = B/T_p$$

Received baseband signal on element *k*, sum over scatterers *i*:

$$s_k(t) = \sum_i a_{ik}\, p(t - \tau_{ik})\, e^{-j 2\pi f_c \tau_{ik}} + n_k(t)$$

Two-way delay with receiver motion during propagation (F8):

$$\tau_{ik} = \frac{\lVert \mathbf{P}_i - \mathbf{T} \rVert + \lVert \mathbf{P}_i - (\mathbf{R}_k + \mathbf{v}\,\tau_{ik}) \rVert}{c}$$

The plugin solves this with one fixed-point iteration. Error is below 1 µm at AUV speeds.

Amplitude:

$$a_{ik} = 10^{SL/20}\,\frac{\sqrt{\mu_i \cos^2\theta_i\,A_{cell}}}{R_{tx} R_{rx}}\, b_{tx}(\theta)\, b_{rx}(\theta)\, 10^{-\alpha (R_{tx}+R_{rx})/20}\,\xi_i$$

- `μ_i`: Lambert backscatter coefficient of the surface (F7).
- `θ_i`: incidence angle.
- `b(θ)`: aperture beam patterns, `sinc(D sinθ / λ)` (F3).
- `α`: Thorp absorption in dB/m (F6).
- `ξ_i`: unit-variance complex Gaussian speckle. It is a deterministic hash of the world cell index (F5).

### 2.2 Why the scatterer field must be world-fixed

SAS sums hundreds of pings coherently. A scatterer must keep the same position and the same complex
reflectivity in every ping. Ray hits move with the vehicle, so raw ray hits are not usable as scatterers.
The plugin reconstructs the surface from the ray image, then quantises it into world-fixed voxels of size
`cell_size`. Each voxel gets a position jitter and a speckle value from a hash of its integer index.
Every ping that sees the voxel then sees the same scatterer.

### 2.3 Design equations and default values

| Quantity | Equation | Default value |
|----------|----------|---------------|
| Wavelength | λ = c / f_c | 1500 / 100 kHz = 1.5 cm |
| Range resolution | δr = c / 2B | 1500 / 40 kHz = 3.75 cm |
| Along-track resolution | δx = D / 2 | 0.04 / 2 = 2 cm |
| Element beamwidth (null-to-null half angle) | sin θ = λ / D | 22° |
| Synthetic aperture length | L_sa = λ R / D | 15 m at R = 40 m |
| Max unambiguous range | R_max ≤ c · PRI / 2 | 60 m ≤ 75 m (PRF 10 Hz) |
| DPCA sampling criterion | v · PRI ≤ N d / 2 | 1.5 · 0.1 = 0.15 m ≤ 16 · 0.04 / 2 = 0.32 m |
| Complex sample rate | f_s ≥ B | 40 kHz (2× oversampled) |
| Samples per ping | N_s = ⌈2 R_max f_s / c⌉ | 3200 |
| Phase-error focus limit | sway error < λ / 8 | 1.9 mm |

The plugin warns at run time when the DPCA criterion or the range-ambiguity limit fails.

---

## 3. Workspace Layout

```
sas_ws/
└── src/
    ├── sas_msgs/                       # F12
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── msg/SasPing.msg
    ├── sas_gazebo/                     # F1–F11, F15
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── src/
    │       ├── sas_sonar_plugin.cpp
    │       └── sas_trajectory_plugin.cpp
    ├── sas_description/
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── urdf/
    │       ├── sas_array.xacro
    │       └── sas_auv.urdf.xacro
    ├── sas_processing/                 # F13, F14
    │   ├── package.xml
    │   ├── setup.py
    │   ├── setup.cfg
    │   ├── resource/sas_processing
    │   └── sas_processing/
    │       ├── __init__.py
    │       ├── common.py
    │       ├── backprojection_node.py
    │       └── waterfall_node.py
    └── sas_bringup/
        ├── CMakeLists.txt
        ├── package.xml
        ├── config/sas_processing.yaml
        ├── launch/sas_sim.launch.py
        └── worlds/sas_seabed.world
```

---

## 4. Prerequisites

```bash
sudo apt update
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-rqt-image-view libfftw3-dev python3-numpy
# Optional GPU backprojection (match your CUDA version):
# pip install cupy-cuda12x
```

The `gpu_ray` sensor needs an OpenGL-capable GPU and display (or a virtual display such as `xvfb-run`).
Without a GPU, launch with `sensor_type:=ray` (CPU ray casting; reduce ray counts, see Section 12).

---

## 5. Package `sas_msgs` (F12)

### `src/sas_msgs/msg/SasPing.msg`

```
# One SAS ping: complex baseband echoes for every receiver element.
std_msgs/Header header             # stamp = transmit time (sim time), frame_id = array frame

uint64 ping_id

# Waveform and sampling
float64 sound_speed                # m/s
float64 center_frequency           # Hz
float64 bandwidth                  # Hz
float64 pulse_length               # s
float64 sample_rate                # Hz, complex baseband
float64 t0                         # s, time of sample 0 relative to transmit
bool range_compressed              # true = matched filter already applied

# Array geometry in the array frame (x = boresight, y = along-track, z = up)
geometry_msgs/Point tx_position
geometry_msgs/Point[] rx_positions

# Platform state at transmit time
geometry_msgs/Pose pose_true       # array frame in world, ground truth
geometry_msgs/Pose pose_nav        # array frame in world, with simulated INS error
geometry_msgs/Vector3 velocity     # world-frame linear velocity of the array origin

# Echo data, element-major: index = k * num_samples + n
uint32 num_elements
uint32 num_samples
float32[] data_real
float32[] data_imag
```

### `src/sas_msgs/CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.8)
project(sas_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SasPing.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

### `src/sas_msgs/package.xml`

```xml
<?xml version="1.0"?>
<package format="3">
  <name>sas_msgs</name>
  <version>0.1.0</version>
  <description>Messages for the synthetic aperture sonar simulator.</description>
  <maintainer email="bojanapu.k@mgtechsoft.com">SAS Sim</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

---

## 6. Package `sas_gazebo` (F1–F11, F15)

### 6.1 Processing pipeline per ping

```
gpu_ray scan (range image + laser_retro)            ← Gazebo renders at update_rate = PRF
        │
        ▼
BuildScatterers()
  1. ray hits → world points
  2. grid quads → reject occlusion edges (incidence > max_incidence) → shadows
  3. subdivide quads at cell_size/2 → snap to world voxel → dedupe
  4. voxel hash → jitter position + complex speckle   (coherent across pings)
        │
        ▼
Synthesize()   (OpenMP, one thread per element)
  1. per scatterer: tx range, beam patterns, Lambert σ
  2. per element: delay with receiver motion, absorption, phase e^{-j2πf_cτ}
  3. band-limited fractional-delay impulse train  h_k[n]
  4. FFT · chirp spectrum  → raw echo  + noise
  5. optional matched filter (FFT · conj(chirp))
        │
        ▼
UpdateNavError() → sas_msgs/SasPing on /sas/ping
```

### 6.2 `src/sas_gazebo/src/sas_sonar_plugin.cpp`

```cpp
// Synthetic aperture sonar sensor plugin for Gazebo Classic 11 / ROS 2 Humble.
// Attach to a <sensor type="gpu_ray"> or <sensor type="ray">.

#include <fftw3.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstring>
#include <memory>
#include <random>
#include <string>
#include <unordered_set>
#include <vector>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/GpuRaySensor.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <rclcpp/rclcpp.hpp>
#include <sas_msgs/msg/sas_ping.hpp>

namespace sas_gazebo
{
using ignition::math::Pose3d;
using ignition::math::Quaterniond;
using ignition::math::Vector3d;
using cd = std::complex<double>;

namespace
{
inline uint64_t SplitMix64(uint64_t x)
{
  x += 0x9E3779B97F4A7C15ULL;
  x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ULL;
  x = (x ^ (x >> 27)) * 0x94D049BB133111EBULL;
  return x ^ (x >> 31);
}

// Uniform in (0, 1) from a 64-bit hash.
inline double U01(uint64_t h)
{
  return (static_cast<double>(h >> 11) + 0.5) * (1.0 / 9007199254740992.0);
}

inline double Sinc(double x)
{
  if (std::abs(x) < 1e-9) {
    return 1.0;
  }
  const double px = M_PI * x;
  return std::sin(px) / px;
}

// Thorp absorption. Input in Hz, output in dB/m.
inline double ThorpDbPerMeter(double f_hz)
{
  const double f = f_hz / 1000.0;
  const double f2 = f * f;
  const double db_per_km =
    0.11 * f2 / (1.0 + f2) + 44.0 * f2 / (4100.0 + f2) + 2.75e-4 * f2 + 0.003;
  return db_per_km / 1000.0;
}

// Pack a signed 3D voxel index into 63 bits (21 bits per axis).
inline uint64_t CellKey(int64_t ix, int64_t iy, int64_t iz)
{
  constexpr int64_t kOffset = 1 << 20;
  constexpr uint64_t kMask = (1ULL << 21) - 1;
  return ((static_cast<uint64_t>(ix + kOffset) & kMask) << 42) |
         ((static_cast<uint64_t>(iy + kOffset) & kMask) << 21) |
         (static_cast<uint64_t>(iz + kOffset) & kMask);
}

inline int CeilSamples(double seconds, double fs)
{
  return static_cast<int>(std::ceil(seconds * fs - 1e-9));
}
}  // namespace

// One range image from the ray sensor. Index = v * nh + h.
struct Scan
{
  int nh = 0;
  int nv = 0;
  double hmin = 0.0, hres = 0.0, vmin = 0.0, vres = 0.0;
  double rmin = 0.0, rmax = 0.0;
  std::vector<double> ranges;
  std::vector<double> retro;
};

// One world-fixed voxel scatterer.
struct Scatterer
{
  Vector3d pos;     // world position (voxel centre + deterministic jitter)
  Vector3d normal;  // unit surface normal, oriented toward the sonar
  double mu;        // Lambert backscatter coefficient (linear)
  cd speckle;       // unit-variance complex Gaussian, fixed per voxel
};

template<typename SensorT>
void ReadScan(const std::shared_ptr<SensorT> & s, Scan & scan)
{
  scan.nh = s->RangeCount();
  scan.nv = s->VerticalRangeCount();
  scan.hmin = s->AngleMin().Radian();
  scan.hres = s->AngleResolution();
  scan.vmin = s->VerticalAngleMin().Radian();
  scan.vres = s->VerticalAngleResolution();
  scan.rmin = s->RangeMin();
  scan.rmax = s->RangeMax();
  s->Ranges(scan.ranges);
  scan.retro.resize(scan.ranges.size());
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    scan.retro[i] = s->Retro(static_cast<int>(i));
  }
}

class SasSonarPlugin : public gazebo::SensorPlugin
{
public:
  ~SasSonarPlugin() override
  {
    update_conn_.reset();
    for (auto * b : buf_) {
      fftwf_free(b);
    }
    if (fwd_) {fftwf_destroy_plan(fwd_);}
    if (inv_) {fftwf_destroy_plan(inv_);}
  }

  void Load(gazebo::sensors::SensorPtr sensor, sdf::ElementPtr sdf) override
  {
    sensor_ = sensor;
    gpu_ray_ = std::dynamic_pointer_cast<gazebo::sensors::GpuRaySensor>(sensor);
    cpu_ray_ = std::dynamic_pointer_cast<gazebo::sensors::RaySensor>(sensor);
    if (!gpu_ray_ && !cpu_ray_) {
      gzerr << "[sas_sonar] Parent sensor must be of type gpu_ray or ray.\n";
      return;
    }

    world_ = gazebo::physics::get_world(sensor->WorldName());
    link_ = std::dynamic_pointer_cast<gazebo::physics::Link>(
      world_->EntityByName(sensor->ParentName()));
    if (!link_) {
      gzerr << "[sas_sonar] Parent link '" << sensor->ParentName() << "' not found.\n";
      return;
    }

    ros_node_ = gazebo_ros::Node::Get(sdf);

    auto get_d = [&](const std::string & key, double def) {
        return sdf->HasElement(key) ? sdf->Get<double>(key) : def;
      };
    auto get_i = [&](const std::string & key, int def) {
        return sdf->HasElement(key) ? sdf->Get<int>(key) : def;
      };
    auto get_b = [&](const std::string & key, bool def) {
        return sdf->HasElement(key) ? sdf->Get<bool>(key) : def;
      };

    // F1: waveform
    c_ = get_d("sound_speed", 1500.0);
    fc_ = get_d("center_frequency", 100e3);
    bw_ = get_d("bandwidth", 20e3);
    tp_ = get_d("pulse_length", 5e-3);
    fs_ = get_d("sample_rate", 40e3);
    rmin_ = get_d("min_range", 2.0);
    rmax_ = get_d("max_range", 60.0);
    // F2, F3: array
    n_el_ = get_i("num_elements", 16);
    d_el_ = get_d("element_spacing", 0.04);
    el_len_ = get_d("element_length", 0.04);
    el_h_ = get_d("element_height", 0.015);
    tx_len_ = get_d("tx_length", 0.04);
    tx_h_ = get_d("tx_height", 0.015);
    tx_pos_ = Vector3d(0.0, get_d("tx_offset_along_track", 0.0), 0.0);
    // F4–F7: scene
    cell_ = get_d("cell_size", 0.04);
    max_incidence_ = get_d("max_incidence_deg", 88.0) * M_PI / 180.0;
    mu_default_ = std::pow(10.0, get_d("default_backscatter_db", -27.0) / 10.0);
    sl_db_ = get_d("source_level_db", 0.0);
    alpha_db_m_ = sdf->HasElement("absorption_db_per_m") ?
      sdf->Get<double>("absorption_db_per_m") : ThorpDbPerMeter(fc_);
    // F9
    noise_std_ = get_d("noise_std", 1e-7);
    compressed_ = get_b("range_compressed", true);
    seed_ = static_cast<uint64_t>(get_i("seed", 12345));
    // F10
    nav_sway_rw_ = get_d("nav_sway_random_walk", 0.0);    // m / sqrt(s)
    nav_heave_rw_ = get_d("nav_heave_random_walk", 0.0);  // m / sqrt(s)
    nav_yaw_rw_ = get_d("nav_yaw_random_walk", 0.0);      // rad / sqrt(s)
    rng_.seed(seed_ ^ 0xA5A5A5A5ULL);
    frame_id_ = sdf->HasElement("frame_id") ?
      sdf->Get<std::string>("frame_id") : std::string("sas_array_link");

    rx_pos_.resize(n_el_);
    for (int k = 0; k < n_el_; ++k) {
      rx_pos_[k] = Vector3d(0.0, (k - 0.5 * (n_el_ - 1)) * d_el_, 0.0);
    }

    SetupDsp();

    // F15: design checks
    pri_ = 1.0 / sensor_->UpdateRate();
    if (2.0 * rmax_ / c_ > pri_) {
      RCLCPP_WARN(
        ros_node_->get_logger(),
        "Range ambiguity: 2*max_range/c = %.3f s > PRI = %.3f s. Lower PRF or max_range.",
        2.0 * rmax_ / c_, pri_);
    }

    pub_ = ros_node_->create_publisher<sas_msgs::msg::SasPing>(
      "ping", rclcpp::QoS(50).reliable());

    update_conn_ = sensor_->ConnectUpdated(std::bind(&SasSonarPlugin::OnScan, this));
    sensor_->SetActive(true);

    RCLCPP_INFO(
      ros_node_->get_logger(),
      "SAS sonar ready: fc=%.1f kHz B=%.1f kHz N=%d d=%.3f m PRF=%.2f Hz Ns=%d Nfft=%d "
      "cell=%.3f m alpha=%.4f dB/m",
      fc_ / 1e3, bw_ / 1e3, n_el_, d_el_, 1.0 / pri_, ns_, nfft_, cell_, alpha_db_m_);
  }

private:
  void SetupDsp()
  {
    ns_ = CeilSamples(2.0 * rmax_ / c_, fs_);
    np_ = CeilSamples(tp_, fs_);
    nfft_ = 1;
    while (nfft_ < ns_ + np_ + 8) {
      nfft_ <<= 1;
    }

    buf_.resize(n_el_);
    for (int k = 0; k < n_el_; ++k) {
      buf_[k] = static_cast<fftwf_complex *>(fftwf_malloc(sizeof(fftwf_complex) * nfft_));
    }
    // In-place plans; reused with fftwf_execute_dft on equally aligned buffers (thread-safe).
    fwd_ = fftwf_plan_dft_1d(nfft_, buf_[0], buf_[0], FFTW_FORWARD, FFTW_MEASURE);
    inv_ = fftwf_plan_dft_1d(nfft_, buf_[0], buf_[0], FFTW_BACKWARD, FFTW_MEASURE);

    // F1: LFM chirp spectrum
    std::memset(buf_[0], 0, sizeof(fftwf_complex) * nfft_);
    const double k_rate = bw_ / tp_;
    for (int n = 0; n < np_; ++n) {
      const double t = n / fs_ - 0.5 * tp_;
      const double ph = M_PI * k_rate * t * t;
      buf_[0][n][0] = static_cast<float>(std::cos(ph));
      buf_[0][n][1] = static_cast<float>(std::sin(ph));
    }
    fftwf_execute_dft(fwd_, buf_[0], buf_[0]);
    chirp_spec_.resize(nfft_);
    for (int n = 0; n < nfft_; ++n) {
      chirp_spec_[n] = std::complex<float>(buf_[0][n][0], buf_[0][n][1]);
    }
  }

  void OnScan()
  {
    if (gpu_ray_) {
      ReadScan(gpu_ray_, scan_);
    } else {
      ReadScan(cpu_ray_, scan_);
    }

    const auto sim_time = world_->SimTime();
    const Pose3d link_pose = link_->WorldPose();
    const Pose3d array_pose = sensor_->Pose() + link_pose;   // array frame in world
    const Vector3d vel = link_->WorldLinearVel(sensor_->Pose().Pos());

    // F15: DPCA along-track sampling check
    const double v_along = std::abs(vel.Dot(array_pose.Rot().RotateVector(Vector3d::UnitY)));
    if (v_along * pri_ > 0.5 * n_el_ * d_el_) {
      RCLCPP_WARN_THROTTLE(
        ros_node_->get_logger(), *ros_node_->get_clock(), 5000,
        "DPCA criterion violated: v*PRI = %.3f m > N*d/2 = %.3f m. Expect along-track aliasing.",
        v_along * pri_, 0.5 * n_el_ * d_el_);
    }

    BuildScatterers(scan_, array_pose);

    auto msg = std::make_unique<sas_msgs::msg::SasPing>();
    msg->header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(sim_time);
    msg->header.frame_id = frame_id_;
    msg->ping_id = ping_id_;
    msg->sound_speed = c_;
    msg->center_frequency = fc_;
    msg->bandwidth = bw_;
    msg->pulse_length = tp_;
    msg->sample_rate = fs_;
    msg->t0 = 0.0;
    msg->range_compressed = compressed_;
    msg->tx_position.x = tx_pos_.X();
    msg->tx_position.y = tx_pos_.Y();
    msg->tx_position.z = tx_pos_.Z();
    msg->rx_positions.resize(n_el_);
    for (int k = 0; k < n_el_; ++k) {
      msg->rx_positions[k].x = rx_pos_[k].X();
      msg->rx_positions[k].y = rx_pos_[k].Y();
      msg->rx_positions[k].z = rx_pos_[k].Z();
    }
    msg->pose_true = gazebo_ros::Convert<geometry_msgs::msg::Pose>(array_pose);
    msg->velocity = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(vel);
    msg->num_elements = n_el_;
    msg->num_samples = ns_;

    Synthesize(array_pose, vel, *msg);

    const double t = sim_time.Double();
    const double dt = last_time_ < 0.0 ? 0.0 : t - last_time_;
    last_time_ = t;
    msg->pose_nav = gazebo_ros::Convert<geometry_msgs::msg::Pose>(
      UpdateNavError(array_pose, link_pose, dt));

    pub_->publish(std::move(msg));
    ++ping_id_;
  }

  // F4, F5, F7: reconstruct the visible surface and quantise it into world-fixed voxels.
  void BuildScatterers(const Scan & s, const Pose3d & array_pose)
  {
    scatterers_.clear();
    const int nh = s.nh;
    const int nv = s.nv;
    if (nh < 2 || nv < 2) {
      return;
    }

    pts_.assign(static_cast<size_t>(nh) * nv, Vector3d::Zero);
    valid_.assign(static_cast<size_t>(nh) * nv, 0);
    const double r_hi = std::min(s.rmax * 0.999, rmax_);
    for (int j = 0; j < nv; ++j) {
      const double el = s.vmin + j * s.vres;
      for (int i = 0; i < nh; ++i) {
        const size_t idx = static_cast<size_t>(j) * nh + i;
        const double r = s.ranges[idx];
        if (!std::isfinite(r) || r >= r_hi || r <= std::max(s.rmin, rmin_)) {
          continue;
        }
        const double az = s.hmin + i * s.hres;
        const Vector3d p_local(
          r * std::cos(el) * std::cos(az), r * std::cos(el) * std::sin(az), r * std::sin(el));
        pts_[idx] = array_pose.CoordPositionAdd(p_local);
        valid_[idx] = 1;
      }
    }

    seen_.clear();
    seen_.reserve(1 << 21);
    const Vector3d origin = array_pose.Pos();
    const double step = 0.5 * cell_;
    const double cos_max = std::cos(max_incidence_);

    for (int j = 0; j < nv - 1; ++j) {
      for (int i = 0; i < nh - 1; ++i) {
        const size_t a = static_cast<size_t>(j) * nh + i;
        const size_t b = a + 1;
        const size_t c = a + nh;
        const size_t d = c + 1;
        if (!(valid_[a] && valid_[b] && valid_[c] && valid_[d])) {
          continue;
        }
        const Vector3d & p00 = pts_[a];
        const Vector3d & p10 = pts_[b];
        const Vector3d & p01 = pts_[c];
        const Vector3d & p11 = pts_[d];

        Vector3d n = (p11 - p00).Cross(p01 - p10);
        const double n_len = n.Length();
        if (n_len < 1e-12) {
          continue;
        }
        n /= n_len;
        const Vector3d centroid = 0.25 * (p00 + p10 + p01 + p11);
        const Vector3d view = (origin - centroid).Normalized();
        if (n.Dot(view) < 0.0) {
          n = -n;
        }
        // Quad almost parallel to the ray = depth discontinuity. Skipping it creates the shadow.
        if (n.Dot(view) < cos_max) {
          continue;
        }

        const double retro = 0.25 * (s.retro[a] + s.retro[b] + s.retro[c] + s.retro[d]);
        const double mu = retro > 0.0 ? retro : mu_default_;

        const int nu = std::clamp(
          static_cast<int>(std::ceil(std::max(p00.Distance(p10), p01.Distance(p11)) / step)),
          1, 512);
        const int nw = std::clamp(
          static_cast<int>(std::ceil(std::max(p00.Distance(p01), p10.Distance(p11)) / step)),
          1, 512);

        for (int u = 0; u < nu; ++u) {
          const double fu = (u + 0.5) / nu;
          for (int w = 0; w < nw; ++w) {
            const double fw = (w + 0.5) / nw;
            const Vector3d p = (1 - fu) * (1 - fw) * p00 + fu * (1 - fw) * p10 +
              (1 - fu) * fw * p01 + fu * fw * p11;
            const int64_t ix = static_cast<int64_t>(std::floor(p.X() / cell_));
            const int64_t iy = static_cast<int64_t>(std::floor(p.Y() / cell_));
            const int64_t iz = static_cast<int64_t>(std::floor(p.Z() / cell_));
            const uint64_t key = CellKey(ix, iy, iz);
            if (!seen_.insert(key).second) {
              continue;
            }
            // Deterministic per-voxel jitter and speckle (identical in every ping).
            const uint64_t h0 = SplitMix64(key ^ seed_);
            const uint64_t h1 = SplitMix64(h0);
            const uint64_t h2 = SplitMix64(h1);
            const uint64_t h3 = SplitMix64(h2);
            const uint64_t h4 = SplitMix64(h3);
            Scatterer sc;
            sc.pos = Vector3d(
              (ix + 0.5 + 0.8 * (U01(h0) - 0.5)) * cell_,
              (iy + 0.5 + 0.8 * (U01(h1) - 0.5)) * cell_,
              (iz + 0.5 + 0.8 * (U01(h2) - 0.5)) * cell_);
            sc.normal = n;
            sc.mu = mu;
            const double rad = std::sqrt(-std::log(U01(h3)));  // |ξ|² ~ Exp(1)
            const double ang = 2.0 * M_PI * U01(h4);
            sc.speckle = cd(rad * std::cos(ang), rad * std::sin(ang));
            scatterers_.push_back(sc);
          }
        }
      }
    }
  }

  // F1, F3, F6, F8, F9: synthesise per-element complex baseband echoes.
  void Synthesize(const Pose3d & pose, const Vector3d & vel, sas_msgs::msg::SasPing & msg)
  {
    const size_t n_sc = scatterers_.size();
    const Vector3d tx_w = pose.CoordPositionAdd(tx_pos_);
    std::vector<Vector3d> rx_w(n_el_);
    for (int k = 0; k < n_el_; ++k) {
      rx_w[k] = pose.CoordPositionAdd(rx_pos_[k]);
    }

    const double lambda = c_ / fc_;
    const double src_amp = std::pow(10.0, sl_db_ / 20.0);
    const double alpha_np = alpha_db_m_ * std::log(10.0) / 20.0;  // amplitude, nepers/m
    const double cell_area = cell_ * cell_;

    // Per-scatterer terms shared by all elements.
    pre_rtx_.resize(n_sc);
    pre_amp_.resize(n_sc);
    #pragma omp parallel for schedule(static)
    for (int64_t i = 0; i < static_cast<int64_t>(n_sc); ++i) {
      const Scatterer & sc = scatterers_[i];
      const Vector3d d = sc.pos - tx_w;
      const double r_tx = d.Length();
      pre_rtx_[i] = r_tx;
      pre_amp_[i] = 0.0;
      if (r_tx < rmin_ || r_tx > rmax_) {
        continue;
      }
      const Vector3d u = pose.Rot().RotateVectorReverse(d / r_tx);  // array frame
      if (u.X() <= 0.0) {
        continue;
      }
      const double sin_az = u.Y();
      const double sin_el = u.Z();
      const double b_tx = Sinc(tx_len_ * sin_az / lambda) * Sinc(tx_h_ * sin_el / lambda);
      const double b_rx = Sinc(el_len_ * sin_az / lambda) * Sinc(el_h_ * sin_el / lambda);
      const double cos_inc = std::max(0.0, -sc.normal.Dot(d / r_tx));
      const double sigma = sc.mu * cos_inc * cos_inc * cell_area;   // Lambert
      pre_amp_[i] = src_amp * std::sqrt(sigma) * b_tx * b_rx / r_tx;
    }

    msg.data_real.assign(static_cast<size_t>(n_el_) * ns_, 0.0f);
    msg.data_imag.assign(static_cast<size_t>(n_el_) * ns_, 0.0f);
    const float inv_nfft = 1.0f / static_cast<float>(nfft_);
    const float inv_mf = 1.0f / static_cast<float>(nfft_ * np_);

    #pragma omp parallel for schedule(dynamic)
    for (int k = 0; k < n_el_; ++k) {
      fftwf_complex * h = buf_[k];
      std::memset(h, 0, sizeof(fftwf_complex) * nfft_);
      const Vector3d r0 = rx_w[k];

      for (size_t i = 0; i < n_sc; ++i) {
        const double amp0 = pre_amp_[i];
        if (amp0 == 0.0) {
          continue;
        }
        const Vector3d & p = scatterers_[i].pos;
        const double r_tx = pre_rtx_[i];
        // F8: receiver keeps moving while the echo propagates.
        double tau = (r_tx + p.Distance(r0)) / c_;
        const double r_rx = p.Distance(r0 + vel * tau);
        tau = (r_tx + r_rx) / c_;

        const double amp = amp0 / r_rx * std::exp(-alpha_np * (r_tx + r_rx));
        const cd a = amp * scatterers_[i].speckle *
          std::polar(1.0, -2.0 * M_PI * fc_ * tau);

        // Band-limited fractional delay: 8-tap Hann-windowed sinc.
        const double nf = tau * fs_;
        const int n0 = static_cast<int>(std::floor(nf));
        if (n0 - 3 < 0 || n0 + 4 >= nfft_) {
          continue;
        }
        for (int m = n0 - 3; m <= n0 + 4; ++m) {
          const double x = nf - m;
          const double w = Sinc(x) * 0.5 * (1.0 + std::cos(M_PI * x / 4.0));
          h[m][0] += static_cast<float>(a.real() * w);
          h[m][1] += static_cast<float>(a.imag() * w);
        }
      }

      // Raw echo = impulse train convolved with the chirp.
      fftwf_execute_dft(fwd_, h, h);
      for (int n = 0; n < nfft_; ++n) {
        const std::complex<float> v = std::complex<float>(h[n][0], h[n][1]) * chirp_spec_[n];
        h[n][0] = v.real() * inv_nfft;
        h[n][1] = v.imag() * inv_nfft;
      }
      fftwf_execute_dft(inv_, h, h);

      // Receiver noise on the raw echo.
      std::mt19937_64 gen(seed_ ^ SplitMix64(ping_id_ * 1315423911ULL + k));
      std::normal_distribution<float> noise(0.0f, static_cast<float>(noise_std_));
      for (int n = 0; n < ns_; ++n) {
        h[n][0] += noise(gen);
        h[n][1] += noise(gen);
      }
      for (int n = ns_; n < nfft_; ++n) {
        h[n][0] = 0.0f;
        h[n][1] = 0.0f;
      }

      // Optional matched filter (peak amplitude normalised to scatterer amplitude).
      if (compressed_) {
        fftwf_execute_dft(fwd_, h, h);
        for (int n = 0; n < nfft_; ++n) {
          const std::complex<float> v =
            std::complex<float>(h[n][0], h[n][1]) * std::conj(chirp_spec_[n]);
          h[n][0] = v.real() * inv_mf;
          h[n][1] = v.imag() * inv_mf;
        }
        fftwf_execute_dft(inv_, h, h);
      }

      const size_t off = static_cast<size_t>(k) * ns_;
      for (int n = 0; n < ns_; ++n) {
        msg.data_real[off + n] = h[n][0];
        msg.data_imag[off + n] = h[n][1];
      }
    }
  }

  // F10: random-walk INS error on cross-track, heave and yaw.
  Pose3d UpdateNavError(const Pose3d & array_pose, const Pose3d & link_pose, double dt)
  {
    std::normal_distribution<double> n01(0.0, 1.0);
    const double sq = std::sqrt(std::max(dt, 0.0));
    err_y_ += nav_sway_rw_ * sq * n01(rng_);
    err_z_ += nav_heave_rw_ * sq * n01(rng_);
    err_yaw_ += nav_yaw_rw_ * sq * n01(rng_);
    const Vector3d offset = link_pose.Rot().RotateVector(Vector3d(0.0, err_y_, err_z_));
    const Quaterniond yaw_err(0.0, 0.0, err_yaw_);
    return Pose3d(array_pose.Pos() + offset, yaw_err * array_pose.Rot());
  }

  // Gazebo and ROS handles
  gazebo::sensors::SensorPtr sensor_;
  gazebo::sensors::GpuRaySensorPtr gpu_ray_;
  gazebo::sensors::RaySensorPtr cpu_ray_;
  gazebo::physics::WorldPtr world_;
  gazebo::physics::LinkPtr link_;
  gazebo::event::ConnectionPtr update_conn_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Publisher<sas_msgs::msg::SasPing>::SharedPtr pub_;

  // Parameters
  double c_{}, fc_{}, bw_{}, tp_{}, fs_{}, rmin_{}, rmax_{};
  int n_el_{};
  double d_el_{}, el_len_{}, el_h_{}, tx_len_{}, tx_h_{};
  Vector3d tx_pos_;
  std::vector<Vector3d> rx_pos_;
  double cell_{}, max_incidence_{}, mu_default_{}, sl_db_{}, alpha_db_m_{}, noise_std_{};
  bool compressed_{true};
  uint64_t seed_{};
  std::string frame_id_;
  double pri_{0.1};

  // Navigation error state
  double nav_sway_rw_{}, nav_heave_rw_{}, nav_yaw_rw_{};
  double err_y_{0.0}, err_z_{0.0}, err_yaw_{0.0};
  std::mt19937_64 rng_;

  // DSP
  int ns_{}, np_{}, nfft_{};
  fftwf_plan fwd_{nullptr};
  fftwf_plan inv_{nullptr};
  std::vector<std::complex<float>> chirp_spec_;
  std::vector<fftwf_complex *> buf_;

  // Per-ping work buffers
  Scan scan_;
  std::vector<Vector3d> pts_;
  std::vector<uint8_t> valid_;
  std::unordered_set<uint64_t> seen_;
  std::vector<Scatterer> scatterers_;
  std::vector<double> pre_rtx_, pre_amp_;
  uint64_t ping_id_{0};
  double last_time_{-1.0};
};

GZ_REGISTER_SENSOR_PLUGIN(SasSonarPlugin)
}  // namespace sas_gazebo
```

### 6.3 `src/sas_gazebo/src/sas_trajectory_plugin.cpp` (F11)

Kinematic AUV motion. Straight survey leg plus optional sinusoidal sway, heave and yaw errors.
Motion is deterministic, so focus tests are repeatable.

```cpp
// Kinematic survey trajectory with configurable motion errors.

#include <algorithm>
#include <cmath>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>

namespace sas_gazebo
{
using ignition::math::Pose3d;
using ignition::math::Quaterniond;
using ignition::math::Vector3d;

class SasTrajectoryPlugin : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    auto get = [&](const std::string & key, double def) {
        return sdf->HasElement(key) ? sdf->Get<double>(key) : def;
      };
    start_ = Vector3d(get("start_x", -10.0), get("start_y", 0.0), get("start_z", 10.0));
    heading_ = get("heading", 0.0);
    speed_ = get("speed", 1.5);
    duration_ = get("duration", 60.0);
    sway_a_ = get("sway_amplitude", 0.0);
    sway_t_ = get("sway_period", 20.0);
    heave_a_ = get("heave_amplitude", 0.0);
    heave_t_ = get("heave_period", 15.0);
    yaw_a_ = get("yaw_amplitude", 0.0);
    yaw_t_ = get("yaw_period", 25.0);

    model_->SetGravityMode(false);
    t_start_ = model_->GetWorld()->SimTime().Double();
    conn_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&SasTrajectoryPlugin::OnUpdate, this, std::placeholders::_1));
  }

private:
  static double Osc(double a, double period, double t)
  {
    return period > 0.0 ? a * std::sin(2.0 * M_PI * t / period) : 0.0;
  }

  static double OscRate(double a, double period, double t)
  {
    return period > 0.0 ? a * 2.0 * M_PI / period * std::cos(2.0 * M_PI * t / period) : 0.0;
  }

  void OnUpdate(const gazebo::common::UpdateInfo & info)
  {
    const double t_raw = info.simTime.Double() - t_start_;
    const bool moving = t_raw < duration_;
    const double t = std::clamp(t_raw, 0.0, duration_);

    const double ch = std::cos(heading_);
    const double sh = std::sin(heading_);
    const double along = speed_ * t;
    const double sway = Osc(sway_a_, sway_t_, t);
    const double heave = Osc(heave_a_, heave_t_, t);
    const double yaw = heading_ + Osc(yaw_a_, yaw_t_, t);

    const Vector3d pos(
      start_.X() + ch * along - sh * sway,
      start_.Y() + sh * along + ch * sway,
      start_.Z() + heave);
    model_->SetWorldPose(Pose3d(pos, Quaterniond(0.0, 0.0, yaw)));

    if (moving) {
      const double v_along = speed_;
      const double v_sway = OscRate(sway_a_, sway_t_, t);
      model_->SetLinearVel(Vector3d(
        ch * v_along - sh * v_sway, sh * v_along + ch * v_sway, OscRate(heave_a_, heave_t_, t)));
      model_->SetAngularVel(Vector3d(0.0, 0.0, OscRate(yaw_a_, yaw_t_, t)));
    } else {
      model_->SetLinearVel(Vector3d::Zero);
      model_->SetAngularVel(Vector3d::Zero);
    }
  }

  gazebo::physics::ModelPtr model_;
  gazebo::event::ConnectionPtr conn_;
  Vector3d start_;
  double heading_{}, speed_{}, duration_{}, t_start_{};
  double sway_a_{}, sway_t_{}, heave_a_{}, heave_t_{}, yaw_a_{}, yaw_t_{};
};

GZ_REGISTER_MODEL_PLUGIN(SasTrajectoryPlugin)
}  // namespace sas_gazebo
```

### 6.4 `src/sas_gazebo/CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.8)
project(sas_gazebo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sas_msgs REQUIRED)
find_package(OpenMP REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(FFTW3F REQUIRED fftw3f)

add_library(sas_sonar_plugin SHARED src/sas_sonar_plugin.cpp)
target_include_directories(sas_sonar_plugin PRIVATE ${FFTW3F_INCLUDE_DIRS})
ament_target_dependencies(sas_sonar_plugin
  gazebo_dev gazebo_ros rclcpp geometry_msgs sas_msgs)
target_link_libraries(sas_sonar_plugin ${FFTW3F_LINK_LIBRARIES} OpenMP::OpenMP_CXX)

add_library(sas_trajectory_plugin SHARED src/sas_trajectory_plugin.cpp)
ament_target_dependencies(sas_trajectory_plugin gazebo_dev)

install(TARGETS sas_sonar_plugin sas_trajectory_plugin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

ament_package()
```

### 6.5 `src/sas_gazebo/package.xml`

```xml
<?xml version="1.0"?>
<package format="3">
  <name>sas_gazebo</name>
  <version>0.1.0</version>
  <description>Gazebo Classic synthetic aperture sonar sensor and trajectory plugins.</description>
  <maintainer email="bojanapu.k@mgtechsoft.com">SAS Sim</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>gazebo_dev</depend>
  <depend>gazebo_ros</depend>
  <depend>rclcpp</depend>
  <depend>geometry_msgs</depend>
  <depend>sas_msgs</depend>
  <!-- FFTW3 (single precision): sudo apt install libfftw3-dev -->

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

---

## 7. Package `sas_description` (F2, F11)

### 7.1 Array geometry and frames

```
           world z (up)
              │       vehicle heading = +x
              │
   ───────────●──────────────►  x   (along-track)
             /│
            / │   sas_array_link (rpy = 0, 0.35, -π/2 from base_link)
           /  │     x_array = boresight: starboard and 20° down
          ▼   │     y_array = along-track (+x world)
   starboard  │     z_array = up-ish
      (-y)

  Receiver k at y_array = (k - (N-1)/2) · d,   k = 0 … N-1
  Transmitter at y_array = tx_offset_along_track
```

Ray fan in the array frame:

- Horizontal angles (about z_array) span along-track: ±0.40 rad, covering the 22° element beam.
- Vertical angles (about y_array) span elevation: −0.80 … +0.20 rad, equal to 9°–66° depression.
  At 10 m altitude this is approximately 5 m–65 m ground range.

### 7.2 `src/sas_description/urdf/sas_array.xacro`

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="sas_array"
               params="parent xyz rpy sensor_type:=gpu_ray prf:=10.0
                       h_samples:=512 v_samples:=512
                       num_elements:=16 element_spacing:=0.04 range_compressed:=true">

    <link name="sas_array_link">
      <inertial>
        <mass value="0.5"/>
        <inertia ixx="1e-3" ixy="0" ixz="0" iyy="1e-3" iyz="0" izz="1e-3"/>
      </inertial>
      <visual>
        <geometry><box size="0.04 ${num_elements * element_spacing} 0.04"/></geometry>
      </visual>
    </link>

    <joint name="sas_array_joint" type="fixed">
      <parent link="${parent}"/>
      <child link="sas_array_link"/>
      <origin xyz="${xyz}" rpy="${rpy}"/>
    </joint>

    <gazebo reference="sas_array_link">
      <sensor type="${sensor_type}" name="sas_sonar">
        <always_on>true</always_on>
        <update_rate>${prf}</update_rate>
        <visualize>false</visualize>
        <ray>
          <scan>
            <horizontal>
              <samples>${h_samples}</samples>
              <resolution>1</resolution>
              <min_angle>-0.40</min_angle>
              <max_angle>0.40</max_angle>
            </horizontal>
            <vertical>
              <samples>${v_samples}</samples>
              <resolution>1</resolution>
              <min_angle>-0.80</min_angle>
              <max_angle>0.20</max_angle>
            </vertical>
          </scan>
          <range>
            <min>1.0</min>
            <max>70.0</max>
            <resolution>0.001</resolution>
          </range>
        </ray>

        <plugin name="sas_sonar_plugin" filename="libsas_sonar_plugin.so">
          <ros>
            <namespace>/sas</namespace>
          </ros>
          <frame_id>sas_array_link</frame_id>

          <!-- F1: waveform -->
          <sound_speed>1500.0</sound_speed>
          <center_frequency>100000.0</center_frequency>
          <bandwidth>20000.0</bandwidth>
          <pulse_length>0.005</pulse_length>
          <sample_rate>40000.0</sample_rate>
          <min_range>2.0</min_range>
          <max_range>60.0</max_range>

          <!-- F2, F3: array -->
          <num_elements>${num_elements}</num_elements>
          <element_spacing>${element_spacing}</element_spacing>
          <element_length>0.04</element_length>
          <element_height>0.015</element_height>
          <tx_length>0.04</tx_length>
          <tx_height>0.015</tx_height>
          <tx_offset_along_track>0.0</tx_offset_along_track>

          <!-- F4–F7: scene -->
          <cell_size>0.04</cell_size>
          <max_incidence_deg>88.0</max_incidence_deg>
          <default_backscatter_db>-27.0</default_backscatter_db>
          <source_level_db>0.0</source_level_db>
          <!-- <absorption_db_per_m>0.034</absorption_db_per_m>  default = Thorp(fc) -->

          <!-- F9 -->
          <range_compressed>${range_compressed}</range_compressed>
          <noise_std>1e-7</noise_std>
          <seed>12345</seed>

          <!-- F10: INS error (random walk per sqrt(s)) -->
          <nav_sway_random_walk>0.0005</nav_sway_random_walk>
          <nav_heave_random_walk>0.0002</nav_heave_random_walk>
          <nav_yaw_random_walk>0.00005</nav_yaw_random_walk>
        </plugin>
      </sensor>
    </gazebo>
  </xacro:macro>
</robot>
```

Fixed joints are lumped by Gazebo. The sensor then belongs to `base_link` with the joint offset as its
pose. The plugin uses `sensor->Pose() + link->WorldPose()`, so lumping does not change the result.

### 7.3 `src/sas_description/urdf/sas_auv.urdf.xacro`

```xml
<?xml version="1.0"?>
<robot name="sas_auv" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:arg name="sensor_type" default="gpu_ray"/>
  <xacro:arg name="prf" default="10.0"/>
  <xacro:arg name="speed" default="1.5"/>
  <xacro:arg name="duration" default="60.0"/>
  <xacro:arg name="sway_amplitude" default="0.0"/>
  <xacro:arg name="heave_amplitude" default="0.0"/>
  <xacro:arg name="yaw_amplitude" default="0.0"/>

  <xacro:include filename="$(find sas_description)/urdf/sas_array.xacro"/>

  <link name="base_link">
    <inertial>
      <mass value="100.0"/>
      <inertia ixx="1.3" ixy="0" ixz="0" iyy="52.7" iyz="0" izz="52.7"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 1.5708 0"/>
      <geometry><cylinder radius="0.16" length="2.5"/></geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 1.5708 0"/>
      <geometry><cylinder radius="0.16" length="2.5"/></geometry>
    </collision>
  </link>

  <!-- Starboard array, boresight 20° (0.35 rad) below horizontal -->
  <xacro:sas_array parent="base_link" xyz="0.3 -0.17 0" rpy="0 0.35 -1.5708"
                   sensor_type="$(arg sensor_type)" prf="$(arg prf)"/>

  <gazebo reference="base_link">
    <gravity>false</gravity>
  </gazebo>

  <!-- F11: kinematic survey leg along +x at 10 m altitude -->
  <gazebo>
    <plugin name="sas_trajectory" filename="libsas_trajectory_plugin.so">
      <start_x>-10.0</start_x>
      <start_y>0.0</start_y>
      <start_z>10.0</start_z>
      <heading>0.0</heading>
      <speed>$(arg speed)</speed>
      <duration>$(arg duration)</duration>
      <sway_amplitude>$(arg sway_amplitude)</sway_amplitude>
      <sway_period>20.0</sway_period>
      <heave_amplitude>$(arg heave_amplitude)</heave_amplitude>
      <heave_period>15.0</heave_period>
      <yaw_amplitude>$(arg yaw_amplitude)</yaw_amplitude>
      <yaw_period>25.0</yaw_period>
    </plugin>
  </gazebo>
</robot>
```

### 7.4 `src/sas_description/CMakeLists.txt` and `package.xml`

```cmake
cmake_minimum_required(VERSION 3.8)
project(sas_description)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
ament_package()
```

```xml
<?xml version="1.0"?>
<package format="3">
  <name>sas_description</name>
  <version>0.1.0</version>
  <description>AUV and SAS array description.</description>
  <maintainer email="bojanapu.k@mgtechsoft.com">SAS Sim</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>sas_gazebo</exec_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
```

---

## 8. Package `sas_processing` (F13, F14)

### 8.1 `sas_processing/common.py`

```python
"""Shared helpers: message decoding, geometry, matched filter, upsampling."""
import numpy as np


def quat_to_rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def pose_to_rt(pose):
    p = pose.position
    return quat_to_rot(pose.orientation), np.array([p.x, p.y, p.z])


def element_positions(msg, pose):
    """World positions of the transmitter and every receiver for the given array pose."""
    rot, t = pose_to_rt(pose)
    tx = rot @ np.array([msg.tx_position.x, msg.tx_position.y, msg.tx_position.z]) + t
    rx = np.array([[p.x, p.y, p.z] for p in msg.rx_positions]) @ rot.T + t
    return rot, tx, rx


def ping_data(msg):
    real = np.asarray(msg.data_real, dtype=np.float32)
    imag = np.asarray(msg.data_imag, dtype=np.float32)
    return (real + 1j * imag).reshape(msg.num_elements, msg.num_samples)


def lfm_chirp(msg):
    """Identical to the chirp generated in the Gazebo plugin."""
    n = int(np.ceil(msg.pulse_length * msg.sample_rate - 1e-9))
    t = np.arange(n) / msg.sample_rate - 0.5 * msg.pulse_length
    k = msg.bandwidth / msg.pulse_length
    return np.exp(1j * np.pi * k * t * t).astype(np.complex64)


def matched_filter(data, msg):
    """Range-compress raw echoes. Pass-through if the plugin already compressed them."""
    if msg.range_compressed:
        return data
    p = lfm_chirp(msg)
    ns = data.shape[-1]
    nfft = 1 << int(np.ceil(np.log2(ns + len(p) + 8)))
    spec = np.fft.fft(data, nfft, axis=-1) * np.conj(np.fft.fft(p, nfft))
    return (np.fft.ifft(spec, axis=-1)[..., :ns] / len(p)).astype(np.complex64)


def upsample(x, factor):
    """FFT interpolation of complex baseband along the last axis."""
    if factor <= 1:
        return x
    n = x.shape[-1]
    spec = np.fft.fft(x, axis=-1)
    half = n // 2
    out = np.zeros(x.shape[:-1] + (n * factor,), dtype=np.complex64)
    out[..., :half] = spec[..., :half]
    out[..., -(n - half):] = spec[..., half:]
    return (np.fft.ifft(out, axis=-1) * factor).astype(np.complex64)
```

### 8.2 `sas_processing/backprojection_node.py` (F13)

Time-domain backprojection onto a flat seabed grid at `z_plane`. Each ping adds into a complex
image. Each pixel uses the exact bistatic delay with receiver motion (same model as the plugin), then
removes the carrier phase with `exp(+j2πf_cτ)`. Set `use_nav_pose: true` to image with the drifting
INS pose and see motion-error defocus.

```python
"""Time-domain backprojection SAS imaging node."""
import queue
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

from sas_msgs.msg import SasPing
from sas_processing.common import element_positions, matched_filter, ping_data, upsample

try:
    import cupy
except ImportError:
    cupy = None


class Backprojection(Node):

    def __init__(self):
        super().__init__('sas_backprojection')
        p = self.declare_parameter
        self.x_min = p('x_min', 15.0).value
        self.x_max = p('x_max', 45.0).value
        self.y_min = p('y_min', -40.0).value
        self.y_max = p('y_max', -10.0).value
        self.z_plane = p('z_plane', 0.0).value
        self.pixel = p('pixel_size', 0.03).value
        self.use_nav = p('use_nav_pose', False).value
        self.up = p('upsample', 4).value
        self.max_squint = np.deg2rad(p('max_squint_deg', 25.0).value)
        self.dyn_range = p('dynamic_range_db', 40.0).value
        self.output_path = p('output_path', '/tmp/sas_image.npy').value
        use_gpu = p('use_gpu', False).value
        topic = p('ping_topic', '/sas/ping').value
        period = p('publish_period', 2.0).value

        self.xp = cupy if (use_gpu and cupy is not None) else np
        if use_gpu and cupy is None:
            self.get_logger().warn('use_gpu=true but cupy is not installed. Using NumPy.')
        xp = self.xp

        xs = np.arange(self.x_min, self.x_max, self.pixel)
        ys = np.arange(self.y_min, self.y_max, self.pixel)
        self.nx, self.ny = len(xs), len(ys)
        gx, gy = np.meshgrid(xs, ys)
        grid = np.stack([gx.ravel(), gy.ravel(), np.full(gx.size, self.z_plane)], axis=1)
        self.grid = xp.asarray(grid)
        self.image = xp.zeros(self.nx * self.ny, dtype=xp.complex64)
        self.lock = threading.Lock()
        self.pings = 0

        self.queue = queue.Queue()
        self.worker = threading.Thread(target=self.run, daemon=True)
        self.worker.start()

        self.sub = self.create_subscription(SasPing, topic, self.queue.put, 100)
        self.pub = self.create_publisher(Image, '/sas/image', 1)
        self.timer = self.create_timer(period, self.publish)
        self.create_service(Trigger, '~/save', self.on_save)
        self.create_service(Trigger, '~/reset', self.on_reset)
        self.get_logger().info(
            f'Grid {self.nx} x {self.ny} px, pixel {self.pixel} m, '
            f'backend {"cupy" if xp is not np else "numpy"}')

    def run(self):
        while rclpy.ok():
            msg = self.queue.get()
            try:
                self.process(msg)
            except Exception as exc:  # keep the worker alive
                self.get_logger().error(f'ping {msg.ping_id}: {exc}')

    def process(self, msg):
        xp = self.xp
        pose = msg.pose_nav if self.use_nav else msg.pose_true
        rot, tx, rx = element_positions(msg, pose)
        data = upsample(matched_filter(ping_data(msg), msg), self.up)
        data = xp.asarray(data)
        fs = msg.sample_rate * self.up
        c, f0, t0 = msg.sound_speed, msg.center_frequency, msg.t0
        n = data.shape[1]
        r_max = 0.5 * c * (t0 + msg.num_samples / msg.sample_rate)
        vel = xp.asarray([msg.velocity.x, msg.velocity.y, msg.velocity.z])

        # Footprint mask: in front of the array and inside the processed beam.
        d = self.grid - xp.asarray(tx)
        r_tx = xp.linalg.norm(d, axis=1)
        u = (d @ xp.asarray(rot)) / r_tx[:, None]      # direction in the array frame
        mask = (u[:, 0] > 0) & (xp.abs(u[:, 1]) < np.sin(self.max_squint)) & (r_tx < r_max)
        idx = xp.nonzero(mask)[0]
        if idx.size == 0:
            return
        pts = self.grid[idx]
        r_tx = r_tx[idx]
        acc = xp.zeros(idx.size, dtype=xp.complex64)

        for k in range(rx.shape[0]):
            r0 = xp.asarray(rx[k])
            tau = (r_tx + xp.linalg.norm(pts - r0, axis=1)) / c
            r_rx = xp.linalg.norm(pts - (r0 + tau[:, None] * vel), axis=1)
            tau = (r_tx + r_rx) / c
            s = (tau - t0) * fs
            i0 = xp.floor(s).astype(xp.int64)
            frac = (s - i0).astype(xp.float32)
            ok = (i0 >= 0) & (i0 < n - 1)
            i0 = xp.clip(i0, 0, n - 2)
            val = data[k, i0] * (1 - frac) + data[k, i0 + 1] * frac
            val = xp.where(ok, val, 0)
            acc += (val * xp.exp(1j * 2 * np.pi * f0 * tau)).astype(xp.complex64)

        with self.lock:
            self.image[idx] += acc
            self.pings += 1

    def to_host(self, arr):
        return cupy.asnumpy(arr) if self.xp is not np else arr

    def publish(self):
        with self.lock:
            mag = np.abs(self.to_host(self.image)).reshape(self.ny, self.nx)
        peak = mag.max()
        if peak <= 0:
            return
        db = 20 * np.log10(mag / peak + 1e-12)
        img = np.clip((db + self.dyn_range) / self.dyn_range, 0, 1)
        img = (255 * img[::-1]).astype(np.uint8)  # row 0 = largest y
        out = Image()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'world'
        out.height, out.width = img.shape
        out.encoding = 'mono8'
        out.step = out.width
        out.data = img.tobytes()
        self.pub.publish(out)

    def on_save(self, _req, res):
        with self.lock:
            img = self.to_host(self.image).reshape(self.ny, self.nx)
        np.save(self.output_path, img)
        res.success = True
        res.message = f'saved {self.pings} pings to {self.output_path}'
        return res

    def on_reset(self, _req, res):
        with self.lock:
            self.image[:] = 0
            self.pings = 0
        res.success = True
        res.message = 'image cleared'
        return res


def main():
    rclpy.init()
    node = Backprojection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
```

### 8.3 `sas_processing/waterfall_node.py` (F14)

Real-aperture sidescan reference: coherent broadside beam over all elements, slant-to-ground range
correction, time-varying gain, one row per ping.

```python
"""Real-aperture sidescan waterfall from SAS pings."""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from sas_msgs.msg import SasPing
from sas_processing.common import matched_filter, ping_data


class Waterfall(Node):

    def __init__(self):
        super().__init__('sas_waterfall')
        p = self.declare_parameter
        self.z_plane = p('z_plane', 0.0).value
        self.ground_res = p('ground_resolution', 0.05).value
        self.max_ground = p('max_ground_range', 60.0).value
        self.rows = p('rows', 600).value
        self.tvg = p('tvg_exponent', 2.0).value
        topic = p('ping_topic', '/sas/ping').value

        self.ground = np.arange(0.0, self.max_ground, self.ground_res)
        self.buffer = []
        self.sub = self.create_subscription(SasPing, topic, self.on_ping, 100)
        self.pub = self.create_publisher(Image, '/sas/waterfall', 1)

    def on_ping(self, msg):
        data = matched_filter(ping_data(msg), msg)
        beam = np.abs(data.sum(axis=0))
        slant = 0.5 * msg.sound_speed * (msg.t0 + np.arange(msg.num_samples) / msg.sample_rate)
        altitude = max(msg.pose_true.position.z - self.z_plane, 0.1)
        r = np.sqrt(self.ground ** 2 + altitude ** 2)
        line = np.interp(r, slant, beam, left=0.0, right=0.0) * r ** self.tvg

        self.buffer.append(line.astype(np.float32))
        self.buffer = self.buffer[-self.rows:]
        img = np.stack(self.buffer[::-1])  # newest ping on top
        scale = np.percentile(img, 99.5) or 1.0
        img = (255 * np.clip(img / scale, 0, 1)).astype(np.uint8)

        out = Image()
        out.header = msg.header
        out.height, out.width = img.shape
        out.encoding = 'mono8'
        out.step = out.width
        out.data = img.tobytes()
        self.pub.publish(out)


def main():
    rclpy.init()
    node = Waterfall()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
```

### 8.4 Packaging

`setup.py`:

```python
from setuptools import setup

package_name = 'sas_processing'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SAS Sim',
    maintainer_email='bojanapu.k@mgtechsoft.com',
    description='SAS image formation and sidescan waterfall nodes.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sas_backprojection = sas_processing.backprojection_node:main',
            'sas_waterfall = sas_processing.waterfall_node:main',
        ],
    },
)
```

`setup.cfg`:

```ini
[develop]
script_dir=$base/lib/sas_processing
[install]
install_scripts=$base/lib/sas_processing
```

`package.xml`:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>sas_processing</name>
  <version>0.1.0</version>
  <description>SAS image formation and sidescan waterfall nodes.</description>
  <maintainer email="bojanapu.k@mgtechsoft.com">SAS Sim</maintainer>
  <license>Apache-2.0</license>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>std_srvs</exec_depend>
  <exec_depend>sas_msgs</exec_depend>
  <exec_depend>python3-numpy</exec_depend>
  <export><build_type>ament_python</build_type></export>
</package>
```

Create the empty marker file `resource/sas_processing` and an empty `sas_processing/__init__.py`.

---

## 9. Package `sas_bringup`

### 9.1 `worlds/sas_seabed.world` (F7)

`<laser_retro>` on each visual sets the Lambert coefficient μ (linear) seen by the `gpu_ray` sensor.
For the CPU `ray` sensor, put the same `<laser_retro>` on the collision elements.
Value 0 means "use `default_backscatter_db`".

| Surface | laser_retro | μ (dB) |
|---------|-------------|--------|
| Sand seabed | 0.002 | −27 |
| Rock | 0.02 | −17 |
| Metal target | 0.3 | −5 |
| Point target | 1.0 | 0 |

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="sas_world">
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <scene>
      <ambient>0.2 0.3 0.4 1</ambient>
      <background>0.0 0.1 0.2 1</background>
      <shadows>false</shadows>
    </scene>
    <include><uri>model://sun</uri></include>

    <!-- Sand seabed, top surface at z = 0 -->
    <model name="seabed">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <pose>0 0 -0.05 0 0 0</pose>
          <geometry><box><size>400 400 0.1</size></box></geometry>
        </collision>
        <visual name="visual">
          <pose>0 0 -0.05 0 0 0</pose>
          <geometry><box><size>400 400 0.1</size></box></geometry>
          <laser_retro>0.002</laser_retro>
          <material><ambient>0.76 0.70 0.50 1</ambient><diffuse>0.76 0.70 0.50 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- Cylinder (mine-like), 0.5 m diameter, 2 m long, lying on the seabed -->
    <model name="cylinder_target">
      <static>true</static>
      <pose>20 -20 0.25 1.5708 0 0.6</pose>
      <link name="link">
        <collision name="c"><geometry><cylinder><radius>0.25</radius><length>2.0</length></cylinder></geometry></collision>
        <visual name="v">
          <geometry><cylinder><radius>0.25</radius><length>2.0</length></cylinder></geometry>
          <laser_retro>0.3</laser_retro>
        </visual>
      </link>
    </model>

    <!-- Sphere, 0.8 m diameter -->
    <model name="sphere_target">
      <static>true</static>
      <pose>28 -25 0.4 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><sphere><radius>0.4</radius></sphere></geometry></collision>
        <visual name="v">
          <geometry><sphere><radius>0.4</radius></sphere></geometry>
          <laser_retro>0.3</laser_retro>
        </visual>
      </link>
    </model>

    <!-- Box, 1.0 x 0.6 x 0.5 m -->
    <model name="box_target">
      <static>true</static>
      <pose>35 -30 0.25 0 0 0.3</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>1.0 0.6 0.5</size></box></geometry></collision>
        <visual name="v">
          <geometry><box><size>1.0 0.6 0.5</size></box></geometry>
          <laser_retro>0.3</laser_retro>
        </visual>
      </link>
    </model>

    <!-- Point target for PSF measurement (Section 11) -->
    <model name="point_target">
      <static>true</static>
      <pose>40 -18 0.05 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><sphere><radius>0.05</radius></sphere></geometry></collision>
        <visual name="v">
          <geometry><sphere><radius>0.05</radius></sphere></geometry>
          <laser_retro>1.0</laser_retro>
        </visual>
      </link>
    </model>

    <!-- Rocks (half-buried spheres) -->
    <model name="rock_1">
      <static>true</static>
      <pose>24 -34 0 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><sphere><radius>0.6</radius></sphere></geometry></collision>
        <visual name="v">
          <geometry><sphere><radius>0.6</radius></sphere></geometry>
          <laser_retro>0.02</laser_retro>
        </visual>
      </link>
    </model>
    <model name="rock_2">
      <static>true</static>
      <pose>32 -14 0 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><sphere><radius>0.4</radius></sphere></geometry></collision>
        <visual name="v">
          <geometry><sphere><radius>0.4</radius></sphere></geometry>
          <laser_retro>0.02</laser_retro>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### 9.2 `config/sas_processing.yaml`

```yaml
sas_backprojection:
  ros__parameters:
    ping_topic: /sas/ping
    x_min: 15.0
    x_max: 45.0
    y_min: -40.0
    y_max: -10.0
    z_plane: 0.0
    pixel_size: 0.03
    upsample: 4
    max_squint_deg: 25.0
    use_nav_pose: false
    use_gpu: false
    dynamic_range_db: 40.0
    publish_period: 2.0
    output_path: /tmp/sas_image.npy

sas_waterfall:
  ros__parameters:
    ping_topic: /sas/ping
    z_plane: 0.0
    ground_resolution: 0.05
    max_ground_range: 60.0
    rows: 600
    tvg_exponent: 2.0
```

### 9.3 `launch/sas_sim.launch.py` (F15)

`--lockstep` makes Gazebo wait for every sensor update. The plugin synthesises each ping inside the
sensor update, so no ping is dropped even when synthesis is slower than real time. The real-time
factor drops instead. A gap in the ping sequence breaks the synthetic aperture, so lockstep is required.

```python
import os

import xacro
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def robot_nodes(context):
    xacro_file = os.path.join(
        get_package_share_directory('sas_description'), 'urdf', 'sas_auv.urdf.xacro')
    mappings = {
        key: LaunchConfiguration(key).perform(context)
        for key in ('sensor_type', 'prf', 'speed', 'duration',
                    'sway_amplitude', 'heave_amplitude', 'yaw_amplitude')
    }
    robot_description = xacro.process_file(xacro_file, mappings=mappings).toxml()
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description, 'use_sim_time': True}]),
        Node(package='gazebo_ros', executable='spawn_entity.py',
             arguments=['-entity', 'sas_auv', '-topic', 'robot_description',
                        '-x', '-10', '-y', '0', '-z', '10'],
             output='screen'),
    ]


def generate_launch_description():
    bringup = get_package_share_directory('sas_bringup')
    world = os.path.join(bringup, 'worlds', 'sas_seabed.world')
    params = os.path.join(bringup, 'config', 'sas_processing.yaml')

    args = [
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('viewer', default_value='true'),
        DeclareLaunchArgument('sensor_type', default_value='gpu_ray'),
        DeclareLaunchArgument('prf', default_value='10.0'),
        DeclareLaunchArgument('speed', default_value='1.5'),
        DeclareLaunchArgument('duration', default_value='60.0'),
        DeclareLaunchArgument('sway_amplitude', default_value='0.0'),
        DeclareLaunchArgument('heave_amplitude', default_value='0.0'),
        DeclareLaunchArgument('yaw_amplitude', default_value='0.0'),
    ]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': world,
            'gui': LaunchConfiguration('gui'),
            'extra_gazebo_args': '--lockstep',
        }.items())

    return LaunchDescription(args + [
        AppendEnvironmentVariable(
            'GAZEBO_PLUGIN_PATH', os.path.join(get_package_prefix('sas_gazebo'), 'lib')),
        gazebo,
        OpaqueFunction(function=robot_nodes),
        Node(package='sas_processing', executable='sas_backprojection',
             parameters=[params, {'use_sim_time': True}], output='screen'),
        Node(package='sas_processing', executable='sas_waterfall',
             parameters=[params, {'use_sim_time': True}], output='screen'),
        Node(package='rqt_image_view', executable='rqt_image_view',
             arguments=['/sas/image'], condition=IfCondition(LaunchConfiguration('viewer'))),
    ])
```

### 9.4 `CMakeLists.txt` and `package.xml`

```cmake
cmake_minimum_required(VERSION 3.8)
project(sas_bringup)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch worlds config DESTINATION share/${PROJECT_NAME})
ament_package()
```

```xml
<?xml version="1.0"?>
<package format="3">
  <name>sas_bringup</name>
  <version>0.1.0</version>
  <description>Launch files, worlds and parameters for the SAS simulator.</description>
  <maintainer email="bojanapu.k@mgtechsoft.com">SAS Sim</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>gazebo_ros</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>rqt_image_view</exec_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>sas_description</exec_depend>
  <exec_depend>sas_gazebo</exec_depend>
  <exec_depend>sas_processing</exec_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
```

---

## 10. Build and Run

```bash
cd ~/mahesh/sas_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# Nominal survey, ideal motion
ros2 launch sas_bringup sas_sim.launch.py

# Headless, CPU ray casting
ros2 launch sas_bringup sas_sim.launch.py gui:=false sensor_type:=ray

# Monitor
ros2 topic hz /sas/ping                       # expect PRF (10 Hz sim time)
ros2 run rqt_image_view rqt_image_view /sas/waterfall

# Save the focused image after the 60 s leg
ros2 service call /sas_backprojection/save std_srvs/srv/Trigger
```

Data flow:

```
gzserver (lockstep)
 ├─ sas_trajectory_plugin ── moves AUV
 └─ gpu_ray ─► sas_sonar_plugin ─► /sas/ping (sas_msgs/SasPing, ~410 kB/ping)
                                     ├─► sas_backprojection ─► /sas/image      (SAS)
                                     └─► sas_waterfall      ─► /sas/waterfall  (sidescan)
```

Record raw pings for offline processing:

```bash
ros2 bag record /sas/ping
```

---

## 11. Validation (F16)

### 11.1 Point-spread function

1. Set seabed `laser_retro` to `1e-6` so the seabed is almost silent.
2. Run the default leg. Save the image.
3. Measure −3 dB widths around the point target at (40, −18).

```python
import numpy as np
img = np.load('/tmp/sas_image.npy')
px = 0.03
x0, y0 = 15.0, -40.0
iy, ix = int((-18 - y0) / px), int((40 - x0) / px)
win = np.abs(img[iy - 20:iy + 20, ix - 20:ix + 20])
pk = np.unravel_index(win.argmax(), win.shape)
for axis, name in ((1, 'along-track'), (0, 'ground range')):
    cut = win[pk[0], :] if axis == 1 else win[:, pk[1]]
    cut = 20 * np.log10(cut / cut.max())
    print(name, 'width -3 dB:', (cut > -3).sum() * px, 'm')
```

Expected (use `pixel_size: 0.005` and a small grid around the target for an accurate measurement):

| Axis | Theory | Notes |
|------|--------|-------|
| Along-track | D/2 ≈ 2 cm | Wider if `max_squint_deg` is smaller than the beam |
| Slant range | c/2B = 3.75 cm | Ground range = slant / cos(grazing) |

Real-aperture waterfall along-track width at 40 m: λR/D ≈ 15 m. The SAS image must be
roughly 750× sharper along-track.

### 11.2 Motion-error tests

| Test | Launch arguments | BP parameter | Expected result |
|------|------------------|--------------|-----------------|
| Baseline | defaults | `use_nav_pose: false` | Sharp PSF |
| Sway, known | `sway_amplitude:=0.05` | `use_nav_pose: false` | Sharp (true pose compensates) |
| Sway, ignored | `sway_amplitude:=0.05` | use straight-line pose | Defocus and paired echoes |
| INS drift | defaults, nav random walk on | `use_nav_pose: true` | Defocus grows with drift > λ/8 |
| Undersampled | `speed:=4.0` | — | Plugin DPCA warning, along-track ghosts |
| Range ambiguity | `prf:=15.0` | — | Plugin warning (2·60/1500 > 1/15) |

### 11.3 Coherence check

Phase stability between pings proves the world-fixed scatterer model works:

1. Stop the vehicle: `speed:=0.0`.
2. Record 20 pings.
3. Correlate element 0 of consecutive pings. The correlation coefficient must be > 0.99
   (only receiver noise decorrelates). A low value means the voxel field is unstable;
   reduce `cell_size` or raise the ray counts.

---

## 12. Performance Tuning

Cost per ping is dominated by the number of voxels (≈ footprint area / cell_size²) multiplied by
the number of elements and 8 interpolation taps.

| Knob | Effect | Default |
|------|--------|---------|
| `cell_size` | Voxel count ∝ 1/cell². Smaller = finer speckle and slower | 0.04 m |
| `h_samples`, `v_samples` | Ray image size. Must keep quads small enough for clean shadows | 512 × 512 |
| `num_elements` | Linear cost in synthesis and backprojection | 16 |
| `max_range` | Footprint area and samples per ping | 60 m |
| `OMP_NUM_THREADS` | Threads for synthesis | all cores |
| BP `pixel_size`, grid extent | Linear in pixel count | 0.03 m, 30 × 30 m |
| BP `use_gpu` | CuPy backend, typically 20–50× faster | false |

Rough budget for defaults: ≈1 M voxels per ping, ≈130 M kernel taps, 0.2–0.5 s per ping on an
8-core CPU. With lockstep, simulation runs at about 0.2–0.5× real time. For a faster preview use
`cell_size: 0.08`, `num_elements: 8` and `pixel_size: 0.05`.

`--symlink-install` and `CMAKE_BUILD_TYPE=Release` matter: a Debug build of the plugin is about 10× slower.

---

## 13. Model Limits and Extensions

Limits of this implementation:

- Constant sound speed. No refraction, no multipath, no surface bounce.
- Geometry comes from rendered visuals (`gpu_ray`). Invisible objects produce no echo.
- Lambert scattering only. No specular highlights or target resonances.
- Beam patterns use the centre frequency only. No frequency-dependent beam width across the band.
- Backprojection uses a flat seabed at `z_plane`. Tall objects show layover.
- Voxelised surfaces: the surface is quantised to `cell_size`. This is coherent but adds a small
  geometric roughness.
- No intra-pulse Doppler. This is acceptable at AUV speeds below 3 m/s.

Extensions:

1. DPCA micronavigation node: cross-correlate overlapping phase centres between consecutive pings
   to estimate sway and yaw. Use the estimate in backprojection instead of `pose_nav`.
2. Frequency-domain image formation (ω-k / wavenumber algorithm) for faster large-area processing.
3. CUDA echo synthesis: move the scatterer-by-element loop to the GPU.
4. Ray-traced sound-speed profile (Bellhop-style eigenrays) for refraction.
5. Interferometric SAS: add a second vertically displaced array and estimate bathymetry from phase difference.
6. Heightmap seabed with sand ripples for realistic texture.
