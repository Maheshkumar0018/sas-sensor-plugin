// Copyright 2026 The SAS Sensor Plugin Authors
// SPDX-License-Identifier: Apache-2.0
//
// Synthetic Aperture Sonar sensor plugin for Gazebo Classic 11.
// Each sensor update is one ping: read the ray image, build the world-fixed scatterer field
// (scatter.hpp), synthesise the raw multi-element echo (synth.hpp), publish sas_msgs/SasPing.

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/GpuLaser.hh>
#include <gazebo/sensors/GpuRaySensor.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sas_msgs/msg/sas_ping.hpp>

#include "sas_gazebo/dsp.hpp"
#include "sas_gazebo/scatter.hpp"
#include "sas_gazebo/synth.hpp"

namespace sas_gazebo
{

using ignition::math::Vector3d;

template<typename SensorT>
void ReadScan(const std::shared_ptr<SensorT> & s, scatter::Scan & scan)
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
    {
      std::lock_guard<std::mutex> lk(state_mx_);
      shutting_down_ = true;
    }
    ping_cv_.notify_all();
    update_conn_.reset();
    laser_frame_conn_.reset();
    world_end_conn_.reset();
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
    ros_node_ = gazebo_ros::Node::Get(sdf);
    link_ = boost::dynamic_pointer_cast<gazebo::physics::Link>(  // physics uses boost::shared_ptr
      world_->EntityByName(sensor->ParentName()));
    if (!link_) {
      RCLCPP_ERROR(
        ros_node_->get_logger(), "Parent link '%s' not found. SAS sonar disabled.",
        sensor->ParentName().c_str());
      return;
    }

    ReadParams(sdf);
    const auto errors = dsp::Validate(p_);
    if (!errors.empty()) {
      for (const auto & e : errors) {
        RCLCPP_ERROR(ros_node_->get_logger(), "Invalid parameter: %s", e.c_str());
      }
      RCLCPP_ERROR(ros_node_->get_logger(), "SAS sonar disabled.");
      return;
    }

    // REQ-002: transmitter at array origin, receivers along array y-axis.
    rx_y_ = dsp::ReceiverPositionsY(p_.num_elements, p_.element_spacing);
    std::vector<Vector3d> rx_pos;
    for (double y : rx_y_) {
      rx_pos.emplace_back(0.0, y, 0.0);
    }
    synth_ = std::make_unique<synth::EchoSynthesizer>(yp_, Vector3d::Zero, rx_pos);

    // REQ-013: range ambiguity, checked once.
    pri_ = 1.0 / sensor_->UpdateRate();
    if (dsp::RangeAmbiguous(p_.max_range, p_.sound_speed, pri_)) {
      RCLCPP_WARN(
        ros_node_->get_logger(),
        "Range ambiguity: 2*max_range/c = %.4f s > PRI = %.4f s. Echoes from beyond %.1f m "
        "fold into the next ping. Lower update_rate or max_range.",
        2.0 * p_.max_range / p_.sound_speed, pri_, 0.5 * p_.sound_speed * pri_);
    }

    // ADR-0010: CPU `ray` sensors get no lockstep from Gazebo, so the plugin gates physics itself.
    // `gpu_ray` is a rendering sensor: `gzserver --lockstep` already holds the rate, and the gate
    // would deadlock against it, so it is never used there.
    strict_rate_ = !gpu_ray_;
    if (sdf->HasElement("strict_rate")) {strict_rate_ = sdf->Get<bool>("strict_rate");}
    if (strict_rate_ && gpu_ray_) {
      RCLCPP_WARN(
        ros_node_->get_logger(),
        "strict_rate is not supported with gpu_ray (deadlocks with --lockstep); disabled. "
        "Run gzserver --lockstep.");
      strict_rate_ = false;
    }
    {
      // Arm the gate from the start: the first ping is due at t = 0, so physics waits after its
      // first step until ping 0 is published instead of running free during ping 0.
      std::lock_guard<std::mutex> lk(state_mx_);
      last_ping_t_ = -pri_;
      have_ping_ = true;
    }

    pub_ = ros_node_->create_publisher<sas_msgs::msg::SasPing>(
      p_.topic, rclcpp::QoS(50).reliable());
    // ADR-0010: per-step state history (exact transmit-time pose) and the strict-rate gate.
    world_end_conn_ = gazebo::event::Events::ConnectWorldUpdateEnd(
      std::bind(&SasSonarPlugin::OnWorldUpdateEnd, this));
    if (gpu_ray_) {
      // Gazebo 11's GpuRaySensor::Retro() is a stub that always returns 0. The retro channel is
      // only available in the raw laser frame (range, retro, unused per ray), delivered in the
      // same order as Ranges() just before the sensor's update event (ADR-0011).
      laser_frame_conn_ = gpu_ray_->ConnectNewLaserFrame(
        std::bind(
          &SasSonarPlugin::OnLaserFrame, this, std::placeholders::_1, std::placeholders::_2,
          std::placeholders::_3, std::placeholders::_4));
    }
    update_conn_ = sensor_->ConnectUpdated(std::bind(&SasSonarPlugin::OnScan, this));
    sensor_->SetActive(true);

    RCLCPP_INFO(
      ros_node_->get_logger(),
      "SAS sonar ready on %s: fc=%.1f kHz B=%.1f kHz Tp=%.2f ms fs=%.1f kHz N=%d d=%.3f m "
      "PRF=%.2f Hz Ns=%d Nfft=%d cell=%.3f m alpha=%.4f dB/m sensor=%s strict_rate=%s",
      p_.topic.c_str(), p_.center_frequency / 1e3, p_.bandwidth / 1e3, p_.pulse_length * 1e3,
      p_.sample_rate / 1e3, p_.num_elements, p_.element_spacing, sensor_->UpdateRate(),
      synth_->num_samples(), synth_->nfft(), sp_.cell_size, synth_->absorption_db_per_m(),
      gpu_ray_ ? "gpu_ray" : "ray", strict_rate_ ? "true" : "false");
  }

private:
  void ReadParams(const sdf::ElementPtr & sdf)
  {
    auto get_d = [&](const char * key, double & v) {
        if (sdf->HasElement(key)) {v = sdf->Get<double>(key);}
      };
    auto get_i = [&](const char * key, int & v) {
        if (sdf->HasElement(key)) {v = sdf->Get<int>(key);}
      };
    auto get_s = [&](const char * key, std::string & v) {
        if (sdf->HasElement(key)) {v = sdf->Get<std::string>(key);}
      };
    get_d("sound_speed", p_.sound_speed);
    get_d("center_frequency", p_.center_frequency);
    get_d("bandwidth", p_.bandwidth);
    get_d("pulse_length", p_.pulse_length);
    get_d("sample_rate", p_.sample_rate);
    get_d("min_range", p_.min_range);
    get_d("max_range", p_.max_range);
    get_i("num_elements", p_.num_elements);
    get_d("element_spacing", p_.element_spacing);
    get_d("element_length", p_.element_length);
    get_d("tx_length", p_.tx_length);
    get_d("source_level", p_.source_level);
    get_d("noise_std", p_.noise_std);
    get_i("seed", p_.seed);
    get_s("topic", p_.topic);
    get_s("frame_id", p_.frame_id);

    // Scene (plan 004). REQ-004, REQ-005, REQ-007
    double cell = sp_.cell_size, max_inc_deg = 88.0, bs_db = -27.0;
    get_d("cell_size", cell);
    get_d("max_incidence_deg", max_inc_deg);
    get_d("default_backscatter_db", bs_db);
    sp_.cell_size = cell;
    sp_.max_incidence = max_inc_deg * M_PI / 180.0;
    sp_.mu_default = std::pow(10.0, bs_db / 10.0);
    sp_.min_range = p_.min_range;
    sp_.max_range = p_.max_range;
    sp_.seed = static_cast<uint64_t>(static_cast<uint32_t>(p_.seed));
    if (sdf->HasElement("debug_scatterers")) {
      debug_scatterers_ = sdf->Get<bool>("debug_scatterers");
    }

    // Synthesis (plan 005). REQ-003, REQ-006, REQ-008, REQ-009
    yp_.sound_speed = p_.sound_speed;
    yp_.center_frequency = p_.center_frequency;
    yp_.bandwidth = p_.bandwidth;
    yp_.pulse_length = p_.pulse_length;
    yp_.sample_rate = p_.sample_rate;
    yp_.min_range = p_.min_range;
    yp_.max_range = p_.max_range;
    yp_.element_length = p_.element_length;
    yp_.tx_length = p_.tx_length;
    get_d("element_height", yp_.element_height);
    get_d("tx_height", yp_.tx_height);
    yp_.source_level = p_.source_level;
    get_d("absorption_db_per_m", yp_.absorption_db_per_m);  // default: Thorp at fc
    yp_.noise_std = p_.noise_std;
    yp_.cell_size = sp_.cell_size;
    yp_.seed = sp_.seed;
  }

  void OnScan()
  {
    // Diagnostics only (plan 009 profile); steady_clock never reaches the output.
    const auto t_in = std::chrono::steady_clock::now();
    if (gpu_ray_) {
      ReadScan(gpu_ray_, scan_);
      ApplyGpuRetro();
    } else {
      ReadScan(cpu_ray_, scan_);
    }
    // REQ-011: ground-truth state at transmit, never perturbed. Physics keeps stepping while this
    // callback runs, so the state is looked up in the per-step history at the scan's measurement
    // time instead of being read from the link now (ADR-0010). The scan and the pose then match.
    const gazebo::common::Time t_tx = sensor_->LastMeasurementTime();
    ignition::math::Pose3d array_pose;
    Vector3d vel;
    LookupState(t_tx.Double(), array_pose, vel);
    CheckTiming(t_tx.Double());
    CheckDpca(array_pose, vel);
    if (debug_scatterers_ && gpu_ray_ && gpu_ray_->LaserCamera()) {
      // Diagnostic: where the GPU laser was rendered from vs the physics state for this ping.
      const auto cam = gpu_ray_->LaserCamera()->WorldPosition();
      const double v = vel.Length();
      const double d = (cam - array_pose.Pos()).Length();
      RCLCPP_INFO(
        ros_node_->get_logger(), "ping %lu render-vs-state offset %.6f m (%.2f ms at |v| %.2f)",
        static_cast<unsigned long>(ping_id_), d, v > 0 ? 1e3 * d / v : 0.0, v);
    }

    const auto t0 = std::chrono::steady_clock::now();  // diagnostics only, never in output
    scatter::BuildScatterers(scan_, array_pose, sp_, ws_, scatterers_);
    const auto t1 = std::chrono::steady_clock::now();

    sas_msgs::msg::SasPing msg;
    synth_->Synthesize(scatterers_, array_pose, vel, ping_id_, msg.data_real, msg.data_imag);
    const auto t2 = std::chrono::steady_clock::now();
    if (debug_scatterers_) {
      LogScatterers(
        std::chrono::duration<double>(t1 - t0).count(),
        std::chrono::duration<double>(t2 - t1).count());
    }

    msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(t_tx);
    msg.header.frame_id = p_.frame_id;
    msg.ping_id = ping_id_++;

    msg.sound_speed = p_.sound_speed;
    msg.center_frequency = p_.center_frequency;
    msg.bandwidth = p_.bandwidth;
    msg.pulse_length = p_.pulse_length;
    msg.sample_rate = p_.sample_rate;
    msg.t0 = 0.0;
    msg.source_level = p_.source_level;
    msg.noise_std = p_.noise_std;

    msg.rx_positions.resize(rx_y_.size());
    for (size_t k = 0; k < rx_y_.size(); ++k) {
      msg.rx_positions[k].y = rx_y_[k];
    }
    msg.element_length = p_.element_length;
    msg.tx_length = p_.tx_length;

    msg.pose = gazebo_ros::Convert<geometry_msgs::msg::Pose>(array_pose);
    msg.velocity = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(vel);

    msg.num_elements = static_cast<uint32_t>(p_.num_elements);
    msg.num_samples = static_cast<uint32_t>(synth_->num_samples());

    const auto t3 = std::chrono::steady_clock::now();
    pub_->publish(msg);
    const auto t4 = std::chrono::steady_clock::now();
    if (debug_scatterers_) {
      auto ms = [](auto a, auto b) {return std::chrono::duration<double, std::milli>(b - a).count();};
      const double period = last_in_valid_ ? ms(last_in_, t_in) : 0.0;
      RCLCPP_INFO(
        ros_node_->get_logger(),
        "ping %lu timing [ms]: scan+state %.1f build %.1f synth %.1f msg %.1f publish %.1f "
        "callback %.1f | wall period %.1f",
        static_cast<unsigned long>(msg.ping_id), ms(t_in, t0), ms(t0, t1), ms(t1, t2), ms(t2, t3),
        ms(t3, t4), ms(t_in, t4), period);
    }
    last_in_ = t_in;
    last_in_valid_ = true;

    // Release the strict-rate gate for this ping.
    {
      std::lock_guard<std::mutex> lk(state_mx_);
      last_ping_t_ = t_tx.Double();
    }
    ping_cv_.notify_all();
  }

  // Rendering thread: copy range and retro channels of the newest GPU laser frame.
  void OnLaserFrame(const float * data, unsigned int w, unsigned int h, unsigned int depth)
  {
    std::lock_guard<std::mutex> lk(frame_mx_);
    const size_t n = static_cast<size_t>(w) * h;
    frame_range_.resize(n);
    frame_retro_.resize(n);
    for (size_t i = 0; i < n; ++i) {
      frame_range_[i] = data[i * depth];
      frame_retro_[i] = depth > 1 ? data[i * depth + 1] : 0.0f;
    }
  }

  // Replace the stub retro values with the frame's retro channel. The frame must be the one the
  // ranges came from: same size, and every finite in-range range must match.
  void ApplyGpuRetro()
  {
    std::lock_guard<std::mutex> lk(frame_mx_);
    bool aligned = frame_retro_.size() == scan_.ranges.size();
    size_t checked = 0;
    for (size_t i = 0; aligned && i < scan_.ranges.size(); ++i) {
      const double r = scan_.ranges[i];
      if (std::isfinite(r) && r < scan_.rmax) {
        aligned = std::abs(r - frame_range_[i]) <= 1e-3 * std::max(1.0, r);
        ++checked;
      }
    }
    if (!aligned || checked == 0) {
      RCLCPP_WARN_THROTTLE(
        ros_node_->get_logger(), steady_clock_, 5000,
        "GPU laser frame does not match the scan (%zu vs %zu rays, %zu checked); "
        "laser_retro ignored for this ping, default_backscatter_db used.",
        frame_retro_.size(), scan_.ranges.size(), checked);
      return;
    }
    for (size_t i = 0; i < scan_.retro.size(); ++i) {
      scan_.retro[i] = frame_retro_[i];
    }
  }

  // Physics thread, after every step: record the array state, then (strict_rate) hold physics at
  // the step that reaches the next ping time until that ping is published (ADR-0010).
  void OnWorldUpdateEnd()
  {
    const double t = world_->SimTime().Double();
    StateSample s{t, sensor_->Pose() + link_->WorldPose(),
      link_->WorldLinearVel(sensor_->Pose().Pos())};
    std::unique_lock<std::mutex> lk(state_mx_);
    history_.push_back(s);
    while (history_.size() > kHistorySize) {
      history_.pop_front();
    }
    if (!strict_rate_ || !have_ping_) {
      return;
    }
    // Physics may run only up to one PRI past the last published ping. While a ping is being
    // computed, the next one is not yet published, so physics waits at the next ping time.
    // Do not arm this at ping start: the sensor thread is then still busy when the next ping
    // time is reached, Gazebo's sensor wake-up event (fired at world update begin) is missed,
    // and the gate stalls until its timeout.
    const double due = last_ping_t_ + pri_;
    if (t + 1e-9 < due) {
      return;
    }
    const bool released = ping_cv_.wait_for(lk, std::chrono::seconds(10), [&] {
          return shutting_down_ || last_ping_t_ + 1e-9 >= due;
        });
    if (!released) {
      RCLCPP_WARN_THROTTLE(
        ros_node_->get_logger(), steady_clock_, 5000,
        "strict_rate: no ping for t = %.3f s after 10 s wall time; letting physics continue.",
        due);
      last_ping_t_ = due;  // skip this ping rather than block forever
    }
  }

  // Exact state at measurement time t; nearest earlier sample (with a warning) if t is missing.
  void LookupState(double t, ignition::math::Pose3d & pose, Vector3d & vel)
  {
    std::lock_guard<std::mutex> lk(state_mx_);
    const StateSample * best = nullptr;
    for (auto it = history_.rbegin(); it != history_.rend(); ++it) {
      if (it->t <= t + 1e-9) {best = &*it; break;}
    }
    if (!best) {
      // Before the first physics step: read the link directly.
      pose = sensor_->Pose() + link_->WorldPose();
      vel = link_->WorldLinearVel(sensor_->Pose().Pos());
      return;
    }
    if (std::abs(best->t - t) > 1e-9) {
      RCLCPP_WARN_THROTTLE(
        ros_node_->get_logger(), steady_clock_, 5000,
        "No state sample at t = %.6f s; using t = %.6f s.", t, best->t);
    }
    pose = best->pose;
    vel = best->vel;
  }

  // Lockstep check: every sensor update must be one PRI after the previous one (REQ-012).
  void CheckTiming(double t)
  {
    if (last_t_ >= 0.0 && dsp::PingIntervalOff(t - last_t_, pri_)) {
      RCLCPP_WARN_THROTTLE(
        ros_node_->get_logger(), steady_clock_, 5000,
        "Ping interval %.4f s differs from PRI %.4f s. Enable strict_rate or run gzserver --lockstep.",
        t - last_t_, pri_);
    }
    last_t_ = t;
  }

  // DPCA along-track sampling check (REQ-013). Warning only.
  void CheckDpca(const ignition::math::Pose3d & array_pose, const Vector3d & vel)
  {
    const double v_along = vel.Dot(array_pose.Rot().RotateVector(Vector3d::UnitY));
    if (dsp::DpcaViolated(v_along, pri_, p_.num_elements, p_.element_spacing)) {
      RCLCPP_WARN_THROTTLE(
        ros_node_->get_logger(), steady_clock_, 5000,
        "DPCA criterion violated: v_along*PRI = %.3f m > N*d/2 = %.3f m (v_along = %.2f m/s, "
        "limit %.2f m/s). Expect along-track aliasing.",
        std::abs(v_along) * pri_, 0.5 * p_.num_elements * p_.element_spacing, v_along,
        0.5 * p_.num_elements * p_.element_spacing / pri_);
    }
  }

  // Debug summary: count, z range, timings, and one tracked voxel (plan 004/005 exit checks).
  void LogScatterers(double build_s, double synth_s)
  {
    double zmin = 1e9, zmax = -1e9, mu_min = 1e9, mu_max = -1e9, retro_max = 0.0;
    size_t bright = 0;
    for (const auto & s : scatterers_) {
      zmin = std::min(zmin, s.pos.Z());
      zmax = std::max(zmax, s.pos.Z());
      mu_min = std::min(mu_min, s.mu);
      mu_max = std::max(mu_max, s.mu);
      bright += s.mu > 0.1 ? 1 : 0;
    }
    for (double r : scan_.retro) {retro_max = std::max(retro_max, r);}
    RCLCPP_INFO(
      ros_node_->get_logger(), "ping %lu: raw retro max=%.4g | mu=[%.4g, %.4g] voxels mu>0.1: %zu",
      static_cast<unsigned long>(ping_id_), retro_max, mu_min, mu_max, bright);
    if (!scatterers_.empty() && tracked_key_ == 0) {
      tracked_key_ = scatterers_[scatterers_.size() / 2].key;
    }
    const scatter::Scatterer * tracked = nullptr;
    for (const auto & s : scatterers_) {
      if (s.key == tracked_key_) {tracked = &s; break;}
    }
    if (tracked) {
      RCLCPP_INFO(
        ros_node_->get_logger(),
        "ping %lu: scatterers=%zu z=[%.3f, %.3f] build=%.1f ms synth=%.1f ms | voxel %lx "
        "pos=(%.9f, %.9f, %.9f) speckle=(%.9f, %.9f) mu=%.4g",
        static_cast<unsigned long>(ping_id_), scatterers_.size(), zmin, zmax, build_s * 1e3,
        synth_s * 1e3, static_cast<unsigned long>(tracked_key_), tracked->pos.X(),
        tracked->pos.Y(), tracked->pos.Z(), tracked->speckle.real(), tracked->speckle.imag(),
        tracked->mu);
    } else {
      RCLCPP_INFO(
        ros_node_->get_logger(),
        "ping %lu: scatterers=%zu z=[%.3f, %.3f] build=%.1f ms synth=%.1f ms | voxel %lx not seen",
        static_cast<unsigned long>(ping_id_), scatterers_.size(), zmin, zmax, build_s * 1e3,
        synth_s * 1e3, static_cast<unsigned long>(tracked_key_));
    }
  }

  gazebo::sensors::SensorPtr sensor_;
  gazebo::sensors::GpuRaySensorPtr gpu_ray_;
  gazebo::sensors::RaySensorPtr cpu_ray_;
  gazebo::physics::WorldPtr world_;
  gazebo::physics::LinkPtr link_;
  gazebo::event::ConnectionPtr update_conn_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Publisher<sas_msgs::msg::SasPing>::SharedPtr pub_;

  dsp::SonarParams p_;
  std::vector<double> rx_y_;
  uint64_t ping_id_ = 0;
  double pri_ = 0.1;
  double last_t_ = -1.0;
  // Throttled warnings use wall time: the node clock is sim time and stays at 0 without /clock.
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  // ADR-0010: per-step state history and strict-rate gate. Guarded by state_mx_.
  struct StateSample
  {
    double t;
    ignition::math::Pose3d pose;
    Vector3d vel;
  };
  static constexpr size_t kHistorySize = 8192;  // ~8 s at a 1 ms physics step
  gazebo::event::ConnectionPtr world_end_conn_;
  // ADR-0011: GPU retro channel from the raw laser frame. Guarded by frame_mx_.
  gazebo::event::ConnectionPtr laser_frame_conn_;
  std::mutex frame_mx_;
  std::vector<float> frame_range_;
  std::vector<float> frame_retro_;
  std::mutex state_mx_;
  std::condition_variable ping_cv_;
  std::deque<StateSample> history_;
  bool strict_rate_ = true;
  bool have_ping_ = false;  // gate armed
  bool shutting_down_ = false;
  double last_ping_t_ = 0.0;  // measurement time of the last published ping

  scatter::Params sp_;
  scatter::Scan scan_;
  scatter::Workspace ws_;
  std::vector<scatter::Scatterer> scatterers_;
  bool debug_scatterers_ = false;
  std::chrono::steady_clock::time_point last_in_;
  bool last_in_valid_ = false;
  uint64_t tracked_key_ = 0;

  synth::Params yp_;
  std::unique_ptr<synth::EchoSynthesizer> synth_;
};

GZ_REGISTER_SENSOR_PLUGIN(SasSonarPlugin)

}  // namespace sas_gazebo
