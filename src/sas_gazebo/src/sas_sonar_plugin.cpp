// Copyright 2026 The SAS Sensor Plugin Authors
// SPDX-License-Identifier: Apache-2.0
//
// Synthetic Aperture Sonar sensor plugin for Gazebo Classic 11.
// Each sensor update is one ping: read the ray image, build the world-fixed scatterer field
// (scatter.hpp), synthesise the raw multi-element echo (synth.hpp), publish sas_msgs/SasPing.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/GpuRaySensor.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
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
    update_conn_.reset();
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

    pub_ = ros_node_->create_publisher<sas_msgs::msg::SasPing>(
      p_.topic, rclcpp::QoS(50).reliable());
    update_conn_ = sensor_->ConnectUpdated(std::bind(&SasSonarPlugin::OnScan, this));
    sensor_->SetActive(true);

    RCLCPP_INFO(
      ros_node_->get_logger(),
      "SAS sonar ready on %s: fc=%.1f kHz B=%.1f kHz Tp=%.2f ms fs=%.1f kHz N=%d d=%.3f m "
      "PRF=%.2f Hz Ns=%d Nfft=%d cell=%.3f m alpha=%.4f dB/m sensor=%s",
      p_.topic.c_str(), p_.center_frequency / 1e3, p_.bandwidth / 1e3, p_.pulse_length * 1e3,
      p_.sample_rate / 1e3, p_.num_elements, p_.element_spacing, sensor_->UpdateRate(),
      synth_->num_samples(), synth_->nfft(), sp_.cell_size, synth_->absorption_db_per_m(),
      gpu_ray_ ? "gpu_ray" : "ray");
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
    if (gpu_ray_) {
      ReadScan(gpu_ray_, scan_);
    } else {
      ReadScan(cpu_ray_, scan_);
    }
    // Array frame in world: sensor pose relative to its link, composed with the link pose.
    // Plan 006 also publishes it in SasPing (REQ-011).
    const auto array_pose = sensor_->Pose() + link_->WorldPose();
    // Receiver motion during propagation (REQ-008) uses the link velocity from plan 006.
    const Vector3d vel = Vector3d::Zero;

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

    msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(world_->SimTime());
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

    // pose and velocity are filled in plan 006 (REQ-011); left at identity / zero here.
    msg.pose.orientation.w = 1.0;

    msg.num_elements = static_cast<uint32_t>(p_.num_elements);
    msg.num_samples = static_cast<uint32_t>(synth_->num_samples());

    pub_->publish(msg);
  }

  // Debug summary: count, z range, timings, and one tracked voxel (plan 004/005 exit checks).
  void LogScatterers(double build_s, double synth_s)
  {
    double zmin = 1e9, zmax = -1e9;
    for (const auto & s : scatterers_) {
      zmin = std::min(zmin, s.pos.Z());
      zmax = std::max(zmax, s.pos.Z());
    }
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

  scatter::Params sp_;
  scatter::Scan scan_;
  scatter::Workspace ws_;
  std::vector<scatter::Scatterer> scatterers_;
  bool debug_scatterers_ = false;
  uint64_t tracked_key_ = 0;

  synth::Params yp_;
  std::unique_ptr<synth::EchoSynthesizer> synth_;
};

GZ_REGISTER_SENSOR_PLUGIN(SasSonarPlugin)

}  // namespace sas_gazebo
