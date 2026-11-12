#include "point_lio_ffi.h"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <limits>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <stdexcept>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <thread>
#include <vector>

static_assert(sizeof(PlioConfig) == 96 && sizeof(PlioPoint) == 40 &&
                  sizeof(PlioImu) == 56 && sizeof(PlioOutput) == 712,
              "C ABI layout mismatch");
using Clock = std::chrono::steady_clock;
using CustomMsg = livox_ros_driver2::msg::CustomMsg;
struct QueuedImu {
  int64_t stamp;
  PlioImu data;
};
struct QueuedScan {
  int64_t start, end;
  std::vector<PlioPoint> points;
  Clock::time_point received;
};

class PointLioNode final : public rclcpp::Node {
public:
  PointLioNode() : Node("point_lio") {
    odom_frame_ = declare_parameter("odom_frame", std::string("odom"));
    imu_frame_ = declare_parameter("imu_frame", std::string("lio_imu"));
    lidar_frame_ = declare_parameter("lidar_frame", std::string("livox_frame"));
    if (odom_frame_ == imu_frame_ || imu_frame_ == lidar_frame_ ||
        odom_frame_ == lidar_frame_ || odom_frame_.empty() ||
        imu_frame_.empty() || lidar_frame_.empty())
      throw std::runtime_error("frame names must be distinct and nonempty");
    path_capacity_ = declare_parameter("path_capacity", 3000);
    if (path_capacity_ < 1 || path_capacity_ > 100000)
      throw std::runtime_error("invalid path_capacity");
    stride_ = declare_parameter("point_stride", 2);
    scan_capacity_ = declare_parameter("scan_queue_capacity", 2);
    imu_capacity_ = declare_parameter("imu_queue_capacity", 4000);
    min_matches_ = declare_parameter("min_matches", 20);
    acceleration_scale_ = declare_parameter("acceleration_scale", 9.81);
    const auto t =
        declare_parameter("lidar_to_imu_translation",
                          std::vector<double>{-0.011, -0.02329, 0.04412});
    const auto q = declare_parameter("lidar_to_imu_quaternion",
                                     std::vector<double>{0., 0., 0., 1.});
    auto map_capacity = declare_parameter("map_capacity", 100000);
    if (stride_ < 1 || scan_capacity_ < 1 || scan_capacity_ > 100 ||
        imu_capacity_ < 100 || imu_capacity_ > 20000 || min_matches_ < 1 ||
        t.size() != 3 || q.size() != 4 || !std::isfinite(acceleration_scale_) ||
        acceleration_scale_ <= 0 || map_capacity < 5)
      throw std::runtime_error("invalid bridge parameters");
    std::copy(t.begin(), t.end(), config_.translation);
    std::copy(q.begin(), q.end(), config_.quaternion);
    config_.min_range = declare_parameter("min_range", 0.5);
    config_.max_range = declare_parameter("max_range", 30.);
    config_.voxel_size = declare_parameter("voxel_size", 0.3);
    config_.filter_size_map = declare_parameter("filter_size_map", 0.3);
    config_.map_capacity = static_cast<uint64_t>(map_capacity);
    if (plio_abi_version() != 1 || !(core_ = plio_create(&config_)))
      throw std::runtime_error("Rust Point-LIO configuration or ABI invalid");
    try {
      odometry_ =
          create_publisher<nav_msgs::msg::Odometry>("point_lio/odometry", 10);
      path_ = create_publisher<nav_msgs::msg::Path>(
          "point_lio/path", rclcpp::QoS(1).transient_local());
      path_timer_ = create_wall_timer(std::chrono::milliseconds(500),
                                      [this] { publish_path(); });
      diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
          "point_lio/diagnostics", 10);
      tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      static_tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
      geometry_msgs::msg::TransformStamped extrinsic;
      extrinsic.header.stamp = now();
      extrinsic.header.frame_id = imu_frame_;
      extrinsic.child_frame_id = lidar_frame_;
      extrinsic.transform.translation.x = t[0];
      extrinsic.transform.translation.y = t[1];
      extrinsic.transform.translation.z = t[2];
      extrinsic.transform.rotation.x = q[0];
      extrinsic.transform.rotation.y = q[1];
      extrinsic.transform.rotation.z = q[2];
      extrinsic.transform.rotation.w = q[3];
      static_tf_->sendTransform(extrinsic);
      imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
          "livox/imu", rclcpp::SensorDataQoS().keep_last(1000),
          [this](sensor_msgs::msg::Imu::ConstSharedPtr m) { receive_imu(*m); });
      lidar_sub_ = create_subscription<CustomMsg>(
          "livox/lidar", rclcpp::SensorDataQoS().keep_last(5),
          [this](CustomMsg::ConstSharedPtr m) { receive_scan(*m); });
      timer_ =
          create_wall_timer(std::chrono::seconds(1), [this] { diagnostic(); });
      worker_ = std::thread([this] { work(); });
    } catch (...) {
      plio_destroy(core_);
      core_ = nullptr;
      throw;
    }
    RCLCPP_INFO(get_logger(),
                "Rust Point-LIO ready; hold MID-360 stationary for IMU "
                "initialization; stride=%d",
                stride_);
  }
  ~PointLioNode() override {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_ = true;
    }
    wake_.notify_one();
    if (worker_.joinable())
      worker_.join();
    plio_destroy(core_);
  }

private:
  // Caller holds mutex. Generation prevents a computation from publishing
  // across reset.
  void reset_locked(const std::string &reason) {
    RCLCPP_WARN(get_logger(), "Reset Point-LIO: %s", reason.c_str());
    path_history_.clear();
    nav_msgs::msg::Path empty;
    empty.header.frame_id = odom_frame_;
    empty.header.stamp = now();
    path_->publish(empty);
    ++generation_;
    reset_pending_ = true;
    imus_.clear();
    scans_.clear();
    last_imu_ = -1;
    last_scan_ = -1;
    state_ = "reset: " + reason;
    ++resets_;
    tracking_ = false;
  }
  void receive_imu(const sensor_msgs::msg::Imu &m) {
    const int64_t stamp = rclcpp::Time(m.header.stamp).nanoseconds();
    if (stamp < 0)
      return;
    QueuedImu s{};
    s.stamp = stamp;
    s.data.gyro[0] = m.angular_velocity.x;
    s.data.gyro[1] = m.angular_velocity.y;
    s.data.gyro[2] = m.angular_velocity.z;
    s.data.acceleration[0] = m.linear_acceleration.x * acceleration_scale_;
    s.data.acceleration[1] = m.linear_acceleration.y * acceleration_scale_;
    s.data.acceleration[2] = m.linear_acceleration.z * acceleration_scale_;
    for (int i = 0; i < 3; ++i)
      if (!std::isfinite(s.data.gyro[i]) ||
          !std::isfinite(s.data.acceleration[i]))
        return;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (last_imu_ >= 0 && stamp <= last_imu_) {
        if (last_imu_ - stamp > 500000000)
          reset_locked("IMU clock rollback");
        else {
          ++reordered_imu_;
          return;
        }
      }
      if (imus_.size() >= static_cast<size_t>(imu_capacity_))
        reset_locked("IMU queue overflow");
      last_imu_ = stamp;
      imus_.push_back(s);
      last_input_ = Clock::now();
    }
    wake_.notify_one();
  }
  void receive_scan(const CustomMsg &m) {
    if (m.timebase >
            static_cast<uint64_t>(std::numeric_limits<int64_t>::max()) ||
        m.point_num != m.points.size() || m.points.size() > 100000 ||
        m.points.empty()) {
      ++invalid_scans_;
      return;
    }
    const auto start = static_cast<int64_t>(m.timebase);
    uint32_t max_offset = 0;
    QueuedScan s{};
    s.start = start;
    s.received = Clock::now();
    s.points.reserve(m.points.size() / stride_ + 1);
    for (size_t i = 0; i < m.points.size(); ++i) {
      const auto &p = m.points[i];
      max_offset = std::max(max_offset, p.offset_time);
      if (i % stride_ != 0 || !std::isfinite(p.x) || !std::isfinite(p.y) ||
          !std::isfinite(p.z))
        continue;
      if ((p.tag & 0x30) != 0 && (p.tag & 0x30) != 0x10)
        continue;
      s.points.push_back(PlioPoint{{p.x, p.y, p.z},
                                   static_cast<double>(p.offset_time) * 1e-9,
                                   static_cast<float>(p.reflectivity)});
    }
    if (max_offset > 200000000 ||
        start > std::numeric_limits<int64_t>::max() - max_offset) {
      ++invalid_scans_;
      return;
    }
    s.end = start + max_offset;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (last_scan_ >= 0 && start <= last_scan_) {
        if (last_scan_ - start > 500000000)
          reset_locked("LiDAR clock rollback");
        else {
          ++invalid_scans_;
          return;
        }
      }
      last_scan_ = start;
      if (scans_.size() >= static_cast<size_t>(scan_capacity_)) {
        scans_.pop_front();
        ++dropped_scans_;
      }
      scans_.push_back(std::move(s));
    }
    wake_.notify_one();
  }
  void work() noexcept {
    try {
      work_loop();
    } catch (const std::exception &e) {
      std::lock_guard<std::mutex> lock(mutex_);
      state_ = std::string("worker failed: ") + e.what();
      failed_ = true;
      tracking_ = false;
      RCLCPP_ERROR(get_logger(), "%s", state_.c_str());
    } catch (...) {
      std::lock_guard<std::mutex> lock(mutex_);
      state_ = "worker failed";
      failed_ = true;
      tracking_ = false;
    }
  }
  void work_loop() {
    int64_t origin = -1, last_fed = -1;
    uint64_t generation = 0;
    while (true) {
      std::deque<QueuedImu> incoming;
      QueuedScan scan{};
      bool have_scan = false, reset = false;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        wake_.wait(lock, [&] {
          return stop_ || reset_pending_ || !imus_.empty() ||
                 (!scans_.empty() && last_fed >= scans_.front().end);
        });
        if (stop_)
          return;
        if (reset_pending_) {
          reset = true;
          reset_pending_ = false;
          generation = generation_;
        }
        incoming.swap(imus_);
        if (!scans_.empty() &&
            std::max(reset ? int64_t(-1) : last_fed,
                     incoming.empty() ? int64_t(-1) : incoming.back().stamp) >=
                scans_.front().end) {
          scan = std::move(scans_.front());
          scans_.pop_front();
          have_scan = true;
        }
      }
      if (reset) {
        plio_reset(core_);
        origin = -1;
        last_fed = -1;
      }
      bool bad = false;
      for (auto &s : incoming) {
        if (origin < 0)
          origin = s.stamp;
        if (last_fed >= 0 && s.stamp - last_fed > 50000000) {
          std::lock_guard<std::mutex> lock(mutex_);
          reset_locked("IMU gap > 50ms");
          bad = true;
          break;
        }
        s.data.time = static_cast<double>(s.stamp - origin) * 1e-9;
        const int result = plio_push_imu(core_, &s.data);
        last_fed = s.stamp;
        if (result != 0 && result != 2) {
          std::lock_guard<std::mutex> lock(mutex_);
          reset_locked("IMU status " + std::to_string(result));
          bad = true;
          break;
        }
        if (result == 2) {
          std::lock_guard<std::mutex> lock(mutex_);
          state_ = "waiting for stationary IMU";
        }
      }
      if (bad || !have_scan || origin < 0)
        continue;
      double state_time = 0.;
      if (plio_state_time(core_, &state_time) != 0) {
        std::lock_guard<std::mutex> lock(mutex_);
        state_ = "initializing stationary IMU";
        continue;
      }
      const double end = static_cast<double>(scan.end - origin) * 1e-9;
      if (end <= state_time)
        continue;
      const double original_start =
          static_cast<double>(scan.start - origin) * 1e-9;
      const double start = std::max(original_start, state_time);
      // Crop only already-consumed acquisition times; never rewrite point
      // order/time.
      auto &points = scan.points;
      points.erase(std::remove_if(points.begin(), points.end(),
                                  [&](PlioPoint &p) {
                                    const double time =
                                        original_start + p.offset;
                                    if (time < start)
                                      return true;
                                    p.offset = time - start;
                                    return false;
                                  }),
                   points.end());
      PlioOutput output{};
      const auto begin = Clock::now();
      const int result =
          plio_scan(core_, start, end, points.data(), points.size(), &output);
      const double compute_ms =
          std::chrono::duration<double, std::milli>(Clock::now() - begin)
              .count();
      const double latency_ms = std::chrono::duration<double, std::milli>(
                                    Clock::now() - scan.received)
                                    .count();
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (generation != generation_)
          continue;
        compute_ms_ = compute_ms;
        latency_ms_ = latency_ms;
        if (result != 0) {
          reset_locked("scan status " + std::to_string(result));
          continue;
        }
        matches_ = output.matches;
        map_points_ = output.map_points;
        processed_++;
        tracking_ = output.matches >= static_cast<uint64_t>(min_matches_);
        state_ = tracking_ ? "tracking" : "insufficient plane matches";
        if (!tracking_)
          continue;
        // Publication under lock prevents reset generation changes between
        // check and send.
        publish(output, scan.end);
        last_output_ = Clock::now();
      }
    }
  }
  void publish(const PlioOutput &s, int64_t stamp) {
    nav_msgs::msg::Odometry m;
    m.header.stamp = rclcpp::Time(stamp);
    m.header.frame_id = odom_frame_;
    m.child_frame_id = imu_frame_;
    m.pose.pose.position.x = s.position[0];
    m.pose.pose.position.y = s.position[1];
    m.pose.pose.position.z = s.position[2];
    auto &q = m.pose.pose.orientation;
    q.x = s.quaternion[0];
    q.y = s.quaternion[1];
    q.z = s.quaternion[2];
    q.w = s.quaternion[3];
    m.twist.twist.linear.x = s.velocity_body[0];
    m.twist.twist.linear.y = s.velocity_body[1];
    m.twist.twist.linear.z = s.velocity_body[2];
    m.twist.twist.angular.x = s.angular_velocity[0];
    m.twist.twist.angular.y = s.angular_velocity[1];
    m.twist.twist.angular.z = s.angular_velocity[2];
    std::copy(std::begin(s.pose_covariance), std::end(s.pose_covariance),
              m.pose.covariance.begin());
    std::copy(std::begin(s.twist_covariance), std::end(s.twist_covariance),
              m.twist.covariance.begin());
    odometry_->publish(m);
    geometry_msgs::msg::PoseStamped pose;
    pose.header = m.header;
    pose.pose = m.pose.pose;
    path_history_.push_back(pose);
    if (path_history_.size() > static_cast<size_t>(path_capacity_))
      path_history_.pop_front();
    geometry_msgs::msg::TransformStamped t;
    t.header = m.header;
    t.child_frame_id = imu_frame_;
    t.transform.rotation = q;
    t.transform.translation.x = s.position[0];
    t.transform.translation.y = s.position[1];
    t.transform.translation.z = s.position[2];
    tf_->sendTransform(t);
  }
  void publish_path() {
    std::lock_guard<std::mutex> lock(mutex_);
    nav_msgs::msg::Path path;
    path.header.frame_id = odom_frame_;
    path.header.stamp = path_history_.empty()
                            ? now()
                            : rclcpp::Time(path_history_.back().header.stamp);
    path.poses.assign(path_history_.begin(), path_history_.end());
    path_->publish(path);
  }
  void diagnostic() {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus s;
    s.name = "point_lio/estimator";
    s.hardware_id = "MID-360";
    {
      std::lock_guard<std::mutex> lock(mutex_);
      s.message = state_;
      const double age =
          std::chrono::duration<double>(Clock::now() - last_output_).count();
      s.level = failed_ ? 2 : ((tracking_ && age < 1.) ? 0 : 1);
      if (age >= 1. && tracking_)
        s.message = "odometry stale";
      auto value = [&](const std::string &name, const auto &v) {
        diagnostic_msgs::msg::KeyValue k;
        k.key = name;
        k.value = std::to_string(v);
        s.values.push_back(k);
      };
      value("processed_scans", processed_);
      value("matches", matches_);
      value("map_points", map_points_);
      value("compute_ms", compute_ms_);
      value("receive_to_output_ms", latency_ms_);
      value("pending_scans", scans_.size());
      value("pending_imu", imus_.size());
      value("dropped_scans", dropped_scans_);
      value("invalid_scans", invalid_scans_.load());
      value("reordered_imu", reordered_imu_);
      value("resets", resets_);
      value("output_age_s", age);
    }
    array.status.push_back(s);
    diagnostics_->publish(array);
  }
  PlioConfig config_{};
  void *core_ = nullptr;
  int stride_, scan_capacity_, imu_capacity_, min_matches_, path_capacity_;
  std::deque<geometry_msgs::msg::PoseStamped> path_history_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_;
  rclcpp::TimerBase::SharedPtr path_timer_;
  double acceleration_scale_;
  std::string odom_frame_, imu_frame_, lidar_frame_;
  std::mutex mutex_;
  std::condition_variable wake_;
  std::thread worker_;
  std::deque<QueuedImu> imus_;
  std::deque<QueuedScan> scans_;
  bool stop_ = false, reset_pending_ = false, tracking_ = false,
       failed_ = false;
  int64_t last_imu_ = -1, last_scan_ = -1;
  uint64_t generation_ = 0, resets_ = 0, dropped_scans_ = 0, reordered_imu_ = 0,
           processed_ = 0, matches_ = 0, map_points_ = 0;
  std::atomic<uint64_t> invalid_scans_{0};
  double compute_ms_ = 0., latency_ms_ = 0.;
  std::string state_ = "initializing stationary IMU";
  Clock::time_point last_input_ = Clock::now(), last_output_ = Clock::now();
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<CustomMsg>::SharedPtr lidar_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostics_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<PointLioNode>());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("point_lio"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
