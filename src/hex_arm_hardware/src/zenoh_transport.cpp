#include "hex_arm_hardware/zenoh_transport.hpp"
#include "hex_arm_hardware/instance_config.hpp"
#include "hex_arm_hardware/disable_receipt.hpp"

#include <algorithm>
#include <cmath>
#include <future>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include "zenoh.hxx"
#include "robot_api.pb.h"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "hex_arm_msgs/msg/driver_state.hpp"
#include "hex_arm_msgs/msg/state_timing.hpp"
#include "hex_arm_msgs/msg/interpolation_state.hpp"
#include "hex_arm_msgs/srv/discover_motors.hpp"
#include "hex_arm_msgs/srv/set_gravity.hpp"
#include "hex_arm_msgs/srv/set_operating_mode.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace hex_arm_hardware
{
namespace pb = robot_api;
namespace
{
hex_arm_msgs::msg::MotorIdentity motor_message(const pb::MotorIdentity & value)
{
  hex_arm_msgs::msg::MotorIdentity result;
  result.node_id = static_cast<std::uint8_t>(value.node_id());
  result.vendor_id = value.vendor_id(); result.product_code = value.product_code();
  result.revision = value.revision(); result.serial_number = value.serial_number();
  result.model = value.model(); result.identity_verified = value.identity_verified();
  return result;
}
bool finite_six(const google::protobuf::RepeatedField<float> & values)
{
  return values.size() == 6 && std::all_of(values.begin(), values.end(),
    [](float value) {return std::isfinite(value);});
}
void require(bool ok, const std::string & reason)
{
  if (!ok) {throw std::runtime_error(reason);}
}
}  // namespace

struct ZenohTransport::Impl
{
  ZenohOptions options;
  rclcpp::Node::SharedPtr node;
  SnapshotMailbox<FeedbackSnapshot> & feedback;
  SnapshotMailbox<CommandSnapshot> & commands;
  TransportTrace & trace;
  std::mutex management, data;
  FeedbackSnapshot latest;
  pb::DriverState driver;
  pb::RobotStatus status;
  std::int64_t driver_ns{}, driver_ros_ns{}, status_ns{}, activated_ns{};
  FeedbackPublication publication;
  std::array<std::size_t, 6> wire_to_ros{};
  std::uint32_t session_id{};
  std::uint64_t generation{};
  std::atomic_bool permitted{false}, stopping{false};
  std::string gate_reason;
  bool remote_restarted{false};
  bool acquisition_uncertain{false};
  bool ownership_confirmed{false};
  CommandFreshness freshness;
  DisableReceipt disable_receipt;
  std::optional<zenoh::Session> session;
  std::vector<zenoh::Subscriber<void>> subscribers;
  std::thread stream, diagnostic_worker;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub;
  rclcpp::Publisher<hex_arm_msgs::msg::DriverState>::SharedPtr driver_pub;
  rclcpp::Publisher<hex_arm_msgs::msg::StateTiming>::SharedPtr timing_pub;
  rclcpp::Publisher<hex_arm_msgs::msg::InterpolationState>::SharedPtr interpolation_pub;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_pub;
  pb::InterpolationState interpolation;
  CommandSnapshot last_sent_command;
  std::uint64_t published_command_sequence{}, published_interpolation_stamp{};
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics;
  std::vector<rclcpp::ServiceBase::SharedPtr> services;

  Impl(const ZenohOptions & settings, const rclcpp::Node::SharedPtr & io,
    SnapshotMailbox<FeedbackSnapshot> & states, SnapshotMailbox<CommandSnapshot> & targets,
    TransportTrace & tracing)
  : options(settings), node(io), feedback(states), commands(targets), trace(tracing) {}

  ~Impl()
  {
    permitted.store(false);
    stopping.store(true);
    if (stream.joinable()) {stream.join();}
    if (diagnostic_worker.joinable()) {diagnostic_worker.join();}
    // Services/executor are quiesced by HexArmSystem before destruction.
    services.clear();
    (void)release();
    subscribers.clear();
    session.reset();
  }

  template<class Response>
  Response query(const std::string & suffix, const std::string & payload, double timeout)
  {
    auto promise = std::make_shared<std::promise<std::string>>();
    auto done = std::make_shared<std::atomic_bool>(false);
    auto future = promise->get_future();
    auto query_options = zenoh::Session::GetOptions::create_default();
    query_options.payload = zenoh::Bytes(payload);
    query_options.timeout_ms = static_cast<std::uint64_t>(timeout * 1000);
    query_options.congestion_control = Z_CONGESTION_CONTROL_DROP;
    session->get(zenoh::KeyExpr(options.prefix + "/" + suffix), "",
      [promise, done](zenoh::Reply & reply) {
        if (reply.is_ok() && !done->exchange(true)) {
          promise->set_value(reply.get_ok().get_payload().as_string());
        }
      }, [promise, done]() {
        if (!done->exchange(true)) {promise->set_exception(
            std::make_exception_ptr(std::runtime_error("query returned no reply")));}
      }, std::move(query_options));
    require(future.wait_for(std::chrono::duration<double>(timeout + 0.1)) ==
      std::future_status::ready, suffix + " timed out");
    Response result;
    require(result.ParseFromString(future.get()), suffix + " returned invalid protobuf");
    return result;
  }

  template<class Request>
  void checked(const std::string & suffix, const Request & request, double timeout = 0)
  {
    const auto result = query<pb::GenericResponse>(suffix, request.SerializeAsString(),
      timeout > 0 ? timeout : options.service_timeout);
    require(result.ok(), suffix + ": " + result.error());
  }

  // Caller holds data. Freshness of all three independent publications and
  // the exact session holder are required; DriverState's boolean is not identity.
  std::string readiness(bool active, std::uint32_t holder = 0)
  {
    const auto now = monotonic_ns();
    const auto stale = [&](std::int64_t received) {
        return received <= 0 || now < received ||
               static_cast<double>(now - received) * 1e-9 > options.state_timeout;
      };
    if (stale(latest.received_ns) || stale(driver_ns) || stale(status_ns)) {
      return "joint/driver/session feedback stale";
    }
    if (!driver.profile_valid() || !driver.calibrated() || !driver.all_motors_online() ||
      !driver.feedback_fresh() || driver.fault_latched())
    {return "driver readiness/fault gate: " + driver.fault_reason();}
    if (active && (!driver.session_owned() || driver.mode() != pb::OPERATING_MODE_ACTIVE ||
      status.session_holder() != holder)) {return "ACTIVE session ownership lost";}
    return {};
  }

  void gate_locked()
  {
    if (!permitted.load()) {return;}
    const auto error = readiness(true, session_id);
    if (!error.empty()) {
      gate_reason = error;
      permitted.store(false);
      trace.emit("cxx_gate_closed", 0, generation);
    }
  }

  void init()
  {
    auto config = zenoh::Config::create_default();
    // Reject quote/control injection in this single endpoint JSON value.
    require(!options.endpoint.empty() && options.endpoint.find_first_of("\"\\\n\r") ==
      std::string::npos, "invalid Zenoh endpoint");
    config.insert_json5("mode", "\"peer\"");
    config.insert_json5("connect/endpoints", "[\"" + options.endpoint + "\"]");
    config.insert_json5("scouting/multicast/enabled", "false");
    session.emplace(zenoh::Session::open(std::move(config)));
    const auto deadline = monotonic_ns() + static_cast<std::int64_t>(options.startup_timeout * 1e9);
    pb::ArmDescription arm;
    while (true) {
      try {
        auto description = query<pb::RobotDescription>("description", "", 0.2);
        require(description.has_api_version() && description.api_version().major() == 0,
          "unsupported robot API major");
        arm = query<pb::ArmDescription>("arm/description", "", 0.2);
        break;
      } catch (const std::exception &) {
        if (monotonic_ns() >= deadline || !rclcpp::ok()) {throw;}
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    }
    require(arm.dof() == 6 && arm.joint_names_size() == 6 && options.joint_names.size() == 6,
      "expected six-joint arm description");
    const std::vector<std::string> wire_names(arm.joint_names().begin(), arm.joint_names().end());
    wire_to_ros = wire_joint_mapping(options.joint_names, options.device_joint_names, wire_names);
    require(std::find(arm.supported_timeouts().begin(), arm.supported_timeouts().end(),
      pb::TIMEOUT_BEHAVIOR_FAULT) != arm.supported_timeouts().end(), "FAULT watchdog required");
    subscribers.push_back(session->declare_subscriber(
      zenoh::KeyExpr(options.prefix + "/arm/joint_state"), [this](const zenoh::Sample & sample) {
        pb::JointState value;
        if (!value.ParseFromString(sample.get_payload().as_string()) || !value.has_header() ||
          !finite_six(value.q()) || !finite_six(value.dq()) || !finite_six(value.tau_est())) {return;}
        std::lock_guard<std::mutex> lock(data);
        if (latest.received_ns && value.header().stamp_ns() <= latest.source_stamp_ns) {return;}
        for (std::size_t i = 0; i < 6; ++i) {
          const auto dest = wire_to_ros[i];
          latest.position[dest] = value.q(static_cast<int>(i));
          latest.velocity[dest] = value.dq(static_cast<int>(i));
          latest.effort[dest] = value.tau_est(static_cast<int>(i));
        }
        latest.received_ns = monotonic_ns();
        latest.received_ros_ns = node->now().nanoseconds();
        latest.source_stamp_ns = value.header().stamp_ns();
        latest.sequence = value.header().seq();
        interpolation = value.interpolation();
        feedback.store(latest);
        trace.emit("direct_state_receive", latest.sequence, 0, latest.source_stamp_ns);
      }, zenoh::closures::none));
    subscribers.push_back(session->declare_subscriber(
      zenoh::KeyExpr(options.prefix + "/driver_state"), [this](const zenoh::Sample & sample) {
        pb::DriverState value;
        if (!value.ParseFromString(sample.get_payload().as_string()) || !value.has_header()) {return;}
        std::lock_guard<std::mutex> lock(data);
        if (driver_ns && value.header().stamp_ns() <= driver.header().stamp_ns()) {return;}
        driver = std::move(value); driver_ns = monotonic_ns();
        driver_ros_ns = node->now().nanoseconds(); gate_locked();
      }, zenoh::closures::none));
    subscribers.push_back(session->declare_subscriber(
      zenoh::KeyExpr(options.prefix + "/status"), [this](const zenoh::Sample & sample) {
        pb::RobotStatus value;
        if (!value.ParseFromString(sample.get_payload().as_string()) || !value.has_header()) {return;}
        std::lock_guard<std::mutex> lock(data);
        if (status_ns && value.header().stamp_ns() < status.header().stamp_ns()) {
          // Driver timestamps restart with its process. Never use a lease ID
          // from the previous process: IDs can be reused by another owner.
          remote_restarted = true; permitted.store(false);
          gate_reason = "driver clock regressed; reconfigure required";
          return;
        }
        if (status_ns && value.header().stamp_ns() == status.header().stamp_ns()) {return;}
        status = std::move(value); status_ns = monotonic_ns();
        disable_receipt.observe_holder(status.session_holder()); gate_locked();
      }, zenoh::closures::none));
    setup_ros();
    stream = std::thread([this]() {
        while (!stopping.load()) {
          try {send_latest();} catch (const std::exception & error) {
            std::lock_guard<std::mutex> lock(data);
            permitted.store(false); gate_reason = error.what();
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
      });
    diagnostic_worker = std::thread([this]() {
        while (!stopping.load() && rclcpp::ok()) {
          try {publish();} catch (const std::exception & error) {
            if (!stopping.load() && rclcpp::ok()) {
              RCLCPP_ERROR(node->get_logger(), "diagnostic publication failed: %s", error.what());
            }
            break;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
      });
  }

  void send_latest()
  {
    {std::lock_guard<std::mutex> lock(data); gate_locked();}
    if (!permitted.load()) {return;}
    std::unique_lock<std::mutex> transition(management, std::try_to_lock);
    if (!transition.owns_lock()) {return;}
    CommandSnapshot target;
    if (!commands.load(target)) {return;}
    // Serialize admission with actuator transactions and gate revocation.
    std::lock_guard<std::mutex> lock(data);
    gate_locked();
    if (!permitted.load() || !freshness.accept(target, generation, activated_ns,
      monotonic_ns(), static_cast<std::int64_t>(options.command_max_age * 1e9))) {return;}
    pb::JointTrajectory message;
    message.mutable_header()->set_seq(target.sequence);
    message.mutable_header()->set_stamp_ns(target.created_ns);
    message.set_session_id(session_id);
    message.set_on_timeout(pb::TIMEOUT_BEHAVIOR_FAULT);
    message.add_t_from_start_ns(static_cast<std::int64_t>(options.command_period * 1e9));
    auto * point = message.add_points();
    for (std::size_t i = 0; i < 6; ++i) {
      point->add_q(static_cast<float>(target.position[wire_to_ros[i]]));
      point->add_dq(static_cast<float>(target.velocity[wire_to_ros[i]]));
    }
    // Empty gains and feed-forward preserve reviewed Rust profile defaults.
    auto put_options = zenoh::Session::PutOptions::create_default();
    put_options.congestion_control = Z_CONGESTION_CONTROL_DROP;
    put_options.is_express = true;
    session->put(zenoh::KeyExpr(options.prefix + "/arm/command"),
      zenoh::Bytes(message.SerializeAsString()), std::move(put_options));
    last_sent_command = target;
    trace.emit("direct_put", target.sequence, generation, target.created_ns);
  }

  bool release()  // management lock held, or workers already stopped
  {
    permitted.store(false);
    if (!session_id) {return !acquisition_uncertain;}
    {
      std::lock_guard<std::mutex> lock(data);
      if (remote_restarted || (ownership_confirmed && status.session_holder() != session_id)) {
        gate_reason = "session invalidated; remote disable cannot be acknowledged by this owner";
        return false;
      }
    }
    try {
      pb::ReleaseSessionRequest request; request.set_session_id(session_id);
      // Rust release acknowledges only after disable_all succeeds.
      checked("rpc/release_session", request);
      std::lock_guard<std::mutex> lock(data);
      disable_receipt.acknowledge(session_id);
      session_id = 0;
      ownership_confirmed = false;
      return true;
    } catch (const std::exception & error) {
      std::lock_guard<std::mutex> lock(data);
      gate_reason = std::string("release unconfirmed; lease retained: ") + error.what();
      RCLCPP_ERROR(node->get_logger(), "%s", gate_reason.c_str());
      return false;
    }
  }

  bool verified_disable()  // management lock held by the service wrapper
  {
    if (!release()) {return false;}
    {
      std::lock_guard<std::mutex> lock(data);
      if (!disable_receipt.acknowledged()) {return false;}
    }
    const auto deadline = monotonic_ns() + static_cast<std::int64_t>(options.service_timeout * 1e9);
    while (monotonic_ns() < deadline) {
      {
        std::lock_guard<std::mutex> lock(data);
        const auto now = monotonic_ns();
        const auto maximum_age = static_cast<std::int64_t>(options.state_timeout * 1e9);
        const bool fresh = driver_ns > 0 && status_ns > 0 && now >= driver_ns && now >= status_ns &&
          now - driver_ns <= maximum_age && now - status_ns <= maximum_age;
        const bool inactive = driver.mode() == pb::OPERATING_MODE_DISABLED ||
          driver.mode() == pb::OPERATING_MODE_FAULT;
        if (disable_receipt.confirms(fresh, inactive, status.session_holder(), remote_restarted)) {
          return true;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
  }

  bool activate(std::uint64_t epoch)
  {
    std::lock_guard<std::mutex> transition(management);
    if (acquisition_uncertain) {return false;}
    if (session_id && !release()) {return false;}
    try {
      const auto ready_deadline = monotonic_ns() +
        static_cast<std::int64_t>(options.service_timeout * 1e9);
      while (true) {
        {
          std::lock_guard<std::mutex> lock(data);
          require(!remote_restarted, "driver restarted; reconfigure before activation");
          const auto error = readiness(false); require(error.empty(), error);
          if (driver.mode() == pb::OPERATING_MODE_DISABLED && status.session_holder() == 0) {break;}
        }
        require(monotonic_ns() < ready_deadline && rclcpp::ok(),
          "activation requires DISABLED and no external session holder");
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      pb::AcquireSessionRequest request; request.set_client_name("ros2_control_zenoh_cpp");
      acquisition_uncertain = true;
      auto acquired = query<pb::AcquireSessionResponse>("rpc/acquire_session",
        request.SerializeAsString(), options.service_timeout);
      acquisition_uncertain = false;
      require(acquired.ok() && acquired.session_id(), "exclusive session acquisition failed");
      {
        std::lock_guard<std::mutex> lock(data);
        session_id = acquired.session_id();
        disable_receipt.invalidate();
        ownership_confirmed = false;
        const auto error = readiness(false); require(error.empty(), error);
      }
      pb::SetModeRequest mode;
      mode.set_session_id(session_id); mode.set_mode(pb::OPERATING_MODE_ACTIVE);
      const auto requested_ns = monotonic_ns();
      checked("rpc/set_mode", mode);
      const auto deadline = monotonic_ns() + static_cast<std::int64_t>(options.service_timeout * 1e9);
      while (monotonic_ns() < deadline && rclcpp::ok()) {
        {
          std::lock_guard<std::mutex> lock(data);
          if (driver_ns > requested_ns && status_ns > requested_ns &&
            readiness(true, session_id).empty())
          {
            generation = epoch; activated_ns = monotonic_ns(); gate_reason.clear();
            ownership_confirmed = true;
            permitted.store(true);
            return true;
          }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      throw std::runtime_error("new ACTIVE feedback and exact session holder not confirmed");
    } catch (const std::exception & error) {
      RCLCPP_ERROR(node->get_logger(), "activation rejected: %s", error.what());
      (void)release();
      return false;
    }
  }

  template<class Service, class Callback>
  void service(const std::string & name, Callback callback)
  {
    services.push_back(node->create_service<Service>(name,
      [this, callback](const std::shared_ptr<typename Service::Request> request,
      std::shared_ptr<typename Service::Response> response) {
        std::lock_guard<std::mutex> transition(management);
        try {callback(*request, *response);} catch (const std::exception & error) {
          response->success = false; response->message = error.what();
        }
      }));
  }

  void setup_ros()
  {
    state_pub = node->create_publisher<sensor_msgs::msg::JointState>(
      "hex_arm/internal/state", rclcpp::SensorDataQoS());
    driver_pub = node->create_publisher<hex_arm_msgs::msg::DriverState>("hex_arm/driver_state", 10);
    timing_pub = node->create_publisher<hex_arm_msgs::msg::StateTiming>("hex_arm/state_timing", 10);
    interpolation_pub = node->create_publisher<hex_arm_msgs::msg::InterpolationState>(
      "hex_arm/interpolation_state", rclcpp::SensorDataQoS());
    command_pub = node->create_publisher<sensor_msgs::msg::JointState>(
      "hex_arm/internal/command", rclcpp::SensorDataQoS());
    diagnostics = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    // No activation service: only the hardware lifecycle can enable command
    // streaming and establish a command epoch. Tools use this same manager.
    service<std_srvs::srv::Trigger>("hex_arm/deactivate_hardware",
      [this](auto &, auto & response) {response.success = verified_disable();
        response.message = response.success ? "physical disable RPC and current inactive state confirmed" :
          "physical disable unconfirmed; no successful owned disable receipt or fresh inactive state";});
    service<std_srvs::srv::Trigger>("hex_arm/damped_stop", [this](auto &, auto & response) {
        require(permitted.load() && session_id, "damped stop requires ACTIVE owned session");
        permitted.store(false);
        pb::DampedStopRequest request; request.set_session_id(session_id);
        checked("rpc/damped_stop", request, 35.0);
        {std::lock_guard<std::mutex> lock(data);
          disable_receipt.acknowledge(session_id); session_id = 0;}
        response.success = true; response.message = "folded, settled and disabled";
      });
    service<std_srvs::srv::Trigger>("hex_arm/clear_fault", [this](auto &, auto & response) {
        require(!permitted.load(), "deactivate hardware before fault recovery");
        require(!acquisition_uncertain, "previous acquisition unconfirmed; driver restart required");
        if (session_id) {require(release(), "previous lease release unconfirmed");}
        pb::AcquireSessionRequest acquire; acquire.set_client_name("ros2_cpp_fault_recovery");
        acquisition_uncertain = true;
        const auto owned = query<pb::AcquireSessionResponse>("rpc/acquire_session",
          acquire.SerializeAsString(), options.service_timeout);
        acquisition_uncertain = false;
        require(owned.ok() && owned.session_id(), "fault recovery session unavailable");
        {std::lock_guard<std::mutex> lock(data);
          session_id = owned.session_id(); disable_receipt.invalidate();}
        try {
          pb::ClearFaultRequest request; request.set_session_id(session_id);
          checked("rpc/clear_fault", request);
        } catch (...) {(void)release(); throw;}
        require(release(), "fault recovery release unconfirmed");
        response.success = true; response.message = "fault cleared; explicit reactivation required";
      });
    service<hex_arm_msgs::srv::SetOperatingMode>("hex_arm/set_mode",
      [this](auto & request, auto & response) {
        require(request.mode == pb::OPERATING_MODE_DISABLED,
          "direct backend tools only allow DISABLED; ACTIVE belongs to hardware lifecycle");
        response.success = release(); response.message = "disable/session release requested";
      });
    service<hex_arm_msgs::srv::SetGravity>("hex_arm/set_gravity",
      [this](auto & request, auto & response) {
        require(session_id != 0, "no owned session");
        pb::SetGravityRequest payload; payload.set_session_id(session_id);
        payload.mutable_gravity()->set_x(static_cast<float>(request.x));
        payload.mutable_gravity()->set_y(static_cast<float>(request.y));
        payload.mutable_gravity()->set_z(static_cast<float>(request.z));
        checked("arm/rpc/set_gravity", payload);
        response.success = true; response.message = "gravity updated";
      });
    service<hex_arm_msgs::srv::DiscoverMotors>("hex_arm/discover_motors",
      [this](auto & request, auto & response) {
        require(!request.refresh || !permitted.load(), "active refresh is forbidden");
        pb::DiscoverMotorsRequest payload; payload.set_refresh(request.refresh);
        const auto result = query<pb::DiscoverMotorsResponse>("arm/rpc/discover",
          payload.SerializeAsString(), options.service_timeout);
        response.success = result.ok(); response.message = result.error();
        for (const auto & motor : result.motors()) {response.motors.push_back(motor_message(motor));}
      });
  }

  void publish()
  {
    FeedbackSnapshot snapshot;
    pb::DriverState driver_copy;
    pb::InterpolationState interpolation_copy;
    CommandSnapshot command_copy;
    std::string reason;
    bool driver_fresh = false;
    std::int64_t driver_received_ros_ns = 0;
    {
      std::lock_guard<std::mutex> lock(data);
      snapshot = latest; driver_copy = driver; reason = gate_reason;
      interpolation_copy = interpolation; command_copy = last_sent_command;
      driver_received_ros_ns = driver_ros_ns;
      driver_fresh = driver_ns && monotonic_ns() - driver_ns <=
        static_cast<std::int64_t>(options.state_timeout * 1e9);
      if (reason.empty()) {reason = readiness(false);}
    }
    const auto now = node->now();
    const auto published_steady_ns = monotonic_ns();
    if (command_copy.sequence && command_copy.sequence != published_command_sequence) {
      sensor_msgs::msg::JointState message;
      message.header.stamp = now;
      message.name = options.joint_names;
      message.position.assign(command_copy.position.begin(), command_copy.position.end());
      message.velocity.assign(command_copy.velocity.begin(), command_copy.velocity.end());
      command_pub->publish(message);
      published_command_sequence = command_copy.sequence;
    }
    if (interpolation_copy.sample_stamp_ns() > published_interpolation_stamp &&
      finite_six(interpolation_copy.command_position()) &&
      finite_six(interpolation_copy.command_velocity()) &&
      finite_six(interpolation_copy.interpolated_position()) &&
      finite_six(interpolation_copy.interpolated_velocity()))
    {
      hex_arm_msgs::msg::InterpolationState message;
      message.stamp = now; message.joint_names = options.joint_names;
      message.source_stamp_ns = static_cast<std::uint64_t>(snapshot.source_stamp_ns);
      message.command_sequence = interpolation_copy.command_sequence();
      message.command_generation = interpolation_copy.command_generation();
      message.segment_start_ns = interpolation_copy.segment_start_ns();
      message.requested_duration_ns = interpolation_copy.requested_duration_ns();
      message.effective_duration_ns = interpolation_copy.effective_duration_ns();
      message.sample_stamp_ns = interpolation_copy.sample_stamp_ns();
      for (std::size_t i = 0; i < 6; ++i) {
        const auto source = static_cast<int>(i);
        const auto dest = wire_to_ros[i];
        message.command_position[dest] = interpolation_copy.command_position(source);
        message.command_velocity[dest] = interpolation_copy.command_velocity(source);
        message.interpolated_position[dest] = interpolation_copy.interpolated_position(source);
        message.interpolated_velocity[dest] = interpolation_copy.interpolated_velocity(source);
      }
      interpolation_pub->publish(message);
      published_interpolation_stamp = interpolation_copy.sample_stamp_ns();
    }
    if (publication.accept(snapshot, published_steady_ns,
      static_cast<std::int64_t>(options.state_timeout * 1e9)))
    {
      sensor_msgs::msg::JointState message;
      message.header.stamp = rclcpp::Time(snapshot.received_ros_ns, node->get_clock()->get_clock_type());
      message.name = options.joint_names;
      message.position.assign(snapshot.position.begin(), snapshot.position.end());
      message.velocity.assign(snapshot.velocity.begin(), snapshot.velocity.end());
      message.effort.assign(snapshot.effort.begin(), snapshot.effort.end());
      state_pub->publish(message);
    }
    if (snapshot.received_ns) {
      hex_arm_msgs::msg::StateTiming timing;
      timing.stamp = now;
      timing.received_stamp = rclcpp::Time(snapshot.received_ros_ns, node->get_clock()->get_clock_type());
      timing.sequence = snapshot.sequence;
      timing.source_stamp_ns = snapshot.source_stamp_ns;
      timing.received_steady_ns = snapshot.received_ns;
      timing.receive_age_s = static_cast<double>(published_steady_ns - snapshot.received_ns) * 1e-9;
      timing.feedback_fresh = published_steady_ns >= snapshot.received_ns &&
        timing.receive_age_s <= options.state_timeout;
      timing.source_clock = timing.DRIVER_PROCESS_MONOTONIC;
      timing.motor_sample_time_available = false;
      timing.clocks_synchronized = false;
      timing.instance = options.hardware_name;
      timing_pub->publish(timing);
    }
    hex_arm_msgs::msg::DriverState message;
    message.stamp = rclcpp::Time(driver_received_ros_ns, node->get_clock()->get_clock_type());
    message.mode = static_cast<std::uint8_t>(driver_copy.mode());
    message.session_owned = driver_copy.session_owned(); message.profile_valid = driver_copy.profile_valid();
    message.calibrated = driver_copy.calibrated(); message.all_motors_online = driver_copy.all_motors_online();
    message.feedback_fresh = driver_fresh && driver_copy.feedback_fresh();
    message.fault_latched = driver_copy.fault_latched();
    message.fault_code = driver_copy.fault_code(); message.fault_reason = driver_copy.fault_reason();
    message.command_age_s = driver_copy.command_age_s(); message.feedback_age_s = driver_copy.feedback_age_s();
    for (const auto & motor : driver_copy.motors()) {message.motors.push_back(motor_message(motor));}
    driver_pub->publish(message);
    // Formatting at 5 Hz does not participate in feedback delivery or gating.
    if (++diagnostic_ticks % 20 != 0) {return;}
    diagnostic_msgs::msg::DiagnosticArray report; report.header.stamp = now;
    diagnostic_msgs::msg::DiagnosticStatus health;
    health.name = std::string(node->get_fully_qualified_name()) + "/zenoh_transport";
    health.hardware_id = options.prefix;
    health.level = reason.empty() && !message.fault_latched ? health.OK : health.ERROR;
    health.message = reason.empty() ? (permitted.load() ? "ACTIVE" : "INACTIVE") : reason;
    report.status.push_back(health); diagnostics->publish(report);
  }
  unsigned diagnostic_ticks{};
};

ZenohTransport::ZenohTransport(const ZenohOptions & options, const rclcpp::Node::SharedPtr & node,
  SnapshotMailbox<FeedbackSnapshot> & feedback, SnapshotMailbox<CommandSnapshot> & commands,
  TransportTrace & trace)
: impl_(std::make_unique<Impl>(options, node, feedback, commands, trace)) {impl_->init();}
ZenohTransport::~ZenohTransport() = default;
bool ZenohTransport::activate(std::uint64_t generation) {return impl_->activate(generation);}
bool ZenohTransport::deactivate()
{
  std::lock_guard<std::mutex> lock(impl_->management); return impl_->release();
}
bool ZenohTransport::allowed() const {return impl_->permitted.load();}
}  // namespace hex_arm_hardware
