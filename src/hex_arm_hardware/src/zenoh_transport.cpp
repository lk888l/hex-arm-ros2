#include "hex_arm_hardware/zenoh_transport.hpp"

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
  std::int64_t driver_ns{}, status_ns{}, activated_ns{};
  std::array<std::size_t, 6> wire_to_ros{};
  std::uint32_t session_id{};
  std::uint64_t generation{};
  std::atomic_bool permitted{false}, stopping{false};
  std::string gate_reason;
  bool remote_restarted{false};
  bool acquisition_uncertain{false};
  bool ownership_confirmed{false};
  CommandFreshness freshness;
  std::optional<zenoh::Session> session;
  std::vector<zenoh::Subscriber<void>> subscribers;
  std::thread stream, diagnostic_worker;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub;
  rclcpp::Publisher<hex_arm_msgs::msg::DriverState>::SharedPtr driver_pub;
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
    std::array<bool, 6> seen{};
    for (std::size_t i = 0; i < 6; ++i) {
      auto found = std::find(options.joint_names.begin(), options.joint_names.end(),
        arm.joint_names(static_cast<int>(i)));
      require(found != options.joint_names.end(), "arm joint name mismatch");
      wire_to_ros[i] = static_cast<std::size_t>(found - options.joint_names.begin());
      require(!seen[wire_to_ros[i]], "duplicate arm joint name");
      seen[wire_to_ros[i]] = true;
    }
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
        latest.received_ns = monotonic_ns(); latest.source_stamp_ns = value.header().stamp_ns();
        latest.sequence = value.header().seq();
        feedback.store(latest);
        trace.emit("direct_state_receive", latest.sequence, 0, latest.source_stamp_ns);
      }, zenoh::closures::none));
    subscribers.push_back(session->declare_subscriber(
      zenoh::KeyExpr(options.prefix + "/driver_state"), [this](const zenoh::Sample & sample) {
        pb::DriverState value;
        if (!value.ParseFromString(sample.get_payload().as_string()) || !value.has_header()) {return;}
        std::lock_guard<std::mutex> lock(data);
        if (driver_ns && value.header().stamp_ns() <= driver.header().stamp_ns()) {return;}
        driver = std::move(value); driver_ns = monotonic_ns(); gate_locked();
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
        status = std::move(value); status_ns = monotonic_ns(); gate_locked();
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
      "/hex_arm/internal/state", rclcpp::SensorDataQoS());
    driver_pub = node->create_publisher<hex_arm_msgs::msg::DriverState>("/hex_arm/driver_state", 10);
    diagnostics = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    // No activation service: only the hardware lifecycle can enable command
    // streaming and establish a command epoch. Tools use this same manager.
    service<std_srvs::srv::Trigger>("/hex_arm/deactivate_hardware",
      [this](auto &, auto & response) {response.success = release();
        response.message = response.success ? "disabled and session released" : "release unconfirmed";});
    service<std_srvs::srv::Trigger>("/hex_arm/damped_stop", [this](auto &, auto & response) {
        require(permitted.load() && session_id, "damped stop requires ACTIVE owned session");
        permitted.store(false);
        pb::DampedStopRequest request; request.set_session_id(session_id);
        checked("rpc/damped_stop", request, 35.0);
        {std::lock_guard<std::mutex> lock(data); session_id = 0;}
        response.success = true; response.message = "folded, settled and disabled";
      });
    service<std_srvs::srv::Trigger>("/hex_arm/clear_fault", [this](auto &, auto & response) {
        require(!permitted.load(), "deactivate hardware before fault recovery");
        require(!acquisition_uncertain, "previous acquisition unconfirmed; driver restart required");
        if (session_id) {require(release(), "previous lease release unconfirmed");}
        pb::AcquireSessionRequest acquire; acquire.set_client_name("ros2_cpp_fault_recovery");
        acquisition_uncertain = true;
        const auto owned = query<pb::AcquireSessionResponse>("rpc/acquire_session",
          acquire.SerializeAsString(), options.service_timeout);
        acquisition_uncertain = false;
        require(owned.ok() && owned.session_id(), "fault recovery session unavailable");
        {std::lock_guard<std::mutex> lock(data); session_id = owned.session_id();}
        try {
          pb::ClearFaultRequest request; request.set_session_id(session_id);
          checked("rpc/clear_fault", request);
        } catch (...) {(void)release(); throw;}
        require(release(), "fault recovery release unconfirmed");
        response.success = true; response.message = "fault cleared; explicit reactivation required";
      });
    service<hex_arm_msgs::srv::SetOperatingMode>("/hex_arm/set_mode",
      [this](auto & request, auto & response) {
        require(request.mode == pb::OPERATING_MODE_DISABLED,
          "direct backend tools only allow DISABLED; ACTIVE belongs to hardware lifecycle");
        response.success = release(); response.message = "disable/session release requested";
      });
    service<hex_arm_msgs::srv::SetGravity>("/hex_arm/set_gravity",
      [this](auto & request, auto & response) {
        require(session_id != 0, "no owned session");
        pb::SetGravityRequest payload; payload.set_session_id(session_id);
        payload.mutable_gravity()->set_x(static_cast<float>(request.x));
        payload.mutable_gravity()->set_y(static_cast<float>(request.y));
        payload.mutable_gravity()->set_z(static_cast<float>(request.z));
        checked("arm/rpc/set_gravity", payload);
        response.success = true; response.message = "gravity updated";
      });
    service<hex_arm_msgs::srv::DiscoverMotors>("/hex_arm/discover_motors",
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
    std::string reason;
    bool driver_fresh = false;
    {
      std::lock_guard<std::mutex> lock(data);
      snapshot = latest; driver_copy = driver; reason = gate_reason;
      driver_fresh = driver_ns && monotonic_ns() - driver_ns <=
        static_cast<std::int64_t>(options.state_timeout * 1e9);
      if (reason.empty()) {reason = readiness(false);}
    }
    const auto now = node->now();
    if (snapshot.received_ns && monotonic_ns() - snapshot.received_ns <=
      static_cast<std::int64_t>(options.state_timeout * 1e9))
    {
      sensor_msgs::msg::JointState message;
      message.header.stamp = now; message.name = options.joint_names;
      message.position.assign(snapshot.position.begin(), snapshot.position.end());
      message.velocity.assign(snapshot.velocity.begin(), snapshot.velocity.end());
      message.effort.assign(snapshot.effort.begin(), snapshot.effort.end());
      state_pub->publish(message);
    }
    hex_arm_msgs::msg::DriverState message;
    message.stamp = now; message.mode = static_cast<std::uint8_t>(driver_copy.mode());
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
    health.name = "hex_arm/zenoh_transport"; health.hardware_id = options.prefix;
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
