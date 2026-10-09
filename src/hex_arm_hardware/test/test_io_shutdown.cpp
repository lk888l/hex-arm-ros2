#include <gtest/gtest.h>
#include <memory>
#include <future>
#include "hex_arm_hardware/hex_arm_system.hpp"
#include "rclcpp/rclcpp.hpp"

class HardwareIoShutdown : public ::testing::Test
{
protected:
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override {if (rclcpp::ok()) {rclcpp::shutdown();}}
};

TEST_F(HardwareIoShutdown, DestructionAfterConfigureJoinsExecutor)
{
  auto system = std::make_unique<hex_arm_hardware::HexArmSystem>();
  ASSERT_EQ(system->on_configure(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  // Activation may fail and ResourceManager can then destroy this object
  // without invoking on_cleanup. A joinable std::thread must not terminate.
  system.reset();
}

TEST_F(HardwareIoShutdown, ShutdownThenDestructionIsIdempotent)
{
  auto system = std::make_unique<hex_arm_hardware::HexArmSystem>();
  ASSERT_EQ(system->on_configure(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(system->on_shutdown(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  system.reset();
}

TEST_F(HardwareIoShutdown, DestructionAfterRosContextShutdownJoinsExecutor)
{
  auto system = std::make_unique<hex_arm_hardware::HexArmSystem>();
  ASSERT_EQ(system->on_configure(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  rclcpp::shutdown();
  system.reset();
}

TEST_F(HardwareIoShutdown, ContextShutdownInterruptsPendingSafetyResponse)
{
  auto server = std::make_shared<rclcpp::Node>("unspun_safety_server");
  auto service = server->create_service<std_srvs::srv::Trigger>(
    "/hex_arm_bridge/deactivate_hardware",
    [](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
       std::shared_ptr<std_srvs::srv::Trigger::Response>) {});
  auto system = std::make_unique<hex_arm_hardware::HexArmSystem>();
  ASSERT_EQ(system->on_configure(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  auto pending = std::async(std::launch::async, [&]() {
    return system->on_deactivate(rclcpp_lifecycle::State());
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(pending.wait_for(std::chrono::milliseconds(1)), std::future_status::timeout);
  rclcpp::shutdown();
  ASSERT_EQ(pending.wait_for(std::chrono::milliseconds(500)), std::future_status::ready);
  EXPECT_EQ(pending.get(), hardware_interface::CallbackReturn::ERROR);
}

TEST_F(HardwareIoShutdown, ServiceDisappearanceInterruptsPendingSafetyResponse)
{
  auto server = std::make_shared<rclcpp::Node>("disappearing_safety_server");
  auto service = server->create_service<std_srvs::srv::Trigger>(
    "/hex_arm_bridge/deactivate_hardware",
    [](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
       std::shared_ptr<std_srvs::srv::Trigger::Response>) {});
  auto system = std::make_unique<hex_arm_hardware::HexArmSystem>();
  ASSERT_EQ(system->on_configure(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  auto pending = std::async(std::launch::async, [&]() {
    return system->on_deactivate(rclcpp_lifecycle::State());
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  EXPECT_EQ(pending.wait_for(std::chrono::milliseconds(1)), std::future_status::timeout);
  service.reset();
  ASSERT_EQ(pending.wait_for(std::chrono::milliseconds(1000)), std::future_status::ready);
  EXPECT_EQ(pending.get(), hardware_interface::CallbackReturn::ERROR);
}

TEST_F(HardwareIoShutdown, MissingStopServiceDoesNotRestartDiscoveryDuringErrorRecovery)
{
  auto system = std::make_unique<hex_arm_hardware::HexArmSystem>();
  ASSERT_EQ(system->on_configure(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  auto pending = std::async(std::launch::async, [&]() {
    EXPECT_EQ(system->on_deactivate(rclcpp_lifecycle::State()),
      hardware_interface::CallbackReturn::ERROR);
    return system->on_error(rclcpp_lifecycle::State());
  });
  ASSERT_EQ(pending.wait_for(std::chrono::milliseconds(500)), std::future_status::ready);
  EXPECT_EQ(pending.get(), hardware_interface::CallbackReturn::SUCCESS);
}

TEST_F(HardwareIoShutdown, FreshFeedbackAloneCannotEnableBeforeCommandSubscriberMatches)
{
  auto server = std::make_shared<rclcpp::Node>("activation_readiness_server");
  std::atomic_uint enables{0};
  auto service = server->create_service<std_srvs::srv::Trigger>(
    "/hex_arm_bridge/activate_hardware",
    [&](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
       std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      ++enables;
      response->success = true;
    });
  auto publisher = server->create_publisher<sensor_msgs::msg::JointState>(
    "/hex_arm/internal/state", rclcpp::SensorDataQoS());
  sensor_msgs::msg::JointState feedback;
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info.name = "readiness_test";
  params.hardware_info.hardware_parameters["activation_timeout_sec"] = "0.3";
  for (int i = 1; i <= 6; ++i) {
    hardware_interface::ComponentInfo joint;
    joint.name = "joint_" + std::to_string(i);
    for (const auto & name : {"position", "velocity", "effort"}) {
      hardware_interface::InterfaceInfo interface;
      interface.name = name;
      joint.state_interfaces.push_back(interface);
      if (interface.name != "effort") {
        joint.command_interfaces.push_back(interface);
      }
    }
    params.hardware_info.joints.push_back(joint);
    feedback.name.push_back(joint.name);
  }
  feedback.position.assign(6, 0.0);
  feedback.velocity.assign(6, 0.0);
  feedback.effort.assign(6, 0.0);
  auto system = std::make_unique<hex_arm_hardware::HexArmSystem>();
  ASSERT_EQ(system->on_init(params), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system->on_configure(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  auto timer = server->create_wall_timer(std::chrono::milliseconds(10), [&]() {
    publisher->publish(feedback);
  });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server);
  std::atomic_bool running{true};
  std::thread spinner([&]() {
    while (running.load()) {executor.spin_once(std::chrono::milliseconds(10));}
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  const auto before_matching = system->on_activate(rclcpp_lifecycle::State());
  const auto calls_before_matching = enables.load();
  auto command_subscriber = server->create_subscription<sensor_msgs::msg::JointState>(
    "/hex_arm/internal/command", 1, [](sensor_msgs::msg::JointState::ConstSharedPtr) {});
  const auto after_matching = system->on_activate(rclcpp_lifecycle::State());
  running.store(false);
  spinner.join();
  EXPECT_EQ(before_matching, hardware_interface::CallbackReturn::ERROR);
  EXPECT_EQ(calls_before_matching, 0U);
  EXPECT_EQ(after_matching, hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(enables.load(), 1U);
}

TEST_F(HardwareIoShutdown, OptionalTraceUsesSharedClockAndDrainsBeforeDestruction)
{
  const auto directory = std::filesystem::temp_directory_path() /
    ("hex-arm-cxx-trace-" + std::to_string(getpid()));
  std::filesystem::create_directories(directory);
  const char * existing = std::getenv("HEX_ARM_TRACE_DIR");
  const std::string saved = existing ? existing : "";
  setenv("HEX_ARM_TRACE_DIR", directory.c_str(), 1);
  hex_arm_hardware::TransportTrace trace;
  trace.start();
  ASSERT_TRUE(trace.enabled());
  timespec before{};
  clock_gettime(CLOCK_MONOTONIC, &before);
  trace.emit("cxx_write", 7, 9, 123456);
  ASSERT_TRUE(trace.stop());
  EXPECT_FALSE(trace.enabled());
  if (existing) {setenv("HEX_ARM_TRACE_DIR", saved.c_str(), 1);} else {unsetenv("HEX_ARM_TRACE_DIR");}
  std::size_t files = 0;
  for (const auto & entry : std::filesystem::directory_iterator(directory)) {
    std::ifstream input(entry.path());
    std::string header, row;
    std::getline(input, header);
    std::getline(input, row);
    EXPECT_EQ(header, "timestamp_ns,pid,stage,seq,generation,source_stamp_ns");
    const auto timestamp = std::stoll(row.substr(0, row.find(',')));
    EXPECT_GE(timestamp, static_cast<std::int64_t>(before.tv_sec) * 1000000000LL + before.tv_nsec);
    EXPECT_NE(row.find(",cxx_write,7,9,123456"), std::string::npos);
    ++files;
  }
  EXPECT_EQ(files, 1U);
  std::filesystem::remove_all(directory);
}

TEST_F(HardwareIoShutdown, FeedbackReadTraceKeepsTheLastAcceptedFrameIdentity)
{
  const auto directory = std::filesystem::temp_directory_path() /
    ("hex-arm-cxx-feedback-trace-" + std::to_string(getpid()));
  std::filesystem::create_directories(directory);
  const char * existing = std::getenv("HEX_ARM_TRACE_DIR");
  const bool had_existing = existing != nullptr;
  const std::string saved = existing ? existing : "";
  setenv("HEX_ARM_TRACE_DIR", directory.c_str(), 1);

  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info.name = "feedback_trace_test";
  params.hardware_info.hardware_parameters["state_topic"] = "/hex_arm/trace_test_state";
  sensor_msgs::msg::JointState feedback;
  for (int i = 1; i <= 6; ++i) {
    hardware_interface::ComponentInfo joint;
    joint.name = "joint_" + std::to_string(i);
    for (const auto & name : {"position", "velocity", "effort"}) {
      hardware_interface::InterfaceInfo interface;
      interface.name = name;
      joint.state_interfaces.push_back(interface);
      if (interface.name != "effort") {joint.command_interfaces.push_back(interface);}
    }
    params.hardware_info.joints.push_back(joint);
    feedback.name.push_back(joint.name);
  }
  feedback.position.assign(6, 1.5);
  feedback.header.stamp.nanosec = 111;
  auto system = std::make_unique<hex_arm_hardware::HexArmSystem>();
  EXPECT_EQ(system->on_init(params), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(system->on_configure(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  auto state = system->export_state_interfaces();
  auto source = std::make_shared<rclcpp::Node>("feedback_trace_source");
  auto publisher = source->create_publisher<sensor_msgs::msg::JointState>(
    "/hex_arm/trace_test_state", rclcpp::SensorDataQoS());
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (publisher->get_subscription_count() == 0 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_GT(publisher->get_subscription_count(), 0U);
  while (state.front().get_optional<double>() != 1.5 && std::chrono::steady_clock::now() < deadline) {
    publisher->publish(feedback);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    system->read(rclcpp::Time(0), rclcpp::Duration(0, 10000000));
  }
  EXPECT_EQ(state.front().get_optional<double>(), 1.5);
  // An invalid newer packet is observed on DDS but cannot replace the frame
  // consumed by read(). Repeated reads must retain the accepted source stamp.
  feedback.name.pop_back();
  feedback.header.stamp.nanosec = 222;
  for (int i = 0; i < 5; ++i) {
    publisher->publish(feedback);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    system->read(rclcpp::Time(0), rclcpp::Duration(0, 10000000));
  }
  system.reset();
  if (had_existing) {setenv("HEX_ARM_TRACE_DIR", saved.c_str(), 1);} else {unsetenv("HEX_ARM_TRACE_DIR");}
  std::string contents;
  for (const auto & entry : std::filesystem::directory_iterator(directory)) {
    std::ifstream input(entry.path());
    contents.append(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
  }
  EXPECT_NE(contents.find(",cxx_read,0,0,111\n"), std::string::npos);
  EXPECT_NE(contents.find(",cxx_state_receive,0,0,222\n"), std::string::npos);
  EXPECT_EQ(contents.find(",cxx_read,0,0,222\n"), std::string::npos);
  std::filesystem::remove_all(directory);
}

TEST_F(HardwareIoShutdown, ErrorThenReconfigureJoinsOldExecutor)
{
  auto system = std::make_unique<hex_arm_hardware::HexArmSystem>();
  for (int iteration = 0; iteration < 3; ++iteration) {
    ASSERT_EQ(system->on_configure(rclcpp_lifecycle::State()),
      hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(system->on_error(rclcpp_lifecycle::State()),
      hardware_interface::CallbackReturn::SUCCESS);
  }
  ASSERT_EQ(system->on_configure(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
}

TEST_F(HardwareIoShutdown, ReconfigureWithoutCleanupJoinsOldExecutor)
{
  auto system = std::make_unique<hex_arm_hardware::HexArmSystem>();
  for (int iteration = 0; iteration < 3; ++iteration) {
    ASSERT_EQ(system->on_configure(rclcpp_lifecycle::State()),
      hardware_interface::CallbackReturn::SUCCESS);
  }
}
