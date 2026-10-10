#include <gtest/gtest.h>
#include <fstream>
#include <unistd.h>
#include "hex_arm_hardware/transport_trace.hpp"

TEST(TransportTrace, UsesSharedClockAndDrainsBeforeDestruction)
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

