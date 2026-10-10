#include <gtest/gtest.h>
#include <thread>
#include "hex_arm_hardware/snapshot_mailbox.hpp"
using namespace hex_arm_hardware;

TEST(SnapshotMailbox, RetainsLatestAndLeavesOutputUntouchedWhenEmpty)
{
  SnapshotMailbox<CommandSnapshot> mailbox;
  CommandSnapshot output; output.sequence = 42;
  EXPECT_FALSE(mailbox.load(output)); EXPECT_EQ(output.sequence, 42U);
  CommandSnapshot input;
  for (std::uint64_t n = 1; n <= 1000; ++n) {input.sequence = n; mailbox.store(input);}
  ASSERT_TRUE(mailbox.load(output)); EXPECT_EQ(output.sequence, 1000U);
  mailbox.reset(); EXPECT_FALSE(mailbox.load(output)); EXPECT_EQ(output.sequence, 1000U);
}

TEST(SnapshotMailbox, ConcurrentReadersNeverObserveTornPayload)
{
  SnapshotMailbox<FeedbackSnapshot> mailbox;
  std::atomic_bool done{false};
  std::atomic_uint64_t accepted{0}, torn{0};
  auto reader = [&]() {
      FeedbackSnapshot value;
      while (!done.load()) {
        if (!mailbox.load(value)) {continue;}
        ++accepted;
        for (std::size_t i = 0; i < 6; ++i) {
          if (value.position[i] != static_cast<double>(value.sequence) ||
            value.velocity[i] != -static_cast<double>(value.sequence) ||
            value.effort[i] != static_cast<double>(value.sequence * 2)) {++torn;}
        }
      }
    };
  std::thread one(reader), two(reader);
  FeedbackSnapshot value;
  for (std::uint64_t n = 1; n <= 100000; ++n) {
    value.sequence = n; value.position.fill(static_cast<double>(n));
    value.velocity.fill(-static_cast<double>(n)); value.effort.fill(static_cast<double>(n * 2));
    mailbox.store(value);
    if (n % 100 == 0) {std::this_thread::yield();}
  }
  done.store(true); one.join(); two.join();
  EXPECT_GT(accepted.load(), 0U); EXPECT_EQ(torn.load(), 0U);
}

TEST(CommandFreshness, ConstantPositionStillNeedsNewCycleAndAge)
{
  CommandFreshness gate;
  CommandSnapshot target; target.sequence = 1; target.generation = 7; target.created_ns = 100;
  EXPECT_TRUE(gate.accept(target, 7, 90, 110, 50));
  EXPECT_FALSE(gate.accept(target, 7, 90, 111, 50));
  ++target.sequence; target.created_ns = 120;
  EXPECT_TRUE(gate.accept(target, 7, 90, 130, 50));
  ++target.sequence;
  EXPECT_FALSE(gate.accept(target, 7, 90, 180, 50));
  EXPECT_FALSE(gate.accept(target, 7, 90, 130, 50));
}

TEST(CommandFreshness, ActivationAndFutureTimeCannotReviveQueuedCommands)
{
  CommandFreshness gate;
  CommandSnapshot target; target.sequence = 1; target.generation = 7; target.created_ns = 100;
  EXPECT_FALSE(gate.accept(target, 8, 120, 130, 50));
  ++target.sequence; target.generation = 8;
  EXPECT_FALSE(gate.accept(target, 8, 120, 130, 50));
  ++target.sequence; target.created_ns = 140;
  EXPECT_FALSE(gate.accept(target, 8, 120, 130, 50));
  ++target.sequence; target.created_ns = 135;
  EXPECT_TRUE(gate.accept(target, 8, 120, 140, 50));
}
