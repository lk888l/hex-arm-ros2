#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <type_traits>

namespace hex_arm_hardware
{

inline std::int64_t monotonic_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

// One serialized producer, any number of readers. Payload words are atomic,
// unlike a seqlock over ordinary C++ objects (which has a data race). All
// accesses use the single seq_cst order: equal even sequence observations
// bracket a coherent payload. A reader makes at most two attempts and keeps
// its previous snapshot on contention. No locks, allocation, spin loop or
// producer acknowledgement. Reset only after stopping all users.
template<class T>
class SnapshotMailbox
{
  static_assert(std::is_trivially_copyable_v<T>);
  static_assert(std::atomic<std::uint64_t>::is_always_lock_free);
  static constexpr std::size_t words = (sizeof(T) + 7) / 8;
public:
  SnapshotMailbox() {for (auto & word : payload_) {word.store(0);}}
  void store(const T & value)
  {
    std::array<std::uint64_t, words> encoded{};
    std::memcpy(encoded.data(), &value, sizeof(T));
    sequence_.store(++producer_sequence_);
    for (std::size_t i = 0; i < words; ++i) {payload_[i].store(encoded[i]);}
    sequence_.store(++producer_sequence_);
  }
  bool load(T & value) const
  {
    for (int attempt = 0; attempt < 2; ++attempt) {
      const auto before = sequence_.load();
      if (!before || (before & 1U)) {continue;}
      std::array<std::uint64_t, words> encoded{};
      for (std::size_t i = 0; i < words; ++i) {encoded[i] = payload_[i].load();}
      if (before == sequence_.load()) {
        std::memcpy(static_cast<void *>(&value), encoded.data(), sizeof(T));
        return true;
      }
    }
    return false;
  }
  void reset() {producer_sequence_ = 0; sequence_.store(0);}
private:
  std::uint64_t producer_sequence_{0};  // producer only; no CAS/LL-SC retry loop
  alignas(64) std::atomic<std::uint64_t> sequence_{0};
  std::array<std::atomic<std::uint64_t>, words> payload_{};
};

struct FeedbackSnapshot
{
  std::array<double, 6> position{}, velocity{}, effort{};
  std::int64_t received_ns{}, source_stamp_ns{};
  std::uint64_t sequence{};
};

struct CommandSnapshot
{
  std::array<double, 6> position{}, velocity{};
  std::int64_t created_ns{};
  std::uint64_t sequence{}, generation{};
};

// Consumed identities never become sendable again, including expired samples.
// Re-activation sets a new generation and rejects all pre-activation writes.
class CommandFreshness
{
public:
  bool accept(const CommandSnapshot & command, std::uint64_t generation,
    std::int64_t activated_ns, std::int64_t now_ns, std::int64_t max_age_ns)
  {
    if (!command.sequence || command.sequence <= last_sequence_) {return false;}
    last_sequence_ = command.sequence;
    return command.generation == generation && command.created_ns >= activated_ns &&
           command.created_ns <= now_ns && now_ns - command.created_ns <= max_age_ns;
  }
private:
  std::uint64_t last_sequence_{0};
};

}  // namespace hex_arm_hardware
