#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <time.h>
#include <unistd.h>

namespace hex_arm_hardware
{

// Opt-in measurement only. Producers never wait for a lock or perform I/O.
// A full/contended bounded ring drops trace records, never control commands.
class TransportTrace
{
public:
  ~TransportTrace() {stop();}

  void start()
  {
    stop();
    const char * directory = std::getenv("HEX_ARM_TRACE_DIR");
    if (!directory || *directory == '\0') {return;}
    std::filesystem::create_directories(directory);
    auto state = std::make_shared<State>();
    state->pid = static_cast<std::int64_t>(getpid());
    const auto path = std::filesystem::path(directory) /
      ("cxx-" + std::to_string(state->pid) + "-" + std::to_string(now_ns()) + ".csv");
    state->output.open(path);
    if (!state->output) {throw std::runtime_error("cannot open transport trace " + path.string());}
    state->output << "timestamp_ns,pid,stage,seq,generation,source_stamp_ns\n";
    state->enabled.store(true);
    worker_ = std::thread([state]() {drain(state);});
    state_ = std::move(state);
  }

  bool enabled() const
  {return state_ && state_->enabled.load(std::memory_order_relaxed);}

  void emit(const char * stage, std::uint64_t seq, std::uint64_t generation,
    std::int64_t source_stamp_ns = 0)
  {
    if (!enabled()) {return;}
    const Record record{now_ns(), stage, seq, generation, source_stamp_ns};
    auto & state = *state_;
    std::unique_lock<std::mutex> lock(state.mutex, std::try_to_lock);
    if (!lock.owns_lock() || state.size == capacity_) {
      state.dropped.fetch_add(1, std::memory_order_relaxed);
      return;
    }
    state.records[state.head] = record;
    state.head = (state.head + 1) % capacity_;
    ++state.size;
  }

  // Disk tracing cannot extend motor shutdown indefinitely. A stalled writer
  // retains its shared state when detached; it cannot access this object again.
  bool stop()
  {
    if (!state_) {return true;}
    auto state = std::move(state_);
    state->enabled.store(false);
    state->stopping.store(true);
    std::unique_lock<std::mutex> lock(state->finished_mutex);
    const bool finished = state->finished_condition.wait_for(
      lock, std::chrono::milliseconds(500), [&state]() {return state->finished;});
    lock.unlock();
    if (worker_.joinable()) {
      if (finished) {worker_.join();} else {worker_.detach();}
    }
    return finished;
  }

private:
  static std::int64_t now_ns()
  {
    timespec value{};
    clock_gettime(CLOCK_MONOTONIC, &value);
    return static_cast<std::int64_t>(value.tv_sec) * 1000000000LL + value.tv_nsec;
  }

  struct Record
  {
    std::int64_t timestamp_ns{};
    const char * stage{};
    std::uint64_t seq{};
    std::uint64_t generation{};
    std::int64_t source_stamp_ns{};
  };
  static constexpr std::size_t capacity_ = 8192;
  struct State
  {
    std::array<Record, capacity_> records{};
    std::size_t head{}, tail{}, size{};
    std::mutex mutex;
    std::ofstream output;
    std::atomic_bool enabled{false}, stopping{false};
    std::atomic<std::uint64_t> dropped{0};
    std::int64_t pid{};
    std::mutex finished_mutex;
    std::condition_variable finished_condition;
    bool finished{false};
  };

  static void drain(const std::shared_ptr<State> & state)
  {
    std::array<Record, 256> batch;
    while (true) {
      std::size_t count = 0;
      bool empty = false;
      {
        std::lock_guard<std::mutex> lock(state->mutex);
        while (state->size && count < batch.size()) {
          batch[count++] = state->records[state->tail];
          state->tail = (state->tail + 1) % capacity_;
          --state->size;
        }
        empty = state->size == 0;
      }
      for (std::size_t i = 0; i < count; ++i) {
        const auto & item = batch[i];
        state->output << item.timestamp_ns << ',' << state->pid << ',' << item.stage << ',' <<
          item.seq << ',' << item.generation << ',' << item.source_stamp_ns << '\n';
      }
      if (!state->output || (state->stopping.load() && empty)) {break;}
      if (!count) {
        state->output.flush();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
    state->enabled.store(false);
    if (state->dropped.load()) {
      state->output << now_ns() << ',' << state->pid << ",trace_dropped," <<
        state->dropped.load() << ",0,0\n";
    }
    state->output.close();
    {
      std::lock_guard<std::mutex> lock(state->finished_mutex);
      state->finished = true;
    }
    state->finished_condition.notify_one();
  }

  std::shared_ptr<State> state_;
  std::thread worker_;
};

}  // namespace hex_arm_hardware
