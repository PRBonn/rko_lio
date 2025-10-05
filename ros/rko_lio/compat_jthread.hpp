#pragma once
#if defined(__GNUC__) && (__GNUC__ < 10)
#include <thread>
namespace rko_lio::compat {
class jthread {
  std::thread thread;

public:
  jthread() noexcept = default;

  template <typename Callable, typename... Args>
  explicit jthread(Callable&& func, Args&&... args)
      : thread(std::forward<Callable>(func), std::forward<Args>(args)...) {}

  jthread(jthread const&) = delete;
  jthread& operator=(jthread const&) = delete;

  jthread(jthread&& other) noexcept : thread(std::move(other.thread)) {}
  jthread& operator=(jthread&& other) noexcept {
    if (thread.joinable()) {
      thread.join();
    }
    thread = std::move(other.thread);
    return *this;
  }

  void join() {
    if (thread.joinable()) {
      thread.join();
    }
  }

  ~jthread() {
    if (thread.joinable()) {
      thread.join();
    }
  }
};
} // namespace rko_lio::compat
#endif // GCC < 10
