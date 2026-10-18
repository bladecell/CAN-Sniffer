// WEB-7: admission for a request that must not run twice at once. Pure, no FreeRTOS; builds for the linux host tests.
#pragma once
#include <atomic>

class SingleFlight
{
  public:
    // Holds the slot for its lifetime when held() is true; a rejected guard releases nothing.
    class Guard
    {
      public:
        explicit Guard(SingleFlight& f) : f_(f), held_(f.tryAcquire()) {}
        ~Guard()
        {
            if (held_) { f_.release(); }
        }
        Guard(const Guard&)            = delete;
        Guard& operator=(const Guard&) = delete;
        bool   held() const { return held_; }

      private:
        SingleFlight& f_;
        bool          held_;
    };

  private:
    bool tryAcquire()
    {
        bool expected = false;
        return busy_.compare_exchange_strong(expected, true, std::memory_order_acquire);
    }
    void release() { busy_.store(false, std::memory_order_release); }

    std::atomic<bool> busy_{false};
};
