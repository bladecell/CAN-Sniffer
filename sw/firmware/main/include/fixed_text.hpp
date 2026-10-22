#pragma once

#include <cstddef>
#include <cstring>

// Bounded, heap-free text for error reasons. C++ exceptions are off, so a std::string that outgrows SSO aborts on OOM.
// Input beyond N-1 bytes is truncated; the buffer is always terminated.
template <size_t N = 96>
class FixedText
{
    static_assert(N >= 2, "FixedText needs room for one character and the terminator");
    char   buf_[N] = {};
    size_t len_    = 0;

public:
    FixedText& set(const char* s)
    {
        len_    = 0;
        buf_[0] = '\0';
        return append(s);
    }
    FixedText& append(const char* s)
    {
        if (s == nullptr) { return *this; }
        const size_t n = strnlen(s, N - 1 - len_);
        memcpy(buf_ + len_, s, n);
        len_ += n;
        buf_[len_] = '\0';
        return *this;
    }
    bool        empty() const { return len_ == 0; }
    const char* c_str() const { return buf_; }
};
