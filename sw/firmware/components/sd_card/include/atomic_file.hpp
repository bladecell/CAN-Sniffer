#pragma once

// Crash-safe file replace (P2.9f, SUP-2). POSIX only, no ESP-IDF includes, so host tests run it on a temp dir.
// The new content goes to "<path>.tmp" and is flushed and fsynced before the old file is touched. FAT cannot rename onto
// an existing file, so the swap is: path -> "<path>.bak", tmp -> path, delete bak. A failed write leaves the old file as it
// was; a power cut anywhere leaves a complete file under one of the three names, and recover() puts it back as `path`.

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <sys/stat.h>
#include <unistd.h>

namespace atomic_file
{
constexpr size_t kPathMax = 263;  // "/sdcard/" plus a 255 character relative path; plus ".tmp"/".bak" and NUL must fit the working buffers below

enum class Result
{
  Ok,
  PathTooLong,
  NoMem,
  WriteFailed,    // old file untouched
  ReplaceFailed,  // old file restored
};

namespace detail
{
inline bool exists(const char* p)
{
  struct stat st;
  return stat(p, &st) == 0;
}

// tmp and bak names in one heap block (the callers run on small task stacks).
struct Names
{
  char* block = nullptr;
  char* tmp   = nullptr;
  char* bak   = nullptr;

  explicit Names(const char* path)
  {
    const size_t len = strlen(path);
    if (len == 0 || len > kPathMax)
    {
      return;
    }
    block = static_cast<char*>(malloc(2 * (kPathMax + 5)));
    if (block == nullptr)
    {
      return;
    }
    tmp = block;
    bak = block + kPathMax + 5;
    // len <= kPathMax, so neither name is truncated
    (void)snprintf(tmp, kPathMax + 5, "%s.tmp", path);
    (void)snprintf(bak, kPathMax + 5, "%s.bak", path);
  }
  ~Names() { free(block); }
  Names(const Names&)            = delete;
  Names& operator=(const Names&) = delete;
};
}  // namespace detail

// Fills the new file: `fill(FILE*)` returns false on any read or write error. The old file is untouched until it succeeds.
template <class Fill>
inline Result writeWith(const char* path, Fill fill)
{
  if (path == nullptr || strlen(path) == 0 || strlen(path) > kPathMax)
  {
    return Result::PathTooLong;
  }
  detail::Names n(path);
  if (n.block == nullptr)
  {
    return Result::NoMem;
  }

  FILE* f = fopen(n.tmp, "w");
  if (f == nullptr)
  {
    return Result::WriteFailed;
  }
  const bool ok = fill(f);
  const bool flushed = fflush(f) == 0;
  const bool synced  = fsync(fileno(f)) == 0;
  const bool closed  = fclose(f) == 0;
  if (!(ok && flushed && synced && closed))
  {
    unlink(n.tmp);
    return Result::WriteFailed;
  }

  unlink(n.bak);  // stale leftover of an interrupted save; fine if absent
  const bool hadOld = rename(path, n.bak) == 0;
  if (!hadOld && errno != ENOENT)
  {
    unlink(n.tmp);
    return Result::ReplaceFailed;
  }
  if (rename(n.tmp, path) != 0)
  {
    if (hadOld)
    {
      (void)rename(n.bak, path);  // best effort: recover() finishes it after a restart
    }
    unlink(n.tmp);
    return Result::ReplaceFailed;
  }
  if (hadOld)
  {
    unlink(n.bak);
  }
  return Result::Ok;
}

inline Result write(const char* path, const char* data, size_t len)
{
  return writeWith(path, [&](FILE* f) { return fwrite(data, 1, len, f) == len; });
}

// Copies `src` over `dest` the same way. `dest` only changes at the final rename, so a read or write error, a full card or a
// `src` that is the same file as `dest` under another name (FAT 8.3 alias) never truncates anything.
inline Result copy(const char* src, const char* dest)
{
  struct stat st;
  if (dest != nullptr && stat(dest, &st) == 0 && !S_ISREG(st.st_mode))
  {
    return Result::WriteFailed;  // a directory (or device) is never replaced
  }
  constexpr size_t kChunk = 2048;
  char*            chunk  = static_cast<char*>(malloc(kChunk));
  if (chunk == nullptr)
  {
    return Result::NoMem;
  }
  const Result r = writeWith(dest, [&](FILE* out) {
    FILE* in = fopen(src, "rb");
    if (in == nullptr)
    {
      return false;
    }
    bool   ok = true;
    size_t got;
    while ((got = fread(chunk, 1, kChunk, in)) > 0)
    {
      if (fwrite(chunk, 1, got, out) != got)
      {
        ok = false;
        break;
      }
    }
    ok = ok && ferror(in) == 0;
    return fclose(in) == 0 && ok;
  });
  free(chunk);
  return r;
}


// Call before reading `path`. If a save was interrupted between the two renames, `path` is missing while a complete
// "<path>.tmp" (only trusted when "<path>.bak" exists, i.e. the write had finished) or the old "<path>.bak" is there.
inline void recover(const char* path)
{
  if (path == nullptr || detail::exists(path))
  {
    return;
  }
  detail::Names n(path);
  if (n.block == nullptr)
  {
    return;
  }
  const bool hasBak = detail::exists(n.bak);
  if (hasBak && detail::exists(n.tmp) && rename(n.tmp, path) == 0)
  {
    unlink(n.bak);
  }
  else if (hasBak)
  {
    (void)rename(n.bak, path);
  }
}
}  // namespace atomic_file
