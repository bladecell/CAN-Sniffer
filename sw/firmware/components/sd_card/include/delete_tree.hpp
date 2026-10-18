#pragma once

// Recursive directory delete (P2.9b, SD-1). POSIX only, no ESP-IDF includes, so host tests can run it on a temp dir.
// One caller-provided path buffer is shared by every level and each level keeps only a DIR*, so stack use does not
// grow with the depth of the tree. Depth is capped: a hostile card can nest directories until FAT runs out.

#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstddef>
#include <cstring>

namespace sd_tree
{
constexpr int kMaxDepth = 8;  // directories below the one being deleted

enum class Result
{
  Ok,
  NotFound,
  TooDeep,
  PathTooLong,
  IoError,
};

// Deletes the directory named by the NUL-terminated `path` (length `len`, buffer capacity `cap`) and everything in it.
// `path` is restored before each return. On TooDeep, entries visited before the over-deep directory are already gone.
inline Result deleteTree(char* path, size_t cap, size_t len, int depth = 0)
{
  if (depth > kMaxDepth)
  {
    return Result::TooDeep;
  }
  DIR* dir = opendir(path);
  if (dir == nullptr)
  {
    return Result::IoError;
  }

  Result result = Result::Ok;
  while (result == Result::Ok)
  {
    const dirent* entry = readdir(dir);
    if (entry == nullptr)
    {
      break;
    }
    const char* name = entry->d_name;
    if (strcmp(name, ".") == 0 || strcmp(name, "..") == 0)
    {
      continue;
    }
    const size_t nameLen = strlen(name);
    if (len + 1 + nameLen >= cap)
    {
      result = Result::PathTooLong;
      break;
    }
    path[len] = '/';
    memcpy(path + len + 1, name, nameLen + 1);
    const size_t childLen = len + 1 + nameLen;

    struct stat st;
    if (stat(path, &st) != 0)
    {
      result = Result::IoError;
    }
    else
    {
      result = S_ISDIR(st.st_mode) ? deleteTree(path, cap, childLen, depth + 1)
                                   : (unlink(path) == 0 ? Result::Ok : Result::IoError);
    }
    path[len] = '\0';
  }
  closedir(dir);

  if (result == Result::Ok && rmdir(path) != 0)
  {
    result = Result::IoError;
  }
  return result;
}
}  // namespace sd_tree
