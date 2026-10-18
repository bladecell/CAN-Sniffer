#pragma once

// Pure SD-card path checks (P2.13a, TEST-1). No ESP-IDF includes, so host tests can feed them hostile strings.
// SDCard::validate_relative_path and SDCard::is_path_under are thin wrappers.

#include <cstddef>
#include <cstring>

namespace sd_path
{

// Accepts "/a/b", "a/b" and one trailing slash. Rejects empty/"."/".." components, repeated slashes, control
// characters, backslash, '?', '#' and '"'. The root ("" or "/") is valid only when allowRoot is set.
inline bool validRelative(const char* path, bool allowRoot)
{
  if (path == nullptr)
  {
    return false;
  }

  const char* p = path;
  if (*p == '/')
  {
    p++;
  }

  if (*p == '\0')
  {
    return allowRoot;
  }

  while (*p != '\0')
  {
    const char* start = p;
    while (*p != '\0' && *p != '/')
    {
      const unsigned char c = static_cast<unsigned char>(*p);
      if (c < 0x20 || c == 0x7f || c == '\\' || c == '?' || c == '#' || c == '"')
      {
        return false;
      }
      p++;
    }

    const size_t len = static_cast<size_t>(p - start);
    if (len == 0 || (len == 1 && start[0] == '.') || (len == 2 && start[0] == '.' && start[1] == '.'))
    {
      return false;
    }

    if (*p == '/')
    {
      p++;
      // A single trailing slash is meaningful to the REST API; empty components elsewhere are not.
      if (*p == '/')
      {
        return false;
      }
    }
  }

  return true;
}

// True when path is a valid path equal to root or inside it (component boundary, not a string prefix).
inline bool isUnder(const char* path, const char* root)
{
  if (path == nullptr || root == nullptr || !validRelative(path, true))
  {
    return false;
  }

  const size_t rootLen = strlen(root);
  if (strncmp(path, root, rootLen) != 0)
  {
    return false;
  }

  return path[rootLen] == '\0' || path[rootLen] == '/';
}

}  // namespace sd_path
