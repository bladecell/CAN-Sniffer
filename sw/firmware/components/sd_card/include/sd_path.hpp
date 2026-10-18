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

// "<mount>/<rel>" for a file operation: rel is a valid relative path ("/a/b" or "a/b"), not the root and not a directory
// (no trailing slash). out is cleared on failure and never holds a truncated path.
inline bool absoluteFile(const char* mount, const char* rel, char* out, size_t cap)
{
  if (out == nullptr || cap == 0)
  {
    return false;
  }
  out[0] = '\0';
  if (mount == nullptr || !validRelative(rel, false))
  {
    return false;
  }
  const size_t relLen = strlen(rel);
  if (rel[relLen - 1] == '/')
  {
    return false;
  }
  const char*  tail     = rel[0] == '/' ? rel + 1 : rel;
  const size_t mountLen = strlen(mount);
  const size_t tailLen  = strlen(tail);
  if (mountLen + 1 + tailLen + 1 > cap)
  {
    return false;
  }
  memcpy(out, mount, mountLen);
  out[mountLen] = '/';
  memcpy(out + mountLen + 1, tail, tailLen + 1);
  return true;
}

}  // namespace sd_path
