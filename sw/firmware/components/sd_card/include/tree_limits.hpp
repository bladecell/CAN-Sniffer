#pragma once

// Bounds for GET /sd_card/tree (P2.11 NB7, M9). No ESP-IDF includes, so host tests run it directly.
// The reply is a cJSON tree built on the heap, and the recorder fills the card with one CSV per trip, so both the depth
// and the number of entries are capped. Truncation is reported to the client, never an error.

#include <cstddef>

namespace sd_tree
{
constexpr int    kDefaultListDepth = 5;    // levels below the requested directory that are expanded (the old fixed depth)
constexpr int    kMaxListDepth     = 5;
constexpr size_t kDefaultLimit     = 256;  // entries (files and directories) in the whole reply
constexpr size_t kMaxLimit         = 512;

struct Limits
{
  int    depth  = kDefaultListDepth;
  size_t limit  = kDefaultLimit;
  size_t offset = 0;  // entries skipped at the top level of the requested directory, in readdir order

  constexpr bool valid() const
  {
    return depth >= 0 && depth <= kMaxListDepth && limit >= 1 && limit <= kMaxLimit;
  }
};

// Walk bookkeeping. Ask next() for every directory entry before looking at it.
class Budget
{
public:
  enum class Verdict
  {
    Take,  // add this entry
    Skip,  // inside the offset window, ignore it
    Stop,  // the limit is reached and more entries exist: stop reading this directory and every parent
  };

  explicit Budget(const Limits& limits) : limits_(limits) {}

  // `level` is 0 for the entries of the requested directory, 1 for the entries of its subdirectories, and so on.
  Verdict next(int level)
  {
    if (stopped_)
    {
      return Verdict::Stop;
    }
    if (level == 0 && skipped_ < limits_.offset)
    {
      skipped_++;
      return Verdict::Skip;
    }
    if (taken_ >= limits_.limit)
    {
      stopped_   = true;
      truncated_ = true;
      return Verdict::Stop;
    }
    taken_++;
    return Verdict::Take;
  }

  // Subdirectories of a directory at `level` are listed only while level < depth.
  bool mayExpand(int level) const { return level < limits_.depth; }

  // A directory was left unlisted (too deep).
  void markTruncated() { truncated_ = true; }

  bool   truncated() const { return truncated_; }
  size_t taken() const { return taken_; }

private:
  Limits limits_;
  size_t skipped_   = 0;
  size_t taken_     = 0;
  bool   stopped_   = false;
  bool   truncated_ = false;
};
}  // namespace sd_tree
