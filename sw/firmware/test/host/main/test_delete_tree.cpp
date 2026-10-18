// P2.9b: SD-1. The recursive delete must cap its depth and keep per-level stack use small.
#include <sys/stat.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "../../../components/sd_card/include/delete_tree.hpp"
#include "unity.h"

namespace
{
std::string makeRoot()
{
  char tmpl[] = "/tmp/deltreeXXXXXX";
  TEST_ASSERT_NOT_NULL(mkdtemp(tmpl));
  return tmpl;
}

void touch(const std::string& p)
{
  FILE* f = fopen(p.c_str(), "w");
  TEST_ASSERT_NOT_NULL(f);
  fclose(f);
}

bool exists(const std::string& p)
{
  struct stat st;
  return stat(p.c_str(), &st) == 0;
}

// Creates root/d0/d1/.../d(levels-1), each with one file; returns the path of d0.
std::string nest(const std::string& root, int levels)
{
  std::string p = root;
  std::string first;
  for (int i = 0; i < levels; i++)
  {
    p += "/d" + std::to_string(i);
    TEST_ASSERT_EQUAL_INT(0, mkdir(p.c_str(), 0700));
    touch(p + "/f.txt");
    if (i == 0)
    {
      first = p;
    }
  }
  return first;
}

sd_tree::Result run(const std::string& dir, size_t cap = 1024)
{
  std::string buf(cap, '\0');
  memcpy(buf.data(), dir.c_str(), dir.size() + 1);
  return sd_tree::deleteTree(buf.data(), cap, dir.size());
}

void test_deletes_nested_tree(void)
{
  const std::string root = makeRoot();
  const std::string top  = nest(root, 5);  // /logs/2026/10/ style, deeper than one level
  TEST_ASSERT_EQUAL(sd_tree::Result::Ok, run(top));
  TEST_ASSERT_FALSE(exists(top));
  rmdir(root.c_str());
}

void test_depth_cap_stops_instead_of_recursing(void)
{
  const std::string root = makeRoot();
  const std::string top  = nest(root, sd_tree::kMaxDepth + 3);
  TEST_ASSERT_EQUAL(sd_tree::Result::TooDeep, run(top));
  TEST_ASSERT_TRUE(exists(top));  // the over-deep tree is not removed
  // exactly kMaxDepth levels below the start are allowed
  const std::string root2 = makeRoot();
  const std::string top2  = nest(root2, sd_tree::kMaxDepth + 1);
  TEST_ASSERT_EQUAL(sd_tree::Result::Ok, run(top2));
  TEST_ASSERT_FALSE(exists(top2));
  // cleanup
  std::string cmd = "rm -rf " + root + " " + root2;
  TEST_ASSERT_EQUAL_INT(0, system(cmd.c_str()));
}

void test_path_too_long_is_an_error_not_an_overflow(void)
{
  const std::string root = makeRoot();
  const std::string top  = nest(root, 3);
  TEST_ASSERT_EQUAL(sd_tree::Result::PathTooLong, run(top, top.size() + 4));
  std::string cmd = "rm -rf " + root;
  TEST_ASSERT_EQUAL_INT(0, system(cmd.c_str()));
}

void test_missing_directory(void)
{
  TEST_ASSERT_EQUAL(sd_tree::Result::IoError, run("/tmp/deltree-does-not-exist"));
}

int64_t g_fakeNow = 0;
int64_t fakeNow()
{
  return g_fakeNow += 10;  // every entry costs 10 us
}

void test_deadline_ends_a_long_delete(void)
{
  const std::string root = makeRoot();
  const std::string top  = nest(root, 3);
  for (int i = 0; i < 20; i++)
  {
    touch(top + "/extra" + std::to_string(i));
  }
  std::string buf(1024, '\0');
  memcpy(buf.data(), top.c_str(), top.size() + 1);
  g_fakeNow                  = 0;
  const sd_tree::Deadline dl = {fakeNow, 50};  // allows a handful of entries
  TEST_ASSERT_EQUAL(sd_tree::Result::TimedOut, sd_tree::deleteTree(buf.data(), buf.size(), top.size(), 0, &dl));
  TEST_ASSERT_TRUE(exists(top));  // the directory itself is only removed after everything in it
  TEST_ASSERT_EQUAL_STRING(top.c_str(), buf.c_str());
  g_fakeNow = 0;
  const sd_tree::Deadline roomy = {fakeNow, 1000000};
  TEST_ASSERT_EQUAL(sd_tree::Result::Ok, sd_tree::deleteTree(buf.data(), buf.size(), top.size(), 0, &roomy));
  TEST_ASSERT_FALSE(exists(top));
  rmdir(root.c_str());
}

void test_path_buffer_restored(void)
{
  const std::string root = makeRoot();
  const std::string top  = nest(root, 2);
  std::string       buf(1024, '\0');
  memcpy(buf.data(), top.c_str(), top.size() + 1);
  TEST_ASSERT_EQUAL(sd_tree::Result::Ok, sd_tree::deleteTree(buf.data(), buf.size(), top.size()));
  TEST_ASSERT_EQUAL_STRING(top.c_str(), buf.c_str());
  rmdir(root.c_str());
}
}  // namespace

extern "C" void run_delete_tree_tests(void)
{
  RUN_TEST(test_deletes_nested_tree);
  RUN_TEST(test_depth_cap_stops_instead_of_recursing);
  RUN_TEST(test_path_too_long_is_an_error_not_an_overflow);
  RUN_TEST(test_missing_directory);
  RUN_TEST(test_deadline_ends_a_long_delete);
  RUN_TEST(test_path_buffer_restored);
}
