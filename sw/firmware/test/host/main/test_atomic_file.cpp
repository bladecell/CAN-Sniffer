// P2.9f: a failed or interrupted PID file save must keep the previous file (it used to fopen(path, "w") = truncate).
#include <sys/stat.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "../../../components/sd_card/include/atomic_file.hpp"
#include "unity.h"

namespace
{
namespace af = atomic_file;

std::string makeDir()
{
  char tmpl[] = "/tmp/atomicXXXXXX";
  TEST_ASSERT_NOT_NULL(mkdtemp(tmpl));
  return tmpl;
}

void put(const std::string& p, const char* text)
{
  FILE* f = fopen(p.c_str(), "w");
  TEST_ASSERT_NOT_NULL(f);
  fputs(text, f);
  fclose(f);
}

std::string get(const std::string& p)
{
  FILE* f = fopen(p.c_str(), "r");
  if (f == nullptr) return "<missing>";
  char   buf[256] = {};
  size_t n        = fread(buf, 1, sizeof(buf) - 1, f);
  fclose(f);
  return std::string(buf, n);
}

bool exists(const std::string& p)
{
  struct stat st;
  return stat(p.c_str(), &st) == 0;
}

void cleanup(const std::string& d)
{
  std::string cmd = "rm -rf " + d;
  TEST_ASSERT_EQUAL_INT(0, system(cmd.c_str()));
}

void test_creates_and_replaces(void)
{
  const std::string d = makeDir(), p = d + "/pid.json";
  TEST_ASSERT_EQUAL(af::Result::Ok, af::write(p.c_str(), "one", 3));
  TEST_ASSERT_EQUAL_STRING("one", get(p).c_str());
  TEST_ASSERT_EQUAL(af::Result::Ok, af::write(p.c_str(), "second", 6));
  TEST_ASSERT_EQUAL_STRING("second", get(p).c_str());
  TEST_ASSERT_FALSE(exists(p + ".tmp"));
  TEST_ASSERT_FALSE(exists(p + ".bak"));
  cleanup(d);
}

void test_failed_write_keeps_previous_file(void)
{
  const std::string d = makeDir(), p = d + "/pid.json";
  put(p, "precious");
  TEST_ASSERT_EQUAL_INT(0, mkdir((p + ".tmp").c_str(), 0700));  // fopen(tmp, "w") now fails
  TEST_ASSERT_EQUAL(af::Result::WriteFailed, af::write(p.c_str(), "new", 3));
  TEST_ASSERT_EQUAL_STRING("precious", get(p).c_str());
  cleanup(d);
}

void test_path_limits(void)
{
  TEST_ASSERT_EQUAL(af::Result::PathTooLong, af::write(nullptr, "x", 1));
  TEST_ASSERT_EQUAL(af::Result::PathTooLong, af::write("", "x", 1));
  const std::string longPath = "/tmp/" + std::string(af::kPathMax, 'a');
  TEST_ASSERT_EQUAL(af::Result::PathTooLong, af::write(longPath.c_str(), "x", 1));
}

void test_recover_finished_write_interrupted_before_final_rename(void)
{
  const std::string d = makeDir(), p = d + "/pid.json";
  put(p + ".bak", "old");
  put(p + ".tmp", "new");  // path itself is gone: power cut between the two renames
  af::recover(p.c_str());
  TEST_ASSERT_EQUAL_STRING("new", get(p).c_str());
  TEST_ASSERT_FALSE(exists(p + ".bak"));
  cleanup(d);
}

void test_recover_old_file_when_only_bak_exists(void)
{
  const std::string d = makeDir(), p = d + "/pid.json";
  put(p + ".bak", "old");
  af::recover(p.c_str());
  TEST_ASSERT_EQUAL_STRING("old", get(p).c_str());
  cleanup(d);
}

void test_recover_ignores_unfinished_tmp(void)
{
  const std::string d = makeDir(), p = d + "/pid.json";
  put(p + ".tmp", "par");  // first ever save, cut during the write: no bak, so tmp may be partial
  af::recover(p.c_str());
  TEST_ASSERT_FALSE(exists(p));
  cleanup(d);
}

void test_recover_leaves_existing_file_alone(void)
{
  const std::string d = makeDir(), p = d + "/pid.json";
  put(p, "live");
  put(p + ".bak", "stale");
  af::recover(p.c_str());
  TEST_ASSERT_EQUAL_STRING("live", get(p).c_str());
  cleanup(d);
}
}  // namespace

extern "C" void run_atomic_file_tests(void)
{
  RUN_TEST(test_creates_and_replaces);
  RUN_TEST(test_failed_write_keeps_previous_file);
  RUN_TEST(test_path_limits);
  RUN_TEST(test_recover_finished_write_interrupted_before_final_rename);
  RUN_TEST(test_recover_old_file_when_only_bak_exists);
  RUN_TEST(test_recover_ignores_unfinished_tmp);
  RUN_TEST(test_recover_leaves_existing_file_alone);
}
