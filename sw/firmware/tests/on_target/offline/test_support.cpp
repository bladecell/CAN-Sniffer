#include <cstdio>
#include <cstdlib>
#include <initializer_list>
#include "support.hpp"
#define CHECK(expr) do { if (!(expr)) { std::fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #expr); std::exit(1); } } while (false)
int main() {
  p28::TaskInjection injection;
  injection.arm(p28::Action::Fail);
  CHECK(injection.take("can_tx", true) == p28::Action::Pass);
  CHECK(injection.take("DataSimTask", false) == p28::Action::Pass);
  CHECK(injection.take("DataSimTask", true) == p28::Action::Fail);
  CHECK(injection.take("DataSimTask", true) == p28::Action::Pass);
  CHECK(injection.hits() == 1);
  injection.arm(p28::Action::Hold);
  CHECK(injection.take("DataSimTask", true) == p28::Action::Hold);
  injection.arm(p28::Action::Pass);
  CHECK(injection.take("DataSimTask", true) == p28::Action::Pass);
  std::puts("task injection policy: PASS");
  p28::SavedKey key{true, 5};
  CHECK(p28::parseKey("absent", key)); CHECK(!key.present && key.value == 0);
  CHECK(p28::parseKey("u32:4294967295", key)); CHECK(key.present && key.value == UINT32_MAX);
  CHECK(p28::parseKey("u32:0", key)); CHECK(key.present && key.value == 0);
  for (const char* bad : {"", "u32:", "u32:-1", "u32:+1", "u32:4294967296", "u32:1x", "blob:1", "absent "}) {
    CHECK(!p28::parseKey(bad, key));
  }
  bool restored = false;
  auto result = p28::withRestoration([] { return false; }, [&] { restored = true; return true; });
  CHECK(!result.scenario && result.restored && restored);
  result = p28::withRestoration([] { return true; }, [] { return false; });
  CHECK(result.scenario && !result.restored);
  std::puts("backup parser and unconditional restoration: PASS");
}
