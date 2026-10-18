// P2.11 NB1 (B20): the route table has no catch-all patterns, so typos such as /api/v1/dtcs reach no handler.
#include <cstring>

#include "../../../main/include/api_routes.hpp"
#include "unity.h"

namespace
{
struct Route
{
  const char* uri;
  const char* method;
};

#define ROUTE_ENTRY(uri, method, handler) {uri, #method},
const Route kRoutes[] = {API_ROUTES(ROUTE_ENTRY)};
#undef ROUTE_ENTRY
constexpr size_t kCount = sizeof(kRoutes) / sizeof(kRoutes[0]);

// Wildcards are allowed only where the handler validates the whole remainder as <pid> or <path>.
const char* const kPathParamPrefixes[] = {"/api/v1/pid_def/*", "/api/v1/pid_data/*", "/api/v1/sd_card/tree/*",
                                          "/api/v1/sd_card/file/*"};

bool isPathParam(const char* uri)
{
  for (const char* p : kPathParamPrefixes)
  {
    if (strcmp(uri, p) == 0)
    {
      return true;
    }
  }
  return false;
}

void test_wildcards_only_on_path_parameter_routes(void)
{
  for (size_t i = 0; i < kCount; i++)
  {
    if (strchr(kRoutes[i].uri, '*') != nullptr || strchr(kRoutes[i].uri, '?') != nullptr)
    {
      TEST_ASSERT_TRUE_MESSAGE(isPathParam(kRoutes[i].uri), kRoutes[i].uri);
    }
  }
}

void test_no_duplicate_uri_method_pairs(void)
{
  for (size_t i = 0; i < kCount; i++)
  {
    for (size_t j = i + 1; j < kCount; j++)
    {
      const bool same = strcmp(kRoutes[i].uri, kRoutes[j].uri) == 0 && strcmp(kRoutes[i].method, kRoutes[j].method) == 0;
      TEST_ASSERT_FALSE_MESSAGE(same, kRoutes[i].uri);
    }
  }
}

void test_commands_and_dtc_are_exact(void)
{
  bool sawDtc = false, sawAutoPoll = false, sawReqDtc = false;
  for (size_t i = 0; i < kCount; i++)
  {
    sawDtc      = sawDtc || strcmp(kRoutes[i].uri, "/api/v1/dtc") == 0;
    sawReqDtc   = sawReqDtc || strcmp(kRoutes[i].uri, "/api/v1/req/dtc") == 0;
    sawAutoPoll = sawAutoPoll || strcmp(kRoutes[i].uri, "/api/v1/req/auto_poll") == 0;
  }
  TEST_ASSERT_TRUE(sawDtc && sawReqDtc && sawAutoPoll);
}

void test_route_count_matches_docs(void)
{
  TEST_ASSERT_EQUAL_UINT32(31, kCount);  // docs/api.md: GET / plus 30 under /api/v1
}
}  // namespace

extern "C" void run_api_routes_tests(void)
{
  RUN_TEST(test_wildcards_only_on_path_parameter_routes);
  RUN_TEST(test_no_duplicate_uri_method_pairs);
  RUN_TEST(test_commands_and_dtc_are_exact);
  RUN_TEST(test_route_count_matches_docs);
}
