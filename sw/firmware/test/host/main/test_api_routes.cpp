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
  bool        gated;
};

#define ROUTE_ENTRY(uri, method, handler, gated) {uri, #method, (gated) != 0},
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
    sawAutoPoll = sawAutoPoll || (strcmp(kRoutes[i].uri, "/api/v1/obd2") == 0 && strcmp(kRoutes[i].method, "PATCH") == 0);
  }
  TEST_ASSERT_TRUE(sawDtc && sawReqDtc && sawAutoPoll);
}

// P2.12: renamed routes are gone, no aliases.
void test_renamed_routes_are_removed(void)
{
  const char* const removed[][2] = {{"POST", "/api/v1/req/auto_poll"}, {"POST", "/api/v1/settings"}};
  for (const auto& r : removed)
  {
    for (size_t i = 0; i < kCount; i++)
    {
      TEST_ASSERT_FALSE_MESSAGE(strcmp(kRoutes[i].uri, r[1]) == 0 && strcmp(kRoutes[i].method, r[0]) == 0, r[1]);
    }
  }
}

void test_route_count_matches_docs(void)
{
  TEST_ASSERT_EQUAL_UINT32(38, kCount);  // docs/api.md: GET / plus 37 under /api/v1
}
// The access-control decision (docs/decisions.md 2026-10-18), spelled out. A route is gated if and only if it is listed;
// adding a route forces a decision here.
struct GatedRoute
{
  const char* method;
  const char* uri;
};
const GatedRoute kGated[] = {
    {"GET", "/api/v1/wifi/scan"},         {"POST", "/api/v1/wifi/scan"},
    {"GET", "/api/v1/settings"},          {"PATCH", "/api/v1/settings"},
    {"GET", "/api/v1/sd_card/tree"},      {"GET", "/api/v1/sd_card/tree/*"},
    {"GET", "/api/v1/sd_card/file/*"},    {"POST", "/api/v1/sd_card/file/*"},
    {"DELETE", "/api/v1/sd_card/file/*"}, {"POST", "/api/v1/sd_card/format"},
    {"POST", "/api/v1/system/copy_file"}, {"POST", "/api/v1/system/reboot"},
    {"POST", "/api/v1/pid_def"},          {"PUT", "/api/v1/pid_def"},
    {"DELETE", "/api/v1/pid_def/*"},      {"POST", "/api/v1/pid_def/save"},
    {"POST", "/api/v1/pid_def/load"},     {"POST", "/api/v1/req/clear_dtc"},
    {"POST", "/api/v1/auth/logout"},      {"PUT", "/api/v1/auth/pin"},
};
constexpr size_t kGatedCount = sizeof(kGated) / sizeof(kGated[0]);

bool listedAsGated(const Route& r)
{
  for (const GatedRoute& g : kGated)
  {
    if (strcmp(g.uri, r.uri) == 0 && strcmp(g.method, r.method) == 0)
    {
      return true;
    }
  }
  return false;
}

void test_gated_set_is_exactly_the_decision(void)
{
  size_t gated = 0;
  for (size_t i = 0; i < kCount; i++)
  {
    TEST_ASSERT_EQUAL_MESSAGE(listedAsGated(kRoutes[i]), kRoutes[i].gated, kRoutes[i].uri);
    gated += kRoutes[i].gated ? 1 : 0;
  }
  TEST_ASSERT_EQUAL_UINT32(kGatedCount, gated);  // every listed route exists in the table
}

void test_open_routes_stay_open(void)
{
  // dashboard reads, /req/*, VIN, pid_def/pid_data GET, sd_card/info, auth entry points
  const char* open[][2] = {{"GET", "/"},
                           {"GET", "/api/v1/system"},
                           {"GET", "/api/v1/can_bus"},
                           {"GET", "/api/v1/obd2"},
                           {"GET", "/api/v1/vin"},
                           {"GET", "/api/v1/dtc"},
                           {"GET", "/api/v1/pid_def"},
                           {"GET", "/api/v1/pid_def/*"},
                           {"GET", "/api/v1/pid_data"},
                           {"GET", "/api/v1/pid_data/*"},
                           {"GET", "/api/v1/sd_card/info"},
                           {"POST", "/api/v1/req/vin"},
                           {"POST", "/api/v1/req/dtc"},
                           {"PATCH", "/api/v1/obd2"},
                           {"POST", "/api/v1/req/static_pid"},
                           {"GET", "/api/v1/auth"},
                           {"POST", "/api/v1/auth/claim"},
                           {"POST", "/api/v1/auth/login"}};
  for (const auto& o : open)
  {
    bool found = false;
    for (size_t i = 0; i < kCount; i++)
    {
      if (strcmp(kRoutes[i].uri, o[1]) == 0 && strcmp(kRoutes[i].method, o[0]) == 0)
      {
        found = true;
        TEST_ASSERT_FALSE_MESSAGE(kRoutes[i].gated, o[1]);
      }
    }
    TEST_ASSERT_TRUE_MESSAGE(found, o[1]);
  }
}
}  // namespace

extern "C" void run_api_routes_tests(void)
{
  RUN_TEST(test_gated_set_is_exactly_the_decision);
  RUN_TEST(test_open_routes_stay_open);
  RUN_TEST(test_wildcards_only_on_path_parameter_routes);
  RUN_TEST(test_renamed_routes_are_removed);
  RUN_TEST(test_no_duplicate_uri_method_pairs);
  RUN_TEST(test_commands_and_dtc_are_exact);
  RUN_TEST(test_route_count_matches_docs);
}
