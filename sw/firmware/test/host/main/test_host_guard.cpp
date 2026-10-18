// P2.10b: Host/Origin guard and Content-Type policy, hardware-free.
#include <cstring>

#include "../../../main/include/host_guard.hpp"
#include "unity.h"

namespace
{
namespace hg = host_guard;
const hg::Ips kBoth{"192.168.4.1", "10.0.0.7"};
const hg::Ips kApOnly{"192.168.4.1", "0.0.0.0"};
const hg::Ips kNone{nullptr, ""};

void test_allowed_hosts(void)
{
  TEST_ASSERT_TRUE(hg::hostAllowed("can-sniffer.local", kBoth));
  TEST_ASSERT_TRUE(hg::hostAllowed("can-sniffer.local:80", kBoth));
  TEST_ASSERT_TRUE(hg::hostAllowed("CAN-Sniffer.LOCAL", kBoth));
  TEST_ASSERT_TRUE(hg::hostAllowed("192.168.4.1", kBoth));
  TEST_ASSERT_TRUE(hg::hostAllowed("192.168.4.1:80", kBoth));
  TEST_ASSERT_TRUE(hg::hostAllowed("10.0.0.7", kBoth));
  TEST_ASSERT_TRUE(hg::hostAllowed("can-sniffer.local", kNone));  // mDNS name needs no lease
}

void test_foreign_hosts_are_refused(void)
{
  const char* bad[] = {"evil.example",          "evil.example:80",         "can-sniffer.local.evil.example",
                       "evilcan-sniffer.local", "can-sniffer.local:8080",  "can-sniffer.local:",
                       "can-sniffer.local:80:80", "can-sniffer.local ",    " can-sniffer.local",
                       "can-sniffer.local@evil.example", "192.168.4.10",   "192.168.4.1.evil.example",
                       "192.168.4.",             "10.0.0.8",               "",
                       ":80",                    "can-sniffer.local\r\nX: y", "can-sniffer.local,evil.example",
                       "[::1]",                  "localhost",              "127.0.0.1",
                       "can-sniffer"};
  for (const char* h : bad)
  {
    TEST_ASSERT_FALSE_MESSAGE(hg::hostAllowed(h, kBoth), h);
  }
  TEST_ASSERT_FALSE(hg::hostAllowed(nullptr, kBoth));  // HTTP/1.1 requires Host
}

void test_unset_ip_never_matches(void)
{
  TEST_ASSERT_FALSE(hg::hostAllowed("0.0.0.0", kApOnly));  // STA without a lease reports 0.0.0.0
  TEST_ASSERT_FALSE(hg::hostAllowed("", kNone));
  TEST_ASSERT_FALSE(hg::hostAllowed("10.0.0.7", kApOnly));
}

void test_overlong_host_is_refused(void)
{
  char longHost[200];
  memset(longHost, 'a', sizeof(longHost));
  longHost[sizeof(longHost) - 1] = '\0';
  TEST_ASSERT_FALSE(hg::hostAllowed(longHost, kBoth));
}

void test_origin(void)
{
  TEST_ASSERT_TRUE(hg::originAllowed("http://can-sniffer.local", kBoth));
  TEST_ASSERT_TRUE(hg::originAllowed("http://192.168.4.1", kBoth));
  TEST_ASSERT_TRUE(hg::originAllowed("http://10.0.0.7:80", kBoth));
  const char* bad[] = {"null",
                       "https://can-sniffer.local",
                       "http://evil.example",
                       "http://can-sniffer.local.evil.example",
                       "http://can-sniffer.local/",
                       "http://can-sniffer.local/path",
                       "http://can-sniffer.local:81",
                       "http://localhost:5173",
                       "can-sniffer.local",
                       "http://",
                       "",
                       "file://",
                       "HTTP://can-sniffer.local",
                       "http://user@can-sniffer.local"};
  for (const char* o : bad)
  {
    TEST_ASSERT_FALSE_MESSAGE(hg::originAllowed(o, kBoth), o);
  }
  TEST_ASSERT_FALSE(hg::originAllowed(nullptr, kBoth));
}

void test_check_order_and_absent_origin(void)
{
  TEST_ASSERT_EQUAL(hg::Verdict::Ok, hg::check("can-sniffer.local", nullptr, kBoth));
  TEST_ASSERT_EQUAL(hg::Verdict::Ok, hg::check("192.168.4.1", "http://192.168.4.1", kBoth));
  TEST_ASSERT_EQUAL(hg::Verdict::BadHost, hg::check("evil.example", nullptr, kBoth));
  TEST_ASSERT_EQUAL(hg::Verdict::BadHost, hg::check(nullptr, "http://can-sniffer.local", kBoth));
  // DNS rebinding: the page's own origin and Host are the attacker's name
  TEST_ASSERT_EQUAL(hg::Verdict::BadHost, hg::check("rebind.evil.example", "http://rebind.evil.example", kBoth));
  // cross-site forgery: right Host, foreign Origin
  TEST_ASSERT_EQUAL(hg::Verdict::BadOrigin, hg::check("192.168.4.1", "http://evil.example", kBoth));
  TEST_ASSERT_EQUAL(hg::Verdict::BadOrigin, hg::check("192.168.4.1", "null", kBoth));
}

void test_json_content_type(void)
{
  const char* ok[] = {"application/json", "Application/JSON", "application/json; charset=utf-8",
                      "application/json;charset=UTF-8", "  application/json ; charset=utf-8  ", "application/json\t"};
  for (const char* c : ok)
  {
    TEST_ASSERT_TRUE_MESSAGE(hg::contentTypeOk(c, hg::Body::Json), c);
  }
  const char* bad[] = {"text/plain",       "text/plain;charset=UTF-8", "application/x-www-form-urlencoded",
                       "multipart/form-data; boundary=x", "application/octet-stream", "application/jsonx",
                       "application/json; charset=latin1", "application/json; boundary=x", "application/json;",
                       "application/json, text/plain", "application/", "", ";", "text/plain; application/json"};
  for (const char* c : bad)
  {
    TEST_ASSERT_FALSE_MESSAGE(hg::contentTypeOk(c, hg::Body::Json), c);
  }
  TEST_ASSERT_FALSE(hg::contentTypeOk(nullptr, hg::Body::Json));
}

void test_octet_content_type(void)
{
  TEST_ASSERT_TRUE(hg::contentTypeOk("application/octet-stream", hg::Body::Octet));
  TEST_ASSERT_TRUE(hg::contentTypeOk("Application/Octet-Stream", hg::Body::Octet));
  TEST_ASSERT_FALSE(hg::contentTypeOk("application/octet-stream; foo=bar", hg::Body::Octet));
  TEST_ASSERT_FALSE(hg::contentTypeOk("application/json", hg::Body::Octet));
  TEST_ASSERT_FALSE(hg::contentTypeOk("text/plain", hg::Body::Octet));
  TEST_ASSERT_FALSE(hg::contentTypeOk(nullptr, hg::Body::Octet));
}
}  // namespace

extern "C" void run_host_guard_tests(void)
{
  RUN_TEST(test_allowed_hosts);
  RUN_TEST(test_foreign_hosts_are_refused);
  RUN_TEST(test_unset_ip_never_matches);
  RUN_TEST(test_overlong_host_is_refused);
  RUN_TEST(test_origin);
  RUN_TEST(test_check_order_and_absent_origin);
  RUN_TEST(test_json_content_type);
  RUN_TEST(test_octet_content_type);
}
