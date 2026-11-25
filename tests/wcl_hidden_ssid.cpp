// Host checks for the hidden-network helpers (WCL.hpp, PR #107): when a beacon hides the network's name, when two
// names are the same, and the all-zero BSSID test that keeps a BSSID the join named.
#include <WCL.hpp>
#include <cassert>
#include <cstdio>
using namespace AirportWCL;

int main() {
    unsigned cases = 0;
    const uint8_t named[] = {0, 4, 'h', 'o', 'm', 'e', 1, 1, 0x82};
    const uint8_t empty[] = {0, 0, 1, 1, 0x82};
    const uint8_t zeros[] = {0, 4, 0, 0, 0, 0, 1, 1, 0x82};
    const uint8_t none[] = {1, 1, 0x82, 3, 1, 36};
    const uint8_t later[] = {1, 1, 0x82, 0, 2, 'h', 'i'};   // SSID element not first, still found
    const uint8_t bad[] = {0, 9, 'x'};                       // malformed: never treated as hidden
    assert(!hidesSsid(named, sizeof(named))), ++cases;
    assert(hidesSsid(empty, sizeof(empty))), ++cases;
    assert(hidesSsid(zeros, sizeof(zeros))), ++cases;
    assert(hidesSsid(none, sizeof(none))), ++cases;
    assert(!hidesSsid(later, sizeof(later))), ++cases;
    assert(!hidesSsid(bad, sizeof(bad))), ++cases;
    assert(!hidesSsid(nullptr, 0)), ++cases;

    const uint8_t a[] = "home", b[] = "home", c[] = "hone";
    assert(sameSsid(a, 4, b, 4)), ++cases;
    assert(!sameSsid(a, 4, c, 4)), ++cases;
    assert(!sameSsid(a, 4, b, 3)), ++cases;
    assert(!sameSsid(a, 0, b, 0)), ++cases;   // empty names never match
    uint8_t long33[33] = {};
    assert(!sameSsid(long33, 33, long33, 33)), ++cases;

    const uint8_t zero[6] = {}, one[6] = {0, 0, 0, 0, 0, 1};
    assert(zeroAddress(zero) && !zeroAddress(one)), ++cases;
    printf("wcl_hidden_ssid: %u cases passed\n", cases);
    return 0;
}
