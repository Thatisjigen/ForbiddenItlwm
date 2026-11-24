// Which scan channel entries select the SSID (WCL.hpp): iwx's UMAC scan selects it on every entry of an
// active scan, passive and radar channels included (#69); iwm/iwn keep probing only active, non-radar channels.
#include <WCL.hpp>
#include <cassert>
#include <cstdio>
using AirportWCL::probeChannel;
using AirportWCL::umacScanSelectsSsid;

int main() {
    assert(umacScanSelectsSsid(true));
    assert(!umacScanSelectsSsid(false));
    // The iwm/iwn rule is unchanged.
    assert(probeChannel(true, false, false));
    assert(!probeChannel(true, true, false));
    assert(!probeChannel(true, false, true));
    assert(!probeChannel(false, false, false));
    std::puts("scan SSID selection (iwx every active entry; iwm/iwn active non-radar only): passed");
    return 0;
}
