// Host checks for the #45 diagnostics (AWDLChannelMix.hpp): macOS's plan with one or two 5 GHz AirDrop
// channels, and nearby devices on 44 and on 149. No firmware or radio.
#include <AWDLChannelMix.hpp>
#include <cassert>
#include <cstdio>
using namespace ItlAWDL;

int main() {
    unsigned cases = 0;
    // The live state-29 plan: Wi-Fi 36, AirDrop 149 in slots 2 and 10, channel 6 in slot 8.
    const uint8_t one[16] = {36, 36, 149, 0, 0, 0, 0, 36, 6, 36, 149, 36, 0, 0, 0, 36};
    PlanFiveGHz p = planFiveGHz(one, 149, 36);
    assert(p.home == 149 && p.homeSlots == 2 && !p.second && !p.secondMask), ++cases;
    // State 8 with a 44-only device nearby: home 149 in 0-5 and 10, the common channel 44 in 9 and 11-13.
    const uint8_t two[16] = {149, 149, 149, 149, 149, 149, 36, 36, 6, 44, 149, 44, 44, 44, 36, 36};
    p = planFiveGHz(two, 149, 36);
    assert(p.home == 149 && p.homeSlots == 7 && p.second == 44 && p.secondSlots == 4), ++cases;
    assert(p.secondMask == ((1u << 9) | (1u << 11) | (1u << 12) | (1u << 13))), ++cases;
    // The Mac's Wi-Fi on 44 is never a second AirDrop channel.
    const uint8_t wifi44[16] = {44, 44, 149, 0, 0, 0, 0, 44, 6, 44, 149, 44, 0, 0, 0, 44};
    p = planFiveGHz(wifi44, 149, 44);
    assert(p.home == 149 && !p.second), ++cases;
    // Wi-Fi and AirDrop sharing 149: home stays 149, nothing second.
    const uint8_t shared[16] = {149, 149, 149, 0, 0, 0, 0, 149, 6, 149, 149, 149, 0, 0, 0, 149};
    p = planFiveGHz(shared, 149, 149);
    assert(p.home == 149 && p.homeSlots == 8 && !p.second), ++cases;
    // No secondary master yet: the most named 5 GHz channel other than Wi-Fi's is home.
    p = planFiveGHz(two, 0, 36);
    assert(p.home == 149 && p.second == 44), ++cases;
    p = planFiveGHz(two, 6, 36);                         // 6 isn't a 5 GHz home
    assert(p.home == 149 && p.second == 44), ++cases;
    // Wi-Fi on 2.4 GHz, AirDrop only on 6: no 5 GHz channel at all.
    const uint8_t none[16] = {6, 6, 6, 0, 0, 0, 0, 6, 6, 6, 6, 6, 0, 0, 0, 6};
    p = planFiveGHz(none, 0, 6);
    assert(!p.home && !p.second), ++cases;

    // PlanMix: counts plans with a second channel and keeps the last one.
    {
        PlanMix m;
        assert(m.note(one, 149, 36, 1000) && m.plans == 1 && !m.withSecond);
        assert(!m.note(one, 149, 36, 2000));                                  // unchanged
        assert(m.note(two, 149, 36, 3000) && m.withSecond == 1 && m.last.second == 44 && m.lastUS == 3000);
        assert(m.note(one, 149, 36, 4000) && m.now.second == 0 && m.last.second == 44 && m.lastPlan[9] == 44);
        assert(m.plans == 4 && m.withSecond == 1);
        (void)m.note(wifi44, 149, 44, 5000);
        assert(m.station == 44 && !m.now.second);                                  // shown as "can't be seen"
        cases += 6;
    }

    // PeerChannelMix: devices on 44 and 149 at once, expiry, replacement, no double counting.
    {
        PeerChannelMix x;
        const uint64_t life = 120000000ULL;
        uint8_t a[6] = {2, 0, 0, 0, 0, 1}, b[6] = {2, 0, 0, 0, 0, 2};
        assert(x.note(a, 149, 1, life));
        PeerChannelMix::Counts c = x.counts(1, life);
        assert(c.on149 == 1 && !c.on44 && !x.mixed && !x.mixedSeen);
        assert(!x.note(a, 149, 2, life));                                     // same device, same channel
        assert(x.note(b, 44, 3, life) && x.mixed && x.mixedSeen == 1 && x.lastMixedUS == 3);
        assert(!x.note(b, 44, 4, life) && x.mixedSeen == 1);                  // still mixed, counted once
        c = x.counts(4, life);
        assert(c.on149 == 1 && c.on44 == 1);
        // a goes quiet; after its two minutes only b counts and the mix ends.
        const uint64_t later = 3 + life + 1;
        assert(x.note(b, 44, later, life) && !x.mixed);
        c = x.counts(later, life);
        assert(c.on44 == 1 && !c.on149);
        // a device that moves from 149 to 44 is one device, not two.
        assert(x.note(a, 149, later + 1, life) && x.mixed && x.mixedSeen == 2);
        assert(x.note(a, 44, later + 2, life) && !x.mixed);
        c = x.counts(later + 2, life);
        assert(c.on44 == 2 && !c.on149 && !c.other);
        cases += 11;
        // nine devices in eight entries: the oldest is replaced, nothing overflows.
        PeerChannelMix y;
        for (uint8_t i = 0; i < 9; ++i) {
            uint8_t d[6] = {2, 0, 0, 0, 1, i};
            y.note(d, i == 8 ? 0 : 149, 10 + i, life);
        }
        c = y.counts(20, life);
        assert(c.on149 == 7 && c.other == 1);
        uint8_t first[6] = {2, 0, 0, 0, 1, 0};
        bool found = false;
        for (const auto &e : y.entries)
            found |= e.used && !memcmp(e.addr, first, 6);
        assert(!found);
        cases += 2;
    }
    printf("AWDL channel mix: %u cases passed (host only)\n", cases);
}
