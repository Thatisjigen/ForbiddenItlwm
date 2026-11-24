// Diagnostics for Apple devices on different 5 GHz AirDrop channels (#45). Nothing here changes the radio.
//
// macOS's plan (IO80211 setChannelSequenceByState) fills its 16 slots by slot type: type 3 slots get macOS's 5 GHz
// home channel (149 here), type 4 slots the AWDL common channel, which IO80211 turns to 44 when a nearby device
// reports 44 as its highest allowed channel. Several of macOS's states mix both types, so the plan can name two
// 5 GHz channels at once, and a real Mac then switches between them. PlanMix counts how often macOS's plan does
// that; PeerChannelMix counts the nearby devices on 44 and on 149, from their own channel sequences.
#pragma once
#include <stdint.h>
#include <string.h>

namespace ItlAWDL {

inline bool isFiveGHz(unsigned ch) { return ch >= 36 && ch <= 177; }

// The 5 GHz channels macOS's plan names besides the Mac's own Wi-Fi channel (`station`): its home channel (`home`
// when that is a 5 GHz channel, else the most named one) and a second one, the most named of the rest.
struct PlanFiveGHz {
    uint8_t home = 0, homeSlots = 0;
    uint8_t second = 0, secondSlots = 0;
    uint16_t secondMask = 0;     // bit n: slot n is on the second channel
};

inline PlanFiveGHz planFiveGHz(const uint8_t (&plan)[16], unsigned home, unsigned station) {
    PlanFiveGHz r;
    const auto count = [&](unsigned ch) {
        unsigned n = 0;
        for (unsigned i = 0; i < 16; ++i)
            n += plan[i] == ch;
        return n;
    };
    // the most named 5 GHz channel other than `skipA` and `skipB` (0 when none).
    const auto most = [&](unsigned skipA, unsigned skipB) {
        unsigned best = 0, bestCount = 0;
        for (unsigned i = 0; i < 16; ++i) {
            const unsigned ch = plan[i];
            if (!isFiveGHz(ch) || ch == skipA || ch == skipB)
                continue;
            const unsigned n = count(ch);
            if (n > bestCount || (n == bestCount && ch < best))
                best = ch, bestCount = n;
        }
        return best;
    };
    unsigned h = isFiveGHz(home) && count(home) ? home : most(station, 0);
    if (!h)
        return r;
    r.home = uint8_t(h);
    r.homeSlots = uint8_t(count(h));
    const unsigned s = most(h, station);
    if (s) {
        r.second = uint8_t(s);
        r.secondSlots = uint8_t(count(s));
        for (unsigned i = 0; i < 16; ++i)
            if (plan[i] == s)
                r.secondMask = uint16_t(r.secondMask | (1u << i));
    }
    return r;
}

// How often macOS's plan named a second 5 GHz channel, and the last such plan.
struct PlanMix {
    uint32_t plans = 0, withSecond = 0;
    PlanFiveGHz now{}, last{};
    uint8_t lastPlan[16] = {};
    uint64_t lastUS = 0;
    uint8_t station = 0;     // the Mac's Wi-Fi channel at the last plan: a second channel equal to it can't be seen

    // A new plan version. True when what the key shows changed.
    bool note(const uint8_t (&plan)[16], unsigned home, unsigned stationChannel, uint64_t nowUS) {
        const PlanFiveGHz p = planFiveGHz(plan, home, stationChannel);
        const bool changed = p.home != now.home || p.second != now.second || p.secondMask != now.secondMask;
        ++plans;
        now = p;
        station = uint8_t(stationChannel <= 255 ? stationChannel : 0);
        if (p.second) {
            ++withSecond;
            last = p;
            memcpy(lastPlan, plan, sizeof(lastPlan));
            lastUS = nowUS;
        }
        return changed;
    }
};

// The nearby devices' 5 GHz AirDrop channels, from their sync frames (44, 149, or 0 when they name neither).
// Addresses stay in memory only and are never published.
struct PeerChannelMix {
    static constexpr unsigned kEntries = 8;
    struct Entry {
        uint8_t addr[6];
        uint8_t channel;
        bool used;
        uint64_t heardUS;
    };
    Entry entries[kEntries] = {};
    uint32_t mixedSeen = 0;      // times devices on 44 and on 149 were both heard within lifeUS
    uint64_t lastMixedUS = 0;
    bool mixed = false;

    struct Counts { unsigned on44 = 0, on149 = 0, other = 0; };

    Counts counts(uint64_t nowUS, uint64_t lifeUS) const {
        Counts c;
        for (const Entry &e : entries) {
            if (!e.used || nowUS - e.heardUS >= lifeUS)
                continue;
            if (e.channel == 44) ++c.on44;
            else if (e.channel == 149) ++c.on149;
            else ++c.other;
        }
        return c;
    }

    // A device's sync frame. True when the counts the key shows may have changed.
    bool note(const uint8_t *addr, uint8_t channel, uint64_t nowUS, uint64_t lifeUS) {
        Entry *slot = nullptr, *oldest = nullptr;
        for (Entry &e : entries) {
            if (e.used && !memcmp(e.addr, addr, 6)) { slot = &e; break; }
            if (!e.used || nowUS - e.heardUS >= lifeUS) {
                if (!oldest || oldest->used) oldest = &e;
            } else if (!oldest || (oldest->used && e.heardUS < oldest->heardUS)) {
                oldest = &e;
            }
        }
        bool changed = false;
        if (!slot) {
            slot = oldest;
            memcpy(slot->addr, addr, 6);
            slot->used = true;
            slot->channel = 0xff;      // forces "changed" below
        }
        if (slot->channel != channel)
            changed = true;
        slot->channel = channel;
        slot->heardUS = nowUS;
        const Counts c = counts(nowUS, lifeUS);
        const bool both = c.on44 && c.on149;
        if (both) {
            lastMixedUS = nowUS;
            if (!mixed)
                ++mixedSeen;
        }
        if (both != mixed)
            changed = true;
        mixed = both;
        return changed;
    }
};

}  // namespace ItlAWDL
