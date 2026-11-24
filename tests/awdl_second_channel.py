"""Exercise the real listener PHY retune against firmware/gate stubs, never a radio."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / "itlwm/hal_iwx/ItlIwx.cpp").read_text()


def function(signature):
    start = source.index(signature)
    return source[start:source.index("\n}\n", start) + 3]


implementations = "\n".join([
    function("static bool awdlSchedSecondAwayTxOpen()"),
    function("static bool awdlSchedSecondTxBlocked()"),
    function("static bool awdlSchedSecondTxAllowed("),
    function("static inline unsigned awdlCoexUnii("),
    function("static inline bool awdlCoexNoIr("),
    function("static bool\nawdlCoexChannelUsable("),
    function("static bool awdlSchedSecondReceiveChannel("),
    function("static bool awdlSchedSecondOwnsPhy()"),
    function("bool ItlIwx::\nawdlSchedSecondRetune("),
    function("void ItlIwx::\nawdlSchedFollowTask("),
])

program = r'''
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include <cstddef>
#include <AWDLSessionFollow.hpp>
#include <AWDLPeerSlot.hpp>
#include <AWDLTxPass.hpp>
#define nitems(x) (sizeof(x) / sizeof((x)[0]))
enum { IEEE80211_CHAN_DFS = 1, IEEE80211_CHAN_PASSIVE = 2, IEEE80211_CHAN_NOADHOC = 4,
       IWX_PHY_VHT_CHANNEL_MODE20 = 0, IWX_PHY_VHT_CHANNEL_MODE80 = 2,
       IWX_FW_CTXT_ACTION_MODIFY = 2, kAwdlCoexPhyId = 1 };
struct ieee80211_channel { unsigned ic_freq = 0, ic_flags = 0; };
enum { IEEE80211_S_RUN = 4, IWX_FLAG_HW_ERR = 1, IWX_FLAG_SHUTDOWN = 2, IWX_FLAG_RFKILL = 4, kAwdlSchedCh6MacId = 4 };
struct ieee80211_node { ieee80211_channel *ni_chan = nullptr; };
struct ieee80211com {
    int ic_state = IEEE80211_S_RUN;
    ieee80211_node *ic_bss = nullptr;
    ieee80211_channel ic_channels[256];
    unsigned ic_chan_active[256] = {}, ic_beacon_hint[256] = {};
};
#define IEEE80211_CHAN_MAX 255
#define IEEE80211_CHAN_ANYC reinterpret_cast<ieee80211_channel *>(-1)
static bool awdlCoexIrConcurrentOff;
static unsigned awdlCoexIrConcurrent[256];
static uint32_t awdlCoexIrConcurrentUsed;
static ieee80211_node bss;
struct iwx_phy_ctxt { ieee80211_channel *channel = nullptr; };
struct iwx_softc {
    int sc_generation = 7, sc_flags = 0;
    ieee80211com sc_ic;
    iwx_phy_ctxt sc_phyctxt[3];
    struct { unsigned queued = 0; } txq[16];
};
#define IEEE80211_IS_CHAN_5GHZ(c) ((c)->ic_freq >= 5000)
static bool isset(const unsigned *set, unsigned ch) { return set[ch] != 0; }
static unsigned ieee80211_chan2ieee(ieee80211com *ic, ieee80211_channel *c) { return unsigned(c - ic->ic_channels); }
static bool awdlSchedSecond = true, awdlCoexAdded, awdlCoexMultiActive, awdlCoexTxHalted, awdlCoexMovePending;
static bool awdlSchedSecondSwitching, awdlSchedSecondDraining, awdlSchedSpActive, awdlSchedSpAwaitStart;
static bool awdlCoexInFlight, awdlSchedSpOff, awdlSchedFollowArmed;
static int awdlCoexGeneration, awdlCoexPhyUncertainGen, awdlCoexStationTransitionGen, awdlSchedSpGen;
static int awdlSchedSecondOffGen, awdlSchedSecondLastErr, awdlCoexOwnWidth, awdlCoexPhyWidth;
static ieee80211_channel *awdlCoexOwnChan, *awdlSchedSecondTuned;
static uint32_t awdlCoexToken, awdlSchedSecondRefused, awdlSchedSecondBusy, awdlSchedSecondStale;
static uint32_t awdlSchedSecondRefusedCh, awdlSchedSecondRefusedWhy, awdlSchedSecondRefusedFlags;
static uint32_t awdlSchedSecondErrors, awdlSchedSecondHome, awdlSchedSecondAway;
static uint64_t awdlSchedSecondTookSumUS, awdlSchedSecondTookMaxUS, nowUS;
static uint32_t awdlSchedSecondVariant, awdlSchedSecondSettleWaits, awdlSchedSecondPeerRemovals, awdlSchedSecondMacErrors;
static bool awdlSchedSecondBootOff;
static uint64_t awdlSchedSpEndedUS;
static constexpr uint64_t kAwdlSchedSecondSettleUS = 30000;
static unsigned peerRemoves, macSends, lastWidth;
static int peerRemoveError;
// The listener's width rule for a channel beside the station (awdlCoexWidthFor), stubbed: tests set what it allows.
static bool widthFits = true; static uint8_t widthAllowed = 2;
struct iwx_softc;
static bool awdlCoexWidthFor(iwx_softc *, unsigned, bool, uint8_t *w, uint8_t *) { *w = widthAllowed; return widthFits; }
static ItlAWDL::PeerSlot awdlPeers[ItlAWDL::kMaxPeerSlots];
static uint64_t awdlUptimeUS() { return nowUS; }
static void awdlSchedSpPublish() {}
static unsigned commands;
static int commandError, mutation;
static int iwx_phy_ctxt_cmd(iwx_softc *sc, iwx_phy_ctxt *, int, int, int, int);
static bool awdlStopping, awdlSchedPaused;
static bool awdlSchedAwdlOff() { return false; }
static int awdlSchedFollowArmedGen;
static uint32_t awdlSchedFollowArmedToken, awdlSchedFollowArmedPlan, awdlSchedFollowArmedMac;
static uint32_t itlAWDLPlanVersion, awdlSchedSpTargetMac;
static unsigned awdlSchedFollowArmedChannel, sessionCalls, sessionDuration;
static uint64_t awdlSchedFollowEndUS, awdlSchedFollowSendAtUS, awdlSchedFollowArmedRunStartUS;
static uint64_t awdlSchedFollowLateSumUS, awdlSchedFollowLateMaxUS, awdlSchedSpRunStartUS, awdlSchedSpPlannedEndUS;
static bool awdlSchedSecondTx;
static ItlAWDL::TxPassList awdlSchedSecondPass;
static constexpr uint64_t kAwdlSchedSecondTxStopUS = 4000;
static unsigned awdlSchedFollowFired, awdlSchedOffStale, awdlSchedFollowStale, awdlSchedFollowLateN, awdlSchedFollowAligned;
static ItlAWDL::Clock awdlSchedFollowArmedClock, awdlSchedPlanClockNow;
using IOReturn = int;
using OSObject = void;
constexpr IOReturn kIOReturnSuccess = 0;
struct WorkLoop { bool inGate() { return true; } };
struct Gate { void runAction(IOReturn (*f)(OSObject *, void *, void *, void *, void *), void *arg) { f(nullptr,arg,nullptr,nullptr,nullptr); } };
#define container_of(ptr,type,member) reinterpret_cast<type *>(reinterpret_cast<char *>(ptr)-offsetof(type,member))
class ItlIwx {
public:
    iwx_softc com;
    WorkLoop loop;
    Gate gate;
    WorkLoop *getMainWorkLoop() { return &loop; }
    Gate *getMainCommandGate() { return &gate; }
    bool awdlSchedSecondRetune(iwx_softc *, unsigned);
    static void awdlSchedFollowTask(void *);
    int awdlSchedSpSend(iwx_softc *, uint32_t duration) { ++sessionCalls; sessionDuration=duration; return 0; }
    int awdlCoexBaRemoveAllPeers(iwx_softc *) { assert(!awdlSchedSecondSwitching && peerRemoves == commands); ++peerRemoves; return peerRemoveError; }
    int awdlCoexMacSend(iwx_softc *, uint32_t action) { assert(action == IWX_FW_CTXT_ACTION_MODIFY && commands); ++macSends; return 0; }
};
// DRIVER
static int iwx_phy_ctxt_cmd(iwx_softc *sc, iwx_phy_ctxt *, int, int, int action, int) {
    assert(action == IWX_FW_CTXT_ACTION_MODIFY);
    assert(awdlSchedSecondSwitching && awdlSchedSecondDraining && awdlSchedSecondTxBlocked());
    ++commands;
    lastWidth = unsigned(awdlCoexPhyWidth);
    nowUS += 700;
    if (mutation == 3) {   // reentrant timer bookkeeping must not rewrite the executing request
        awdlSchedFollowEndUS = 9999999;
        awdlSchedFollowArmedMac = kAwdlSchedCh6MacId;
    } else if (mutation == 4) {
        itlAWDLPlanVersion += 2;
    } else if (mutation == 5) {
        awdlSchedPlanClockNow.anchorEndUS += 5000;
    } else if (mutation == 6) {
        nowUS = 200000;
    } else if (mutation) {
        if (mutation == 1) ++sc->sc_generation;
        else ++awdlCoexToken;
        // The new listener is home; it must not be overwritten when the old command returns.
        sc->sc_phyctxt[1].channel = &sc->sc_ic.ic_channels[149];
        awdlSchedSecondTuned = nullptr;
        awdlSchedSecondSwitching = awdlSchedSecondDraining = false;
    }
    return commandError;
}
static void reset(iwx_softc &sc) {
    sc = iwx_softc{};
    for (unsigned ch : {44u, 149u}) {
        sc.sc_ic.ic_channels[ch].ic_freq = 5000 + ch * 5;
        sc.sc_ic.ic_chan_active[ch] = 1;
    }
    awdlCoexOwnChan = &sc.sc_ic.ic_channels[149];
    sc.sc_phyctxt[1].channel = awdlCoexOwnChan;
    awdlSchedSecond = true;
    awdlCoexAdded = awdlCoexMultiActive = true;
    awdlCoexTxHalted = awdlCoexMovePending = awdlCoexInFlight = false;
    awdlSchedSecondSwitching = awdlSchedSecondDraining = awdlSchedSpActive = awdlSchedSpAwaitStart = false;
    awdlSchedSpOff = awdlSchedFollowArmed = false;
    awdlCoexGeneration = awdlSchedSpGen = 7;
    awdlCoexPhyUncertainGen = awdlCoexStationTransitionGen = awdlSchedSecondOffGen = -1;
    awdlCoexToken = 9;
    awdlCoexOwnWidth = 80;
    awdlCoexPhyWidth = -1;
    awdlSchedSecondTuned = nullptr;
    for (auto &peer : awdlPeers) peer = ItlAWDL::PeerSlot{};
    awdlCoexIrConcurrentOff = false;
    memset(awdlCoexIrConcurrent, 0, sizeof(awdlCoexIrConcurrent));
    bss.ni_chan = &sc.sc_ic.ic_channels[36];
    sc.sc_ic.ic_bss = &bss;
    nowUS = 10000;
    commands = mutation = commandError = sessionCalls = sessionDuration = 0;
    awdlStopping = awdlSchedPaused = false;
    awdlSchedFollowArmedGen = 7;
    awdlSchedFollowArmedToken = 9;
    awdlSchedFollowArmedPlan = itlAWDLPlanVersion = 2;
    awdlSchedFollowArmedMac = 1;
    awdlSchedFollowArmedChannel = 44;
    awdlSchedFollowEndUS = 110000;
    awdlSchedFollowSendAtUS = 10000;
    awdlSchedFollowArmedRunStartUS = 12000;
    awdlSchedFollowArmedClock = awdlSchedPlanClockNow = ItlAWDL::ownClock();
    awdlSchedSecondStale = awdlSchedSecondBusy = awdlSchedSecondErrors = 0;
    awdlSchedSecondVariant = awdlSchedSecondSettleWaits = awdlSchedSecondPeerRemovals = awdlSchedSecondMacErrors = 0;
    awdlSchedSecondBootOff = false; awdlSchedSpEndedUS = 0; awdlSchedSpOff = false;
    peerRemoves = macSends = lastWidth = 0; peerRemoveError = 0;
    widthFits = true; widthAllowed = IWX_PHY_VHT_CHANNEL_MODE80;
}
static int awdlCoexQueue = 2;
int main() {
    ItlIwx driver;
    iwx_softc sc;
    reset(sc);
    assert(!awdlSchedSecondOwnsPhy());
    awdlSchedFollowArmed = true;
    assert(awdlSchedSecondOwnsPhy());
    awdlSchedFollowArmed = false;
    assert(driver.awdlSchedSecondRetune(&sc, 44) && commands == 1);
    assert(awdlSchedSecondOwnsPhy());
    assert(awdlSchedSecondTuned == &sc.sc_ic.ic_channels[44] && awdlSchedSecondTxBlocked());
    assert(awdlSchedSecondReceiveChannel(&sc.sc_ic, 44));
    assert(!awdlSchedSecondReceiveChannel(&sc.sc_ic, 149));
    assert(driver.awdlSchedSecondRetune(&sc, 149) && commands == 2 && !awdlSchedSecondTxBlocked());
    for (unsigned q = 0; q < ItlAWDL::kMaxPeerSlots; ++q) {
        reset(sc);
        awdlPeers[q].txQueue = 3;
        sc.txq[3].queued = 1;
        assert(!driver.awdlSchedSecondRetune(&sc, 44) && !commands && awdlSchedSecondDraining);
        sc.txq[3].queued = 0;
        assert(driver.awdlSchedSecondRetune(&sc, 44));
    }
    reset(sc);
    sc.txq[2].queued = 1;
    assert(!driver.awdlSchedSecondRetune(&sc, 44) && !commands);
    // A missed run cancels its drain simply by staying home, without dropping the queued frame.
    assert(driver.awdlSchedSecondRetune(&sc, 149) && !awdlSchedSecondDraining && sc.txq[2].queued == 1);
    reset(sc);
    awdlCoexInFlight = true;
    assert(!driver.awdlSchedSecondRetune(&sc, 44) && !commands);
    for (unsigned mode = 0; mode < 3; ++mode) {
        reset(sc);
        awdlPeers[1].adding = mode == 0;
        awdlPeers[1].queueAllocPending = mode == 1;
        awdlPeers[1].resumePending = mode == 2;
        assert(!driver.awdlSchedSecondRetune(&sc, 44) && !commands);
    }
    for (unsigned flags : {1u, 4u}) {
        reset(sc);
        sc.sc_ic.ic_channels[44].ic_flags = flags;
        assert(!driver.awdlSchedSecondRetune(&sc, 44) && !commands && awdlSchedSecondOffGen == 7);
        assert(awdlSchedSecondRefusedCh == 44 && awdlSchedSecondRefusedFlags == flags);
        assert(awdlSchedSecondRefusedWhy == (flags == IEEE80211_CHAN_DFS ? 32u : 64u));
        // the next try in the same generation says it was already off
        assert(!driver.awdlSchedSecondRetune(&sc, 44) && (awdlSchedSecondRefusedWhy & 1u));
    }
    // PASSIVE without an allowed no-IR exception is refused; existing permission makes the same channel usable.
    reset(sc); sc.sc_ic.ic_channels[44].ic_flags = IEEE80211_CHAN_PASSIVE;
    assert(!driver.awdlSchedSecondRetune(&sc, 44) && !commands && awdlSchedSecondRefusedWhy == 16u);
    reset(sc); sc.sc_ic.ic_channels[44].ic_flags = IEEE80211_CHAN_PASSIVE;
    awdlCoexIrConcurrent[44] = 1;
    assert(driver.awdlSchedSecondRetune(&sc, 44) && commands == 1);
    // The firmware bit alone is insufficient: association must be in the same UNII sub-band.
    reset(sc); sc.sc_ic.ic_channels[44].ic_flags = IEEE80211_CHAN_PASSIVE;
    awdlCoexIrConcurrent[44] = 1; bss.ni_chan = &sc.sc_ic.ic_channels[149];
    assert(!driver.awdlSchedSecondRetune(&sc, 44) && !commands);
    reset(sc); sc.sc_ic.ic_channels[44].ic_flags = IEEE80211_CHAN_PASSIVE;
    awdlCoexIrConcurrent[44] = 1; sc.sc_ic.ic_bss = nullptr;
    assert(!driver.awdlSchedSecondRetune(&sc, 44) && !commands);
    reset(sc); sc.sc_ic.ic_channels[44].ic_flags = IEEE80211_CHAN_PASSIVE;
    awdlCoexIrConcurrent[44] = 1; awdlCoexIrConcurrentOff = true;
    assert(!driver.awdlSchedSecondRetune(&sc, 44) && !commands);
    reset(sc); sc.sc_ic.ic_channels[44].ic_flags = IEEE80211_CHAN_PASSIVE;
    assert(!driver.awdlSchedSecondRetune(&sc, 44) && !commands);
    reset(sc); sc.sc_ic.ic_chan_active[44] = 0;
    assert(!driver.awdlSchedSecondRetune(&sc, 44) && !commands);
    reset(sc); sc.sc_ic.ic_channels[44].ic_freq = 0;
    assert(!driver.awdlSchedSecondRetune(&sc, 44) && !commands);
    reset(sc);
    assert(!driver.awdlSchedSecondRetune(&sc, 256) && !commands);
    awdlSchedSpActive = true;
    assert(!driver.awdlSchedSecondRetune(&sc, 44) && !commands);
    awdlSchedSpActive = false; awdlSchedSpAwaitStart = true;
    assert(!driver.awdlSchedSecondRetune(&sc, 44) && !commands);
    reset(sc); commandError = 5;
    assert(!driver.awdlSchedSecondRetune(&sc, 44));
    // The scheduler is left on (it resumes after the firmware restart); the experiment is off for the boot.
    assert(awdlCoexTxHalted && !awdlSchedSpOff && awdlSchedSecondBootOff && awdlCoexPhyUncertainGen == 7 &&
           awdlSchedSecondTxBlocked());
    assert(!driver.awdlSchedSecondRetune(&sc, 149) && commands == 1);
    {   // a later generation (after the restart) still refuses the away channel
        const bool off = awdlSchedSecondBootOff;
        reset(sc); awdlSchedSecondBootOff = off; sc.sc_generation = awdlCoexGeneration = awdlSchedSpGen = 8;
        assert(!driver.awdlSchedSecondRetune(&sc, 44) && !commands && (awdlSchedSecondRefusedWhy & 128u));
    }
    // a restart during the MODIFY also turns it off for the boot
    reset(sc); mutation = 1;
    assert(!driver.awdlSchedSecondRetune(&sc, 44) && awdlSchedSecondBootOff);
    // default: away at the home width (80), no peer removal or MAC command
    reset(sc);
    assert(driver.awdlSchedSecondRetune(&sc, 44) && lastWidth == IWX_PHY_VHT_CHANNEL_MODE80 && !peerRemoves && !macSends);
    assert(driver.awdlSchedSecondRetune(&sc, 149) && lastWidth == IWX_PHY_VHT_CHANNEL_MODE80);
    // only 20 MHz allowed there while home is at 80: skipped (it asserted the firmware), not off for the generation
    reset(sc); widthAllowed = IWX_PHY_VHT_CHANNEL_MODE20;
    assert(!driver.awdlSchedSecondRetune(&sc, 44) && !commands && awdlSchedSecondRefusedWhy == 512u &&
           awdlSchedSecondOffGen != 7 && !awdlSchedSecondDraining);
    widthAllowed = IWX_PHY_VHT_CHANNEL_MODE80;
    assert(driver.awdlSchedSecondRetune(&sc, 44) && commands == 1);
    // no usable width there
    reset(sc); widthFits = false;
    assert(!driver.awdlSchedSecondRetune(&sc, 44) && !commands && awdlSchedSecondRefusedWhy == 256u);
    // home at 20: away at 20 too, even where 80 would be allowed
    reset(sc); awdlCoexOwnWidth = 20;
    assert(driver.awdlSchedSecondRetune(&sc, 44) && lastWidth == IWX_PHY_VHT_CHANNEL_MODE20);
    // with the stations removed first (variant 2), a 20 MHz-only channel may be visited
    reset(sc); awdlSchedSecondVariant = 2; widthAllowed = IWX_PHY_VHT_CHANNEL_MODE20;
    assert(driver.awdlSchedSecondRetune(&sc, 44) && lastWidth == IWX_PHY_VHT_CHANNEL_MODE20 && peerRemoves == 1);
    // variant 1: waits 30 ms after the last session's end
    reset(sc); awdlSchedSecondVariant = 1; awdlSchedSpEndedUS = nowUS - 1000;
    assert(!driver.awdlSchedSecondRetune(&sc, 44) && !commands && awdlSchedSecondSettleWaits == 1);
    nowUS = awdlSchedSpEndedUS + kAwdlSchedSecondSettleUS;
    assert(driver.awdlSchedSecondRetune(&sc, 44) && commands == 1);
    // variant 2: peers removed before, MAC MODIFY after, both ways; a removal failure stops the retune
    reset(sc); awdlSchedSecondVariant = 2;
    assert(driver.awdlSchedSecondRetune(&sc, 44) && peerRemoves == 1 && macSends == 1);
    assert(driver.awdlSchedSecondRetune(&sc, 149) && peerRemoves == 2 && macSends == 2);
    reset(sc); awdlSchedSecondVariant = 2; peerRemoveError = 5;
    assert(!driver.awdlSchedSecondRetune(&sc, 44) && !commands && awdlSchedSecondErrors == 1 && !awdlSchedSecondDraining);
    for (int change : {1, 2}) {
        reset(sc); mutation = change;
        assert(!driver.awdlSchedSecondRetune(&sc, 44));
        assert(awdlSchedSecondStale == 1 && sc.sc_phyctxt[1].channel == awdlCoexOwnChan);
        assert(!awdlSchedSecondTuned && !awdlCoexTxHalted);
    }
    // Actual callback: its duration and MAC come from the frozen request, not mutable timer globals.
    reset(driver.com); awdlSchedFollowArmed = true; mutation = 3;
    ItlIwx::awdlSchedFollowTask(&driver.com);
    assert(sessionCalls == 1 && awdlSchedSpTargetMac == 1 && awdlSchedSpPlannedEndUS == 110000);
    assert(sessionDuration == ItlAWDL::followDurationTU(nowUS, 110000));
    // A changed plan/clock or a run that expires during MODIFY cannot start a stale session.
    for (int change : {4, 5, 6}) {
        reset(driver.com); awdlSchedFollowArmed = true; mutation = change;
        ItlIwx::awdlSchedFollowTask(&driver.com);
        assert(commands == 1 && sessionCalls == 0);
    }
    reset(driver.com); awdlSchedFollowArmed = true; awdlSchedFollowArmedToken = 8;
    ItlIwx::awdlSchedFollowTask(&driver.com);
    assert(!commands && !sessionCalls);
    reset(sc); awdlSchedSecond = false;
    awdlSchedSecondSwitching = awdlSchedSecondDraining = awdlSchedSpActive = true;
    assert(!awdlSchedSecondOwnsPhy() && !awdlSchedSecondTxBlocked());
    {   // -itlschedsecondtx: during an open visit only the pass list's peers' unicast data may be submitted
        static const uint8_t onThere[6] = {2, 1, 1, 1, 1, 1}, home[6] = {2, 2, 2, 2, 2, 2}, group[6] = {0x33, 0x33, 0, 0, 0, 1};
        reset(sc);
        assert(driver.awdlSchedSecondRetune(&sc, 44) && awdlSchedSecondTuned);
        awdlSchedSecondPass.clear(); awdlSchedSecondPass.active = true; awdlSchedSecondPass.add(onThere);
        awdlSchedSecondTx = false; awdlSchedSpPlannedEndUS = nowUS + 50000;
        assert(awdlSchedSecondTxBlocked() && !awdlSchedSecondTxAllowed(onThere));   // flag off: listen only
        awdlSchedSecondTx = true;
        assert(awdlSchedSecondAwayTxOpen() && !awdlSchedSecondTxBlocked());
        assert(awdlSchedSecondTxAllowed(onThere) && !awdlSchedSecondTxAllowed(home) &&
               !awdlSchedSecondTxAllowed(group) && !awdlSchedSecondTxAllowed(nullptr));
        nowUS = awdlSchedSpPlannedEndUS - kAwdlSchedSecondTxStopUS;   // the last few ms: closed
        assert(!awdlSchedSecondAwayTxOpen() && awdlSchedSecondTxBlocked() && !awdlSchedSecondTxAllowed(onThere));
        awdlSchedSpPlannedEndUS = 0;                                  // no session asked for this visit: closed
        assert(!awdlSchedSecondAwayTxOpen());
        awdlSchedSpPlannedEndUS = nowUS + 50000;
        awdlSchedSecondDraining = true;                                // leaving: closed
        assert(awdlSchedSecondTxBlocked());
        awdlSchedSecondDraining = false;
        assert(driver.awdlSchedSecondRetune(&sc, 149) && !awdlSchedSecondTuned);
        assert(awdlSchedSecondTxAllowed(home) && awdlSchedSecondTxAllowed(group));   // home: as before
        awdlSchedSecondTx = false; awdlSchedSpPlannedEndUS = 0; awdlSchedSecondPass.clear();
    }
    puts("Second-channel real retune: queue drains, TX barrier, regulatory refusal, session guards, stale ownership and failure quarantine passed (host only)");
}
'''
# The queue number must be visible to the extracted production method.
program = program.replace("// DRIVER", "static int awdlCoexQueue = 2;\n" + implementations)
program = program.replace("static int awdlCoexQueue = 2;\nint main()", "int main()")
with tempfile.TemporaryDirectory(prefix="forbidden-second-channel-") as directory:
    cpp = Path(directory) / "test.cpp"
    cpp.write_text(program)
    binary = Path(directory) / "test"
    subprocess.run(["xcrun", "clang++", "-std=c++14", "-fsanitize=address,undefined",
                    "-I" + str(root / "include"), str(cpp), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
