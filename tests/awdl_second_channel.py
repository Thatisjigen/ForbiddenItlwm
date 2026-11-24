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
    function("static bool awdlSchedSecondTxBlocked()"),
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
static uint32_t awdlSchedSecondErrors, awdlSchedSecondHome, awdlSchedSecondAway;
static uint64_t awdlSchedSecondTookSumUS, awdlSchedSecondTookMaxUS, nowUS;
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
};
// DRIVER
static int iwx_phy_ctxt_cmd(iwx_softc *sc, iwx_phy_ctxt *, int, int, int action, int) {
    assert(action == IWX_FW_CTXT_ACTION_MODIFY);
    assert(awdlSchedSecondSwitching && awdlSchedSecondDraining && awdlSchedSecondTxBlocked());
    ++commands;
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
    }
    // PASSIVE without an allowed no-IR exception is refused; existing permission makes the same channel usable.
    reset(sc); sc.sc_ic.ic_channels[44].ic_flags = IEEE80211_CHAN_PASSIVE;
    assert(!driver.awdlSchedSecondRetune(&sc, 44) && !commands);
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
    assert(awdlCoexTxHalted && awdlSchedSpOff && awdlCoexPhyUncertainGen == 7 && awdlSchedSecondTxBlocked());
    assert(!driver.awdlSchedSecondRetune(&sc, 149) && commands == 1);
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
