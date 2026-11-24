"""Run the driver's real per-peer receive Block Ack step (awdlCoexBaRxPeer, awdlCoexBaAddPeer, awdlCoexBaRemovePeer)
for several peers against mocked firmware commands.

Checks that each peer gets its own station and session and its own answer, that an idle further slot gives its
station back, and that one slot's failure stays in that slot. Bookkeeping and command order only: no firmware,
radio or timing.
"""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / "itlwm/hal_iwx/ItlIwx.cpp").read_text()


def method(signature):
    start = source.index(signature)
    return source[start:source.index("\n}\n", start) + 3]


def static(name):
    start = source.index(name)
    return source[start:source.index("\n}\n", start) + 3]


bodies = "\n".join([
    static("static bool awdlPeerAnyAdded()"),
    static("static void awdlCoexBaFail(ItlAWDL::PeerSlot &slot, int gen)"),
    method("int ItlIwx::\nawdlCoexBaRemovePeer("),
    method("int ItlIwx::\nawdlCoexBaAddPeer("),
    method("bool ItlIwx::\nawdlCoexBaRxPeer("),
])

program = r'''
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include <string>
#include <vector>
#include <AWDLObservation.hpp>
#include <AWDLBlockAck.hpp>
#include <AWDLPeerSlot.hpp>
#define htole16(x) (x)
#define htole32(x) (x)
#define nitems(a) (sizeof(a) / sizeof((a)[0]))
#define IEEE80211_ADDR_COPY(d, s) memcpy(d, s, 6)
#define IWX_FW_CMD_ID_AND_COLOR(id, c) ((id) | ((c) << 8))
enum { IWX_ADD_STA = 0x18, IWX_REMOVE_STA = 0x19, IWX_STA_LINK = 0, IWX_STA_MODE_MODIFY = 1,
    IWX_STA_MODIFY_ADD_BA_TID = 0x8, IWX_STA_MODIFY_REMOVE_BA_TID = 0x10, IWX_ADD_STA_SUCCESS = 1,
    IWX_ADD_STA_STATUS_MASK = 0xff, IWX_ADD_STA_BAID_VALID_MASK = 0x8000, IWX_FW_CTXT_ACTION_MODIFY = 2,
    IWX_STA_FLG_FAT_EN_20MHZ = 0, IWX_STA_FLG_FAT_EN_80MHZ = 1 << 26, IWX_STA_FLG_MIMO_EN_MIMO2 = 1 << 28,
    IWX_STA_FLG_MAX_AGG_SIZE_64K = 3 << 19, IWX_STA_FLG_MAX_AGG_SIZE_1024K = 6 << 19, IWX_STA_FLG_FAT_EN_MSK = 3 << 26,
    IWX_STA_FLG_MIMO_EN_MSK = 3 << 28, IWX_STA_FLG_MAX_AGG_SIZE_MSK = 15 << 19, IWX_STA_FLG_AGG_MPDU_DENS_MSK = 7 << 23,
    kAwdlCoexMacId = 2, kCoexBaTxOff = 0, kCoexSlotBaAnswer = 3, kCoexSlotBaAsk, kCoexSlotDelba };
struct iwx_add_sta_cmd { uint8_t add_modify; uint16_t tid_disable_tx; uint32_t mac_id_n_color; uint8_t addr[6];
    uint8_t sta_id, modify_mask; uint8_t station_type; uint32_t station_flags, station_flags_msk;
    uint8_t add_immediate_ba_tid, remove_immediate_ba_tid; uint16_t add_immediate_ba_ssn, rx_ba_window; };
struct iwx_rm_sta_cmd { uint8_t sta_id; };
struct iwx_tx_ring { int queued, cur; };
struct ieee80211com {};
struct iwx_softc { int sc_generation = 7; iwx_tx_ring txq[8] = {}; ieee80211com sc_ic; };
static ItlAWDL::PeerSlot awdlPeers[ItlAWDL::kMaxPeerSlots];
static ItlAWDL::PeerSlot &awdlPeer = awdlPeers[0];
static unsigned awdlPeerSlotsUsable = 3;
static uint32_t awdlPeerSlotAdds, awdlPeerSlotRemoves, awdlPeerSlotFailures;
static int awdlCoexBaFailedGen = -1, awdlCoexBaTxResetting = -1;
static bool awdlCoexAdded = true, awdlCoexBaFilterOff, awdlCoexBaTxEnabled = true;
static uint32_t awdlCoexToken = 5, awdlCoexBaStatus, awdlCoexBaSessions, awdlCoexBaAnswers, awdlCoexBaAnswerErrors,
    awdlCoexBaOtherPeer, awdlCoexBaRxIdleTeardowns, awdlCoexBaDelbasSent, awdlCoexDelbaForced, awdlCoexDelbaConfirmed,
    awdlCoexDelbaRxTidRemoved, awdlCoexRateWindowOK, awdlCoexRateWindowFailed;
static unsigned awdlCoexRateStep;
static uint16_t awdlCoexBaSequence, awdlCoexLastMgmtEnqueueId;
static uint64_t awdlCoexTxPauseUntilUS;
static uint8_t awdlCoexAddr[6] = {2, 9, 9, 9, 9, 9};
static ItlAWDL::BaFilterState awdlCoexBaFilter;
static struct { bool applied, restorePending; } awdlCoexSameChannelProbe;
static uint64_t clockUS = 100000000ULL;
static uint64_t awdlUptimeUS() { return clockUS; }
static bool known = true;
static bool awdlCoexPeerKnown(const uint8_t *, uint64_t, bool = false) { return known; }
static void awdlCoexBaEvent(char, unsigned, unsigned, unsigned, unsigned = 0, unsigned = 0) {}
static int awdlCoexBaWidth(ieee80211com *) { return 80; }
struct Command { int id; uint8_t sta, mask, tid; uint8_t addr[6]; };
static std::vector<Command> commands;
static std::vector<std::vector<uint8_t>> sent;
static int failAddFor = -1;     // ADD_STA (new station) fails for this station number
class ItlIwx {
public:
    int iwx_send_cmd_pdu_status(iwx_softc *, int id, size_t, const void *data, uint32_t *status) {
        const auto *c = static_cast<const iwx_add_sta_cmd *>(data);
        Command k{id, c->sta_id, c->modify_mask, c->modify_mask == IWX_STA_MODIFY_ADD_BA_TID ? c->add_immediate_ba_tid :
                  c->remove_immediate_ba_tid, {}};
        memcpy(k.addr, c->addr, 6);
        commands.push_back(k);
        if (!c->add_modify && c->sta_id == failAddFor) {
            *status = 0;
            return 0;
        }
        *status = IWX_ADD_STA_SUCCESS | (c->modify_mask == IWX_STA_MODIFY_ADD_BA_TID ? IWX_ADD_STA_BAID_VALID_MASK : 0);
        return 0;
    }
    int iwx_send_cmd_pdu(iwx_softc *, int id, int, size_t, const void *data) {
        const auto *c = static_cast<const iwx_rm_sta_cmd *>(data);
        commands.push_back(Command{id, c->sta_id, 0, 0, {}});
        return 0;
    }
    int iwx_flush_sta_tids(iwx_softc *, int, int) { return 0; }
    void iwx_reset_tx_ring(iwx_softc *, iwx_tx_ring *) {}
    int awdlCoexMacSend(iwx_softc *, uint32_t) { return 0; }
    bool awdlCoexSendMgmt(iwx_softc *, const uint8_t *frame, size_t length, uint8_t) {
        sent.push_back(std::vector<uint8_t>(frame, frame + length));
        return true;
    }
    int awdlCoexBaRemovePeer(iwx_softc *, ItlAWDL::PeerSlot &);
    int awdlCoexBaAddPeer(iwx_softc *, ItlAWDL::PeerSlot &, const uint8_t *);
    bool awdlCoexBaRxPeer(iwx_softc *, ItlAWDL::PeerSlot &, int, uint32_t, uint64_t);
};
''' + bodies + r'''
static ItlIwx driver;
static iwx_softc hardware;
static ItlAWDL::AddbaRequest request(uint8_t last, uint8_t tid, uint8_t token) {
    ItlAWDL::AddbaRequest r{};
    r.immediate = true;
    r.peer[0] = 2;
    r.peer[5] = last;
    r.tid = tid;
    r.token = token;
    r.window = 64;
    return r;
}
// What the receive path does with an ADDBA request (iwx_rx_frame).
static bool deliver(const ItlAWDL::AddbaRequest &r) {
    ItlAWDL::PeerSlot *slot = ItlAWDL::slotForRequest(awdlPeers, awdlPeerSlotsUsable, r.peer, hardware.sc_generation);
    return slot && slot->requests.note(r);
}
// The task's receive part: slot 0, then the others; a pass ends at the first slot that sent a command.
static void pass() {
    const int gen = hardware.sc_generation;
    if (!driver.awdlCoexBaRxPeer(&hardware, awdlPeer, gen, awdlCoexToken, clockUS))
        return;
    for (unsigned i = 1; i < awdlPeerSlotsUsable; ++i)
        if (!driver.awdlCoexBaRxPeer(&hardware, awdlPeers[i], gen, awdlCoexToken, clockUS))
            return;
}
static void settle() { for (int i = 0; i < 40; ++i) pass(); }
static unsigned count(int id, uint8_t sta, uint8_t mask = 0xff) {
    unsigned n = 0;
    for (const auto &c : commands)
        n += c.id == id && c.sta == sta && (mask == 0xff || c.mask == mask);
    return n;
}
static void reset() {
    // Every slot starts with the default station number (3), as after a reset that doesn't renumber: the
    // driver must still use 3 + slot (the stick test on 2026-11-24 showed every slot at 3).
    for (unsigned i = 0; i < ItlAWDL::kMaxPeerSlots; ++i)
        awdlPeers[i] = ItlAWDL::PeerSlot{};
    commands.clear();
    sent.clear();
    failAddFor = -1;
    awdlCoexBaFailedGen = -1;
    awdlPeerSlotsUsable = 3;
    hardware = iwx_softc{};
    clockUS = 100000000ULL;
    known = true;
}
int main() {
    unsigned cases = 0;
    // Two peers asking at once: each gets its own station, session and answer.
    reset();
    assert(deliver(request(10, 0, 1)) && deliver(request(11, 0, 2)) && deliver(request(11, 5, 3)));
    settle();
    assert(awdlPeers[0].added && awdlPeers[1].added && !awdlPeers[2].added);
    assert(ItlAWDL::sameAddress(awdlPeers[0].addr, request(10, 0, 0).peer));
    assert(ItlAWDL::sameAddress(awdlPeers[1].addr, request(11, 0, 0).peer));
    assert(count(0x18, 3, 0) == 1 && count(0x18, 4, 0) == 1);            // one new station each
    assert(awdlPeers[0].station == 3 && awdlPeers[1].station == 4);
    for (const auto &c : commands)                                         // and each command names its own
        if (c.id == 0x18 && !c.mask)
            assert(c.sta == 3 ? ItlAWDL::sameAddress(c.addr, request(10, 0, 0).peer) :
                   ItlAWDL::sameAddress(c.addr, request(11, 0, 0).peer));
    assert(count(0x18, 3, 0x8) == 1 && count(0x18, 4, 0x8) == 2);        // A: TID 0; B: TIDs 0 and 5
    assert(awdlPeers[0].rxTids == 0x01 && awdlPeers[1].rxTids == 0x21);
    unsigned toA = 0, toB = 0;
    for (const auto &f : sent) {
        toA += ItlAWDL::sameAddress(f.data() + 4, request(10, 0, 0).peer);
        toB += ItlAWDL::sameAddress(f.data() + 4, request(11, 0, 0).peer);
    }
    assert(toA == 1 && toB == 2 && awdlCoexBaAnswers == 0 + sent.size());
    ++cases;
    // A third peer with a third slot; a fourth finds none.
    assert(deliver(request(12, 0, 4)));
    settle();
    assert(awdlPeers[2].added && count(0x18, 5, 0) == 1);
    assert(!deliver(request(13, 0, 5)));
    ++cases;
    // A further slot gives its station back once idle: receive sessions first (DELBA, forced after 3 s), then the
    // station after 60 s without data. Slot 0 keeps its station for ten minutes.
    clockUS += 61000000ULL;
    for (unsigned i = 0; i < 3; ++i)
        for (unsigned t = 0; t < 8; ++t)
            if (awdlPeers[i].rxTids & (1U << t))
                awdlPeers[i].rxTidLastUS[t] = clockUS - 61000000ULL;
    for (unsigned i = 0; i < 3; ++i)
        awdlPeers[i].lastUS = clockUS - 61000000ULL;
    for (int round = 0; round < 12; ++round) {
        settle();
        clockUS += 1000000ULL;
    }
    assert(awdlPeers[0].added && !awdlPeers[1].added && !awdlPeers[2].added);
    assert(count(0x19, 4) == 1 && count(0x19, 5) == 1 && count(0x19, 3) == 0);
    assert(count(0x18, 4, 0x10) == 2 && count(0x18, 5, 0x10) == 1);    // its sessions went first
    ++cases;
    // A failed station add takes only that slot out of use; others go on, and the next peer gets the next slot.
    reset();
    failAddFor = 4;
    assert(deliver(request(10, 0, 1)) && deliver(request(11, 0, 2)));
    settle();
    assert(awdlPeers[0].added && !awdlPeers[1].added && awdlPeers[1].failedGen == hardware.sc_generation);
    assert(awdlCoexBaFailedGen == -1 && awdlPeerSlotFailures);
    assert(deliver(request(12, 0, 3)));
    settle();
    assert(awdlPeers[2].added && ItlAWDL::sameAddress(awdlPeers[2].addr, request(12, 0, 0).peer));
    assert(!deliver(request(11, 0, 4)));                                   // slot 1 stays out this generation
    ++cases;
    // Slot 0's failure still quarantines all Block Ack, as before.
    reset();
    failAddFor = 3;
    assert(deliver(request(10, 0, 1)));
    settle();
    assert(awdlCoexBaFailedGen == hardware.sc_generation && !awdlPeers[0].added);
    ++cases;
    // One slot (no -itlawdlmultipeer): a second peer is refused while the first has the station.
    reset();
    awdlPeerSlotsUsable = 1;
    assert(deliver(request(10, 0, 1)));
    settle();
    assert(awdlPeers[0].added && !deliver(request(11, 0, 2)) && count(0x18, 4) == 0);
    ++cases;
    // A peer that already has a station in slot 0 never gets a second one in another slot.
    reset();
    assert(deliver(request(10, 0, 1)));
    settle();
    assert(awdlPeers[1].requests.note(request(10, 3, 2)));
    settle();
    assert(!awdlPeers[1].added && awdlPeers[1].requests.empty() && count(0x18, 4) == 0);
    ++cases;
    printf("PASS %u multi-peer receive Block Ack scenarios on the real task step (host mocks only)\n", cases);
}
'''

with tempfile.TemporaryDirectory(prefix="forbiddenitlwm-multipeer-") as directory:
    src = Path(directory) / "test.cpp"
    src.write_text(program)
    exe = Path(directory) / "test"
    subprocess.run(["xcrun", "clang++", "-std=c++14", "-I" + str(root / "include"),
                    "-fsanitize=address,undefined", str(src), "-o", str(exe)], check=True)
    print(subprocess.check_output([str(exe)], text=True), end="")
