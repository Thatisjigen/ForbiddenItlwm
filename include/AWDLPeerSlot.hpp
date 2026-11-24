// What the listener keeps for one AWDL peer it exchanges data with: its firmware station, receive Block Ack
// sessions, send agreement and aggregate queue, the station pause and the bulk detectors. These used to be
// separate globals for the one peer station (3). Holding them per peer is the first step towards a
// station for every peer with traffic, as Apple's firmware keeps per-peer state for each such peer, up to its
// own limit. Slot 0 is the peer we may also send aggregated to; with -itlawdlmultipeer the others give further
// peers their own station and receive sessions.
#pragma once
#include "AWDLBlockAck.hpp"
#include "AWDLPeerSchedule.hpp"
namespace ItlAWDL {
// Firmware stations for AWDL peers: 3 and up (0 is the Wi-Fi network's, 1 the aux station, 2 the listener's).
constexpr uint8_t kFirstPeerStation = 3;
constexpr unsigned kMaxPeerSlots = 12;
struct PeerSlot {
    // The peer and its firmware station.
    uint8_t station = kFirstPeerStation;
    int failedGen = -1;          // a firmware command failed for this slot's station in this generation
    uint8_t addr[6] = {};
    bool added = false;          // the station exists in the firmware
    bool accepted = false;       // the peer accepted our send agreement
    uint32_t epoch = 0;          // bumped on every add, so work for an older station is recognised
    uint32_t width = 0;          // 80 or 20, the width the station was added with (0 before the first)
    uint64_t lastUS = 0;         // last heard from or sent to
    // Receive Block Ack: requests waiting, sessions by TID, and our answer.
    PendingAddbaRequests requests;
    uint8_t rxTids = 0, rxTidTeardown = 0, rxIdleTids = 0;
    uint8_t rxTidToken[8] = {};
    uint64_t rxTidLastUS[8] = {};
    uint8_t answer[kAddbaResponseLength] = {};
    size_t answerLength = 0;
    uint8_t answerTid = 0;
    uint64_t answerRetryUS = 0;
    // Our DELBA for an idle receive session, keyed by firmware generation, peer epoch and TID.
    int delbaWaitTid = -1, delbaWaitGen = -1;
    uint32_t delbaWaitEpoch = 0, delbaTries = 0;
    uint16_t delbaAttemptId = 0;
    uint8_t delbaResult = 0;     // 0 none or pending, 1 acknowledged, 2 failed
    uint64_t delbaDeadlineUS = 0, delbaNextTryUS = 0;
    // Send Block Ack: agreement state, aggregate queue and its sequence.
    int txState = 0;             // kCoexBaTxOff
    int txQueue = -1;
    uint16_t txSeq = 0, txWindow = 0;
    uint8_t txToken = 0x5a;
    BaTxBulkState txBulk{};
    uint64_t txLastAskUS = 0, txRestUntilUS = 0, txLastAggUS = 0, txLastNotifUS = 0;
    bool queueAllocPending = false;
    bool probeTried = false;
    uint64_t probeUntilUS = 0, lastRxPayloadUS = 0;
    // Pausing the aggregate queue while the peer is away, and its confirmed resume.
    PeerPauseState pause{};
    PeerBulkState bulk{};
    bool resumePending = false;
    uint64_t resumeRetryUS = 0;
    uint32_t resumeFailStreak = 0;
};

inline bool samePeer(const uint8_t *a, const uint8_t *b) { return a && b && !memcmp(a, b, 6); }

// The slot whose station exists for `peer`, among the first `usable`.
inline PeerSlot *addedSlotFor(PeerSlot *slots, unsigned usable, const uint8_t *peer) {
    for (unsigned i = 0; i < usable; ++i)
        if (slots[i].added && samePeer(slots[i].addr, peer))
            return &slots[i];
    return nullptr;
}

// Where an ADDBA request from `peer` waits. With one slot, the rule from before several slots: the slot, unless
// its station is another peer's (the slot's own queue then refuses a request while it holds another peer's).
// With more: the peer's own slot (station added, or requests waiting), else the first free slot that hasn't
// failed in this generation. Null when there is none.
inline PeerSlot *slotForRequest(PeerSlot *slots, unsigned usable, const uint8_t *peer, int generation) {
    if (!usable)
        return nullptr;
    if (usable == 1)
        return slots[0].added && !samePeer(slots[0].addr, peer) ? nullptr : &slots[0];
    for (unsigned i = 0; i < usable; ++i) {
        const PeerSlot &s = slots[i];
        if ((s.added && samePeer(s.addr, peer)) || (!s.added && samePeer(s.requests.peer(), peer)))
            return &slots[i];
    }
    for (unsigned i = 0; i < usable; ++i) {
        PeerSlot &s = slots[i];
        if (!s.added && s.requests.empty() && (i == 0 || s.failedGen != generation))
            return &s;
    }
    return nullptr;
}

// How many slots a listener can use: one for each firmware station from kFirstPeerStation up, at most
// kMaxPeerSlots. `firmwareStations` is the firmware's station count (0 when it doesn't say: 16).
inline unsigned usablePeerSlots(bool several, unsigned firmwareStations) {
    if (!several)
        return 1;
    const unsigned stations = firmwareStations ? firmwareStations : 16;
    if (stations <= kFirstPeerStation + 1)
        return 1;
    const unsigned n = stations - kFirstPeerStation;
    return n < kMaxPeerSlots ? n : kMaxPeerSlots;
}
}
