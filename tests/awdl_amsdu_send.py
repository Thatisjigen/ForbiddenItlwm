"""The real awdlCoexAmsduTry with buffers, awdl0's queue and iwx_tx mocked: packs only what fits, keeps order, never
drops or puts back a frame, and falls back to separate frames in order. Host only (-itlawdlamsdutx)."""
from pathlib import Path
import subprocess, tempfile
root = Path(__file__).resolve().parents[1]
source = (root / 'itlwm/hal_iwx/ItlIwx.cpp').read_text()
a = source.index('int ItlIwx::\nawdlCoexAmsduTry(')
b = source.index('\n}\n', a) + 3
program = r'''
#include <cassert>
#include <cstdio>
#include <cstring>
#include <vector>
#include <AWDLObservation.hpp>
#include <AWDLPeerSlot.hpp>
#define MAX(a, b) ((a) > (b) ? (a) : (b))
enum { MBUF_DONTWAIT = 0, EDCA_AC_BE = 0, kCoexBaTxAgreed = 3, kAwdlBaTxTid = 0 };
using IOReturn = int;
constexpr IOReturn kIOReturnSuccess = 0, kIOReturnNotFound = 1;
struct Packet { std::vector<uint8_t> bytes; size_t length = 0; bool freed = false; int id = 0; };
using mbuf_t = Packet *;
static std::vector<Packet *> allocated;
static Packet *make(size_t cap) { auto *p = new Packet; p->bytes.resize(cap); allocated.push_back(p); return p; }
static void *mbuf_data(mbuf_t m) { return m->bytes.data(); }
static size_t mbuf_len(mbuf_t m) { return m->length; }
static size_t mbuf_maxlen(mbuf_t m) { return m->bytes.size(); }
static mbuf_t mbuf_next(mbuf_t) { return nullptr; }
static void mbuf_setlen(mbuf_t m, size_t n) { m->length = n; }
static void mbuf_pkthdr_setlen(mbuf_t, size_t) {}
static void mbuf_freem(mbuf_t m) { assert(!m->freed); m->freed = true; }
static bool allocFail;
static int mbuf_allocpacket(int, size_t n, unsigned int *chunks, mbuf_t *m) {
    assert(chunks && *chunks == 1);
    if (allocFail) return 1;
    *m = make(n); return 0;
}
struct ieee80211_node {}; static ieee80211_node bssNode;
struct ieee80211com { ieee80211_node *ic_bss = &bssNode; };
static unsigned refs;
static ieee80211_node *ieee80211_ref_node(ieee80211_node *n) { ++refs; return n; }
static void ieee80211_release_node(ieee80211com *, ieee80211_node *) { --refs; }
struct iwx_softc { ieee80211com sc_ic; struct { unsigned queued = 0; } txq[16]; };
struct IOEthernetController {}; static IOEthernetController controller;
static ItlAWDL::PeerSlot awdlPeers[ItlAWDL::kMaxPeerSlots];
struct AwdlSendingPeer { explicit AwdlSendingPeer(const ItlAWDL::PeerSlot &) {} };
static bool awdlAmsduTx = true, awdlCoexBaSendingAgg;
static uint32_t awdlAmsduTxSent, awdlAmsduTxSubframes, awdlAmsduTxNotReady, awdlAmsduTxNoRoom, awdlAmsduTxNoNext,
    awdlAmsduTxSplit, awdlCoexBaTxFrames, awdlCoexBaTxPeakQueued, awdlPeerTxAggFrames, awdlCoexDataTx,
    awdlCoexBaTxSubmitErrors;
static uint64_t awdlCoexLastDataUS;
static uint64_t awdlUptimeUS() { return 5000; }
static constexpr size_t kAwdlAmsduTxBuffer = 4096;
// awdl0's queue: frames by id, taken only under popTransmitFor's rule (destination and length).
static std::vector<Packet *> queue;
static IOReturn itlAWDLBuildTransmitFor(IOEthernetController *, mbuf_t *p, const uint8_t *dest, size_t maxLength) {
    if (queue.empty()) return kIOReturnNotFound;
    Packet *head = queue.front();
    if (memcmp(head->bytes.data() + 4, dest, 6) || head->length > maxLength) return kIOReturnNotFound;
    queue.erase(queue.begin()); *p = head; return kIOReturnSuccess;
}
// What went out, in order: an A-MSDU (ids packed), an aggregate frame, or a single frame.
struct Sent { char kind; std::vector<int> ids; };
static std::vector<Sent> sent;
static int txFail, aggFail;
static int idOf(const uint8_t *body) { return body[0]; }   // each frame's body starts with its id
class ItlIwx {
public:
    IOEthernetController *getController() { return &controller; }
    int iwx_tx(iwx_softc *, mbuf_t m, ieee80211_node *, int, int) {
        if (txFail) { --txFail; mbuf_freem(m); return 5; }
        const uint8_t *f = m->bytes.data();
        assert(ItlAWDL::amsduWellFormed(f, m->length, 2) && awdlCoexBaSendingAgg);
        Sent s{'A', {}};
        size_t at = 26;
        while (at < m->length) {
            const size_t body = size_t(f[at + 12]) << 8 | f[at + 13];
            s.ids.push_back(idOf(f + at + 14));
            at += 14 + body; if (at < m->length) at += ItlAWDL::amsduPad(14 + body);
        }
        sent.push_back(s); mbuf_freem(m); return 0;
    }
    bool awdlCoexPeerSendAgg(iwx_softc *, ItlAWDL::PeerSlot &, mbuf_t m) {
        if (aggFail) { --aggFail; return false; }
        sent.push_back({'Q', {idOf(m->bytes.data() + 24)}}); mbuf_freem(m); return true;
    }
    bool awdlCoexPeerSendSingle(iwx_softc *, mbuf_t m) { sent.push_back({'S', {idOf(m->bytes.data() + 24)}}); mbuf_freem(m); return true; }
    int awdlCoexAmsduTry(iwx_softc *, ItlAWDL::PeerSlot &, mbuf_t);
};
'''
program += source[a:b]
program += r'''
static const uint8_t P[6] = {2, 1, 1, 1, 1, 1}, Q[6] = {2, 2, 2, 2, 2, 2}, own[6] = {2, 9, 9, 9, 9, 9};
static Packet *frame(const uint8_t *to, int id, size_t body) {
    Packet *p = make(2048);
    uint8_t *f = p->bytes.data();
    memset(f, 0, 24); f[0] = 0x08; memcpy(f + 4, to, 6); memcpy(f + 10, own, 6);
    f[16] = 0; f[17] = 0x25;
    f[24] = uint8_t(id); p->length = 24 + body; p->id = id;
    return p;
}
static ItlAWDL::PeerSlot &ready() {
    ItlAWDL::PeerSlot &p = awdlPeers[0];
    p = ItlAWDL::PeerSlot{};
    memcpy(p.addr, P, 6); p.added = true; p.txState = kCoexBaTxAgreed; p.txQueue = 3; p.txAmsduAgreed = true;
    p.amsduTlc = ItlAWDL::AmsduTlcState{true, 3839, 1, 1};
    return p;
}
static void reset() { sent.clear(); queue.clear(); txFail = aggFail = 0; allocFail = false; awdlAmsduTx = true; }
int main() {
    ItlIwx d; iwx_softc sc;
    {   // two full frames for P: one A-MSDU, both consumed, one sequence number used
        reset(); auto &p = ready(); p.txSeq = 10;
        Packet *f1 = frame(P, 1, 1481); queue.push_back(frame(P, 2, 1481));
        assert(d.awdlCoexAmsduTry(&sc, p, f1) == 1);
        assert(sent.size() == 1 && sent[0].kind == 'A' && sent[0].ids == std::vector<int>({1, 2}));
        assert(queue.empty() && f1->freed && p.txSeq == 11 && awdlAmsduTxSent == 1 && awdlAmsduTxSubframes == 2);
        assert(refs == 1);   // held by the TX slot until completion, as for every frame
        refs = 0;
    }
    {   // the next frame is another peer's: nothing taken, the caller sends the first as before
        reset(); auto &p = ready();
        Packet *f1 = frame(P, 1, 1481); queue.push_back(frame(Q, 2, 1481));
        const uint32_t noNext = awdlAmsduTxNoNext;
        assert(d.awdlCoexAmsduTry(&sc, p, f1) == 0 && !f1->freed && queue.size() == 1 && sent.empty());
        assert(awdlAmsduTxNoNext == noNext + 1);
    }
    {   // the next frame is P's but too long to fit with this one: left in the queue
        reset(); auto &p = ready(); p.amsduTlc.size = 2600;
        Packet *f1 = frame(P, 1, 1481); queue.push_back(frame(P, 2, 1481));
        assert(d.awdlCoexAmsduTry(&sc, p, f1) == 0 && queue.size() == 1 && sent.empty());
        queue.clear(); queue.push_back(frame(P, 3, 900));                  // a shorter one fits
        assert(d.awdlCoexAmsduTry(&sc, p, f1) == 1 && sent[0].ids == std::vector<int>({1, 3}));
        refs = 0;
    }
    {   // not ready: no agreement, TID not enabled, no TLC update yet, flag off, another peer's frame
        reset(); auto &p = ready(); Packet *f1 = frame(P, 1, 500); queue.push_back(frame(P, 2, 500));
        p.txAmsduAgreed = false; assert(d.awdlCoexAmsduTry(&sc, p, f1) == 0); p.txAmsduAgreed = true;
        p.amsduTlc.tids = 2; assert(d.awdlCoexAmsduTry(&sc, p, f1) == 0); p.amsduTlc.tids = 1;
        p.amsduTlc.known = false; assert(d.awdlCoexAmsduTry(&sc, p, f1) == 0); p.amsduTlc.known = true;
        p.txState = 1; assert(d.awdlCoexAmsduTry(&sc, p, f1) == 0); p.txState = kCoexBaTxAgreed;
        awdlAmsduTx = false; assert(d.awdlCoexAmsduTry(&sc, p, f1) == 0); awdlAmsduTx = true;
        Packet *fq = frame(Q, 9, 500); assert(d.awdlCoexAmsduTry(&sc, p, fq) == 0);
        assert(queue.size() == 1 && sent.empty() && !f1->freed);
    }
    {   // both taken but the A-MSDU couldn't go: each goes on its own, first then second (aggregated, else singly)
        reset(); auto &p = ready(); Packet *f1 = frame(P, 1, 1000); queue.push_back(frame(P, 2, 1000));
        txFail = 1;
        assert(d.awdlCoexAmsduTry(&sc, p, f1) == 2);
        assert(sent.size() == 2 && sent[0].kind == 'Q' && sent[0].ids[0] == 1 && sent[1].kind == 'Q' && sent[1].ids[0] == 2);
        assert(awdlCoexBaTxSubmitErrors == 1 && refs == 0 && queue.empty());
        reset(); p = ready(); f1 = frame(P, 1, 1000); queue.push_back(frame(P, 2, 1000));
        allocFail = true; aggFail = 1;                                    // no buffer; the first's copy fails too
        assert(d.awdlCoexAmsduTry(&sc, p, f1) == 2);
        assert(sent.size() == 2 && sent[0].kind == 'S' && sent[0].ids[0] == 1 && sent[1].kind == 'Q' && sent[1].ids[0] == 2);
    }
    puts("A-MSDU send: packs only what fits, keeps order, nothing dropped or put back, split fallback in order (host mocks only)");
    return 0;
}
'''
with tempfile.TemporaryDirectory(prefix='awdl-amsdu-send-') as directory:
    p = Path(directory) / 'test.cpp'; p.write_text(program); e = Path(directory) / 'test'
    subprocess.run(['xcrun', 'clang++', '-std=c++14', '-I' + str(root / 'include'), '-fsanitize=address,undefined',
                    str(p), '-o', str(e)], check=True)
    subprocess.run([str(e)], check=True)
