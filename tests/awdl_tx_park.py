"""The real popTransmit with its queues mocked: frames held for one peer wait in their order while others go."""
from pathlib import Path
import subprocess, tempfile
root = Path(__file__).resolve().parents[1]
source = (root / 'ForbiddenItlwm/AirportItlwmV2.cpp').read_text()
a = source.index('bool ItlAWDLInterface::popTransmit(')
b = source.index('\nbool ItlAWDLInterface::serviceTransmit()', a)
program = r'''
#include <AWDLDataFrame.hpp>
#include <AWDLPeerProbe.hpp>
#include <AWDLObservation.hpp>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <vector>
static unsigned max(unsigned x, unsigned y) { return x > y ? x : y; }
struct ItlAWDLInterface {
    struct TransmitFrame { uint16_t length; uint8_t bytes[1514]; };
    unsigned transmitFrameCount = 8, parkedFrameCount = 4;
    TransmitFrame transmitStore[8], parkedStore[4];
    TransmitFrame *transmitFrames = transmitStore, *parkedFrames = parkedStore;
    unsigned transmitHead = 0, transmitQueued = 0, transmitDropped = 0, transmitHeldShort = 0;
    unsigned transmitPopped = 0, transmitHeldMark = 0;
    unsigned parkedHead = 0, parkedQueued = 0, parkedPeak = 0, transmitParked = 0, transmitPassed = 0;
    uint8_t parkedPeer[6] = {};
    uint32_t transmitDepthBins[9] = {};
    uint16_t transmitSequence = 0;
    uint8_t localFrameAddress[6] = {2, 0, 0, 0, 0, 1};
    bool popTransmit(uint8_t *, size_t, size_t *, const uint8_t *holdPayloadFor, bool holdEverySize = false);
    bool popTransmitFor(uint8_t *, size_t, size_t *, const uint8_t *destination, size_t maxLength);
    // An IPv6 frame from us to `to`, `size` bytes of payload, its id in the payload.
    void push(const uint8_t *to, uint8_t id, size_t size) {
        assert(transmitQueued < transmitFrameCount);
        auto &f = transmitFrames[(transmitHead + transmitQueued++) % transmitFrameCount];
        memset(f.bytes, 0, sizeof(f.bytes));
        memcpy(f.bytes, to, 6); memcpy(f.bytes + 6, localFrameAddress, 6);
        f.bytes[12] = 0x86; f.bytes[13] = 0xdd;
        uint8_t *ip = f.bytes + 14;
        ip[0] = 0x60; ip[4] = uint8_t(size >> 8); ip[5] = uint8_t(size); ip[6] = 17; ip[7] = 64;
        ip[8] = 0xfe; ip[9] = 0x80; ip[24] = 0xfe; ip[25] = 0x80;
        memcpy(ip + 40, "PARK", 4); ip[44] = id;
        f.length = uint16_t(14 + 40 + size);
    }
};
'''
program += source[a:b]
program += r'''
static const uint8_t P[6] = {6, 1, 1, 1, 1, 1}, Q[6] = {6, 2, 2, 2, 2, 2}, G[6] = {0x33, 0x33, 0, 0, 0, 1};
static uint8_t out[2048];
// The id of the frame popped, or -1 for none, and its 802.11 sequence number.
static int pop(ItlAWDLInterface &t, const uint8_t *hold, bool every = false, unsigned *sequence = nullptr) {
    size_t written = 0;
    if (!t.popTransmit(out, sizeof(out), &written, hold, every)) return -1;
    if (sequence) *sequence = ItlAWDL::read16(out + 22) >> 4;
    for (size_t i = 0; i + 5 <= written; ++i)
        if (!memcmp(out + i, "PARK", 4)) return out[i + 4];
    assert(!"popped frame has no id");
    return -2;
}
int main() {
    {   // No hold: the FIFO as it was.
        ItlAWDLInterface t;
        t.push(P, 1, 1300); t.push(Q, 2, 100); t.push(G, 3, 100);
        assert(pop(t, nullptr) == 1 && pop(t, nullptr) == 2 && pop(t, nullptr) == 3 && pop(t, nullptr) == -1);
        assert(!t.transmitParked && t.transmitPopped == 3);
    }
    {   // P held: its payload frames and every later frame of its own wait, in order; Q's and the group's go.
        ItlAWDLInterface t;
        t.push(P, 1, 1300); t.push(P, 2, 1300); t.push(Q, 3, 1300); t.push(P, 4, 100); t.push(G, 5, 100);
        t.push(Q, 6, 100);
        unsigned s1, s2, s3;
        assert(pop(t, P, false, &s1) == 3 && pop(t, P, false, &s2) == 5 && pop(t, P, false, &s3) == 6);
        assert(s2 == s1 + 1 && s3 == s2 + 1);
        assert(pop(t, P) == -1 && t.parkedQueued == 3 && t.transmitParked == 3 && t.transmitPassed == 3);
        assert(t.transmitHeldShort == 1 && t.parkedPeak == 3);
        // P's new frames join the parked ones while it is still held, even short ones.
        t.push(P, 7, 50);
        assert(pop(t, P) == -1 && t.parkedQueued == 4);
        // Released: P's frames in their order, numbered after what already went.
        unsigned s;
        assert(pop(t, nullptr, false, &s) == 1 && s == s3 + 1);
        assert(pop(t, nullptr) == 2 && pop(t, nullptr) == 4 && pop(t, nullptr) == 7 && pop(t, nullptr) == -1);
        assert(!t.parkedQueued && !t.transmitQueued && t.transmitPopped == 7 && !t.transmitDropped);
    }
    {   // A short frame for P at the head goes while nothing of P's is parked and every size isn't held.
        ItlAWDLInterface t;
        t.push(P, 1, 100); t.push(Q, 2, 100);
        assert(pop(t, P) == 1 && pop(t, P) == 2 && !t.transmitParked);
        // With holdEverySize it is held too.
        t.push(P, 3, 100); t.push(Q, 4, 100);
        assert(pop(t, P, true) == 4 && t.parkedQueued == 1 && pop(t, nullptr) == 3);
    }
    {   // The hold moves from P to Q: P's parked frames leave first, then Q's are parked behind nothing.
        ItlAWDLInterface t;
        t.push(P, 1, 1300); t.push(Q, 2, 1300);
        assert(pop(t, P) == 2 && t.parkedQueued == 1);
        t.push(Q, 3, 1300); t.push(P, 4, 1300);
        assert(pop(t, Q) == 1);            // P's parked frame first: P is no longer held.
        assert(pop(t, Q) == 4);            // Q's frame is parked, P's goes.
        assert(pop(t, Q) == -1 && t.parkedQueued == 1 && !memcmp(t.parkedPeer, Q, 6));
        assert(pop(t, nullptr) == 3 && pop(t, nullptr) == -1);
    }
    {   // The parking is full: the head waits with what is behind it, as before, and nothing is lost.
        ItlAWDLInterface t;
        for (uint8_t i = 1; i <= 5; ++i) t.push(P, i, 1300);
        t.push(Q, 6, 100);
        assert(pop(t, P) == -1 && t.parkedQueued == 4 && t.transmitQueued == 2);
        assert(pop(t, P) == -1 && t.transmitQueued == 2);
        for (uint8_t i = 1; i <= 6; ++i) assert(pop(t, nullptr) == i);
        assert(pop(t, nullptr) == -1 && !t.transmitDropped);
    }
    {   // Without parking storage, the old behaviour exactly.
        ItlAWDLInterface t; t.parkedFrames = nullptr;
        t.push(P, 1, 1300); t.push(Q, 2, 100);
        assert(pop(t, P) == -1 && t.transmitQueued == 2 && pop(t, nullptr) == 1 && pop(t, nullptr) == 2);
    }
    {   // The parking ring wraps.
        ItlAWDLInterface t;
        for (unsigned round = 0; round < 5; ++round) {
            t.push(P, uint8_t(10 * round + 1), 1300); t.push(P, uint8_t(10 * round + 2), 1300);
            t.push(Q, uint8_t(10 * round + 3), 100);
            assert(pop(t, P) == int(10 * round + 3));
            assert(pop(t, nullptr) == int(10 * round + 1) && pop(t, nullptr) == int(10 * round + 2));
        }
    }
    {   // -itlawdlamsdutx: popTransmitFor takes the head only for that peer, within the length, with nothing parked;
        // otherwise nothing changes and popTransmit returns the same frame next.
        ItlAWDLInterface t;
        t.push(P, 1, 1300); t.push(Q, 2, 100); t.push(P, 3, 100);
        size_t w = 0;
        assert(!t.popTransmitFor(out, sizeof(out), &w, Q, 2000) && t.transmitQueued == 3);   // head is P's
        assert(!t.popTransmitFor(out, sizeof(out), &w, P, 1300 + 40) && t.transmitQueued == 3);   // too long
        const unsigned before = t.transmitSequence;
        assert(t.popTransmitFor(out, sizeof(out), &w, P, 2000) && w == 14 + 40 + 1300 + 26 && t.transmitQueued == 2);
        assert(t.transmitSequence == before + 1 && out[0] == 0x08 && !memcmp(out + 4, P, 6));
        unsigned s;
        assert(pop(t, nullptr, false, &s) == 2 && s == before + 1);   // numbering continues in order
        assert(t.popTransmitFor(out, sizeof(out), &w, P, 2000) && t.transmitQueued == 0);
        assert(!t.popTransmitFor(out, sizeof(out), &w, P, 2000));                             // empty
        // a group frame at the head is never taken
        t.push(G, 4, 100);
        assert(!t.popTransmitFor(out, sizeof(out), &w, G, 2000) && pop(t, nullptr) == 4);
        // parked frames pending: nothing taken (they come first)
        t.push(P, 5, 1300); t.push(Q, 6, 1300);
        assert(pop(t, P) == 6 && t.parkedQueued == 1);
        t.push(P, 7, 100);
        assert(!t.popTransmitFor(out, sizeof(out), &w, P, 2000) && t.transmitQueued == 1);
        assert(pop(t, nullptr) == 5 && pop(t, nullptr) == 7);
        // an invalid frame at the head is left for popTransmit, which drops it as before
        t.push(P, 8, 100); t.transmitFrames[t.transmitHead].bytes[12] = 0x08;   // not IPv6
        assert(!t.popTransmitFor(out, sizeof(out), &w, P, 2000) && t.transmitQueued == 1);
        assert(pop(t, nullptr) == -1 && t.transmitDropped == 1);
    }
    puts("held peer's frames parked in order, others pass; A-MSDU next-frame take: passed (host mocks only)");
}
'''
with tempfile.TemporaryDirectory(prefix='awdl-tx-park-') as directory:
    p = Path(directory) / 'test.cpp'; p.write_text(program); e = Path(directory) / 'test'
    subprocess.run(['xcrun', 'clang++', '-std=c++14', '-I' + str(root / 'include'), '-fsanitize=address,undefined',
                    str(p), '-o', str(e)], check=True)
    subprocess.run([str(e)], check=True)
