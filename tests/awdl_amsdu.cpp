// A-MSDU sending helpers (-itlawdlamsdutx): framing, limits, negotiation bits and TLC state. Host only.
#include <cassert>
#include <cstdio>
#include <cstring>
#include "AWDLAmsdu.hpp"
#include "AWDLBlockAck.hpp"

using namespace ItlAWDL;

static size_t frame(uint8_t *f, const uint8_t *da, const uint8_t *sa, size_t body, uint8_t fill) {
    static const uint8_t bssid[6] = {0x00, 0x25, 0x00, 0xff, 0x94, 0x73};
    memset(f, 0, 24);
    f[0] = 0x08;
    memcpy(f + 4, da, 6);
    memcpy(f + 10, sa, 6);
    memcpy(f + 16, bssid, 6);
    for (size_t i = 0; i < body; ++i) f[24 + i] = uint8_t(fill + i);
    return 24 + body;
}

int main() {
    unsigned cases = 0;
    static const uint8_t peer[6] = {0x2a, 1, 2, 3, 4, 5}, own[6] = {0x3a, 9, 8, 7, 6, 5}, other[6] = {0x2a, 1, 2, 3, 4, 6};
    static uint8_t a[2048], b[2048], c[2048], out[8192];
    // Framing: two full-size frames, padding between, none after the last; QoS control with the A-MSDU bit.
    {
        const size_t la = frame(a, peer, own, 1481, 0x10), lb = frame(b, peer, own, 1300, 0x80);
        const uint8_t *fs[2] = {a, b};
        const size_t ls[2] = {la, lb};
        const size_t n = buildAmsdu(fs, ls, 2, 0, 0x123, out, sizeof(out));
        const size_t sub1 = 14 + 1481, pad1 = amsduPad(sub1);
        assert(pad1 == 1 && n == 26 + sub1 + pad1 + 14 + 1300);
        assert(out[0] == 0x88 && !memcmp(out + 4, a + 4, 18) && out[24] == 0x80 && out[25] == 0);
        assert(((out[22] | out[23] << 8) >> 4) == 0x123);
        assert(!memcmp(out + 26, peer, 6) && !memcmp(out + 32, own, 6) && out[38] == (1481 >> 8) && out[39] == (1481 & 0xff));
        assert(!memcmp(out + 40, a + 24, 1481) && out[40 + 1481] == 0);
        const size_t s2 = 26 + sub1 + pad1;
        assert(!memcmp(out + s2, peer, 6) && out[s2 + 12] == (1300 >> 8) && out[s2 + 13] == (1300 & 0xff));
        assert(!memcmp(out + s2 + 14, b + 24, 1300));
        assert(amsduMpduLength(ls, 2) == n + 4 && amsduBodyLength(ls, 2) == n - 26);
        assert(buildAmsdu(fs, ls, 1, 5, 7, out, sizeof(out)) == 26 + 14 + 1481 && out[24] == 0x85);   // one subframe
        cases += 3;
    }
    // Refusals: different peer, group, QoS/ToDS frames, too small capacity, bad TID; nothing partial matters.
    {
        const size_t la = frame(a, peer, own, 200, 1), lb = frame(b, other, own, 200, 2);
        const uint8_t *fs[2] = {a, b};
        const size_t ls[2] = {la, lb};
        assert(!buildAmsdu(fs, ls, 2, 0, 0, out, sizeof(out)));          // another receiver
        frame(b, peer, own, 200, 2);
        b[4] = 0x01; assert(!buildAmsdu(fs, ls, 2, 0, 0, out, sizeof(out))); b[4] = peer[0];   // group
        b[0] = 0x88; assert(!buildAmsdu(fs, ls, 2, 0, 0, out, sizeof(out))); b[0] = 0x08;      // already QoS
        b[1] = 0x01; assert(!buildAmsdu(fs, ls, 2, 0, 0, out, sizeof(out))); b[1] = 0;         // ToDS
        assert(buildAmsdu(fs, ls, 2, 0, 0, out, sizeof(out)));
        assert(!buildAmsdu(fs, ls, 2, 0, 0, out, amsduMpduLength(ls, 2) - 5));               // one byte short
        assert(buildAmsdu(fs, ls, 2, 0, 0, out, amsduMpduLength(ls, 2) - 4));
        assert(!buildAmsdu(fs, ls, 2, 8, 0, out, sizeof(out)));
        const size_t shortLen[2] = {24, lb};
        assert(!amsduMpduLength(shortLen, 2) && !buildAmsdu(fs, shortLen, 2, 0, 0, out, sizeof(out)));
        cases += 9;
    }
    // Limits: every one must hold.
    {
        const size_t la = frame(a, peer, own, 1481, 0), lb = frame(b, peer, own, 1481, 0), lc = frame(c, peer, own, 100, 0);
        const size_t two[2] = {la, lb}, three[3] = {la, lb, lc}, small[2] = {lc, lc};
        AmsduLimits l{true, 1U, 3839, kAmsduBaselineMpdu, 4096};
        assert(amsduMpduLength(two, 2) == 26 + 1495 + 1 + 1495 + 4);   // 3021: two full frames fit the VHT baseline
        assert(amsduAllowed(l, 0, two, 2));
        assert(!amsduAllowed(l, 0, three, 3));                          // more than two subframes
        assert(!amsduAllowed(l, 0, two, 1));                            // one frame isn't an A-MSDU
        assert(!amsduAllowed(l, 1, two, 2));                            // firmware didn't enable TID 1
        l.peerAgreed = false; assert(!amsduAllowed(l, 0, two, 2)); l.peerAgreed = true;
        l.firmwareSize = 2000; assert(!amsduAllowed(l, 0, two, 2) && amsduAllowed(l, 0, small, 2)); l.firmwareSize = 3839;
        l.firmwareSize = 0; assert(!amsduAllowed(l, 0, small, 2)); l.firmwareSize = 3839;
        l.txBuffer = 2048; assert(!amsduAllowed(l, 0, two, 2) && amsduAllowed(l, 0, small, 2)); l.txBuffer = 4096;
        l.maxMpdu = 3000; assert(!amsduAllowed(l, 0, two, 2)); l.maxMpdu = kAmsduBaselineMpdu;
        assert(!amsduAllowed(l, 8, two, 2));
        cases += 10;
    }
    // ADDBA: the offer bit, off by default (byte-identical), and the answer's bit.
    {
        static const uint8_t bssid[6] = {0x00, 0x25, 0x00, 0xff, 0x94, 0x73};
        uint8_t q0[40], q1[40];
        assert(buildAddbaRequest(q0, sizeof(q0), peer, own, bssid, 7, 0, 64, 10, 3) == 33);
        assert(buildAddbaRequest(q1, sizeof(q1), peer, own, bssid, 7, 0, 64, 10, 3, true) == 33);
        assert(!(q0[27] & 1) && (q1[27] & 1) && q0[27] == (q1[27] & ~1) && !memcmp(q0 + 28, q1 + 28, 5));
        uint8_t r[33] = {};
        r[0] = 0xd0; memcpy(r + 4, own, 6); memcpy(r + 10, peer, 6); r[24] = 3; r[25] = 1; r[26] = 7;
        const uint16_t params = uint16_t(1U | 2U | 0U << 2 | 64U << 6);
        r[29] = uint8_t(params); r[30] = uint8_t(params >> 8);
        AddbaAnswer ans{};
        assert(parseAddbaAnswer(r, 33, own, &ans) && ans.amsdu && ans.immediate && ans.window == 64);
        r[29] &= uint8_t(~1U);
        assert(parseAddbaAnswer(r, 33, own, &ans) && !ans.amsdu);
        cases += 4;
    }
    // TLC: A-MSDU updates are kept apart from rate updates; a zero size or no TIDs means off.
    {
        AmsduTlcState st{};
        assert(!amsduTlcNote(st, kTlcNotifRate, 3839, 1) && !st.known);
        assert(amsduTlcNote(st, kTlcNotifRate | kTlcNotifAmsdu, 3839, 0x1) && st.known && st.size == 3839 && st.tids == 1);
        assert(amsduTlcNote(st, kTlcNotifAmsdu, 0, 0xff) && st.size == 0 && st.tids == 0);
        assert(amsduTlcNote(st, kTlcNotifAmsdu, 7935, 0) && st.size == 0 && st.tids == 0 && st.updates == 3);
        assert(amsduTlcNote(st, kTlcNotifAmsdu, 7935, 0x1ff) && st.tids == 0xff);
        cases += 5;
    }
    // Room for the next frame, and the well-formed check on what buildAmsdu produces.
    {
        const size_t la = frame(a, peer, own, 1481, 3);
        const size_t one[1] = {la};
        AmsduLimits l{true, 1U, 7935, kAmsduBaselineMpdu, 4096};          // the peer's MPDU length binds
        const size_t room = amsduRoomForNext(l, 0, one, 1);
        size_t two[2] = {la, room};
        assert(room > 24 && amsduAllowed(l, 0, two, 2) && amsduMpduLength(two, 2) == kAmsduBaselineMpdu);
        two[1] = room + 1;
        assert(!amsduAllowed(l, 0, two, 2));                              // one byte more doesn't fit
        l.firmwareSize = 2600;                                          // the firmware's size binds instead
        const size_t room2 = amsduRoomForNext(l, 0, one, 1);
        two[1] = room2; assert(amsduAllowed(l, 0, two, 2) && amsduBodyLength(two, 2) == 2600);
        two[1] = room2 + 1; assert(!amsduAllowed(l, 0, two, 2));
        l.firmwareSize = 1400; assert(!amsduRoomForNext(l, 0, one, 1)); l.firmwareSize = 3839;   // no room left
        l.txBuffer = 3000;                                              // our buffer binds
        const size_t room3 = amsduRoomForNext(l, 0, one, 1);
        two[1] = room3; assert(amsduAllowed(l, 0, two, 2) && amsduMpduLength(two, 2) - 4 == 3000);
        two[1] = room3 + 1; assert(!amsduAllowed(l, 0, two, 2)); l.txBuffer = 4096;
        const size_t twoFull[2] = {la, la};
        assert(!amsduRoomForNext(l, 0, twoFull, 2));                     // already two subframes
        l.peerAgreed = false; assert(!amsduRoomForNext(l, 0, one, 1)); l.peerAgreed = true;
        assert(!amsduRoomForNext(l, 3, one, 1));
        // well-formed: what we build passes; damage of each kind fails
        const size_t lb = frame(b, peer, own, 777, 9);
        const uint8_t *fs[2] = {a, b};
        const size_t ls[2] = {la, lb};
        const size_t n = buildAmsdu(fs, ls, 2, 0, 1, out, sizeof(out));
        assert(n && amsduWellFormed(out, n, 2) && !amsduWellFormed(out, n, 1));
        assert(!amsduWellFormed(out, n - 1, 2) && !amsduWellFormed(out, n + 1, 2));
        out[24] &= 0x7f; assert(!amsduWellFormed(out, n, 2)); out[24] |= 0x80;
        out[26] ^= 1; assert(!amsduWellFormed(out, n, 2)); out[26] ^= 1;      // subframe DA differs
        out[39] ^= 0x10; assert(!amsduWellFormed(out, n, 2)); out[39] ^= 0x10;   // first length wrong
        assert(amsduWellFormed(out, n, 2));
        const size_t single = buildAmsdu(fs, ls, 1, 0, 1, out, sizeof(out));
        assert(single && !amsduWellFormed(out, single, 2));                // one subframe isn't one of ours
        uint8_t plain[200]; const size_t lp = frame(plain, peer, own, 100, 0);
        assert(!amsduWellFormed(plain, lp, 2));
        cases += 16;
    }
    printf("AWDL A-MSDU send helpers: %u cases passed (host only)\n", cases);
    return 0;
}
