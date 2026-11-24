// Host checks for the per-transmitter duplicate state (AWDLRxDup.hpp), with the receive path's duplicate rule
// (iwx_detect_duplicate's: a retry with the last sequence number of its TID is a duplicate). No firmware or radio.
#include <AWDLRxDup.hpp>
#include <cassert>
#include <cstdio>
using namespace ItlAWDL;

struct Dup {
    uint16_t last_seq[9];
    uint8_t last_sub_frame[9];
};

// iwx_detect_duplicate's rule for one non-A-MSDU frame.
static bool duplicate(Dup *d, unsigned tid, uint16_t seq, bool retry) {
    if (retry && d->last_seq[tid] == seq && d->last_sub_frame[tid] >= 0)
        return true;
    d->last_seq[tid] = seq;
    d->last_sub_frame[tid] = 0;
    return false;
}

int main() {
    unsigned cases = 0;
    const uint8_t a[6] = {2, 0, 0, 0, 0, 1}, b[6] = {2, 0, 0, 0, 0, 2};
    {
        // The review's case: A sends TID 0 SN 7; B's first copy of its own SN 7 is a retry. With one shared record
        // B's frame was dropped; with one record each it passes.
        Dup shared{};
        assert(!duplicate(&shared, 0, 7, false));
        assert(duplicate(&shared, 0, 7, true));                                // the old bug, reproduced
        TransmitterStates<Dup, 4> t;
        assert(!duplicate(t.stateFor(a), 0, 7, false));
        assert(!duplicate(t.stateFor(b), 0, 7, true));
        // A's real retry is still caught, after B's frames in between.
        assert(!duplicate(t.stateFor(b), 0, 8, false));
        assert(duplicate(t.stateFor(a), 0, 7, true));
        cases += 5;
    }
    {
        // Least recently used record goes; a new owner starts clear; the count of replacements.
        TransmitterStates<Dup, 2> t;
        Dup *sa = t.stateFor(a);
        sa->last_seq[0] = 99;
        (void)t.stateFor(b);
        (void)t.stateFor(a);                                                   // a is now the newest
        const uint8_t c[6] = {2, 0, 0, 0, 0, 3};
        Dup *sc = t.stateFor(c);                                               // replaces b
        assert(t.replaced == 1 && sc->last_seq[0] == 0);
        assert(t.stateFor(a) == sa && sa->last_seq[0] == 99);                  // a kept its record
        Dup *sb = t.stateFor(b);                                               // b comes back clear, replacing c
        assert(sb->last_seq[0] == 0 && t.replaced == 2);
        t.clear();
        assert(t.stateFor(a)->last_seq[0] == 0);
        cases += 4;
    }
    printf("AWDL rx duplicate state: %u cases passed (host only)\n", cases);
}
