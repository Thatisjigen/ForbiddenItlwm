// Host checks for the per-antenna receive statistics (AWDLChainRssi.hpp). No firmware or radio.
#include <AWDLChainRssi.hpp>
#include <cassert>
#include <cstdio>
using namespace ItlAWDL;

int main() {
    {
        ChainRssi c;
        c.note(0, 0);                         // nothing measured: not counted
        assert(c.frames == 0 && c.meanA() == 0 && c.meanB() == 0);
        c.note(40, 66);                       // A -40, B -66: A 26 dB ahead
        c.note(42, 64);
        assert(c.frames == 2 && c.nA == 2 && c.nB == 2);
        assert(c.meanA() == -41 && c.meanB() == -65);
        assert(c.aAhead10 == 2 && c.bAhead10 == 0);
        assert(c.minA == -42 && c.maxA == -40 && c.minB == -66 && c.maxB == -64);
    }
    {
        ChainRssi c;
        c.note(50, 0);                        // only chain A
        c.note(0, 55);                        // only chain B
        c.note(50, 52);                       // within 10 dB
        assert(c.frames == 3 && c.onlyA == 1 && c.onlyB == 1);
        assert(c.aAhead10 == 0 && c.bAhead10 == 0);
        assert(c.meanA() == -50 && c.meanB() == -53);
    }
    puts("AWDL per-antenna receive statistics: passed (host only)");
}
