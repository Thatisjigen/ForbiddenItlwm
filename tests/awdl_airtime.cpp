// Host checks for the airtime accounting (AWDLAirtime.hpp): state order, interval crediting, the bulk account, the
// gap cap and restarts. No firmware or radio.
#include <AWDLAirtime.hpp>
#include <cassert>
#include <cstdio>
using namespace ItlAWDL;

static AirtimeSample S(bool local, bool peer, bool host, bool policy, bool fw) {
    AirtimeSample s;
    s.localWindow = local;
    s.peerAvailable = peer;
    s.hostReady = host;
    s.policyAllows = policy;
    s.firmwareQueued = fw;
    return s;
}

int main() {
    unsigned cases = 0;
    // Order: not on the channel wins over everything, then the peer away, then demand, then policy.
    assert(airtimeClassify(S(false, true, true, true, true)) == kAirtimeNoAirtime), ++cases;
    assert(airtimeClassify(S(true, false, true, true, true)) == kAirtimePeerAway), ++cases;
    assert(airtimeClassify(S(true, true, false, true, false)) == kAirtimeNoDemand), ++cases;
    assert(airtimeClassify(S(true, true, false, false, false)) == kAirtimeNoDemand), ++cases;   // nothing to block
    assert(airtimeClassify(S(true, true, true, false, false)) == kAirtimePolicyBlocked), ++cases;
    assert(airtimeClassify(S(true, true, true, true, false)) == kAirtimeEmptyWithReady), ++cases;
    assert(airtimeClassify(S(true, true, true, true, true)) == kAirtimeQueued), ++cases;
    assert(airtimeClassify(S(true, true, false, true, true)) == kAirtimeQueued), ++cases;   // fw busy, host empty
    assert(airtimeClassify(S(true, true, true, false, true)) == kAirtimeQueued), ++cases;   // held, fw still busy

    // Each sample opens the interval up to the next one.
    AirtimeMeter m;
    assert(m.sample(1000, S(true, true, true, true, true), true)), ++cases;          // queued from 1000
    assert(m.sample(4000, S(true, true, true, true, false), true)), ++cases;         // empty-ready from 4000
    assert(m.sample(5000, S(false, true, true, true, false), false)), ++cases;       // no airtime, not bulk
    assert(m.sample(7000, S(true, true, false, true, false), false)), ++cases;
    assert(m.all.us[kAirtimeQueued] == 3000 && m.all.us[kAirtimeEmptyWithReady] == 1000), ++cases;
    assert(m.all.us[kAirtimeNoAirtime] == 2000 && m.all.totalUS() == 6000), ++cases;
    // Only intervals opened in bulk count towards the bulk account.
    assert(m.bulk.us[kAirtimeQueued] == 3000 && m.bulk.us[kAirtimeEmptyWithReady] == 1000), ++cases;
    assert(m.bulk.us[kAirtimeNoAirtime] == 0 && m.bulk.totalUS() == 4000), ++cases;
    assert(m.all.samples == 4 && m.bulk.samples == 2), ++cases;
    assert(m.bulk.share(kAirtimeQueued) == 750 && m.bulk.share(kAirtimeEmptyWithReady) == 250), ++cases;
    assert(m.all.maxGapUS == 3000 && m.bulk.maxGapUS == 3000), ++cases;

    // A gap past the cap goes to "unsampled", not to the state before it.
    const uint64_t before = m.all.us[kAirtimeNoDemand];
    assert(m.sample(7000 + AirtimeMeter::kGapCapUS + 1, S(true, true, true, true, true), true)), ++cases;
    assert(m.all.us[kAirtimeNoDemand] == before && m.all.unsampledUS == AirtimeMeter::kGapCapUS + 1), ++cases;
    assert(m.bulk.unsampledUS == 0), ++cases;   // the stretch opened outside bulk
    // A gap exactly at the cap is still credited.
    const uint64_t t0 = 7000 + AirtimeMeter::kGapCapUS + 1;
    assert(m.sample(t0 + AirtimeMeter::kGapCapUS, S(true, true, true, true, true), true)), ++cases;
    assert(m.bulk.us[kAirtimeQueued] == 3000 + AirtimeMeter::kGapCapUS), ++cases;
    assert(m.bulk.maxGapUS == AirtimeMeter::kGapCapUS), ++cases;

    // A sample older than the last changes nothing.
    const AirtimeAccount snap = m.all;
    assert(!m.sample(t0, S(true, true, true, true, true), true)), ++cases;
    assert(m.all.totalUS() == snap.totalUS() && m.all.samples == snap.samples), ++cases;

    // After a restart the next sample opens a fresh interval: the time since the last one isn't credited.
    const uint64_t total = m.all.totalUS(), unsampled = m.all.unsampledUS;
    m.restart();
    assert(m.sample(t0 + AirtimeMeter::kGapCapUS + 500, S(true, false, true, true, false), true)), ++cases;
    assert(m.all.totalUS() == total && m.all.unsampledUS == unsampled), ++cases;
    // A restarted meter also accepts a sample with an earlier time (a new clock base).
    m.restart();
    assert(m.sample(10, S(true, false, true, true, false), true)), ++cases;
    assert(m.sample(110, S(true, true, true, true, true), true)), ++cases;
    assert(m.bulk.us[kAirtimePeerAway] == 100), ++cases;

    // clear() empties everything.
    m.clear();
    assert(m.all.totalUS() == 0 && m.bulk.totalUS() == 0 && m.all.samples == 0 && m.all.share(0) == 0), ++cases;
    assert(m.all.share(kAirtimeStates) == 0), ++cases;   // out of range

    printf("awdl_airtime: %u cases passed\n", cases);
    return 0;
}
