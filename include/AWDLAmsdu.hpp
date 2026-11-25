// A-MSDU sending for AWDL data (-itlawdlamsdutx): several of awdl0's data frames for the same peer packed into one
// QoS data MPDU inside an A-MPDU, as Apple devices send to us (0.7.0-alpha.3's -itlawdlamsdurx measured them).
// Pure helpers, host-tested; the driver decides when a frame may join (negotiated, firmware-enabled, same peer/TID,
// within every limit) and falls back to single MSDUs otherwise.
#ifndef AWDLAmsdu_hpp
#define AWDLAmsdu_hpp

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace ItlAWDL {

constexpr size_t kAmsduSubHeader = 14;        // DA, SA, length (big-endian)
constexpr size_t kAmsduQosHeader = 26;        // 802.11 data header + QoS control
constexpr size_t kAmsduFcs = 4;
// The largest MPDU every VHT device must accept (IEEE 802.11 VHT capabilities, maximum MPDU length 0). A-MSDUs inside
// an A-MPDU are bounded by the MPDU length; a larger value needs the peer's advertised capability.
constexpr size_t kAmsduBaselineMpdu = 3895;
// Two MSDUs per A-MSDU to start (the Windows-driver study's cautious first step; Intel's own builder allows more).
constexpr unsigned kAmsduMaxSubframes = 2;

inline size_t amsduPad(size_t subframeLength) { return (4 - (subframeLength & 3)) & 3; }

// Whether `f` (one of awdl0's frames, plain no-DS data: 24-byte header, type 0x08, unicast) can be packed at all.
inline bool amsduCandidate(const uint8_t *f, size_t length) {
    return f && length > 24 && f[0] == 0x08 && !(f[1] & 0x03) && !(f[4] & 1);
}

// Whether `next` may join an A-MSDU that starts with `first`: the same receiver, transmitter and BSSID (one MPDU
// header describes them all). Ordering is the caller's: only the frame that would be sent next may join.
inline bool amsduSameHeader(const uint8_t *first, const uint8_t *next) {
    return first && next && !memcmp(first + 4, next + 4, 18);
}

// The MPDU length an A-MSDU of `count` frames with these lengths (each including its 24-byte header) would have,
// FCS included. 0 if any length is too short.
inline size_t amsduMpduLength(const size_t *lengths, unsigned count) {
    size_t total = kAmsduQosHeader;
    for (unsigned i = 0; i < count; ++i) {
        if (lengths[i] <= 24) return 0;
        const size_t sub = kAmsduSubHeader + (lengths[i] - 24);
        total += sub + (i + 1 < count ? amsduPad(sub) : 0);
    }
    return total + kAmsduFcs;
}

// The A-MSDU's own length (its subframes and padding: what the firmware's amsdu_size bounds). 0 if invalid.
inline size_t amsduBodyLength(const size_t *lengths, unsigned count) {
    const size_t mpdu = amsduMpduLength(lengths, count);
    return mpdu ? mpdu - kAmsduQosHeader - kAmsduFcs : 0;
}

// Limits for one peer and TID. All must hold: the peer agreed (ADDBA answer bit 0 to our offer), the firmware's rate
// control enabled A-MSDU on that TID (TLC update, amsdu_enabled) with a size (amsdu_size), the MPDU fits the peer's
// maximum MPDU length and our TX buffer, and the subframe count stays within kAmsduMaxSubframes.
struct AmsduLimits {
    bool peerAgreed;
    uint32_t firmwareTids;     // amsdu_enabled bitmap from the last TLC update for this station
    uint32_t firmwareSize;     // amsdu_size from that update (bytes of subframes)
    size_t maxMpdu;            // the peer's maximum MPDU length (kAmsduBaselineMpdu unless known larger)
    size_t txBuffer;           // our DMA buffer per TX slot, header included
};
inline bool amsduAllowed(const AmsduLimits &l, uint8_t tid, const size_t *lengths, unsigned count) {
    if (count < 2 || count > kAmsduMaxSubframes || tid >= 8 || !l.peerAgreed || !(l.firmwareTids & (1U << tid)) ||
        !l.firmwareSize)
        return false;
    const size_t mpdu = amsduMpduLength(lengths, count);
    if (!mpdu)
        return false;
    const size_t body = mpdu - kAmsduQosHeader - kAmsduFcs;
    return body <= l.firmwareSize && mpdu <= l.maxMpdu && mpdu - kAmsduFcs <= l.txBuffer;
}

// Builds one QoS data MPDU carrying `count` frames as A-MSDU subframes: the first frame's header with QoS control
// (TID, A-MSDU present bit 7), then per frame DA (addr1), SA (addr2), body length (big-endian) and its body, each but
// the last padded to 4 bytes with zeros. Returns the length without FCS, or 0 (nothing usable written) if a frame
// isn't a candidate, the headers differ, or it doesn't fit `capacity`.
inline size_t buildAmsdu(const uint8_t *const *frames, const size_t *lengths, unsigned count, uint8_t tid,
                         uint16_t sequence, uint8_t *out, size_t capacity) {
    if (!frames || !lengths || !out || count < 1 || count > 8 || tid >= 8)
        return 0;
    for (unsigned i = 0; i < count; ++i)
        if (!amsduCandidate(frames[i], lengths[i]) || !amsduSameHeader(frames[0], frames[i]))
            return 0;
    const size_t mpdu = amsduMpduLength(lengths, count);
    if (!mpdu || mpdu - kAmsduFcs > capacity)
        return 0;
    memcpy(out, frames[0], 24);
    out[0] = 0x88;
    const uint16_t sequenceControl = uint16_t((sequence & 0xfff) << 4);
    out[22] = uint8_t(sequenceControl);
    out[23] = uint8_t(sequenceControl >> 8);
    out[24] = uint8_t((tid & 7U) | 0x80U);
    out[25] = 0;
    size_t at = kAmsduQosHeader;
    for (unsigned i = 0; i < count; ++i) {
        const size_t body = lengths[i] - 24;
        memcpy(out + at, frames[i] + 4, 6);        // DA = addr1 (no-DS)
        memcpy(out + at + 6, frames[i] + 10, 6);   // SA = addr2
        out[at + 12] = uint8_t(body >> 8);
        out[at + 13] = uint8_t(body);
        memcpy(out + at + kAmsduSubHeader, frames[i] + 24, body);
        at += kAmsduSubHeader + body;
        if (i + 1 < count) {
            const size_t pad = amsduPad(kAmsduSubHeader + body);
            memset(out + at, 0, pad);
            at += pad;
        }
    }
    return at;
}

// The A-MSDU part of a TLC update for one station (iwx_tlc_update_notif, flags bit 1). The rate part (bit 0) is
// separate: an update may carry either or both.
struct AmsduTlcState {
    bool known;
    uint32_t size, tids, updates;
};
constexpr uint32_t kTlcNotifRate = 1U << 0, kTlcNotifAmsdu = 1U << 1;
inline bool amsduTlcNote(AmsduTlcState &st, uint32_t flags, uint32_t size, uint32_t tids) {
    if (!(flags & kTlcNotifAmsdu))
        return false;
    st.known = true;
    st.size = tids ? size : 0;
    st.tids = st.size ? (tids & 0xffU) : 0;
    ++st.updates;
    return true;
}

}  // namespace ItlAWDL

#endif
