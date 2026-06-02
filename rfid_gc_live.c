// rfid_gc_live.c
// Dual-antenna RFID live scanner with STICKY per-EPC ownership.
//
// Physical setup this is tuned for:
//   - Two CAEN R3100C Lepton3 antennas mounted under a beer-pour drip
//     tray, 150 mm centre-to-centre.
//   - A passive RFID tag is glued to the bottom of each beer mug. When
//     a mug is placed on the tray over antenna A, the tag is ~5-7 cm
//     above A's face and ~15 cm above the opposite antenna.
//   - In practice the opposite antenna still picks the tag up via
//     leakage / coupling, and the RSSI delta between the "home" and
//     "leakage" antenna is often only 2-4 dB -- too small to arbitrate
//     reliably on RSSI alone.
//
// Goal: while a mug sits on the tray, the tag must appear continuously
// in its home antenna's slot (constant, not event-driven), and must
// NEVER appear in the other antenna's slot, even when the leakage
// antenna momentarily returns more or stronger reads.
//
// How: each EPC carries a persistent OWNER (antenna 0 or 1) across
// decision windows. Inside a 250 ms window we accumulate, per EPC, the
// per-antenna read count and max RSSI. At window close we apply:
//
//   1. If neither antenna saw the EPC this window: bump unseen counter.
//      After UNSEEN_TTL_WINDOWS empty windows in a row, drop the EPC
//      entirely (mug picked up -> slot clears).
//
//   2. If only one antenna saw it: that antenna wins this window
//      (subject to MIN_READS and RSSI_FLOOR). Owner is set if not set.
//      Owner is FLIPPED only if it had been the other antenna AND
//      this antenna got >= COUNT_DOM_FLIP reads (i.e. sustained, not a
//      single stray ping).
//
//   3. If BOTH antennas saw it this window:
//        a. If no owner yet, claim for the antenna with more reads,
//           but only when count[a] >= COUNT_DOM_FIRST * count[b]
//           AND count[a] >= MIN_READS. Otherwise leave un-owned
//           (will not be displayed) until evidence is clear.
//        b. If the current owner is the one with more reads, keep.
//        c. If the current owner is the LOSER this window, only flip
//           when count[challenger] >= COUNT_DOM_FLIP * count[owner].
//           Otherwise stay sticky -- this is the no-cross-reads guard.
//
// Once an EPC has an owner and was seen this window, it is rendered
// in the owner's slot using the max RSSI that owner observed in the
// window. If the owner did NOT see it this window but the other
// antenna did, we still render in the owner's slot using the *last*
// known owner RSSI so the line never flickers off while the mug is
// physically present.
//
// Net effect: zero cross-reads (a stray leakage read never moves
// the tag to the wrong slot) AND zero missed reads (sticky owner
// keeps the slot lit while the mug is on the tray).
//
// Output format (identical to the reference rfid_gc_live):
//
//   [S0=N S1=N] []                                  -- nothing attributed
//   [S0=N S1=N] [TX=316 mW] [(0)(-58.3) EPC,   (1)(-61.7) EPC]
//   [S0=N S1=N] [TX=316 mW] [(0)(-58.3) EPC,                                      ]
//   [S0=N S1=N] [TX=316 mW] [                                    ,   (1)(-61.7) EPC]
//
// S0 / S1 are the InventoryTag calls completed on each antenna during
// the closing window. Antenna index in YELLOW, Src0 tag EPC in GREEN,
// Src1 tag EPC in RED.
//
// Usage:
//   ./rfid_gc_live              both antennas at 316 mW (default)
//   ./rfid_gc_live <mW>         both antennas at <mW> (global power)
//   ./rfid_gc_live -h | --help  show usage

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>

#include "CAENRFIDLib_Light.h"
#include "host.h"

// Configuration
#define GC_PORT             "/dev/ttyACM0"
#define GC_BAUDRATE         921600
#define DEFAULT_POWER_MW    30            // matches working rfid_standard on this rig; pass 316 if needed
#define MIN_POWER_MW        1
#define MAX_POWER_MW        316
#define GC_WINDOW_MS        100           // align with rfid_standard line rate
#define GC_MAX_TAGS         64
#define ANTENNA_COUNT       2
#define MAX_ID_LENGTH       64

// First claim: both antennas often see the same tag with equal counts (~3
// scans/window each). Requiring 2x count dominance blocked ALL output.
// Use RSSI margin for initial owner; stay sticky on flip (3x count).
#define RSSI_FLOOR_DBM10       (-800)     // -80.0 dBm
#define RSSI_CLAIM_MARGIN_DB10 ( 15)      // 1.5 dB: prefer stronger antenna on first claim
#define MIN_READS              ( 1)
#define COUNT_DOM_FIRST        ( 2)       // optional count path when counts differ
#define COUNT_DOM_FLIP         ( 3)       // flip existing owner only with strong count dominance
#define UNSEEN_TTL_WINDOWS     ( 8)       // ~0.8 s empty before slot clears

// ANSI colours
#define GREEN  "\033[0;32m"
#define RED    "\033[0;31m"
#define YELLOW "\033[0;33m"
#define CYAN   "\033[0;36m"
#define RESET  "\033[0m"

volatile int running = 0;

// Per-EPC state. Persists across decision windows -- this is what
// gives us sticky ownership and therefore "no missed reads while
// the mug is present".
//
//   owner             : -1 until first confident attribution, then 0 or 1
//   last_owner_rssi   : last RSSI the owning antenna observed (tenths dBm)
//   wcount[ant]       : reads accumulated for this EPC inside the current
//                       window, per antenna
//   wmax_rssi[ant]    : max RSSI seen for this EPC inside the current
//                       window, per antenna (INT16_MIN = not seen)
//   unseen_windows    : consecutive windows in which neither antenna
//                       saw this EPC; cleared on any sighting
//   in_use            : false slots are available for re-use after eviction
typedef struct {
    char     epc[2 * MAX_ID_LENGTH + 1];
    int      owner;
    int16_t  last_owner_rssi;
    unsigned wcount[ANTENNA_COUNT];
    int16_t  wmax_rssi[ANTENNA_COUNT];
    int      unseen_windows;
    bool     in_use;
} TagState;

typedef struct {
    char    tag[2 * MAX_ID_LENGTH + 1];
    int     antenna;
    int16_t rssi;
} TagEntry;

static void hex_str(uint8_t *bytes, uint16_t len, char *out) {
    for (int i = 0; i < len; i++)
        sprintf(out + (i * 2), "%02X", bytes[i]);
    out[len * 2] = '\0';
}

static void usage(const char *prog) {
    fprintf(stderr,
        "Usage:\n"
        "  %s              both antennas at %d mW (default)\n"
        "  %s <mW>         both antennas at <mW> (global power)\n"
        "  %s -h | --help  show this message\n"
        "\nValid power range: %d..%d mW\n",
        prog, DEFAULT_POWER_MW, prog, prog,
        MIN_POWER_MW, MAX_POWER_MW);
}

static bool parse_power(const char *s, uint32_t *out) {
    if (s == NULL || *s == '\0') return false;
    char *end = NULL;
    long v = strtol(s, &end, 10);
    if (end == s || *end != '\0') return false;
    if (v < MIN_POWER_MW || v > MAX_POWER_MW) return false;
    *out = (uint32_t)v;
    return true;
}

static void handle_sigint(int sig) {
    (void)sig;
    printf("\n" YELLOW "[GC] Stopping..." RESET "\n");
    running = 0;
}

// Width (visible chars, ignoring ANSI codes) of each antenna slot.
// Fits "(N)(-XXX.X) " + a 24-char EPC-96 hex string.
#define SLOT_WIDTH 38

// Locate (or allocate) the persistent state row for `epc`. Returns
// NULL only if the table is full -- in normal beer-pour operation
// at most 2 mugs are on the tray at once so this never trips.
static TagState *state_find_or_insert(TagState *table, const char *epc)
{
    int free_slot = -1;
    for (int i = 0; i < GC_MAX_TAGS; i++) {
        if (table[i].in_use) {
            if (strcmp(table[i].epc, epc) == 0) return &table[i];
        } else if (free_slot < 0) {
            free_slot = i;
        }
    }
    if (free_slot < 0) return NULL;
    TagState *s = &table[free_slot];
    memset(s, 0, sizeof *s);
    strncpy(s->epc, epc, sizeof s->epc - 1);
    s->epc[sizeof s->epc - 1] = '\0';
    s->owner            = -1;
    s->last_owner_rssi  = INT16_MIN;
    s->wmax_rssi[0]     = INT16_MIN;
    s->wmax_rssi[1]     = INT16_MIN;
    s->in_use           = true;
    return s;
}

// Reset per-window accumulators only; persistent fields (owner,
// last_owner_rssi, unseen_windows, in_use) survive into the next window.
static void state_window_reset(TagState *s)
{
    s->wcount[0]    = 0;
    s->wcount[1]    = 0;
    s->wmax_rssi[0] = INT16_MIN;
    s->wmax_rssi[1] = INT16_MIN;
}

// Pick initial owner when both antennas see the tag (typical on this rig).
// Never requires 2x count when counts are equal -- RSSI breaks the tie.
static bool state_claim_owner(TagState *s)
{
    unsigned c0 = s->wcount[0], c1 = s->wcount[1];
    if (c0 == 0 && c1 == 0) return false;

    int owner;
    if (c0 > 0 && c1 == 0) {
        owner = 0;
    } else if (c1 > 0 && c0 == 0) {
        owner = 1;
    } else {
        int16_t r0 = s->wmax_rssi[0], r1 = s->wmax_rssi[1];
        int16_t rssi_margin = r0 - r1;

        if (c0 > c1)
            owner = 0;
        else if (c1 > c0)
            owner = 1;
        else if (rssi_margin >= RSSI_CLAIM_MARGIN_DB10)
            owner = 0;
        else if (rssi_margin <= -RSSI_CLAIM_MARGIN_DB10)
            owner = 1;
        else if (c0 >= (unsigned)COUNT_DOM_FIRST * c1)
            owner = 0;
        else if (c1 >= (unsigned)COUNT_DOM_FIRST * c0)
            owner = 1;
        else
            owner = (r0 >= r1) ? 0 : 1; /* equal counts, tiny RSSI gap */
    }

    if (s->wcount[owner] < (unsigned)MIN_READS) return false;
    if (s->wmax_rssi[owner] < RSSI_FLOOR_DBM10) return false;

    s->owner           = owner;
    s->last_owner_rssi = s->wmax_rssi[owner];
    return true;
}

// Sticky arbitration at window close.
static bool state_arbitrate(TagState *s)
{
    unsigned c0 = s->wcount[0], c1 = s->wcount[1];

    if (c0 == 0 && c1 == 0) {
        s->unseen_windows++;
        if (s->unseen_windows >= UNSEEN_TTL_WINDOWS)
            s->in_use = false;
        return (s->owner >= 0); /* keep showing last owner while mug present */
    }

    s->unseen_windows = 0;

    if (s->owner < 0)
        return state_claim_owner(s);

    /* Refresh RSSI when owner was seen this window. */
    if (s->wcount[s->owner] > 0)
        s->last_owner_rssi = s->wmax_rssi[s->owner];

    /* Challenger is the antenna with more reads this window (RSSI tiebreak). */
    int chall, other;
    if (c0 > c1)                                      { chall = 0; other = 1; }
    else if (c1 > c0)                                 { chall = 1; other = 0; }
    else if (s->wmax_rssi[0] >= s->wmax_rssi[1])      { chall = 0; other = 1; }
    else                                              { chall = 1; other = 0; }

    if (chall != s->owner) {
        if (s->wcount[other] == 0) {
            if (s->wcount[chall] >= (unsigned)COUNT_DOM_FLIP) {
                s->owner           = chall;
                s->last_owner_rssi = s->wmax_rssi[chall];
            }
        } else if (s->wcount[chall] >= (unsigned)COUNT_DOM_FLIP * s->wcount[other]) {
            s->owner           = chall;
            s->last_owner_rssi = s->wmax_rssi[chall];
        }
        /* else: stay on sticky owner (no cross-read flip) */
    }

    return true;
}

// Sweep output:
//   - "[S0=N S1=N] []" when nothing is currently attributed
//   - otherwise: [S0=N S1=N] [TX=…] [<slot0>,   <slot1>]
// Each slot is padded to SLOT_WIDTH so the comma column never shifts.
static void print_sweep_line(uint32_t power,
                             TagEntry bucket[ANTENNA_COUNT][GC_MAX_TAGS],
                             const int cnt[ANTENNA_COUNT],
                             const unsigned long scans[ANTENNA_COUNT])
{
    if (cnt[0] == 0 && cnt[1] == 0) {
        printf(CYAN "[S0=%lu S1=%lu]" RESET " []\n",
               scans[0], scans[1]);
        fflush(stdout);
        return;
    }

    printf(CYAN "[S0=%lu S1=%lu]" RESET " "
           CYAN "[TX=%u mW]" RESET " [",
           scans[0], scans[1], (unsigned)power);

    for (int ant = 0; ant < ANTENNA_COUNT; ant++) {
        if (ant > 0)
            printf(",   ");

        if (cnt[ant] == 0) {
            printf("%*s", SLOT_WIDTH, "");
            continue;
        }

        int visible = 3; /* "(N)" */
        for (int i = 0; i < cnt[ant]; i++) {
            char rbuf[24];
            int rlen = snprintf(rbuf, sizeof rbuf,
                                "(%.1f) ",
                                bucket[ant][i].rssi / 10.0);
            if (i > 0) visible += 1;
            visible += rlen + (int)strlen(bucket[ant][i].tag);
        }

        const char *tagcol = (ant == 0) ? GREEN : RED;
        printf(YELLOW "(%d)" RESET, ant);
        for (int i = 0; i < cnt[ant]; i++) {
            if (i > 0) printf(" ");
            printf("(%.1f) %s%s" RESET,
                   bucket[ant][i].rssi / 10.0,
                   tagcol, bucket[ant][i].tag);
        }
        int pad = SLOT_WIDTH - visible;
        if (pad > 0) printf("%*s", pad, "");
    }
    printf("]\n");
    fflush(stdout);
}

int main(int argc, char **argv) {

    uint32_t power = DEFAULT_POWER_MW;

    if (argc == 2) {
        if (strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0) {
            usage(argv[0]);
            return 0;
        }
        if (!parse_power(argv[1], &power)) {
            usage(argv[0]);
            return 1;
        }
    } else if (argc != 1) {
        usage(argv[0]);
        return 1;
    }

    CAENRFIDErrorCodes ec;
    CAENRFIDReader reader = {
        .connect       = _connect,
        .disconnect    = _disconnect,
        .tx            = _tx,
        .rx            = _rx,
        .clear_rx_data = _clear_rx_data,
        .enable_irqs   = _enable_irqs,
        .disable_irqs  = _disable_irqs
    };

    RS232_params port_params = {
        .com         = GC_PORT,
        .baudrate    = GC_BAUDRATE,
        .dataBits    = 8,
        .stopBits    = 1,
        .parity      = 0,
        .flowControl = 0,
    };

    const char *sources[ANTENNA_COUNT] = { "Source_0", "Source_1" };
    char model[64]  = {0};
    char serial[64] = {0};

    signal(SIGINT, handle_sigint);

    printf(CYAN "===== Dual-Antenna RFID Live Scanner (sticky owner) =====" RESET "\n");
    printf("Port      : %s @ %d baud\n", GC_PORT, GC_BAUDRATE);
    printf("Power     : %u mW (both antennas)\n", power);
    printf("Window    : %d ms\n", GC_WINDOW_MS);
    printf("Antennas  : %s, %s  (150 mm centre-to-centre)\n", sources[0], sources[1]);
    printf("Sticky    : floor=%.1f dBm, RSSI claim=%.1f dB, flip>=%dx, TTL=%d windows\n\n",
           RSSI_FLOOR_DBM10 / 10.0,
           RSSI_CLAIM_MARGIN_DB10 / 10.0,
           COUNT_DOM_FLIP, UNSEEN_TTL_WINDOWS);

    printf("[GC] Connecting...\n");
    ec = CAENRFID_Connect(&reader, CAENRFID_RS232, &port_params);
    if (ec != CAENRFID_StatusOK) {
        printf("[GC] ERROR: Could not connect (code %d)\n", ec);
        printf("  - Check USB cable\n");
        printf("  - Try: sudo chmod 666 %s\n", GC_PORT);
        printf("  - Or:  sudo usermod -a -G dialout $USER  (then re-login)\n");
        return -1;
    }

    ec = CAENRFID_GetReaderInfo(&reader, model, serial);
    if (ec == CAENRFID_StatusOK)
        printf("[GC] Reader: %s  Serial: %s\n", model, serial);

    char fwrel[MAX_FWREL_LENGTH + 1] = {0};
    if (CAENRFID_GetFirmwareRelease(&reader, fwrel) == CAENRFID_StatusOK)
        printf("[GC] Firmware: %s\n", fwrel);

    ec = CAENRFID_SetPower(&reader, power);
    if (ec != CAENRFID_StatusOK) {
        printf("[GC] WARNING: SetPower(%u) returned %d -- "
               "value may be above the reader's hardware ceiling.\n",
               power, ec);
    }
    printf("[GC] Ready. Each line is prefixed with [S0=N S1=N] scan counts. Ctrl+C to stop.\n\n");

    running = 1;

    /* Persistent EPC table -- owner, last_owner_rssi and unseen
       counter survive across windows. Per-window accumulators are
       cleared at each window close. */
    TagState states[GC_MAX_TAGS] = {0};

    unsigned long scans[ANTENNA_COUNT] = { 0UL, 0UL };

    struct timespec t_window_mono;
    clock_gettime(CLOCK_MONOTONIC, &t_window_mono);

    while (running) {

        for (int ant = 0; ant < ANTENNA_COUNT && running; ant++) {
            CAENRFIDTagList *tag_list = NULL, *node;
            uint16_t num_tags = 0;

            ec = CAENRFID_InventoryTag(&reader, (char *)sources[ant],
                                       0, 0, 0,
                                       NULL, 0,
                                       RSSI,
                                       &tag_list, &num_tags);

            scans[ant]++;

            node = tag_list;
            while (node != NULL) {
                if (ec == CAENRFID_StatusOK && num_tags > 0) {
                    char epc[2 * MAX_ID_LENGTH + 1];
                    hex_str(node->Tag.ID, node->Tag.Length, epc);
                    TagState *s = state_find_or_insert(states, epc);
                    if (s != NULL) {
                        s->wcount[ant]++;
                        if (node->Tag.RSSI > s->wmax_rssi[ant])
                            s->wmax_rssi[ant] = node->Tag.RSSI;
                    }
                }
                CAENRFIDTagList *next = node->Next;
                free(node);
                node = next;
            }
        }

        /* Window close? Run sticky arbitration for every EPC in the
           table, emit one summary line, then clear per-window
           accumulators (persistent fields are preserved). */
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        double elapsed_ms = (now.tv_sec  - t_window_mono.tv_sec) * 1000.0 +
                            (now.tv_nsec - t_window_mono.tv_nsec) / 1e6;

        if (elapsed_ms >= (double)GC_WINDOW_MS) {
            TagEntry disp_bucket[ANTENNA_COUNT][GC_MAX_TAGS];
            int      disp_cnt[ANTENNA_COUNT] = {0, 0};

            for (int i = 0; i < GC_MAX_TAGS; i++) {
                TagState *s = &states[i];
                if (!s->in_use) continue;

                bool show = state_arbitrate(s);

                /* state_arbitrate may have evicted the row (TTL). */
                if (!s->in_use) continue;
                if (!show || s->owner < 0) {
                    state_window_reset(s);
                    continue;
                }
                if (disp_cnt[s->owner] >= GC_MAX_TAGS) {
                    state_window_reset(s);
                    continue;
                }

                TagEntry *e = &disp_bucket[s->owner][disp_cnt[s->owner]++];
                strncpy(e->tag, s->epc, sizeof e->tag - 1);
                e->tag[sizeof e->tag - 1] = '\0';
                e->antenna = s->owner;
                e->rssi    = s->last_owner_rssi;

                state_window_reset(s);
            }

            print_sweep_line(power, disp_bucket, disp_cnt, scans);

            scans[0]      = 0;
            scans[1]      = 0;
            t_window_mono = now;
        }
    }

    CAENRFID_Disconnect(&reader);
    printf("[GC] Disconnected.\n");
    return 0;
}
