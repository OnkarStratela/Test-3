// rfid_lowest.c
// Live-state RFID scanner using two antennas (Source_0 and Source_1).
//
// Cross-read suppression variant of rfid_standard.c. Two rules are
// applied on every print (sweep):
//
//   1. NEAR-FIELD FLOOR: any detection weaker than NEAR_FIELD_DBM10
//      (default -65.0 dBm) is discarded outright. A mug physically on
//      an antenna reads ~-55..-60 dBm; leakage / cross reads from the
//      other antenna read ~-65 dBm or weaker, so the floor removes them
//      even when the home antenna missed that mug in the same sweep.
//
//   2. STRONGEST WINS: if the SAME mug (same EPC) still survives on BOTH
//      antennas, it is mapped to the source with the HIGHEST RSSI (the
//      nearest / home antenna) and the weaker (lower RSSI) cross read is
//      discarded.
//
//   3. STICKY OWNER: each mug remembers its antenna and only moves after
//      the other antenna wins FLIP_DEBOUNCE sweeps in a row, so a single
//      noisy sweep can never produce a momentary overlap.
//
// "Highest RSSI" means the larger (closer-to-zero) dBm value, e.g.
// -57.0 dBm is higher than -65.0 dBm, so the -57.0 reading is kept and
// the -65.0 reading is dropped. Everything else (formatting, colours,
// fixed slots) is identical to rfid_standard.
//
// Output format (unchanged):
//   []                                          empty sweep
//   [TX=30 mW] [(0)(-57.0) <EPC>,                                      ]
//   [TX=30 mW] [                                    ,   (1)(-57.0) <EPC>]
//
// Antenna index in YELLOW; Src0 tag EPC in GREEN, Src1 tag EPC in RED.
//
// Usage:
//   ./rfid_lowest              -> both antennas at default power
//   ./rfid_lowest <mW>         -> both antennas at <mW> (global power)
//   ./rfid_lowest -h | --help  -> show usage

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <signal.h>

#include "CAENRFIDLib_Light.h"
#include "host.h"

// Configuration
#define GC_PORT             "/dev/ttyACM0"
#define GC_BAUDRATE         921600
#define DEFAULT_POWER_MW    30            // sensible default for ~7 cm read zone
#define MIN_POWER_MW        1             // reader rejects below its hardware floor
#define MAX_POWER_MW        316           // R3100C Lepton3 max (25 dBm)
#define GC_SCAN_MS          100           // ms between scan cycles (= line rate when printing every cycle)
#define GC_MAX_TAGS         64            // max tags merged across both antennas per sweep
#define ANTENNA_COUNT       2
#define MAX_ID_LENGTH       64

// Near-field RSSI floor in tenths of dBm. Any read weaker (more
// negative) than this is treated as leakage / a cross read and dropped.
// -65.0 dBm is the cleanest home-vs-leakage split on this rig.
#define NEAR_FIELD_DBM10    (-650)

// Sticky-owner hysteresis. Once a mug is attributed to an antenna it
// stays there; it only moves to the other antenna after that antenna
// wins FLIP_DEBOUNCE sweeps in a row. This kills the occasional single-
// sweep overlap caused by a momentary stronger leakage read. An EPC is
// forgotten after UNSEEN_TTL sweeps without any sighting.
#define FLIP_DEBOUNCE       3
#define UNSEEN_TTL          6

// ANSI colours
#define GREEN  "\033[0;32m"
#define RED    "\033[0;31m"
#define YELLOW "\033[0;33m"
#define CYAN   "\033[0;36m"
#define RESET  "\033[0m"

volatile int running = 0;

typedef struct {
    char    tag[2 * MAX_ID_LENGTH + 1];
    int     antenna;
    int16_t rssi;        /* tenths of dBm, as reported by the reader */
} TagEntry;

// Persistent per-mug ownership used for the sticky-owner hysteresis.
//   owner       : -1 until first attribution, then 0 or 1
//   last_rssi   : last RSSI shown for this mug (tenths dBm)
//   flip_streak : consecutive sweeps the NON-owner antenna won
//   unseen      : consecutive sweeps with no sighting (drives eviction)
typedef struct {
    char     epc[2 * MAX_ID_LENGTH + 1];
    int      owner;
    int16_t  last_rssi;
    int      flip_streak;
    int      unseen;
    bool     in_use;
} TagOwn;

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

// Remove a single entry from one antenna bucket, shifting the rest down.
static void bucket_remove(TagEntry bucket[GC_MAX_TAGS], int *cnt, int idx)
{
    for (int k = idx; k < *cnt - 1; k++)
        bucket[k] = bucket[k + 1];
    (*cnt)--;
}

// Cross-read suppression applied per sweep:
//   1. Drop any read weaker than the near-field floor (leakage).
//   2. For a mug surviving on BOTH antennas, keep the HIGHEST RSSI
//      (nearest / home antenna) and discard the weaker cross read.
static void suppress_cross_reads(TagEntry bucket[ANTENNA_COUNT][GC_MAX_TAGS],
                                 int cnt[ANTENNA_COUNT])
{
    // 1. Near-field floor: discard anything weaker than the threshold.
    for (int ant = 0; ant < ANTENNA_COUNT; ant++) {
        for (int i = 0; i < cnt[ant]; i++) {
            if (bucket[ant][i].rssi < NEAR_FIELD_DBM10) {
                bucket_remove(bucket[ant], &cnt[ant], i);
                i--;
            }
        }
    }

    // 2. Strongest wins for any mug still seen on both antennas.
    for (int i = 0; i < cnt[0]; i++) {
        for (int j = 0; j < cnt[1]; j++) {
            if (strcmp(bucket[0][i].tag, bucket[1][j].tag) != 0)
                continue;

            // Same mug on both antennas at the same time.
            if (bucket[0][i].rssi >= bucket[1][j].rssi) {
                // Antenna 0 has the higher RSSI -> keep it, drop antenna 1's copy.
                bucket_remove(bucket[1], &cnt[1], j);
                j--;
            } else {
                // Antenna 1 has the higher RSSI -> keep it, drop antenna 0's copy.
                bucket_remove(bucket[0], &cnt[0], i);
                i--;
                break; // bucket[0][i] changed; restart inner scan for it
            }
        }
    }
}

// Locate (or allocate) the ownership row for `epc`. Returns NULL only if
// the table is full (never happens with a couple of mugs on the tray).
static TagOwn *own_find_or_insert(TagOwn *tbl, const char *epc)
{
    int free_slot = -1;
    for (int i = 0; i < GC_MAX_TAGS; i++) {
        if (tbl[i].in_use) {
            if (strcmp(tbl[i].epc, epc) == 0) return &tbl[i];
        } else if (free_slot < 0) {
            free_slot = i;
        }
    }
    if (free_slot < 0) return NULL;
    TagOwn *s = &tbl[free_slot];
    memset(s, 0, sizeof *s);
    strncpy(s->epc, epc, sizeof s->epc - 1);
    s->epc[sizeof s->epc - 1] = '\0';
    s->owner       = -1;
    s->last_rssi   = INT16_MIN;
    s->flip_streak = 0;
    s->unseen      = 0;
    s->in_use      = true;
    return s;
}

// Force every surviving mug onto its sticky owner antenna. Each EPC is
// on at most one antenna here (suppress_cross_reads already collapsed
// same-sweep duplicates). A mug only changes antenna after the other
// antenna wins FLIP_DEBOUNCE sweeps in a row, so a single stray read
// can never produce an overlap.
static void apply_sticky_owner(TagOwn *tbl,
                               TagEntry bucket[ANTENNA_COUNT][GC_MAX_TAGS],
                               int cnt[ANTENNA_COUNT])
{
    // Snapshot this sweep's survivors before we rebuild the buckets.
    TagEntry survivors[ANTENNA_COUNT * GC_MAX_TAGS];
    int ns = 0;
    for (int ant = 0; ant < ANTENNA_COUNT; ant++)
        for (int i = 0; i < cnt[ant]; i++)
            survivors[ns++] = bucket[ant][i];

    bool seen[GC_MAX_TAGS] = { false };

    cnt[0] = 0;
    cnt[1] = 0;

    for (int s = 0; s < ns; s++) {
        TagOwn *st = own_find_or_insert(tbl, survivors[s].tag);
        int cand = survivors[s].antenna;

        if (st == NULL) {
            // Table full: fall back to showing it where it was read.
            if (cnt[cand] < GC_MAX_TAGS)
                bucket[cand][cnt[cand]++] = survivors[s];
            continue;
        }

        seen[(int)(st - tbl)] = true;
        st->unseen = 0;

        if (st->owner < 0) {
            st->owner = cand;            // first sighting claims ownership
            st->flip_streak = 0;
        } else if (cand == st->owner) {
            st->flip_streak = 0;         // owner confirmed, reset challenger
        } else {
            st->flip_streak++;           // other antenna is challenging
            if (st->flip_streak >= FLIP_DEBOUNCE) {
                st->owner = cand;        // sustained -> hand over ownership
                st->flip_streak = 0;
            }
        }

        int16_t rssi;
        if (cand == st->owner) {
            st->last_rssi = survivors[s].rssi;
            rssi = survivors[s].rssi;
        } else {
            // Read on the non-owner this sweep: keep it on the owner slot
            // using the last RSSI the owner saw (falls back to this read).
            rssi = (st->last_rssi != INT16_MIN) ? st->last_rssi
                                                : survivors[s].rssi;
        }

        int ant = st->owner;
        if (cnt[ant] < GC_MAX_TAGS) {
            strncpy(bucket[ant][cnt[ant]].tag, st->epc,
                    sizeof bucket[ant][cnt[ant]].tag - 1);
            bucket[ant][cnt[ant]].tag[sizeof bucket[ant][cnt[ant]].tag - 1] = '\0';
            bucket[ant][cnt[ant]].antenna = ant;
            bucket[ant][cnt[ant]].rssi    = rssi;
            cnt[ant]++;
        }
    }

    // Age out mugs that were not seen this sweep so the slot can clear.
    for (int k = 0; k < GC_MAX_TAGS; k++) {
        if (tbl[k].in_use && !seen[k]) {
            if (++tbl[k].unseen >= UNSEEN_TTL)
                tbl[k].in_use = false;
        }
    }
}

// Width (visible chars, ignoring ANSI colour codes) reserved for each
// antenna slot inside the brackets.
#define SLOT_WIDTH 38

static void print_sweep_line(uint32_t power,
                             TagEntry bucket[ANTENNA_COUNT][GC_MAX_TAGS],
                             const int cnt[ANTENNA_COUNT])
{
    if (cnt[0] == 0 && cnt[1] == 0) {
        printf("[]\n");
        fflush(stdout);
        return;
    }

    printf(CYAN "[TX=%u mW]" RESET " [", (unsigned)power);

    for (int ant = 0; ant < ANTENNA_COUNT; ant++) {
        if (ant > 0)
            printf(",   "); /* fixed separator between the two slots */

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
            if (i > 0) visible += 1; /* space between multiple tags */
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

    printf(CYAN "===== Dual-Antenna RFID Live Scanner (cross-read suppression) =====" RESET "\n");
    printf("Port      : %s @ %d baud\n", GC_PORT, GC_BAUDRATE);
    printf("Power     : %u mW (both antennas)\n", power);
    printf("Cycle     : %d ms\n", GC_SCAN_MS);
    printf("Antennas  : %s, %s\n", sources[0], sources[1]);
    printf("Rule      : drop reads weaker than %.1f dBm; same mug on both -> keep HIGHEST RSSI\n",
           NEAR_FIELD_DBM10 / 10.0);
    printf("Sticky    : owner flips only after %d sweeps; forget after %d unseen\n\n",
           FLIP_DEBOUNCE, UNSEEN_TTL);

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
               "value may be below the reader's hardware floor.\n",
               power, ec);
    }
    printf("[GC] Ready. Empty sweeps print []. Tagged sweeps prepend [TX …]. Ctrl+C to stop.\n\n");

    running = 1;

    TagEntry bucket[ANTENNA_COUNT][GC_MAX_TAGS];
    int      cnt[ANTENNA_COUNT];

    /* Persistent ownership table for the sticky-owner hysteresis. */
    TagOwn owners[GC_MAX_TAGS] = {0};

    while (running) {

        cnt[0] = 0;
        cnt[1] = 0;

        for (int ant = 0; ant < ANTENNA_COUNT && running; ant++) {
            CAENRFIDTagList *tag_list = NULL, *node;
            uint16_t num_tags = 0;

            ec = CAENRFID_InventoryTag(&reader, (char *)sources[ant],
                                       0, 0, 0,
                                       NULL, 0,
                                       RSSI,
                                       &tag_list, &num_tags);

            if (ec == CAENRFID_StatusOK && num_tags > 0) {
                node = tag_list;
                while (node != NULL) {
                    if (cnt[ant] < GC_MAX_TAGS && cnt[0] + cnt[1] < GC_MAX_TAGS) {
                        hex_str(node->Tag.ID, node->Tag.Length,
                                bucket[ant][cnt[ant]].tag);
                        bucket[ant][cnt[ant]].antenna = ant;
                        bucket[ant][cnt[ant]].rssi    = node->Tag.RSSI;
                        cnt[ant]++;
                    }
                    CAENRFIDTagList *next = node->Next;
                    free(node);
                    node = next;
                }
            } else {
                // Free list if returned with non-OK code
                node = tag_list;
                while (node != NULL) {
                    CAENRFIDTagList *next = node->Next;
                    free(node);
                    node = next;
                }
            }
        }

        // Drop leakage below the near-field floor; keep highest-RSSI source per mug.
        suppress_cross_reads(bucket, cnt);

        // Pin each mug to its sticky owner so a stray sweep can't overlap.
        apply_sticky_owner(owners, bucket, cnt);

        print_sweep_line(power, bucket, cnt);

        usleep(GC_SCAN_MS * 1000);
    }

    CAENRFID_Disconnect(&reader);
    printf("[GC] Disconnected.\n");
    return 0;
}
