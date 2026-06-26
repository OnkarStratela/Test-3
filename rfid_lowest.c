// rfid_lowest.c
// Live-state RFID scanner using two antennas (Source_0 and Source_1).
//
// This is a variant of rfid_standard.c with ONE extra rule applied on
// every print (sweep):
//
//   If the SAME mug (same EPC) is detected on BOTH antennas at the same
//   time, it is mapped to the source with the LOWEST RSSI and the
//   highest-RSSI detection is completely discarded.
//
// "Lowest RSSI" means the smaller (more negative) dBm value, e.g.
// -65.0 dBm is lower than -63.1 dBm, so the -65.0 reading is kept and
// the -63.1 reading is dropped. A mug seen on only one antenna is left
// untouched. Everything else (formatting, colours, fixed slots) is
// identical to rfid_standard.
//
// Output format (unchanged):
//   []                                          empty sweep
//   [TX=30 mW] [(0)(-65.0) <EPC>,                                      ]
//   [TX=30 mW] [                                    ,   (1)(-65.0) <EPC>]
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

// Map each mug to the source with the LOWEST RSSI and discard the
// highest-RSSI detection.
//
// For every EPC seen on BOTH antennas in this sweep we keep only the
// reading with the lower (more negative) RSSI and remove the other one.
// Mugs seen on a single antenna are left as-is.
static void map_lowest_discard_highest(TagEntry bucket[ANTENNA_COUNT][GC_MAX_TAGS],
                                       int cnt[ANTENNA_COUNT])
{
    for (int i = 0; i < cnt[0]; i++) {
        for (int j = 0; j < cnt[1]; j++) {
            if (strcmp(bucket[0][i].tag, bucket[1][j].tag) != 0)
                continue;

            // Same mug on both antennas at the same time.
            if (bucket[0][i].rssi <= bucket[1][j].rssi) {
                // Antenna 0 is the lowest RSSI -> keep it, drop antenna 1's copy.
                for (int k = j; k < cnt[1] - 1; k++)
                    bucket[1][k] = bucket[1][k + 1];
                cnt[1]--;
                j--;
            } else {
                // Antenna 1 is the lowest RSSI -> keep it, drop antenna 0's copy.
                for (int k = i; k < cnt[0] - 1; k++)
                    bucket[0][k] = bucket[0][k + 1];
                cnt[0]--;
                i--;
                break; // bucket[0][i] changed; restart inner scan for it
            }
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

    printf(CYAN "===== Dual-Antenna RFID Live Scanner (lowest-RSSI mapping) =====" RESET "\n");
    printf("Port      : %s @ %d baud\n", GC_PORT, GC_BAUDRATE);
    printf("Power     : %u mW (both antennas)\n", power);
    printf("Cycle     : %d ms\n", GC_SCAN_MS);
    printf("Antennas  : %s, %s\n", sources[0], sources[1]);
    printf("Rule      : same mug on both antennas -> keep LOWEST RSSI, discard highest\n\n");

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

        // Keep the lowest-RSSI source per mug, discard highest when seen on both.
        map_lowest_discard_highest(bucket, cnt);

        print_sweep_line(power, bucket, cnt);

        usleep(GC_SCAN_MS * 1000);
    }

    CAENRFID_Disconnect(&reader);
    printf("[GC] Disconnected.\n");
    return 0;
}
