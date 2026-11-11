/*
 * MIT License
 *
 * Copyright (c) 2026 ksjo
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#define _FILE_OFFSET_BITS 64    /* /dev/mem offsets reach 0xFE000000 on 32-bit */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/mman.h>
#include "dshot.h"

/* =========================================================================
 * Peripheral map. Offsets are from the SoC peripheral base, which is read
 * from the device tree so no board table is needed.
 * ========================================================================= */
#define MAP_SIZE            0x1000
#define REG(x)              ((x) / 4)
#define DT_SOC_RANGES       "/proc/device-tree/soc/ranges"
#define BASE_BCM2835        0x20000000u
#define BASE_BCM2837        0x3F000000u
#define BASE_BCM2711        0xFE000000u

#define OFS_CLOCK           0x101000u
#define OFS_GPIO            0x200000u
#define OFS_SMI             0x600000u

#define GPCLR0              0x28
#define GPLEV0              0x34
#define GPIO_FSEL_OUTPUT    1u
#define GPIO_FSEL_ALT1      5u      /* SMI on GPIO 0-25 */

/* SMI registers. Layout and bit meanings follow the Raspberry Pi kernel's
 * include/linux/broadcom/bcm2835_smi.h; the block has no public datasheet. */
#define SMI_CS      0x00    /* control and status */
#define SMI_L       0x04    /* transfer count of a programmed transfer */
#define SMI_A       0x08    /* address and device select */
#define SMI_D       0x0C    /* data FIFO */
#define SMI_DSR0    0x10    /* device 0 read settings */
#define SMI_DSW0    0x14    /* device 0 write settings */
#define SMI_DMC     0x30    /* DMA control */
#define SMI_DCS     0x34    /* direct-mode control */
#define SMI_FD      0x40    /* FIFO debug: word count in [5:0] */

#define SMI_CS_ENABLE   (1u << 0)
#define SMI_CS_DONE     (1u << 1)
#define SMI_CS_ACTIVE   (1u << 2)
#define SMI_CS_START    (1u << 3)   /* write 1: start; reads back 0 */
#define SMI_CS_CLEAR    (1u << 4)   /* write 1: empty the FIFOs */
#define SMI_CS_WRITE    (1u << 5)   /* direction: 1 = out to the pins */
#define SMI_CS_SETERR   (1u << 13)  /* settings written while enabled; write 1 to clear */
#define SMI_CS_PXLDAT   (1u << 14)  /* pack several transfers into one FIFO word */
#define SMI_CS_AFERR    (1u << 25)  /* FIFO read empty / written full; write 1 to clear */
#define SMI_CS_TXD      (1u << 28)  /* FIFO asks for more */
#define SMI_CS_TXE      (1u << 30)  /* FIFO empty */
#define SMI_CS_ERRORS   (SMI_CS_AFERR | SMI_CS_SETERR)

/* DSR0/DSW0: width [31:30], setup [29:24], hold [21:16], strobe [6:0].
 * Width codes are the register's (2 = 18 bit), not the kernel's enum. */
#define SMI_DS_WIDTH_SHIFT   30
#define SMI_DS_SETUP_SHIFT   24
#define SMI_DS_HOLD_SHIFT    16
#define SMI_WIDTH_CODE_8     0u
#define SMI_WIDTH_CODE_16    1u
#define SMI_WIDTH_CODE_18    2u
#define SMI_CYCLE_MAX        (63 + 127 + 63)
#define SMI_FD_FCNT_MASK     0x3Fu

/* Clock manager, SMI clock: PLLD / 1..15 (4-bit integer divider). */
#define CM_SMICTL       0xB0
#define CM_SMIDIV       0xB4
#define CM_PASSWD       (0x5Au << 24)
#define CM_BUSY         (1u << 7)
#define CM_KILL         (1u << 5)
#define CM_ENAB         (1u << 4)
#define CM_SRC_PLLD     6u
#define SMI_DIVI_MIN    2
#define SMI_DIVI_MAX    15

#define DSHOT_BITS              16
#define MAX_WORDS               (DSHOT_BITS * 8)
#define FIFO_PROBE_MAX          256
#define FIFO_OVERFILL_MAX       8
#define SELFTEST_RUNS           5
#define SELFTEST_TRIES          50
#define SELFTEST_POLL_GAP_NS    20000LL
#define KERNEL_SMI_DRIVER       "/sys/bus/platform/drivers/smi-bcm2835"
#define PLLD_RATE_PATH          "/sys/kernel/debug/clk/plld_per/clk_rate"

static bool     s_initialized = false;
static int      s_count       = 0;
static int      s_gpios[DSHOT_MAX_MOTORS];
static uint32_t s_pin_bit[DSHOT_MAX_MOTORS];   /* bit within one SMI sample */
static uint32_t s_all_bits  = 0;
static uint32_t s_gpio_mask = 0;                   /* same pins as a bank0 mask */

static DshotConfig s_cfg;
static DshotInfo   s_info;

static volatile uint32_t *s_gpio = NULL;
static volatile uint32_t *s_smi  = NULL;
static volatile uint32_t *s_cm   = NULL;
static uint32_t  s_width_code = 0;
static uint32_t  s_cs_base = 0;        /* ENABLE | WRITE (| PXLDAT) */
static int64_t   s_last_start_ns = 0;
static int64_t   s_stall_ns = 0;       /* busy this long after a start = stalled */

/* One DShot bit as SMI samples: s_seg_h high, s_seg_d carrying the bit, the
 * rest low. */
static int s_seg_h, s_seg_d, s_seg_l;
/* Sample index -> FIFO word index and bit position inside that word. */
static int s_word_shift, s_word_mask;
static int s_shift[4];

static const uint32_t s_zero_words[MAX_WORDS];

static inline uint32_t s_rd(int off)             { return s_smi[REG(off)]; }
static inline void     s_wr(int off, uint32_t v) { s_smi[REG(off)] = v; }

static inline int64_t s_now_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000000000LL + t.tv_nsec;
}

static void s_busy_wait_ns(int64_t ns)
{
    int64_t t0 = s_now_ns();
    while (s_now_ns() - t0 < ns)
        ;
}

/* =========================================================================
 * Register mapping
 * ========================================================================= */
static uint32_t s_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | p[3];
}

/* /soc/ranges maps bus address 0x7E000000 to the CPU physical base. The
 * parent address is one cell on BCM2835/2837 and two on BCM2711. */
static uint32_t s_periph_base(void)
{
    uint8_t b[12] = { 0 };
    FILE *fp = fopen(DT_SOC_RANGES, "rb");
    if (!fp) return 0;
    size_t n = fread(b, 1, sizeof(b), fp);
    fclose(fp);
    if (n < 8) return 0;

    uint32_t base = s_be32(b + 4);
    if (base == 0 && n >= 12) base = s_be32(b + 8);
    if (base != BASE_BCM2835 && base != BASE_BCM2837 && base != BASE_BCM2711)
        return 0;
    return base;
}

static volatile uint32_t *s_map(int fd, uint32_t phys)
{
    void *p = mmap(NULL, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd,
                   (off_t)phys);
    return (p == MAP_FAILED) ? NULL : (volatile uint32_t *)p;
}

static void s_unmap(void)
{
    if (s_gpio) { munmap((void *)s_gpio, MAP_SIZE); s_gpio = NULL; }
    if (s_smi)  { munmap((void *)s_smi,  MAP_SIZE); s_smi  = NULL; }
    if (s_cm)   { munmap((void *)s_cm,   MAP_SIZE); s_cm   = NULL; }
}

static int s_map_all(uint32_t base)
{
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        perror("dshot: open(/dev/mem)");
        return -1;
    }
    s_gpio = s_map(fd, base + OFS_GPIO);
    s_smi  = s_map(fd, base + OFS_SMI);
    s_cm   = s_map(fd, base + OFS_CLOCK);
    close(fd);
    if (!s_gpio || !s_smi || !s_cm) {
        fprintf(stderr, "dshot: mmap of the peripheral registers failed\n");
        s_unmap();
        return -1;
    }
    return 0;
}

static void s_gpio_fsel(int gpio, uint32_t fsel)
{
    int      idx   = gpio / 10;
    int      shift = (gpio % 10) * 3;
    uint32_t v     = s_gpio[idx];
    s_gpio[idx] = (v & ~(7u << shift)) | (fsel << shift);
}

static void s_pins_plain_low(void)
{
    if (s_cfg.dry_run) return;
    s_gpio[REG(GPCLR0)] = s_gpio_mask;
    for (int i = 0; i < s_count; i++)
        s_gpio_fsel(s_gpios[i], GPIO_FSEL_OUTPUT);
}

/* =========================================================================
 * Clock
 * ========================================================================= */
static uint32_t s_plld_hz(void)
{
    int fd = open(PLLD_RATE_PATH, O_RDONLY);
    if (fd >= 0) {
        char b[24] = { 0 };
        ssize_t n = read(fd, b, sizeof(b) - 1);
        close(fd);
        if (n > 0) {
            long hz = atol(b);
            if (hz >= 100000000L && hz <= 2000000000L)
                return (uint32_t)hz;
        }
    }
    uint32_t hz = (s_info.periph_base == BASE_BCM2711) ? 750000000u : 500000000u;
    fprintf(stderr, "dshot: cannot read %s, assuming PLLD %u MHz\n",
            PLLD_RATE_PATH, hz / 1000000u);
    return hz;
}

/* Every bounded wait here reads the clock before the status it then judges.
 * The other order lets a preemption between the two reads pair a stale status
 * with a late time and report a timeout that never happened. */
static bool s_cm_wait(uint32_t want_busy)
{
    int64_t t0 = s_now_ns();
    for (;;) {
        int64_t t = s_now_ns();
        if ((s_cm[REG(CM_SMICTL)] & CM_BUSY) == want_busy) return true;
        if (t - t0 > 1000000LL) return false;
    }
}

static void s_clock_stop(void)
{
    uint32_t src = s_cm[REG(CM_SMICTL)] & 0xFu;
    s_cm[REG(CM_SMICTL)] = CM_PASSWD | src;          /* ENAB off, source kept */
    if (!s_cm_wait(0)) {
        s_cm[REG(CM_SMICTL)] = CM_PASSWD | CM_KILL;
        s_cm_wait(0);
    }
}

static int s_clock_start(int divi)
{
    s_clock_stop();
    s_cm[REG(CM_SMIDIV)] = CM_PASSWD | ((uint32_t)divi << 12);
    s_busy_wait_ns(10000);
    s_cm[REG(CM_SMICTL)] = CM_PASSWD | CM_SRC_PLLD;
    s_busy_wait_ns(10000);
    s_cm[REG(CM_SMICTL)] = CM_PASSWD | CM_SRC_PLLD | CM_ENAB;
    if (!s_cm_wait(CM_BUSY)) {
        fprintf(stderr, "dshot: SMI clock did not start (CM_SMICTL=0x%08X)\n",
                (unsigned)s_cm[REG(CM_SMICTL)]);
        return -1;
    }
    return 0;
}

/* Pick the SMI clock divider and the cycle length in clocks that come closest
 * to want_ns. Slower clocks win ties, and cycles of fewer than 8 clocks are
 * only used when nothing longer fits. */
static int s_solve_timing(double want_ns, double pll_ns,
                          int *divi, int *clocks, double *got_ns)
{
    for (int pass = 0; pass < 2; pass++) {
        int    n_min = (pass == 0) ? 8 : 3;
        double best  = -1.0;
        for (int d = SMI_DIVI_MAX; d >= SMI_DIVI_MIN; d--) {
            double tc = d * pll_ns;
            int    n  = (int)(want_ns / tc + 0.5);
            if (n < n_min || n > SMI_CYCLE_MAX) continue;
            double err = n * tc - want_ns;
            if (err < 0) err = -err;
            if (best < 0 || err < best - 1e-6) {
                best    = err;
                *divi   = d;
                *clocks = n;
                *got_ns = n * tc;
            }
        }
        if (best >= 0) return 0;
    }
    return -1;
}

/* =========================================================================
 * Frame encoding
 * ========================================================================= */
static uint16_t s_dshot_frame(uint16_t value)
{
    uint16_t packet = (uint16_t)((value & 0x7FF) << 1);   /* telem=0 */
    uint16_t crc    = (uint16_t)((packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F);
    return (uint16_t)((packet << 4) | crc);
}

static void s_build_words(const uint16_t *frames, uint32_t *w)
{
    memset(w, 0, sizeof(uint32_t) * (size_t)s_info.frame_words);

    int si = 0;
    for (int b = DSHOT_BITS - 1; b >= 0; b--) {            /* MSB first */
        uint32_t ones = 0;
        for (int m = 0; m < s_count; m++)
            if ((frames[m] >> b) & 1u) ones |= s_pin_bit[m];

        for (int k = 0; k < s_seg_h; k++, si++)
            w[si >> s_word_shift] |= s_all_bits << s_shift[si & s_word_mask];
        for (int k = 0; k < s_seg_d; k++, si++)
            w[si >> s_word_shift] |= ones << s_shift[si & s_word_mask];
        si += s_seg_l;
    }
}

/* =========================================================================
 * Transfer
 * ========================================================================= */
static inline bool s_smi_busy(uint32_t cs)
{
    return (cs & SMI_CS_ACTIVE) || !(cs & SMI_CS_TXE);
}

/* Load one frame and start it. The SMI must be idle. */
static void s_load_and_start(const uint32_t *w)
{
    int n = s_info.frame_words;
    for (int i = 0; i < n; i++)
        s_wr(SMI_D, w[i]);
    s_wr(SMI_CS, s_cs_base | SMI_CS_START);
    s_last_start_ns = s_now_ns();
}

/* Program the transfer count. SMI_L is a setup register: written while the
 * block is enabled it raises SETERR, so the block is disabled around the
 * write, which also aborts a transfer in progress and empties the FIFO.
 * WRITE stays set throughout: dropping it turns the data pins around. */
static void s_set_len(int transfers)
{
    s_wr(SMI_CS, s_cs_base & ~SMI_CS_ENABLE);
    int64_t t0 = s_now_ns();
    for (;;) {
        int64_t t = s_now_ns();
        if (!(s_rd(SMI_CS) & (SMI_CS_ENABLE | SMI_CS_ACTIVE))) break;
        if (t - t0 > 100000LL) break;
    }
    s_wr(SMI_L, (uint32_t)transfers);
    s_wr(SMI_CS, s_cs_base | SMI_CS_CLEAR | SMI_CS_ERRORS);
}

/* Abort whatever the SMI is stuck in and leave the pins low. */
static void s_recover(void)
{
    s_set_len(s_info.len_units);
    /* The aborted frame may have stopped on a high sample. */
    s_load_and_start(s_zero_words);
}

bool dshot_busy(void)
{
    if (!s_initialized) return false;
    return s_smi_busy(s_rd(SMI_CS));
}

int dshot_send_raw(const uint16_t *frames)
{
    if (!s_initialized || !frames) return -1;

    uint32_t cs = s_rd(SMI_CS);
    if (s_smi_busy(cs)) {
        s_info.busy_skips++;
        /* Busy long past a frame time means the SMI is stuck. The status is
         * read again after the clock, so that a preemption since the read
         * above cannot make a frame that has since finished look stuck. */
        if (s_now_ns() - s_last_start_ns > s_stall_ns && dshot_busy()) {
            s_info.stall_recover++;
            s_recover();
        }
        return -2;
    }
    if (cs & SMI_CS_ERRORS) {
        s_info.fifo_errors++;
        s_wr(SMI_CS, s_cs_base | (cs & SMI_CS_ERRORS));
    }

    uint32_t w[MAX_WORDS];
    s_build_words(frames, w);
    s_load_and_start(w);
    s_info.sends++;
    return 0;
}

int dshot_send(const uint16_t *values)
{
    if (!s_initialized || !values) return -1;

    uint16_t frames[DSHOT_MAX_MOTORS];
    for (int m = 0; m < s_count; m++)
        frames[m] = s_dshot_frame(values[m]);
    return dshot_send_raw(frames);
}

uint32_t dshot_read_pins(void)
{
    return s_gpio ? s_gpio[REG(GPLEV0)] : 0;
}

/* =========================================================================
 * Init-time checks
 * ========================================================================= */
static void s_dump_regs(const char *label)
{
    fprintf(stderr, "dshot: %s  CS=0x%08X L=%u DSW0=0x%08X FD=0x%08X "
            "CM_CTL=0x%08X CM_DIV=0x%08X\n", label,
            (unsigned)s_rd(SMI_CS), (unsigned)s_rd(SMI_L),
            (unsigned)s_rd(SMI_DSW0), (unsigned)s_rd(SMI_FD),
            (unsigned)s_cm[REG(CM_SMICTL)], (unsigned)s_cm[REG(CM_SMIDIV)]);
}

/* Measure the FIFO: how many words it takes before TXD drops, and how many
 * before it flags an overflow. It keeps taking a couple of words after TXD
 * drops; that headroom is there for DMA beats already under way. */
static int s_probe_fifo(void)
{
    s_wr(SMI_CS, s_cs_base | SMI_CS_CLEAR | SMI_CS_ERRORS);
    if (!(s_rd(SMI_CS) & SMI_CS_TXE)) {
        s_dump_regs("FIFO not empty after CLEAR");
        return -1;
    }

    int n = 0;
    while (n < FIFO_PROBE_MAX && (s_rd(SMI_CS) & SMI_CS_TXD)) {
        s_wr(SMI_D, 0);
        n++;
    }
    if (n == 0 || n >= FIFO_PROBE_MAX) {
        s_dump_regs(n == 0 ? "FIFO takes no data" : "FIFO never reports full");
        return -1;
    }

    int cap = n;
    while (cap < n + FIFO_OVERFILL_MAX) {
        s_wr(SMI_D, 0);
        if (s_rd(SMI_CS) & SMI_CS_AFERR) break;
        cap++;
    }
    if (cap == n + FIFO_OVERFILL_MAX) cap = n;     /* never overflowed: unknown */

    s_wr(SMI_CS, s_cs_base | SMI_CS_CLEAR | SMI_CS_ERRORS);
    if (!(s_rd(SMI_CS) & SMI_CS_TXE)) {
        s_dump_regs("FIFO not empty after CLEAR");
        return -1;
    }
    s_info.fifo_words    = n;
    s_info.fifo_capacity = cap;
    return 0;
}

/* A frame loaded without START has to stay in the FIFO, or its first samples
 * reach the pins before the rest is there. Checked after a finished transfer,
 * which is the state every send starts from. */
static int s_check_hold(void)
{
    int n = (s_info.frame_words < s_info.fifo_words) ? s_info.frame_words
                                                     : s_info.fifo_words;
    for (int i = 0; i < n; i++)
        s_wr(SMI_D, 0);
    s_busy_wait_ns(2LL * s_info.frame_ns + 20000LL);

    uint32_t cs  = s_rd(SMI_CS);
    int      cnt = (int)(s_rd(SMI_FD) & SMI_FD_FCNT_MASK);
    bool     ok  = !(cs & SMI_CS_ACTIVE) && !(cs & SMI_CS_TXE) && cnt == n;
    if (!ok)
        s_dump_regs("FIFO drains before START");

    s_wr(SMI_CS, s_cs_base | SMI_CS_CLEAR | SMI_CS_ERRORS);
    return ok ? 0 : -1;
}

enum { ST_OK = 0, ST_RETRY, ST_EARLY, ST_NO_START, ST_STUCK };

/* Send one all-low frame and time it from START to idle. */
static int s_selftest_once(uint32_t *ns)
{
    int64_t limit = 4LL * s_info.frame_ns + 1000000LL;
    bool    seen_active = false;

    s_load_and_start(s_zero_words);
    int64_t t0 = s_last_start_ns;

    for (;;) {
        int64_t  ta = s_now_ns() - t0;
        uint32_t cs = s_rd(SMI_CS);
        int64_t  tb = s_now_ns() - t0;

        if (cs & SMI_CS_ACTIVE) {
            seen_active = true;
        } else {
            if (cs & SMI_CS_TXE) {
                /* Finished. The time only counts if nothing got in between
                 * the two clock reads around the status. */
                if (tb - ta > SELFTEST_POLL_GAP_NS) return ST_RETRY;
                *ns = (uint32_t)tb;
                return ST_OK;
            }
            if (seen_active) return ST_EARLY;   /* stopped with data left */
        }
        if (ta > limit) {
            if (cs & SMI_CS_ACTIVE) return ST_STUCK;
            int left = (int)(s_rd(SMI_FD) & SMI_FD_FCNT_MASK);
            return (left != (s_info.frame_words & (int)SMI_FD_FCNT_MASK)) ? ST_EARLY
                                                                          : ST_NO_START;
        }
    }
}

/* Find the SMI_L value that makes one frame a complete transfer, and time
 * the frame. SMI_L counts transfers; a transfer that ends short is retried
 * with the byte count, which is what some user-space SMI code passes. */
static int s_selftest(void)
{
    int bytes_per_sample = (s_info.width == 8) ? 1 : (s_info.width == 16) ? 2 : 4;
    int candidates[2]    = { s_info.frame_samples,
                             s_info.frame_samples * bytes_per_sample };
    int n_cand = (candidates[0] == candidates[1]) ? 1 : 2;

    for (int c = 0; c < n_cand; c++) {
        s_info.len_units = candidates[c];
        s_set_len(s_info.len_units);

        uint32_t best = 0;
        int      good = 0, rc = ST_RETRY;
        for (int t = 0; t < SELFTEST_TRIES && good < SELFTEST_RUNS; t++) {
            uint32_t ns = 0;
            rc = s_selftest_once(&ns);
            if (rc == ST_RETRY) continue;
            if (rc != ST_OK) break;
            if (best == 0 || ns < best) best = ns;
            good++;
        }

        if (good > 0 && (rc == ST_OK || rc == ST_RETRY)) {
            if (s_rd(SMI_CS) & SMI_CS_ERRORS) {
                s_dump_regs("self-test frame raised a FIFO/settings error");
                return -1;
            }
            s_info.selftest_ns = best;
            return 0;
        }
        if (rc == ST_EARLY && c + 1 < n_cand)
            continue;

        s_dump_regs(rc == ST_EARLY    ? "self-test frame ended with data left in the FIFO" :
                    rc == ST_NO_START ? "self-test frame never started" :
                    rc == ST_STUCK    ? "self-test frame never finished" :
                                        "self-test frame could not be timed");
        return -1;
    }
    return -1;
}

static bool s_kernel_smi_bound(void)
{
    DIR *d = opendir(KERNEL_SMI_DRIVER);
    if (!d) return false;
    bool bound = false;
    struct dirent *e;
    while ((e = readdir(d)) != NULL) {
        if (strstr(e->d_name, ".smi")) { bound = true; break; }
    }
    closedir(d);
    return bound;
}

/* Program clock, timing and FIFO mode for this many samples per bit and
 * measure the FIFO. Returns 0, 1 when a frame does not fit the FIFO, -1.
 *
 * Bluejay reads a 0 only while its high time lies between its threshold
 * (DShot_Pwm_Thr) and twice that: at DShot600 about 333-625 ns, or 20-37.5 %
 * of the bit. The 37.5 % of the DShot write-ups sits on the upper edge and
 * 25 % close to the lower one, so a slow edge or noise from a running motor
 * pushes either out. 7 samples per bit put a 0 at 2/7 (28.6 %), the middle
 * of that window, and a 1 at 5/7 (71.4 %), close to what Betaflight sends. */
static int s_configure(int spb)
{
    switch (spb) {
        case 7: s_seg_h = 2; s_seg_d = 3; break;
        case 4: s_seg_h = 1; s_seg_d = 2; break;
        default: s_seg_h = 1; s_seg_d = 1; break;
    }
    s_seg_l = spb - s_seg_h - s_seg_d;

    s_info.samples_per_bit = spb;
    s_info.frame_samples   = DSHOT_BITS * spb;
    s_info.frame_words     = s_info.frame_samples / s_info.samples_per_word;

    double pll_ns = 1e9 / (double)s_info.plld_hz;
    double bit_ns = 1e6 / (double)s_cfg.kbit;
    double got_ns = 0.0;
    int    clocks = 0;
    if (s_solve_timing(bit_ns / spb, pll_ns, &s_info.clk_divi, &clocks, &got_ns) < 0) {
        fprintf(stderr, "dshot: no SMI timing for %.1f ns per sample "
                "(PLLD %u Hz)\n", bit_ns / spb, (unsigned)s_info.plld_hz);
        return -1;
    }
    s_info.setup  = clocks / 4;
    if (s_info.setup < 1) s_info.setup = 1;
    s_info.hold   = s_info.setup;
    s_info.strobe = clocks - s_info.setup - s_info.hold;
    s_info.sample_ps = (uint32_t)(got_ns * 1000.0 + 0.5);
    s_info.bit_ns    = (uint32_t)(got_ns * spb + 0.5);
    s_info.t0h_ns    = (uint32_t)(got_ns * s_seg_h + 0.5);
    s_info.t1h_ns    = (uint32_t)(got_ns * (s_seg_h + s_seg_d) + 0.5);
    s_info.frame_ns  = (uint32_t)(got_ns * s_info.frame_samples + 0.5);
    s_stall_ns       = 4LL * s_info.frame_ns + 100000LL;

    s_wr(SMI_CS, 0);
    s_wr(SMI_DCS, 0);
    s_wr(SMI_DMC, 0);
    if (s_clock_start(s_info.clk_divi) < 0) return -1;

    uint32_t ds = (s_width_code << SMI_DS_WIDTH_SHIFT) |
                  ((uint32_t)s_info.setup << SMI_DS_SETUP_SHIFT) |
                  ((uint32_t)s_info.hold  << SMI_DS_HOLD_SHIFT) |
                  (uint32_t)s_info.strobe;
    s_wr(SMI_A, 0);
    s_wr(SMI_DSR0, ds);
    s_wr(SMI_DSW0, ds);
    s_wr(SMI_L, (uint32_t)s_info.frame_samples);

    s_wr(SMI_CS, s_cs_base | SMI_CS_CLEAR | SMI_CS_ERRORS);
    if (s_rd(SMI_CS) & SMI_CS_SETERR) {
        s_dump_regs("settings were not accepted (SETERR)");
        return -1;
    }

    if (s_probe_fifo() < 0) return -1;
    return (s_info.frame_words > s_info.fifo_capacity) ? 1 : 0;
}

static void s_teardown(void)
{
    if (s_smi) {
        s_wr(SMI_DMC, 0);
        s_wr(SMI_CS, 0);
    }
    if (s_cm)
        s_clock_stop();
    s_unmap();
}

static int s_fail(void)
{
    s_teardown();
    return -1;
}

/* =========================================================================
 * Public init / exit
 * ========================================================================= */
int dshot_init(const int *gpios, int count, const DshotConfig *cfg)
{
    if (s_initialized) return 0;
    if (!gpios || !cfg || count < 1 || count > DSHOT_MAX_MOTORS) {
        fprintf(stderr, "dshot_init: motor count 1-%d, got %d\n",
                DSHOT_MAX_MOTORS, count);
        return -1;
    }

    s_cfg = *cfg;
    memset(&s_info, 0, sizeof(s_info));

    if (s_cfg.kbit != 150 && s_cfg.kbit != 300 &&
        s_cfg.kbit != 600 && s_cfg.kbit != 1200) {
        fprintf(stderr, "dshot_init: speed must be 150/300/600/1200, got %d\n",
                s_cfg.kbit);
        return -1;
    }
    if (s_cfg.samples_per_bit != 0 && s_cfg.samples_per_bit != 7 &&
        s_cfg.samples_per_bit != 4 && s_cfg.samples_per_bit != 3) {
        fprintf(stderr, "dshot_init: samples per bit must be 0, 7, 4 or 3, "
                "got %d\n", s_cfg.samples_per_bit);
        return -1;
    }

    int gpio_max = 0;
    s_count     = count;
    s_all_bits  = 0;
    s_gpio_mask = 0;
    for (int i = 0; i < count; i++) {
        int g = gpios[i];
        if (g < DSHOT_GPIO_MIN || g > DSHOT_GPIO_MAX) {
            fprintf(stderr, "dshot_init: GPIO %d is not an SMI data pin "
                    "(GPIO %d-%d)\n", g, DSHOT_GPIO_MIN, DSHOT_GPIO_MAX);
            return -1;
        }
        if (s_gpio_mask & (1u << g)) {
            fprintf(stderr, "dshot_init: GPIO %d given twice\n", g);
            return -1;
        }
        s_gpios[i]   = g;
        s_pin_bit[i] = 1u << (g - DSHOT_GPIO_MIN);
        s_all_bits  |= s_pin_bit[i];
        s_gpio_mask |= 1u << g;
        if (g > gpio_max) gpio_max = g;
    }

    /* Sample order inside a FIFO word. 16 bit: low half first. 8 bit: bytes
     * 1, 0, 3, 2. 18 bit is not packed. */
    if (gpio_max <= DSHOT_GPIO_MIN + 7) {
        s_info.width = 8;
        s_width_code = SMI_WIDTH_CODE_8;
        s_info.samples_per_word = 4;
        s_shift[0] = 8; s_shift[1] = 0; s_shift[2] = 24; s_shift[3] = 16;
    } else if (gpio_max <= DSHOT_GPIO_MIN + 15) {
        s_info.width = 16;
        s_width_code = SMI_WIDTH_CODE_16;
        s_info.samples_per_word = 2;
        s_shift[0] = 0; s_shift[1] = 16;
    } else {
        s_info.width = 18;
        s_width_code = SMI_WIDTH_CODE_18;
        s_info.samples_per_word = 1;
        s_shift[0] = 0;
    }
    s_word_mask  = s_info.samples_per_word - 1;
    s_word_shift = (s_info.samples_per_word == 4) ? 2 :
                   (s_info.samples_per_word == 2) ? 1 : 0;
    s_cs_base = SMI_CS_ENABLE | SMI_CS_WRITE |
                (s_info.samples_per_word > 1 ? SMI_CS_PXLDAT : 0);

    s_info.periph_base = s_periph_base();
    if (!s_info.periph_base) {
        fprintf(stderr, "dshot_init: no BCM2835/2836/2837/2711 peripheral "
                "base in %s (the Raspberry Pi 5 has no SMI)\n", DT_SOC_RANGES);
        return -1;
    }
    if (s_kernel_smi_bound()) {
        fprintf(stderr, "dshot_init: the kernel SMI driver owns the block "
                "(%s); remove dtoverlay=smi\n", KERNEL_SMI_DRIVER);
        return -1;
    }
    if (s_map_all(s_info.periph_base) < 0) return -1;
    s_info.plld_hz = s_plld_hz();

    /* The pins idle low as plain outputs until the SMI is known to work. */
    s_pins_plain_low();

    static const int all_spb[3] = { 7, 3, 4 };
    int rc = 1;
    for (int i = 0; i < 3 && rc == 1; i++) {
        if (s_cfg.samples_per_bit != 0 && s_cfg.samples_per_bit != all_spb[i])
            continue;
        rc = s_configure(all_spb[i]);
    }
    if (rc < 0) return s_fail();
    if (rc == 1) {
        fprintf(stderr, "dshot_init: a frame is %d FIFO words but the FIFO "
                "holds %d. Use fewer samples per bit, or pins on lower GPIOs "
                "(a narrower width packs more samples per word).\n",
                s_info.frame_words, s_info.fifo_capacity);
        return s_fail();
    }

    if (s_selftest() < 0) return s_fail();
    if (s_check_hold() < 0) return s_fail();

    uint32_t tol = s_info.frame_ns / 10 + 3000;
    if (s_info.selftest_ns + tol < s_info.frame_ns ||
        s_info.selftest_ns > s_info.frame_ns + tol) {
        fprintf(stderr, "dshot: ! self-test frame took %u ns, expected %u ns "
                "- the SMI clock is not what was assumed (PLLD %u Hz)\n",
                (unsigned)s_info.selftest_ns, (unsigned)s_info.frame_ns,
                (unsigned)s_info.plld_hz);
    }

    /* The self-test left an all-low sample on the data lines. */
    if (!s_cfg.dry_run) {
        for (int i = 0; i < s_count; i++)
            s_gpio_fsel(s_gpios[i], GPIO_FSEL_ALT1);
    }

    s_initialized = true;

    printf("dshot: init OK  DShot%d  %d samples/bit  width %d (%d/word)\n"
           "  sample %u.%03u ns  bit %u ns (T0H %u, T1H %u)  frame %u ns  "
           "(PLLD %u MHz / %d, setup %d strobe %d hold %d)\n"
           "  FIFO %d words (TXD drops at %d), frame %d words, SMI_L %d, "
           "self-test frame %u ns\n",
           s_cfg.kbit, s_info.samples_per_bit,
           s_info.width, s_info.samples_per_word,
           (unsigned)(s_info.sample_ps / 1000), (unsigned)(s_info.sample_ps % 1000),
           (unsigned)s_info.bit_ns, (unsigned)s_info.t0h_ns, (unsigned)s_info.t1h_ns,
           (unsigned)s_info.frame_ns,
           (unsigned)(s_info.plld_hz / 1000000u), s_info.clk_divi,
           s_info.setup, s_info.strobe, s_info.hold,
           s_info.fifo_capacity, s_info.fifo_words, s_info.frame_words,
           s_info.len_units, (unsigned)s_info.selftest_ns);
    if (s_cfg.dry_run) {
        printf("  dry run: pins not switched, nothing reaches the header\n");
    } else {
        printf("  pins:");
        for (int i = 0; i < s_count; i++)
            printf(" GPIO%d=SD%d", s_gpios[i], s_gpios[i] - DSHOT_GPIO_MIN);
        printf("\n");
    }
    return 0;
}

void dshot_exit(void)
{
    if (!s_initialized) return;

    int64_t t0 = s_now_ns();
    while (dshot_busy() && s_now_ns() - t0 < 2000000LL)
        ;

    s_pins_plain_low();
    s_teardown();

    s_initialized = false;
    s_count = 0;
}

void dshot_get_info(DshotInfo *info)
{
    if (info) *info = s_info;
}

void dshot_dump(void)
{
    if (!s_smi) {
        printf("dshot_dump: not initialized\n");
        return;
    }
    uint32_t cs = s_rd(SMI_CS);
    uint32_t fd = s_rd(SMI_FD);

    printf("---- dshot dump ----\n");
    printf("SMI_CS=0x%08X  enable=%d write=%d pxldat=%d active=%d done=%d "
           "txe=%d txd=%d aferr=%d seterr=%d\n",
           (unsigned)cs, !!(cs & SMI_CS_ENABLE), !!(cs & SMI_CS_WRITE),
           !!(cs & SMI_CS_PXLDAT), !!(cs & SMI_CS_ACTIVE), !!(cs & SMI_CS_DONE),
           !!(cs & SMI_CS_TXE), !!(cs & SMI_CS_TXD),
           !!(cs & SMI_CS_AFERR), !!(cs & SMI_CS_SETERR));
    printf("SMI_L=%u  SMI_DSW0=0x%08X  SMI_FD=0x%08X (count=%u)\n",
           (unsigned)s_rd(SMI_L), (unsigned)s_rd(SMI_DSW0), (unsigned)fd,
           (unsigned)(fd & SMI_FD_FCNT_MASK));
    printf("CM_SMICTL=0x%08X(BUSY=%d)  CM_SMIDIV=0x%08X(DIVI=%u)\n",
           (unsigned)s_cm[REG(CM_SMICTL)], !!(s_cm[REG(CM_SMICTL)] & CM_BUSY),
           (unsigned)s_cm[REG(CM_SMIDIV)],
           (unsigned)((s_cm[REG(CM_SMIDIV)] >> 12) & 0xFu));
    printf("sends=%u busy_skips=%u fifo_errors=%u stall_recover=%u\n",
           (unsigned)s_info.sends, (unsigned)s_info.busy_skips,
           (unsigned)s_info.fifo_errors, (unsigned)s_info.stall_recover);
    printf("------------------------\n");
}
