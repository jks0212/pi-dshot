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

/*
 * dshot_test.c — bench test for the DShot ESC library (dshot_esc.h)
 *
 * Drives the motors through the esc_proto_* interface at a fixed rate and
 * prints, once a second, how long an apply takes and how many frames were
 * skipped.
 *
 * Build  : make
 * Run    : sudo ./dshot_test [options] <gpio> [<gpio> ...]     (GPIO 8-25)
 *
 *   -s <150|300|600|1200>  DShot speed (default 300)
 *   -r <hz>                apply calls per second (default 4000)
 *   -t <value>             DShot value after arming: 0 or 48-2047 (default 0)
 *   -a <sec>               MOTOR_STOP time before -t takes effect (default 3)
 *   -d <sec>               stop after this long (default: until Ctrl-C)
 *   -y                     dry run: the SMI runs, the pins are not switched
 *   -l                     loopback check, then exit: read the pins back while
 *                          test frames go out and decode them by pulse width.
 *                          The frames carry a bad checksum, so an ESC on the
 *                          line ignores them.
 *
 * First run on a board:   sudo ./dshot_test -y -d 3 12 13
 * Pins without a scope:   sudo ./dshot_test -l -s 300 12 13
 * Motor (propellers off): sudo ./dshot_test -s 300 -t 150 12
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <sched.h>
#include <time.h>
#include <unistd.h>
#include <sys/mman.h>
#include "dshot.h"
#include "dshot_esc.h"

#define MIN_IDLE_NS     20000LL     /* gap an ESC needs between frames */
#define RT_PRIORITY     80
#define LB_MAX_SAMPLES  65536
#define LB_TRIES        5
#define LB_BITS         16

static volatile sig_atomic_t s_stop = 0;

static void on_signal(int sig) { (void)sig; s_stop = 1; }

static int64_t now_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000000000LL + t.tv_nsec;
}

static void sleep_until(int64_t t_ns)
{
    struct timespec t = { .tv_sec  = (time_t)(t_ns / 1000000000LL),
                          .tv_nsec = (long)(t_ns % 1000000000LL) };
    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL) != 0 && !s_stop)
        ;
}

/* =========================================================================
 * Loopback: the GPIO level register shows a pin's state whatever function
 * drives it, so a frame can be read back while the SMI puts it out. The read
 * loop samples every 50-250 ns: enough to tell a 0 from a 1 by its high time
 * and to see a missing, extra or misplaced pulse, not to measure jitter.
 * ========================================================================= */
typedef struct {
    int      ok, pulses, idle_high;
    uint16_t got;
    double   t0h_ns, t1h_ns, bit_ns;
} LbPin;

static uint32_t s_lb[LB_MAX_SAMPLES];

static uint16_t bad_checksum(uint16_t frame)
{
    uint16_t packet = frame >> 4;
    int ok = ((packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F) == (frame & 0x0F);
    return ok ? (uint16_t)(frame ^ 1u) : frame;
}

static int loopback_once(const int *gpios, int count, const uint16_t *frames,
                         const DshotInfo *info, LbPin *res, double *sample_ns)
{
    int64_t span = (int64_t)info->frame_ns + 30000;

    while (dshot_busy())
        ;
    if (dshot_send_raw(frames) != 0) return -1;

    int64_t t0 = now_ns(), t1 = t0;
    int     n  = 0;
    while (n < LB_MAX_SAMPLES) {
        s_lb[n++] = dshot_read_pins();
        if ((n & 15) == 0) {
            t1 = now_ns();
            if (t1 - t0 > span) break;
        }
    }
    t1 = now_ns();
    double dt  = (double)(t1 - t0) / n;
    double thr = (info->t0h_ns + info->t1h_ns) / 2.0;
    *sample_ns = dt;

    for (int m = 0; m < count; m++) {
        uint32_t mask = 1u << gpios[m];
        int rise[LB_BITS], fall[LB_BITS], np = 0;

        for (int i = 0; i < n; ) {
            if (!(s_lb[i] & mask)) { i++; continue; }
            int s = i;
            while (i < n && (s_lb[i] & mask)) i++;
            if (np < LB_BITS) { rise[np] = s; fall[np] = i; }
            np++;
        }

        LbPin *r = &res[m];
        memset(r, 0, sizeof(*r));
        r->pulses    = np;
        r->idle_high = (s_lb[n - 1] & mask) != 0;
        if (np != LB_BITS) continue;

        /* Sampling starts just after START, so the first rising edge is
         * usually already gone. Place it one bit before the second. */
        double bit_samples = (double)(rise[LB_BITS - 1] - rise[1]) / (LB_BITS - 2);
        double first_rise  = (rise[0] > 0) ? rise[0] : rise[1] - bit_samples;
        double sum0 = 0, sum1 = 0;
        int    n0 = 0, n1 = 0;
        for (int k = 0; k < LB_BITS; k++) {
            double high = (fall[k] - (k == 0 ? first_rise : rise[k])) * dt;
            int    bit  = high > thr;
            r->got = (uint16_t)((r->got << 1) | bit);
            if (bit) { sum1 += high; n1++; } else { sum0 += high; n0++; }
        }
        r->t0h_ns = n0 ? sum0 / n0 : 0;
        r->t1h_ns = n1 ? sum1 / n1 : 0;
        r->bit_ns = bit_samples * dt;
        r->ok     = (r->got == frames[m]) && !r->idle_high;
    }
    return 0;
}

/* Two frame sets, the second the complement of the first, so every bit of
 * every pin goes out as both a 0 and a 1. */
static int loopback_check(const int *gpios, int count)
{
    static const uint16_t pat[4] = { 0xA5C0, 0x3C96, 0x0FF0, 0x9669 };
    DshotInfo info;
    int all_ok = 1;

    dshot_get_info(&info);
    printf("loopback: reading GPIO levels back while frames go out "
           "(expect T0H %u ns, T1H %u ns, bit %u ns)\n",
           (unsigned)info.t0h_ns, (unsigned)info.t1h_ns, (unsigned)info.bit_ns);

    for (int set = 0; set < 2; set++) {
        uint16_t frames[MAX_MOTORS];
        LbPin    res[MAX_MOTORS];
        double   dt = 0;
        int      pass = 0, tries = 0;

        for (int m = 0; m < count; m++) {
            uint16_t f = pat[m % 4];
            frames[m] = bad_checksum(set ? (uint16_t)~f : f);
        }
        /* A preempted read loop spoils one attempt; another settles it. */
        while (tries < LB_TRIES && !pass) {
            tries++;
            if (loopback_once(gpios, count, frames, &info, res, &dt) < 0) return -1;
            pass = 1;
            for (int m = 0; m < count; m++)
                if (!res[m].ok) pass = 0;
        }

        printf("  set %d, %.0f ns per sample, %d %s\n", set + 1, dt, tries,
               tries == 1 ? "try" : "tries");
        for (int m = 0; m < count; m++) {
            const LbPin *r = &res[m];
            if (r->pulses != LB_BITS)
                printf("    GPIO%-2d sent 0x%04X  read %d pulses instead of %d%s  FAIL\n",
                       gpios[m], frames[m], r->pulses, LB_BITS,
                       r->idle_high ? ", line left high" : "");
            else
                printf("    GPIO%-2d sent 0x%04X  read 0x%04X  T0H %.0f  T1H %.0f  "
                       "bit %.0f ns%s  %s\n",
                       gpios[m], frames[m], r->got, r->t0h_ns, r->t1h_ns, r->bit_ns,
                       r->idle_high ? "  line left high" : "", r->ok ? "ok" : "FAIL");
        }
        if (dt * 3.0 > info.t0h_ns)
            printf("    ! %.0f ns per sample is coarse against a %u ns T0H; "
                   "a slower -s gives a firmer check\n", dt, (unsigned)info.t0h_ns);
        if (!pass) all_ok = 0;
    }

    printf("loopback: %s\n", all_ok ? "PASS" : "FAIL");
    return all_ok ? 0 : 1;
}

static void usage(const char *prog)
{
    fprintf(stderr,
        "Usage: sudo %s [options] <gpio> [<gpio> ...]     (GPIO %d-%d)\n"
        "  -s <150|300|600|1200>  DShot speed (default 300)\n"
        "  -r <hz>                apply calls per second (default 4000)\n"
        "  -t <value>             DShot value after arming: 0 or 48-2047 (default 0)\n"
        "  -a <sec>               MOTOR_STOP time before -t takes effect (default 3)\n"
        "  -d <sec>               stop after this long (default: until Ctrl-C)\n"
        "  -y                     dry run: the SMI runs, the pins are not switched\n"
        "  -l                     loopback check (read the pins back), then exit\n",
        prog, DSHOT_GPIO_MIN, DSHOT_GPIO_MAX);
}

int main(int argc, char **argv)
{
    int    kbit = 300, rate_hz = 4000, throttle = 0;
    int    dry = 0, loopback = 0;
    double arm_s = 3.0, run_s = 0.0;

    int opt;
    while ((opt = getopt(argc, argv, "s:r:t:a:d:ylh")) != -1) {
        switch (opt) {
            case 's': kbit     = atoi(optarg); break;
            case 'r': rate_hz  = atoi(optarg); break;
            case 't': throttle = atoi(optarg); break;
            case 'a': arm_s    = atof(optarg); break;
            case 'd': run_s    = atof(optarg); break;
            case 'y': dry      = 1; break;
            case 'l': loopback = 1; break;
            default:  usage(argv[0]); return 1;
        }
    }

    int gpios[MAX_MOTORS];
    int count = argc - optind;
    if (count < 1 || count > MAX_MOTORS) {
        usage(argv[0]);
        return 1;
    }
    for (int i = 0; i < count; i++)
        gpios[i] = atoi(argv[optind + i]);

    EscProto proto = kbit == 150  ? ESC_PROTO_DSHOT150 :
                     kbit == 300  ? ESC_PROTO_DSHOT300 :
                     kbit == 600  ? ESC_PROTO_DSHOT600 :
                     kbit == 1200 ? ESC_PROTO_DSHOT1200 : ESC_PROTO_COUNT;
    if (proto == ESC_PROTO_COUNT) {
        fprintf(stderr, "-s takes 150, 300, 600 or 1200\n");
        return 1;
    }
    if (rate_hz < 1 || rate_hz > 100000) {
        fprintf(stderr, "rate %d out of range\n", rate_hz);
        return 1;
    }
    if (throttle != 0 && (throttle < 48 || throttle > 2047)) {
        fprintf(stderr, "-t takes 0 (MOTOR_STOP) or 48-2047, got %d\n", throttle);
        return 1;
    }
    if (loopback && dry) {
        fprintf(stderr, "-l reads the pins, so it cannot go with -y\n");
        return 1;
    }

    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
        perror("mlockall");
    struct sched_param sp = { .sched_priority = RT_PRIORITY };
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0)
        perror("sched_setscheduler(SCHED_FIFO)");

    esc_proto_set_dry_run(dry);
    EscProtoCtx *esc = esc_proto_init(proto, gpios, count);
    if (!esc) return 1;
    printf("%s\n", esc_proto_name(proto));

    if (loopback) {
        int rc = loopback_check(gpios, count);
        esc_proto_deinit(esc);
        return rc == 0 ? 0 : 2;
    }

    DshotInfo info;
    dshot_get_info(&info);
    int64_t period_ns = 1000000000LL / rate_hz;
    if (period_ns < (int64_t)info.frame_ns + MIN_IDLE_NS) {
        fprintf(stderr, "%d Hz leaves no gap after a %u ns frame\n",
                rate_hz, (unsigned)info.frame_ns);
        esc_proto_deinit(esc);
        return 1;
    }

    printf("value %d after %.1f s of MOTOR_STOP, %d Hz\n", throttle, arm_s, rate_hz);
    if (!dry && throttle != 0)
        printf("! The motor will spin. Propellers off.\n");

    int64_t  t_start = now_ns(), t_next = t_start, t_report = t_start;
    int64_t  ap_min = 0, ap_max = 0, ap_sum = 0, late_max = 0;
    uint32_t sent = 0, skipped = 0, total = 0;

    while (!s_stop) {
        t_next += period_ns;
        sleep_until(t_next);

        int64_t now = now_ns();
        int64_t lag = now - t_next;
        if (lag > late_max) late_max = lag;
        if (lag > 10 * period_ns) t_next = now;     /* fell far behind: resync */
        if (run_s > 0 && (now - t_start) >= (int64_t)(run_s * 1e9)) break;

        int armed = (now - t_start) >= (int64_t)(arm_s * 1e9);
        esc_proto_set_all(esc, armed ? throttle : 0);

        int64_t a  = now_ns();
        int     rc = esc_proto_apply(esc);
        int64_t d  = now_ns() - a;
        if (rc < 0) {
            fprintf(stderr, "esc_proto_apply failed\n");
            break;
        }
        if (rc == 1) {
            skipped++;
        } else {
            if (sent == 0 || d < ap_min) ap_min = d;
            if (sent == 0 || d > ap_max) ap_max = d;
            ap_sum += d;
            sent++;
        }

        if (now - t_report >= 1000000000LL) {
            dshot_get_info(&info);
            printf("[%4lld s] sent %u skipped %u | apply %.1f/%.1f/%.1f us | "
                   "late max %.0f us | fifo_err %u stall %u\n",
                   (long long)((now - t_start) / 1000000000LL), sent, skipped,
                   ap_min / 1000.0, sent ? ap_sum / 1000.0 / sent : 0.0, ap_max / 1000.0,
                   late_max / 1000.0,
                   (unsigned)info.fifo_errors, (unsigned)info.stall_recover);
            fflush(stdout);
            total += sent;
            sent = skipped = 0;
            ap_sum = 0;
            late_max = 0;
            t_report = now;
        }
    }
    total += sent;

    printf("\nframes sent %u, skipped %u\n", total, esc_proto_skipped(esc));
    esc_proto_dump(esc);
    esc_proto_deinit(esc);
    return 0;
}
