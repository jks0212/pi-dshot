/*
 * quad_example.c — four ESCs driven from a fixed-rate control loop
 *
 * The shape a flight controller uses the library in:
 *   1. esc_proto_init() once, with the four motor pins
 *   2. arming: MOTOR_STOP on every tick until the ESCs have armed
 *   3. flying: one frame with the four mixer outputs on every tick
 *   4. Ctrl-C: esc_proto_deinit(), which sends MOTOR_STOP for 50 ms and
 *      leaves the pins low
 *
 * The "mixer" here only holds every motor at -f (default 0, which keeps the
 * motors stopped). Take the propellers off before giving it a value.
 *
 * Build  : make example
 * Run    : sudo ./quad_example [-y] [-f <0..1>] [-d <sec>]
 *   -y        dry run: the SMI runs, the pins are not switched
 *   -f <frac> motor output after arming, 0..1 of the throttle range (default 0)
 *   -d <sec>  stop after this long (default: until Ctrl-C)
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <sched.h>
#include <time.h>
#include <unistd.h>
#include <sys/mman.h>
#include "dshot_esc.h"

#define N_MOTORS      4
#define LOOP_HZ       2000           /* control rate; one frame per tick */
#define ARM_SECONDS   3.0            /* MOTOR_STOP stream before throttle */

/* Motor 1..4 -> GPIO. Any of GPIO 8-25; the highest pin sets the SMI width
 * (here GPIO 23: 16 bit, 7 samples per DShot bit). */
static const int MOTOR_GPIO[N_MOTORS] = { 17, 12, 23, 13 };

static volatile sig_atomic_t s_stop = 0;
static void on_signal(int sig) { (void)sig; s_stop = 1; }

static void timespec_add_ns(struct timespec *t, long ns)
{
    t->tv_nsec += ns;
    while (t->tv_nsec >= 1000000000L) {
        t->tv_nsec -= 1000000000L;
        t->tv_sec++;
    }
}

/* 0..1 of the throttle range -> DShot value. 0 is MOTOR_STOP; anything above
 * it maps onto 48..2047, where 48 is the slowest the ESC spins. */
static int frac_to_dshot(const EscProtoCtx *esc, double f)
{
    if (f <= 0.0) return 0;
    if (f > 1.0) f = 1.0;
    int lo = esc_proto_min_us(esc), hi = esc_proto_max_us(esc);
    return lo + (int)(f * (hi - lo) + 0.5);
}

int main(int argc, char **argv)
{
    int    dry = 0;
    double frac = 0.0, run_s = 0.0;

    int opt;
    while ((opt = getopt(argc, argv, "yf:d:")) != -1) {
        switch (opt) {
            case 'y': dry   = 1; break;
            case 'f': frac  = atof(optarg); break;
            case 'd': run_s = atof(optarg); break;
            default:
                fprintf(stderr, "Usage: sudo %s [-y] [-f <0..1>] [-d <sec>]\n", argv[0]);
                return 1;
        }
    }

    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);

    /* A control loop wants no page faults and no time-sharing. */
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
        perror("mlockall");
    struct sched_param sp = { .sched_priority = 80 };
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0)
        perror("sched_setscheduler");

    /* 1. Init: maps the registers, self-tests the SMI, sends one MOTOR_STOP. */
    esc_proto_set_dry_run(dry);
    EscProtoCtx *esc = esc_proto_init(ESC_PROTO_DSHOT300, MOTOR_GPIO, N_MOTORS);
    if (!esc) {
        fprintf(stderr, "esc_proto_init failed (root? dtoverlay=smi?)\n");
        return 1;
    }
    printf("%s on %d motors, %d Hz\n",
           esc_proto_name(ESC_PROTO_DSHOT300), esc_proto_motor_count(esc), LOOP_HZ);

    const long period_ns  = 1000000000L / LOOP_HZ;
    const long arm_ticks  = (long)(ARM_SECONDS * LOOP_HZ);
    const long stop_ticks = (run_s > 0.0) ? (long)(run_s * LOOP_HZ) : -1;

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    for (long tick = 0; !s_stop && tick != stop_ticks; tick++) {
        if (tick < arm_ticks) {
            /* 2. Arming: the ESC arms on a steady stream of MOTOR_STOP.
             * Nothing repeats on its own, so this goes out on every tick. */
            esc_proto_arm(esc);
        } else {
            if (tick == arm_ticks)
                printf("armed, motor output %.3f\n", frac);

            /* 3. Flying: a real controller runs its mixer here. */
            for (int m = 0; m < N_MOTORS; m++)
                esc_proto_set(esc, MOTOR_GPIO[m], frac_to_dshot(esc, frac));
            esc_proto_apply(esc);   /* 1 = previous frame still on the wire */
        }

        timespec_add_ns(&next, period_ns);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
    }

    printf("stopping, %u frames skipped\n", esc_proto_skipped(esc));
    esc_proto_dump(esc);

    /* 4. Exit: MOTOR_STOP for SAFE_END_MILLIS, then the pins go low. */
    esc_proto_deinit(esc);
    return 0;
}
