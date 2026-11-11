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
 * dshot_esc.h — DShot ESC protocol library
 *
 * Per-motor interface over dshot.h. A program includes this header and
 * compiles dshot_esc.c + dshot.c.
 *
 *   - DShot150/300/600/1200.
 *   - Pins are GPIO 8-25 (SMI data lines).
 *   - Nothing repeats on its own. Every apply/arm/stop puts exactly one frame
 *     on the wire, so the caller has to keep calling esc_proto_apply() at its
 *     control rate, also while idle. An ESC that stops seeing frames stops
 *     the motor.
 */

#ifndef DSHOT_ESC_H
#define DSHOT_ESC_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Values are raw DShot throttle, not microseconds despite the _us names:
 * min_us/max_us read 48 and 2047, and 0 is MOTOR_STOP. */
typedef enum {
    ESC_PROTO_DSHOT150   = 0,
    ESC_PROTO_DSHOT300   = 1,
    ESC_PROTO_DSHOT600   = 2,
    ESC_PROTO_DSHOT1200  = 3,
    ESC_PROTO_COUNT
} EscProto;

#define MAX_MOTORS  8

/* How long esc_proto_deinit() keeps sending MOTOR_STOP before it lets go of
 * the pins. */
#define SAFE_END_MILLIS 50

typedef struct EscProtoCtx EscProtoCtx;

/*
 *   gpios[] : array of GPIO numbers (8-25)
 *   count   : 1-MAX_MOTORS
 * Maps the registers, sets up and self-tests the SMI, then sends one
 * MOTOR_STOP frame. Needs root. Returns NULL on failure. Only one context
 * may exist at a time.
 */
EscProtoCtx *esc_proto_init(EscProto proto, const int *gpios, int count);

/* Sends MOTOR_STOP for SAFE_END_MILLIS, then leaves the pins as plain outputs
 * driven low. Sleeps, so not for a real-time thread. NULL-safe. */
void esc_proto_deinit(EscProtoCtx *ctx);

/*
 * pulse_us is a DShot value within [min_us, max_us], or 0 (MOTOR_STOP).
 * Changes are buffered until esc_proto_apply(). Returns 0 / -1 (unknown gpio
 * or out of range).
 */
int esc_proto_set(EscProtoCtx *ctx, int gpio, int pulse_us);
int esc_proto_set_all(EscProtoCtx *ctx, int pulse_us);

/*
 * Put one frame with the buffered values on the wire. No syscalls, so it can
 * be called from a real-time thread. Returns
 *    0  frame started
 *    1  the previous frame was still going out, so this one was skipped
 *   -1  error
 * Called slower than one frame time it never returns 1.
 */
int esc_proto_apply(EscProtoCtx *ctx);

/* Set every motor to min_us and send one MOTOR_STOP frame. An ESC arms on a
 * stream of them, so keep calling this (or apply with value 0) until it has. */
int esc_proto_arm(EscProtoCtx *ctx);

/* Send one MOTOR_STOP frame. The buffered values are kept. */
int esc_proto_stop(EscProtoCtx *ctx);

int esc_proto_min_us(const EscProtoCtx *ctx);
int esc_proto_max_us(const EscProtoCtx *ctx);
int esc_proto_motor_count(const EscProtoCtx *ctx);
int esc_proto_current_us(const EscProtoCtx *ctx, int motor_idx);
const char *esc_proto_name(EscProto proto);
int esc_proto_is_digital(const EscProtoCtx *ctx);

/* Print the SMI registers and counters. NULL-safe. */
void esc_proto_dump(const EscProtoCtx *ctx);

/* Call before esc_proto_init(). With dry run on, the SMI is set up and run
 * the same but the pins are left alone. Off by default. */
void esc_proto_set_dry_run(bool on);

/* Frames skipped because the previous one was still going out. */
unsigned esc_proto_skipped(const EscProtoCtx *ctx);

#ifdef __cplusplus
}
#endif

#endif /* DSHOT_ESC_H */
