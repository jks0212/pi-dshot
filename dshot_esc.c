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

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include "dshot.h"
#include "dshot_esc.h"

#define DSHOT_MIN   48
#define DSHOT_MAX   2047

typedef struct {
    const char *name;
    int         kbit;
} EscProtoInfo;

static const EscProtoInfo PROTO_TABLE[ESC_PROTO_COUNT] = {
    [ESC_PROTO_DSHOT150]   = { "DShot150   (value 48-2047)",  150 },
    [ESC_PROTO_DSHOT300]   = { "DShot300   (value 48-2047)",  300 },
    [ESC_PROTO_DSHOT600]   = { "DShot600   (value 48-2047)",  600 },
    [ESC_PROTO_DSHOT1200]  = { "DShot1200  (value 48-2047)", 1200 },
};

struct EscProtoCtx {
    const EscProtoInfo *proto;
    int                 gpios[MAX_MOTORS];
    int                 current_us[MAX_MOTORS];
    int                 count;
    unsigned            skipped;
};

static bool         s_dry_run = false;
static EscProtoCtx *s_active  = NULL;

static const uint16_t s_stop[MAX_MOTORS];

/* 0 sent, 1 skipped (previous frame still on the wire), -1 error. */
static int s_send(EscProtoCtx *ctx, const uint16_t *values)
{
    int rc = dshot_send(values);
    if (rc == -2) {
        ctx->skipped++;
        return 1;
    }
    return rc;
}

static int find_motor(const EscProtoCtx *ctx, int gpio)
{
    for (int i = 0; i < ctx->count; i++) {
        if (ctx->gpios[i] == gpio)
            return i;
    }
    return -1;
}

static int value_ok(int v)
{
    return v == 0 || (v >= DSHOT_MIN && v <= DSHOT_MAX);
}

void esc_proto_set_dry_run(bool on)
{
    s_dry_run = on;
}

EscProtoCtx *esc_proto_init(EscProto proto, const int *gpios, int count)
{
    if (proto < 0 || proto >= ESC_PROTO_COUNT) {
        fprintf(stderr, "esc_proto_init: invalid protocol %d\n", proto);
        return NULL;
    }
    if (!gpios || count < 1 || count > MAX_MOTORS) {
        fprintf(stderr, "esc_proto_init: motor count must be 1-%d, got %d\n",
                MAX_MOTORS, count);
        return NULL;
    }
    if (s_active) {
        fprintf(stderr, "esc_proto_init: a context is already active\n");
        return NULL;
    }

    EscProtoCtx *ctx = calloc(1, sizeof(*ctx));
    if (!ctx) return NULL;

    ctx->proto = &PROTO_TABLE[proto];
    ctx->count = count;
    for (int i = 0; i < count; i++) {
        ctx->gpios[i]      = gpios[i];
        ctx->current_us[i] = DSHOT_MIN;
    }

    DshotConfig cfg = { .kbit = ctx->proto->kbit, .dry_run = s_dry_run };
    if (dshot_init(ctx->gpios, count, &cfg) < 0) {
        fprintf(stderr, "esc_proto_init: dshot_init failed\n");
        free(ctx);
        return NULL;
    }

    /* Pins low is not a frame. Start the line off with one MOTOR_STOP; the
     * spinning values only go out from esc_proto_apply(). */
    s_send(ctx, s_stop);
    s_active = ctx;
    return ctx;
}

void esc_proto_deinit(EscProtoCtx *ctx)
{
    if (!ctx) return;

    for (int ms = 0; ms < SAFE_END_MILLIS; ms++) {
        s_send(ctx, s_stop);
        usleep(1000);
    }
    dshot_exit();

    if (s_active == ctx) s_active = NULL;
    free(ctx);
}

int esc_proto_set(EscProtoCtx *ctx, int gpio, int pulse_us)
{
    if (!ctx) return -1;

    if (!value_ok(pulse_us)) {
        fprintf(stderr, "esc_proto_set: GPIO %d value %d out of range [%d, %d]\n",
                gpio, pulse_us, DSHOT_MIN, DSHOT_MAX);
        return -1;
    }
    int idx = find_motor(ctx, gpio);
    if (idx < 0) {
        fprintf(stderr, "esc_proto_set: GPIO %d not registered\n", gpio);
        return -1;
    }
    ctx->current_us[idx] = pulse_us;
    return 0;
}

int esc_proto_set_all(EscProtoCtx *ctx, int pulse_us)
{
    if (!ctx) return -1;

    if (!value_ok(pulse_us)) {
        fprintf(stderr, "esc_proto_set_all: value %d out of range [%d, %d]\n",
                pulse_us, DSHOT_MIN, DSHOT_MAX);
        return -1;
    }
    for (int i = 0; i < ctx->count; i++)
        ctx->current_us[i] = pulse_us;
    return 0;
}

int esc_proto_apply(EscProtoCtx *ctx)
{
    if (!ctx) return -1;

    uint16_t vals[MAX_MOTORS];
    for (int i = 0; i < ctx->count; i++)
        vals[i] = (uint16_t)ctx->current_us[i];
    return s_send(ctx, vals);
}

int esc_proto_arm(EscProtoCtx *ctx)
{
    if (!ctx) return -1;

    for (int i = 0; i < ctx->count; i++)
        ctx->current_us[i] = DSHOT_MIN;
    return s_send(ctx, s_stop);
}

int esc_proto_stop(EscProtoCtx *ctx)
{
    if (!ctx) return -1;
    return s_send(ctx, s_stop);
}

int esc_proto_min_us(const EscProtoCtx *ctx)
{
    return ctx ? DSHOT_MIN : -1;
}

int esc_proto_max_us(const EscProtoCtx *ctx)
{
    return ctx ? DSHOT_MAX : -1;
}

int esc_proto_motor_count(const EscProtoCtx *ctx)
{
    return ctx ? ctx->count : 0;
}

int esc_proto_current_us(const EscProtoCtx *ctx, int motor_idx)
{
    if (!ctx || motor_idx < 0 || motor_idx >= ctx->count) return -1;
    return ctx->current_us[motor_idx];
}

const char *esc_proto_name(EscProto proto)
{
    if (proto < 0 || proto >= ESC_PROTO_COUNT) return "(unknown)";
    return PROTO_TABLE[proto].name;
}

int esc_proto_is_digital(const EscProtoCtx *ctx)
{
    return ctx ? 1 : 0;
}

void esc_proto_dump(const EscProtoCtx *ctx)
{
    if (!ctx) return;
    dshot_dump();
    printf("esc: %s, %d motors, skipped frames %u\n",
           ctx->proto->name, ctx->count, ctx->skipped);
}

unsigned esc_proto_skipped(const EscProtoCtx *ctx)
{
    return ctx ? ctx->skipped : 0;
}
