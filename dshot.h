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

#ifndef DSHOT_H
#define DSHOT_H

#include <stdint.h>
#include <stdbool.h>

/* =========================================================================
 * dshot — DShot frames out of the Secondary Memory Interface
 * =========================================================================
 * Self-contained: it maps the GPIO, SMI and clock-manager registers through
 * /dev/mem itself and needs no kernel driver, DMA channel or other library.
 * Root is required.
 *
 * The SMI block clocks parallel words out on SD0-SD17 (GPIO 8-25) with a
 * cycle time its own clock sets in nanosecond steps. A DShot bit is drawn as
 * 7, 3 or 4 SMI samples. The CPU writes a whole frame into the SMI FIFO and
 * only then starts the transfer, so once a frame is on the wire nothing has
 * to keep up with it and nothing on the bus can stretch a sample.
 *
 * One frame goes out per send and nothing repeats on its own. The caller
 * sends at its control rate; an ESC that stops seeing frames stops the motor.
 *
 * The frame has to fit the SMI FIFO (64 words). Narrower SMI widths pack more
 * samples into a FIFO word, so the pins decide what is available:
 *     GPIO 8-15 (width  8): 7, 3 or 4 samples per bit
 *     GPIO 8-23 (width 16): 7, 3 or 4 samples per bit
 *     GPIO 8-25 (width 18): 3 or 4 samples per bit
 *
 * ! BCM2835/2836/2837/2711 only; the Raspberry Pi 5 has no SMI.
 * ! Only the pins passed to init become SMI outputs. SPI0 sits on GPIO 7-11.
 * ========================================================================= */

#define DSHOT_MAX_MOTORS  8
#define DSHOT_GPIO_MIN    8      /* SD0  */
#define DSHOT_GPIO_MAX    25     /* SD17 */

typedef struct {
    int  kbit;              /* 150 / 300 / 600 / 1200 */
    /* SMI samples per DShot bit. 0 picks the first that fits the FIFO, in
     * this order:
     *   7: T0H 2/7 (28.6 %), T1H 5/7 (71.4 %)
     *   3: T0H 1/3 (33.3 %), T1H 2/3 (66.7 %)
     *   4: T0H 1/4 (25 %),   T1H 3/4 (75 %)
     * Bluejay reads a 0 only for a high time of 20-37.5 % of the bit (about
     * 333-625 ns at DShot600); 7 samples put a 0 in the middle of that. */
    int  samples_per_bit;
    /* Leave the pins alone. The SMI is set up, tested and run exactly the
     * same, but nothing reaches the header. */
    bool dry_run;
} DshotConfig;

typedef struct {
    /* fixed at init */
    int      width;             /* 8, 16 or 18: narrowest reaching every pin */
    int      samples_per_word;
    int      samples_per_bit;
    int      frame_samples;
    int      frame_words;
    int      fifo_words;        /* words the FIFO takes before TXD drops */
    int      fifo_capacity;     /* words it takes before it flags an overflow */
    int      len_units;         /* value SMI_L needs for one frame */
    int      clk_divi, setup, strobe, hold;
    uint32_t periph_base;
    uint32_t plld_hz;
    uint32_t sample_ps;         /* one SMI cycle as programmed */
    uint32_t bit_ns;            /* one DShot bit as generated */
    uint32_t t0h_ns, t1h_ns;    /* high time of a 0 and of a 1 as generated */
    uint32_t frame_ns;          /* 16 bits as generated */
    uint32_t selftest_ns;       /* measured START-to-idle time of one frame */
    /* running counters */
    uint32_t sends;
    uint32_t busy_skips;        /* send found the previous frame still going out */
    uint32_t fifo_errors;       /* SMI flagged a FIFO or settings error */
    uint32_t stall_recover;     /* SMI was reset because it never went idle */
} DshotInfo;

/*
 * Map the registers, set up the SMI clock and timing, measure the FIFO,
 * self-test an all-low frame, then switch the pins to the SMI function
 * (unless dry_run). gpios[] are GPIO 8-25, count 1-DSHOT_MAX_MOTORS.
 * Returns 0 / -1 with the reason on stderr. The pins only become SMI outputs
 * once every check has passed; a failure leaves them untouched or as plain
 * outputs driven low.
 */
int  dshot_init(const int *gpios, int count, const DshotConfig *cfg);

/* Wait for the frame in flight, return the pins to plain outputs driven low,
 * then stop the SMI and its clock. */
void dshot_exit(void);

/*
 * Send one frame. values[] are DShot values (0-2047) in the init gpios[]
 * order; the checksum is added here. No syscalls, so it can be called from a
 * real-time thread. Returns
 *    0  frame loaded and started (it is on the wire for frame_ns)
 *   -1  not initialised / bad argument
 *   -2  the previous frame is still going out; nothing was sent
 */
int  dshot_send(const uint16_t *values);

/* Same, but frames[] are the 16 bits to put on the wire as they are, MSB
 * first. For test patterns. */
int  dshot_send_raw(const uint16_t *frames);

/* Whether a frame is still going out. A register read, no syscalls. */
bool dshot_busy(void);

/* Level of GPIO 0-31 as the pads see it, whatever function drives them. */
uint32_t dshot_read_pins(void);

void dshot_get_info(DshotInfo *info);

/* Print the live SMI/clock registers and the counters. */
void dshot_dump(void);

#endif /* DSHOT_H */
