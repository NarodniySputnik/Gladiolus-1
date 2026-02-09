/**
 * SSTV Mini Library - Standalone C implementation (header-only)
 * Based on RadioLib SSTV/AFSK implementation
 * Compatible with STM32Cube and other bare-metal environments
 *
 * Usage:
 *   1. #define SSTV_MINI_IMPLEMENTATION in ONE .c/.cpp file before including
 *   2. Implement tone_func and delay_us_func callbacks
 *   3. Call sstv_init() with mode and callbacks
 *   4. Call sstv_send_header()
 *   5. Call sstv_send_line() for each line
 *   6. Call sstv_no_tone() when done
 */

#ifndef SSTV_MINI_H
#define SSTV_MINI_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Error codes */
#define SSTV_ERR_NONE           0
#define SSTV_ERR_NOT_INIT       -1
#define SSTV_ERR_INVALID_FREQ   -2

/* VIS codes */
#define SSTV_VIS_SCOTTIE_1      60
#define SSTV_VIS_SCOTTIE_2      56
#define SSTV_VIS_SCOTTIE_DX     76
#define SSTV_VIS_MARTIN_1       44
#define SSTV_VIS_MARTIN_2       40
#define SSTV_VIS_WRASSE_SC2_180 55
#define SSTV_VIS_PASOKON_P3     113
#define SSTV_VIS_PASOKON_P5     114
#define SSTV_VIS_PASOKON_P7     115
#define SSTV_VIS_ROBOT_36       8
#define SSTV_VIS_ROBOT_72       12

/* SSTV tones in Hz */
#define SSTV_TONE_LEADER        1900
#define SSTV_TONE_BREAK         1200
#define SSTV_TONE_VIS_1         1100
#define SSTV_TONE_VIS_0         1300
#define SSTV_TONE_BRIGHTNESS_MIN 1500
#define SSTV_TONE_BRIGHTNESS_MAX 2300

/* Header timing in microseconds */
#define SSTV_HEADER_LEADER_LEN  300000
#define SSTV_HEADER_BREAK_LEN   10000
#define SSTV_HEADER_BIT_LEN     30000

/* Maximum tones per line */
#define SSTV_MAX_TONES          9

/* Tone types */
typedef enum {
    SSTV_TONE_GENERIC = 0,
    SSTV_TONE_SCAN_GREEN_Y,
    SSTV_TONE_SCAN_BLUE_CB,
    SSTV_TONE_SCAN_RED_CR
} sstv_tone_type_t;

/* Tone structure */
typedef struct {
    sstv_tone_type_t type;
    uint32_t len;   /* Length in microseconds, 0 for scan tones */
    uint16_t freq;  /* Frequency in Hz, 0 for scan tones */
} sstv_tone_t;

/* SSTV mode structure */
typedef struct {
    uint8_t vis_code;
    uint16_t width;
    uint16_t height;
    uint16_t scan_pixel_len;
    uint8_t num_tones;
    sstv_tone_t tones[SSTV_MAX_TONES];
} sstv_mode_t;

/* Callback function types */
typedef void (*sstv_tone_func_t)(uint16_t freq);
typedef void (*sstv_no_tone_func_t)(void);
typedef void (*sstv_delay_us_func_t)(uint32_t us);
typedef uint32_t (*sstv_micros_func_t)(void);

/* SSTV context structure */
typedef struct {
    sstv_mode_t mode;
    uint32_t line_count;
    float correction;

    /* Callbacks */
    sstv_tone_func_t tone_func;
    sstv_no_tone_func_t no_tone_func;
    sstv_delay_us_func_t delay_us_func;
    sstv_micros_func_t micros_func;

    bool initialized;
} sstv_ctx_t;

/* Predefined SSTV modes */
extern const sstv_mode_t SSTV_MODE_SCOTTIE_1;
extern const sstv_mode_t SSTV_MODE_SCOTTIE_2;
extern const sstv_mode_t SSTV_MODE_SCOTTIE_DX;
extern const sstv_mode_t SSTV_MODE_MARTIN_1;
extern const sstv_mode_t SSTV_MODE_MARTIN_2;
extern const sstv_mode_t SSTV_MODE_WRASSE;
extern const sstv_mode_t SSTV_MODE_PASOKON_P3;
extern const sstv_mode_t SSTV_MODE_PASOKON_P5;
extern const sstv_mode_t SSTV_MODE_PASOKON_P7;
extern const sstv_mode_t SSTV_MODE_ROBOT_36;
extern const sstv_mode_t SSTV_MODE_ROBOT_72;

/* Function declarations */
int16_t sstv_init(sstv_ctx_t *ctx, const sstv_mode_t *mode,
                  sstv_tone_func_t tone_fn, sstv_no_tone_func_t no_tone_fn,
                  sstv_delay_us_func_t delay_us_fn, sstv_micros_func_t micros_fn);
int16_t sstv_set_correction(sstv_ctx_t *ctx, float correction);
void sstv_idle(sstv_ctx_t *ctx);
void sstv_send_header(sstv_ctx_t *ctx);
void sstv_send_line(sstv_ctx_t *ctx, const uint32_t *img_line);
void sstv_no_tone(sstv_ctx_t *ctx);
uint16_t sstv_get_picture_height(const sstv_ctx_t *ctx);
uint16_t sstv_get_picture_width(const sstv_ctx_t *ctx);

#ifdef __cplusplus
}
#endif

/* ========================================================================== */
/*                              IMPLEMENTATION                                 */
/* ========================================================================== */
#ifdef SSTV_MINI_IMPLEMENTATION

#ifdef __cplusplus
extern "C" {
#endif

/* Predefined SSTV modes */
const sstv_mode_t SSTV_MODE_SCOTTIE_1 = {
    .vis_code = SSTV_VIS_SCOTTIE_1, .width = 320, .height = 256, .scan_pixel_len = 432, .num_tones = 7,
    .tones = {
        { SSTV_TONE_GENERIC, 1500, 1500 }, { SSTV_TONE_SCAN_GREEN_Y, 0, 0 },
        { SSTV_TONE_GENERIC, 1500, 1500 }, { SSTV_TONE_SCAN_BLUE_CB, 0, 0 },
        { SSTV_TONE_GENERIC, 9000, 1200 }, { SSTV_TONE_GENERIC, 1500, 1500 },
        { SSTV_TONE_SCAN_RED_CR, 0, 0 }
    }
};

const sstv_mode_t SSTV_MODE_SCOTTIE_2 = {
    .vis_code = SSTV_VIS_SCOTTIE_2, .width = 320, .height = 256, .scan_pixel_len = 275, .num_tones = 7,
    .tones = {
        { SSTV_TONE_GENERIC, 1500, 1500 }, { SSTV_TONE_SCAN_GREEN_Y, 0, 0 },
        { SSTV_TONE_GENERIC, 1500, 1500 }, { SSTV_TONE_SCAN_BLUE_CB, 0, 0 },
        { SSTV_TONE_GENERIC, 9000, 1200 }, { SSTV_TONE_GENERIC, 1500, 1500 },
        { SSTV_TONE_SCAN_RED_CR, 0, 0 }
    }
};

const sstv_mode_t SSTV_MODE_SCOTTIE_DX = {
    .vis_code = SSTV_VIS_SCOTTIE_DX, .width = 320, .height = 256, .scan_pixel_len = 1080, .num_tones = 7,
    .tones = {
        { SSTV_TONE_GENERIC, 1500, 1500 }, { SSTV_TONE_SCAN_GREEN_Y, 0, 0 },
        { SSTV_TONE_GENERIC, 1500, 1500 }, { SSTV_TONE_SCAN_BLUE_CB, 0, 0 },
        { SSTV_TONE_GENERIC, 9000, 1200 }, { SSTV_TONE_GENERIC, 1500, 1500 },
        { SSTV_TONE_SCAN_RED_CR, 0, 0 }
    }
};

const sstv_mode_t SSTV_MODE_MARTIN_1 = {
    .vis_code = SSTV_VIS_MARTIN_1, .width = 320, .height = 256, .scan_pixel_len = 458, .num_tones = 8,
    .tones = {
        { SSTV_TONE_GENERIC, 4862, 1200 }, { SSTV_TONE_GENERIC, 572, 1500 },
        { SSTV_TONE_SCAN_GREEN_Y, 0, 0 },  { SSTV_TONE_GENERIC, 572, 1500 },
        { SSTV_TONE_SCAN_BLUE_CB, 0, 0 },  { SSTV_TONE_GENERIC, 572, 1500 },
        { SSTV_TONE_SCAN_RED_CR, 0, 0 },   { SSTV_TONE_GENERIC, 572, 1500 }
    }
};

const sstv_mode_t SSTV_MODE_MARTIN_2 = {
    .vis_code = SSTV_VIS_MARTIN_2, .width = 320, .height = 256, .scan_pixel_len = 229, .num_tones = 8,
    .tones = {
        { SSTV_TONE_GENERIC, 4862, 1200 }, { SSTV_TONE_GENERIC, 572, 1500 },
        { SSTV_TONE_SCAN_GREEN_Y, 0, 0 },  { SSTV_TONE_GENERIC, 572, 1500 },
        { SSTV_TONE_SCAN_BLUE_CB, 0, 0 },  { SSTV_TONE_GENERIC, 572, 1500 },
        { SSTV_TONE_SCAN_RED_CR, 0, 0 },   { SSTV_TONE_GENERIC, 572, 1500 }
    }
};

const sstv_mode_t SSTV_MODE_WRASSE = {
    .vis_code = SSTV_VIS_WRASSE_SC2_180, .width = 320, .height = 256, .scan_pixel_len = 734, .num_tones = 5,
    .tones = {
        { SSTV_TONE_GENERIC, 5523, 1200 }, { SSTV_TONE_GENERIC, 500, 1500 },
        { SSTV_TONE_SCAN_RED_CR, 0, 0 },   { SSTV_TONE_SCAN_GREEN_Y, 0, 0 },
        { SSTV_TONE_SCAN_BLUE_CB, 0, 0 }
    }
};

const sstv_mode_t SSTV_MODE_PASOKON_P3 = {
    .vis_code = SSTV_VIS_PASOKON_P3, .width = 640, .height = 496, .scan_pixel_len = 208, .num_tones = 7,
    .tones = {
        { SSTV_TONE_GENERIC, 5208, 1200 }, { SSTV_TONE_GENERIC, 1042, 1500 },
        { SSTV_TONE_SCAN_RED_CR, 0, 0 },   { SSTV_TONE_GENERIC, 1042, 1500 },
        { SSTV_TONE_SCAN_GREEN_Y, 0, 0 },  { SSTV_TONE_GENERIC, 1042, 1500 },
        { SSTV_TONE_SCAN_BLUE_CB, 0, 0 }
    }
};

const sstv_mode_t SSTV_MODE_PASOKON_P5 = {
    .vis_code = SSTV_VIS_PASOKON_P5, .width = 640, .height = 496, .scan_pixel_len = 312, .num_tones = 7,
    .tones = {
        { SSTV_TONE_GENERIC, 7813, 1200 }, { SSTV_TONE_GENERIC, 1563, 1500 },
        { SSTV_TONE_SCAN_RED_CR, 0, 0 },   { SSTV_TONE_GENERIC, 1563, 1500 },
        { SSTV_TONE_SCAN_GREEN_Y, 0, 0 },  { SSTV_TONE_GENERIC, 1563, 1500 },
        { SSTV_TONE_SCAN_BLUE_CB, 0, 0 }
    }
};

const sstv_mode_t SSTV_MODE_PASOKON_P7 = {
    .vis_code = SSTV_VIS_PASOKON_P7, .width = 640, .height = 496, .scan_pixel_len = 417, .num_tones = 7,
    .tones = {
        { SSTV_TONE_GENERIC, 10417, 1200 }, { SSTV_TONE_GENERIC, 2083, 1500 },
        { SSTV_TONE_SCAN_RED_CR, 0, 0 },    { SSTV_TONE_GENERIC, 2083, 1500 },
        { SSTV_TONE_SCAN_GREEN_Y, 0, 0 },   { SSTV_TONE_GENERIC, 2083, 1500 },
        { SSTV_TONE_SCAN_BLUE_CB, 0, 0 }
    }
};

const sstv_mode_t SSTV_MODE_ROBOT_36 = {
    .vis_code = SSTV_VIS_ROBOT_36, .width = 320, .height = 240, .scan_pixel_len = 275, .num_tones = 6,
    .tones = {
        { SSTV_TONE_GENERIC, 9000, 1200 },  { SSTV_TONE_GENERIC, 3000, 1500 },
        { SSTV_TONE_SCAN_GREEN_Y, 0, 0 },   { SSTV_TONE_GENERIC, 4500, 1500 },
        { SSTV_TONE_GENERIC, 1500, 1900 },  { SSTV_TONE_SCAN_BLUE_CB, 0, 0 }
    }
};

const sstv_mode_t SSTV_MODE_ROBOT_72 = {
    .vis_code = SSTV_VIS_ROBOT_72, .width = 320, .height = 240, .scan_pixel_len = 431, .num_tones = 9,
    .tones = {
        { SSTV_TONE_GENERIC, 9000, 1200 },  { SSTV_TONE_GENERIC, 3000, 1500 },
        { SSTV_TONE_SCAN_GREEN_Y, 0, 0 },   { SSTV_TONE_GENERIC, 4500, 1500 },
        { SSTV_TONE_GENERIC, 1500, 1900 },  { SSTV_TONE_SCAN_RED_CR, 0, 0 },
        { SSTV_TONE_GENERIC, 4500, 2300 },  { SSTV_TONE_GENERIC, 1500, 1500 },
        { SSTV_TONE_SCAN_BLUE_CB, 0, 0 }
    }
};

/* Internal: send tone for specified duration */
static void sstv_tone_internal(sstv_ctx_t *ctx, uint16_t freq, uint32_t len_us)
{
    if (ctx->tone_func) {
        ctx->tone_func(freq);
    }
    if (len_us > 0 && ctx->delay_us_func) {
        ctx->delay_us_func(len_us);
    }
}

int16_t sstv_init(sstv_ctx_t *ctx, const sstv_mode_t *mode,
                  sstv_tone_func_t tone_fn, sstv_no_tone_func_t no_tone_fn,
                  sstv_delay_us_func_t delay_us_fn, sstv_micros_func_t micros_fn)
{
    if (ctx == NULL || mode == NULL || tone_fn == NULL || delay_us_fn == NULL) {
        return SSTV_ERR_NOT_INIT;
    }

    memcpy(&ctx->mode, mode, sizeof(sstv_mode_t));
    ctx->line_count = 0;
    ctx->correction = 1.0f;
    ctx->tone_func = tone_fn;
    ctx->no_tone_func = no_tone_fn;
    ctx->delay_us_func = delay_us_fn;
    ctx->micros_func = micros_fn;
    ctx->initialized = true;

    return SSTV_ERR_NONE;
}

int16_t sstv_set_correction(sstv_ctx_t *ctx, float correction)
{
    if (ctx == NULL || !ctx->initialized) {
        return SSTV_ERR_NOT_INIT;
    }

    ctx->mode.scan_pixel_len = (uint16_t)(ctx->mode.scan_pixel_len * correction);
    for (uint8_t i = 0; i < ctx->mode.num_tones; i++) {
        ctx->mode.tones[i].len = (uint32_t)(ctx->mode.tones[i].len * correction);
    }
    ctx->correction = correction;

    return SSTV_ERR_NONE;
}

void sstv_idle(sstv_ctx_t *ctx)
{
    if (ctx == NULL || !ctx->initialized) return;
    sstv_tone_internal(ctx, SSTV_TONE_LEADER, 0);
}

void sstv_send_header(sstv_ctx_t *ctx)
{
    if (ctx == NULL || !ctx->initialized) return;

    ctx->line_count = 0;

    /* Leader-break-leader */
    sstv_tone_internal(ctx, SSTV_TONE_LEADER, SSTV_HEADER_LEADER_LEN);
    sstv_tone_internal(ctx, SSTV_TONE_BREAK, SSTV_HEADER_BREAK_LEN);
    sstv_tone_internal(ctx, SSTV_TONE_LEADER, SSTV_HEADER_LEADER_LEN);

    /* VIS start bit */
    sstv_tone_internal(ctx, SSTV_TONE_BREAK, SSTV_HEADER_BIT_LEN);

    /* VIS code (7 bits) */
    uint8_t parity_count = 0;
    for (uint8_t mask = 0x01; mask < 0x80; mask <<= 1) {
        if (ctx->mode.vis_code & mask) {
            sstv_tone_internal(ctx, SSTV_TONE_VIS_1, SSTV_HEADER_BIT_LEN);
            parity_count++;
        } else {
            sstv_tone_internal(ctx, SSTV_TONE_VIS_0, SSTV_HEADER_BIT_LEN);
        }
    }

    /* VIS parity bit */
    if (parity_count % 2 == 0) {
        sstv_tone_internal(ctx, SSTV_TONE_VIS_0, SSTV_HEADER_BIT_LEN);
    } else {
        sstv_tone_internal(ctx, SSTV_TONE_VIS_1, SSTV_HEADER_BIT_LEN);
    }

    /* VIS stop bit */
    sstv_tone_internal(ctx, SSTV_TONE_BREAK, SSTV_HEADER_BIT_LEN);
}

void sstv_send_line(sstv_ctx_t *ctx, const uint32_t *img_line)
{
    if (ctx == NULL || !ctx->initialized || img_line == NULL) return;

    uint8_t vis = ctx->mode.vis_code;
    bool is_scottie = (vis == SSTV_VIS_SCOTTIE_1 || vis == SSTV_VIS_SCOTTIE_2 || vis == SSTV_VIS_SCOTTIE_DX);
    bool is_robot = (vis == SSTV_VIS_ROBOT_36 || vis == SSTV_VIS_ROBOT_72);

    /* First line sync for Scottie modes */
    if (ctx->line_count == 0 && is_scottie) {
        sstv_tone_internal(ctx, SSTV_TONE_BREAK, 9000);
    }

    /* Send all tones in sequence */
    for (uint8_t i = 0; i < ctx->mode.num_tones; i++) {
        sstv_tone_t *t = &ctx->mode.tones[i];

        if (t->type == SSTV_TONE_GENERIC && t->len > 0) {
            /* Robot36 has different separator tones for even/odd lines */
            uint16_t freq = t->freq;
            if (vis == SSTV_VIS_ROBOT_36 && i == 3) {
                freq = (ctx->line_count % 2) ? 2300 : t->freq;
            }
            sstv_tone_internal(ctx, freq, t->len);
        } else {
            /* Scan lines */
            for (uint16_t j = 0; j < ctx->mode.width; j++) {
                uint32_t color = img_line[j];
                uint32_t len = ctx->mode.scan_pixel_len;

                /* Robot modes use YCbCr */
                if (is_robot) {
                    uint8_t r = (color >> 16) & 0xFF;
                    uint8_t g = (color >> 8) & 0xFF;
                    uint8_t b = color & 0xFF;

                    uint8_t y  = (uint8_t)(16.0f + 0.003906f * (65.738f * r + 129.057f * g + 25.064f * b));
                    uint8_t cb = (uint8_t)(128.0f + 0.003906f * (-37.945f * r - 74.494f * g + 112.439f * b));
                    uint8_t cr = (uint8_t)(128.0f + 0.003906f * (112.439f * r - 94.154f * g - 18.285f * b));

                    color = ((uint32_t)y << 8);
                    if (vis == SSTV_VIS_ROBOT_36) {
                        color |= (ctx->line_count % 2) ? cb : cr;
                    } else {
                        color |= ((uint32_t)cr << 16) | cb;
                    }
                }

                uint8_t component = 0;
                switch (t->type) {
                    case SSTV_TONE_SCAN_RED_CR:
                        component = (color >> 16) & 0xFF;
                        if (is_robot) len /= 2;
                        break;
                    case SSTV_TONE_SCAN_GREEN_Y:
                        component = (color >> 8) & 0xFF;
                        break;
                    case SSTV_TONE_SCAN_BLUE_CB:
                        component = color & 0xFF;
                        if (is_robot) len /= 2;
                        break;
                    default:
                        break;
                }

                uint16_t freq = SSTV_TONE_BRIGHTNESS_MIN + (uint16_t)(component * 3.1372549f);
                sstv_tone_internal(ctx, freq, len);
            }
        }
    }

    ctx->line_count++;
}

void sstv_no_tone(sstv_ctx_t *ctx)
{
    if (ctx == NULL) return;
    if (ctx->no_tone_func) {
        ctx->no_tone_func();
    }
}

uint16_t sstv_get_picture_height(const sstv_ctx_t *ctx)
{
    if (ctx == NULL || !ctx->initialized) return 0;
    return ctx->mode.height;
}

uint16_t sstv_get_picture_width(const sstv_ctx_t *ctx)
{
    if (ctx == NULL || !ctx->initialized) return 0;
    return ctx->mode.width;
}

#ifdef __cplusplus
}
#endif

#endif /* SSTV_MINI_IMPLEMENTATION */

#endif /* SSTV_MINI_H */
