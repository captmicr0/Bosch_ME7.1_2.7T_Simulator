#pragma once
#include <stdint.h>
#include <stdbool.h>

// Engine profile. All angles are CRANK degrees relative to the first
// tooth SLOT after the missing-tooth gap (slot zero). A complete
// four-stroke cycle is 720 crank degrees = 120 slots of 6 degrees.
// NOTE: APB cam phase offsets are CALIBRATION PLACEHOLDERS, not measured
// factory geometry. Verify on an engine scope before treating as OEM.
#define ENGINE_NAME "AUDI_APB_27T_PROVISIONAL"
#define ENGINE_CYLINDERS 6
static const uint8_t engine_firing_order[ENGINE_CYLINDERS] = {1,4,3,6,2,5};
// Configurable wheel geometry. A 'slot' is one nominal tooth interval.
// The missing-tooth gap is placed at the END of each revolution, so slot 0
// is always the first nominal tooth after the gap / 720-degree marker.
#define ENGINE_DEFAULT_TEETH 60u
#define ENGINE_DEFAULT_MISSING 2u
#define ENGINE_MIN_TEETH 12u
#define ENGINE_MAX_TEETH 120u
#define ENGINE_MAX_MISSING 16u

typedef struct { uint16_t teeth; uint16_t missing; bool invert; } crank_shape_t;

// Cam timing on GP2 (G40) / GP3 (G163); crank waveform via MCP4922 SPI DAC.
// Phase in degrees crank, width in degrees crank. Individual outputs
// can be inverted and shifted independently for other engines.
typedef struct { int16_t start_deg; uint16_t width_deg; bool invert; } cam_shape_t;
typedef struct { cam_shape_t g40, g163; crank_shape_t crank; } engine_shape_t;

static inline uint16_t normalize_720(int32_t deg) {
    int32_t a=deg%720;
    return (uint16_t)(a<0 ? a+720 : a);
}
static inline uint16_t engine_cycle_slots(const engine_shape_t *s) {
    return (uint16_t)(2u * s->crank.teeth);
}
static inline bool crank_shape_valid(const crank_shape_t *c) {
    return c->teeth >= ENGINE_MIN_TEETH && c->teeth <= ENGINE_MAX_TEETH &&
           c->missing >= 1u && c->missing <= ENGINE_MAX_MISSING &&
           c->missing < c->teeth;
}
static inline uint16_t engine_slot_angle(const engine_shape_t *s, uint16_t slot) {
    return (uint16_t)(((uint32_t)slot * 360u) / s->crank.teeth);
}
static inline bool cam_level_at(const engine_shape_t *s, const cam_shape_t *cam, uint16_t slot) {
    uint16_t angle=engine_slot_angle(s,slot);
    uint16_t diff=normalize_720((int32_t)angle-cam->start_deg);
    bool high=(cam->width_deg >= 720u) || diff < cam->width_deg;
    return high != cam->invert;
}
// Replace this to emulate custom cam wheels (multiple edges per 720-degree cycle).
static inline uint8_t engine_cam_bits(const engine_shape_t *s, uint16_t slot) {
    return (cam_level_at(s,&s->g40,slot)?1u:0u) |
           (cam_level_at(s,&s->g163,slot)?2u:0u);
}
// GPIO2..5 packed as two 4-bit output nibbles for a single tooth slot.
// The PIO program divides each slot into two equal halves (50% duty).
static inline uint32_t engine_slot_word(const engine_shape_t *s, uint16_t slot) {
    uint8_t cams=engine_cam_bits(s,slot);
    bool tooth=(slot%s->crank.teeth)<(s->crank.teeth-s->crank.missing);
    bool level=tooth != s->crank.invert;
    uint8_t first=(uint8_t)((cams<<1)|(level?1u:0u)|((slot==0u)?8u:0u));
    uint8_t second=(uint8_t)((cams<<1)|(s->crank.invert?1u:0u)|((slot==0u)?8u:0u));
    return (uint32_t)first | ((uint32_t)second<<4);
}
