#ifndef JCFX_H_
#define JCFX_H_

#include <inttypes.h>
#include "st7789.h"

struct JCCTX_t;

typedef struct
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
} JCFXCOLOUR_t;

#define JCFX_BLACK      (JCFXCOLOUR_t) { .r = 0,    .g = 0,     .b = 0      }
#define JCFX_WHITE      (JCFXCOLOUR_t) { .r = 255,  .g = 255,   .b = 255    }
#define JCFX_RED        (JCFXCOLOUR_t) { .r = 255,  .g = 0,     .b = 0      }
#define JCFX_GREEN      (JCFXCOLOUR_t) { .r = 0,    .g = 255,   .b = 0      }
#define JCFX_BLUE       (JCFXCOLOUR_t) { .r = 0,    .g = 0,     .b = 255    }
#define JCFX_DARK_GRAY  (JCFXCOLOUR_t) { .r = 51,   .g = 51,    .b = 51     }

#define JCFX_98_GRAY    (JCFXCOLOUR_t) { .r = 192,  .g = 192,   .b = 192    }
#define JCFX_98_BLUE    (JCFXCOLOUR_t) { .r = 0,    .g = 0,     .b = 123    }

typedef struct
{
    uint8_t bytes[3];
} JCFXCOLOURBYTES_t;

struct JCCTX_t* jcfx_init(TFT_t* tftHandle, uint16_t width, uint16_t height, rgb_interface bytesPerPixel);
void jcfx_begin_frame(struct JCCTX_t* ctx);
void jcfx_end_frame(struct JCCTX_t* ctx);

JCFXCOLOURBYTES_t jcfx_convert_colour_to_bytes(struct JCCTX_t* ctx, JCFXCOLOUR_t colour);

void jcfx_write_colour_bytes(struct JCCTX_t* ctx, JCFXCOLOURBYTES_t colourBytes, uint16_t x, uint16_t y);
void jcfx_draw_rect(struct JCCTX_t* ctx, JCFXCOLOUR_t colour, uint16_t x, uint16_t y, uint16_t width, uint16_t height);
void jcfx_draw_gradient_rect(struct JCCTX_t* ctx, JCFXCOLOUR_t colour1, JCFXCOLOUR_t colour2, uint16_t x, uint16_t y, uint16_t width, uint16_t height);
void jcfx_blit(struct JCCTX_t* ctx, JCFXCOLOUR_t colour, uint16_t x, uint16_t y, char* lines, uint8_t lineCount);

#endif