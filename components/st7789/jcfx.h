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

#define JCFX_BLACK  (JCFXCOLOUR_t) { .r = 0,    .g = 0,     .b = 0      }
#define JCFX_RED    (JCFXCOLOUR_t) { .r = 255,  .g = 0,     .b = 0      }
#define JCFX_GREEN  (JCFXCOLOUR_t) { .r = 0,    .g = 255,   .b = 0      }
#define JCFX_BLUE   (JCFXCOLOUR_t) { .r = 0,    .g = 0,     .b = 255    }


typedef struct
{
    uint8_t bytes[3];
} JCFXCOLOURBYTES_t;

struct JCCTX_t* jcfx_init(TFT_t* tftHandle, uint16_t width, uint16_t height, rgb_interface bytesPerPixel);
void jcfx_begin_frame(struct JCCTX_t* ctx);
void jcfx_end_frame(struct JCCTX_t* ctx);

void jcfx_draw_rect(struct JCCTX_t* ctx, JCFXCOLOUR_t colour, uint16_t x, uint16_t y, uint16_t width, uint16_t height);
void jcfx_blit(struct JCCTX_t* ctx, JCFXCOLOUR_t colour, uint16_t x, uint16_t y, char* lines, uint8_t lineCount);

#endif