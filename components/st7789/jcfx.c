#include "jcfx.h"
#include "string.h"
#include "st7789.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

struct JCCTX_t {
    uint16_t width;
    uint16_t height;

    uint16_t firstChangedY;
    uint16_t lastChangedY;
    bool hasChanged;

    rgb_interface rgbInterface;
    uint8_t bytesPerPixel;
    
    TFT_t* tftHandle;
    uint8_t* framebuffer;
};

struct JCCTX_t* jcfx_init(TFT_t* tftHandle, uint16_t width, uint16_t height, rgb_interface rgbInterface)
{
    struct JCCTX_t* ctx = heap_caps_malloc(sizeof(struct JCCTX_t), MALLOC_CAP_DMA);

    ctx->width = width;
    ctx->height = height;
    ctx->tftHandle = tftHandle;

    ctx->hasChanged = false;

    ctx->rgbInterface = rgbInterface & 0b111;
    ctx->bytesPerPixel = rgbInterface == CIC18BPP ? 3 : 2;

    ctx->framebuffer = heap_caps_malloc(width * height * ctx->bytesPerPixel, MALLOC_CAP_DMA);
    return ctx;
}

void jcfx_begin_frame(struct JCCTX_t* ctx)
{
}

int get_pixel_idx(struct JCCTX_t* ctx, uint16_t x, uint16_t y)
{
    return y * ctx->width * ctx->bytesPerPixel + x * ctx->bytesPerPixel;
}

void write_colour_bytes(struct JCCTX_t* ctx, JCFXCOLOURBYTES_t colourBytes, uint16_t x, uint16_t y)
{
    uint32_t pixelIdx = get_pixel_idx(ctx, x, y);
    memcpy(&ctx->framebuffer[pixelIdx], colourBytes.bytes, ctx->bytesPerPixel);
}

JCFXCOLOURBYTES_t jcfx_convert_colour_to_bytes(struct JCCTX_t* ctx, JCFXCOLOUR_t colour)
{
    JCFXCOLOURBYTES_t rtv;

    switch (ctx->rgbInterface)
    {
        case CIC16BPP:
            // Top 5 bytes of red and top 3 of green
            rtv.bytes[0] = (colour.r & 0b11111000) | (colour.g >> 5);

            // Bottom 3 bytes of green and top 5 of blue
            rtv.bytes[1] = (colour.g << 5) | (colour.b >> 3);
            break;
        default:
            break;
    }

    return rtv;
}

void jcfx_draw_rect(struct JCCTX_t* ctx, JCFXCOLOUR_t colour, uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{
    ctx->hasChanged = true;
    ctx->firstChangedY = y;
    ctx->lastChangedY = y + height;

    JCFXCOLOURBYTES_t colorBytes = jcfx_convert_colour_to_bytes(ctx, colour);

    for (uint16_t j = y; j < y + height; j++)
    {
        for (uint16_t i = x; i < x + width; i++)
        {
            size_t pixelIdx = (j * ctx->width * ctx->bytesPerPixel) + i * ctx->bytesPerPixel;
            memcpy(&ctx->framebuffer[pixelIdx], colorBytes.bytes, ctx->bytesPerPixel);
        }
    }
}

void jcfx_blit(struct JCCTX_t* ctx, JCFXCOLOUR_t colour, uint16_t x, uint16_t y, char* lines, uint8_t lineCount)
{
    ctx->hasChanged = true;
    ctx->firstChangedY = y;
    ctx->lastChangedY = y + lineCount;

    JCFXCOLOURBYTES_t colourBytes = jcfx_convert_colour_to_bytes(ctx, colour);
    for (int j = 0; j < lineCount; j++)
    {
        for (int i = 0; i < 8; i++)
        {
            if (lines[j] & (1 << i))
            {
                write_colour_bytes(ctx, colourBytes, x + i, y + j);
            }
        }
    }
}

void jcfx_end_frame(struct JCCTX_t* ctx)
{
    if (!ctx->hasChanged)
        return;

    uint8_t* offset = &ctx->framebuffer[ctx->firstChangedY * ctx->width * ctx->bytesPerPixel];
    uint8_t* offsetEnd = &ctx->framebuffer[ctx->lastChangedY * ctx->width * ctx->bytesPerPixel];
    lcdDrawPixels(ctx->tftHandle, 0, ctx->firstChangedY, ctx->width, ctx->height, offset, offsetEnd - offset);
    ctx->hasChanged = false;
}