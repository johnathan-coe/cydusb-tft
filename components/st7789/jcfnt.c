#include "string.h"

#include "jcfnt.h"
#include "font8x8_basic.h"

#define FONT_HEIGHT 8
#define FONT_WIDTH 8

size_t word_length(char* string)
{
    char* offset = string;
    while (*offset != '\0' && *offset != ' ' && *offset != '\n')
        offset++;

    return offset - string;
}

void jcfnt_write(struct JCCTX_t* ctx, char* string, JCFXCOLOUR_t colour, uint16_t x, uint16_t y, uint16_t width)
{
    uint16_t currentX = x;
    uint16_t currentY = y;

    bool isFirstInLine = true;
    for (size_t i = 0; i < strlen(string); i++)
	{
        char c = string[i];

        bool shouldWrap = false;
        shouldWrap |= currentX >= (x + width);
        shouldWrap |= c == '\n';
        shouldWrap |= c == ' ' && ((currentX + (word_length(&string[i + 1]) + 1) * FONT_WIDTH) > (x + width));

        if (shouldWrap)
        {
            currentX = x;
            currentY += FONT_HEIGHT;
            isFirstInLine = true;

            // Don't write whitespace
            if (c == '\n' || c == ' ')
                continue;
        }

        if (isFirstInLine && c == ' ')
            continue;

        char* bmp = font8x8_basic[(size_t) c];
		jcfx_blit(ctx, colour, currentX, currentY, bmp, 8);
        currentX += FONT_WIDTH;

        isFirstInLine = false;
	}
}