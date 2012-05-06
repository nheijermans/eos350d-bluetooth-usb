#include "compiler.h"
#include "utils.h"
#include "stdio.h"

static char
to_printable(uint8_t byte)
{
    if (byte >= 0x20 && byte <= 0x7e)
    {
        return (char) byte;
    }
    return '.';
}

#define CHARS_PER_LINE 16

void
dump_hex(uint8_t *buf, uint32_t buf_size)
{
    uint32_t lines = buf_size / CHARS_PER_LINE;
    uint32_t i;
    uint32_t j;

    if (buf_size % CHARS_PER_LINE)
    {
        lines++;
    }

    for (i = 0; i < lines; i++)
    {
        uint8_t chars_this_line = (i == (lines-1))? buf_size % CHARS_PER_LINE: CHARS_PER_LINE;

        if (!chars_this_line)
        {
            break;
        }

        printf("%08lx: ", i * CHARS_PER_LINE);

        for (j = 0; j < chars_this_line; j++)
        {
            uint8_t c = buf[i * CHARS_PER_LINE + j];
            printf("%02x%s", c, ((j+1) %8 )? " ": "  ");
        }

        for (j = chars_this_line; j < CHARS_PER_LINE; j++)
        {
            printf("  %s", ((j+1) %8 )? " ": "  ");
        }

        while (j++ < CHARS_PER_LINE)
        {
            printf("  %s", ((j+1) %8 )? " ": "  ");
        }

        printf("    |");
        for (j = 0; j < chars_this_line && --buf_size; j++)
        {
            uint8_t c = buf[i * CHARS_PER_LINE + j];
            printf("%c", to_printable(c));
        }

        for (j = chars_this_line; j < CHARS_PER_LINE; j++)
        {
            printf(" ");
        }

        printf("|\n");
    }
}

