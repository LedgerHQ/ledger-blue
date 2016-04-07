/*******************************************************************************
*   Ledger Blue - Non secure firmware
*   (c) 2016 Ledger
*
*  Licensed under the Apache License, Version 2.0 (the "License");
*  you may not use this file except in compliance with the License.
*  You may obtain a copy of the License at
*
*      http://www.apache.org/licenses/LICENSE-2.0
*
*  Unless required by applicable law or agreed to in writing, software
*  distributed under the License is distributed on an "AS IS" BASIS,
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*  See the License for the specific language governing permissions and
*  limitations under the License.
********************************************************************************/

#ifndef BAGL_H_
#define BAGL_H_

#define BAGL_NOFILL 0
#define BAGL_FILL 1
#define BAGL_OUTLINE 2
#define BAGL_NOICON 3

#define BAGL_FILL_CIRCLE_1_OCTANT 1
#define BAGL_FILL_CIRCLE_2_OCTANT 2
#define BAGL_FILL_CIRCLE_3_OCTANT 4
#define BAGL_FILL_CIRCLE_4_OCTANT 8
#define BAGL_FILL_CIRCLE_5_OCTANT 16
#define BAGL_FILL_CIRCLE_6_OCTANT 32
#define BAGL_FILL_CIRCLE_7_OCTANT 64
#define BAGL_FILL_CIRCLE_8_OCTANT 128
#define BAGL_FILL_CIRCLE 0xFF
#define BAGL_FILL_CIRCLE_3PI2_2PI                                              \
    (BAGL_FILL_CIRCLE_1_OCTANT | BAGL_FILL_CIRCLE_2_OCTANT)
#define BAGL_FILL_CIRCLE_PI_3PI2                                               \
    (BAGL_FILL_CIRCLE_3_OCTANT | BAGL_FILL_CIRCLE_4_OCTANT)
#define BAGL_FILL_CIRCLE_0_PI2                                                 \
    (BAGL_FILL_CIRCLE_5_OCTANT | BAGL_FILL_CIRCLE_6_OCTANT)
#define BAGL_FILL_CIRCLE_PI2_PI                                                \
    (BAGL_FILL_CIRCLE_7_OCTANT | BAGL_FILL_CIRCLE_8_OCTANT)

#if 0
// no event
#define BAGL_EVENT_NONE 0
//
#define BAGL_EVENT_OVER 1
#define BAGL_EVENT_TOUCH 2
#define BAGL_EVENT_OUT 3

// differenciated from the high level events to allow for touch/out concepts
#define BAGL_HAL_EVENT_TOUCHING 4
#define BAGL_HAL_EVENT_RELEASING 5
#endif

// --------------------------------------------------------------------------------------

typedef enum bagl_components_type_e_ {
    BAGL_NONE = 0, // for fast memset clearing all components
    BAGL_BUTTON = 1,
    // BAGL_TEXTBOX, // NOT NECESSARY
    // BAGL_SENSEZONE, // sense only
    BAGL_LABEL,
    BAGL_RECTANGLE,
    BAGL_LINE,
    BAGL_ICON,
    BAGL_CIRCLE,
    BAGL_FLAG_TOUCHABLE = 0x80,
} bagl_components_type_e;
#define BAGL_TYPE_FLAGS_MASK 0x80

// --------------------------------------------------------------------------------------

typedef struct {
    bagl_components_type_e type;
    unsigned char userid;
    unsigned short x;
    unsigned short y;
    unsigned short width;
    unsigned short height;
    unsigned char stroke;
    unsigned char radius;
    unsigned char fill;
    unsigned int fgcolor;
    unsigned int bgcolor;
    unsigned short font_id;
    unsigned char icon_id;
} bagl_component_t;

#define BAGL_FONT_ID_MASK 0x0FFF
#define BAGL_FONT_ALIGNMENT_HORIZONTAL_MASK 0xC000
#define BAGL_FONT_ALIGNMENT_LEFT 0x0000
#define BAGL_FONT_ALIGNMENT_RIGHT 0x4000
#define BAGL_FONT_ALIGNMENT_CENTER 0x8000
#define BAGL_FONT_ALIGNMENT_VERTICAL_MASK 0x3000
#define BAGL_FONT_ALIGNMENT_TOP 0x0000
#define BAGL_FONT_ALIGNMENT_BOTTOM 0x1000
#define BAGL_FONT_ALIGNMENT_MIDDLE 0x2000

// --------------------------------------------------------------------------------------

typedef struct {
    unsigned int width;
    unsigned int height;
    unsigned int bits_per_pixel;
    const unsigned int
        *default_colors; // color array entry count is (1<<bits_per_pixel)
    const unsigned char *bitmap;
} bagl_glyph_array_entry_t;

extern const bagl_glyph_array_entry_t const C_glyph_array[];
extern const unsigned int C_glyph_count;

// --------------------------------------------------------------------------------------

typedef struct {
    unsigned char char_width;
    unsigned char bitmap_byte_count;
    unsigned char const *bitmap;
} bagl_font_character_t;

typedef struct {
    unsigned char char_height;
    unsigned char baseline_height;
    unsigned char char_kerning; // specific to the font
    unsigned short first_char;
    unsigned short last_char;
    bagl_font_character_t const *characters;
} bagl_font_t;

#define BAGL_ENCODING_LATIN1 0

// must correspond to the order in C_bagl_fonts
typedef enum {
    BAGL_FONT_LUCIDA_CONSOLE_8,
    BAGL_FONT_OPEN_SANS_BOLD_13px,
    BAGL_FONT_OPEN_SANS_BOLD_21px,
    BAGL_FONT_OPEN_SANS_LIGHT_13px,
    BAGL_FONT_OPEN_SANS_LIGHT_14px,
    BAGL_FONT_OPEN_SANS_LIGHT_16px,
    BAGL_FONT_OPEN_SANS_LIGHT_21px,
    BAGL_FONT_OPEN_SANS_SEMIBOLD_18px,
    BAGL_FONT_COMIC_SANS_MS_20px,

    BAGL_FONT_LAST // MUST ALWAYS BE THE LAST, FOR AUTOMATED INVALID VALUE
                   // CHECKS
} bagl_font_id_e;

// --------------------------------------------------------------------------------------

#define BAGL_BUTTON_INNERSPACING 2

// --------------------------------------------------------------------------------------

// Glyphs definitions
// must correspond to the order in the C_glyph_array
enum bagl_glyph_e {
    BAGL_GLYPH_NOGLYPH = 0,
    BAGL_GLYPH_LOGO_LEDGER_100,
    BAGL_GLYPH_LOGO_LEDGER_BLUE_120,
    BAGL_GLYPH_ICON_GEARS_50,
    BAGL_GLYPH_ICON_CLEAR_16,
    BAGL_GLYPH_ICON_BACKSPACE_20,
};

// --------------------------------------------------------------------------------------
// return y<<16+x after string have been printed
int bagl_draw_string(unsigned short font_id, unsigned int color1,
                     unsigned int color0, unsigned short x, unsigned short y,
                     unsigned short width, unsigned short height, void *text,
                     unsigned short text_length, unsigned char text_encoding);
void bagl_draw_bg(unsigned int color);
void bagl_draw_with_context(bagl_component_t *component, void *text,
                            unsigned short text_length,
                            unsigned char text_encoding);
void bagl_draw(bagl_component_t *component);
// void bagl_undraw_all(void);
// void bagl_undraw_id(unsigned short id);
// void bagl_user_input(unsigned short x, unsigned short y, unsigned char
// event_kind);

// for user to setup the glyph matrix to be used
void bagl_set_glyph_array(const bagl_glyph_array_entry_t *array,
                          unsigned int count);

// --------------------------------------------------------------------------------------
void bagl_hal_draw_bitmap_within_rect(
    unsigned short x, unsigned short y, unsigned short width,
    unsigned short height, unsigned int color_count, unsigned int *colors,
    unsigned int bit_per_pixel, unsigned char *bitmap2,
    unsigned int bitmap_length_bits);
void bagl_hal_draw_bitmap_continue(unsigned int bit_per_pixel,
                                   unsigned char *bitmap,
                                   unsigned int bitmap_length_bits);

void bagl_hal_draw_rect(unsigned int color, unsigned short x, unsigned short y,
                        unsigned short width, unsigned short height);
void bagl_action(bagl_component_t *component, unsigned char event_kind);

// --------------------------------------------------------------------------------------
// internal exposed for smoother coding of printf functions

#endif // BAGL_H_
