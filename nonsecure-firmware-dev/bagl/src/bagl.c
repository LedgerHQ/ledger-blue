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
*   Unless required by applicable law or agreed to in writing, software
*   distributed under the License is distributed on an "AS IS" BASIS,
*   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*   See the License for the specific language governing permissions and
*   limitations under the License.
********************************************************************************/

#ifdef HAVE_BAGL

#include "bagl.h"
#include <string.h>
#include <stdio.h>

/**
 Coordinate system for BAGL:
 ===========================

       0   X axis
      0 +----->
        |
Y axis  |   ##### 
        v  #######
           ##   ##
           #######
            #####
*/


// --------------------------------------------------------------------------------------
// Checks
// -------------------------------------------------------------------------------------- 

/*
#ifndef BAGL_COMPONENT_MAXCOUNT
#error BAGL_COMPONENT_MAXCOUNT not set
#endif // !BAGL_COMPONENT_MAXCOUNT
*/

#ifndef BAGL_WIDTH
#error BAGL_WIDTH not set
#endif // !BAGL_WIDTH

#ifndef BAGL_HEIGHT
#error BAGL_HEIGHT not set
#endif // !BAGL_HEIGHT

// --------------------------------------------------------------------------------------
// Definitions
// -------------------------------------------------------------------------------------- 

#define ICON_WIDTH 0
#define MIN(x,y) ((x)<(y)?(x):(y))
#define MAX(x,y) ((x)>(y)?(x):(y))

// --------------------------------------------------------------------------------------
// Variables
// -------------------------------------------------------------------------------------- 

//bagl_component_t bagl_components[BAGL_COMPONENT_MAXCOUNT];
unsigned int bagl_bgcolor;

extern bagl_font_t const * C_bagl_fonts [];
// handle the touch with release out of zone
unsigned int last_touched_not_released_component_idx;

const bagl_glyph_array_entry_t* G_glyph_array;
unsigned int                    G_glyph_count; 


// --------------------------------------------------------------------------------------
// Dummy HAL
// -------------------------------------------------------------------------------------- 


// --------------------------------------------------------------------------------------

/*
// to draw font faster as the pixel loop is at the lower layer (if not directly within transport)
__weak void bagl_hal_draw_bitmap2_within_rect(unsigned int color_0, unsigned int color_1, unsigned short x, unsigned short y, unsigned short width, unsigned short height, unsigned char* bitmap2, unsigned short bitmap_length) {

}

// --------------------------------------------------------------------------------------

__weak void bagl_hal_draw_rect(unsigned int color, unsigned short x, unsigned short y, unsigned short width, unsigned short height) {

}

// --------------------------------------------------------------------------------------

// angle_start 0 => trigonometric 0, range is [0:360[
__weak void bagl_hal_draw_circle(unsigned int color, unsigned short x_center, unsigned short y_center, unsigned short radius, unsigned short angle_start, unsigned short angle_stop) {

}
*/

// --------------------------------------------------------------------------------------
// API
// -------------------------------------------------------------------------------------- 

// --------------------------------------------------------------------------------------
void bagl_draw_bg(unsigned int color) {
  bagl_component_t c;
  memset(&c, 0, sizeof(c));
  c.type = BAGL_RECTANGLE;
  c.userid = BAGL_NONE;
  bagl_bgcolor = c.fgcolor = color;
  c.x = 0;
  c.y = 0;
  c.width = BAGL_WIDTH;
  c.height = BAGL_HEIGHT;
  c.fill = BAGL_FILL;
  // draw the rect
  bagl_draw_with_context(&c, NULL, 0, 0);
}

// --------------------------------------------------------------------------------------
// return the width of a text (first line only) for alignment processing
unsigned short bagl_compute_line_width(unsigned short font_id, unsigned short width, void* text, unsigned char text_length, unsigned char text_encoding) {
  unsigned short xx;

  font_id &= (BAGL_FONT_ID_MASK);

  // check valid font
  if (font_id >= BAGL_FONT_LAST || C_bagl_fonts[font_id] == NULL) {
    return;
  }

  // initialize first index
  xx = 0;

  //printf("display text: %s\n", text);

  unsigned char ch_kerning = C_bagl_fonts[font_id]->char_kerning;

  // depending on encoding
  while (text_length--) {
    unsigned int ch = 0;
    // TODO support other encoding than ascii ISO8859 Latin
    switch(text_encoding) {
      default:
      case BAGL_ENCODING_LATIN1:
        ch = *((unsigned char*)text);
        text = (void*)(((unsigned char*)text)+1);
        if (ch < C_bagl_fonts[font_id]->first_char || ch > C_bagl_fonts[font_id]->last_char) {
          //printf("invalid char");
          // can't proceed
          return 0;
        }
        ch -= C_bagl_fonts[font_id]->first_char;
        break;
    }

    // retrieve the char bitmap
    unsigned char ch_width = C_bagl_fonts[font_id]->characters[ch].char_width;

    // go to next line if needed
    if (xx + ch_width > width) {
      return xx;
    }

    // prepare for next char
    xx += ch_width + ch_kerning;
  }
  return xx;
}

// --------------------------------------------------------------------------------------
// draw char until a char fit before reaching width
// TODO support hyphenation ??
int bagl_draw_string(unsigned short font_id, unsigned int fgcolor, unsigned int bgcolor, unsigned short x, unsigned short y, unsigned short width, unsigned short height, void* text, unsigned short text_length, unsigned char text_encoding) {
  unsigned short xx;
  unsigned int colors[2];
  colors[0] = bgcolor;
  colors[1] = fgcolor;

  font_id &= (BAGL_FONT_ID_MASK);

  // check valid font
  if (font_id >= BAGL_FONT_LAST || C_bagl_fonts[font_id] == NULL) {
    return;
  }

  // always comparing this way, very optimized etc
  width += x;
  height += y;

  // initialize first index
  xx = x;

  //printf("display text: %s\n", text);

  unsigned char ch_height = C_bagl_fonts[font_id]->char_height;
  unsigned char ch_kerning = C_bagl_fonts[font_id]->char_kerning;

  // depending on encoding
  while (text_length--) {
    unsigned int ch = 0;
    // TODO support other encoding than ascii ISO8859 Latin
    switch(text_encoding) {
      default:
      case BAGL_ENCODING_LATIN1:
        ch = *((unsigned char*)text);
        text = (void*)(((unsigned char*)text)+1);
        if (ch < C_bagl_fonts[font_id]->first_char || ch > C_bagl_fonts[font_id]->last_char) {
          //printf("invalid char");
          // can't proceed
          if (ch == '\n' || ch == '\t') {
            y += ch_height; // no interleave

            // IGNORED for first line
            if (y + ch_height > height) {
              // we're writing half height of the last line ... probably better to put some dashes
              return (y<<16)|(xx&0xFFFF);
            }

            // newline starts back at first x offset
            xx = x;

            continue;
          }
          return (y<<16)|(xx&0xFFFF);
        }
        ch -= C_bagl_fonts[font_id]->first_char;
        break;
    }

    // retrieve the char bitmap
    unsigned char const * ch_bitmap = C_bagl_fonts[font_id]->characters[ch].bitmap;
    unsigned char ch_width = C_bagl_fonts[font_id]->characters[ch].char_width;

    // go to next line if needed
    if (xx + ch_width > width) {
      y += ch_height; // no interleave

      // IGNORED for first line
      if (y + ch_height > height) {
        // we're writing half height of the last line ... probably better to put some dashes
        return (y<<16)|(xx&0xFFFF);
      }

      // newline starts back at first x offset
      xx = x;
    }

    /* IGNORED for first line
    if (y + ch_height > height) {
        // we're writing half height of the last line ... probably better to put some dashes
        return;
    }
    */

    // chars are storred LSB to MSB in each char, packed chars. horizontal scan
    bagl_hal_draw_bitmap_within_rect(xx, y, ch_width, ch_height, 2, colors, 1, ch_bitmap, ch_width*ch_height); // note, last parameter is computable could be avoided
    // prepare for next char
    xx += ch_width + ch_kerning;
  }

  // return newest position, for upcoming printf
  return (y<<16)|(xx&0xFFFF);
}

// --------------------------------------------------------------------------------------

void bagl_draw_circle_helper(unsigned int color, unsigned short x_center, unsigned short y_center, unsigned short radius, unsigned char octants) {
  unsigned short x, y;
  short d;
  x = 0;
  y = radius;
  d = radius - 1;
  while (x <= y) {
    if (octants & 1)
      bagl_hal_draw_rect(color, x_center,   y+y_center, x, 1);
    if (octants & 2)
      bagl_hal_draw_rect(color, x_center,   x+y_center, y, 1);
    if (octants & 4)
      bagl_hal_draw_rect(color, x_center-x, y+y_center, x, 1);
    if (octants & 8)
      bagl_hal_draw_rect(color, x_center-y, x+y_center, y, 1);
    if (octants & 16)
      bagl_hal_draw_rect(color, x_center,   y_center-y, x, 1);
    if (octants & 32)
      bagl_hal_draw_rect(color, x_center,   y_center-x, y, 1);
    if (octants & 64)
      bagl_hal_draw_rect(color, x_center-x, y_center-y, x, 1);
    if (octants & 128)
      bagl_hal_draw_rect(color, x_center-y, y_center-x, y, 1);

    if (d >= 2*x) {
      d = d - 2*x - 1;
      x = x + 1;
    }
    else if (d < 2*(radius - y)) {
      d = d + 2*y - 1;
      y = y - 1;
    }
    else {
      d = d + 2*(y-x-1);
      y = y - 1;
      x = x + 1;
    }
  }
}

// --------------------------------------------------------------------------------------

void bagl_set_glyph_array(const bagl_glyph_array_entry_t* array, unsigned int count) {
  G_glyph_array = array;
  G_glyph_count = count;
}

// --------------------------------------------------------------------------------------

void bagl_draw_with_context(bagl_component_t* component, void* context, unsigned short context_length, unsigned char context_encoding) {
  //unsigned char comp_idx;
  unsigned int halignment=0;
  unsigned int valignment=0;
  unsigned int x,y;

  // DESIGN NOTE: always consider drawing onto a bg color filled image. (done upon undraw)

  /*
  // check if userid already exist, if yes, reuse entry
  for (comp_idx=0; comp_idx < BAGL_COMPONENT_MAXCOUNT; comp_idx++) {
    if (bagl_components[comp_idx].userid == component->userid) {
      goto idx_ok;
    }
  }

  // find the first empty entry
  for (comp_idx=0; comp_idx < BAGL_COMPONENT_MAXCOUNT; comp_idx++) {
    if (bagl_components[comp_idx].userid == BAGL_NONE) {
      goto idx_ok;
    }
  }
  // no more space :(
  //BAGL_THROW(NO_SPACE);
  return;


idx_ok:
  */
  
  // strip the flags to match kinds
  unsigned int type = component->type&~(BAGL_TYPE_FLAGS_MASK);

  // compute alignment if text provided and requiring special alignment
  if (context && context_length && type != BAGL_ICON) {
    switch (component->font_id & BAGL_FONT_ALIGNMENT_HORIZONTAL_MASK ) {
      default:
      case BAGL_FONT_ALIGNMENT_LEFT:
        halignment = 0;
        break;
      case BAGL_FONT_ALIGNMENT_RIGHT:
        halignment = component->width - bagl_compute_line_width(component->font_id, component->width, context, context_length, context_encoding);
        break;
      case BAGL_FONT_ALIGNMENT_CENTER:
        // x   xalign      strwidth width
        // '     '            '     '
        //       ^
        // xalign = x+ (width/2) - (strwidth/2) => align -x
        halignment = component->width/2 - bagl_compute_line_width(component->font_id, component->width, context, context_length, context_encoding)/2;
        break;
    }

    switch (component->font_id & BAGL_FONT_ALIGNMENT_VERTICAL_MASK ) {
      default:
      case BAGL_FONT_ALIGNMENT_TOP:
        valignment = 0;
        break;
      case BAGL_FONT_ALIGNMENT_BOTTOM:
        valignment = component->height - C_bagl_fonts[component->font_id&BAGL_FONT_ID_MASK]->baseline_height;
        break;
      case BAGL_FONT_ALIGNMENT_MIDDLE:
        // y                 yalign           charheight        height
        // '                    '          v  '                 '
        //                           baseline
        // yalign = y+ (height/2) - (baseline/2) => align - y
        valignment = component->height/2 - C_bagl_fonts[component->font_id&BAGL_FONT_ID_MASK]->baseline_height/2;
        break;
    }
  }

  // only check the type only, ignore the touchable flag
  switch(type) {
    case BAGL_RECTANGLE:
    case BAGL_BUTTON: // textbox + rectangle
    draw_round_rect:
      // draw the background
      /* shall not be needed
      if (component->fill == BAGL_FILL) {
        bagl_hal_draw_rect(component->bgcolor, 
                           component->x+component->radius,                  
                           component->y, 
                           component->width-2*component->radius, 
                           component->stroke); // top
      }
      */
      
      if (component->fill != BAGL_FILL) {

        // centered top to bottom
        bagl_hal_draw_rect(component->bgcolor, 
                           component->x+component->radius,                  
                           component->y, 
                           component->width-2*component->radius, 
                           component->height);
        // left to center rect
        bagl_hal_draw_rect(component->bgcolor, 
                           component->x,                                    
                           component->y+component->radius, 
                           component->radius, 
                           component->height-2*component->radius); 

        // center rect to right
        bagl_hal_draw_rect(component->bgcolor, 
                           component->x+component->width-component->radius, 
                           component->y+component->radius, 
                           component->radius, 
                           component->height-2*component->radius);

        // 4 rectangles (with last pixel of each corner not set)
        bagl_hal_draw_rect(component->fgcolor, 
                           component->x+component->radius,                  
                           component->y, 
                           component->width-2*component->radius, 
                           component->stroke); // top
        bagl_hal_draw_rect(component->fgcolor, 
                           component->x+component->radius,                  
                           component->y+component->height-component->stroke, 
                           component->width-2*component->radius, 
                           component->stroke); // bottom
        bagl_hal_draw_rect(component->fgcolor, 
                           component->x,                                    
                           component->y+component->radius, 
                           component->stroke, 
                           component->height-2*component->radius); // left
        bagl_hal_draw_rect(component->fgcolor, 
                           component->x+component->width-component->stroke, 
                           component->y+component->radius, 
                           component->stroke, 
                           component->height-2*component->radius); // right
      }
      else {
        // centered top to bottom
        bagl_hal_draw_rect(component->fgcolor, 
                           component->x+component->radius,                  
                           component->y, 
                           component->width-2*component->radius, 
                           component->height);
        // left to center rect
        bagl_hal_draw_rect(component->fgcolor, 
                           component->x,                                    
                           component->y+component->radius, 
                           component->radius, 
                           component->height-2*component->radius); 

        // center rect to right
        bagl_hal_draw_rect(component->fgcolor, 
                           component->x+component->width-component->radius, 
                           component->y+component->radius, 
                           component->radius, 
                           component->height-2*component->radius);
      }

      // draw corners
      if (component->radius > 1) {
        bagl_draw_circle_helper(component->fgcolor, component->x+component->radius, component->y+component->radius, component->radius, BAGL_FILL_CIRCLE_PI2_PI);
        bagl_draw_circle_helper(component->fgcolor, component->x+component->width-component->radius, component->y+component->radius, component->radius, BAGL_FILL_CIRCLE_0_PI2);
        bagl_draw_circle_helper(component->fgcolor, component->x+component->radius, component->y+component->height-component->radius-1, component->radius, BAGL_FILL_CIRCLE_PI_3PI2);
        bagl_draw_circle_helper(component->fgcolor, component->x+component->width-component->radius, component->y+component->height-component->radius-1, component->radius, BAGL_FILL_CIRCLE_3PI2_2PI);
        // carve round when not filling
        if (component->fill != BAGL_FILL) {
          bagl_draw_circle_helper(component->bgcolor, component->x+component->radius, component->y+component->radius, component->radius-component->stroke, BAGL_FILL_CIRCLE_PI2_PI);
          bagl_draw_circle_helper(component->bgcolor, component->x+component->width-component->radius, component->y+component->radius, component->radius-component->stroke, BAGL_FILL_CIRCLE_0_PI2);
          bagl_draw_circle_helper(component->bgcolor, component->x+component->radius, component->y+component->height-component->radius-1, component->radius-component->stroke, BAGL_FILL_CIRCLE_PI_3PI2);
          bagl_draw_circle_helper(component->bgcolor, component->x+component->width-component->radius, component->y+component->height-component->radius-1, component->radius-component->stroke, BAGL_FILL_CIRCLE_3PI2_2PI);
        }
      }

      
      // 1 textarea (reframed)
      if (context && context_length) {
        bagl_draw_string(component->font_id,
                         // draw '1' pixels in bg when filled
                         (component->fill == BAGL_FILL) ? component->bgcolor:component->fgcolor, 
                         // draw '0' pixels in fg when filled
                         (component->fill == BAGL_FILL) ? component->fgcolor:component->bgcolor, 
                         component->x + MAX(halignment, BAGL_BUTTON_INNERSPACING), 
                         component->y + MAX(valignment, BAGL_BUTTON_INNERSPACING), 
                         component->width - halignment - (MAX(1,component->stroke)*2+BAGL_BUTTON_INNERSPACING*2), 
                         component->height - valignment - (MAX(1,component->stroke)*2+BAGL_BUTTON_INNERSPACING*2),
                         context,
                         context_length,
                         context_encoding);
      }

      // centered by default
      if ( component->icon_id ) {
        goto case_BAGL_ICON;
      }
      break;
    case BAGL_LABEL: 
      // draw background rect
      if (component->fill == BAGL_FILL) {
        bagl_hal_draw_rect(component->bgcolor, component->x, component->y, component->width, component->height);
      }
      if (context && context_length) {
        // debug centering
        //bagl_hal_draw_rect(component->fgcolor, component->x, component->y, 2, 2);
        bagl_draw_string(component->font_id,
                         component->fgcolor, 
                         component->bgcolor, 
                         component->x + halignment + component->stroke, 
                         component->y + valignment + component->stroke, 
                         component->width - 2*component->stroke - halignment, 
                         component->height - 2*component->stroke - valignment,
                         context,
                         context_length,
                         context_encoding);
        // debug centering
        //bagl_hal_draw_rect(component->fgcolor, component->x+component->width-2, component->y+component->height-2, 2, 2);
      }
      if (component->stroke > 0) {
        goto outline_rect;
      }
      break;

    case BAGL_LINE: // a line is just a flat rectangle :p
      component->fill = BAGL_FILL; // mandat !!
    //case BAGL_RECTANGLE:
      if(component->fill == BAGL_FILL && component->radius == 0) {
        bagl_hal_draw_rect(component->fgcolor, component->x, component->y, component->width, component->height);
      }
      else {
        outline_rect:
        goto draw_round_rect;
        /*
        // not filled, respect the stroke
        // left
        bagl_hal_draw_rect(component->fgcolor, component->x, component->y, MAX(1,component->stroke), component->height);
        // top
        bagl_hal_draw_rect(component->fgcolor, component->x, component->y, component->width, MAX(1,component->stroke));
        // right
        bagl_hal_draw_rect(component->fgcolor, component->x+component->width-MAX(1,component->stroke), component->y, MAX(1,component->stroke), component->height);
        // bottom
        bagl_hal_draw_rect(component->fgcolor, component->x, component->y+component->height-MAX(1,component->stroke), component->width, MAX(1,component->stroke));
        */
      }
      break;

    case BAGL_ICON: {

      bagl_glyph_array_entry_t* glyph_array;
      unsigned int glyph_count;

    case_BAGL_ICON:
      x = component->x;
      y = component->y;

      if (component->icon_id == 0) {
        break;
      }

      // select the default or custom glyph array
      if (component->type == BAGL_ICON && context_encoding && G_glyph_array && G_glyph_count > 0) {
        glyph_array = G_glyph_array;
        glyph_count = G_glyph_count;
      }
      else {
        glyph_array = C_glyph_array;
        glyph_count = C_glyph_count;
      }

      if (component->icon_id >= glyph_count) {
        // glyph doesn't exist, avoid printing random stuff from the memory
        break;
      }

      // color accounted as bytes in the context length
      if (context_length) {
        if ((1<<glyph_array[component->icon_id].bits_per_pixel)*4 != context_length) {
          // invalid color count
          break;
        }
        context_length /= 4;
      }

      // use default colors
      if (!context_length || !context) {
        context_length = 1<<(glyph_array[component->icon_id].bits_per_pixel);
        context = glyph_array[component->icon_id].default_colors;
      }

      // center glyph in rect
      // draw the glyph from the bitmap using the context for colors
      bagl_hal_draw_bitmap_within_rect(x + (component->width / 2 - glyph_array[component->icon_id].width / 2), 
                                       y + (component->height / 2 - glyph_array[component->icon_id].height / 2), 
                                       glyph_array[component->icon_id].width, 
                                       glyph_array[component->icon_id].height, 
                                       context_length, 
                                       (unsigned int*)context, // Endianness remarkably ignored !
                                       glyph_array[component->icon_id].bits_per_pixel, 
                                       glyph_array[component->icon_id].bitmap, 
                                       glyph_array[component->icon_id].bits_per_pixel*(glyph_array[component->icon_id].width*glyph_array[component->icon_id].height));
      break;
    }
    
    case BAGL_CIRCLE:
      // draw the circle (all 8 octants)
      bagl_draw_circle_helper(component->fgcolor, component->x, component->y, component->radius, 0xFF);
      if(component->fill != BAGL_FILL && component->stroke) {
        // hole in the circle (all 8 octants)
        bagl_draw_circle_helper(component->bgcolor, component->x, component->y, component->radius-component->stroke, 0xFF);
      }
      break;

    case BAGL_NONE:
      // performing, but not registering
      bagl_hal_draw_rect(component->fgcolor, component->x, component->y, component->width, component->height);
      return;
    default:
      return;
  }

  /*
  // remember drawn component for user action
  memcpy(&bagl_components[comp_idx], component, sizeof(bagl_component_t));  
  */
}

// --------------------------------------------------------------------------------------

void bagl_draw(bagl_component_t* component) {
  // component without text
  bagl_draw_with_context(component, NULL, 0, 0);
}

#endif // HAVE_BAGL
