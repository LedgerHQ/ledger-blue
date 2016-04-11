#!/bin/bash
#*******************************************************************************
#*   Ledger Blue - Non secure firmware
#*   (c) 2016 Ledger
#*
#*  Licensed under the Apache License, Version 2.0 (the "License");
#*  you may not use this file except in compliance with the License.
#*  You may obtain a copy of the License at
#*
#*      http://www.apache.org/licenses/LICENSE-2.0
#*
#*   Unless required by applicable law or agreed to in writing, software
#*   distributed under the License is distributed on an "AS IS" BASIS,
#*   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#*   See the License for the specific language governing permissions and
#*   limitations under the License.
#********************************************************************************/

sed -e 's/const UCHAR abc_font/const unsigned char bitmap/g' \
    -e 's/const BFC_CHARINFO font/const bagl_font_character_t characters/g' \
    -e 's/, abc_font/, bitmap/g' \
    -e 's/_CharInfo//g' \
    $1 > $1.tmp

linecount=`cat $1 | wc -l`
linecount=$(($linecount - 14))
head -n $linecount $1.tmp > $1.tmp1

sed '0,/extern const/d' $1.tmp1 > $1.tmp2


# const BFC_FONT_PROP fontOPEN_SANS_LIGHT18H_Prop1 = {
# 13:    0x0020, /* first character */
# 12:    0x007F, /* last character  */
# 11:    &fontOPEN_SANS_LIGHT18H_CharInfo[   0],  /* address of first character */
# 10:    (const BFC_FONT_PROP *)0				/* pointer to next BFC_FONT_PROP */
# };

# 7: const BFC_FONT fontOPEN_SANS_LIGHT18H = {
# 6:   FONTTYPE_PROP | ENCODING_ASCII | DATALENGTH_8, /* font type */
# 5:   18 , /* font height in pixels   */
# 4:   13 , /* font ascent (baseline) in pixels  */
# 3:   0  , /* reversed, =0 */
# 2:   {&fontOPEN_SANS_LIGHT18H_Prop1}  
# 1: };

# font struct name
cat $1.tmp | tail -n 7 | head -n 1| sed -e 's/const BFC_FONT /const bagl_font_t /g' >> $1.tmp2
# font height
cat $1.tmp | tail -n 5 | head -n 1 >> $1.tmp2
# ascent
cat $1.tmp | tail -n 4 | head -n 1 >> $1.tmp2
# kerning
echo "   0, /* kerning */" >> $1.tmp2
# first char
cat $1.tmp | tail -n 13 | head -n 1 >> $1.tmp2
#last char
cat $1.tmp | tail -n 12 | head -n 1 >> $1.tmp2
# bitmap indirect array
cat $1.tmp | tail -n 7 | head -n 1 | cut -f1 -d "=" | sed -e 's/const BFC_FONT font/   \&characters/g' >> $1.tmp2
# end of struct
echo "};" >> $1.tmp2

# const bagl_font_t fontLUCIDA_CONSOLE_15 = {
#    15 , /* font height in pixels   */
#    12 , /* baseline distance from top */
#    0, /* kerning */
#    0x0020, /* first character */
#    0x00FF, /* last character  */
#    &charactersLUCIDA_CONSOLE_15[0] /* address of first character */
# };
