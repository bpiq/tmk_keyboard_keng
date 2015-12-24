/*
Copyright 2015 Kai Ryu <kai1103@gmail.com>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
 * scan matrix
 */
#include <stdint.h>
#include <stdbool.h>
#include <avr/io.h>
#include <util/delay.h>
#include "timer.h"
#include "print.h"
#include "debug.h"
#include "util.h"
#include "matrix.h"
#include "keymap_common.h"
#include "rgb.h"


#ifndef DEBOUNCE
#   define DEBOUNCE	5
#endif
static uint8_t debouncing = DEBOUNCE;

/* matrix state(1:on, 0:off) */
static matrix_row_t matrix = 0;
static matrix_row_t matrix_debouncing = 0;

static matrix_row_t read_cols(void);
static void init_cols(void);

inline
uint8_t matrix_rows(void)
{
    return MATRIX_ROWS;
}

inline
uint8_t matrix_cols(void)
{
    return MATRIX_COLS;
}

void matrix_init(void)
{
    rgb_init();

    // initialize cols
    init_cols();
}

#ifdef KENG
uint8_t pos=1;
uint8_t pos2=0;
void Wheel(uint8_t WheelPos, struct cRGB *rgb) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
	  rgb->r = 255 - WheelPos * 3;
	  rgb->g = 0;
	  rgb->b = WheelPos * 3;
    return;
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
	  rgb->b = 255 - WheelPos * 3;
	  rgb->r = 0;
	  rgb->g = WheelPos * 3;
	  return;
  }
  WheelPos -= 170;
	  rgb->g = 255 - WheelPos * 3;
	  rgb->b = 0;
	  rgb->r = WheelPos * 3;
}

void rgb_refresh2(void)
{

    //struct cRGB rgb_color[1];
    struct cRGB rgb_color[6];
    Wheel(pos, rgb_color);
    Wheel(pos+30, &(rgb_color[1]));
    Wheel(pos+60, &(rgb_color[2]));
    Wheel(pos+90, &(rgb_color[3]));
    Wheel(pos+120, &(rgb_color[4]));
    Wheel(pos+150, &(rgb_color[5]));
    //ws2812_setleds(rgb_color, 1);
    ws2812_setleds(rgb_color, 6);
}
#endif

uint8_t matrix_scan(void)
{

#ifdef KENG
	pos2+=2;
	if(pos2==0)
	{
		pos++;
		rgb_refresh2();
	}
#endif

    matrix_row_t cols = read_cols();
    if (matrix_debouncing != cols) {
        matrix_debouncing = cols;
        if (debouncing) {
            debug("bounce!: "); debug_hex(debouncing); debug("\n");
        }
        debouncing = DEBOUNCE;
    }

    if (debouncing) {
        if (--debouncing) {
            _delay_ms(1);
        } else {
            matrix = matrix_debouncing;
        }
    }

    return 1;
}

bool matrix_is_modified(void)
{
    if (debouncing) return false;
    return true;
}

inline
bool matrix_is_on(uint8_t row, uint8_t col)
{
    return (matrix & ((matrix_row_t)1<<col));
}

inline
matrix_row_t matrix_get_row(uint8_t row)
{
    return matrix;
}

void matrix_print(void)
{
    print("\nr/c 0123456789ABCDEF\n");
    for (uint8_t row = 0; row < MATRIX_ROWS; row++) {
        phex(row); print(": ");
        pbin_reverse16(matrix_get_row(row));
        print("\n");
    }
}

uint8_t matrix_key_count(void)
{
    return bitpop16(matrix);
}

#ifdef KENG
/* Column pin configuration
 * col: 0   1   2   3   4
 * pin: B1  B2  B3  C6  C7
 */
static void  init_cols(void)
{
    // Input with pull-up(DDR:0, PORT:1)
    DDRB  &= ~(1<<PB1 | 1<<PB2 | 1<<PB3);
    PORTB |=  (1<<PB1 | 1<<PB2 | 1<<PB3);
    DDRC  &= ~(1<<PC6 | 1<<PC7);
    PORTC |=  (1<<PC6 | 1<<PC7);
}

/* Column pin configuration
 * col: 0   1   2   3   4
 * pin: B1  B2  B3  C6  C7
 */
static matrix_row_t read_cols(void)
{
	  return (PINB&(1<<PB1) ? 0 : (1<<0)) |
           (PINB&(1<<PB2) ? 0 : (1<<1)) |
           (PINB&(1<<PB3) ? 0 : (1<<2)) |
           (PINC&(1<<PC6) ? 0 : (1<<3)) |
           (PINC&(1<<PC7) ? 0 : (1<<4));
}
#else
/* Column pin configuration
 * col: 0   1   2   3   4
 * pin: D0  D1  D2  D3  D4
 */
static void  init_cols(void)
{
    // Input with pull-up(DDR:0, PORT:1)
    DDRD  &= ~(1<<PD0 | 1<<PD1 | 1<<PD2 | 1<<PD3 | 1<<PD4);
    PORTD |=  (1<<PD0 | 1<<PD1 | 1<<PD2 | 1<<PD3 | 1<<PD4);
}

/* Column pin configuration
 * col: 0   1   2   3   4
 * pin: D0  D1  D2  D3  D4
 */
static matrix_row_t read_cols(void)
{
    return (~PIND) & 0b00011111;
}
#endif
