/*##############################################################################

    RGB-CTL · 2011 Florian Knodt <adlerweb@adlerweb.info>

    TWI-Stuff based on:
        USI TWI Slave driver 1.3
            Martin Junghans <jtronics@gmx.de>
            Markus Schatzl
    RGB-Stuff based on:
        moodlamp-rf - fnordlicht firmware next generation
             Alexander Neumann <alexander@bumpern.de>
            Lars Noschinski <lars@public.noschinski.de>
            Kiu
            Mazzoo
            Tobias Schneider(schneider@blinkenlichts.net)
            Soft-PWM - http://www.mikrocontroller.net/articles/Soft-PWM

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License version 2 as
  published by the Free Software Foundation.
 
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 
  For more information on the GPL, please go to:
  http://www.gnu.org/copyleft/gpl.html

//# CONFIGURATION ############################################################*/

// TWI Address (LSB must be 0!)
// Define is inside the Makefile
#define     SLAVE_ADDR_ATTINY       TWI_ADDRESS
  
//# IMPORT LIBS ##############################################################*/

#include    <stdlib.h>
#include    <avr/io.h>
#include    <avr/interrupt.h>
#include    <avr/pgmspace.h>
#include    <avr/wdt.h>
#include    <avr/sleep.h>
#include    "usiTwiSlave.h"

#ifndef     F_CPU
    #define     F_CPU 8000000UL
#endif

//# MACROS ###################################################################*/

#define     uniq(LOW,HEIGHT)    ((HEIGHT << 8)|LOW)         // Create 16 bit number from two bytes
#define     LOW_BYTE(x)         (x & 0xff)                  // Get low byte from 16 bit number  
#define     HIGH_BYTE(x)        ((x >> 8) & 0xff)           // Get high byte from 16 bit number

#define     sbi(ADDRESS,BIT)    ((ADDRESS) |= (1<<(BIT)))   // Set bit
#define     cbi(ADDRESS,BIT)    ((ADDRESS) &= ~(1<<(BIT)))  // Clear bit
#define     toggle(ADDRESS,BIT) ((ADDRESS) ^= (1<<BIT))     // Toggle bit

#define     bis(ADDRESS,BIT)    (ADDRESS & (1<<BIT))        // Is bit set?
#define     bic(ADDRESS,BIT)    (!(ADDRESS & (1<<BIT)))     // Is bit clear?

//# GLOBAL VARS ##############################################################*/

uint8_t  fade[3]   = {0,0,0};                   //Color fade delay
uint8_t  target[3] = {0,0,0};                   //Target color
uint8_t  color[3]  = {0,0,0};                   //Active color
uint16_t colort[3] = {0,0,0};                   //Timer compare for active color
volatile uint8_t pwmstate=0;                   //Used to detect a completed PWM-Cycle
volatile uint16_t pwm_cnt=0;                    //PWM-Cycle counter

ISR(TIMER1_COMPA_vect) {
    TCNT1 = 0;
    if (colort[0] <= pwm_cnt) 
		cbi(PORTB, PB1);
    if (colort[1] <= pwm_cnt) 
		cbi(PORTB, PB3);
    if (colort[2] <= pwm_cnt) 
		cbi(PORTB, PB4);
    if (pwm_cnt>=768) {
        pwm_cnt=0;
        pwmstate=0;
        if(colort[0] != 0) sbi(PORTB, PB1);
        if(colort[1] != 0) sbi(PORTB, PB3);
        if(colort[2] != 0) sbi(PORTB, PB4);
    } else
        pwm_cnt++;
}

void i2c_process(void) {
        //###Process incoming I²C-Data###

        //Target Colors
        if(twibuffer[0]<=170) target[0] = twibuffer[0];
        if(twibuffer[1]<=170) target[1] = twibuffer[1];
        if(twibuffer[2]<=170) target[2] = twibuffer[2];

        //Fade type
        fade[0] = twibuffer[3];
        fade[1] = twibuffer[4];
        fade[2] = twibuffer[5];

        //###Write new values to twibuffer so we can read current status###
        twibuffer[6] = color[0];
        twibuffer[7] = color[1];
        twibuffer[8] = color[2];

        twibuffer[9] = LOW_BYTE(colort[0]);
        twibuffer[10] = HIGH_BYTE(colort[0]);
        twibuffer[11] = LOW_BYTE(colort[1]);
        twibuffer[12] = HIGH_BYTE(colort[1]);
        twibuffer[13] = LOW_BYTE(colort[2]);
        twibuffer[14] = HIGH_BYTE(colort[2]);
}

void pwm_update(void) {
    //wait until current pwm-sequence is completed
    pwmstate=1;
    while(pwmstate != 0)  {
		set_sleep_mode(SLEEP_MODE_IDLE);
        sleep_mode();
	}
}

//# MAIN LOOP ################################################################*/
int main(void)
{     
  
    /*  Red   = PB1
        Green = PB3
        Blue  = PB4
    */

    uint8_t wait[3]={0,0,0};                        //Fading delay temp storage
    uint8_t i=0;                                    //Color processing loop temp storage

    static const uint16_t timeslot_table[] PROGMEM =
    { 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 9, 9, 10, 10, 11, 11, 11, 12, 13, 13, 14, 14, 15, 16, 16, 17, 18, 19, 19, 20, 21, 22, 23, 24, 25, 26, 27, 29, 30, 31, 32, 34, 35, 37, 39, 40, 42, 44, 46, 48, 50, 52, 55, 57, 60, 62, 65, 68, 71, 74, 77, 81, 84, 88, 92, 96, 100, 105, 109, 114, 119, 125, 130, 136, 142, 148, 155, 161, 169, 176, 184, 192, 200, 209, 219, 228, 238, 249, 260, 272, 284, 296, 309, 323, 337, 352, 368, 384, 401, 419, 437, 457, 477, 498, 520, 543, 567, 592, 618, 646, 674, 704, 735, 768 }; //Werte 0 - 170

    cli();                                          //Disable interrupts
    wdt_enable(WDTO_500MS);                         //Enable Watchdog

    DDRB = 0xFF;                                    //Port als Ausgang
    PORTB = 0x00;                                   //LEDs aus

    usiTwiSlaveInit(SLAVE_ADDR_ATTINY);             //TWI slave init

    sbi(TCCR1,CS10);                                //Timer ohne Prescaler
    OCR1A = 128;                                    //PWM-Takt
    TIMSK |= (1<<OCIE1A);                           //Interrupt freischalten*/

    sei();                                          //Enable Interrupts
    while(1) {
        wdt_reset();                                //Watchdog reset

        i2c_process();                              //Read I²C and write current status

        //Fading and PWM-programming
        for(i=0; i<=2; i++) {
            if(target[i] != color[i]) {
                if(fade[i] == 0) {
                    color[i]=target[i];
                }else{
                    if(wait[i] >= fade[i]) {
                        if(target[i] > color[i]) color[i]++;
                        else color[i]--;
                        wait[i]=0;
                    }else wait[i]++;
                }   
                colort[i]=pgm_read_word(&timeslot_table[color[i]]);
            }
        }

        wdt_reset();

        pwm_update(); //Controller waits there until one PWM-cycle has completed
    }
}

