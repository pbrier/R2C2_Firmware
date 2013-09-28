/* Copyright (C) 2013 Peter Brier   */
/* All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/

#include "r2c2.h"
#include "inkshield.h"
#include "debug.h"
#include "pinout.h"
#include "machine.h"
#include "config.h"

#include "lpc17xx_timer.h"
static unsigned int ink_enable_mask = 0xFFFF;
static unsigned int ink_pulse_length = 20;

#define INK_BUFFER 512 // nr of bytes in inkshield buffer
static unsigned char ink_buffer[INK_BUFFER];
static int ink_write = 0;
static int ink_read = 0;
static int ink_read_mask = 1; 
static int ink_nozzle = 0; // 0..11




/* 
 * Configure pin for output 
 */
void ink_pin_cfg(int port, int pin)
{
  PINSEL_CFG_Type PinCfg;

  PinCfg.Funcnum = PINSEL_FUNC_0; /* GPIO function */
  PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
  PinCfg.Pinmode =  PINSEL_PINMODE_PULLDOWN;
  PinCfg.Portnum = port;
  PinCfg.Pinnum = pin;
  PINSEL_ConfigPin(&PinCfg);
  digital_write(port,pin, 0);
  pin_mode(port,pin,OUTPUT);
}



/**
*** Reset the ink buffer
**/
void ink_reset()
{
  sersendf("write/read: %d %d\r\n", ink_write, ink_read);
  ink_write = 0;
  ink_read = 0;
  ink_buffer[0] = 0;
  ink_enable_mask = 0;
  ink_nozzle = 0;
  ink_read_mask = 1;
}

/**
*** Send byte to ink buffer
**/
void ink_send(unsigned int n)
{
  ink_buffer[ink_write] = n;
  ink_write++;
  if ( ink_write >= INK_BUFFER )
    ink_write = 0;
}



/**
*** Initialize the inkshield I/O
**/
void ink_init()
{
  ink_pin_cfg(INK_A_PORT, INK_A_BIT);
  ink_pin_cfg(INK_B_PORT, INK_B_BIT);
  ink_pin_cfg(INK_C_PORT, INK_C_BIT);
  ink_pin_cfg(INK_D_PORT, INK_D_BIT);
  ink_pin_cfg(INK_P_PORT, INK_P_BIT);
  ink_reset();
}

/* Enable specific nozzles */
void ink_enable(unsigned int mask)
{
  ink_enable_mask = mask;
}

/* Set the pulse length */
void ink_set_pulse_length(unsigned int len)
{
  if ( len > 15 ) len = 15;
  ink_pulse_length = len;
}

/*
 * Fire a single nozzle (only if enabled)
 */
void ink_fire(unsigned int c)
{
  int val;
  
  val = 
   (ink_nozzle & 1 ? INK_A_BIT : 0) |
   (ink_nozzle & 2 ? INK_B_BIT : 0) |
   (ink_nozzle & 4 ? INK_C_BIT : 0) |
   (ink_nozzle & 8 ? INK_D_BIT : 0);
  if ( ink_buffer[ink_read] & ink_read_mask )
  {
    digital_write(INK_A_PORT, val, 1 );
    digital_write(INK_A_PORT, val, 1 );
    for(int i=0; i<ink_pulse_length; i++) // effectively a delay
      digital_write(INK_P_PORT, INK_P_BIT, 1 );
    digital_write(INK_P_PORT, INK_P_BIT, 0 );
    digital_write(INK_A_PORT, val,0 );
  }

  // Increment nozzle number and data bit/byte pointers
  ink_nozzle++;
  if ( ink_nozzle > 11) 
    ink_nozzle = 0;
  ink_read_mask = ink_read_mask << 1;
  if ( ink_read_mask > 128 )
  {
    ink_read_mask = 1;
    ink_read++;
  }    
  if ( ink_read >= ink_write ) 
  {
    ink_read = 0;
    ink_read_mask = 1;
  }
}


// set the pulse length 
// MR0 causes the timer to reset ands stop
// MR5 SET 
// MR6 CLEAR
// we need a sequence: CLEAR, SET, STOP to make a pulse
// length is in usec, so we need to convert to ticks (25Mhz clock)
void PWM1_length(long l)
{
  l *= 25;
  LPC_PWM1->LER = 1<<0 | 1<<1 | 1<<2 | 1<<3 | 1<<4 | 1<<5 | 1<<6; // latch all new settings
  LPC_PWM1->MR6 = 10; // clear
  LPC_PWM1->MR5 = l; // set
  LPC_PWM1->MR0 = l+10; // stop 
  LPC_PWM1->TC = 0; 
}


// enable the timer
void PWM1_pulse()
{
  LPC_PWM1->TCR = 1;        // Enable Timer 
}

// init the pwm to make pulses, we select PWM1.6 (P2.5) to output the pulse
// in double edge mode MR5 sets and MR6 clears the register
// The output needs to be set ON at rest, and for a fixed period to OFF when we want to pulse
// so we output 6, 
// RESET by MR6 (at t=1)
// SET by MR5 (at t=tend and stopped)
// to make a new pulse, the counter is reset to zero
void pwm_init()
{
  PINSEL_CFG_Type PinCfg;
  PinCfg.Funcnum = PINSEL_FUNC_1; /* PWM function */
  PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
  PinCfg.Pinmode = PINSEL_PINMODE_PULLDOWN;
  PinCfg.Portnum = PINSEL_PORT_2;
  PinCfg.Pinnum = PINSEL_PIN_5;
  PINSEL_ConfigPin(&PinCfg);

  LPC_SC->PCONP |= 1 <<6;   // Power on PWM1
  LPC_PWM1->CTCR = 0;       // counter control register: 0 == timer mode
  LPC_PWM1->PR = 0;         // prescale pclck = 24MHz. if PCR is set to 0: clk = 24MHz, 1: clk = pclk/2 = 12MHz
  LPC_PWM1->PCR = 1<<2 | 1<<4 | 1<< 6 | 1<<10 | 1<<12 | 1<<14; // PWM control: double edge PWM and outputs enable for 2,4,6
  PWM1_length(100); // pulses

  LPC_PWM1->MCR = (1<<1) | (1<<2);  //  Reset and stop timer on MR0
  LPC_PWM1->TCR = 1;        // Enable Timer mode      
  LPC_PWM1->TC = 0; // set counter to zero
}





