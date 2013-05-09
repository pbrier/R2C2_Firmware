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

#ifndef _INKSHIELD_H
#define _INKSHIELD_H

#define INK_A_PORT           1 
#define INK_A_BIT            (1<<0) 

#define INK_B_PORT           1   
#define INK_B_BIT            (1<<9) 

#define INK_C_PORT           1 
#define INK_C_BIT            (1<<10) 

#define INK_D_PORT           1 
#define INK_D_BIT            (1<<14)
 
#define INK_P_PORT           2 
#define INK_P_BIT            (1<<1) 

extern void ink_init();        // Initialize IO
extern void ink_fire(unsigned int n);  // Fire nozzle [n] (1..12)
extern void ink_enable(unsigned int n); // bitmask of nozzles to enable
extern void ink_set_pulse_length(unsigned int len); // set pulse length (in approx 500nsec units)
#include "ios.h"
#include "machine.h"

#endif