/* Copyright (c) 2011 Jorge Pinto - casainho@gmail.com       */
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

#ifndef CONFIG_H
#define CONFIG_H

#include "stdint.h"
#include "stdbool.h"

#include "ios.h"

// TODO: sort out use of max number vs actual number configured
#define NUM_AXES 4

// axis map for 3D printer with 1 extruder for compatibility with existing code
#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2
#define E_AXIS 3


// 6 axes should be enough for anyone?
#define MAX_AXES 6

/*
  // pin config
  input
  output
  pullup/down/etc
  special function: pwm etc
*/

// axis configs: 
// X Y Z E
// X Y Z A
// X Y A B

// values for machine model
#define MM_REPRAP_MENDEL  0
#define MM_RAPMAN         1

typedef struct
{
  bool    is_configured;
  char    letter_code;      // X,Y,Z,A,B,C
  
  double  steps_per_mm;
  int32_t maximum_feedrate;
  double  acceleration;
  
  int32_t home_direction;   // -1 or +1, 0 if no homing
  int32_t homing_feedrate;
  int32_t search_feedrate;
  int32_t home_pos;
  
  int32_t printing_vol;   // axis max travel

  int32_t dir_invert;     // reverse direction of movement, same effect as setting active low/high on direction pin
  
  // IO pin configuration

  // step, dir, enable, reset
  // polarity: active low/high
  // pulse len low,high
  tPinDef pin_step;
  tPinDef pin_dir;
  tPinDef pin_enable;
  tPinDef pin_reset;

  bool    have_min_limit;
  bool    have_max_limit;
  
  tPinDef pin_min_limit;
  tPinDef pin_max_limit;
  
} tAxisSettings;


struct configuration
{
  int32_t machine_model;
  
  int32_t num_axes;
  tAxisSettings axis[MAX_AXES];

  double  acceleration;   // global default
  double  junction_deviation;

  int32_t auto_power_off_time; // seconds
  
  int32_t control_panel;      // = 0 none, 1 = Makerbot
  
  int32_t tcp_ip_enabled;
  int32_t network_interface;

  // rate when homing (fast)
  // direction to move when homing (depends on endstop locations)
  // position at home
  // printable volume size
  
  int32_t debug_flags;
  
  int32_t step_led_flash_method; // how we control the Step pin to flash the stepper LED
  
  // The following are specific to printers

  // dump pos
  int32_t have_dump_pos;
  int32_t dump_pos_x;
  int32_t dump_pos_y;
  
  // rest pos
  int32_t have_rest_pos;
  int32_t rest_pos_x;
  int32_t rest_pos_y;

  // wipe pos
  int32_t have_wipe_pos;
  int32_t wipe_entry_pos_x;
  int32_t wipe_entry_pos_y;
  int32_t wipe_exit_pos_x;
  int32_t wipe_exit_pos_y;
  
  //
  int32_t steps_per_revolution_e;
  
  // options
  int32_t wait_on_temp;
  int32_t enable_extruder_1;
} tApplicationConfiguration;

extern struct configuration config;

void read_config (void);
void print_config (void);

#endif /* CONFIG_H */
