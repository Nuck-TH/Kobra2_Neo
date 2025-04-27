/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "../platforms.h"

#ifdef HAL_STM32

#include "../../inc/MarlinConfig.h"

// Array to support sticky frequency sets per timer
static uint16_t timer_freq[TIMER_NUM];

void MarlinHAL::set_pwm_duty(const pin_t pin, const uint16_t v, const uint16_t v_size/*=255*/, const bool invert/*=false*/) {
  const uint16_t duty = invert ? v_size - v : v;
  if (PWM_PIN(pin)) {
    const PinName pin_name = digitalPinToPinName(pin);
    TIM_TypeDef * const Instance = (TIM_TypeDef *)pinmap_peripheral(pin_name, PinMap_PWM);

    const timer_index_t index = get_timer_index(Instance);
    const bool needs_freq = (HardwareTimer_Handle[index] == nullptr);
    if (needs_freq) // A new instance must be set to the default frequency of PWM_FREQUENCY
      HardwareTimer_Handle[index]->__this = new HardwareTimer((TIM_TypeDef *)pinmap_peripheral(pin_name, PinMap_PWM));

    HardwareTimer * const HT = (HardwareTimer *)(HardwareTimer_Handle[index]->__this);
    const uint32_t channel = STM_PIN_CHANNEL(pinmap_function(pin_name, PinMap_PWM));
    const TimerModes_t previousMode = HT->getMode(channel);
    if (previousMode != TIMER_OUTPUT_COMPARE_PWM1)
      HT->setMode(channel, TIMER_OUTPUT_COMPARE_PWM1, pin);

    if (needs_freq && timer_freq[index] == 0)     // If the timer is unconfigured and no freq is set then default PWM_FREQUENCY
      set_pwm_frequency(pin_name, PWM_FREQUENCY); // Set the frequency and save the value to the assigned index no.

    // Note the resolution is sticky here, the input can be upto 16 bits and that would require RESOLUTION_16B_COMPARE_FORMAT (16)
    // If such a need were to manifest then we would need to calc the resolution based on the v_size parameter and add code for it.
    
    const PinName fan_pin_name = digitalPinToPinName(pin_t(FAN_PIN));
    const uint32_t fan_channel = STM_PIN_CHANNEL(pinmap_function(fan_pin_name, PinMap_PWM));
    if( (pin_name == fan_pin_name) && (channel == fan_channel) ) { //PB5 TIM3 CH2
      int16_t duty_calc = 0;
      if(duty < 5) { //0..5 f(x) = 72.2824 * (x)/5 Rup
        duty_calc = 72 * int32_t(duty) / 5 + 1;
      } else if(duty < 145) { //5..145 f(x) = (168.6589*x + 9276.2348)/140 Rdown
        duty_calc = (169 * int32_t(duty) + 9276) / 140;
      } else if(duty < 200) { //145..200 f(x) = (481.8823*x - 56621.1675)/55 Rdown
        duty_calc = (482 * int32_t(duty) - 56621) / 55;
      } else { //200..255 f(x) = (3373.1765*x - 634880.0075)/55 Rup
        duty_calc = (3373 * int32_t(duty) - 634880) / 55 + 1;
      }
      duty_calc = duty_calc < 0 ? 0 : ( duty_calc > 4095 ? 4095 : duty_calc );
      HT->setCaptureCompare(channel, duty_calc, RESOLUTION_12B_COMPARE_FORMAT);
    } else { //any other
      HT->setCaptureCompare(channel, duty, RESOLUTION_8B_COMPARE_FORMAT); // Set the duty, the calc is done in the library :)
    }
    
    pinmap_pinout(pin_name, PinMap_PWM); // Make sure the pin output state is set.
    if (previousMode != TIMER_OUTPUT_COMPARE_PWM1) HT->resume();
  }
  else {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, duty < v_size / 2 ? LOW : HIGH);
  }
}

void MarlinHAL::set_pwm_frequency(const pin_t pin, const uint16_t f_desired) {
  if (!PWM_PIN(pin)) return; // Don't proceed if no hardware timer
  const PinName pin_name = digitalPinToPinName(pin);
  TIM_TypeDef * const Instance = (TIM_TypeDef *)pinmap_peripheral(pin_name, PinMap_PWM); // Get HAL timer instance
  const timer_index_t index = get_timer_index(Instance);

  // Protect used timers.
  #ifdef STEP_TIMER
    if (index == TIMER_INDEX(STEP_TIMER)) return;
  #endif
  #ifdef TEMP_TIMER
    if (index == TIMER_INDEX(TEMP_TIMER)) return;
  #endif
  #if defined(PULSE_TIMER) && MF_TIMER_PULSE != MF_TIMER_STEP
    if (index == TIMER_INDEX(PULSE_TIMER)) return;
  #endif

  if (HardwareTimer_Handle[index] == nullptr) // If frequency is set before duty we need to create a handle here.
    HardwareTimer_Handle[index]->__this = new HardwareTimer((TIM_TypeDef *)pinmap_peripheral(pin_name, PinMap_PWM));
  HardwareTimer * const HT = (HardwareTimer *)(HardwareTimer_Handle[index]->__this);
  HT->setOverflow(f_desired, HERTZ_FORMAT);
  timer_freq[index] = f_desired; // Save the last frequency so duty will not set the default for this timer number.
}

#endif // HAL_STM32
