## Timers
Timer0 8bit: Used internally by arduino sdk

Timer1 16bit: Encoder timer
Encoder frequeny at max motor speed probaly around 20-30kHz
Use Prescaler 8 
  -> 16MHz / (8*2^16) = ~30Hz encoder freq => ~2 rpm min speed
  -> 16MHz / (8*25000) = 80 (enough resolution at max speed)

Timer2 8bit: Motor PWM
Use Prescaler 32 -> 16MHZ / (256 * 32) = ~1.95kHz PWM frequency
- decoupling of motor unclear, higher freq would require changing decoupling capacitor
- probably a bit noisy but whatever, efficiency should be ok

## Control loop

time_per_pulse = smooth_encoder_count / ENCODER_TICK
current_pulses = 1 / time_per_pulse
current_rps = current_pulses / ENCODER_PULSES

e = target_speed - current_rps
u_i += e * Ki

*Windup limiting*
u_i = clamp(u_i, -Li, +Li)

u = e * Kp + u_i

PWM = int(clamp(u, 0, 0xFF))

### Simplification
current_rps = (1 / (smooth_encoder_count / ENCODER_TICK)) / ENCODER_PULSES

= (ENCODER_TICK / smooth_encoder_count) / ENCODER_PULSES

= (ENCODER_TICK / ENCODER_PULSES) / smooth_encoder_count

### Anti-Windup
Schema from https://doc.synapticon.com/node/sw5.1/motion_control/advanced_control_options/anti_windup.html
