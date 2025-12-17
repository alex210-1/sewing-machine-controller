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
