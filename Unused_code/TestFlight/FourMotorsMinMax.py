from machine import Pin, PWM

esc_1 = PWM(Pin(15)) # Top Left - weaker one - changed to pin 15, originally pin 14 
esc_2 = PWM(Pin(2)) # bottom left - ccw
esc_3 = PWM(Pin(28)) # Bottom right
esc_4 = PWM(Pin(16)) # Top right - ccw

esc_1.freq(50)
esc_2.freq(50)
esc_3.freq(50)
esc_4.freq(50)

def set_throttle(pulse_width_ms):
    period_ms = 20  # 1 / 50 Hz = 20 ms
    duty_cycle = int((pulse_width_ms / period_ms) * 65535)
#     esc_1.duty_u16(duty_cycle)
#     esc_2.duty_u16(duty_cycle)
#     esc_3.duty_u16(duty_cycle)
    esc_4.duty_u16(duty_cycle)
#     
#max throttle 
#set_throttle(2)

#min throttle
set_throttle(1)

