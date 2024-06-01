#ifndef Macros_H
#define Macros_H

#define Red_Led 1
#define Blue_Led 2
#define Green_Led 3

#define setBit(Register, Bit) (Register |= (1 << Bit))
#define clearBit(Register, Bit) (Register &= ~(1 << Bit))

void Motor_Up(void);
void Motor_Down(void);
void Motor_Off(void);
void Leds_Off(void);
#endif
