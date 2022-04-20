#include "example_usart.h"
#include "params.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_tim_ex.h"
#include "xnucleoihm02a1.h"
#include <math.h>
#include <stdlib.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_conf.h"
#include "stm32f4xx_hal_rcc.h"
#include "example_usart.h"

long map(long x, long in_min, long in_max, long out_min, long out_max);
void delay(int ms);
long map1(long x, long in_min, long in_max, long out_min, long out_max);
void servo_write(uint8_t angle);
void servo_sweep(void);
