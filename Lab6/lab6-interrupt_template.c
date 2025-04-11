/**
 * lab6-interrupt_template.c
 *
 * Example solution for Lab 6, Part 2
 * Demonstrates interrupt-based UART receive/echo,
 * with NO calls to any blocking or non-blocking uart_receive().
 *
 * @author Jeremiah Baccam, Luke Patterson
 * @date 2025-03-13
 */

#include "Timer.h"
#include "lcd.h"
#include "cyBot_Scan.h"
#include "cyBot_uart.h"

#include "uart-interrupt.h"

#define LEFT_CAL  1335250
#define RIGHT_CAL 343000

int main(void)
{
    timer_init();    // Must be called before lcd_init()
    lcd_init();
    uart_interrupt_init();
    cyBOT_init_Scan(0b0111);

    uart_sendStr("Type characters in PuTTY, they are echoed back.\r\n");

    // Loop forever. The actual echo is done by the ISR, not by main code.
    while (1)
    {
        // do nothing here, the UART1_Handler ISR is triggered whenever
        // a new character arrives from PuTTY, and it sends that character back.
    }

    return 0;
}
