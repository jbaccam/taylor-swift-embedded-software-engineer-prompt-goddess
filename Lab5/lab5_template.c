/**
 * lab5_template.c
 *
 * Template file for CprE 288 Lab 5
 *
 * @author Zhao Zhang, Chad Nelson, Zachary Glanz
 * @date 08/14/2016
 *
 * @author Phillip Jones, updated 6/4/2019
 * @author Diane Rover, updated 2/25/2021, 2/17/2022
 */

#include "button.h"
#include "timer.h"
#include "lcd.h"
#include "uart.h"

#include "cyBot_uart.h"  // Functions for communicating between CyBot and Putty (via UART1)
// PuTTy: Baud=115200, 8 data bits, No Flow Control, No Parity, COM1

#include "cyBot_Scan.h"  // Scan using CyBot servo and sensors

#define REPLACEME 0
#define MAX_CHARS 20

int main(void)
{
    timer_init(); // Must be called before lcd_init(), which uses timer functions
    lcd_init();
    button_init();

    cyBot_uart_init_clean();  // Clean UART1 initialization, before running your UART1 GPIO init code
    uart_init();

    uart_sendStr("Type up to 20 chars or press ENTER.\r\n");

    while (1)
    {
        // buffer and count for the new string
        char buffer[MAX_CHARS + 1];
        int count = 0;
        buffer[0] = '\0';

        // get chars until ENTER or 20 chars
        while (1)
        {
            char c = uart_receive();

            // user pressed ENTER, break
            if (c == '\r')
            {
                break;
            }

            // otherwise store if we have space
            if (count < MAX_CHARS)
            {
                buffer[count] = c;
                count++;
                buffer[count] = '\0'; // keep null-terminated
            }

            // if we reached 20 chars, break
            if (count == MAX_CHARS)
            {
                break;
            }
        }

        // clear the LCD
        lcd_clear();

        // print entire string on the first line
        if (count == 20) {
            lcd_printf("%s%d", buffer, count);
        } else {
            lcd_printf("%s\n%d", buffer, count);
        }

        // send the entire string back to PuTTY
        char temp[50];
        sprintf(temp, "You typed:\r\n%s\r\n%d\r\n", buffer, count);
        uart_sendStr(temp);
        timer_waitMillis(100);

    }

    return 0;
}
