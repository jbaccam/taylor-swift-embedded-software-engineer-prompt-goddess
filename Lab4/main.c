/* Jereiah and kube main c
 * */

#include "Timer.h"
#include "button.h"
#include "lcd.h"
#include "open_interface.h"
#include "cyBot_uart.h"

int main(void)
{
    timer_init();       // Initialize the timer
    button_init();      // Initialize Port E buttons
    lcd_init();         // Initialize the LCD
    cyBot_uart_init();

    // Correct way to initialize a string in C
    char nums[] = "1234";

    while (1)
    {

        int result = button_getButton();

        // Each button_getButton() return value is a bitmask
        // 0b00000001 means PE0 pressed
        // 0b00000010 means PE1 pressed
        // 0b00000100 means PE2 pressed
        // 0b00001000 means PE3 pressed

        if (result == 0b00000001)
        {
            lcd_printf("%c", nums[0]);  // Print "1"
            cyBot_sendByte(nums[0]);
            timer_waitMillis(100);
        }
        else if (result == 0b00000010)
        {
            lcd_printf("%c", nums[1]);  // Print "2"
            cyBot_sendByte(nums[1]);
            timer_waitMillis(100);

        }
        else if (result == 0b00000100)
        {
            lcd_printf("%c", nums[2]);  // Print "3"
            cyBot_sendByte(nums[2]);
            timer_waitMillis(100);

        }
        else if (result == 0b00001000)
        {
            lcd_printf("%c", nums[3]);  // Print "4"
            cyBot_sendByte(nums[3]);
            timer_waitMillis(100);

        }
        // If result == 0, no button is pressed, so do nothing
    }

    return 0;
}
