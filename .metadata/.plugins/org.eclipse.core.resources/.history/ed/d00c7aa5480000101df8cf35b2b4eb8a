

/**
 * main.c
 */
#include "Timer.h"
#include "lcd.h"
#include <string.h>

void lcd_rotatingBanner();

int main (void) {
    timer_init(); //init timer
    lcd_init();   // init lcd (clears display)

    lcd_rotatingBanner();

    return 0;
}


void lcd_rotatingBanner() {
    char message[] = "Bingus Mario kart";
    int length = strlen(message);
    int displayWidth = 20;
    int i;

    char scrollBuffer[displayWidth + length];

    while(1) {
        memset(scrollBuffer, ' ', displayWidth + length); //set all characters in the scrollBuffer array to ' '

        strcpy(scrollBuffer + displayWidth, message); // copy message into scroll buffer starting after the displayWidth of the screen

        // loop to shift the message down the lcd and display the substring of the scrollBuffer
        for (i = 0; i < length + displayWidth; i++) {
            // print a substring of scrollBuffer with a fixed width of displayWidth, using string formatting
            lcd_printf("%.*s", displayWidth, scrollBuffer + i);



            // wait for 300ms
            timer_waitMillis(300);
        }

        lcd_printf("                    ");  // clear the screen and wait before starting over
        timer_waitMillis(300);
    }

}
