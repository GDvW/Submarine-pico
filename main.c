#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include <string.h>
 

#define UART_ID uart0
#define BAUD_RATE 115200

// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#define UART_TX_PIN 0
#define UART_RX_PIN 1

const uint LED_PIN = 25;

int main() {
    stdio_init_all();
    // Set up our UART with the required speed.
    uart_init(UART_ID, BAUD_RATE);
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Use some the various UART functions to send out data
    unsigned long long messageReceived = 0x000uLL;
    uint messageLengthReceived = 0;
    gpio_put(LED_PIN, 1);
    while (true){
        if (uart_is_readable(UART_ID)){
            int read = uart_getc(UART_ID);
            if (read & 0b00000001){
                printf("received total of %llu, length was %u of %u\n", messageReceived, messageLengthReceived, MESSAGE_LENGTH);  
            }
            messageLengthReceived += 1;
            messageReceived += read;
            printf("received %u", read);
            printf("received %u of %llu\n", messageLengthReceived, messageReceived);
            if (messageLengthReceived == MESSAGE_LENGTH){
                printf("received %u of %llu\n", messageLengthReceived, messageReceived);
                printf("received total of %llu, length was %u of %u\n", messageReceived, messageLengthReceived, MESSAGE_LENGTH);  
                printf("received %u of %llu\n", messageLengthReceived, messageReceived);
                //printf("received total of %llu, length was %d of %d\n", 0xFFFuLL, 8, 8    );  
                messageLengthReceived = 0;
                messageReceived = 0x000ULL;
            }
            messageReceived <<= 8;
        }
    }
}

