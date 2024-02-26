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
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Variables for UART receiving
    unsigned long long messageReceived;
    uint messageLengthReceived;

    // Variables for controlling
    bool ledState;
    bool stabilizeState;
    int speed;
    int camAngle;
    int xJoystick;
    int yJoystick;

    // Let user know loop starts
    gpio_put(LED_PIN, 1);

    while (true){
        // Initialize message
        messageReceived = 0x000uLL;
        messageLengthReceived = 0;
        // Get data
        while (true){
            // Read the UART for instructions
            if (uart_is_readable(UART_ID)){
                // Read UART
                int read = uart_getc(UART_ID);
                // Create the real message, without the stopbit at the end
                unsigned long long readMessage = read >> 1;
                // Set message at correct position. Message is transmitted from end to beginning
                readMessage <<= (messageLengthReceived * 7);
                // Calculate total message
                messageReceived += readMessage;
                //Decomment for debugging
                //printf("received %d, was %d, %u of %llu\n", read, readMessage, messageLengthReceived, messageReceived);
                // Add one to the messageLength
                messageLengthReceived += 1;
                // Check for stopBit
                if (read & 0b00000001){
                    //Decomment for debugging
                    //printf("received total of %llu, length was %u\n", messageReceived, messageLengthReceived); 
                    //End of message, so continue program
                    break;
                }
            }
        }
        //Decode message
        yJoystick = messageReceived & 0b111111111;
        messageReceived >>= 9;
        if (messageReceived & 0b1){
            yJoystick *= -1;
        }
        messageReceived >>= 1;
        xJoystick = messageReceived & 0b111111111;
        messageReceived >>= 9;
        if (messageReceived & 0b1){
            xJoystick *= -1;
        }
        messageReceived >>= 1;
        camAngle = messageReceived & 0b1111111111;
        messageReceived >>= 10;
        speed = messageReceived & 0b111111111;
        messageReceived >>= 9;
        if (messageReceived & 0b1){
            speed *= -1;
        }
        messageReceived >>= 1;
        if (messageReceived & 0b1){
            stabilizeState = true;
        } else{
            stabilizeState = false;
        }
        messageReceived >>= 1;
        if (messageReceived & 0b1){
            ledState = true;
        } else{
            ledState = false;
        }
    }
}

