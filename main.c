// Including stdio.h to enable use of printf, which outputs over USB 
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "pico/binary_info.h"
#include <string.h>
 
#define UART_ID uart0
#define BAUD_RATE 115200

// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#define UART_TX_PIN 0
#define UART_RX_PIN 1

// Pin constants
const uint LED_PIN = 25;

const uint RUDDER_TOP_PIN = 18;
const uint RUDDER_RIGHT_PIN = 19;
const uint RUDDER_BOTTOM_PIN = 20;
const uint RUDDER_LEFT_PIN = 21;

// Motor driver pins
const uint MOTOR_PWM_PIN = 15;
const uint MOTOR_IN4_PIN = 14;
const uint MOTOR_IN3_PIN = 13;

// PWM constants
const uint PWM_DIVIDER = 125;
const uint PWM_WRAP_TIME = 20000; // 20 ms

// Set motor speed based on speed value between -511 and 511
void setMotorSpeed(int speed, uint slice, uint channel){
    if (speed == 0){
        gpio_put(MOTOR_IN3_PIN, 0);
        gpio_put(MOTOR_IN4_PIN, 0);
    } else {
        if (speed > 0){
            gpio_put(MOTOR_IN3_PIN, 1);
            gpio_put(MOTOR_IN4_PIN, 0);
        } else {
            gpio_put(MOTOR_IN3_PIN, 0);
            gpio_put(MOTOR_IN4_PIN, 1);
        }
        //This PWM slice is configured to have a value between 1 and 1000
        pwm_set_chan_level(slice, channel, abs(speed)*(1000/511));
    }
    
}

int main() {
    bi_decl(bi_program_description("This is a the main binary of the submarine pico."));
    bi_decl(bi_1pin_with_name(LED_PIN, "On-board LED"));
    bi_decl(bi_1pin_with_name(RUDDER_TOP_PIN, "Pin for the rudder at the top position."));
    bi_decl(bi_1pin_with_name(RUDDER_RIGHT_PIN, "Pin for the rudder at the right position."));
    bi_decl(bi_1pin_with_name(RUDDER_BOTTOM_PIN, "Pin for the rudder at the bottom position."));
    bi_decl(bi_1pin_with_name(RUDDER_LEFT_PIN, "Pin for the rudder at the left position."));
    bi_decl(bi_1pin_with_name(MOTOR_PWM_PIN, "Pin connecting to the PWM pin of the motor driver."));
    bi_decl(bi_1pin_with_name(MOTOR_IN4_PIN, "Pin connecting to IN4 on the motor driver."));
    bi_decl(bi_1pin_with_name(MOTOR_IN3_PIN, "Pin connecting to IN3 on the motor driver. "));

    stdio_init_all();

    // Set up our UART with the required speed.
    uart_init(UART_ID, BAUD_RATE);

    // Initialize LED gpio pin
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    
    // Initialize the motor pins
    gpio_init(MOTOR_IN3_PIN);
    gpio_set_dir(MOTOR_IN3_PIN, GPIO_OUT);
    gpio_init(MOTOR_IN4_PIN);
    gpio_set_dir(MOTOR_IN4_PIN, GPIO_OUT);
    gpio_set_function(MOTOR_PWM_PIN, GPIO_FUNC_PWM);
    // Save properties for motor driver PWM pin
    uint sliceMotor = pwm_gpio_to_slice_num(MOTOR_PWM_PIN);
    uint channelMotor = pwm_gpio_to_channel(MOTOR_PWM_PIN);
    // Configure PWM for motor driver
    pwm_set_clkdiv(sliceMotor, 6);
    pwm_set_wrap(sliceMotor, 1000);
    pwm_set_enabled(sliceMotor, true);

    //Initialize the rudder servos PWM pins
    gpio_set_function(RUDDER_TOP_PIN, GPIO_FUNC_PWM);
    gpio_set_function(RUDDER_RIGHT_PIN, GPIO_FUNC_PWM);
    gpio_set_function(RUDDER_BOTTOM_PIN, GPIO_FUNC_PWM);
    gpio_set_function(RUDDER_LEFT_PIN, GPIO_FUNC_PWM);

    //Save slice and channel from rudder servos PWM pins
    uint sliceTop = pwm_gpio_to_slice_num(RUDDER_TOP_PIN);
    uint channelTop = pwm_gpio_to_channel(RUDDER_TOP_PIN);
    uint sliceRight = pwm_gpio_to_slice_num(RUDDER_RIGHT_PIN);
    uint channelRight = pwm_gpio_to_channel(RUDDER_RIGHT_PIN);
    uint sliceBottom = pwm_gpio_to_slice_num(RUDDER_BOTTOM_PIN);
    uint channelBottom = pwm_gpio_to_channel(RUDDER_BOTTOM_PIN);
    uint sliceLeft = pwm_gpio_to_slice_num(RUDDER_LEFT_PIN);
    uint channelLeft = pwm_gpio_to_channel(RUDDER_LEFT_PIN);

    //setting rudder servos PWM properties
    pwm_set_clkdiv(sliceTop,PWM_DIVIDER);
    pwm_set_clkdiv(sliceRight,PWM_DIVIDER);
    pwm_set_clkdiv(sliceBottom,PWM_DIVIDER);
    pwm_set_clkdiv(sliceLeft,PWM_DIVIDER);
    pwm_set_wrap(sliceTop,PWM_WRAP_TIME);
    pwm_set_wrap(sliceRight,PWM_WRAP_TIME);
    pwm_set_wrap(sliceBottom,PWM_WRAP_TIME);
    pwm_set_wrap(sliceLeft,PWM_WRAP_TIME);
    pwm_set_enabled(sliceTop, true);
    pwm_set_enabled(sliceRight, true);
    pwm_set_enabled(sliceBottom, true);
    pwm_set_enabled(sliceLeft, true);
    
    // Set the TX and RX pins of UART communication with Arduino
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

        // Excecute commands given by controller
        // LED is not connected yet
        // Stabilize is not implemented yet because of unconnected gyro
        setMotorSpeed(speed, sliceMotor, channelMotor);
    }
}

