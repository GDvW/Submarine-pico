// Including stdio.h to enable use of printf, which outputs over USB 
#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "pico/binary_info.h"
#include <string.h>
#include <math.h>

 
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

const uint BATTERY_VOLTAGE_PIN = 26;


// Motor driver pins
const uint MOTOR_PWM_PIN = 15;
const uint MOTOR_IN4_PIN = 14;
const uint MOTOR_IN3_PIN = 13;

// PWM constants
const uint PWM_DIVIDER = 125;
const uint PWM_WRAP_TIME = 20000; // 20 ms

bool motorWasRunning = false;

// Set motor speed based on speed value between -511 and 511
void setMotorSpeed(int speed, uint slice, uint channel){
    if (speed == 0){
        gpio_put(MOTOR_IN3_PIN, 0);
        gpio_put(MOTOR_IN4_PIN, 0);
        motorWasRunning = false;
    } else {
        // Motor driver works like xor
        // Table:
        //  IN3     | IN4   | Motor direction
        // ----------------------------------
        //  0       | 0     | Off
        //  1       | 0     | Forward
        //  0       | 1     | Backward
        //  1       | 1     | Off
        if (speed > 0){
            gpio_put(MOTOR_IN3_PIN, 1);
            gpio_put(MOTOR_IN4_PIN, 0);
        } else {
            gpio_put(MOTOR_IN3_PIN, 0);
            gpio_put(MOTOR_IN4_PIN, 1);
        }
        //This PWM slice is configured to have a value between 1 and 1000
        //Send a short pulse to wake controller
        /*if (! motorWasRunning){
            pwm_set_chan_level(slice, channel, 1000);
            sleep_us(1);
            motorWasRunning = true;
        }*/
        pwm_set_chan_level(slice, channel, abs(speed)*(1000/511));
    }
}
// Turn servo based on value between -511 and 511
void turnServo(int value, uint slice, uint channel){
    // PWM slice is configured to have an value between 500 and 2500
    // 1500 is middle, with 90 degree to turn on both sides
    // We want max turn to be 25 degree, so approx between values 1222 and 1778
    // so max deviation from 1500 is 278
    int level = (int)round(1500.0+(value*(278.0/511.0)));
    pwm_set_chan_level(slice, channel, level);
}

// Write data to serial1, i is the length of the data to send
// appendStopByte is if the stopByte, indicating the end of the transmission, should be send at the end
void writeData(long long data, int i, bool appendStopByte){
  uint8_t *pointer = (uint8_t *)&data;
  int mask;
  unsigned int toSend = 0x00;
  int stopBit = 0b1;
  bool finalByte = false;
  while (true){
    // Create mask
    // Mask is variable, because we have to send 64 bits one time
    // so at the back we pad it with zero's to get length of 70
    if (i - 7 < 0){
        mask = (1 << i) - 1;
        // Get the last 7 bits
        toSend = data & mask;
        // Shift data according to how many bits are needed
        data >>= i;
        i = 0;
    } else {
        mask = 0b1111111;
        // Get the last 7 bits
        toSend = data & mask;
        // Shift 7 bits
        data >>= 7;
        i -= 7;
    }
    // Shift toSend 1 position to create space for stopbit
    toSend <<= 1;
    // Append stop bit if needed
    if (i <= 0){
      if (appendStopByte) {
        toSend = toSend | stopBit;
      }
      finalByte = true;
    }
    uart_putc_raw(UART_ID, toSend);
    printf("Send: %u of %d\n", toSend, i);
    if (finalByte){
      break;
    }
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
    bi_decl(bi_1pin_with_name(MOTOR_IN3_PIN, "Pin connecting to IN3 on the motor driver."));
    bi_decl(bi_1pin_with_name(BATTERY_VOLTAGE_PIN, "Pin connecting to voltage divider for battery voltage measurements."));

    // Initialize printf
    stdio_init_all();

    // Initialize ADC for battery measurements
    adc_init();
    adc_gpio_init(BATTERY_VOLTAGE_PIN);


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
    pwm_set_clkdiv(sliceMotor, 120);
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
    
    // Set servo's at standard position at value 1500
    pwm_set_chan_level(sliceBottom, channelBottom, 1500);
    pwm_set_chan_level(sliceTop,channelTop,1500);
    pwm_set_chan_level(sliceLeft,channelLeft,1500);
    pwm_set_chan_level(sliceRight,channelRight,1500);

    // Set the TX and RX pins of UART communication with Arduino
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Variables for UART receiving
    unsigned long long messageReceived;
    uint messageLengthReceived;

    // Variables for UART sending
    // Using two variables because data is too large for one variable
    unsigned long long messageToSend1;
    unsigned int messageToSend2;

    // Variables for controlling
    bool ledState;
    bool stabilizeState;
    int speed;
    int camAngle;
    int xJoystick;
    int yJoystick;

    // Variables for measuring
    bool waterPresent;
    unsigned short int batteryVoltage;
    unsigned int depth;
    unsigned short int gyroX;
    unsigned short int gyroY;
    unsigned short int gyroZ;

    // Let user know loop starts
    gpio_put(LED_PIN, 1);

    while (true){
        // Initialize message
        messageReceived = 0x000uLL;
        messageLengthReceived = 0;
        printf("get data\n");
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
                // Add one to the messageLength
                messageLengthReceived += 1;
                // Check for stopBit
                if (read & 0b00000001){
                    //End of message, so continue program
                    break;
                }
            }
        }
        printf("decoding...\n");
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
        printf("executing y: %d; x: %d; cam: %d; speed: %d; stab: %d; led: %d\n", yJoystick, xJoystick, camAngle, speed, stabilizeState, ledState);
        gpio_put(LED_PIN, 0);
        // Excecute commands given by controller
        // LED is not connected yet
        // Stabilize is not implemented yet because of unconnected gyro
        // CamAngle servo is not yet connected
        setMotorSpeed(speed, sliceMotor, channelMotor);
        turnServo(xJoystick, sliceBottom, channelBottom);
        turnServo(xJoystick, sliceTop, channelTop);
        turnServo(yJoystick, sliceLeft, channelLeft);
        turnServo(yJoystick, sliceRight, channelRight);
        printf("getting data to send\n");

        // Initialize messageToSend variables
        messageToSend1 = 0x000uLL;
        messageToSend2 = 0x00u;

        // Measure things needed for feedback message to controller
        // Variables are fixed now, but an appropriate method should be used to fill them
        gyroX = 65534;
        gyroY = 650;
        gyroZ = 0;
        depth = 16777214;
        printf("starting ADC\n");
        //adc_select_input(BATTERY_VOLTAGE_PIN - 26);
        //batteryVoltage = adc_read();
        batteryVoltage = 0;
        printf("end ADC\n");
        waterPresent = true;
        printf("Assembling message to send \n");

        // Assemble message
        messageToSend1 += gyroX;
        messageToSend1 <<= 16;
        messageToSend1 += gyroY;
        messageToSend1 <<= 16;
        messageToSend1 += gyroZ;
        messageToSend1 <<= 16;
        //Only add the first 16 bits of the depth, because otherwise we will have a too large long long
        messageToSend1 += (depth >> 8);

        messageToSend2 += (depth & 0b11111111);
        messageToSend2 <<= 12;
        messageToSend2 += batteryVoltage;
        messageToSend2 <<= 1;
        messageToSend2 += waterPresent;
        printf("Sending message, gx: %hu; gy: %hu; gz: %hu; depth: %u, bV: %hu; wp: %d\n", gyroX, gyroY, gyroZ, depth, batteryVoltage, waterPresent);

        writeData(messageToSend1, 64, false);
        writeData(messageToSend2, 21, true);
        printf("end send \n0");
        gpio_put(LED_PIN, 1);
    }
}

