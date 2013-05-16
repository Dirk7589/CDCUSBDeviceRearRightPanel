/**
*@file rotary.h
*@author Dirk Dubois
*@date July 18, 2012
*/

#ifndef _ROTARY_H
#define _ROTARY_H

/**Includes*/
#include <stdint.h>
//Internal defines

// Rotary encoder
#define RE_A PORTBbits.RB7
#define RE_B PORTBbits.RB8
#define NUMBER_OF_TICKS 12 /**<The number of ticks for a complete revolution of the rotary encoder*/

// Switch
#define SWITCH0	PORTBbits.RB9 /**<Define the push button of the rotary encoder*/

// Overall hardware state
struct rotaryHardwareState {
	
    // Rotary Encoder position
    int encoderPosition; /**< The postion of the rotary encoder*/
    unsigned char encoderState;
    unsigned char encoderPreviousState;
    int encoderDirection;

    // Push button
    unsigned char buttonState;
    unsigned char buttonDebounce;

    //State
    int8_t direction;
    int8_t position;

};
void initRotaryStruct(struct rotaryHardwareState* state);
/**
 * @brief A function that reads the current state of the rotary encoder.
 * @param state The structure containing the current and previous state of the encoder.
 */
void readRotary(struct rotaryHardwareState* state);

/*A function that acts on information stored in the hardware struct. Sets a bit high when you turn
left or right. Also allows for a button push. 1 is considered active.*/
int8_t rotaryFunction();

#endif