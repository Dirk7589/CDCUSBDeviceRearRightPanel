/**
*@file rotary.c
*@author Dirk Dubois
*@date April 29, 2013
*/
#include "Compiler.h"
#include "rotary.h"

void initRotaryStruct(struct rotaryHardwareState* state)
{
    state->buttonDebounce = 0;
    state->buttonState = 0;
    state->encoderDirection = 0;
    state->encoderPosition = 0;
    state->encoderPreviousState = 0;
    state->encoderState = 0;

    state->direction = 0;
    state->position = 0;
}

void readRotary(struct rotaryHardwareState* hardware)
{
    hardware->direction = hardware->encoderDirection;
    hardware->position = hardware->encoderPosition;
    
    // Rotary encoder -------------------------------------------------------------------------
    hardware->encoderState = RE_A | RE_B << 1;
    if(hardware->encoderPreviousState != 0xFF) // Check for first time check
    {
        if(hardware->encoderPreviousState == 0b00 && hardware->encoderState == 0b01)
        {
                // Going counter-clockwise
                hardware->encoderDirection = -1;
                hardware->encoderPosition--;
        }

        else if(hardware->encoderPreviousState == 0b01 && hardware->encoderState == 0b00)
        {
                // Going clockwise
                hardware->encoderDirection = 1;
                hardware->encoderPosition++;
        }
    }
    // Save the current state
    hardware->encoderPreviousState = hardware->encoderState;

    // Our encoder has 12 ticks per rotation
    if (hardware->encoderPosition > NUMBER_OF_TICKS)
    {
        hardware->encoderPosition = 0;
    }
    if (hardware->encoderPosition < 0)
    {
        hardware->encoderPosition = NUMBER_OF_TICKS;
    }

    if(hardware->encoderDirection == 1)
    {
            hardware->direction = 1;
            
    }
    if(hardware->encoderDirection == -1)
    {
           hardware->direction = -1;
    }

    if (!SWITCH0)
    {
        hardware->buttonState = 1;
    }
}
int8_t rotaryFunction()
{

}
