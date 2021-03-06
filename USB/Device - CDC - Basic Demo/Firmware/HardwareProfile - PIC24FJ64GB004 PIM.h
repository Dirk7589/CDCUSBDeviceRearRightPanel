/********************************************************************
 FileName:     	HardwareProfile - PIC24FJ64GB004 PIM.h
 Dependencies:  See INCLUDES section
 Processor:     PIC24FJ64GB004
 Hardware:      PIC24FJ64GB004 PIM
 Compiler:      Microchip C30
 Company:       Microchip Technology, Inc.

 Software License Agreement:

 The software supplied herewith by Microchip Technology Incorporated
 (the �Company�) for its PIC� Microcontroller is intended and
 supplied to you, the Company�s customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.

 THIS SOFTWARE IS PROVIDED IN AN �AS IS� CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

********************************************************************
 File Description:

 Change History:
  Rev   Date         Description
  1.0   11/19/2004   Initial release
  2.1   02/26/2007   Updated for simplicity and to use common
                     coding style
  2.3   09/15/2008   Broke out each hardware platform into its own
                     "HardwareProfile - xxx.h" file
  2.4b  04/08/2009   Initial support for PIC24FJ64GB004 family
********************************************************************/

#ifndef HARDWARE_PROFILE_PIC24FJ64GB004_PIM_H
#define HARDWARE_PROFILE_PIC24FJ64GB004_PIM_H

    /*******************************************************************/
    /******** USB stack hardware selection options *********************/
    /*******************************************************************/
    //This section is the set of definitions required by the MCHPFSUSB
    //  framework.  These definitions tell the firmware what mode it is
    //  running in, and where it can find the results to some information
    //  that the stack needs.
    //These definitions are required by every application developed with
    //  this revision of the MCHPFSUSB framework.  Please review each
    //  option carefully and determine which options are desired/required
    //  for your application.

    //#define USE_SELF_POWER_SENSE_IO
    #define tris_self_power     TRISAbits.TRISA2    // Input
    #define self_power          1

    //#define USE_USB_BUS_SENSE_IO
    #define tris_usb_bus_sense  TRISBbits.TRISB5    // Input
    #define USB_BUS_SENSE       1 
   
    //Uncomment this to make the output HEX of this project 
    //   to be able to be bootloaded using the HID bootloader
    #define PROGRAMMABLE_WITH_USB_HID_BOOTLOADER	

    //If the application is going to be used with the HID bootloader
    //  then this will provide a function for the application to 
    //  enter the bootloader from the application (optional)
    #if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)
        #define EnterBootloader() __asm__("goto 0x400")
    #endif   

    /*******************************************************************/
    /*******************************************************************/
    /*******************************************************************/
    /******** Application specific definitions *************************/
    /*******************************************************************/
    /*******************************************************************/
    /*******************************************************************/

    /** Board definition ***********************************************/
    //These defintions will tell the main() function which board is
    //  currently selected.  This will allow the application to add
    //  the correct configuration bits as wells use the correct
    //  initialization functions for the board.  These defitions are only
    //  required in the stack provided demos.  They are not required in
    //  final application design.
    #define DEMO_BOARD PIC24FJ64GB004_PIM
    #define EXPLORER_16
    #define PIC24FJ64GB004_PIM
    #define CLOCK_FREQ 32000000

    /** I/O pin definitions ********************************************/
    #define INPUT_PIN 1
    #define OUTPUT_PIN 0

    /** Buttons ********************************************************/
    #define LEGT_BUTTON PORTBbits.RB9 /**<Left EGT Button*/
    #define REGT_BUTTON PORTCbits.RC6 /**<Right EGT Button*/
    #define ENG_VIBE_BUTTON PORTCbits.RC7 /**<Engine vibrations test button*/
    #define CKT_TEST_BUTTON PORTCbits.RC8 /**<Circuit Test button*/
    #define LWS_TEST_BUTTON PORTCbits.RC9 /**<LWS system test button*/
    /** LED ************************************************************/
    
    /** SWITCH *********************************************************/
    #define INU_SWITCH PORTBbits.RB2
    #define INU_HEATER_SWITCH PORTBbits.RB3
    #define LWS_POWER_SWITCH PORTCbits.RC0
    #define CM_POWER_SWITCH PORTCbits.RC1
    #define CM_TEST_SWITCH PORTCbits.RC2

    #define REAR_LIGHT_SWITCH PORTCbits.RC3
    #define HYDR_SWITCH PORTCbits.RC4
    #define EKRAN_SWITCH PORTCbits.RC5

    /** Potentiometers *************************************************/
    #define BLUE_KNOB 0
    #define ADI_KNOB 1
    #define LEFT_RIGHT_KNOB 2
    #define REAR_KNOB 3
    /** LEDS ***********************************************************/
    #define LWS_LAMP_LED PORTAbits.RA10

    /** Structure Defines*/
    typedef struct 
    {
        char b1;
        char b2;
        char b3;
        char b4;
        char b5;
    }buttonState;

    typedef struct
    {
        char s1;
        char s2;
        char s3;
        char s4;
        char s5;
        char s6;
        char s7;
        char s8;
    }switchState;
#endif  //HARDWARE_PROFILE_PIC24FJ64GB004_PIM_H
