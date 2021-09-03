/*
 *  PIBHardware.h
 *  Author:  Alex St. Clair
 *  Created: July 2019
 *  Updated for MonDo Board: November 2020
 *
 *  Pin and port definitions for the PIB
 */

#ifndef PIBHARDWARE_H
#define PIBHARDWARE_H

// Serial Ports
#define DEBUG_SERIAL    Serial
#define ZEPHYR_SERIAL   Serial1
#define MCB_SERIAL      Serial2
#define PU_SERIAL       Serial3

// Digital Pins
#define PU_PWR_ENABLE   2
#define FORCEON_232     41 //Unused on MonDo
#define FORCEOFF_232    42 //Unused on MonDo
#define SAFE_PIN        31
//#define MCB_IO_1        32
#define PULSE_LED       36

// Analog Pins
#define VMON_3V3        A16
#define VMON_12V        A14
#define VMON_15V        A2
#define IMON_PU         A8 //Profiler Current Monitor using 0.2 Ohm R on negative 
#define IMON_PU_BTS     A9 //Profiler Current Monitor using BTS Current feedback

// LoRa Module
#define SS_PIN          24
#define RESET_PIN       27
#define INTERUPT_PIN    25


#endif /* PIBHARDWARE_H */
