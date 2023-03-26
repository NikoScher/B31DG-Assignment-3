/*
*   freeRTOS.h
*   
*   Header file for assignment 3 for ESP32C3 Dev Module
*   Nikolaus Scherwitzel (H00298068)
*   Created: 22/03/23
*/

#ifndef __FREERTOS_H__
#define __FREERTOS_H__

// Pin numbers
#define T1_PIN        1   // Task 1 output signal pin
#define T2_PIN        2   // Task 2 input measure signal pin
#define T3_PIN        3   // Task 3 input measure signal pin
#define T4_ANIN_PIN   4   // Task 4 analogue input pin
#define T4_LED_PIN    0   // Task 4 LED output pin
#define T6_PIN        0   // Task 6 push button input pin
#define T7_PIN        0   // Task 7 LED output pin

// Task periods (ms)
#define TASK1_P       4   // blah
#define TASK2_P       20    // blah
#define TASK3_P       8   // blah
#define TASK4_P       20    // blah
#define TASK5_P       100    // blah
#define TASK6_P       100   // blah
#define TASK7_P       100   // blah

// Task parameters
#define BAUD_RATE       9600  // Baud rate

#define TASK2_TIMEOUT   3100  // Timeout for pulseIn = worst case in us
#define TASK2_MINFREQ   333   // Task 2 minimum frequency of waveform in Hz
#define TASK2_MAXFREQ   1000  // Task 2 maximum frequency of waveform in Hz

#define TASK3_TIMEOUT   2100  // Timeout for pulseIn = worst case in us
#define TASK3_MINFREQ   500   // Task 3 minimum frequency of waveform in Hz
#define TASK3_MAXFREQ   1000  // Task 3 maximum frequency of waveform in Hz

#define NUM_PARAMS      4     // Task 4 length of array for storing past measurements
#define TASK4_THRESH    2048  // Task 4 threshold for turning LED on

#define TASK5_MIN       0     // Task 5 lower bound of range
#define TASK5_MAX       99    // Task 5 upper bound of range

// Helpful typedefs
typedef unsigned char uint8;
typedef unsigned int uint16;
typedef unsigned long uint32;

// Data structure for holding task 2 & 3 frequencies
typedef struct Freqs {
    double freq_t2;
    double freq_t3;
} Freqs;

// Funtion Prototypes
void task1(void *pvParameters);
void task2(void *pvParameters);
void task3(void *pvParameters);
void task4(void *pvParameters);
void task5(void *pvParameters);
void task6(void *pvParameters);
void task7(void *pvParameters);

#endif  // __FREERTOS_H__
