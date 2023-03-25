
/*
*   freeRTOS.ino
*   
*   Sketch for assignment 3 for ESP32C3 Dev Module
*   Nikolaus Scherwitzel (H00298068)
*   Created: 25/03/23
*/

// Libraries
#include "freeRTOS.h"

// Convert period in us to frequency in Hz
#define periodToFreq_us(T) (1 / (T / 1000000))

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

Freqs freqs;              // Data structure for storing task 2 & 3 frequencies

uint16 anIn[NUM_PARAMS];  // Array for storing previous analogue measurements
uint8 currInd;            // Current index to overwrite in anIn[]

void setup() {
  Serial.begin(BAUD_RATE);

  // Define pin inputs/outputs
  pinMode(T1_PIN, OUTPUT);
  pinMode(T2_PIN, INPUT);
  pinMode(T3_PIN, INPUT);
  pinMode(T4_ANIN_PIN, INPUT);
  pinMode(T4_LED_PIN, OUTPUT);
  pinMode(T6_PIN, INPUT);
  pinMode(T7_PIN, OUTPUT);

  // Initialise task 4 analogue input array with 0's
  currInd = 0;
  for (uint8 i = 0; i < NUM_PARAMS; i++) {
    anIn[i] = 0;
  }

  // Initialise task 2 & 3 frequencies to 0
  freqs.freq_t2 = 0;
  freqs.freq_t3 = 0;

}

// Period = 4ms / Rate = 250Hz
void task1(void *pvParameters) {
  (void) pvParameters;

  // Generate waveform
  digitalWrite(T1_PIN, HIGH);
  delayMicroseconds(200);
  digitalWrite(T1_PIN, LOW);
  delayMicroseconds(50);
  digitalWrite(T1_PIN, HIGH);
  delayMicroseconds(30);
  digitalWrite(T1_PIN, LOW);
}

// Period = 20ms / Rate = 50Hz
void task2(void *pvParameters) {
  (void) pvParameters;

  // Measure period when signal is HIGH
  // Multiply by 2 because 50% duty cycle
  double period = (double) pulseIn(T2_PIN, HIGH) * 2;
  freqT2 = periodToFreq_us(period); // Convert to frequency using T = 1/f
}

// Period = 8ms / Rate = 125Hz
void task3(void *pvParameters) {
  (void) pvParameters;

  // Measure period when signal is HIGH
  // Multiply by 2 because 50% duty cycle
  double period = (double) pulseIn(T3_PIN, HIGH) * 2;
  freqT3 = periodToFreq_us(period); // Convert to frequency using T = 1/f
}

// Period = 20ms / Rate = 50Hz
void task4(void *pvParameters) {
  (void) pvParameters;

  // Read analogue signal and increment index for next reading
  // Analogue signal converted to 12 bit integer
  anIn[currInd] = analogRead(T4_ANIN_PIN);
  currInd = (currInd + 1) % NUM_PARAMS;
  
  // Sum array and divide to get average
  double filtAnIn = 0;
  for (uint8 i = 0; i < NUM_PARAMS; i++) {
    filtAnIn += anIn[i];
  }
  filtAnIn /= NUM_PARAMS;

  // Turn on LED if average is above half maximium of 12 bit integer (4096 / 2 = THRESH)
  digitalWrite(T4_LED_PIN, (filtAnIn > TASK4_THRESH));
}

// Period = 100ms / Rate = 10Hz
void task5(void *pvParameters) {
  (void) pvParameters;
  
  // Map frequencies from 333Hz & 500Hz - 1000Hz to 0 - 99
  // Constrain to 0 - 99 as map() only creates gradient
  int normFreqT2 = map(freqT2, TASK2_MINFREQ, TASK2_MAXFREQ, TASK5_MIN, TASK5_MAX);
  normFreqT2 = constrain(normFreqT2, TASK5_MIN, TASK5_MAX);
  int normFreqT3 = map(freqT3, TASK3_MINFREQ, TASK3_MAXFREQ, TASK5_MIN, TASK5_MAX);
  normFreqT3 = constrain(normFreqT3, TASK5_MIN, TASK5_MAX);

  // Print to serial monitor
  Serial.print(normFreqT2);
  Serial.print(",");
  Serial.println(normFreqT3);
}

void loop() {}
