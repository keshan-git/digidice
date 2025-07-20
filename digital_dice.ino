#include <avr/sleep.h>

#define DATA_PIN   0  // PB0 (Pin 5)
#define CLOCK_PIN  2  // PB2 (Pin 7)
#define LATCH_PIN  3  // PB3 (Pin 2)

#define SENSOR_PIN 1  // PB1 (Pin 6)
#define BUZZER_PIN 4  // PB4 (Pin 3)

#define NOTE_A4  440

const uint8_t dicePatterns[6] = {
  0b00010000, // Dice 1
  0b10000001, // Dice 2
  0b10010001, // Dice 3
  0b11000101, // Dice 4
  0b11010101, // Dice 5
  0b11101101  // Dice 6
};

enum State {
  IDLE,
  SHUFFLING,
  PRE_SELECTED,
  SELECTED
};

State currentState = IDLE;
uint16_t seed = 7564;

uint8_t dice1Selected = 0;
uint8_t dice2Selected = 0;

uint8_t ticks = 0;
uint8_t durationTicks = 0;

void setup() {
  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(LATCH_PIN, OUTPUT);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(SENSOR_PIN, INPUT_PULLUP);
  
  GIMSK |= (1 << PCIE);    // Enable Pin Change Interrupts
  PCMSK |= (1 << PCINT1);  // Enable PCINT for PB1
  sei();  

  currentState = IDLE;
  dice1Selected = tinyRandom();
  dice2Selected = tinyRandom();
}

ISR(PCINT0_vect) {
    if (currentState == IDLE) {
      beep(NOTE_A4);
      seed = seed + seed * durationTicks + seed * ticks * ticks;
      ticks = 0;
      durationTicks = 0;
      currentState = SHUFFLING;
  }
}

void powerOff() {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Power-down mode
  sleep_enable();
  sleep_cpu();  // Enter sleep
  sleep_disable();
}

void loop() {
  switch (currentState) {
    case IDLE:
      if (ticks < 5) {
        showDice(0, 1);
        delay(100);
        ticks++;
      } else if (ticks < 10) {
        showDice(1, 0);
        delay(100);
        ticks++;
      } else {
        ticks = 0;
        durationTicks++;
      }

      if(durationTicks == 2 * 60) {
        showDice(0, 0);
        powerOff();
      }
      break;

    case SHUFFLING:
      if (ticks < 50) {
        showDice(dice2Selected-- % 6 + 1, dice1Selected++ % 6 + 1);
        ticks++;
        delay(100);
      } else {
        ticks = 0;
        dice1Selected = dice1Selected % 6 + 1;
        dice2Selected = dice2Selected % 6 + 1;
        currentState = PRE_SELECTED;
      }
      break;

    case PRE_SELECTED:
      if (ticks < 5) {
        showDice(dice1Selected, dice2Selected);
        delay(100);
        ticks++;
      } else if (ticks < 10) {
        showDice(0, 0);
        delay(100);
        ticks++;
      } else {
        ticks = 0;
        durationTicks++;
      }

      if (durationTicks >= 5) {
        ticks = 0;
        durationTicks = 0;
        currentState = SELECTED;
      }
      break;

    case SELECTED:
      showDice(dice1Selected, dice2Selected);
      if(ticks == 0) {
        ticks++;
        beep(NOTE_A4);
      }

      if (ticks >= 50) {
        ticks = 0;
        durationTicks = 0;
        currentState = IDLE;
      } else {
        ticks++;
        delay(100);
      }
      break;
  }
}

void showDice(uint8_t dice1, uint8_t dice2) {
  uint8_t grid1 = dice1 != 0 ? dicePatterns[dice1 - 1] : 0;
  uint8_t grid2 = dice2 != 0 ? dicePatterns[dice2 - 1] : 0;

  digitalWrite(LATCH_PIN, LOW);
  shiftOutFast(DATA_PIN, CLOCK_PIN, grid2); // Second shift register
  shiftOutFast(DATA_PIN, CLOCK_PIN, grid1); // First shift register
  digitalWrite(LATCH_PIN, HIGH);
}

void shiftOutFast(uint8_t dataPin, uint8_t clockPin, uint8_t val) {
  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(dataPin, (val & (1 << (7 - i))) ? HIGH : LOW);
    digitalWrite(clockPin, HIGH);
    digitalWrite(clockPin, LOW);
  }
}

void beep(uint16_t frequency) {
  // uint16_t delay_us = 500000UL / frequency; // Half period
  // uint16_t cycles = (100000UL / (delay_us * 2)); // ~100ms worth of cycles

  // for (uint16_t i = 0; i < cycles; i++) {
  //   PORTB |= (1 << PB4);          // Set PB4 HIGH (faster than digitalWrite)
  //   delayMicroseconds(delay_us);  // Wait half period
  //   PORTB &= ~(1 << PB4);         // Set PB4 LOW
  //   delayMicroseconds(delay_us);  // Wait half period
  // }
}

uint8_t tinyRandom() {
  seed = seed * 2053 + 13849; // LCG parameters
  return (seed >> 8) % 6;
}
