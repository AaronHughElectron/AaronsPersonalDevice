#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "esp32-hal-ledc.h"
#include "esp_err.h"
#include "esp_sleep.h"

// Pin definitions
const int batteryMonitor = 2;  // Battery monitor circuit
const int plugDetectPin = 3;  // Plug detection circuit
const int buttonPin = 4;  // Push button input (active LOW with internal pull-up)
const int pwmPin = 5;  // PWM output pin (to drive the MOSFET)
const int ledPin = 6;  // LED DIN pin

// Variables for button debouncing & button state tracking
int buttonState = HIGH;         // current stable state
int lastButtonState = HIGH;     // previous reading
const unsigned long debounceDelay = 50;
const unsigned long longPressTime = 600;
const unsigned long superlongPressTime = 1800;
unsigned long lastDebounceTime = 0;  // last time the input changed
unsigned long buttonPressStart = 0;  // time when button was pressed
bool longPressEventTriggered = false; // ensure long press is triggered only once
int stateAtPress = 0;

// PWM configuration parameters & Motor configurations
const int pwmFrequency  = 50000;   // 5 kHz PWM frequency
const int pwmResolution = 8;      // 8-bit resolution (duty: 0-255)
int motorState = 0;
uint8_t speeds[5] = {0, 220, 235, 245, 255};

// LED configuration
const int numLEDs = 3;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(numLEDs, ledPin, NEO_GRB + NEO_KHZ800);
const uint32_t primaryLEDColor = strip.Color(220, 185, 156);
const uint32_t maxModeLEDColor = strip.Color(116, 22, 204);
const int maxBrightness = 220;
const int maxModeFadeInBrightness = 100;
int currentBrightness = maxModeFadeInBrightness;
int maxModeFadeAmount = 5;

// Battery management settings
const int batteryIndicatorTimer = 650;  // Show battery status for 0.65 seconds on boot
const int showBatIndicatorTimer = 500;  // Show battery status for 0.65 seconds in operation when triggered
const uint32_t fullBatteryLEDColor = strip.Color(0, 255, 0);
const uint32_t midRangeBatteryLEDColor = strip.Color(255, 255, 0);
const uint32_t lowBatteryLEDColor = strip.Color(255, 0, 0);
const int batIndicatorFadeInBrightness = 0;
int currentBatIndicatorBrightness = batIndicatorFadeInBrightness;
int batIndicatorFadeAmount = 1;
int batIndicatorBrightness = 30;
bool isCurrentChargeCycleComplete = false;

// Deep sleep settings
const uint64_t WAKE_UP_LOW_PIN_MASK = (1ULL << buttonPin);
const uint64_t WAKE_UP_HIGH_PIN_MASK = (1ULL << plugDetectPin);
const int dormantTimer = 120000;  // Two minutes of inactivity will put the device into sleep mode
const int deepSleepPrepTimer = 2000;  // Warn for two seconds before going into sleep mode
const int sleepModeBrightness = 100;
const uint32_t sleepModeLEDColor = strip.Color(0, 191, 255);
bool isUSBPlugged = false;


void setup() {
  Serial.begin(115200);
  Serial.flush();

  // Initialize tact button
  pinMode(buttonPin, INPUT_PULLUP); // Enable internal pull-up

  // Initialize voltage monitoring
  pinMode(batteryMonitor, INPUT);
  pinMode(plugDetectPin, INPUT);

  // Initialize the LED strip
  strip.begin();
  unsigned long batteryIndicatorFadeEndTime = millis() + batteryIndicatorTimer;
  int currentBatteryIndicatorBrightness = 0;
  int delta = 5;
  while(millis() < batteryIndicatorFadeEndTime) {
    currentBatteryIndicatorBrightness += delta;
    if (currentBatteryIndicatorBrightness == 0 || currentBatteryIndicatorBrightness == batIndicatorBrightness) {
      delta = -delta;
    }
    strip.setBrightness(currentBatteryIndicatorBrightness);
    setBatteryIndicator();
    strip.show();
    delay(30); // Adjust delay for a smoother fade effect
  }
  
  setAllLEDsColor(0);
  strip.show();
  delay(100);

  // Set up the LEDC PWM channel:
  ledcAttach(pwmPin, pwmFrequency, pwmResolution);
  ledcWrite(pwmPin, 0); // Set initial duty cycle (motor off)
  
  // Log
  // delay(300);
  // Serial.println("Setup complete!");
  // Serial.println("Current vibration strength mode: OFF");
}

void loop() {
  int reading = digitalRead(buttonPin);
  unsigned long currentTime = millis();
  int plugState = digitalRead(plugDetectPin);

  // Disable circuit on plugged in (charging)
  if (plugState == HIGH) {
    if (motorState != 0) {
      motorState = 0;
      ledcWrite(pwmPin, speeds[motorState]);
      updateLEDs();
    }

    // Update lastButtonState to avoid false triggers when unplugged later.
    lastButtonState = digitalRead(buttonPin);

    // Go straight to sleep when USB is unplugged.
    lastDebounceTime = currentTime;  // Avoid entering default deep sleep prep
    isUSBPlugged = true;

    // Display battery level: Stop animating indicator lights when the battery is 99% charged
    float Vbattf = measureBatteryLevel();
    if (Vbattf >= 4.185) { isCurrentChargeCycleComplete = true; }
    if (!isCurrentChargeCycleComplete) {
      currentBatIndicatorBrightness += batIndicatorFadeAmount;
      if (currentBatIndicatorBrightness == batIndicatorFadeInBrightness || currentBatIndicatorBrightness == batIndicatorBrightness) {
        batIndicatorFadeAmount = -batIndicatorFadeAmount;
      }
      strip.setBrightness(currentBatIndicatorBrightness);
      setBatteryIndicator();
      strip.show();
    } else {
      strip.setBrightness(batIndicatorBrightness);
      setBatteryIndicator();
      strip.show();
    }
    delay(45);

    return;

  } else {
    batIndicatorFadeAmount = abs(batIndicatorFadeAmount);
    currentBatIndicatorBrightness = batIndicatorFadeInBrightness;
    isCurrentChargeCycleComplete = false;
  }

  // If the button reading changes, reset the debounce timer
  if (reading != lastButtonState) {
    lastDebounceTime = currentTime;
  }

  // Only consider the reading stable if it has been stable for the debounce period
  if ((currentTime - lastDebounceTime) > debounceDelay) {
    // If the stable reading is different from the last known state, update the state.
    if (reading != buttonState) {
      buttonState = reading;

      if (buttonState == LOW) {
        // Button just pressed: record the time and reset the long press flag.
        buttonPressStart = currentTime;
        longPressEventTriggered = false;
        stateAtPress = motorState;

      } else {
        // Button released: determine press duration.
        unsigned long pressDuration = currentTime - buttonPressStart;
        if (pressDuration < longPressTime) {
          // Short press: cycle through states.
          motorState = (motorState + 1) % sizeof(speeds);
          ledcWrite(pwmPin, speeds[motorState]);
        } else {
          // Long press detected: reset motor state to OFF.
          motorState = 0;
          ledcWrite(pwmPin, speeds[motorState]);
          // At State 0: Show battery power by long pressing or enter deep sleep by (super) long pressing
          if ((stateAtPress == 0) && (pressDuration < superlongPressTime)) {
            unsigned long showBatIndicatorEndTime = millis() + showBatIndicatorTimer;
            int currentBatIndicatorBrightness = 0;
            int delta = 5;
            while(millis() < showBatIndicatorEndTime) {
              currentBatIndicatorBrightness += delta;
              if (currentBatIndicatorBrightness == 0 || currentBatIndicatorBrightness == batIndicatorBrightness) {
                delta = -delta;
              }
              strip.setBrightness(currentBatIndicatorBrightness);
              setBatteryIndicator();
              strip.show();
              delay(30);
            }
          } else if ((stateAtPress == 0) && (pressDuration >= superlongPressTime)) {
            // Fading amber light to let the user know the device is about to enter power saving / "deep sleep" mode
            unsigned long sleepModeFadeEndTime = millis() + deepSleepPrepTimer;
            int currentSleepModeBrightness = 0;
            int delta = 5;
            while(millis() < sleepModeFadeEndTime) {
              currentSleepModeBrightness += delta;
              if (currentSleepModeBrightness == 0 || currentSleepModeBrightness == sleepModeBrightness) {
                delta = -delta;
              }
              strip.setBrightness(currentSleepModeBrightness);
              strip.setPixelColor(1, sleepModeLEDColor);
              strip.show();
              delay(30); // Adjust delay for a smoother fade effect
            }

            // Turn off all LEDs and switch off motor before going into deep sleep
            setAllLEDsColor(0);
            strip.show();
            ledcWrite(pwmPin, 0);
            delay(100);

            // Enable button & USB interrupt to turn the device back on
            esp_deep_sleep_enable_gpio_wakeup(WAKE_UP_LOW_PIN_MASK, ESP_GPIO_WAKEUP_GPIO_LOW);
            esp_deep_sleep_enable_gpio_wakeup(WAKE_UP_HIGH_PIN_MASK, ESP_GPIO_WAKEUP_GPIO_HIGH);
            delay(100);
            esp_deep_sleep_start();
          }
        }
      }
    } else {
      // While the button is continuously pressed, check for a long press.
      if (buttonState == LOW && !longPressEventTriggered && (currentTime - buttonPressStart >= longPressTime)) {
        longPressEventTriggered = true; // ensure it triggers only once
        motorState = 0;
        ledcWrite(pwmPin, speeds[motorState]);
        // At State 0: Show battery power by long pressing or enter deep sleep by (super) long pressing
        if ((stateAtPress == 0) && (currentTime - buttonPressStart < superlongPressTime)) {
          unsigned long showBatIndicatorEndTime = millis() + showBatIndicatorTimer;
          int currentBatIndicatorBrightness = 0;
          int delta = 5;
          while(millis() < showBatIndicatorEndTime) {
            currentBatIndicatorBrightness += delta;
            if (currentBatIndicatorBrightness == 0 || currentBatIndicatorBrightness == batIndicatorBrightness) {
              delta = -delta;
            }
            strip.setBrightness(currentBatIndicatorBrightness);
            setBatteryIndicator();
            strip.show();
            delay(30);
          }
        } else if ((stateAtPress == 0) && (currentTime - buttonPressStart >= superlongPressTime)) {
          // Fading amber light to let the user know the device is about to enter power saving / "deep sleep" mode
          unsigned long sleepModeFadeEndTime = millis() + deepSleepPrepTimer;
          int currentSleepModeBrightness = 0;
          int delta = 5;
          while(millis() < sleepModeFadeEndTime) {
            currentSleepModeBrightness += delta;
            if (currentSleepModeBrightness == 0 || currentSleepModeBrightness == sleepModeBrightness) {
              delta = -delta;
            }
            strip.setBrightness(currentSleepModeBrightness);
            strip.setPixelColor(1, sleepModeLEDColor);
            strip.show();
            delay(30); // Adjust delay for a smoother fade effect
          }

          // Turn off all LEDs and switch off motor before going into deep sleep
          setAllLEDsColor(0);
          strip.show();
          ledcWrite(pwmPin, 0);
          delay(100);

          // Enable button & USB interrupt to turn the device back on
          esp_deep_sleep_enable_gpio_wakeup(WAKE_UP_LOW_PIN_MASK, ESP_GPIO_WAKEUP_GPIO_LOW);
          esp_deep_sleep_enable_gpio_wakeup(WAKE_UP_HIGH_PIN_MASK, ESP_GPIO_WAKEUP_GPIO_HIGH);
          delay(100);
          esp_deep_sleep_start();
        }
      }
    }
  }
  
  lastButtonState = reading;

  // Update LEDs for all modes
  updateLEDs();  // Update LEDs for all modes except MAX mode
  // MAX mode LEDs are special
  if (motorState == 4) {
    currentBrightness += maxModeFadeAmount;
    if (currentBrightness == maxModeFadeInBrightness || currentBrightness == maxBrightness) {
      maxModeFadeAmount = -maxModeFadeAmount;
    }
    strip.setBrightness(currentBrightness);
    setAllLEDsColor(maxModeLEDColor);
    strip.show();
    delay(15);
  }

  // Enter deep sleep if not in use
  if (motorState == 0 && (currentTime - lastDebounceTime >= dormantTimer)) {
    // Fading amber light to let the user know the device is about to enter power saving / "deep sleep" mode
    unsigned long sleepModeFadeEndTime = millis() + deepSleepPrepTimer;
    int currentSleepModeBrightness = 0;
    int delta = 5;
    while(millis() < sleepModeFadeEndTime) {
      currentSleepModeBrightness += delta;
      if (currentSleepModeBrightness == 0 || currentSleepModeBrightness == sleepModeBrightness) {
        delta = -delta;
      }
      strip.setBrightness(currentSleepModeBrightness);
      strip.setPixelColor(1, sleepModeLEDColor);
      strip.show();
      delay(30); // Adjust delay for a smoother fade effect
    }

    // Turn off all LEDs and switch off motor before going into deep sleep
    setAllLEDsColor(0);
    strip.show();
    ledcWrite(pwmPin, 0);
    delay(100);

    // Enable button & USB interrupt to turn the device back on 
    esp_deep_sleep_enable_gpio_wakeup(WAKE_UP_LOW_PIN_MASK, ESP_GPIO_WAKEUP_GPIO_LOW);
    esp_deep_sleep_enable_gpio_wakeup(WAKE_UP_HIGH_PIN_MASK, ESP_GPIO_WAKEUP_GPIO_HIGH);
    delay(100);
    esp_deep_sleep_start();
  }

  // Enter deep sleep after USB is unplugged
  if (isUSBPlugged) {
    // Fading amber light to let the user know the device is about to enter power saving / "deep sleep" mode
    int currentSleepModeBrightness = batIndicatorBrightness;
    int delta = 5;
    while(currentSleepModeBrightness != 0) {
      currentSleepModeBrightness += delta;
      if (currentSleepModeBrightness == 0 || currentSleepModeBrightness == maxBrightness / 2) {
        delta = -delta;
      }
      strip.setBrightness(currentSleepModeBrightness);
      setBatteryIndicator();
      strip.show();
      delay(10); // Adjust delay for a smoother fade effect
    }

    // Turn off all LEDs and switch off motor before going into deep sleep
    setAllLEDsColor(0);
    strip.show();
    ledcWrite(pwmPin, 0);
    delay(100);

    isUSBPlugged = false;

    // Enable button & USB interrupt to turn the device back on
    esp_deep_sleep_enable_gpio_wakeup(WAKE_UP_LOW_PIN_MASK, ESP_GPIO_WAKEUP_GPIO_LOW);
    esp_deep_sleep_enable_gpio_wakeup(WAKE_UP_HIGH_PIN_MASK, ESP_GPIO_WAKEUP_GPIO_HIGH);
    delay(100);
    esp_deep_sleep_start();
  }
}


// Function to update the LED display based on the current state (from OFF up to penultimate state)
void updateLEDs() {
  if (motorState == 4) { return; }
  // For OFF and three modes
  strip.setBrightness(maxBrightness);
  for (int i = 0; i < numLEDs; i++) {
    if (i < motorState) {
      // Turn on the LED at nth state
      strip.setPixelColor(i, primaryLEDColor);
    } else {
      // Turn off the LED for all other states
      strip.setPixelColor(i, 0);
    }
  }
  // Refresh the strip to show changes
  strip.show();
  // Reset parameters for max mode
  currentBrightness = maxModeFadeInBrightness;
  maxModeFadeAmount = abs(maxModeFadeAmount);
}

// Function to set the same color for ALL LEDs
void setAllLEDsColor(uint32_t color) {
  // Set every LED in the strip to the specified color
  for (int i = 0; i < numLEDs; i++) {
    strip.setPixelColor(i, color);
  }
}

// Function to calculate battery level
float measureBatteryLevel() {
  // Battery level calibration
  uint32_t Vbatt = 0;
  for(int i = 0; i < 16; i++) {
    Vbatt = Vbatt + analogReadMilliVolts(A0); // ADC with correction   
  }
  float Vbattf = 2 * Vbatt / 16 / 1000.0;     // attenuation ratio 1/2, mV --> V
  return Vbattf;
}

// Function to calculate current battery level and set battery level indicator
void setBatteryIndicator() {
  float Vbattf = measureBatteryLevel();
  float VbattPercent = (Vbattf - 3) / 1.2;    // Battery percentage relative to 3V, minimum voltage requirement

  // Set indicator colour according to battery level
  if (VbattPercent >= 0.7) {
    for (int i = 0; i < numLEDs; i++) {
      strip.setPixelColor(i, fullBatteryLEDColor);
    }
  } else if (VbattPercent < 0.7 && VbattPercent >= 0.3) {
    for (int i = 0; i < numLEDs - 1; i++) {
      strip.setPixelColor(i, midRangeBatteryLEDColor);
    }
  } else {
    strip.setPixelColor(0, lowBatteryLEDColor);
  }
}