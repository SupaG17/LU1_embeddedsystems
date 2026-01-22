#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <LiquidCrystal_I2C.h>
#include <Arduino.h>

/* FreeRTOS includes */
#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

/* Hardware initialization */
LiquidCrystal_I2C lcd(0x27, 16, 2);  
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);

/* Shared data structure */
struct ColorData {
  uint16_t r;
  uint16_t g;
  uint16_t b;
  uint16_t c;
  uint16_t colorTemp;
  uint16_t lux;
  char detectedColor[16];
  float confidence;
};

/* Color sample buffer for averaging */
#define SAMPLE_SIZE 5
struct ColorSample {
  float r;
  float g;
  float b;
};

ColorSample sampleBuffer[SAMPLE_SIZE];
int sampleIndex = 0;
bool bufferFilled = false;

/* Global variables with mutex protection */
ColorData currentColorData;
SemaphoreHandle_t colorDataMutex;

/* Task handles */
TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t displayTaskHandle = NULL;

/* Advanced color detection using HSV and normalized RGB */
void determineColorAdvanced(uint16_t rawR, uint16_t rawG, uint16_t rawB, uint16_t rawC, char* colorName, float* confidence) {
  // Avoid division by zero
  if (rawC == 0) {
    strcpy(colorName, "No Object");
    *confidence = 0.0;
    return;
  }
  
  // Normalize RGB values (0.0 to 1.0)
  // The (float) is a way of casting the int to a float.
  float r = (float)rawR / rawC;
  float g = (float)rawG / rawC;
  float b = (float)rawB / rawC;
  
  // Calculate total intensity
  float intensity = (r + g + b) / 3.0;
  
  // Calculate HSV values
  float maxVal = max(max(r, g), b);
  float minVal = min(min(r, g), b);
  float delta = maxVal - minVal;
  
  float hue = 0.0;
  float saturation = 0.0;
  float value = maxVal;
  
  // Calculate saturation
  if (maxVal > 0.001) {
    saturation = delta / maxVal;
  }
  
  // Calculate hue
  // fmod used is floating point modulo function
  if (delta > 0.001) {
    if (maxVal == r) {
      hue = 60.0 * fmod(((g - b) / delta), 6.0);
    } else if (maxVal == g) {
      hue = 60.0 * (((b - r) / delta) + 2.0);
    } else {
      hue = 60.0 * (((r - g) / delta) + 4.0);
    }
  }
  
  if (hue < 0) hue += 360.0;
  
  // Check for white/gray/black based on saturation and value
  /*if (saturation < 0.18) {  
    if (value > 0.60) {
      strcpy(colorName, "White");
      *confidence = (1.0 - saturation) * value;
    } else if (value > 0.20) {
      strcpy(colorName, "Gray");
      *confidence = (1.0 - saturation) * (1.0 - abs(value - 0.45));
    } else {
      strcpy(colorName, "Black");
      *confidence = (1.0 - saturation) * (1.0 - value);
    }
    return;
  }*/
  
  // Detect chromatic colors using improved hue ranges and saturation
  *confidence = (saturation * 1.2) * value;
  
  // More precise hue-based color detection with adjusted green range
  if (hue >= 345 || hue < 15) {
    strcpy(colorName, "Red");
  } else if (hue >= 15 && hue < 45) {
    strcpy(colorName, "Orange");
  } else if (hue >= 45 && hue < 80) {
    strcpy(colorName, "Yellow");
  } else if (hue >= 80 && hue < 170) {
    strcpy(colorName, "Green");
  } else if (hue >= 170 && hue < 240) {
    strcpy(colorName, "Blue");
  }/* else if (hue >= 240 && hue < 290) {
    strcpy(colorName, "Purple");
  }*/ else {
    strcpy(colorName, "Unknown");
    *confidence = 0.0;
  }
  
  // Boost confidence for well-saturated colors
  if (saturation > 0.5 && value > 0.3) {
    *confidence = min(1.0, *confidence * 1.2);
  }
}

/* Add sample to moving average buffer */
void addSample(float r, float g, float b) {
  sampleBuffer[sampleIndex].r = r;
  sampleBuffer[sampleIndex].g = g;
  sampleBuffer[sampleIndex].b = b;
  
  sampleIndex = (sampleIndex + 1) % SAMPLE_SIZE;
  
  if (sampleIndex == 0) {
    bufferFilled = true;
  }
}

/* Get averaged values from buffer */
void getAveragedValues(float* avgR, float* avgG, float* avgB) {
  *avgR = 0;
  *avgG = 0;
  *avgB = 0;
  
  int count = bufferFilled ? SAMPLE_SIZE : sampleIndex;
  
  if (count == 0) {
    return;
  }
  
  for (int i = 0; i < count; i++) {
    *avgR += sampleBuffer[i].r;
    *avgG += sampleBuffer[i].g;
    *avgB += sampleBuffer[i].b;
  }
  
  *avgR /= count;
  *avgG /= count;
  *avgB /= count;
}

/* Task 1: Read sensor data */
void sensorReadTask(void *pvParameters) {
  (void) pvParameters;
  
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(100); // Read every 100ms
  
  while (1) {
    ColorData tempData;
    
    // Read raw data from sensor
    tcs.getRawData(&tempData.r, &tempData.g, &tempData.b, &tempData.c);
    tempData.colorTemp = tcs.calculateColorTemperature(tempData.r, tempData.g, tempData.b);
    tempData.lux = tcs.calculateLux(tempData.r, tempData.g, tempData.b);
    
    // Add to moving average buffer
    addSample(tempData.r, tempData.g, tempData.b);
    
    // Get averaged values
    float avgR, avgG, avgB;
    getAveragedValues(&avgR, &avgG, &avgB);
    
    // Use averaged clear value
    float avgC = (avgR + avgG + avgB);
    
    // Determine color
    determineColorAdvanced((uint16_t)avgR, (uint16_t)avgG, (uint16_t)avgB, 
                          (uint16_t)avgC, tempData.detectedColor, &tempData.confidence);
    
    // Map RGB values to 0-255 range for display
    uint8_t redColour = map((uint16_t)avgR, 0, 2500, 0, 255);
    uint8_t greenColour = map((uint16_t)avgG, 0, 2500, 0, 255);
    uint8_t blueColour = map((uint16_t)avgB, 0, 2500, 0, 255);
    
    // Print to Serial with detailed HSV info
    Serial.print("Color Temp: "); Serial.print(tempData.colorTemp, DEC); Serial.print(" K - ");
    Serial.print("Lux: "); Serial.print(tempData.lux, DEC); Serial.print(" - ");
    Serial.print("R: "); Serial.print(redColour, DEC); Serial.print(" ");
    Serial.print("G: "); Serial.print(greenColour, DEC); Serial.print(" ");
    Serial.print("B: "); Serial.print(blueColour, DEC); Serial.println(" ");
    
        // Calculate and display HSV for debugging
    float normR = avgR / avgC;
    float normG = avgG / avgC;
    float normB = avgB / avgC;
    float maxVal = max(max(normR, normG), normB);
    float minVal = min(min(normR, normG), normB);
    float delta = maxVal - minVal;
    float hue = 0.0;
    float saturation = (maxVal > 0.001) ? (delta / maxVal) : 0.0;
    
    if (delta > 0.001) {
      if (maxVal == normR) {
        hue = 60.0 * fmod(((normG - normB) / delta), 6.0);
      } else if (maxVal == normG) {
        hue = 60.0 * (((normB - normR) / delta) + 2.0);
      } else {
        hue = 60.0 * (((normR - normG) / delta) + 4.0);
      }
    }
    if (hue < 0) hue += 360.0;
    
    Serial.print("| H:"); Serial.print(hue, 1); 
    Serial.print(" S:"); Serial.print(saturation, 2); 
    Serial.print(" V:"); Serial.print(maxVal, 2);
    Serial.print(" | Detected: "); Serial.print(tempData.detectedColor);
    Serial.print(" (Conf: "); Serial.print(tempData.confidence * 100, 1); Serial.println("%)");
    
    // Update shared data with mutex protection
    if (xSemaphoreTake(colorDataMutex, portMAX_DELAY) == pdTRUE) {
      currentColorData = tempData;
      xSemaphoreGive(colorDataMutex);
    }
    
    // Wait for next cycle
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

/* Task 2: Update LCD display */
void displayUpdateTask(void *pvParameters) {
  (void) pvParameters;
  
  char lastDisplayedColor[16] = "";
  float lastConfidence = 0.0;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(250); // Update display every 250ms
  
  while (1) {
    ColorData localData;
    
    // Get current data with mutex protection
    if (xSemaphoreTake(colorDataMutex, portMAX_DELAY) == pdTRUE) {
      localData = currentColorData;
      xSemaphoreGive(colorDataMutex);
    }
    
    // Update LCD if color changed OR if confidence crossed the threshold
    bool colorChanged = strcmp(localData.detectedColor, lastDisplayedColor) != 0;
    bool confidenceChanged = (lastConfidence < 0.25 && localData.confidence >= 0.25) || 
                            (lastConfidence >= 0.25 && localData.confidence < 0.25);
    
    if (colorChanged || confidenceChanged) {
      lcd.clear();
      lcd.setCursor(0, 0);
      
      // Only display if confidence is above threshold
      if (localData.confidence >= 0.25) {
        lcd.print(localData.detectedColor);
        lcd.setCursor(0, 1);
        
        // Show confidence bar
        lcd.print("Conf:");
        int barLength = (int)(localData.confidence * 10);
        for (int i = 0; i < 10; i++) {
          if (i < barLength) {
            lcd.print((char)0xFF); // Full block character
          } else {
            lcd.print("-");
          }
        }
      } else {
        lcd.print("....Looking....");
        lcd.setCursor(0, 1);
        lcd.print("...for colour...");
      }
      
      strcpy(lastDisplayedColor, localData.detectedColor);
      lastConfidence = localData.confidence;
    }
    
    // Wait for next cycle
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void setup(void) {
  Serial.begin(9600);
  
  // Initialize LCD
  lcd.init();   
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");
  
  // Initialize sensor
  if (tcs.begin()) {
    Serial.println("Found sensor");
    lcd.setCursor(0, 1);
    lcd.print("Sensor OK");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Sensor Error!");
    while (1);
  }
  
  delay(2000);
  lcd.clear();
  
  // Create mutex
  colorDataMutex = xSemaphoreCreateMutex();
  
  if (colorDataMutex == NULL) {
    Serial.println("Failed to create mutex!");
    while (1);
  }
  
  // Initialize shared data
  strcpy(currentColorData.detectedColor, "Looking...");
  currentColorData.confidence = 0.0;
  
  // Create tasks
  xTaskCreatePinnedToCore(
    sensorReadTask,           /* Task function */
    "SensorRead",             /* Name */
    4096,                     /* Stack size */
    NULL,                     /* Parameters */
    2,                        /* Priority (higher) */
    &sensorTaskHandle,        /* Task handle */
    ARDUINO_RUNNING_CORE      /* Core */
  );
  
  xTaskCreatePinnedToCore(
    displayUpdateTask,        /* Task function */
    "DisplayUpdate",          /* Name */
    2048,                     /* Stack size */
    NULL,                     /* Parameters */
    1,                        /* Priority (lower) */
    &displayTaskHandle,       /* Task handle */
    ARDUINO_RUNNING_CORE      /* Core */
  );
  
  Serial.println("FreeRTOS tasks created successfully!");
  Serial.println("Advanced color detection enabled with:");
  Serial.println("- HSV color space analysis");
  Serial.println("- Moving average filtering");
  Serial.println("- Normalized RGB values");
  Serial.println("- Confidence scoring");
}

void loop(void) {
  // Empty - FreeRTOS tasks handle everything
  vTaskDelay(portMAX_DELAY);
}
