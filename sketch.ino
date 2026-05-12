#include <Arduino.h>
#include "DHT.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ===== OLED SETTINGS =====
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ===== PIN DEFINITIONS =====
#define DHTPIN 4
#define DHTTYPE DHT22

#define BUTTON_PIN 15
#define RELAY_PIN 5
#define BUZZER_PIN 19
#define LED_PIN 16
#define POT_PIN 34

// ===== GLOBAL VARIABLES =====
DHT dht(DHTPIN, DHTTYPE);

volatile bool emergencyStop = false;

float temperature = 0;
float filteredTemp = 0;

int potValue = 0;

float hvacSetpoint = 30.0;

SemaphoreHandle_t mutex;

// ===== DEBOUNCE VARIABLES =====
volatile unsigned long lastInterruptTime = 0;

// ===== BUTTON INTERRUPT HANDLER =====
void IRAM_ATTR buttonISR() {

    unsigned long now = esp_timer_get_time() / 1000;

    if (now - lastInterruptTime > 200) {

        emergencyStop = true;
        lastInterruptTime = now;
    }
}

// ===== TASK HANDLES =====
TaskHandle_t sensorHandle = NULL;
TaskHandle_t controlHandle = NULL;
TaskHandle_t logHandle = NULL;

// ===== TASK 1: SENSOR READ =====
float lastTemp = NAN;

void sensorTask(void *pvParameters) {

    while (1) {

        esp_task_wdt_reset();

        float temp = dht.readTemperature();
        int adc = analogRead(POT_PIN);

        // ===== SENSOR ANOMALY DETECTION =====
        bool anomaly = false;

        if (isnan(temp) || temp < -40 || temp > 100) {
            anomaly = true;
        }
        else if (!isnan(lastTemp) && abs(temp - lastTemp) > 15.0) {
            anomaly = true;
        }

        if (anomaly) {

            emergencyStop = true;

            Serial.println("⚠️ Sensor anomaly detected! Emergency stop triggered.");

        } else {

            lastTemp = temp;

            // ===== LOW PASS FILTER =====
            filteredTemp = 0.8 * filteredTemp + 0.2 * temp;

            float newSetpoint = (adc / 4095.0) * 50.0;

            xSemaphoreTake(mutex, portMAX_DELAY);

            temperature = filteredTemp;
            hvacSetpoint = newSetpoint;
            potValue = adc;

            xSemaphoreGive(mutex);
        }

        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

// ===== TASK 2: CONTROL LOGIC =====
void controlTask(void *pvParameters) {

    static bool lastState = false;

    while (1) {

        esp_task_wdt_reset();

        xSemaphoreTake(mutex, portMAX_DELAY);

        float temp = temperature;
        float setpoint = hvacSetpoint;
        bool estop = emergencyStop;

        xSemaphoreGive(mutex);

        if (estop && !lastState) {
            Serial.println("EMERGENCY STOP ACTIVATED");
        }

        lastState = estop;

        if (estop) {

            // ===== FAIL SAFE =====
            digitalWrite(RELAY_PIN, LOW);

            digitalWrite(BUZZER_PIN, HIGH);
            digitalWrite(LED_PIN, HIGH);

        } else {

            if (temp >= 35.0) {

                digitalWrite(LED_PIN, HIGH);
                digitalWrite(BUZZER_PIN, HIGH);

            } else {

                digitalWrite(LED_PIN, LOW);
                digitalWrite(BUZZER_PIN, LOW);
            }

            // ===== HVAC CONTROL =====
            digitalWrite(RELAY_PIN, temp < setpoint ? HIGH : LOW);
        }

        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

// ===== TASK 3: SERIAL + OLED LOGGING =====
void logTask(void *pvParameters) {

    while (1) {

        esp_task_wdt_reset();

        xSemaphoreTake(mutex, portMAX_DELAY);

        float temp = temperature;
        float setpoint = hvacSetpoint;
        int pot = potValue;
        bool estop = emergencyStop;

        xSemaphoreGive(mutex);

        // ===== SERIAL OUTPUT =====
        Serial.print("Temp: ");
        Serial.print(temp);
        Serial.print(" °C | Setpoint: ");
        Serial.print(setpoint);
        Serial.print(" °C | ADC: ");
        Serial.print(pot);
        Serial.print(" | E-Stop: ");
        Serial.println(estop ? "ON" : "OFF");

        // ===== OLED DISPLAY =====
        display.clearDisplay();

        display.setTextSize(1);
        display.setTextColor(WHITE);

        display.setCursor(0,0);
        display.println("HVAC MONITOR");

        display.setCursor(0,15);
        display.print("Temp: ");
        display.print(temp);
        display.println(" C");

        display.setCursor(0,30);
        display.print("Setpoint: ");
        display.print(setpoint);
        display.println(" C");

        display.setCursor(0,45);

        if(estop) {
            display.println("EMERGENCY STOP");
        } else {
            display.println("System Normal");
        }

        display.display();

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

// ===== SETUP =====
void setup() {

    Serial.begin(115200);

    // ===== OLED INIT =====
    Wire.begin(21, 22);

    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {

        Serial.println("OLED allocation failed");

        for(;;);
    }

    display.clearDisplay();

    display.setTextSize(1);
    display.setTextColor(WHITE);

    display.setCursor(0,0);
    display.println("HVAC System Boot");

    display.display();

    delay(1500);

    // ===== PIN MODES =====
    pinMode(RELAY_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);

    // ===== FAIL SAFE STARTUP =====
    digitalWrite(RELAY_PIN, LOW);

    // ===== BUTTON INPUT =====
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    attachInterrupt(
        digitalPinToInterrupt(BUTTON_PIN),
        buttonISR,
        FALLING
    );

    dht.begin();

    mutex = xSemaphoreCreateMutex();

    filteredTemp = dht.readTemperature();

    // ===== WATCHDOG CONFIG =====
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = 5000,
        .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
        .trigger_panic = true
    };

    esp_task_wdt_init(&wdt_config);

    // ===== CREATE TASKS =====
    xTaskCreate(
        sensorTask,
        "Sensor Task",
        2048,
        NULL,
        2,
        &sensorHandle
    );

    xTaskCreate(
        controlTask,
        "Control Task",
        2048,
        NULL,
        3,
        &controlHandle
    );

    xTaskCreate(
        logTask,
        "Log Task",
        4096,
        NULL,
        1,
        &logHandle
    );

    // ===== ADD TO WATCHDOG =====
    esp_task_wdt_add(sensorHandle);
    esp_task_wdt_add(controlHandle);
    esp_task_wdt_add(logHandle);
}

// ===== MAIN LOOP =====
void loop() {

    // FreeRTOS handles everything
}