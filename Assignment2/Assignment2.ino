#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

// Pin Definitions
#define Task1_PIN           9
#define T4LED_PIN           0
#define BUTTON_PIN          1
#define T7LED_PIN           19
#define POTENTIOMETER_PIN   4
#define Task2_PIN           5
#define Task3_PIN           6

// Frequency Measurement Range
#define TASK2_MIN_FREQ      333
#define TASK2_MAX_FREQ      1000
#define TASK3_MIN_FREQ      500
#define TASK3_MAX_FREQ      1000

// Periods and Rates
#define TASK1_PERIOD        4
#define TASK2_PERIOD        20
#define TASK3_PERIOD        8
#define TASK4_PERIOD        20
#define TASK5_PERIOD        200
#define TASK7_PERIOD        10
#define TASK8_PERIOD        20

// Function Prototypes
void Task1_Signal(void *pvParameters);
void Task2_Measurefrequency(void *pvParameters);
void Task3_Measurefrequency(void *pvParameters);
void Task4_AnalogueInput(void *pvParameters);
void Task5_LogFrequencies(void *pvParameters);
void Task7_ControlLED(void *pvParameters);
void Task8_CPUWork(void *pvParameters);
void handleButtonPress();

// Global structure to store measured frequencies
typedef struct {
    int frequency_T2;
    int frequency_T3;
} FrequencyData;

FrequencyData frequencies;

// Semaphore to protect access to the global structure
SemaphoreHandle_t frequencySemaphore;

// Queue handle for button events
QueueHandle_t buttonQueue;

void setup() {
    Serial.begin(9600);

    pinMode(Task1_PIN, OUTPUT);
    pinMode(T4LED_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(T7LED_PIN, OUTPUT);
    pinMode(POTENTIOMETER_PIN, INPUT);
    pinMode(Task2_PIN, INPUT);
    pinMode(Task3_PIN, INPUT);

    // Create semaphore
    frequencySemaphore = xSemaphoreCreateMutex();

    // Check if semaphore creation failed
    if (frequencySemaphore == NULL) {
        Serial.println("Semaphore creation failed");
    }

    // Create queue
    buttonQueue = xQueueCreate(10, sizeof(int));
    if (buttonQueue == NULL) {
        Serial.println("Queue creation failed");
    }

    
    xTaskCreate(Task1_Signal, "Task1_Signal", 1000, NULL, 2, NULL);
    xTaskCreate(Task2_Measurefrequency, "Task2_Measurefrequency", 1000, NULL, 3, NULL);
    xTaskCreate(Task3_Measurefrequency, "Task3_Measurefrequency", 1000, NULL, 2, NULL);
    xTaskCreate(Task4_AnalogueInput, "Task4_AnalogueInput", 1000, NULL, 2, NULL);
    xTaskCreate(Task5_LogFrequencies, "Task5_LogFrequencies", 2000, NULL, 2, NULL);
    xTaskCreate(Task7_ControlLED, "Task7_ControlLED", 1024, NULL, 1, NULL);
    xTaskCreate(Task8_CPUWork, "Task8_CPUWork", 1024, NULL, 1, NULL);

    // Attach interrupt for button press
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButtonPress, FALLING);
}

void loop() {
    // Empty
}

// Task 1 - Generate digital signal
void Task1_Signal(void *pvParameters) {
    while (1) {
        digitalWrite(Task1_PIN, HIGH);
        delayMicroseconds(180);
        digitalWrite(Task1_PIN, LOW);
        delayMicroseconds(40);
        digitalWrite(Task1_PIN, HIGH);
        delayMicroseconds(530);
        digitalWrite(Task1_PIN, LOW);
        delay(3);
        vTaskDelay(TASK1_PERIOD / portTICK_PERIOD_MS);
    }
}

// Task 2 - Measure frequency of Task2_PIN signal
void Task2_Measurefrequency(void *pvParameters) {
    portTickType lastWakeTime;
    lastWakeTime = xTaskGetTickCount();

    while (1) {
        int frequency = pulseIn(Task2_PIN, HIGH); // Measure frequency

        // Scale frequency to 0-99
        frequency = map(frequency, TASK2_MIN_FREQ, TASK2_MAX_FREQ, 0, 99);

        // Acquire semaphore before accessing shared data
        if (xSemaphoreTake(frequencySemaphore, portMAX_DELAY) == pdTRUE) {
            frequencies.frequency_T2 = frequency;
            xSemaphoreGive(frequencySemaphore); // Release semaphore
        }

        vTaskDelayUntil(&lastWakeTime, TASK2_PERIOD / portTICK_PERIOD_MS);
    }
}

// Task 3 - Measure frequency of Task3_PIN signal
void Task3_Measurefrequency(void *pvParameters) {
    portTickType lastWakeTime;
    lastWakeTime = xTaskGetTickCount();

    while (1) {
        int frequency = pulseIn(Task3_PIN, HIGH); // Measure frequency

        // Scale frequency to 0-99
        frequency = map(frequency, TASK3_MIN_FREQ, TASK3_MAX_FREQ, 0, 99);

        // Acquire semaphore before accessing shared data
        if (xSemaphoreTake(frequencySemaphore, portMAX_DELAY) == pdTRUE) {
            frequencies.frequency_T3 = frequency;
            xSemaphoreGive(frequencySemaphore); // Release semaphore
        }

        vTaskDelayUntil(&lastWakeTime, TASK3_PERIOD / portTICK_PERIOD_MS);
    }
}

// Task 4 - Sample analogue input and update running average
void Task4_AnalogueInput(void *pvParameters) {
    int readValues[10] = {0}; // Array to store recent readings
    int readIndex = 0; // Current index in the readings array
    long sum = 0; // Sum of the readings for calculating the average

    while (1) {
       
        int currentValue = analogRead(POTENTIOMETER_PIN);
        sum = sum - readValues[readIndex] + currentValue;
        readValues[readIndex] = currentValue;
        readIndex = (readIndex + 1) % 10;
        float average = sum / 10.0;

        // Visualize error using LED if average is greater than half of maximum range
        if (average > 2047) {
            digitalWrite(T4LED_PIN, HIGH); 
        } else {
            digitalWrite(T4LED_PIN, LOW); 
        }

        vTaskDelay(TASK4_PERIOD / portTICK_PERIOD_MS);
    }
}

// Task 5 - Log frequencies to serial port
void Task5_LogFrequencies(void *pvParameters) {
    portTickType lastWakeTime;
    lastWakeTime = xTaskGetTickCount();

    while (1) {
        // Acquire semaphore before accessing shared data
        if (xSemaphoreTake(frequencySemaphore, portMAX_DELAY) == pdTRUE) {
            int scaledFrequencyT2 = frequencies.frequency_T2;
            int scaledFrequencyT3 = frequencies.frequency_T3;
            xSemaphoreGive(frequencySemaphore); // Release semaphore

            // Print scaled frequencies to serial port
            Serial.print(scaledFrequencyT2);
            Serial.print(",");
            Serial.println(scaledFrequencyT3);
        }

        vTaskDelayUntil(&lastWakeTime, TASK5_PERIOD / portTICK_PERIOD_MS);
    }
}

// Task 7 - Control LED based on button press
void Task7_ControlLED(void *pvParameters) {
    int lastButtonState = HIGH;
    int currentButtonState;
    int msg = 1; // Message to send to the queue for toggling LED state

    while (1) {
        currentButtonState = digitalRead(BUTTON_PIN);

        if (currentButtonState == LOW && lastButtonState == HIGH) {
            // Send message to the queue to toggle LED state
            xQueueSend(buttonQueue, &msg, portMAX_DELAY);
            
            // Delay to avoid debounce
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }

        lastButtonState = currentButtonState;
        vTaskDelay(TASK7_PERIOD / portTICK_PERIOD_MS);
    }
}

// Task 8 - Simulate CPU work for approximately 'time' milliseconds
void Task8_CPUWork(void *pvParameters) {
    portTickType lastWakeTime;
    lastWakeTime = xTaskGetTickCount();

    while (1) {
        // Capture start time
        unsigned long startTime = micros();

        // Simulate CPU work
        CPU_work(2); // Call CPU_work to busy the CPU for approximately 2ms

        // Capture end time and calculate duration
        unsigned long endTime = micros();
        unsigned long duration = endTime - startTime;

        // Print the duration that CPU_work took
        Serial.print("CPU_work duration: ");
        Serial.print(duration);
        Serial.println(" microseconds");

        // Wait for the next cycle, ensuring the task runs with a period of 20ms
        vTaskDelayUntil(&lastWakeTime, TASK8_PERIOD / portTICK_PERIOD_MS);
    }
}

// Simulated CPU work function
void CPU_work(int time) {
    // Calibrated loop count per millisecond (adjust based on calibration)
    volatile long loopCountPerMs = 33850; // Based on 16MHz processor

    long loops = time * loopCountPerMs;

    for (long i = 0; i < loops; i++) {
        asm("nop"); // Assembly instruction for "no operation"
    }
}

// Button press interrupt handler
void handleButtonPress() {
    int msg = 1;
    xQueueSendFromISR(buttonQueue, &msg, NULL);
}
