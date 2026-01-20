// Copyright 2024 Espressif Systems (Shanghai) PTE LTD
// Лицензия: Apache License, Version 2.0

/**
 * @brief Двухканальный Zigbee-выключатель + BME280 на ESP32-C6 (режим роутера)
 *
 * Особенности:
 * - 1 светодиод: горит при подключении к сети, дышит при потере сети
 * - Поддержка Zigbee OTA (через zbLight1)
 * - 2 кнопки: короткое — переключение реле, длинное (>10 сек) — разное действие
 *   Кнопка 1: короткое — реле 1, длинное — перезагрузка ESP
 *   Кнопка 2: короткое — реле 2, длинное — сброс Zigbee сети
 * - BME280 → T/H/P в Home Assistant (2 endpoint'а: T+H и P)
 * - Безопасный вывод в Serial через очередь FreeRTOS
 */

#ifndef ZIGBEE_MODE_ZCZR
#error "В меню Tools->Zigbee mode должен быть выбран режим координатора/роутера"
#endif

#include "Zigbee.h"
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <stdarg.h>

// === Пины ===
#define BUTTON1_PIN   19
#define BUTTON2_PIN   20
#define RELAY1_PIN    18
#define RELAY2_PIN    14
#define LED_PIN       15  // Обычный GPIO с PWM

// I2C
#define I2C_SDA_PIN   1
#define I2C_SCL_PIN   2

// === PWM для светодиода ===
#define LED_PWM_FREQ        5000
#define LED_PWM_RESOLUTION  10      // 0..1023
#define BREATH_DELAY_MS     25
#define BREATH_STEP         7

// === Endpoints ===
#define LIGHT_EP1               10
#define LIGHT_EP2               11
#define TEMP_HUMIDITY_SENSOR_EP 12
#define PRESSURE_SENSOR_EP      13

// === OTA версии ===
#define OTA_RUNNING_VERSION     0x01010100
#define OTA_DOWNLOADED_VERSION  0x01010101
#define OTA_HW_VERSION          0x0001

// === Индикация длительного нажатия ===
#define LONG_PRESS_LED_BLINK_DELAY  100  // Интервал моргания при длительном нажатии (мс)

#define LONG_PRESS_THRESHOLD_MS     10000 // Порог длительного нажатия (10 секунд)
#define BUTTON_SYNC_INTERVAL        10000
#define SENSOR_UPDATE_INTERVAL      30000

static const char *manufacturer = "Eric-1dev";
static const char *model = "DoubleSwitch";

// === Очередь для логов ===
static QueueHandle_t logQueue = nullptr;
static const int LOG_QUEUE_SIZE = 32;
static const int LOG_MESSAGE_MAX_LEN = 256;

// Структура для сообщений в очередь
typedef struct {
    char message[LOG_MESSAGE_MAX_LEN];
} log_message_t;

// === Флаг для глобального управления морганием светодиода ===
static volatile bool ledBlinkingMode = false;  // true = режим моргания (для длинного нажатия)

// === Функция для безопасного логирования ===
void safeLog(const char* format, ...) {
    if (logQueue == nullptr) return;
    
    log_message_t msg;
    va_list args;
    va_start(args, format);
    vsnprintf(msg.message, LOG_MESSAGE_MAX_LEN, format, args);
    va_end(args);
    
    // Неблокирующая отправка, если очередь полна - пропускаем сообщение
    xQueueSend(logQueue, &msg, pdMS_TO_TICKS(2));
}

// === Глобальные переменные ===
static bool relayState[2] = {false, false};
static bool zigbeeConnected = false;
static volatile bool otaRunning = false;

static QueueHandle_t buttonQueue = nullptr;

// Переменные для дыхания
static int breath_brightness = 0;
static int breath_direction = 1;

// === Zigbee устройства ===
ZigbeeLight zbLight1 = ZigbeeLight(LIGHT_EP1);
ZigbeeLight zbLight2 = ZigbeeLight(LIGHT_EP2);
ZigbeeTempSensor zbTempHumSensor = ZigbeeTempSensor(TEMP_HUMIDITY_SENSOR_EP);
ZigbeePressureSensor zbPressureSensor = ZigbeePressureSensor(PRESSURE_SENSOR_EP);

// === BME280 ===
Adafruit_BME280 bme;

// === Управление LED через analogWrite ===
void setLED(int brightness) {
    if (brightness < 0) brightness = 0;
    if (brightness > 1023) brightness = 1023;
    analogWrite(LED_PIN, brightness);
}

// === Управление реле ===
void updateRelays() {
    digitalWrite(RELAY1_PIN, relayState[0] ? HIGH : LOW);
    digitalWrite(RELAY2_PIN, relayState[1] ? HIGH : LOW);
}

void setRelay(uint8_t index, bool state) {
    if (index >= 2) return;
    if (relayState[index] == state) return;

    relayState[index] = state;
    updateRelays();

    safeLog("Реле %d → %s", index + 1, state ? "ВКЛ" : "ВЫКЛ");
}

// === Callbacks ===
void onLightChange1(bool state) { setRelay(0, state); }
void onLightChange2(bool state) { setRelay(1, state); }

void otaStateCallback(bool active) {
    otaRunning = active;
    if (active) {
        safeLog("OTA обновление начато...");
    } else {
        safeLog("OTA обновление завершено");
    }
}

// === Прерывания кнопок ===
void IRAM_ATTR gpioInterruptHandler(void* arg) {
    uint8_t pin = *(uint8_t*)arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(buttonQueue, &pin, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

void setupGPIOs() {
    pinMode(RELAY1_PIN, OUTPUT);
    pinMode(RELAY2_PIN, OUTPUT);
    pinMode(BUTTON1_PIN, INPUT_PULLUP);
    pinMode(BUTTON2_PIN, INPUT_PULLUP);
    digitalWrite(RELAY1_PIN, LOW);
    digitalWrite(RELAY2_PIN, LOW);

    // Настраиваем PWM для конкретного пина
    analogWriteResolution(LED_PIN, LED_PWM_RESOLUTION);
    analogWriteFrequency(LED_PIN, LED_PWM_FREQ);

    setLED(0);

    buttonQueue = xQueueCreate(10, sizeof(uint8_t));
    static uint8_t p1 = BUTTON1_PIN, p2 = BUTTON2_PIN;
    attachInterruptArg(BUTTON1_PIN, gpioInterruptHandler, &p1, FALLING);
    attachInterruptArg(BUTTON2_PIN, gpioInterruptHandler, &p2, FALLING);
}

// === Задача кнопок ===
void buttonTask(void* pvParameters) {
    while (1) {
        uint8_t btnPin;
        if (xQueueReceive(buttonQueue, &btnPin, portMAX_DELAY) == pdTRUE) {
            vTaskDelay(20 / portTICK_PERIOD_MS);  // Дребезг
            if (digitalRead(btnPin) != LOW) continue;

            uint32_t startTime = xTaskGetTickCount();
            bool longPressDetected = false;
            bool longPressTriggered = false;
            
            // Включаем режим моргания, если кнопка нажата долго
            while (digitalRead(btnPin) == LOW) {
                uint32_t pressDuration = (xTaskGetTickCount() - startTime) * portTICK_PERIOD_MS;
                
                // Если прошло больше порога длительного нажатия
                if (pressDuration > LONG_PRESS_THRESHOLD_MS) {
                    if (!longPressDetected) {
                        longPressDetected = true;
                        safeLog("Длительное нажатие кнопки %d (порог достигнут)", 
                                (btnPin == BUTTON1_PIN) ? 1 : 2);
                    }
                    
                    // Включаем режим моргания светодиода
                    if (!ledBlinkingMode) {
                        ledBlinkingMode = true;
                        safeLog("Светодиод: режим моргания (отпустите кнопку для действия)");
                    }
                    
                    // Проверяем, не пора ли выполнить действие
                    if (pressDuration > LONG_PRESS_THRESHOLD_MS + 2000) {
                        // Дополнительная защита от случайного срабатывания
                        if (!longPressTriggered) {
                            safeLog("⚠ ВНИМАНИЕ: длительное удержание кнопки! Отпустите для действия.");
                            longPressTriggered = true;
                        }
                    }
                }
                
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
            
            // Кнопка отпущена
            uint32_t totalPressTime = (xTaskGetTickCount() - startTime) * portTICK_PERIOD_MS;
            
            // Выключаем режим моргания
            if (ledBlinkingMode) {
                ledBlinkingMode = false;
                vTaskDelay(10 / portTICK_PERIOD_MS); // Даем время на смену режима
            }
            
            // Определяем тип нажатия и выполняем действие
            if (longPressDetected) {
                // Длинное нажатие выполнено
                safeLog("Кнопка %d отпущена после %lu мс (длинное нажатие)", 
                        (btnPin == BUTTON1_PIN) ? 1 : 2, totalPressTime);
                
                if (otaRunning) {
                    safeLog("OTA в процессе — действие отменено");
                    continue;
                }
                
                if (btnPin == BUTTON1_PIN) {
                    // Кнопка 1: перезагрузка устройства
                    safeLog("Выполняется перезагрузка устройства...");
                    vTaskDelay(300 / portTICK_PERIOD_MS);
                    ESP.restart();
                } else {
                    // Кнопка 2: сброс Zigbee сети
                    safeLog("Выполняется factory reset устройства...");
                    Zigbee.factoryReset();
                    vTaskDelay(800 / portTICK_PERIOD_MS);
                    ESP.restart();
                }
            } else {
                // Короткое нажатие
                safeLog("Кнопка %d: короткое нажатие (%lu мс)", 
                        (btnPin == BUTTON1_PIN) ? 1 : 2, totalPressTime);
                
                uint8_t idx = (btnPin == BUTTON1_PIN) ? 0 : 1;
                bool newState = !relayState[idx];
                
                setRelay(idx, newState);
                
                if (idx == 0)
                    zbLight1.setLight(newState);
                else
                    zbLight2.setLight(newState);
            }
        }
    }
}

// === Задача управления светодиодом ===
void ledTask(void* pvParameters) {
    static bool ledState = false;
    static uint32_t lastBlinkTime = 0;
    
    while (1) {
        if (otaRunning) {
            // При OTA - быстро мигаем
            setLED(0);
            vTaskDelay(pdMS_TO_TICKS(100));
            setLED(1023);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (ledBlinkingMode) {
            // Режим моргания при длительном нажатии
            uint32_t currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (currentTime - lastBlinkTime > LONG_PRESS_LED_BLINK_DELAY) {
                ledState = !ledState;
                setLED(ledState ? 1023 : 0);
                lastBlinkTime = currentTime;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (zigbeeConnected) {
            // Подключено к сети Zigbee - просто горим
            setLED(0);
            vTaskDelay(pdMS_TO_TICKS(1000));
        } else {
            // Нет сети Zigbee - режим дыхания
            breath_brightness += breath_direction * BREATH_STEP;

            if (breath_brightness >= 1023) {
                breath_brightness = 1023;
                breath_direction = -1;
            }
            else if (breath_brightness <= 0) {
                breath_brightness = 0;
                breath_direction = 1;
            }

            setLED(breath_brightness);
            vTaskDelay(pdMS_TO_TICKS(BREATH_DELAY_MS));
        }
    }
}

// === Задача сенсора ===
void sensorTask(void* pvParameters) {
    const TickType_t interval = pdMS_TO_TICKS(SENSOR_UPDATE_INTERVAL);
    bool bmeInitialized = false;

    while (1) {
        if (!bmeInitialized) {
            bmeInitialized = bme.begin(0x76, &Wire) || bme.begin(0x77, &Wire);
            if (!bmeInitialized) {
                safeLog("Ошибка: датчик BME280 не найден!");
                vTaskDelay(interval);
                continue;
            }
        }

        float t = bme.readTemperature();
        float h = bme.readHumidity();
        float p = bme.readPressure() / 100.0F;

        // Проверяем корректность данных
        if (!isnan(t) && !isnan(h) && !isnan(p)) {
            safeLog("─────────────── Показания датчика ───────────────");
            safeLog("Температура:   %.1f °C", t);
            safeLog("Влажность:     %.1f %%", h);
            safeLog("Давление:      %.1f гПа", p);
            safeLog("───────────────────────────────────────────────");

            if (zigbeeConnected) {
                zbTempHumSensor.setTemperature(t);
                zbTempHumSensor.setHumidity(h);
                zbPressureSensor.setPressure(p);
            }
        } else {
            safeLog("Ошибка чтения данных BME280");
            bmeInitialized = false; // Переинициализируем в следующем цикле
        }

        vTaskDelay(interval);
    }
}

// === Задача периодической синхронизации состояния реле ===
void syncRelayTask(void* pvParameters) {
    const TickType_t interval = pdMS_TO_TICKS(BUTTON_SYNC_INTERVAL); // каждые 60 секунд

    while (1) {
        if (zigbeeConnected && !otaRunning) {
            // Принудительно отправляем текущее состояние
            zbLight1.setLight(relayState[0]);
            zbLight2.setLight(relayState[1]);
            safeLog("Состояние реле синхронизировано с Zigbee");
        }
        vTaskDelay(interval);
    }
}

// === Задача вывода в Serial ===
void serialTask(void* pvParameters) {
    log_message_t msg;
    
    while (1) {
        if (xQueueReceive(logQueue, &msg, portMAX_DELAY) == pdTRUE) {
            // Вывод в Serial
            Serial.println(msg.message);
            
            // Небольшая пауза, если очередь почти полна
            if (uxQueueMessagesWaiting(logQueue) > 5) {
                vTaskDelay(pdMS_TO_TICKS(5));
            }
        }
    }
}

// === Setup ===
void setup() {
    Serial.begin(115200);
    delay(500);
    
    // Создаем очередь для логов
    logQueue = xQueueCreate(LOG_QUEUE_SIZE, sizeof(log_message_t));
    
    // Инициализируем GPIO
    setupGPIOs();
    
    // Инициализируем I2C
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    
    // Выводим начальное сообщение
    safeLog("\n\nЗапуск Zigbee Double Switch + BME280");
    safeLog("Версия прошивки: 2025-xx-xx");
    safeLog("Режим светодиода:");
    safeLog("  - горит: сеть Zigbee есть");
    safeLog("  - дышит: сети Zigbee нет");
    safeLog("  - моргает: длительное нажатие кнопки");
    safeLog("Управление кнопками:");
    safeLog("  Кнопка 1: короткое - реле 1, длинное - перезагрузка");
    safeLog("  Кнопка 2: короткое - реле 2, длинное - сброс Zigbee сети");
    safeLog("  Длительное нажатие: моргание светодиода, действие при отпускании");
    safeLog("----------------------------------------");

    // Настраиваем Zigbee устройства
    zbLight1.setManufacturerAndModel(manufacturer, model);
    zbLight2.setManufacturerAndModel(manufacturer, model);
    zbTempHumSensor.setManufacturerAndModel(manufacturer, model);
    zbPressureSensor.setManufacturerAndModel(manufacturer, model);

    zbTempHumSensor.addHumiditySensor();
    zbTempHumSensor.addTimeCluster();

    zbLight1.onLightChange(onLightChange1);
    zbLight2.onLightChange(onLightChange2);

    zbLight1.addOTAClient(OTA_RUNNING_VERSION, OTA_DOWNLOADED_VERSION, OTA_HW_VERSION);
    zbLight1.onOTAStateChange(otaStateCallback);

    Zigbee.addEndpoint(&zbLight1);
    Zigbee.addEndpoint(&zbLight2);
    Zigbee.addEndpoint(&zbTempHumSensor);
    Zigbee.addEndpoint(&zbPressureSensor);

    safeLog("Запуск Zigbee в режиме роутера...");
    if (Zigbee.begin(ZIGBEE_ROUTER)) {
        zbTempHumSensor.setReporting(30, 300, 0.5);
        zbTempHumSensor.setHumidityReporting(30, 300, 1.0);
        zbPressureSensor.setReporting(30, 300, 1.0);
        
        zbLight1.requestOTAUpdate();

        safeLog("Zigbee инициализирован успешно");
    } else {
        safeLog("Ошибка инициализации Zigbee. Работаем в автономном режиме");
    }

    // Создаем задачи FreeRTOS
    xTaskCreate(serialTask, "Serial", 4096, nullptr, 1, nullptr);    // Низкий приоритет
    xTaskCreate(buttonTask, "Btn",   2048, nullptr, 3, nullptr);     // Средний приоритет
    xTaskCreate(sensorTask, "Sens",  4096, nullptr, 2, nullptr);     // Низкий приоритет
    xTaskCreate(ledTask,    "LED",   2048, nullptr, 4, nullptr);     // Высокий приоритет
    xTaskCreate(syncRelayTask, "Sync", 2048, nullptr, 2, nullptr);   // Низкий приоритет

    safeLog("Инициализация завершена, запуск задач...");
}

// === Основной цикл ===
void loop() {
    static uint32_t lastCheck = 0;
    static bool lastZigbeeState = false;

    if (millis() - lastCheck > 2000) {   // Проверяем каждые 2 секунды
        bool nowConnected = Zigbee.connected();

        if (nowConnected != zigbeeConnected) {
            zigbeeConnected = nowConnected;
            
            safeLog(zigbeeConnected ? "✅ Zigbee: сеть настроена" : "❌ Zigbee: устройство не подключено к сети");
        }

        lastCheck = millis();
    }

    vTaskDelay(pdMS_TO_TICKS(500));  // Освобождаем процессор
}