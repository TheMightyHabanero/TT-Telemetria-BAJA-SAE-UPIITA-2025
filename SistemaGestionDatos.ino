#include <Wire.h>                                           // Comunicación I2C
#include <SPI.h>                                            // Comunicación SPI
#include <SD.h>                                             // Manejo de tarjeta SD
#include <WiFi.h>                                           // Necesaria para ESP32 (aunque no se usa WiFi aquí)
#include <esp_now.h>                                        // Comunicación inalámbrica directa (tampoco se usa en este código)
#include <Adafruit_ICM20948.h>                              // Sensor IMU de 9 ejes
#include <Adafruit_MLX90614.h>                              // Sensor de temperatura infrarrojo
#include <Adafruit_SHT4x.h>                                 // Sensor SHT45
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>           // Sensor GPS NEO M9N
#include <HX711.h>                                          // Sensor de Presion y HX711
#include <HardwareSerial.h>                                 // Definicion de Serial para LoRa
#include <esp_task_wdt.h>                                   // Para controlar el Watch Dog
#include <stdio.h>                                          // Para snprintf
#include <Adafruit_NeoPixel.h>                              // Para utilizar el Led
#include <MadgwickAHRS.h>                                   // Filtro para Pitch, Roll y Yaw

// BUFFER //////////////////////////

#define BUFFER_SIZE 512
static char txBuffer[BUFFER_SIZE];


// DEFINICION ENTRADAS ///////////////////////////////////////////////////////////////////////////////

// I2C_1 (SHT45, MLX90614, ICM20948_3)
#define SDA1_PIN 2
#define SCL1_PIN 1

// I2C_2 (ICM20948_1, ICM20948_2, GPS NEO M9N)
#define SDA2_PIN 8
#define SCL2_PIN 9

// SPI (Conector Micro SD)
#define SPI_MOSI 13 
#define SPI_SCK  12  
#define SPI_MISO 11 
#define SD_CS    14 

// PWM  (RPM)
#define INTERRUPT_PIN 15

//HX711 (Presion)
#define DOUT 47
#define CLK  21

// UART (LoRa)
#define RXD2 44
#define TXD2 43

// Dirección MAC del receptor
uint8_t broadcastAddress[] = {0xA0, 0x85, 0xE3, 0xE3, 0x43, 0xA0}; 
//uint8_t broadcastAddress[] = {0x48, 0x31, 0xB7, 0x57, 0xA4, 0x54}; 

// SENSOR OBJECTS ///////////////////////////////////////////////////////////////////////////////
Adafruit_ICM20948 IMU1;
Adafruit_ICM20948 IMU2;
Adafruit_ICM20948 IMU3;
Adafruit_MLX90614 T_CVT;
Adafruit_SHT4x SHT45;
SFE_UBLOX_GNSS NEO_M9N;
HX711 scale;
TwoWire I2C_2 = TwoWire(1);
HardwareSerial LoRaSerial(2);
Adafruit_NeoPixel strip(1, 48, NEO_GRB + NEO_KHZ800);
Madgwick filter1, filter2, filter3;

// Task handles ///////////////////////////////////////////////////////////////////////////////
TaskHandle_t IMU1_TaskHandle = NULL;
TaskHandle_t IMU2_TaskHandle = NULL;
TaskHandle_t IMU3_TaskHandle = NULL;
TaskHandle_t NEOM9N_TaskHandle = NULL;
TaskHandle_t T_CVT_TaskHandle = NULL;
TaskHandle_t print_TaskHandle = NULL;
TaskHandle_t SHT45_TaskHandle = NULL;
TaskHandle_t PWM_TaskHandle = NULL;
TaskHandle_t LoRa_TaskHandle = NULL;
TaskHandle_t HX711_TaskHandle = NULL;
TaskHandle_t ESP_NOW_TaskHandle = NULL;


// Variables compartidas ///////////////////////////////////////////////////////////////////////////////
float accelX1 = 0, accelY1 = 0, accelZ1 = 0, 
      accelX2 = 0, accelY2 = 0, accelZ2 = 0,
      accelX3 = 0, accelY3 = 0, accelZ3 = 0;

float gyroX1 = 0, gyroY1 = 0, gyroZ1 = 0,
      gyroX2 = 0, gyroY2 = 0, gyroZ2 = 0,
      gyroX3 = 0, gyroY3 = 0, gyroZ3 = 0;

float magX1 = 0, magY1 = 0, magZ1 = 0,
      magX2 = 0, magY2 = 0, magZ2 = 0,
      magX3 = 0, magY3 = 0, magZ3 = 0;

float pitch1, roll1, yaw1,
      pitch2, roll2, yaw2,
      pitch3, roll3, yaw3;

float tempAmbMLX = 0, tempObjMLX = 0, Velocidad = 0;
int RPMS = 0;
long lastTime = 0;
float tempSHT = 0, humSHT = 0;
long longitud = 0, altitud = 0, latitud = 0, SIVG = 0, Dia_UTC = 0, Hora_UTC = 0, Min_UTC = 0, Seg_UTC = 0;


// Factor de escala calibrado
float peso_kg = 0.0;
float factor_escala = 17759.54;

String dataString = "";

// Variables para calibración del magnetómetro
float mag_offsets[3] = {0, 0, 0};
bool calibrated = false;

// Mutex for synchronization
SemaphoreHandle_t xI2C1Mutex = NULL;  // Mutex para I2C1 (Wire)
SemaphoreHandle_t xI2C2Mutex = NULL;  // Mutex para I2C2 (I2C_2)
SemaphoreHandle_t xDataMutex = NULL;  // Para variables globales
SemaphoreHandle_t xBufferMutex = NULL;  // Para Buffer

// PWM Signal Variables
volatile unsigned long lastInterruptTime = 0;
volatile unsigned long timeDifference = 0;

// Frequency Averaging
const int numReads = 15;
float frequencyBuffer[numReads] = {0.0};
int bufferIndex = 0;
float frequencySum = 0.0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// Creacion de funciones Core 0 ///////////////////////////////////////////////////////////////////////////////
void IMU1_Task(void *pvParameters);
void IMU2_Task(void *pvParameters);
void IMU3_Task(void *pvParameters);
void T_CV_TTask(void *pvParameters);
void SHT45_Task(void *pvParameters);
void NEOM9N_Task(void *pvParameters);
void PWM_Task(void *pvParameters);
void HX711_Task(void *pvParameters);


// Creacion de funciones Core 1 ///////////////////////////////////////////////////////////////////////////////
void LoRa_Task(void *pvParameters);
void ESPNOW_Task(void *pvParameters);
void sendCmd(String cmd);
void print_Task(void *pvParameters);

// Creacion de funciones Generales ///////////////////////////////////////////////////////////////////////////////

void IRAM_ATTR onInterrupt() {
    static uint32_t last_interrupt = 0;
    uint32_t now = micros();
    
    // Debounce de 10ms
    if (now - last_interrupt > 10000) {  
        portENTER_CRITICAL_ISR(&mux);
        if (lastInterruptTime != 0) {
            timeDifference = now - lastInterruptTime;
        }
        lastInterruptTime = now;
        portEXIT_CRITICAL_ISR(&mux);
        last_interrupt = now;
    }
}

// Funcion de enviar datos ESP (Direccion MAC)

//void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    //Serial.printf("Estado de envío a %02X:%02X:%02X:%02X:%02X:%02X: %s\n",
                 //mac_addr[0], mac_addr[1], mac_addr[2],
                 //mac_addr[3], mac_addr[4], mac_addr[5],
                 //status == ESP_NOW_SEND_SUCCESS ? "Éxito" : "Fallo");
//}

// Funcion de recibir datos ESP

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  String receivedString = String((char*)incomingData).substring(0, len);
  Serial.println("Datos recibidos: " + receivedString);
}

//// SET - UP //////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
    
    Serial.begin(115200);
    scale.begin(DOUT, CLK);                                             // Inicia el sensor de presion con HX711
    vTaskDelay(1000 / portTICK_PERIOD_MS); 
   
    strip.begin();
    strip.setPixelColor(0,55,0,0); // Rojo
    strip.show(); // 

    filter1.begin(100);
    filter2.begin(100);
    filter3.begin(100);

    // Config Watch Dog ///
    esp_task_wdt_config_t wdt_config = {
    .timeout_ms = 30000,                                               // 30 segundos WatchDog
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
    .trigger_panic = true
    };
    ESP_ERROR_CHECK(esp_task_wdt_init(&wdt_config));

    // Esperar a que el puerto serie esté disponible
    int timeout = 0;
    while (!Serial && timeout < 50) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        timeout++;
    }
        
    // Inicializar el mutex antes de las tareas
    xI2C1Mutex = xSemaphoreCreateMutex();  // Crear mutex para I2C1
    xI2C2Mutex = xSemaphoreCreateMutex();  // Crear mutex para I2C2
    xDataMutex = xSemaphoreCreateMutex();  // Crear mutex para data
    xBufferMutex = xSemaphoreCreateMutex();  // Crear mutex para Buffer

    if (xI2C1Mutex == NULL) {
        Serial.println("Error al crear el mutex de I2C_1");
        return;
    }
    
    if (xI2C2Mutex == NULL) {
        Serial.println("Error al crear el mutex de I2C_2");
        return;
    }

    if (xDataMutex == NULL) {
        Serial.println("Error al crear el mutex de Datos");
        return;
    }

    if (xBufferMutex == NULL) {
        Serial.println("Error al crear el mutex de buffer");
        return;
    }

    // Inicializar I2C y SPI
    Wire.begin(SDA1_PIN, SCL1_PIN);
    Wire.setClock(100000);                      // 50kHz inicial
    Wire.setTimeOut(500);                        // Timeout en ms
    I2C_2.begin(SDA2_PIN, SCL2_PIN);
    I2C_2.setClock(100000);                     // 50kHz inicial
    I2C_2.setTimeOut(500);                       // Timeout en ms
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    
    // Inicializacion de Sensores ///////////////////////////////////////////////////////////////////////////////
    if (!scale.is_ready()) {
    Serial.println("HX711 Error Inicializacion");
    while (1) delay(10); // Bucle infinito si no se encuentra el sensor
    } Serial.println("HX711 Listo");

     //Calibración HX711 - Pedal freno
    scale.set_scale(factor_escala);
    scale.tare();

    
    if (!SD.begin(SD_CS)) {
        Serial.println("SD Card Error Inicializacion 1er Intento");
        const TickType_t xFrequency = pdMS_TO_TICKS(100);
        if (!SD.begin(SD_CS)) {
            Serial.println("SD Card Error Inicializacion 2do Intento");
        }
    }
    
    // SD.remove("/Prueba1.txt");  //Codigo para Borrar archivo cada inicio
    // Serial.print("Archivo borrado.");
    Serial.println("SD Card Listo");

    // Inicialización de IMU1 con comprobación de errores
    if (!IMU1.begin_I2C(ICM20948_I2CADDR_DEFAULT, &Wire)) {
        Serial.println("IMU1 Error Inicializacion 1er Intento");
        const TickType_t xFrequency = pdMS_TO_TICKS(100);
        if (!IMU1.begin_I2C(ICM20948_I2CADDR_DEFAULT, &Wire)) {
            Serial.println("IMU1 Error Inicializacion 2do Intento");
        }
    }
    Serial.println("IMU1 Listo");

    // Inicialización de IMU2 con comprobación de errores
    if (!IMU2.begin_I2C(ICM20948_I2CADDR_DEFAULT, &I2C_2)) {
        Serial.println("IMU2 Error Inicializacion 1er Intento");
        const TickType_t xFrequency = pdMS_TO_TICKS(100);
        if (!IMU2.begin_I2C(ICM20948_I2CADDR_DEFAULT, &I2C_2)) {
            Serial.println("IMU2 Error Inicializacion 2do Intento");
        }
    }
    Serial.println("IMU2 Listo");

    // Inicialización de IMU3 con comprobación de errores
    if (!IMU3.begin_I2C(ICM20948_I2CADDR_DEFAULT, &I2C_2)) {
        Serial.println("IMU3 Error Inicializacion 1er Intento");
        const TickType_t xFrequency = pdMS_TO_TICKS(100);
        if (!IMU3.begin_I2C(ICM20948_I2CADDR_DEFAULT, &I2C_2)) {
            Serial.println("IMU3 Error Inicializacion 2do Intento");
        }
    }
    Serial.println("IMU3 Listo");
    
    // Inicialización de SHT45 con comprobación de errores
    if (!SHT45.begin(&Wire)) {
        Serial.println("SHT45 Error Inicializacion 1er Intento");
        const TickType_t xFrequency = pdMS_TO_TICKS(100);
        if (!SHT45.begin(&Wire)) {
            Serial.println("SHT45 Error Inicializacion 2do Intento");
        }
    }
    Serial.println("SHT45 Listo");

    // Inicialización de MLX90614 con comprobación de errores
    
    if (!T_CVT.begin(MLX90614_I2CADDR, &Wire)) {
        Serial.println("MLX90614 Error Inicializacion 1er Intento");
        const TickType_t xFrequency = pdMS_TO_TICKS(100);
        if (!T_CVT.begin(MLX90614_I2CADDR, &Wire)) {
            Serial.println("MLX90614 Error Inicializacion 2do Intento");
        }
    }
    Serial.println("MLX90614 Listo");

    // Inicialización del GPS con comprobación de errores
    if (!NEO_M9N.begin(I2C_2)) {
        Serial.println("GPS Error Inicializacion 1er Intento");
        if (!NEO_M9N.begin(I2C_2)) {
            Serial.println("GPS Error Inicializacion 2do Intento");
        }
    }
    Serial.println("GPS Listo");

    // Configuración de IMU1
    IMU1.setMagDataRate(AK09916_MAG_DATARATE_100_HZ);
    IMU1.setAccelRange(ICM20948_ACCEL_RANGE_8_G);
    IMU1.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);

    // Configuración de IMU2
    IMU2.setMagDataRate(AK09916_MAG_DATARATE_100_HZ);
    IMU2.setAccelRange(ICM20948_ACCEL_RANGE_8_G);
    IMU2.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);

    // Configuración de IMU3
    IMU3.setMagDataRate(AK09916_MAG_DATARATE_100_HZ);
    IMU3.setAccelRange(ICM20948_ACCEL_RANGE_8_G);
    IMU3.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);

    // Configuración de la interrupción para RPM
    pinMode(INTERRUPT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), onInterrupt, FALLING);

    // Configuración del GPS
    NEO_M9N.setI2COutput(COM_TYPE_UBX);   // Salida en UBX para eficiencia
    NEO_M9N.setDynamicModel(DYN_MODEL_AUTOMOTIVE);
    NEO_M9N.setNavigationFrequency(1);   // 10Hz de actualización
    NEO_M9N.setAutoPVT(true);

    // Configuración de SHT45
    SHT45.setPrecision(SHT4X_HIGH_PRECISION);
    SHT45.setHeater(SHT4X_NO_HEATER);
    
    // Lora Set Up /////////////////////////////////////////////////////////////////////////////////////////
    LoRaSerial.begin(115200, SERIAL_8N1, RXD2, TXD2);
    vTaskDelay(500 / portTICK_PERIOD_MS); // Dar tiempo al módulo para inicializarse
    
    Serial.println("Configurando parámetros antena LoRa");
    sendCmd("AT+ADDRESS=1");
    sendCmd("AT+NETWORKID=18");
    sendCmd("AT+BAND=915000000");
    sendCmd("AT+PARAMETER=9,9,4,24");
    sendCmd("AT+MODE=0");  // Modo normal explícito
    sendCmd("AT+CRFOP=22"); // Potencia de transmisión
    // Verificar configuración
    sendCmd("AT+PARAMETER?");
    sendCmd("AT+ADDRESS?");

    // Configuracion ESP-NOW ///////////////////////////////////////////////////////////////////////////////

    // 1. Configurar WiFi
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    
    // 2. Inicializar ESP-NOW
    esp_err_t initStatus = esp_now_init();
    if (initStatus != ESP_OK) {
        Serial.printf("Error inicializando ESP-NOW: 0x%X\n", initStatus);
        if (initStatus == ESP_ERR_ESPNOW_NOT_INIT) {
            Serial.println("WiFi no inicializado correctamente");
        }
        ESP.restart(); // Reiniciar en caso de error grave
        return;
    }

    // 3. Registrar callbacks ANTES de agregar peers
    //esp_now_register_send_cb(OnDataSent);
    
    // 4. Configurar peer con más parámetros
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo)); // Limpiar la estructura
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;                   // Mismo canal que el receptor
    peerInfo.ifidx = WIFI_IF_STA;           // Usar interfaz Station
    peerInfo.encrypt = false;                // Sin encriptación
    
    // 5. Agregar peer con verificación
    esp_err_t peerStatus = esp_now_add_peer(&peerInfo);
    if (peerStatus != ESP_OK) {
        Serial.printf("Error agregando peer: 0x%X\n", peerStatus);
        if (peerStatus == ESP_ERR_ESPNOW_NOT_FOUND) {
            Serial.println("Verifica que el receptor esté encendido");
        }
        // No salir, permitir reintentos
    } else {
        Serial.println("Peer agregado correctamente");
    }

    // Procesamiento por Cores ///////////////////////////////////////////////////////////////////////////////
    // Core 0


    if (xTaskCreatePinnedToCore(IMU1_Task, "ICM1 Task", 6144, NULL, 4, &IMU1_TaskHandle, 0) != pdPASS) {
        Serial.println("Error al crear tarea IMU1");
    }

    if (xTaskCreatePinnedToCore(IMU2_Task, "ICM2 Task", 6144, NULL, 4, &IMU1_TaskHandle, 0) != pdPASS) {
        Serial.println("Error al crear tarea IMU2");
    }

    if (xTaskCreatePinnedToCore(IMU3_Task, "ICM3 Task", 6144, NULL, 4, &IMU1_TaskHandle, 0) != pdPASS) {
        Serial.println("Error al crear tarea IMU3");
    }
    

    if (xTaskCreatePinnedToCore(T_CV_TTask, "MLX Task", 4096, NULL, 3, &T_CVT_TaskHandle, 0) != pdPASS) {
        Serial.println("Error al crear tarea CVT");
    }
    
    if (xTaskCreatePinnedToCore(SHT45_Task, "SHT45 Task", 4096, NULL, 3, &SHT45_TaskHandle, 0) != pdPASS) {
        Serial.println("Error al crear tarea SHT45");
    }
    
    if (xTaskCreatePinnedToCore(NEOM9N_Task, "NEOM9N Task", 4096, NULL, 4, &NEOM9N_TaskHandle, 0) != pdPASS) {
        Serial.println("Error al crear tarea GPS");
    }
    
    if (xTaskCreatePinnedToCore(PWM_Task, "PWM Task", 4096, NULL, 3, &PWM_TaskHandle, 0) != pdPASS) {
        Serial.println("Error al crear tarea PWM");
    }

    if (xTaskCreatePinnedToCore(HX711_Task, "HX711 Task", 4096, NULL, 3, &HX711_TaskHandle, 0) != pdPASS) {
        Serial.println("Error al crear tarea Presion");
    }

    // Core 1
    if (xTaskCreatePinnedToCore(print_Task, "Print Task", 8192, NULL, 1, &print_TaskHandle, 1) != pdPASS) {
        Serial.println("Error al crear tarea Print");
    }
    
    
    if (xTaskCreatePinnedToCore(LoRa_Task, "LoRa Task", 8192, NULL, 1, &LoRa_TaskHandle, 1) != pdPASS) {
        Serial.println("Error al crear tarea LoRa");
    }

    
    if (xTaskCreatePinnedToCore(ESPNOW_Task, "ESPNOW Task", 8192, NULL, 2, &ESP_NOW_TaskHandle, 1) != pdPASS) {
        Serial.println("Error al crear tarea ESP - NOW");
    }
    
    if (xTaskCreatePinnedToCore(resourceMonitor, "Monitor", 4096, NULL, 1, NULL, 1) != pdPASS) {
        Serial.println("Error al crear tarea Monitor");
    }

    strip.setPixelColor(0,55,0,55); // Rojo
    strip.show(); // 

}

void loop() {
    // Mantener el loop() mínimo cuando se usa FreeRTOS
    vTaskDelay(100 / portTICK_PERIOD_MS);
}

void HX711_Task(void *pvParameters) { 
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
  esp_task_wdt_add(NULL);

  while(1) {
    vTaskDelay(1 / portTICK_PERIOD_MS);
    esp_task_wdt_reset();
    peso_kg = scale.get_units(); // Promediar 3 lecturas
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  esp_task_wdt_delete(NULL);
}

void IMU1_Task(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(20);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    esp_task_wdt_add(NULL);

    while (1) {
        //vTaskDelay(pdMS_TO_TICKS(1));
        esp_task_wdt_reset();
        sensors_event_t accel, gyro, mag, temp;
        if(xSemaphoreTake(xI2C1Mutex, pdMS_TO_TICKS(30)) == pdTRUE) {
            bool success = IMU1.getEvent(&accel, &gyro, &mag, &temp);
            xSemaphoreGive(xI2C1Mutex);

            if (!success) {
                Serial.println("[IMU1] Error al leer datos I2C");
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }

            if (success && xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(30)) == pdTRUE) {
                accelX1 = accel.acceleration.x;
                accelY1 = accel.acceleration.y;
                accelZ1 = accel.acceleration.z;

                gyroX1 = gyro.gyro.x;
                gyroY1 = gyro.gyro.y;
                gyroZ1 = gyro.gyro.z;

                magX1 = mag.magnetic.x;
                magY1 = mag.magnetic.y;
                magZ1 = mag.magnetic.z;

                filter1.update(gyroX1, gyroY1, gyroZ1, 
                               accelX1, accelY1, accelZ1, 
                               magX1, magY1, magZ1);

                // Obtiene los ángulos del filtro
                roll1 = filter1.getRoll();
                pitch1 = filter1.getPitch();
                yaw1 = filter1.getYaw();

                xSemaphoreGive(xDataMutex);
            }
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
    esp_task_wdt_delete(NULL);
}

void IMU2_Task(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(20);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    esp_task_wdt_add(NULL);

    while (1) {
        //vTaskDelay(pdMS_TO_TICKS(1));
        esp_task_wdt_reset();
        sensors_event_t accel, gyro, mag, temp;
        if(xSemaphoreTake(xI2C1Mutex, pdMS_TO_TICKS(30)) == pdTRUE) {
            bool success = IMU2.getEvent(&accel, &gyro, &mag, &temp);
            xSemaphoreGive(xI2C1Mutex);

            if (!success) {
                Serial.println("[IMU2] Error al leer datos I2C");
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }

            if (success && xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(30)) == pdTRUE) {
                accelX2 = accel.acceleration.x;
                accelY2 = accel.acceleration.y;
                accelZ2 = accel.acceleration.z;

                gyroX2 = gyro.gyro.x;
                gyroY2 = gyro.gyro.y;
                gyroZ2 = gyro.gyro.z;

                magX2 = mag.magnetic.x;
                magY2 = mag.magnetic.y;
                magZ2 = mag.magnetic.z;

                filter2.update(gyroX2, gyroY2, gyroZ2, 
                accelX2, accelY2, accelZ2, 
                magX2, magY2, magZ2);

                // Obtiene los ángulos del filtro
                roll2 = filter2.getRoll();
                pitch2 = filter2.getPitch();
                yaw2 = filter2.getYaw();

                xSemaphoreGive(xDataMutex);
            }
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
    esp_task_wdt_delete(NULL);
}

void IMU3_Task(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(20);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    esp_task_wdt_add(NULL);

    while (1) {
        //vTaskDelay(pdMS_TO_TICKS(1));
        esp_task_wdt_reset();
        sensors_event_t accel, gyro, mag, temp;
        if(xSemaphoreTake(xI2C1Mutex, pdMS_TO_TICKS(30)) == pdTRUE) {
            bool success = IMU3.getEvent(&accel, &gyro, &mag, &temp);
            xSemaphoreGive(xI2C1Mutex);

            if (!success) {
                Serial.println("[IMU3] Error al leer datos I2C");
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }

            if (success && xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(30)) == pdTRUE) {
                accelX3 = accel.acceleration.x;
                accelY3 = accel.acceleration.y;
                accelZ3 = accel.acceleration.z;

                gyroX3 = gyro.gyro.x;
                gyroY3 = gyro.gyro.y;
                gyroZ3 = gyro.gyro.z;

                magX3 = mag.magnetic.x;
                magY3 = mag.magnetic.y;
                magZ3 = mag.magnetic.z;

                filter3.update(gyroX3, gyroY3, gyroZ3, 
                accelX3, accelY3, accelZ3, 
                magX3, magY3, magZ3);

                // Obtiene los ángulos del filtro
                roll3 = filter3.getRoll();
                pitch3 = filter3.getPitch();
                yaw3 = filter3.getYaw();

                xSemaphoreGive(xDataMutex);
            }
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
    esp_task_wdt_delete(NULL);
}


void SHT45_Task(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(200);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    esp_task_wdt_add(NULL);

    while(1) {
        //vTaskDelay(pdMS_TO_TICKS(1));
        esp_task_wdt_reset();
        sensors_event_t humidity, temp;
        if(xSemaphoreTake(xI2C1Mutex, pdMS_TO_TICKS(30)) == pdTRUE) {
            bool success = SHT45.getEvent(&humidity, &temp);
            xSemaphoreGive(xI2C1Mutex);

            if (!success) {
                Serial.println("[SHT45] Error al leer datos I2C");
            }

            if (success && xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(30)) == pdTRUE) {
                tempSHT = temp.temperature; 
                humSHT = humidity.relative_humidity;
                xSemaphoreGive(xDataMutex);
            }
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
    esp_task_wdt_delete(NULL);
}

void NEOM9N_Task(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(500);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    esp_task_wdt_add(NULL);
    bool success = false;

    while (1) {
        //vTaskDelay(pdMS_TO_TICKS(1));
        esp_task_wdt_reset();
        // Intento rápido de lectura I2C (timeout más corto)
        if(xSemaphoreTake(xI2C2Mutex, pdMS_TO_TICKS(30)) == pdTRUE) {
            success = NEO_M9N.getPVT();
            xSemaphoreGive(xI2C2Mutex);
            
            if (success) {
                // Actualizar datos solo si la lectura fue exitosa
                if(xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(30)) == pdTRUE) {
                    latitud = NEO_M9N.getLatitude();
                    longitud = NEO_M9N.getLongitude();
                    altitud = NEO_M9N.getAltitude();
                    SIVG = NEO_M9N.getSIV();
                    Dia_UTC = NEO_M9N.getDay();
                    Hora_UTC = NEO_M9N.getHour() - 6; if (Hora_UTC < 0) Hora_UTC += 24; Dia_UTC--;
                    Min_UTC = NEO_M9N.getMinute();
                    Seg_UTC = NEO_M9N.getSecond();
                    Velocidad = NEO_M9N.getGroundSpeed() * 0.0036;
                    lastTime = millis();
                    
                    xSemaphoreGive(xDataMutex);
                }
            } else {
                Serial.println("[GPS] Error al obtener ubicacion");
                // Reintentar inmediatamente en el próximo ciclo
            }
        } else {
            Serial.println("[GPS] Timeout en I2C mutex");
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
    esp_task_wdt_delete(NULL);
}

void T_CV_TTask(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(400); // Mejor práctica para vTaskDelayUntil
    TickType_t xLastWakeTime = xTaskGetTickCount();
    esp_task_wdt_add(NULL);

    while (1) {
        //vTaskDelay(pdMS_TO_TICKS(1));
        esp_task_wdt_reset();
        float tempObjCVT, tempAmbCVT;
        bool readSuccess = false;

        // 1. Proteger acceso al bus I2C1
        if(xSemaphoreTake(xI2C1Mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            tempObjCVT = T_CVT.readObjectTempC();
            tempAmbCVT = T_CVT.readAmbientTempC();
            xSemaphoreGive(xI2C1Mutex);
            readSuccess = true;
        }

        if (!readSuccess) {
                Serial.println("[MLX90614] Error al leer datos I2C");
            }

        // 2. Si la lectura fue exitosa, actualizar datos compartidos
        if(readSuccess && (xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(30)) == pdTRUE)) {
            tempObjMLX = tempObjCVT;
            tempAmbMLX = tempAmbCVT;
            xSemaphoreGive(xDataMutex);
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
    esp_task_wdt_delete(NULL);
}

void print_Task(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(200);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    esp_task_wdt_add(NULL);

    // Asegurarse de que el archivo existe y está listo
    File archivo = SD.open("/Prueba1.txt", FILE_WRITE);
    if (archivo) {
        archivo.println("[SD] Inicio de registro de datos");
        archivo.close();
    } else {
        Serial.println("[SD] Error al abrir el archivo inicial");
        vTaskDelete(NULL);  // Terminar la tarea si hay un error crítico
        return;
    }
    
    while (1) {
        //vTaskDelay(pdMS_TO_TICKS(1));
        esp_task_wdt_reset();
        float ax1, ay1, az1, pt1, yw1, rll1, 
              ax2, ay2, az2, pt2, yw2, rll2, 
              ax3, ay3, az3, pt3, yw3, rll3, 
              tempObjCVT, tempAmbCVT, t45, hum45, KKG, vel;
        int rpm;
        long lat, lon, alt, siv, dia, hora, min, seg;

        if(xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(30)) == pdTRUE) {
            ax1 = accelX1; ay1 = accelY1; az1 = accelZ1;
            ax2 = accelX2; ay2 = accelY2; az2 = accelZ2;
            ax3 = accelX3; ay3 = accelY3; az3 = accelZ3;

            pt1 = pitch1;  pt2 = pitch2;  pt3 = pitch3;
            yw1 = yaw1;   yw2 = yaw2;   yw3 = yaw3;   
            rll1 = roll1;  rll2 = roll2;  rll3 = roll3;

            tempObjCVT = tempObjMLX;
            tempAmbCVT = tempAmbMLX;
            KKG = peso_kg;
            rpm = RPMS;
            t45 = tempSHT;
            hum45 = humSHT;
            
            lat = latitud;
            lon = longitud;
            alt = altitud;
            siv = SIVG;
            dia = Dia_UTC;
            hora = Hora_UTC;
            min = Min_UTC;
            seg = Seg_UTC;
            
            vel = Velocidad;

            xSemaphoreGive(xDataMutex);
        }

        if(xSemaphoreTake(xBufferMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            int bytesWritten = snprintf(txBuffer, BUFFER_SIZE, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%.2f,%.2f,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld",
                                            ax1, ay1, az1, pt1, yw1, rll1, 
                                            ax2, ay2, az2, pt2, yw2, rll2, 
                                            ax3, ay3, az3, pt3, yw3, rll3, 
                                            vel, tempObjCVT, tempAmbCVT, KKG, rpm, t45, hum45,
                                            lat, lon, alt, siv, dia, hora, min, seg);
            

            if(bytesWritten >= BUFFER_SIZE) {
                Serial.printf("[ERROR] Buffer overflow en %s\n", pcTaskGetName(NULL));
                memset(txBuffer, 0, BUFFER_SIZE);
                strncpy(txBuffer, "ERROR_DATA", BUFFER_SIZE-1);
            }
                                            
            // Escribir en la tarjeta SD
            File archivo = SD.open("/Prueba1.txt", FILE_APPEND);
            if (archivo) {
                archivo.println(txBuffer);
                archivo.close();
                Serial.printf("[SD] Escritos %d bytes\n", strlen(txBuffer));
            } else {
                Serial.println("Error de escritura SD");
            }
            
            Serial.printf("[SD] Valores escritos: %s\n", txBuffer);  
            xSemaphoreGive(xBufferMutex);
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
    esp_task_wdt_delete(NULL);
}

void PWM_Task(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(50);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    esp_task_wdt_add(NULL);

    while (1) {
        //vTaskDelay(pdMS_TO_TICKS(1));
        esp_task_wdt_reset();
        unsigned long timeDiff;

        portENTER_CRITICAL(&mux);
        timeDiff = timeDifference;
        portEXIT_CRITICAL(&mux);

        if (timeDiff > 13000 && timeDiff < 50000) {
            float frequency = 1000000.0 / timeDiff;
            
            if(xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(30)) == pdTRUE) {
                frequencySum -= frequencyBuffer[bufferIndex];
                frequencyBuffer[bufferIndex] = frequency;
                frequencySum += frequency;
                bufferIndex = (bufferIndex + 1) % numReads;
                RPMS = (int)((frequencySum / numReads) * 60);
                xSemaphoreGive(xDataMutex);
            }
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
    esp_task_wdt_delete(NULL);
}

void LoRa_Task(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(500); // Intervalo de ejecución de 500ms
    TickType_t xLastWakeTime = xTaskGetTickCount();
    esp_task_wdt_add(NULL);

    #define TX_BUFFER_SIZE 512
    #define LORA_TIMEOUT_MS 1000 // Timeout aumentado a 1 segundo
    char txBuffer[TX_BUFFER_SIZE];
    char atCommandBuffer[TX_BUFFER_SIZE + 30];

    // Inicialización del módulo LoRa
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    while (1) {
        esp_task_wdt_reset(); // Reset del watchdog al inicio del ciclo

        // 1. Obtener datos protegidos por mutex
        float ax1, ay1, az1, pt1, yw1, rll1, 
              ax2, ay2, az2, pt2, yw2, rll2, 
              ax3, ay3, az3, pt3, yw3, rll3, 
              tempObjCVT, tempAmbCVT, t45, hum45, KKG, vel;
        int rpm;
        long lat, lon, alt, siv, dia, hora, min, seg;

        if(xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(30)) == pdTRUE) {
            ax1 = accelX1; ay1 = accelY1; az1 = accelZ1;
            ax2 = accelX2; ay2 = accelY2; az2 = accelZ2;
            ax3 = accelX3; ay3 = accelY3; az3 = accelZ3;

            pt1 = pitch1;  pt2 = pitch2;  pt3 = pitch3;
            yw1 = yaw1;   yw2 = yaw2;   yw3 = yaw3;   
            rll1 = roll1;  rll2 = roll2;  rll3 = roll3;

            tempObjCVT = tempObjMLX;
            tempAmbCVT = tempAmbMLX;
            KKG = peso_kg;
            rpm = RPMS;
            t45 = tempSHT;
            hum45 = humSHT;
            
            lat = latitud;
            lon = longitud;
            alt = altitud;
            siv = SIVG;
            dia = Dia_UTC;
            hora = Hora_UTC;
            min = Min_UTC;
            seg = Seg_UTC;
            
            vel = Velocidad;

            xSemaphoreGive(xDataMutex);
        }

        if(xSemaphoreTake(xBufferMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            int bytesWritten = snprintf(txBuffer, BUFFER_SIZE, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%.2f,%.2f,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld",
                                            ax1, ay1, az1, pt1, yw1, rll1, 
                                            ax2, ay2, az2, pt2, yw2, rll2, 
                                            ax3, ay3, az3, pt3, yw3, rll3, 
                                            vel, tempObjCVT, tempAmbCVT, KKG, rpm, t45, hum45,
                                            lat, lon, alt, siv, dia, hora, min, seg);

            if(bytesWritten >= TX_BUFFER_SIZE) {
                strncpy(txBuffer, "ERR_OVF", TX_BUFFER_SIZE-1);
                bytesWritten = strlen(txBuffer);
            }

            // 3. Envío rápido sin esperar respuesta
            LoRaSerial.flush(); // Limpiar buffer de salida
            while(LoRaSerial.available()) LoRaSerial.read(); // Limpiar buffer de entrada
            
            LoRaSerial.printf("AT+SEND=2,%d,%s\r\n", bytesWritten, txBuffer);
            Serial.printf("[LoRa] Enviados %d bytes\n", bytesWritten);

            // Pequeña pausa para permitir el envío, luego limpiar cualquier respuesta
            vTaskDelay(pdMS_TO_TICKS(20));
            strip.setPixelColor(0,0,0,55); // Azul
            strip.show(); // 
            vTaskDelay(pdMS_TO_TICKS(100));
            strip.setPixelColor(0,55,0,55); // Azul
            strip.show(); // 
            while(LoRaSerial.available()) LoRaSerial.read();
        xSemaphoreGive(xBufferMutex);
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
    
    esp_task_wdt_delete(NULL);
}


void sendCmd(String cmd) {
    const uint32_t timeout = 1000; // 1 segundo de timeout
    const TickType_t delayBetweenChecks = pdMS_TO_TICKS(10); // Verificar cada 10ms
    
    if (!LoRaSerial) {
        Serial.println("Error: Puerto LoRa no disponible");
        return;
    }

    // Limpiar buffer de entrada antes de enviar
    while (LoRaSerial.available()) {
        LoRaSerial.read();
    }

    // Enviar comando
    LoRaSerial.println(cmd);
    Serial.println("Enviado: " + cmd);

    // Esperar respuesta con timeout
    uint32_t start = millis();
    String response = "";
    
    while (millis() - start < timeout) {
        while (LoRaSerial.available()) {
            char c = LoRaSerial.read();
            response += c;
            
            // Si recibimos fin de línea, procesar la respuesta
            if (c == '\n') {
                Serial.print("Respuesta LoRa: ");
                Serial.println(response);
                return; // Salir al recibir respuesta completa
            }
        }
        vTaskDelay(delayBetweenChecks); // Liberar CPU
    }

    Serial.println("Error: Timeout esperando respuesta de LoRa");
}

void ESPNOW_Task(void *pvParameters){
    const TickType_t xFrequency = pdMS_TO_TICKS(500);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    esp_task_wdt_add(NULL);
    esp_err_t result;

    while (1) {
        //vTaskDelay(pdMS_TO_TICKS(1));
        esp_task_wdt_reset();
        float ax1, ay1, az1, pt1, yw1, rll1, 
              ax2, ay2, az2, pt2, yw2, rll2, 
              ax3, ay3, az3, pt3, yw3, rll3, 
              tempObjCVT, tempAmbCVT, t45, hum45, KKG, vel;
        int rpm;
        long lat, lon, alt, siv, dia, hora, min, seg;

        if(xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(30)) == pdTRUE) {
            ax1 = accelX1; ay1 = accelY1; az1 = accelZ1;
            ax2 = accelX2; ay2 = accelY2; az2 = accelZ2;
            ax3 = accelX3; ay3 = accelY3; az3 = accelZ3;

            pt1 = pitch1;  pt2 = pitch2;  pt3 = pitch3;
            yw1 = yaw1;   yw2 = yaw2;   yw3 = yaw3;   
            rll1 = roll1;  rll2 = roll2;  rll3 = roll3;

            tempObjCVT = tempObjMLX;
            tempAmbCVT = tempAmbMLX;
            KKG = peso_kg;
            rpm = RPMS;
            t45 = tempSHT;
            hum45 = humSHT;
            
            lat = latitud;
            lon = longitud;
            alt = altitud;
            siv = SIVG;
            dia = Dia_UTC;
            hora = Hora_UTC;
            min = Min_UTC;
            seg = Seg_UTC;
            
            vel = Velocidad;

            xSemaphoreGive(xDataMutex);
        }

        if(xSemaphoreTake(xBufferMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            int bytesWritten = snprintf(txBuffer, BUFFER_SIZE, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%.2f,%.2f,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld",
                                            ax1, ay1, az1, pt1, yw1, rll1, 
                                            ax2, ay2, az2, pt2, yw2, rll2, 
                                            ax3, ay3, az3, pt3, yw3, rll3, 
                                            vel, tempObjCVT, tempAmbCVT, KKG, rpm, t45, hum45,
                                            lat, lon, alt, siv, dia, hora, min, seg);

            

            if(bytesWritten >= BUFFER_SIZE) {
                Serial.printf("[ERROR] Buffer overflow en %s\n", pcTaskGetName(NULL));
                memset(txBuffer, 0, BUFFER_SIZE);
                strncpy(txBuffer, "ERROR_DATA", BUFFER_SIZE-1);
            }
            
            // Enviar datos 
            const char* dataToSend = txBuffer;
            Serial.printf("[ESP NOW] Enviados %d bytes\n", bytesWritten);  
            esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)dataToSend, strlen(dataToSend));

            esp_now_register_recv_cb(OnDataRecv);               // Funcion de recibir datos

        xSemaphoreGive(xBufferMutex);
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
    esp_task_wdt_delete(NULL);
}

void resourceMonitor(void *pv) {
    while(1) {
        Serial.printf("[MEM] Free: %d | Min: %d\n", 
            esp_get_free_heap_size(), 
            esp_get_minimum_free_heap_size());
        vTaskDelay(10000/portTICK_PERIOD_MS);
    }
}
