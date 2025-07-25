#include "project.h"
#include <stdio.h>
#include <stdbool.h>

// Dirección I2C del MAX30102
#define MAX30102_ADDRESS    0x57

// Registros principales MAX30102
#define REG_FIFO_WR_PTR     0x04
#define REG_OVF_COUNTER     0x05
#define REG_FIFO_RD_PTR     0x06
#define REG_FIFO_DATA       0x07
#define REG_FIFO_CONFIG     0x08
#define REG_MODE_CONFIG     0x09
#define REG_SPO2_CONFIG     0x0A
#define REG_LED1_PA         0x0C
#define REG_LED2_PA         0x0D
#define REG_PART_ID         0xFF

// Configuración MAX30102
#define SAMPLE_RATE         100     // 100 Hz
#define BUFFER_SIZE         200     // Tamaño del buffer para análisis
#define IR_THRESHOLD        50000   // Umbral para detección de dedo
#define DC_FILTER_ALPHA     0.95    // Factor de filtrado DC

// Algoritmo SpO2
#define SPO2_BUFFER_SIZE    100
#define MAX_HR              200
#define MIN_HR              30
#define MAX_SPO2            100
#define MIN_SPO2            70

// Variables globales para sensores
volatile uint32 pulseCount = 0;
volatile uint8 flowMeasurementReady = 0;
char uartBuffer[128];

// Estructura SpO2
typedef struct {
    uint32_t ir[SPO2_BUFFER_SIZE];
    uint32_t red[SPO2_BUFFER_SIZE];
    uint8_t count;
    uint8_t head;
    uint8_t tail;
} SpO2Buffer;

// Variables MAX30102
uint32_t irBuffer[BUFFER_SIZE];
uint32_t redBuffer[BUFFER_SIZE];
int bufferIndex = 0;
bool fingerDetected = false;
int heartRate = 0;
int systolicBP = 0;
int diastolicBP = 0;
SpO2Buffer spo2Buffer;
int32_t spO2 = 0;
int8_t validSPO2 = 0;





// prototipo funciones
// Prototipos de funciones
void sendString(const char* str);
uint8_t writeI2C(uint8_t reg, uint8_t value);
uint8_t readI2CMultipleBytes(uint8_t reg, uint8_t *data, uint8_t len);
uint8_t checkSensor();
uint8_t configureSensor();
uint8_t getFIFOSamples();
uint8_t readFIFOData(uint32_t *redValue, uint32_t *irValue);
void initSystemTick();
void calculateHeartRate();
void calculateSpO2();
void estimateBloodPressure();
void updateBuffers(uint32_t red, uint32_t ir);
void spo2_update(uint32_t irValue, uint32_t redValue);
void spo2_calculate();
void spo2_init();
float spo2_compute_r_ratio();
void spo2_remove_dc_component();
float spo2_compute_ac_rms(uint32_t* signal);
float mySqrt(float x);
float myFabs(float x);


//flujo


// Interrupt Service Routines
CY_ISR(FlowSensor_ISR) {
    pulseCount++;
    FlowSensor_Pin_ClearInterrupt();
}

CY_ISR(Timer_ISR) {
    flowMeasurementReady = 1;
    Timer_ReadStatusRegister();
}

// Implementación de funciones matemáticas
float mySqrt(float x) {
    if (x <= 0) return 0;
    
    float y = x;
    float z = (y + x / y) / 2;
    
    while (myFabs(y - z) > 0.0001f) {
        y = z;
        z = (y + x / y) / 2;
    }
    return z;
}

float myFabs(float x) {
    return (x < 0) ? -x : x;
}




// Implementaciones de funciones MAX30102 (iguales al código original)
// [Todas las funciones desde sendString() hasta estimateBloodPressure()]
// ... (sin cambios respecto al código max.cpp original) ...

// Función para enviar string por UART
void sendString(const char* str) {
    while (*str) {
        UART_1_PutChar(*str);
        str++;
    }
}

// Función mejorada para escribir registro I2C
uint8_t writeI2C(uint8_t reg, uint8_t value) {
    uint8_t writeBuffer[2] = {reg, value};
    uint8_t status;
    
    I2C_1_MasterClearStatus();
    
    for (int retry = 0; retry < 3; retry++) {
        status = I2C_1_MasterWriteBuf(MAX30102_ADDRESS, writeBuffer, 2, I2C_1_MODE_COMPLETE_XFER);
        
        if (status != I2C_1_MSTR_NO_ERROR) {
            CyDelay(10);
            continue;
        }
        
        uint16_t timeout = 5000;
        while ((I2C_1_MasterStatus() & I2C_1_MSTAT_XFER_INP) && timeout--) {
            CyDelay(1);
        }
        
        if (timeout == 0) {
            I2C_1_MasterClearStatus();
            CyDelay(10);
            continue;
        }
        
        uint8_t masterStatus = I2C_1_MasterStatus();
        if (masterStatus & I2C_1_MSTAT_ERR_MASK) {
            I2C_1_MasterClearStatus();
            CyDelay(10);
            continue;
        }
        
        return 1;
    }
    return 0;
}

// Función para lectura multi-byte
uint8_t readI2CMultipleBytes(uint8_t reg, uint8_t *data, uint8_t len) {
    I2C_1_MasterClearStatus();
    
    for (int retry = 0; retry < 3; retry++) {
        // Fase de escritura
        uint8_t status = I2C_1_MasterWriteBuf(MAX30102_ADDRESS, &reg, 1, I2C_1_MODE_COMPLETE_XFER);
        if (status != I2C_1_MSTR_NO_ERROR) {
            CyDelay(10);
            continue;
        }
        
        uint16_t timeout = 1000;
        while ((I2C_1_MasterStatus() & I2C_1_MSTAT_XFER_INP) && timeout--) CyDelay(1);
        if (!timeout || (I2C_1_MasterStatus() & I2C_1_MSTAT_ERR_MASK)) {
            I2C_1_MasterClearStatus();
            CyDelay(10);
            continue;
        }
        
        // Fase de lectura
        status = I2C_1_MasterReadBuf(MAX30102_ADDRESS, data, len, I2C_1_MODE_COMPLETE_XFER);
        if (status != I2C_1_MSTR_NO_ERROR) {
            CyDelay(10);
            continue;
        }
        
        timeout = 1000;
        while ((I2C_1_MasterStatus() & I2C_1_MSTAT_XFER_INP) && timeout--) CyDelay(1);
        if (!timeout || (I2C_1_MasterStatus() & I2C_1_MSTAT_ERR_MASK)) {
            I2C_1_MasterClearStatus();
            CyDelay(10);
            continue;
        }
        
        return 1;
    }
    return 0;
}

// Función para verificar sensor
uint8_t checkSensor() {
    uint8_t partID;
    
    if (readI2CMultipleBytes(REG_PART_ID, &partID, 1)) {
        if (partID == 0x15) {
            sendString("Sensor detectado correctamente!\r\n");
            return 1;
        }
    }
    sendString("Error: Sensor no detectado\r\n");
    return 0;
}

// Configuración del sensor
uint8_t configureSensor() {
    // Reset
    if (!writeI2C(REG_MODE_CONFIG, 0x40)) {
        sendString("Error en reset\r\n");
        return 0;
    }
    CyDelay(500);
    
    // Limpiar FIFO
    writeI2C(REG_FIFO_WR_PTR, 0x00);
    writeI2C(REG_OVF_COUNTER, 0x00);
    writeI2C(REG_FIFO_RD_PTR, 0x00);
    
    // Configurar FIFO
    if (!writeI2C(REG_FIFO_CONFIG, 0x4F)) {
        sendString("Error en config FIFO\r\n");
        return 0;
    }
    
    // Modo SpO2
    if (!writeI2C(REG_MODE_CONFIG, 0x03)) {
        sendString("Error en modo SpO2\r\n");
        return 0;
    }
    
    // Configuración SpO2
    if (!writeI2C(REG_SPO2_CONFIG, 0x27)) {
        sendString("Error en config SpO2\r\n");
        return 0;
    }
    
    // Corriente LED (ajustado para mejor señal)
    if (!writeI2C(REG_LED1_PA, 0xFF)) {
        sendString("Error en LED rojo\r\n");
        return 0;
    }
    if (!writeI2C(REG_LED2_PA, 0xFF)) {
        sendString("Error en LED IR\r\n");
        return 0;
    }
    
    sendString("Sensor configurado!\r\n");
    return 1;
}

// Obtener muestras disponibles
uint8_t getFIFOSamples() {
    uint8_t writePtr, readPtr;
    
    if (!readI2CMultipleBytes(REG_FIFO_WR_PTR, &writePtr, 1) || 
        !readI2CMultipleBytes(REG_FIFO_RD_PTR, &readPtr, 1)) {
        return 0;
    }
    
    uint8_t samples = (writePtr - readPtr) & 0x1F;
    return samples;
}

// Leer datos del FIFO
uint8_t readFIFOData(uint32_t *redValue, uint32_t *irValue) {
    uint8_t fifoData[6];
    
    if (!readI2CMultipleBytes(REG_FIFO_DATA, fifoData, 6)) {
        sendString("Error lectura FIFO\r\n");
        return 0;
    }
    
    *redValue = ((uint32_t)fifoData[0] << 16) | ((uint32_t)fifoData[1] << 8) | fifoData[2];
    *irValue  = ((uint32_t)fifoData[3] << 16) | ((uint32_t)fifoData[4] << 8) | fifoData[5];
    
    *redValue &= 0x3FFFF;
    *irValue  &= 0x3FFFF;
    
    return 1;
}

// Inicializar SysTick
void initSystemTick() {
    CySysTickStart();
}

// Algoritmo para cálculo de ritmo cardíaco
void calculateHeartRate() {
    static uint32_t lastPeakTime = 0;
    static int peakCount = 0;
    uint32_t minVal = 0xFFFFFFFF;
    uint32_t maxVal = 0;
    
    // Encontrar mínimo y máximo
    for (int i = 0; i < BUFFER_SIZE; i++) {
        if (irBuffer[i] < minVal) minVal = irBuffer[i];
        if (irBuffer[i] > maxVal) maxVal = irBuffer[i];
    }
    
    // Umbral dinámico
    uint32_t threshold = (minVal + maxVal) / 2;
    bool inPeak = false;
    
    // Detectar picos
    for (int i = 1; i < BUFFER_SIZE - 1; i++) {
        if (irBuffer[i] > threshold && 
            irBuffer[i] > irBuffer[i-1] && 
            irBuffer[i] > irBuffer[i+1]) {
            
            if (!inPeak) {
                inPeak = true;
                peakCount++;
                
                if (lastPeakTime != 0) {
                    uint32_t peakInterval = CySysTickGetValue() - lastPeakTime;
                    heartRate = 60000 / peakInterval; // BPM
                }
                lastPeakTime = CySysTickGetValue();
            }
        } else {
            inPeak = false;
        }
    }
}

// SOP22222//////
// Inicialización del buffer SpO2
void spo2_init() {
    spo2Buffer.head = 0;
    spo2Buffer.tail = 0;
    spo2Buffer.count = 0;
}
// Actualizar buffer con nuevas muestras
void spo2_update(uint32_t irValue, uint32_t redValue) {
    spo2Buffer.ir[spo2Buffer.head] = irValue;
    spo2Buffer.red[spo2Buffer.head] = redValue;
    spo2Buffer.head = (spo2Buffer.head + 1) % SPO2_BUFFER_SIZE;
    
    if (spo2Buffer.count < SPO2_BUFFER_SIZE) {
        spo2Buffer.count++;
    } else {
        spo2Buffer.tail = (spo2Buffer.tail + 1) % SPO2_BUFFER_SIZE;
    }
}
// Remover componente DC
void spo2_remove_dc_component() {
    static uint32_t irDC = 0;
    static uint32_t redDC = 0;
    const float alpha = 0.95;
    
    for (int i = 0; i < spo2Buffer.count; i++) {
        uint8_t idx = (spo2Buffer.tail + i) % SPO2_BUFFER_SIZE;
        
        irDC = alpha * irDC + (1 - alpha) * spo2Buffer.ir[idx];
        redDC = alpha * redDC + (1 - alpha) * spo2Buffer.red[idx];
        
        spo2Buffer.ir[idx] -= irDC;
        spo2Buffer.red[idx] -= redDC;
    }
}
// Calcular RMS de la componente AC
float spo2_compute_ac_rms(uint32_t* signal) {
    float sum = 0.0;
    
    for (int i = 0; i < spo2Buffer.count; i++) {
        uint8_t idx = (spo2Buffer.tail + i) % SPO2_BUFFER_SIZE;
        float value = (float)signal[idx];
        sum += value * value;
    }
    
    return mySqrt(sum / spo2Buffer.count);
}

// Calcular relación R
float spo2_compute_r_ratio() {
    float irAC = spo2_compute_ac_rms(spo2Buffer.ir);
    float redAC = spo2_compute_ac_rms(spo2Buffer.red);
    
    float irDC = 0;
    float redDC = 0;
    
    for (int i = 0; i < spo2Buffer.count; i++) {
        uint8_t idx = (spo2Buffer.tail + i) % SPO2_BUFFER_SIZE;
        irDC += spo2Buffer.ir[idx];
        redDC += spo2Buffer.red[idx];
    }
    
    irDC /= spo2Buffer.count;
    redDC /= spo2Buffer.count;
    
    return (redAC / redDC) / (irAC / irDC);
}



// Algoritmo principal para calcular SpO2 (CORREGIDO)
void spo2_calculate() {
    // Paso 1: Filtrar componentes DC
    spo2_remove_dc_component();
    
    // Paso 2: Calcular relación R
    float r_ratio = spo2_compute_r_ratio();
    
    // Paso 3: Calcular SpO2 usando curva de calibración
    float spo2_value = 110.0 - 25.0 * r_ratio;
    
    // Validar y ajustar valores
    if (spo2_value > MAX_SPO2) spo2_value = MAX_SPO2;
    if (spo2_value < MIN_SPO2) spo2_value = MIN_SPO2;
    
    spO2 = (int32_t)spo2_value;
    validSPO2 = 1;  // Ahora la variable está declarada
}



// Estimación de presión arterial (basado en PTT)
void estimateBloodPressure() {
    // Encuentra el pico en IR (sístole)
    int systolicIndex = 0;
    uint32_t maxIR = 0;
    for (int i = 0; i < BUFFER_SIZE; i++) {
        if (irBuffer[i] > maxIR) {
            maxIR = irBuffer[i];
            systolicIndex = i;
        }
    }
    
    // Encuentra el valle (diástole)
    int diastolicIndex = 0;
    uint32_t minIR = 0xFFFFFFFF;
    for (int i = systolicIndex; i < BUFFER_SIZE; i++) {
        if (irBuffer[i] < minIR) {
            minIR = irBuffer[i];
            diastolicIndex = i;
        }
    }
    
    // Calcular Pulse Transit Time (PTT)
    float PTT = (diastolicIndex - systolicIndex) * (1000.0 / SAMPLE_RATE); // ms
    
    // Modelo empírico
    systolicBP = 120 - (int)(PTT * 0.5);
    diastolicBP = 80 - (int)(PTT * 0.3);
    
    // Limitar valores
    if (systolicBP < 80) systolicBP = 80;
    if (systolicBP > 180) systolicBP = 180;
    if (diastolicBP < 50) diastolicBP = 50;
    if (diastolicBP > 120) diastolicBP = 120;
}

// Actualizar buffer de muestras
void updateBuffers(uint32_t red, uint32_t ir) {
    static uint32_t irDC = 0;
    static uint32_t redDC = 0;
    
    // Filtrado DC
    irDC = DC_FILTER_ALPHA * irDC + (1 - DC_FILTER_ALPHA) * ir;
    redDC = DC_FILTER_ALPHA * redDC + (1 - DC_FILTER_ALPHA) * red;
    
    // Almacenar valores AC
    irBuffer[bufferIndex] = ir - irDC;
    redBuffer[bufferIndex] = red - redDC;
    
    // Actualizar buffer SpO2
    spo2_update(ir, red);
    
    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
    
    // Detección de dedo
    fingerDetected = (ir > IR_THRESHOLD);
}

// Función principal
int main(void)
{
    CyGlobalIntEnable; // Habilitar interrupciones globales

    // --- Variables para flujo (las puedes tener globales también si prefieres) ---
    char uartBuffer[128];
    uint32 debugCounter = 0;
    uint32 lastPulseCount = 0;
    uint32 timeCounter = 0;
    
    uint16_t loopCounter = 0;  // Contador para periodicidad de 1 segundo

    // --- Inicializar sensores de flujo ---
    UART_1_Start();
    Timer_Start();
    FlowSensor_ISR_StartEx(FlowSensor_ISR);
    Timer_ISR_StartEx(Timer_ISR);

    // --- Mensajes iniciales de flujo ---
    sprintf(uartBuffer, "Espirometro con sensor YF-S401 iniciado\r\n");
    UART_1_PutString(uartBuffer);
    sprintf(uartBuffer, "Configuracion: 115200 baud, 8N1, paridad par\r\n");
    UART_1_PutString(uartBuffer);
    sprintf(uartBuffer, "Iniciando mediciones... Estado pin: %d\r\n", FlowSensor_Pin_Read());
    UART_1_PutString(uartBuffer);

    // --- Inicializar sensores MAX30102 ---
    I2C_1_Start();
    initSystemTick();
    spo2_init();  // Inicializar buffer SpO2
    CyDelay(2000);
    sendString("\r\n=== Sistema de Monitorizacion de Salud ===\r\n");

    if (!checkSensor()) {
        sendString("Error: Sensor MAX30102 no detectado. Verifique conexiones.\r\n");
        while(1);
    }
    if (!configureSensor()) {
        sendString("Error: Configuracion MAX30102 fallida. Reinicie el sistema.\r\n");
        while(1);
    }
    sendString("Sensor MAX30102 configurado. Coloque el dedo...\r\n");

    // --- Main loop ---
   for(;;)
    {
        // ---------- FLUJO (YF-S401) ----------
        if(flowMeasurementReady)
        {
            CyGlobalIntDisable;
            uint32 currentPulseCount = pulseCount;
            pulseCount = 0;
            flowMeasurementReady = 0;
            CyGlobalIntEnable;

            // Calcular y almacenar flujo (sin imprimir)
            lastFlowRate_x100 = (currentPulseCount * 85) / 75; // x100 para 2 decimales
        }

        // ---------- MAX30102 (HR, BP, SpO2) ----------
        uint32_t red, ir;
        uint8_t samples = getFIFOSamples();

        if (samples > 0 && readFIFOData(&red, &ir)) {
            updateBuffers(red, ir);

            if (fingerDetected) {
                // Procesar cada 100 muestras (~1 segundo)
                if (bufferIndex % 100 == 0) {
                    calculateHeartRate();
                    estimateBloodPressure();
                    
                    // Almacenar valores
                    lastHR = heartRate;
                    lastSystolic = systolicBP;
                    lastDiastolic = diastolicBP;
                }

                // Calcular SpO2 cada 25 muestras
                if (spo2Buffer.count >= 25) {
                    spo2_calculate();
                    static uint8_t spo2Counter = 0;
                    if (++spo2Counter >= 4) {  // Reportar cada ~4 segundos
                        spo2Counter = 0;
                        lastSpO2 = (int)spO2;
                    }
                }
            } 
            else {
                // Resetear valores si no hay dedo
                lastHR = 0;
                lastSystolic = 0;
                lastDiastolic = 0;
                lastSpO2 = 0;
                
                if (bufferIndex % 50 == 0) {
                    sendString("Coloque el dedo en el sensor...\r\n");
                }
            }
        }

        // ---------- Envío unificado cada 1 segundo ----------
        loopCounter++;
        if (loopCounter >= 100) {  // 100 * 10ms = 1 segundo
            loopCounter = 0;
            
            if (fingerDetected) {
                sprintf(uartBuffer, "DATA: HR=%d | BP=%d/%d | SpO2=%d%% | Flujo=%lu.%02lu L/min\r\n",
                        lastHR, 
                        lastSystolic, 
                        lastDiastolic, 
                        lastSpO2,
                        lastFlowRate_x100 / 100, 
                        lastFlowRate_x100 % 100);
            } else {
                sprintf(uartBuffer, "DATA: HR=-- | BP=--/-- | SpO2=--%% | Flujo=%lu.%02lu L/min\r\n",
                        lastFlowRate_x100 / 100, 
                        lastFlowRate_x100 % 100);
            }
            UART_1_PutString(uartBuffer);
        }

        CyDelay(10);  // Espera 10ms (controla frecuencia del loop)
    }
}