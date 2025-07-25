// main.c - Sistema unificado corregido: Sensor de flujo YF-S401 + MAX30102 para PSoC 5LP
// Configuración UART: 115200 baud, 8 bits, paridad par, 1 bit de parada
// CONFIGURACIÓN DEL PIN DEL SENSOR DE FLUJO:
// - Drive Mode: Resistive pull up
// - Threshold: CMOS
// - Interrupt: Rising edge

#include "project.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

// ===== CONFIGURACIÓN DEL MAX30102 =====
#define MAX30102_ADDRESS    0x57

// Registros principales
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

// Configuración
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

// ===== CONFIGURACIÓN DEL SENSOR DE FLUJO YF-S401 =====
// El YF-S401 genera 5.5 pulsos por mL según datasheet
// Flujo (L/min) = (Pulsos por segundo * 60) / (5.5 * 1000)
#define FLOW_PULSES_PER_ML      5.5f
#define FLOW_MEASUREMENT_TIME   1000    // 1 segundo en ms

// ===== VARIABLES GLOBALES =====
// Buffer UART
char uartBuffer[128];

// Variables para sensor de flujo (corregidas)
volatile uint32 flowPulseCount = 0;
volatile uint8 flowMeasurementReady = 0;
uint32 lastFlowRate_x100 = 0;  // Flujo actual en formato x100

// Estructura SpO2
typedef struct {
    uint32_t ir[SPO2_BUFFER_SIZE];
    uint32_t red[SPO2_BUFFER_SIZE];
    uint8_t count;
    uint8_t head;
    uint8_t tail;
} SpO2Buffer;

// Variables para MAX30102
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

// Variables para almacenar últimos valores válidos
int lastHR = 0;
int lastSystolic = 0;
int lastDiastolic = 0;
int lastSpO2 = 0;

// ===== INTERRUPCIONES =====
CY_ISR(FlowSensor_ISR) {
    flowPulseCount++;
    FlowSensor_Pin_ClearInterrupt();
}

CY_ISR(Timer_ISR) {
    flowMeasurementReady = 1;
    Timer_ReadStatusRegister();  // Limpiar bandera de interrupción
}

// ===== PROTOTIPOS DE FUNCIONES =====
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
void processFlowSensor();
void sendUnifiedResults();
void testFlowSensor();

// ===== IMPLEMENTACIÓN DE FUNCIONES MATEMÁTICAS =====
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

// ===== FUNCIONES PARA COMUNICACIÓN =====
void sendString(const char* str) {
    while (*str) {
        UART_1_PutChar(*str);
        str++;
    }
}

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

// ===== FUNCIONES PARA MAX30102 =====
uint8_t checkSensor() {
    uint8_t partID;
    
    if (readI2CMultipleBytes(REG_PART_ID, &partID, 1)) {
        if (partID == 0x15) {
            sendString("MAX30102 detectado correctamente!\r\n");
            return 1;
        }
    }
    sendString("Error: MAX30102 no detectado\r\n");
    return 0;
}

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
    
    // Corriente LED
    if (!writeI2C(REG_LED1_PA, 0xFF)) {
        sendString("Error en LED rojo\r\n");
        return 0;
    }
    if (!writeI2C(REG_LED2_PA, 0xFF)) {
        sendString("Error en LED IR\r\n");
        return 0;
    }
    
    sendString("MAX30102 configurado!\r\n");
    return 1;
}

uint8_t getFIFOSamples() {
    uint8_t writePtr, readPtr;
    
    if (!readI2CMultipleBytes(REG_FIFO_WR_PTR, &writePtr, 1) || 
        !readI2CMultipleBytes(REG_FIFO_RD_PTR, &readPtr, 1)) {
        return 0;
    }
    
    uint8_t samples = (writePtr - readPtr) & 0x1F;
    return samples;
}

uint8_t readFIFOData(uint32_t *redValue, uint32_t *irValue) {
    uint8_t fifoData[6];
    
    if (!readI2CMultipleBytes(REG_FIFO_DATA, fifoData, 6)) {
        return 0;
    }
    
    *redValue = ((uint32_t)fifoData[0] << 16) | ((uint32_t)fifoData[1] << 8) | fifoData[2];
    *irValue  = ((uint32_t)fifoData[3] << 16) | ((uint32_t)fifoData[4] << 8) | fifoData[5];
    
    *redValue &= 0x3FFFF;
    *irValue  &= 0x3FFFF;
    
    return 1;
}

void initSystemTick() {
    CySysTickStart();
}

void calculateHeartRate() {
    static uint32_t lastPeakTime = 0;
    static uint32_t peakTimes[10];
    static uint8_t peakIndex = 0;
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
            irBuffer[i] > irBuffer[i+1] && 
            !inPeak) {
            
            inPeak = true;
            uint32_t currentTime = CySysTickGetValue();
            
            if (lastPeakTime != 0) {
                uint32_t interval = currentTime - lastPeakTime;
                if (interval > 0) {
                    peakTimes[peakIndex] = interval;
                    peakIndex = (peakIndex + 1) % 10;
                    
                    // Calcular promedio de intervalos
                    uint32_t avgInterval = 0;
                    for (int j = 0; j < 10; j++) {
                        avgInterval += peakTimes[j];
                    }
                    avgInterval /= 10;
                    
                    if (avgInterval > 0) {
                        heartRate = 60000 / avgInterval; // BPM
                        
                        // Validar rango
                        if (heartRate < MIN_HR || heartRate > MAX_HR) {
                            heartRate = 0;
                        }
                    }
                }
            }
            lastPeakTime = currentTime;
        } else if (irBuffer[i] < threshold) {
            inPeak = false;
        }
    }
}

// ===== FUNCIONES PARA SPO2 =====
void spo2_init() {
    spo2Buffer.head = 0;
    spo2Buffer.tail = 0;
    spo2Buffer.count = 0;
}

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

void spo2_remove_dc_component() {
    static uint32_t irDC = 0;
    static uint32_t redDC = 0;
    const float alpha = 0.95;
    
    for (int i = 0; i < spo2Buffer.count; i++) {
        uint8_t idx = (spo2Buffer.tail + i) % SPO2_BUFFER_SIZE;
        
        irDC = alpha * irDC + (1 - alpha) * spo2Buffer.ir[idx];
        redDC = alpha * redDC + (1 - alpha) * spo2Buffer.red[idx];
        
        spo2Buffer.ir[idx] = (spo2Buffer.ir[idx] > irDC) ? spo2Buffer.ir[idx] - irDC : 0;
        spo2Buffer.red[idx] = (spo2Buffer.red[idx] > redDC) ? spo2Buffer.red[idx] - redDC : 0;
    }
}

float spo2_compute_ac_rms(uint32_t* signal) {
    float sum = 0.0;
    
    for (int i = 0; i < spo2Buffer.count; i++) {
        uint8_t idx = (spo2Buffer.tail + i) % SPO2_BUFFER_SIZE;
        float value = (float)signal[idx];
        sum += value * value;
    }
    
    return mySqrt(sum / spo2Buffer.count);
}

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
    
    if (irDC == 0 || redDC == 0) return 0;
    
    return (redAC / redDC) / (irAC / irDC);
}

void spo2_calculate() {
    if (spo2Buffer.count < 25) return;
    
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
    validSPO2 = 1;
}

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

void updateBuffers(uint32_t red, uint32_t ir) {
    static uint32_t irDC = 0;
    static uint32_t redDC = 0;
    
    // Filtrado DC
    irDC = DC_FILTER_ALPHA * irDC + (1 - DC_FILTER_ALPHA) * ir;
    redDC = DC_FILTER_ALPHA * redDC + (1 - DC_FILTER_ALPHA) * red;
    
    // Almacenar valores AC
    irBuffer[bufferIndex] = (ir > irDC) ? ir - irDC : 0;
    redBuffer[bufferIndex] = (red > redDC) ? red - redDC : 0;
    
    // Actualizar buffer SpO2
    spo2_update(ir, red);
    
    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
    
    // Detección de dedo
    fingerDetected = (ir > IR_THRESHOLD);
}

// ===== FUNCIONES PARA SENSOR DE FLUJO =====
void processFlowSensor() {
    if (flowMeasurementReady) {
        // Deshabilitar interrupciones temporalmente
        CyGlobalIntDisable;
        uint32 currentPulseCount = flowPulseCount;
        flowPulseCount = 0;
        flowMeasurementReady = 0;
        CyGlobalIntEnable;
        
        // Fórmula corregida
        lastFlowRate_x100 = (currentPulseCount * 12) / 11;
        
        // Debug: enviar información sobre pulsos
        sprintf(uartBuffer, "FLUJO: %u pulsos -> %u.%02u L/min\r\n",
                (unsigned int)currentPulseCount,
                (unsigned int)(lastFlowRate_x100/100),
                (unsigned int)(lastFlowRate_x100%100));
        UART_1_PutString(uartBuffer);
    }
}

// ===== FUNCIÓN PARA PROBAR EL SENSOR DE FLUJO =====
void testFlowSensor() {
    sendString("=== Prueba del Sensor de Flujo ===\r\n");
    sendString("Sople suavemente en el sensor...\r\n");
    
    // Resetear contador
    CyGlobalIntDisable;
    flowPulseCount = 0;
    CyGlobalIntEnable;
    
    uint32_t lastCount = 0;
    for (int i = 0; i < 10; i++) {
        CyDelay(1000);
        CyGlobalIntDisable;
        uint32_t currentCount = flowPulseCount;
        uint32_t pulsesPerSec = currentCount - lastCount;
        lastCount = currentCount;
        CyGlobalIntEnable;
        
        sprintf(uartBuffer, "[%ds] Pulsos totales=%lu, Pulsos/s=%lu\r\n", i+1, currentCount, pulsesPerSec);


        UART_1_PutString(uartBuffer);
    }
    
    sendString("Fin de la prueba.\r\n");
}

void sendUnifiedResults() {
    if (fingerDetected) {
        sprintf(uartBuffer, "DATA: HR=%d | BP=%d/%d | SpO2=%d%% | Flujo=%u.%02u L/min\r\n",
                lastHR, 
                lastSystolic, 
                lastDiastolic, 
                lastSpO2,
                (unsigned int)(lastFlowRate_x100/100),
                (unsigned int)(lastFlowRate_x100%100));
    } else {
        sprintf(uartBuffer, "DATA: HR=-- | BP=--/-- | SpO2=--%% | Flujo=%u.%02u L/min\r\n",
                (unsigned int)(lastFlowRate_x100/100),
                (unsigned int)(lastFlowRate_x100%100));
    }
    UART_1_PutString(uartBuffer);
}

// ===== FUNCIÓN PRINCIPAL =====
int main(void)
{
    CyGlobalIntEnable; // Habilitar interrupciones globales
    
    // Inicializar componentes
    UART_1_Start();
    I2C_1_Start();
   // INICIALIZACIÓN CRÍTICA DEL TIMER (CORREGIDO)
    Timer_Start();  // Iniciar componente hardware
    Timer_WritePeriod(9999);  // Configurar periodo para 1 segundo (24MHz/24=1MHz → 1000000 ticks/s)
    Timer_Start();  // Reiniciar timer
    
    initSystemTick();
    spo2_init();
    
    // Configurar interrupciones
    FlowSensor_ISR_StartEx(FlowSensor_ISR);
    Timer_ISR_StartEx(Timer_ISR);
    
    // Habilitar interrupciones del timer (CORREGIDO)
    Timer_Enable();
    
    CyDelay(2000);
    
    // Mensajes iniciales
    sendString("=== Sistema Unificado de Monitoreo ===\r\n");
    sendString("Sensor de Flujo YF-S401 + MAX30102\r\n");
    sendString("Configuracion: 115200 baud, 8N1, paridad par\r\n");
    sendString("Config Pin Flujo: Resistive pull up, Rising edge\r\n");
    
    // Prueba del sensor de flujo
    testFlowSensor();  // Prueba inicial de 10 segundos
    
    // Verificar y configurar MAX30102
    uint8_t sensorOK = 0;
    if (checkSensor()) {
        if (configureSensor()) {
            sendString("MAX30102 configurado correctamente.\r\n");
            sensorOK = 1;
        } else {
            sendString("Error: Configuracion MAX30102 fallida.\r\n");
        }
    } else {
        sendString("Error: MAX30102 no detectado. Continuando solo con sensor de flujo.\r\n");
    }
    
    sendString("Sistema iniciado.\r\n");
    if (sensorOK) {
        sendString("Coloque el dedo en el sensor MAX30102...\r\n");
    }
    sendString("Sensor de flujo YF-S401 activo.\r\n");
    
    // Variables para control de tiempo
    static uint32_t lastResultTime = 0;
    static uint32_t sampleCounter = 0;
    
    for(;;)
    {
        // Procesar sensor de flujo
        processFlowSensor();
        
        // Procesar MAX30102 solo si está disponible
        if (sensorOK) {
            uint32_t red, ir;
            uint8_t samples = getFIFOSamples();
            
            if (samples > 0) {
                if (readFIFOData(&red, &ir)) {
                    updateBuffers(red, ir);
                    sampleCounter++;
                    
                    if (fingerDetected) {
                        // Procesar cada 100 muestras (1 segundo a 100Hz)
                        if (sampleCounter >= 100) {
                            calculateHeartRate();
                            estimateBloodPressure();
                            
                            // Almacenar valores válidos
                            if (heartRate > 0) lastHR = heartRate;
                            if (systolicBP > 0) lastSystolic = systolicBP;
                            if (diastolicBP > 0) lastDiastolic = diastolicBP;
                            
                            sampleCounter = 0;
                        }
                        
                        // Calcular SpO2 cuando hay suficientes muestras
                        if (spo2Buffer.count >= 25) {
                            spo2_calculate();
                            if (validSPO2 && spO2 > 0) {
                                lastSpO2 = (int)spO2;
                            }
                        }
                    } else {
                        // Resetear solo si no hay dedo por un tiempo
                        if (sampleCounter > 200) { // 2 segundos sin dedo
                            lastHR = 0;
                            lastSystolic = 0;
                            lastDiastolic = 0;
                            lastSpO2 = 0;
                            sampleCounter = 0;
                        }
                    }
                }
            }
        }
        
        // Enviar resultados cada segundo
        uint32_t currentTime = CySysTickGetValue();
        if (currentTime - lastResultTime >= 1000) { // 1 segundo
            sendUnifiedResults();
            lastResultTime = currentTime;
        }
        
        CyDelay(10); // Pequeño delay para no saturar el sistema
    }
}
//[file content end]