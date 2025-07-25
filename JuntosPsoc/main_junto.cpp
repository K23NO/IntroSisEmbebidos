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
#define FLOW_DEBOUNCE_TIME      5       // 5ms de debounce para evitar pulsos falsos

// ===== VARIABLES GLOBALES =====
// Buffer UART
char uartBuffer[128];

// Variables para sensor de flujo (corregidas)
volatile uint32 flowPulseCount = 0;
volatile uint8 flowMeasurementReady = 0;
volatile uint32 lastFlowRate_x100 = 0;  // Flujo actual en formato x100
volatile uint32 lastValidFlowRate_x100 = 0;  // Último flujo válido (no cero)
volatile uint32 totalVolume_mL = 0;  // Volumen total acumulado en mL
volatile uint32 lastPulseTime = 0;    // Tiempo del último pulso para debounce

// ===== VARIABLES GLOBALES MODIFICADAS =====
uint32 lastFlowTime = 0;  // Tiempo de la última medición
#define SYSTICK_FREQ 24000000u // Frecuencia del SysTick (24 MHz)


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
    // Verificar que realmente hay un flanco ascendente
    if (FlowSensor_Pin_Read() == 1) {
        uint32_t currentTime = CySysTickGetValue();
        
        // Debounce - Ignorar pulsos demasiado cercanos
        if ((currentTime - lastPulseTime) > FLOW_DEBOUNCE_TIME) {
            flowPulseCount++;
            
            // Cálculo de volumen: cada pulso representa aproximadamente 1/5.5 mL
            // Convertir a formato entero (mL x 1000000) para precisión
            totalVolume_mL += (uint32_t)(1000000.0f / FLOW_PULSES_PER_ML);
            
            // Actualizar tiempo del último pulso
            lastPulseTime = currentTime;
            
            // Debugging - enviar punto para ver pulsos en tiempo real
            // UART_1_PutChar('.');
        }
    }
    FlowSensor_Pin_ClearInterrupt();
}

CY_ISR(Timer_ISR) {
    static uint16_t counter = 0;
    static uint32_t lastPulseCount = 0;
    static uint32_t pulseHistogram[5] = {0};  // Historial de pulsos para los últimos 500ms
    static uint8_t histIndex = 0;
    
    // Actualizar contador para medición de 1 segundo
    counter++;
    
    // Cada 100ms, actualizar el flujo de corto plazo
    if (counter % 100 == 0) {
        // Capturar pulsos de forma segura
        uint8_t intStatus = CyEnterCriticalSection();
        uint32_t currentPulseCount = flowPulseCount;
        uint32_t pulseDiff = currentPulseCount - lastPulseCount;
        lastPulseCount = currentPulseCount;
        CyExitCriticalSection(intStatus);
        
        // Actualizar histograma de pulsos (5 periodos de 100ms = 500ms)
        pulseHistogram[histIndex] = pulseDiff;
        histIndex = (histIndex + 1) % 5;
        
        // Sumar todos los pulsos en el histograma (últimos 500ms)
        uint32_t totalPulses = 0;
        for (int i = 0; i < 5; i++) {
            totalPulses += pulseHistogram[i];
        }
        
        // Calcular flujo en L/min
        // Formula: (Pulsos en 0.5 segundos * 120) / (Pulsos por mL * 1000)
        // (multiplicamos por 120 porque: 0.5s -> 60s = 120x)
        float flowRate = (totalPulses * 120.0f) / (FLOW_PULSES_PER_ML * 1000.0f);
        
        // Convertir a formato x100 y almacenar
        uint32_t newFlowRate_x100 = (uint32_t)(flowRate * 100.0f + 0.5f);
        
        // Actualizar variables globales con protección
        intStatus = CyEnterCriticalSection();
        lastFlowRate_x100 = newFlowRate_x100;
        if (newFlowRate_x100 > 0) {
            lastValidFlowRate_x100 = newFlowRate_x100;
        }
        CyExitCriticalSection(intStatus);
    }
    
    // Cada segundo, activar bandera para el procesamiento completo
    if (counter >= 1000) { // 1000 * 1ms = 1 segundo
        flowMeasurementReady = 1;
        counter = 0;
    }
    
    Timer_ReadStatusRegister();
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
void resetFlowMeasurement();

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
    // Configurar SysTick para funcionar con el clock del sistema
    CySysTickStart();
    // No necesitamos llamar a CySysTickInit() ya que CySysTickStart() ya lo hace
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
void resetFlowMeasurement() {
    CyGlobalIntDisable;
    flowPulseCount = 0;
    totalVolume_mL = 0;
    lastFlowRate_x100 = 0;
    lastValidFlowRate_x100 = 0;
    lastPulseTime = 0;
    CyGlobalIntEnable;
}

void processFlowSensor() {
    // Solo procesar cuando la bandera está activa (cada segundo)
    if (flowMeasurementReady) {
        flowMeasurementReady = 0;  // Resetear bandera
        
        // Capturar datos actuales de forma segura
        uint8_t intStatus = CyEnterCriticalSection();
        uint32_t pulsesCaptured = flowPulseCount;
        uint32_t flowRate = lastFlowRate_x100;
        uint32_t validFlowRate = lastValidFlowRate_x100;
        uint32_t volume = totalVolume_mL;
        CyExitCriticalSection(intStatus);
        
        // Si no hay flujo actual pero hay un valor válido previo, usarlo para mostrar
        if (flowRate == 0 && validFlowRate > 0) {
            flowRate = validFlowRate;
        }
        
        // Extraer partes enteras y decimales para formateo seguro
        uint32_t flowWhole = flowRate / 100;
        uint32_t flowFrac = flowRate % 100;
        
        uint32_t volWhole = volume / 1000000;
        uint32_t volFrac = ((volume % 1000000) * 1000) / 1000000; // 3 decimales
        
        // Mostrar información completa con formato seguro (usando %u para uint32_t)
        sprintf(uartBuffer, "FLUJO: Pulsos=%u | Tasa=%u.%02u L/min | Vol.Total=%u.%03u L\r\n",
                (unsigned int)pulsesCaptured, 
                (unsigned int)flowWhole, (unsigned int)flowFrac,
                (unsigned int)volWhole, (unsigned int)volFrac);
        UART_1_PutString(uartBuffer);
        
        // Validar que tenemos datos correctos
        if (flowRate == 0 && pulsesCaptured > 0) {
            sprintf(uartBuffer, "[ALERTA] Pulsos detectados pero flujo=0! Verifica cálculos.\r\n");
            UART_1_PutString(uartBuffer);
        }
    }
}


// ===== FUNCIÓN PARA PROBAR EL SENSOR DE FLUJO =====
void testFlowSensor() {
    sendString("\r\n=== Prueba del Sensor de Flujo ===\r\n");
    sendString("Sople suavemente en el sensor durante 10 segundos...\r\n");
    
    // Resetear todas las variables de medición
    resetFlowMeasurement();
    
    // Información sobre la configuración
    sprintf(uartBuffer, "Configuración: %.1f pulsos por mL, 1 pulso = %.4f mL\r\n", 
            FLOW_PULSES_PER_ML, 1.0f/FLOW_PULSES_PER_ML);
    UART_1_PutString(uartBuffer);
    
    uint32_t lastCount = 0;
    uint32_t lastVolume = 0;
    
    // Probar durante 10 segundos con actualizaciones cada segundo
    for (int i = 0; i < 10; i++) {
        // Esperar 1 segundo mientras verificamos pulsos cada 200ms
        for (int j = 0; j < 5; j++) {
            CyDelay(200); // 5 intervalos de 200ms = 1 segundo
            
            // Mostrar pulso en tiempo real para feedback visual
            UART_1_PutString(".");
        }
        UART_1_PutString("\r\n");
        
        // Capturar estado actual sin resetear
        uint8_t intStatus = CyEnterCriticalSection();
        uint32_t currentPulseCount = flowPulseCount;
        uint32_t currentVolume = totalVolume_mL;
        CyExitCriticalSection(intStatus);
        
        // Calcular pulsos por segundo
        uint32_t pulsesPerSec = currentPulseCount - lastCount;
        
        // Calcular flujo manual para comparar (para validación)
        float manualFlowRate = (pulsesPerSec * 60.0f) / (FLOW_PULSES_PER_ML * 1000.0f);
        
        // Actualizar el flujo actual - necesario para que se muestre en tiempo real
        uint32_t flowRate_x100 = (uint32_t)(manualFlowRate * 100.0f + 0.5f);
        
        // Actualizar variables globales para que sean visibles fuera de esta función
        intStatus = CyEnterCriticalSection();
        lastFlowRate_x100 = flowRate_x100;
        // Si tenemos pulsos, actualizar último valor válido
        if (pulsesPerSec > 0) {
            lastValidFlowRate_x100 = flowRate_x100;
        }
        CyExitCriticalSection(intStatus);
        
        // Calcular volumen incremental en litros
        float volumeIncrement = (currentVolume - lastVolume) / 1000000.0f;
        
        // Extraer partes enteras y decimales para formateo seguro
        uint32_t flowWhole = flowRate_x100 / 100;
        uint32_t flowFrac = flowRate_x100 % 100;
        
        uint32_t manualWhole = (uint32_t)(manualFlowRate);
        uint32_t manualFrac = (uint32_t)((manualFlowRate - manualWhole) * 100 + 0.5f);
        
        uint32_t volWhole = currentVolume / 1000000;
        uint32_t volFrac = ((currentVolume % 1000000) * 10000) / 1000000; // 4 decimales
        
        uint32_t volIncWhole = (uint32_t)volumeIncrement;
        uint32_t volIncFrac = (uint32_t)((volumeIncrement - volIncWhole) * 10000 + 0.5f);
        
        // Mostrar estado completo con formato seguro (usando %u para uint32_t)
        sprintf(uartBuffer, "[%ds] Pulsos=%u (+%u/s) | Flujo=%u.%02u L/min | Calc.Man=%u.%02u L/min | Vol=%u.%04u L (+%u.%04u)\r\n", 
                i+1, 
                (unsigned int)currentPulseCount,
                (unsigned int)pulsesPerSec,
                (unsigned int)flowWhole, (unsigned int)flowFrac,
                (unsigned int)manualWhole, (unsigned int)manualFrac,
                (unsigned int)volWhole, (unsigned int)volFrac,
                (unsigned int)volIncWhole, (unsigned int)volIncFrac);
        UART_1_PutString(uartBuffer);
        
        // Actualizar contadores para siguiente iteración
        lastCount = currentPulseCount;
        lastVolume = currentVolume;
    }
    
    sendString("Fin de la prueba. Iniciando monitoreo continuo...\r\n\r\n");
}

void sendUnifiedResults() {
    // Capturar valores actuales de forma segura
    uint8_t intStatus = CyEnterCriticalSection();
    uint32_t flowToShow = lastFlowRate_x100;
    uint32_t volumeToShow = totalVolume_mL;
    uint32_t validFlowRate = lastValidFlowRate_x100;
    CyExitCriticalSection(intStatus);
    
    // Si no hay flujo actual pero hay un valor válido previo, usarlo
    if (flowToShow == 0 && validFlowRate > 0) {
        flowToShow = validFlowRate;
    }
    
    // Extraer la parte entera y decimal del flujo
    uint32_t flowWhole = flowToShow / 100;
    uint32_t flowFrac = flowToShow % 100;
    
    // Extraer la parte entera y decimal del volumen (convertido a L)
    float volumeL = volumeToShow / 1000000.0f;  // Convertir a litros
    uint32_t volWhole = (uint32_t)volumeL;
    uint32_t volFrac = (uint32_t)((volumeL - volWhole) * 1000);
    
    // Formatear la salida usando partes enteras y decimales directamente
    if (fingerDetected) {
        sprintf(uartBuffer, "\r\nDATA: HR=%d | BP=%d/%d | SpO2=%d%% | Flujo=%u.%02u L/min | Vol=%u.%03u L\r\n",
                lastHR, 
                lastSystolic, 
                lastDiastolic, 
                lastSpO2,
                (unsigned int)flowWhole, (unsigned int)flowFrac,
                (unsigned int)volWhole, (unsigned int)volFrac);
    } else {
        sprintf(uartBuffer, "\r\nDATA: HR=-- | BP=--/-- | SpO2=--%% | Flujo=%u.%02u L/min | Vol=%u.%03u L\r\n",
                (unsigned int)flowWhole, (unsigned int)flowFrac,
                (unsigned int)volWhole, (unsigned int)volFrac);
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
    
    // Iniciar el System Tick para medición de tiempo
    initSystemTick();
    
    // INICIALIZACIÓN CRÍTICA DEL TIMER (CORREGIDO)
    Timer_Stop();
    Timer_WritePeriod(23999);  // Para 24MHz: 24,000,000 / 24 = 1MHz → 1000 ticks/ms
    Timer_Start();
    
    // Inicializar otros componentes
    spo2_init();
    
    // Inicializar variables
    lastFlowRate_x100 = 0;
    lastValidFlowRate_x100 = 0;  // Inicializar variable de flujo válido
    totalVolume_mL = 0;  // Inicializar volumen acumulado
    
    // Configurar interrupciones
    FlowSensor_ISR_StartEx(FlowSensor_ISR);
    Timer_ISR_StartEx(Timer_ISR);
    
    // Habilitar interrupciones del timer (CORREGIDO)
    Timer_Enable();
    
    CyDelay(2000);
    
    // Mensajes iniciales
    sendString("\r\n=== Sistema Unificado de Monitoreo ===\r\n");
    sendString("Sensor de Flujo YF-S401 + MAX30102\r\n");
    
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
    
    // Variables para control de tiempo
    uint32_t lastResultTime = 0;
    uint32_t sampleCounter = 0;
    
    // Inicializar valores predeterminados para mostrar algo al inicio
    lastHR = 0;
    lastSystolic = 80;
    lastDiastolic = 50;
    lastSpO2 = 0;
    
    for(;;)
    {
        // Procesar sensor de flujo (por interrupción del timer cada 1 segundo)
        processFlowSensor();
        
        // Verificar si hay nuevos datos del sensor de flujo (actualización en tiempo real)
        // El timer ISR maneja la actualización de lastFlowRate_x100 cada 100ms
        
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
                    }
                }
            }
        }
        
        // Enviar resultados cada 5 segundos para evitar saturación
        uint32_t currentTime = CySysTickGetValue();
        if ((currentTime - lastResultTime) >= 5000) { // 5 segundos
            sendUnifiedResults();
            lastResultTime = currentTime;
        }
        
        CyDelay(10); // Pequeño delay para no saturar el sistema
    }
}
//[file content end]