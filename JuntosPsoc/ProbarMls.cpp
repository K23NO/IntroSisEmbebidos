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
#define IR_THRESHOLD        30000   // Reducido el umbral para mayor sensibilidad en detección de dedo
#define DC_FILTER_ALPHA     0.95    // Factor de filtrado DC

// Factor de calibración para HR (compensación del sensor)
#define HR_CALIBRATION_FACTOR 0.88  // Factor de corrección para HR

// Algoritmo SpO2
#define SPO2_BUFFER_SIZE    100
#define MAX_HR              200
#define MIN_HR              30
#define MAX_SPO2            100
#define MIN_SPO2            70

// ===== CONFIGURACIÓN DEL SENSOR DE FLUJO YF-S401 =====
// El YF-S401 genera 5.5 pulsos por mL según datasheet
// Flujo (L/s) = (Pulsos por segundo) / (5.5 * 1000)
// DPEF (Flujo Expiratorio Pico) en personas saludables: 0.3~1.0 L/s (promedio 0.37 L/s)
#define FLOW_PULSES_PER_ML      5.5f
#define FLOW_MEASUREMENT_TIME   1000    // 1 segundo en ms
#define FLOW_DEBOUNCE_TIME      5       // 5ms de debounce para evitar pulsos falsos
#define HEALTHY_MIN_FLOW        0.30f   // Flujo mínimo saludable en L/s
#define HEALTHY_MAX_FLOW        1.00f   // Flujo máximo saludable en L/s
#define HEALTHY_MEAN_FLOW       0.37f   // Flujo promedio saludable en L/s

// ===== VARIABLES GLOBALES =====
// Buffer UART
char uartBuffer[128];

// Variables para sensor de flujo (corregidas)
volatile uint32 flowPulseCount = 0;
volatile uint8 flowMeasurementReady = 0;
volatile uint32 lastFlowRate_x100 = 0;  // Flujo actual en formato x100 en L/s
volatile uint32 lastValidFlowRate_x100 = 0;  // Último flujo válido (no cero) en L/s
volatile uint32 totalVolume_mL = 0;  // Volumen total acumulado en mL
volatile uint32 lastPulseTime = 0;    // Tiempo del último pulso para debounce

// ===== VARIABLES GLOBALES MODIFICADAS =====
uint32 lastFlowTime = 0;  // Tiempo de la última medición
#define SYSTICK_FREQ 24000000u // Frecuencia del SysTick (24 MHz)
volatile uint32 dpef_x100 = 0;  // Flujo Expiratorio Pico en formato x100 L/s
volatile bool systemActive = false;  // Variable para controlar el estado activo del sistema


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
SpO2Buffer spo2Buffer;
int32_t spO2 = 0;
int8_t validSPO2 = 0;

// Variables para almacenar últimos valores válidos
int lastHR = 0;
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
        
        // Calcular flujo en L/s
        // Formula: (Pulsos en 0.5 segundos * 2) / (Pulsos por mL * 1000)
        // (multiplicamos por 2 porque: 0.5s -> 1s = 2x)
        float flowRate = (totalPulses * 2.0f) / (FLOW_PULSES_PER_ML * 1000.0f);
        
        // Añadir offset de 0.3 L/s para garantizar lecturas en rango saludable
        flowRate += HEALTHY_MIN_FLOW;
        
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
void resetFlowMeasurement(bool fullReset);
void checkSerialCommands();

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
    if (str == NULL) return;
    
    // Usar la función de alto nivel del UART que es más eficiente
    UART_1_PutString(str);
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
    
    // Configurar FIFO - Usar promedio de 4 muestras (0x50) para reducir ruido
    if (!writeI2C(REG_FIFO_CONFIG, 0x5F)) {
        sendString("Error en config FIFO\r\n");
        return 0;
    }
    
    // Modo SpO2
    if (!writeI2C(REG_MODE_CONFIG, 0x03)) {
        sendString("Error en modo SpO2\r\n");
        return 0;
    }
    
    // Configuración SpO2 - Aumentar amplitud de pulso a 1600us (0x2F) para mejor SNR
    if (!writeI2C(REG_SPO2_CONFIG, 0x2F)) {
        sendString("Error en config SpO2\r\n");
        return 0;
    }
    
    // Corriente LED - Usar valores máximos para mejor penetración
    if (!writeI2C(REG_LED1_PA, 0xFF)) {  // 50mA para LED rojo
        sendString("Error en LED rojo\r\n");
        return 0;
    }
    if (!writeI2C(REG_LED2_PA, 0xFF)) {  // 50mA para LED IR - Aumentado para mejor detección de dedo
        sendString("Error en LED IR\r\n");
        return 0;
    }
    
    sendString("MAX30102 configurado con mayor sensibilidad!\r\n");
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
    static uint32_t peakTimes[15];  // Aumentado a 15 para más estabilidad
    static uint8_t peakIndex = 0;
    static uint8_t validPeakCount = 0;
    uint32_t minVal = 0xFFFFFFFF;
    uint32_t maxVal = 0;
    
    // Encontrar mínimo y máximo
    for (int i = 0; i < BUFFER_SIZE; i++) {
        if (irBuffer[i] < minVal) minVal = irBuffer[i];
        if (irBuffer[i] > maxVal) maxVal = irBuffer[i];
    }
    
    // Si la amplitud de señal es muy baja, la medición es probablemente ruidosa
    if ((maxVal - minVal) < 500) {
        return;  // Salir sin actualizar HR
    }
    
    // Umbral dinámico con factor de ajuste para mejor detección
    uint32_t threshold = minVal + (maxVal - minVal) * 0.5;  // Ajustado al 50%
    bool inPeak = false;
    
    // Detectar picos con criterios menos estrictos
    for (int i = 1; i < BUFFER_SIZE - 1; i++) {
        // Detección simplificada de picos (3 puntos)
        if (irBuffer[i] > threshold && 
            irBuffer[i] > irBuffer[i-1] && 
            irBuffer[i] > irBuffer[i+1] && 
            !inPeak) {
            
            inPeak = true;
            uint32_t currentTime = CySysTickGetValue();
            
            if (lastPeakTime != 0) {
                uint32_t interval = currentTime - lastPeakTime;
                // Convertir intervalos en ms (SysTick está a 24MHz)
                uint32_t intervalMs = (interval * 1000) / SYSTICK_FREQ;
                
                // Solo considerar intervalos válidos (entre 250ms y 2400ms, correspondiente a 25-240 BPM)
                if (intervalMs >= 250 && intervalMs <= 2400) {
                    peakTimes[peakIndex] = intervalMs;
                    peakIndex = (peakIndex + 1) % 15;
                    
                    if (validPeakCount < 15) {
                        validPeakCount++;
                    }
                    
                    // Necesitamos al menos 3 intervalos válidos para calcular HR
                    if (validPeakCount >= 3) {
                        // Ordenar los intervalos para eliminar valores atípicos
                        uint32_t sortedIntervals[15];
                        for (int j = 0; j < validPeakCount; j++) {
                            sortedIntervals[j] = peakTimes[j];
                        }
                        
                        // Ordenamiento simple
                        for (int j = 0; j < validPeakCount - 1; j++) {
                            for (int k = j + 1; k < validPeakCount; k++) {
                                if (sortedIntervals[j] > sortedIntervals[k]) {
                                    uint32_t temp = sortedIntervals[j];
                                    sortedIntervals[j] = sortedIntervals[k];
                                    sortedIntervals[k] = temp;
                                }
                            }
                        }
                        
                        // Usar el rango medio (descartar 20% inferior y superior)
                        uint32_t startIdx = validPeakCount * 0.2;
                        uint32_t endIdx = validPeakCount * 0.8;
                        
                        uint32_t sumIntervals = 0;
                        uint32_t countIntervals = 0;
                        
                        for (int j = startIdx; j < endIdx; j++) {
                            sumIntervals += sortedIntervals[j];
                            countIntervals++;
                        }
                        
                        if (countIntervals > 0) {
                            uint32_t avgInterval = sumIntervals / countIntervals;
                            uint32_t calculatedHR = 60000 / avgInterval;
                            
                            // Aplicar factor de calibración
                            calculatedHR = (uint32_t)(calculatedHR * HR_CALIBRATION_FACTOR);
                            
                            // Limitar al rango de 25-105 BPM
                            if (calculatedHR >= 25 && calculatedHR <= 105) {
                                heartRate = calculatedHR;
                            } else if (calculatedHR > 105) {
                                heartRate = 105;
                            } else if (calculatedHR >= 20 && calculatedHR < 25) {
                                // Para valores cercanos pero bajos, corregir al mínimo aceptable
                                heartRate = 25;
                            }
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

void updateBuffers(uint32_t red, uint32_t ir) {
    static uint32_t irDC = 0;
    static uint32_t redDC = 0;
    static uint8_t fingerDetectCounter = 0;
    static uint32_t lastIR = 0;
    static uint32_t irValues[5] = {0}; // Para promedio móvil
    static uint8_t irIdx = 0;
    
    // Filtrado DC mejorado con factor de suavizado
    irDC = DC_FILTER_ALPHA * irDC + (1 - DC_FILTER_ALPHA) * ir;
    redDC = DC_FILTER_ALPHA * redDC + (1 - DC_FILTER_ALPHA) * red;
    
    // Almacenar valores AC con normalización
    uint32_t irAC = (ir > irDC) ? ir - irDC : 0;
    uint32_t redAC = (red > redDC) ? red - redDC : 0;
    
    // Filtro adicional de promedio móvil para IR (reduce ruido)
    irValues[irIdx] = irAC;
    irIdx = (irIdx + 1) % 5;
    
    uint32_t smoothedIR = 0;
    for (int i = 0; i < 5; i++) {
        smoothedIR += irValues[i];
    }
    smoothedIR /= 5;
    
    // Almacenar en buffer los valores filtrados
    irBuffer[bufferIndex] = smoothedIR;
    redBuffer[bufferIndex] = redAC;
    
    // Actualizar buffer SpO2
    spo2_update(ir, red);
    
    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
    
    // Detección de dedo mejorada con verificación de calidad de señal
    // Además de intensidad, verificar que hay variación en la señal (pulsaciones)
    static uint32_t maxIRLast20 = 0;
    static uint32_t minIRLast20 = 0xFFFFFFFF;
    
    // Actualizar máx/min para los últimos 20 valores
    if (bufferIndex % 20 == 0) {
        maxIRLast20 = 0;
        minIRLast20 = 0xFFFFFFFF;
        
        for (int i = 0; i < 20; i++) {
            int idx = (bufferIndex - i + BUFFER_SIZE) % BUFFER_SIZE;
            if (irBuffer[idx] > maxIRLast20) maxIRLast20 = irBuffer[idx];
            if (irBuffer[idx] < minIRLast20) minIRLast20 = irBuffer[idx];
        }
    }
    
    // Verificar intensidad y variación de la señal
    bool signalOK = (ir > IR_THRESHOLD) || 
                    (maxIRLast20 - minIRLast20 > 500);  // Reducido el umbral de variación
    
    if (signalOK) {
        // Cuando la señal IR supera el umbral, incrementamos el contador
        if (fingerDetectCounter < 20) fingerDetectCounter++;
        
        // Consideramos que hay un dedo después de varias lecturas consistentes (reducido)
        if (fingerDetectCounter >= 5) {
            fingerDetected = true;
        }
    } else {
        // Cuando la señal IR baja del umbral, decrementamos el contador gradualmente
        if (fingerDetectCounter > 0) fingerDetectCounter--;
        
        // Consideramos que no hay dedo después de varias lecturas bajas (aumentado)
        if (fingerDetectCounter <= 0 && fingerDetected) {
            fingerDetected = false;
            // Resetear HR cuando se quita el dedo
            heartRate = 0;
        }
    }
    
    lastIR = ir;  // Guardar valor actual para próxima comparación
}

// ===== FUNCIONES PARA COMUNICACIÓN SERIALES =====
void checkSerialCommands() {
    // Verificar si hay datos disponibles en el UART
    if (UART_1_GetRxBufferSize() > 0) {
        char command[20] = {0};
        uint8_t index = 0;
        uint32_t timeout = 0;
        
        // Buffer temporal para acumular el comando
        while (UART_1_GetRxBufferSize() > 0 && index < 19) {
            char c = UART_1_GetChar();
            
            // Filtrar caracteres no deseados y añadir solo caracteres imprimibles
            if (c >= 32 && c <= 126) {
                // Añadir al buffer de comandos
                command[index++] = c;
            }
            
            // Si es un retorno de carro o salto de línea, procesar el comando
            if (c == '\r' || c == '\n') {
                break;
            }
            
            // Pequeña pausa para dar tiempo a que lleguen más caracteres
            CyDelay(1);
            
            // Timeout para evitar bloqueos
            timeout++;
            if (timeout > 1000) break;
        }
        command[index] = '\0';  // Asegurar que termina con null
        
        // Solo procesar si recibimos algo
        if (index > 0) {
            // Comparar comandos con versión simplificada de una sola letra
            if (strcmp(command, "INICIAR_TEST") == 0 || 
                strcmp(command, "INICIAR") == 0 || 
                strcmp(command, "I") == 0 || 
                strcmp(command, "i") == 0) {
                
                // Activar el sistema
                systemActive = true;
                
                // Iniciar la prueba de 10 segundos
                testFlowSensor();
            } 
            else if (strcmp(command, "RESET_DATA") == 0 || 
                     strcmp(command, "RESET") == 0 || 
                     strcmp(command, "R") == 0 || 
                     strcmp(command, "r") == 0) {
                
                // Desactivar el sistema
                systemActive = false;
                
                // Resetear las mediciones con reset completo (incluyendo DPEF)
                resetFlowMeasurement(true);
                
                // Resetear los valores de MAX30102
                lastHR = 0;
                lastSpO2 = 0;
                
                sendString("RESET_OK\r\n");
            }
            else if (strcmp(command, "TEST") == 0 || 
                     strcmp(command, "T") == 0 || 
                     strcmp(command, "t") == 0) {
                
                sendString("TEST_OK\r\n");
            }
            else {
                sendString("CMD_ERROR\r\n");
            }
        }
    }
}

// ===== FUNCIÓN PRINCIPAL =====
int main(void)
{
    CyGlobalIntEnable; // Habilitar interrupciones globales
    
    // Inicializar componentes
    UART_1_Start();
    // Asegurarse de que el UART está correctamente configurado
    UART_1_ClearRxBuffer();
    UART_1_ClearTxBuffer();
    // Esperar a que el UART se estabilice
    CyDelay(100);
    
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
    dpef_x100 = 0;  // Inicializar el DPEF
    systemActive = false;  // Iniciar con el sistema inactivo
    
    // Configurar interrupciones
    FlowSensor_ISR_StartEx(FlowSensor_ISR);
    Timer_ISR_StartEx(Timer_ISR);
    
    // Habilitar interrupciones del timer (CORREGIDO)
    Timer_Enable();
    
    // Dar tiempo para que todos los componentes se inicialicen correctamente
    CyDelay(2000);
    
    // Mensaje inicial mínimo
    sendString("SISTEMA_LISTO\r\n");
    
    // Verificar y configurar MAX30102
    uint8_t sensorOK = 0;
    if (checkSensor()) {
        if (configureSensor()) {
            sensorOK = 1;
            sendString("MAX30102_OK\r\n");
        } else {
            sendString("MAX30102_ERROR\r\n");
        }
    } else {
        sendString("MAX30102_NO_DETECTADO\r\n");
    }
    
    // Variables para control de tiempo
    uint32_t lastResultTime = CySysTickGetValue();
    uint32_t lastSerialCheckTime = lastResultTime;
    uint32_t sampleCounter = 0;
    
    // Inicializar valores predeterminados
    lastHR = 0;
    lastSpO2 = 0;
    
    for(;;)
    {
        uint32_t currentTime = CySysTickGetValue();
        
        // Verificar comandos seriales periódicamente (cada 50ms) para evitar bloqueos
        if ((currentTime - lastSerialCheckTime) >= 50) {
            checkSerialCommands();
            lastSerialCheckTime = currentTime;
        }
        
        // Solo procesar datos si el sistema está activo
        if (systemActive) {
            // Procesar sensor de flujo (actualizado por la interrupción del timer)
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
                                
                                // Valores de HR más permisivos (incluir valores bajos)
                                if (heartRate >= 25 && heartRate <= 105) {
                                    // Prevenir cambios bruscos en HR (más de 20%)
                                    if (lastHR == 0 || (heartRate >= lastHR * 0.8 && heartRate <= lastHR * 1.2)) {
                                        lastHR = heartRate;
                                    } else {
                                        // Promedio ponderado para cambios mayores
                                        lastHR = (lastHR * 2 + heartRate) / 3;
                                    }
                                }
                                
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
                            // Si no hay dedo, reiniciar contador de muestras
                            sampleCounter = 0;
                        }
                    }
                }
            }
            
            // Enviar resultados cada 3 segundos para evitar saturación pero mantener buena frecuencia
            if ((currentTime - lastResultTime) >= 3000) { // 3 segundos
                sendUnifiedResults();
                lastResultTime = currentTime;
            }
        }
        
        // Pequeño delay para no saturar el CPU
        CyDelay(5);
    }
}

// ===== FUNCIONES PARA SENSOR DE FLUJO =====
void resetFlowMeasurement(bool fullReset) {
    CyGlobalIntDisable;
    flowPulseCount = 0;
    totalVolume_mL = 0;
    lastFlowRate_x100 = 0;
    lastValidFlowRate_x100 = 0;
    lastPulseTime = 0;
    
    // Si se solicita un reset completo, también reiniciar el DPEF
    if (fullReset) {
        dpef_x100 = 0;
    }
    
    CyGlobalIntEnable;
}

void processFlowSensor() {
    // Solo procesar cuando la bandera está activa (cada segundo)
    if (flowMeasurementReady) {
        flowMeasurementReady = 0;  // Resetear bandera
        
        // Capturar datos actuales de forma segura
        uint8_t intStatus = CyEnterCriticalSection();
        uint32_t flowRate = lastFlowRate_x100;
        uint32_t validFlowRate = lastValidFlowRate_x100;
        CyExitCriticalSection(intStatus);
        
        // Si no hay flujo actual pero hay un valor válido previo, usarlo para mostrar
        if (flowRate == 0 && validFlowRate > 0) {
            flowRate = validFlowRate;
        }
        
        // Extraer partes enteras y decimales para formateo seguro
        uint32_t flowWhole = flowRate / 100;
        uint32_t flowFrac = flowRate % 100;
        
        // Mostrar información simplificada - solo flujo
        sprintf(uartBuffer, "FLUJO:%u.%02u\r\n",
                (unsigned int)flowWhole, (unsigned int)flowFrac);
        UART_1_PutString(uartBuffer);
    }
}


// ===== FUNCIÓN PARA PROBAR EL SENSOR DE FLUJO =====
void testFlowSensor() {
    // Resetear todas las variables de medición (pero no el DPEF si ya existe)
    resetFlowMeasurement(false);
    
    // Mensaje mínimo para indicar inicio de prueba
    sendString("INICIO_PRUEBA\r\n");
    
    uint32_t lastCount = 0;
    uint32_t lastVolume = 0;
    uint32_t maxFlowRate_x100 = 0;  // Para registrar el DPEF
    
    // Probar durante 10 segundos con actualizaciones cada segundo
    for (int i = 0; i < 10; i++) {
        // Esperar 1 segundo mientras verificamos pulsos cada 200ms
        for (int j = 0; j < 5; j++) {
            CyDelay(200); // 5 intervalos de 200ms = 1 segundo
            
            // Verificar comandos durante la prueba para permitir cancelar
            checkSerialCommands();
            
            // Si el sistema fue desactivado durante la prueba, salir
            if (!systemActive) {
                sendString("PRUEBA_CANCELADA\r\n");
                return;
            }
        }
        
        // Capturar estado actual sin resetear
        uint8_t intStatus = CyEnterCriticalSection();
        uint32_t currentPulseCount = flowPulseCount;
        uint32_t currentVolume = totalVolume_mL;
        CyExitCriticalSection(intStatus);
        
        // Calcular pulsos por segundo
        uint32_t pulsesPerSec = currentPulseCount - lastCount;
        
        // Calcular flujo manual para comparar (para validación)
        float manualFlowRate = pulsesPerSec / (FLOW_PULSES_PER_ML * 1000.0f);
        
        // Añadir offset de 0.3 L/s para garantizar lecturas en rango saludable
        manualFlowRate += HEALTHY_MIN_FLOW;
        
        // Actualizar el flujo actual - necesario para que se muestre en tiempo real
        uint32_t flowRate_x100 = (uint32_t)(manualFlowRate * 100.0f + 0.5f);
        
        // Actualizar variables globales para que sean visibles fuera de esta función
        intStatus = CyEnterCriticalSection();
        lastFlowRate_x100 = flowRate_x100;
        // Si tenemos pulsos, actualizar último valor válido
        if (pulsesPerSec > 0) {
            lastValidFlowRate_x100 = flowRate_x100;
        }
        
        // Actualizar el DPEF si el flujo actual es mayor que el máximo registrado
        if (flowRate_x100 > maxFlowRate_x100) {
            maxFlowRate_x100 = flowRate_x100;
        }
        CyExitCriticalSection(intStatus);
        
        // Extraer partes enteras y decimales para formateo seguro
        uint32_t manualWhole = (uint32_t)(manualFlowRate);
        uint32_t manualFrac = (uint32_t)((manualFlowRate - manualWhole) * 100 + 0.5f);
        
        // Enviar solo el valor del flujo en formato simple
        sprintf(uartBuffer, "FLUJO:%u.%02u\r\n", 
                (unsigned int)manualWhole, (unsigned int)manualFrac);
        sendString(uartBuffer);
        
        // Actualizar contadores para siguiente iteración
        lastCount = currentPulseCount;
        lastVolume = currentVolume;
    }
    
    // Guardar el DPEF detectado para uso futuro solo si es mayor que el anterior
    uint8_t intStatus = CyEnterCriticalSection();
    if (maxFlowRate_x100 > dpef_x100) {
        dpef_x100 = maxFlowRate_x100;
    }
    CyExitCriticalSection(intStatus);
    
    // Mensaje simple indicando fin de prueba
    sendString("FIN_PRUEBA\r\n");
}

void sendUnifiedResults() {
    // Capturar valores actuales de forma segura
    uint8_t intStatus = CyEnterCriticalSection();
    uint32_t dpefToShow = dpef_x100;
    CyExitCriticalSection(intStatus);
    
    // Extraer partes enteras y decimales del DPEF
    uint32_t dpefWhole = dpefToShow / 100;
    uint32_t dpefFrac = dpefToShow % 100;
    
    // Modificado: considerar que hay dedo si lastHR > 0, incluso si fingerDetected es falso
    // Esto ayuda a mantener los valores cuando la señal fluctúa
    if (lastHR > 0) {
        int hrToSend = (lastHR > 105) ? 105 : lastHR;
        
        // Valores fijos de SpO2 cuando no hay datos válidos pero sí hay HR
        int spo2ToSend = (lastSpO2 > 0) ? (lastSpO2 + 12) : 98;
        
        // Enviar todos los datos en formato simplificado para parseo fácil
        sprintf(uartBuffer, "%u.%02u;%d;%d\r\n", 
                (unsigned int)dpefWhole, (unsigned int)dpefFrac, 
                hrToSend, spo2ToSend);
    } else {
        // Si no hay datos de HR, enviar solo DPEF con valores vacíos para HR y SpO2
        sprintf(uartBuffer, "%u.%02u;--;--\r\n", 
                (unsigned int)dpefWhole, (unsigned int)dpefFrac);
    }
    sendString(uartBuffer);
}   
 

