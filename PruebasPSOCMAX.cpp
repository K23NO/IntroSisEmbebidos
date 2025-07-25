#include "project.h"
#include <stdio.h>

// Direcciones I2C del MAX30105
#define MAX30105_ADDRESS    0x57
#define MAX30105_FIFO_WR_PTR    0x04
#define MAX30105_FIFO_RD_PTR    0x06
#define MAX30105_FIFO_DATA      0x07
#define MAX30105_MODE_CONFIG    0x09
#define MAX30105_SPO2_CONFIG    0x0A
#define MAX30105_LED1_PA        0x0C
#define MAX30105_LED2_PA        0x0D

// Configuraciones del sensor
#define BUFFER_SIZE 100
#define SAMPLE_RATE 100 // Hz

// Variables globales
uint32_t redBuffer[BUFFER_SIZE];
uint32_t irBuffer[BUFFER_SIZE];
uint8_t bufferIndex = 0;


//Prototipo de funciones
// Prototipos de funciones
uint8_t detectPulse(uint32_t irValue);
float calculateBPM();


// Función para escribir registro I2C
void MAX30105_writeRegister(uint8_t reg, uint8_t value) {
    uint8_t writeBuffer[2] = {reg, value};
    I2C_Master_MasterWriteBuf(MAX30105_ADDRESS, writeBuffer, 2, 
                              I2C_Master_MODE_COMPLETE_XFER);
    while (I2C_Master_MasterStatus() & I2C_Master_MSTAT_XFER_INP);
}

// Función para leer registro I2C
uint8_t MAX30105_readRegister(uint8_t reg) {
    uint8_t readValue;
    
    // Escribir dirección del registro
    I2C_Master_MasterWriteBuf(MAX30105_ADDRESS, &reg, 1, 
                              I2C_Master_MODE_NO_STOP);
    while (I2C_Master_MasterStatus() & I2C_Master_MSTAT_XFER_INP);
    
    // Leer el valor
    I2C_Master_MasterReadBuf(MAX30105_ADDRESS, &readValue, 1, 
                             I2C_Master_MODE_COMPLETE_XFER);
    while (I2C_Master_MasterStatus() & I2C_Master_MSTAT_XFER_INP);
    
    return readValue;
}

// Inicialización del sensor
void MAX30105_init() {
    // Reset del sensor
    MAX30105_writeRegister(MAX30105_MODE_CONFIG, 0x40);
    CyDelay(100);
    
    // Configurar modo SpO2
    MAX30105_writeRegister(MAX30105_MODE_CONFIG, 0x03); // SpO2 mode
    
    // Configurar SpO2
    // SPO2_ADC_RGE = 4096nA, SPO2_SR = 100Hz, LED_PW = 411μs
    MAX30105_writeRegister(MAX30105_SPO2_CONFIG, 0x27);
    
    // Configurar amplitud de LEDs
    MAX30105_writeRegister(MAX30105_LED1_PA, 0x3C); // Red LED
    MAX30105_writeRegister(MAX30105_LED2_PA, 0x3C); // IR LED
}

// Leer datos del FIFO
void MAX30105_readFifoData(uint32_t *redData, uint32_t *irData) {
    uint8_t fifoData[6];
    uint8_t reg = MAX30105_FIFO_DATA;
    
    // Leer 6 bytes del FIFO (3 bytes Red + 3 bytes IR)
    I2C_Master_MasterWriteBuf(MAX30105_ADDRESS, &reg, 1, 
                              I2C_Master_MODE_NO_STOP);
    while (I2C_Master_MasterStatus() & I2C_Master_MSTAT_XFER_INP);
    
    I2C_Master_MasterReadBuf(MAX30105_ADDRESS, fifoData, 6, 
                             I2C_Master_MODE_COMPLETE_XFER);
    while (I2C_Master_MasterStatus() & I2C_Master_MSTAT_XFER_INP);
    
    // Convertir bytes a valores de 18 bits
    *redData = ((uint32_t)fifoData[0] << 16) | 
               ((uint32_t)fifoData[1] << 8) | 
               fifoData[2];
    *redData &= 0x3FFFF; // Máscara de 18 bits
    
    *irData = ((uint32_t)fifoData[3] << 16) | 
              ((uint32_t)fifoData[4] << 8) | 
              fifoData[5];
    *irData &= 0x3FFFF; // Máscara de 18 bits
}


int main() {
    char uartBuffer[100];
    uint32_t redValue, irValue;
    uint8_t sampleCount = 0;
    
    // Inicializar componentes
    CyGlobalIntEnable;
    I2C_Master_Start();
    UART_PC_Start();
    Timer_Sample_Start();
    
    // Mensaje de inicio
    sprintf(uartBuffer, "Iniciando sensor MAX30105...\r\n");
    UART_PC_PutString(uartBuffer);
    
    // Inicializar sensor
    MAX30105_init();
    CyDelay(1000);
    
    sprintf(uartBuffer, "Sensor inicializado. Coloque el dedo...\r\n");
    UART_PC_PutString(uartBuffer);
    
    for(;;) {
        // Leer datos del sensor
        MAX30105_readFifoData(&redValue, &irValue);
        
        // Almacenar en buffer
        redBuffer[bufferIndex] = redValue;
        irBuffer[bufferIndex] = irValue;
        
        // Enviar datos por UART
       sprintf(uartBuffer, "RED=%lu, IR=%lu\r\n", (unsigned long)redValue, (unsigned long)irValue);
        UART_PC_PutString(uartBuffer);
        
        // Incrementar índice del buffer
        bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
        
        // Procesar datos cada 100 muestras (similar al código Arduino)
        if (sampleCount >= BUFFER_SIZE) {
            // Aquí implementarías los algoritmos de cálculo de SpO2 y BPM
            // Puedes portar las funciones del archivo spo2_algorithm.h
            
            float bpm = calculateBPM();
            if (bpm > 0) {
                sprintf(uartBuffer, "BPM calculado: %.2f\r\n", bpm);
                UART_PC_PutString(uartBuffer);
            }
            sampleCount = 0;
        }
        
        sampleCount++;
        
        // Indicador visual
        LED_Status_Write(~LED_Status_Read());
        
        // Delay para controlar la frecuencia de muestreo
        CyDelay(10); // 100Hz aprox
    }
}


// Función simple para detectar pulsos
uint8_t detectPulse(uint32_t irValue) {
    static uint32_t lastIR = 0;
    static uint8_t pulseDetected = 0;
    static uint32_t threshold = 50000;
    
    if (irValue > threshold && lastIR <= threshold) {
        pulseDetected = 1;
    } else {
        pulseDetected = 0;
    }
    
    lastIR = irValue;
    return pulseDetected;
}

// Cálculo simple de BPM
float calculateBPM() {
    static uint32_t lastBeatTime = 0;
    static uint32_t beatCount = 0;
    static uint32_t startTime = 0;
    
    uint32_t currentTime = Timer_Sample_ReadCounter();
    
    if (startTime == 0) {
        startTime = currentTime;
    }
    
    // Si ha pasado más de 1 minuto, calcular BPM
    if (currentTime - startTime >= 60000) { // 60 segundos
        float bpm = (beatCount * 60.0) / ((currentTime - startTime) / 1000.0);
        beatCount = 0;
        startTime = currentTime;
        return bpm;
    }
    
    beatCount++; // Contamos un pulso cada vez que se llama
    return 0;
}
