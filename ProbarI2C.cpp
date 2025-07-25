#include "project.h"
#include <stdio.h>

// Direcciones I2C del MAX30105
#define MAX30105_ADDRESS    0x57
#define MAX30105_PART_ID    0xFF
#define MAX30105_REV_ID     0xFE
#define MAX30105_FIFO_WR_PTR    0x04
#define MAX30105_FIFO_RD_PTR    0x06
#define MAX30105_FIFO_DATA      0x07
#define MAX30105_FIFO_CONFIG    0x08
#define MAX30105_MODE_CONFIG    0x09
#define MAX30105_SPO2_CONFIG    0x0A
#define MAX30105_LED1_PA        0x0C
#define MAX30105_LED2_PA        0x0D
#define MAX30105_LED3_PA        0x0E

// Variables globales
char uartBuffer[200];
uint8_t deviceFound = 0;

// Función para escribir registro I2C con verificación de errores
uint8_t MAX30105_writeRegister(uint8_t reg, uint8_t value) {
    uint8_t writeBuffer[2] = {reg, value};
    uint8_t status;
    
    status = I2C_Master_MasterWriteBuf(MAX30105_ADDRESS, writeBuffer, 2, 
                                       I2C_Master_MODE_COMPLETE_XFER);
    
    if (status != I2C_Master_MSTR_NO_ERROR) {
        sprintf(uartBuffer, "ERROR: Write failed, status = 0x%02X\r\n", status);
        UART_PC_PutString(uartBuffer);
        return 0;
    }
    
    // Esperar hasta que la transferencia termine
    while (I2C_Master_MasterStatus() & I2C_Master_MSTAT_XFER_INP) {
        CyDelay(1);
    }
    
    // Verificar si hubo error
    uint8_t masterStatus = I2C_Master_MasterStatus();
    if (masterStatus & I2C_Master_MSTAT_ERR_MASK) {
        sprintf(uartBuffer, "ERROR: I2C Master error = 0x%02X\r\n", masterStatus);
        UART_PC_PutString(uartBuffer);
        return 0;
    }
    
    return 1;
}

// Función para leer registro I2C con verificación de errores
uint8_t MAX30105_readRegister(uint8_t reg, uint8_t *value) {
    uint8_t status;
    
    // Escribir dirección del registro
    status = I2C_Master_MasterWriteBuf(MAX30105_ADDRESS, &reg, 1, 
                                       I2C_Master_MODE_NO_STOP);
    
    if (status != I2C_Master_MSTR_NO_ERROR) {
        sprintf(uartBuffer, "ERROR: Read write phase failed, status = 0x%02X\r\n", status);
        UART_PC_PutString(uartBuffer);
        return 0;
    }
    
    // Esperar hasta que la transferencia termine
    while (I2C_Master_MasterStatus() & I2C_Master_MSTAT_XFER_INP) {
        CyDelay(1);
    }
    
    // Leer el valor
    status = I2C_Master_MasterReadBuf(MAX30105_ADDRESS, value, 1, 
                                      I2C_Master_MODE_COMPLETE_XFER);
    
    if (status != I2C_Master_MSTR_NO_ERROR) {
        sprintf(uartBuffer, "ERROR: Read phase failed, status = 0x%02X\r\n", status);
        UART_PC_PutString(uartBuffer);
        return 0;
    }
    
    // Esperar hasta que la transferencia termine
    while (I2C_Master_MasterStatus() & I2C_Master_MSTAT_XFER_INP) {
        CyDelay(1);
    }
    
    // Verificar si hubo error
    uint8_t masterStatus = I2C_Master_MasterStatus();
    if (masterStatus & I2C_Master_MSTAT_ERR_MASK) {
        sprintf(uartBuffer, "ERROR: I2C Master read error = 0x%02X\r\n", masterStatus);
        UART_PC_PutString(uartBuffer);
        return 0;
    }
    
    return 1;
}

// Función para verificar si el dispositivo está conectado
uint8_t MAX30105_checkDevice() {
    uint8_t partID, revID;
    
    sprintf(uartBuffer, "Verificando dispositivo MAX30105...\r\n");
    UART_PC_PutString(uartBuffer);
    
    // Leer Part ID
    if (!MAX30105_readRegister(MAX30105_PART_ID, &partID)) {
        sprintf(uartBuffer, "ERROR: No se pudo leer Part ID\r\n");
        UART_PC_PutString(uartBuffer);
        return 0;
    }
    
    // Leer Revision ID
    if (!MAX30105_readRegister(MAX30105_REV_ID, &revID)) {
        sprintf(uartBuffer, "ERROR: No se pudo leer Revision ID\r\n");
        UART_PC_PutString(uartBuffer);
        return 0;
    }
    
    sprintf(uartBuffer, "Part ID: 0x%02X, Revision ID: 0x%02X\r\n", partID, revID);
    UART_PC_PutString(uartBuffer);
    
    // Verificar Part ID (debe ser 0x15 para MAX30105)
    if (partID != 0x15) {
        sprintf(uartBuffer, "ERROR: Part ID incorrecto. Esperado: 0x15, Recibido: 0x%02X\r\n", partID);
        UART_PC_PutString(uartBuffer);
        return 0;
    }
    
    sprintf(uartBuffer, "MAX30105 detectado correctamente!\r\n");
    UART_PC_PutString(uartBuffer);
    return 1;
}

// Función para escanear dispositivos I2C
void I2C_scan() {
    uint8_t address;
    uint8_t dummy;
    uint8_t devicesFound = 0;
    
    sprintf(uartBuffer, "Escaneando dispositivos I2C...\r\n");
    UART_PC_PutString(uartBuffer);
    
    for (address = 0x08; address < 0x78; address++) {
        // Intentar leer del dispositivo
        uint8_t status = I2C_Master_MasterReadBuf(address, &dummy, 1, 
                                                  I2C_Master_MODE_COMPLETE_XFER);
        
        // Esperar a que termine la transferencia
        while (I2C_Master_MasterStatus() & I2C_Master_MSTAT_XFER_INP) {
            CyDelay(1);
        }
        
        // Verificar si el dispositivo respondió
        uint8_t masterStatus = I2C_Master_MasterStatus();
        if (!(masterStatus & I2C_Master_MSTAT_ERR_MASK)) {
            sprintf(uartBuffer, "Dispositivo encontrado en: 0x%02X\r\n", address);
            UART_PC_PutString(uartBuffer);
            devicesFound++;
        }
        
        CyDelay(10); // Pequeña pausa entre intentos
    }
    
    if (devicesFound == 0) {
        sprintf(uartBuffer, "No se encontraron dispositivos I2C\r\n");
        UART_PC_PutString(uartBuffer);
    } else {
        sprintf(uartBuffer, "Total de dispositivos encontrados: %d\r\n", devicesFound);
        UART_PC_PutString(uartBuffer);
    }
}

// Inicialización del sensor con diagnóstico
uint8_t MAX30105_init() {
    uint8_t regValue;
    
    sprintf(uartBuffer, "Iniciando configuracion del sensor...\r\n");
    UART_PC_PutString(uartBuffer);
    
    // Reset del sensor
    sprintf(uartBuffer, "Reseteando sensor...\r\n");
    UART_PC_PutString(uartBuffer);
    
    if (!MAX30105_writeRegister(MAX30105_MODE_CONFIG, 0x40)) {
        sprintf(uartBuffer, "ERROR: No se pudo resetear el sensor\r\n");
        UART_PC_PutString(uartBuffer);
        return 0;
    }
    
    CyDelay(100);
    
    // Verificar que el reset se completó
    if (!MAX30105_readRegister(MAX30105_MODE_CONFIG, &regValue)) {
        sprintf(uartBuffer, "ERROR: No se pudo leer registro de modo\r\n");
        UART_PC_PutString(uartBuffer);
        return 0;
    }
    
    sprintf(uartBuffer, "Registro de modo des reset: 0x%02X\r\n", regValue);
    UART_PC_PutString(uartBuffer);
    
    // Configurar FIFO
    sprintf(uartBuffer, "Configurando FIFO...\r\n");
    UART_PC_PutString(uartBuffer);
    
    if (!MAX30105_writeRegister(MAX30105_FIFO_CONFIG, 0x4F)) { // Sample avg = 1, fifo rollover = false, fifo almost full = 17
        sprintf(uartBuffer, "ERROR: No se pudo configurar FIFO\r\n");
        UART_PC_PutString(uartBuffer);
        return 0;
    }
    
    // Configurar modo SpO2
    sprintf(uartBuffer, "Configurando modo SpO2...\r\n");
    UART_PC_PutString(uartBuffer);
    
    if (!MAX30105_writeRegister(MAX30105_MODE_CONFIG, 0x03)) { // SpO2 mode
        sprintf(uartBuffer, "ERROR: No se pudo configurar modo SpO2\r\n");
        UART_PC_PutString(uartBuffer);
        return 0;
    }
    
    // Configurar SpO2 settings
    sprintf(uartBuffer, "Configurando parametros SpO2...\r\n");
    UART_PC_PutString(uartBuffer);
    
    if (!MAX30105_writeRegister(MAX30105_SPO2_CONFIG, 0x27)) { // ADC=4096nA, SR=100Hz, PW=411μs
        sprintf(uartBuffer, "ERROR: No se pudo configurar parametros SpO2\r\n");
        UART_PC_PutString(uartBuffer);
        return 0;
    }
    
    // Configurar amplitud de LEDs
    sprintf(uartBuffer, "Configurando LEDs...\r\n");
    UART_PC_PutString(uartBuffer);
    
    if (!MAX30105_writeRegister(MAX30105_LED1_PA, 0x24)) { // Red LED = 7.0mA
        sprintf(uartBuffer, "ERROR: No se pudo configurar LED Rojo\r\n");
        UART_PC_PutString(uartBuffer);
        return 0;
    }
    
    if (!MAX30105_writeRegister(MAX30105_LED2_PA, 0x24)) { // IR LED = 7.0mA
        sprintf(uartBuffer, "ERROR: No se pudo configurar LED IR\r\n");
        UART_PC_PutString(uartBuffer);
        return 0;
    }
    
    // Verificar configuración
    sprintf(uartBuffer, "Verificando configuracion...\r\n");
    UART_PC_PutString(uartBuffer);
    
    if (!MAX30105_readRegister(MAX30105_MODE_CONFIG, &regValue)) {
        sprintf(uartBuffer, "ERROR: No se pudo verificar configuracion\r\n");
        UART_PC_PutString(uartBuffer);
        return 0;
    }
    
    sprintf(uartBuffer, "Modo configurado: 0x%02X\r\n", regValue);
    UART_PC_PutString(uartBuffer);
    
    sprintf(uartBuffer, "Sensor inicializado correctamente!\r\n");
    UART_PC_PutString(uartBuffer);
    
    return 1;
}

// Función para leer datos del FIFO con diagnóstico
uint8_t MAX30105_readFifoData(uint32_t *redData, uint32_t *irData) {
    uint8_t fifoData[6];
    uint8_t reg = MAX30105_FIFO_DATA;
    
    // Leer 6 bytes del FIFO (3 bytes Red + 3 bytes IR)
    uint8_t status = I2C_Master_MasterWriteBuf(MAX30105_ADDRESS, &reg, 1, 
                                               I2C_Master_MODE_NO_STOP);
    
    if (status != I2C_Master_MSTR_NO_ERROR) {
        sprintf(uartBuffer, "ERROR: FIFO write failed, status = 0x%02X\r\n", status);
        UART_PC_PutString(uartBuffer);
        return 0;
    }
    
    while (I2C_Master_MasterStatus() & I2C_Master_MSTAT_XFER_INP) {
        CyDelay(1);
    }
    
    status = I2C_Master_MasterReadBuf(MAX30105_ADDRESS, fifoData, 6, 
                                      I2C_Master_MODE_COMPLETE_XFER);
    
    if (status != I2C_Master_MSTR_NO_ERROR) {
        sprintf(uartBuffer, "ERROR: FIFO read failed, status = 0x%02X\r\n", status);
        UART_PC_PutString(uartBuffer);
        return 0;
    }
    
    while (I2C_Master_MasterStatus() & I2C_Master_MSTAT_XFER_INP) {
        CyDelay(1);
    }
    
    // Verificar errores
    uint8_t masterStatus = I2C_Master_MasterStatus();
    if (masterStatus & I2C_Master_MSTAT_ERR_MASK) {
        sprintf(uartBuffer, "ERROR: FIFO read I2C error = 0x%02X\r\n", masterStatus);
        UART_PC_PutString(uartBuffer);
        return 0;
    }
    
    // Convertir bytes a valores de 18 bits
    *redData = ((uint32_t)fifoData[0] << 16) | 
               ((uint32_t)fifoData[1] << 8) | 
               fifoData[2];
    *redData &= 0x3FFFF; // Máscara de 18 bits
    
    *irData = ((uint32_t)fifoData[3] << 16) | 
              ((uint32_t)fifoData[4] << 8) | 
              fifoData[5];
    *irData &= 0x3FFFF; // Máscara de 18 bits
    
    return 1;
}

int main() {
    uint32_t redValue, irValue;
    uint8_t initSuccess = 0;
    uint32_t sampleCount = 0;
    
    // Inicializar componentes
    CyGlobalIntEnable;
    I2C_Master_Start();
    UART_PC_Start();
    
    // Mensaje de inicio
    sprintf(uartBuffer, "\r\n=== DIAGNOSTICO MAX30105 ===\r\n");
    UART_PC_PutString(uartBuffer);
    
    sprintf(uartBuffer, "PSoC 5LP iniciado\r\n");
    UART_PC_PutString(uartBuffer);
    
    CyDelay(1000);
    
    // Escanear dispositivos I2C
    I2C_scan();
    
    CyDelay(1000);
    
    // Verificar dispositivo
    deviceFound = MAX30105_checkDevice();
    
    if (deviceFound) {
        // Inicializar sensor
        initSuccess = MAX30105_init();
        
        if (initSuccess) {
            sprintf(uartBuffer, "\r\n=== SENSOR LISTO ===\r\n");
            UART_PC_PutString(uartBuffer);
            sprintf(uartBuffer, "Coloque el dedo en el sensor...\r\n");
            UART_PC_PutString(uartBuffer);
        } else {
            sprintf(uartBuffer, "\r\n=== ERROR EN INICIALIZACION ===\r\n");
            UART_PC_PutString(uartBuffer);
        }
    } else {
        sprintf(uartBuffer, "\r\n=== SENSOR NO DETECTADO ===\r\n");
        UART_PC_PutString(uartBuffer);
        sprintf(uartBuffer, "Verifique conexiones:\r\n");
        UART_PC_PutString(uartBuffer);
        sprintf(uartBuffer, "- SDA: P12[0] -> SDA sensor\r\n");
        UART_PC_PutString(uartBuffer);
        sprintf(uartBuffer, "- SCL: P12[1] -> SCL sensor\r\n");
        UART_PC_PutString(uartBuffer);
        sprintf(uartBuffer, "- GND: GND -> GND sensor\r\n");
        UART_PC_PutString(uartBuffer);
        sprintf(uartBuffer, "- 3.3V: 3.3V -> VIN sensor\r\n");
        UART_PC_PutString(uartBuffer);
        sprintf(uartBuffer, "- Pull-ups: 4.7K ohm a 3.3V en SDA y SCL\r\n");
        UART_PC_PutString(uartBuffer);
    }
    
    for(;;) {
        if (deviceFound && initSuccess) {
            // Leer datos del sensor
            if (MAX30105_readFifoData(&redValue, &irValue)) {
                sampleCount++;
                
                // Enviar datos por UART
                sprintf(uartBuffer, "Sample %lu: RED=%lu, IR=%lu\r\n",
                    (unsigned long)sampleCount,
                    (unsigned long)redValue,
                    (unsigned long)irValue);
                UART_PC_PutString(uartBuffer);
                
                // Verificar si hay señal (finger detection)
                if (irValue > 50000) {
                    sprintf(uartBuffer, "Dedo detectado! IR=%lu\r\n", (unsigned long)irValue);
                    UART_PC_PutString(uartBuffer);
                } else if (sampleCount % 50 == 0) { // Cada 50 muestras
                    sprintf(uartBuffer, "Esperando dedo... (IR=%lu)\r\n", (unsigned long)irValue);
                    UART_PC_PutString(uartBuffer);
                }
                
                CyDelay(100); // 10 Hz sampling rate
            } else {
                sprintf(uartBuffer, "ERROR: No se pudieron leer datos\r\n");
                UART_PC_PutString(uartBuffer);
                CyDelay(1000);
            }
        } else {
            // Reintentar inicialización cada 5 segundos
            CyDelay(5000);
            sprintf(uartBuffer, "Reintentando deteccion...\r\n");
            UART_PC_PutString(uartBuffer);
            
            I2C_scan();
            deviceFound = MAX30105_checkDevice();
            
            if (deviceFound) {
                initSuccess = MAX30105_init();
            }
        }
    }
}