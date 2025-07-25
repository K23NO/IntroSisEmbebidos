// main.c - Sensor de flujo YF-S401 para PSoC 5LP
// Configuración UART: 115200 baud, 8 bits, paridad par, 1 bit de parada

#include "project.h"
#include <stdio.h>
#include <stdlib.h>

// Variables globales para el contador de pulsos
volatile uint32 pulseCount = 0;
volatile uint8 measurementReady = 0;

// Interrupt Service Routine para el sensor de flujo
CY_ISR(FlowSensor_ISR)
{
    pulseCount++;
    // Limpiar la interrupción
    FlowSensor_Pin_ClearInterrupt();
}

// Interrupt Service Routine para el timer (1 segundo)
CY_ISR(Timer_ISR)
{
    measurementReady = 1;
    // Limpiar la interrupción del timer
    Timer_ReadStatusRegister();
}

int main(void)
{
    CyGlobalIntEnable; // Habilitar interrupciones globales
    
    // Variables para el cálculo del flujo
    float flowRate = 0.0;
    char uartBuffer[64];
    uint32 debugCounter = 0;
    uint32 lastPulseCount = 0;
    
    // Inicializar componentes
    UART_1_Start();
    Timer_Start();
    
    // Configurar y habilitar la interrupción del pin del sensor
    FlowSensor_ISR_StartEx(FlowSensor_ISR);
    
    // Configurar y habilitar la interrupción del timer
    Timer_ISR_StartEx(Timer_ISR);
    
    // Mensaje inicial
    sprintf(uartBuffer, "Espirometro con sensor YF-S401 iniciado\r\n");
    UART_1_PutString(uartBuffer);
    sprintf(uartBuffer, "Configuracion: 115200 baud, 8N1, paridad par\r\n");
    UART_1_PutString(uartBuffer);
    
    // Mensaje de debug inicial
    sprintf(uartBuffer, "Iniciando mediciones... Estado pin: %d\r\n", FlowSensor_Pin_Read());
    UART_1_PutString(uartBuffer);
    
    // Test del timer - enviar mensaje cada 2 segundos sin usar measurementReady
    uint32 timeCounter = 0;
    
    for(;;)
    {
        // Verificar si hay una medición lista del timer (cada segundo)
        if(measurementReady)
        {
            // Deshabilitar interrupciones temporalmente para leer pulseCount
            CyGlobalIntDisable;
            uint32 currentPulseCount = pulseCount;
            pulseCount = 0; // Resetear contador para la próxima medición
            measurementReady = 0;
            CyGlobalIntEnable;
            
            // Calcular flujo de aire usando aritmética entera
            // Fórmula: flowRate = (pulseCount / 7.5) * 0.85 = (pulseCount * 85) / 750
            uint32 flowRate_x100 = (currentPulseCount * 85) / 75; // x100 para 2 decimales
            
            // Formatear y enviar datos por UART
            sprintf(uartBuffer, "Flujo: %lu.%02lu L/min, Pulsos/seg: %lu\r\n", 
                    flowRate_x100/100, flowRate_x100%100, currentPulseCount);
            UART_1_PutString(uartBuffer);
            
            // También enviar en formato CSV para fácil procesamiento
            sprintf(uartBuffer, "DATA,%lu.%02lu,%lu\r\n", 
                    flowRate_x100/100, flowRate_x100%100, currentPulseCount);
            UART_1_PutString(uartBuffer);
        }
        
        // Test simple del timer usando delay (solo para debug)
        CyDelay(2000); // 2 segundos
        timeCounter++;
        
        // Leer pulseCount de forma segura (solo para debug)
        CyGlobalIntDisable;
        uint32 currentPulseCount = pulseCount;
        CyGlobalIntEnable;
        
        // Debug: mostrar estado cada 2 segundos
        sprintf(uartBuffer, "DEBUG %lu: Pin=%d, Pulsos=%lu, Delta=%lu\r\n", 
                timeCounter, FlowSensor_Pin_Read(), currentPulseCount, 
                currentPulseCount - lastPulseCount);
        UART_1_PutString(uartBuffer);
        
        lastPulseCount = currentPulseCount;
        
        // Calcular y mostrar flujo si hay pulsos (debug)
        if(currentPulseCount > 0)
        {
            // Usar aritmética entera para evitar problemas con float
            // Fórmula: flowRate = (pulseCount / 7.5) * 0.85
            // Reorganizado: flowRate = (pulseCount * 0.85) / 7.5
            // Más seguro: flowRate = (pulseCount * 85) / (75 * 10) = (pulseCount * 85) / 750
            
            uint32 totalFlowRate_x100 = (currentPulseCount * 85) / 75; // x100 para 2 decimales
            uint32 deltaFlowRate_x100 = ((currentPulseCount - lastPulseCount) * 85) / 75;
            
            sprintf(uartBuffer, "FLUJO Total: %lu.%02lu L/min, Instantaneo: %lu.%02lu L/min\r\n", 
                    totalFlowRate_x100/100, totalFlowRate_x100%100,
                    deltaFlowRate_x100/100, deltaFlowRate_x100%100);
            UART_1_PutString(uartBuffer);
        }
    }
}

/* [] END OF FILE */