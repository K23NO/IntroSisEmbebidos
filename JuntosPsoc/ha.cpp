int main(void)
{
    CyGlobalIntEnable; // Habilitar interrupciones globales

    // --- Variables para flujo ---
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

            // Calcular y almacenar flujo
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

                // Calcular SpO2 cuando hay suficientes muestras
                if (spo2Buffer.count >= 25) {
                    spo2_calculate();
                    lastSpO2 = (int)spO2;
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