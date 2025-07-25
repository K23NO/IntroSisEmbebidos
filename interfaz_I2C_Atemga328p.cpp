#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include <stdio.h>

// Dirección del LCD
#define LCD_ADDR 0x27
#define TWBR_VALUE 72

// Configuración de UART
#define BAUD 115200
#define UBRR_VALUE ((F_CPU / (16UL * BAUD)) - 1)

// Tamaño del buffer de recepción UART
#define RX_BUFFER_SIZE 64

// Variables globales
volatile uint8_t cont = 0;              // Contador para cambiar los mensajes
volatile uint8_t last_cont = 0xFF;      // Último valor del contador
volatile char rx_buffer[RX_BUFFER_SIZE]; // Buffer para recepción UART
volatile uint8_t rx_index = 0;          // Índice para el buffer
volatile uint8_t data_received = 0;     // Flag para indicar datos recibidos

// Valores medidos
float dpef_value = 0.0;
int bpm_value = 0;
int spo2_value = 0;

// Mensajes a mostrar (plantillas)
const char msg1_template[] PROGMEM = "     BPM: %3d     ";
const char msg2[] PROGMEM = "LATIDOS x MINUTO";

const char msg3_template[] PROGMEM = "FLUJO(L/s): %.2f";
const char msg4[] PROGMEM = " TASA DE VOLUMEN  ";

const char msg5_template[] PROGMEM = " SpO2(%%): %3d    ";
const char msg6[] PROGMEM = " SAT. d OXIGENO";

// Buffer para formatear mensajes
char display_buffer[20];


// Funciones principales
void TWI_init(void);
void TWI_start(void);
void TWI_stop(void);
void TWI_write(uint8_t data);

void lcd_init(void);
void lcd_command(uint8_t cmd);
void lcd_data(uint8_t data);
void lcd_send(uint8_t value, uint8_t mode);
void lcd_set_cursor(uint8_t row, uint8_t col);
void lcd_print(const char* msg);
void lcd_print_text(const char* msg);

void LCD_PRINT_msj1(void);
void LCD_PRINT_msj2(void);
void LCD_PRINT_msj3(void);

// Funciones para UART
void UART_init(void);
void parse_data(void);
void reset_values(void);

// Interrupción para cambio de mensaje
ISR(INT0_vect) {
    cont++;
    if (cont > 2) cont = 0;
}

// Interrupción para recepción UART
ISR(USART_RX_vect) {
    char data = UDR0;
    
    // Detectar el comando 'i' para iniciar la prueba
    if (data == 'i') {
        // Resetear los valores para iniciar una nueva prueba
        reset_values();
        rx_index = 0;
        return;
    }
    
    // Detectar el comando 'r' para resetear
    if (data == 'r') {
        reset_values();
        rx_index = 0;
        return;
    }
    
    // Almacenar datos en el buffer
    if (data == '\n' || data == '\r') {
        if (rx_index > 0) {
            rx_buffer[rx_index] = '\0';  // Terminar la cadena
            data_received = 1;           // Marcar que hay datos para procesar
            rx_index = 0;                // Resetear el índice para el próximo mensaje
        }
    } else {
        if (rx_index < RX_BUFFER_SIZE - 1) {
            rx_buffer[rx_index++] = data;
        }
    }
}

// Inicializar UART
void UART_init(void) {
    // Configurar baudrate
    UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
    UBRR0L = (uint8_t)UBRR_VALUE;
    
    // Habilitar receptor y transmisor, y habilitar interrupción de recepción
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
    
    // Configurar frame: 8 bits de datos, 1 bit de parada, sin paridad
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// Parsear datos recibidos en formato "DPEF;BPM;SpO2"
void parse_data(void) {
    if (!data_received) return;
    
    char *token;
    char *rest = (char*)rx_buffer;
    
    // Extraer DPEF (primer valor)
    token = strtok_r(rest, ";", &rest);
    if (token) {
        dpef_value = atof(token);
    }
    
    // Extraer BPM (segundo valor)
    token = strtok_r(rest, ";", &rest);
    if (token) {
        bpm_value = atoi(token);
    }
    
    // Extraer SpO2 (tercer valor)
    token = strtok_r(rest, ";", &rest);
    if (token) {
        spo2_value = atoi(token);
    }
    
    data_received = 0; // Limpiar flag de datos recibidos
}

// Resetear todos los valores
void reset_values(void) {
    dpef_value = 0.0;
    bpm_value = 0;
    spo2_value = 0;
}

// Función para imprimir texto normal (no desde PROGMEM)
void lcd_print_text(const char* msg) {
    char c;
    while ((c = *msg++) != 0) {
        lcd_data(c);
    }
}

int main(void) {
    // Configurar INT0 en flanco de subida (PD2)
    EICRA = (1 << ISC01) | (1 << ISC00);
    EIMSK = (1 << INT0);
    
    // Configurar PD2 como entrada con pull-up
    DDRD &= ~(1 << PD2);
    PORTD |= (1 << PD2);

    // Inicializar I2C, LCD y UART
    TWI_init();
    lcd_init();
    UART_init();

    sei();  // Habilitar interrupciones

    // Mensaje inicial
    LCD_PRINT_msj1();

    while (1) {
        // Procesar datos recibidos
        if (data_received) {
            parse_data();
        }
        
        // Actualizar pantalla si cambió el mensaje a mostrar
        if (cont != last_cont) {
            last_cont = cont;

            lcd_command(0x01); // Limpiar pantalla
            _delay_ms(5);

            switch (cont) {
                case 0:
                LCD_PRINT_msj1();
                break;
                case 1:
                LCD_PRINT_msj2();
                break;
                case 2:
                LCD_PRINT_msj3();
                break;
            }
        }
    }
}




void TWI_init(void) {
	TWSR = 0x00;
	TWBR = TWBR_VALUE;
	TWCR = (1 << TWEN);
}

void TWI_start(void) {
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
}

void TWI_stop(void) {
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
	while (TWCR & (1 << TWSTO));
}

void TWI_write(uint8_t data) {
	TWDR = data;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
}

void lcd_send(uint8_t value, uint8_t mode) {
	uint8_t high = (value & 0xF0) | mode | 0x08;
	uint8_t low = ((value << 4) & 0xF0) | mode | 0x08;

	TWI_start();
	TWI_write(LCD_ADDR << 1);

	// Enviar nibble alto con enable
	TWI_write(high | 0x04);
	TWI_write(high & ~0x04);

	// Enviar nibble bajo con enable
	TWI_write(low | 0x04);
	TWI_write(low & ~0x04);

	TWI_stop();
}

void lcd_command(uint8_t cmd) {
	lcd_send(cmd, 0);
	_delay_ms(2);
}

void lcd_data(uint8_t data) {
	lcd_send(data, 1);
	_delay_ms(2);
}

void lcd_init(void) {
	_delay_ms(50);

	lcd_command(0x02); // Modo 4-bit
	lcd_command(0x28); // 2 líneas, 5x8 dots
	lcd_command(0x0C); // Display ON, cursor OFF
	lcd_command(0x06); // Entrada modo automático
	lcd_command(0x01); // Clear display
	_delay_ms(5);
}

void lcd_set_cursor(uint8_t row, uint8_t col) {
	uint8_t addr = (row == 0) ? 0x80 : 0xC0;
	lcd_command(addr + col);
}

void lcd_print(const char* msg) {
	char c;
	while ((c = pgm_read_byte(msg++)) != 0) {
		lcd_data(c);
	}
}

void LCD_PRINT_msj1(void) {
    // Mostrar BPM
    lcd_set_cursor(0, 0);
    
    // Copiar plantilla de PROGMEM a RAM
    char format_buffer[20];
    strcpy_P(format_buffer, msg1_template);
    
    // Formatear mensaje con el valor real
    sprintf(display_buffer, format_buffer, bpm_value);
    lcd_print_text(display_buffer);
    
    lcd_set_cursor(1, 0);
    lcd_print(msg2);
}

void LCD_PRINT_msj2(void) {
    // Mostrar DPEF (Flujo)
    lcd_set_cursor(0, 0);
    
    // Copiar plantilla de PROGMEM a RAM
    char format_buffer[20];
    strcpy_P(format_buffer, msg3_template);
    
    // Formatear mensaje con el valor real
    sprintf(display_buffer, format_buffer, dpef_value);
    lcd_print_text(display_buffer);
    
    lcd_set_cursor(1, 0);
    lcd_print(msg4);
}

void LCD_PRINT_msj3(void) {
    // Mostrar SpO2
    lcd_set_cursor(0, 0);
    
    // Copiar plantilla de PROGMEM a RAM
    char format_buffer[20];
    strcpy_P(format_buffer, msg5_template);
    
    // Formatear mensaje con el valor real
    sprintf(display_buffer, format_buffer, spo2_value);
    lcd_print_text(display_buffer);
    
    lcd_set_cursor(1, 0);
    lcd_print(msg6);
}