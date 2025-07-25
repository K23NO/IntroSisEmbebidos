#define F_CPU 8000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <stdlib.h>

// Configuración para 1MHz

#define BAUD 9600
#define UBRR_VALUE ((F_CPU/(16UL*BAUD))-1)

// Dirección del LCD
#define LCD_ADDR 0x27
#define TWBR_VALUE ((F_CPU/100000UL-16)/2)  // Para 100kHz I2C a 1MHz

// Variables globales
volatile uint8_t cont = 0;
volatile uint8_t last_cont = 0xFF;
volatile uint8_t uart_flag = 0;

// Buffer para recepción UART
#define BUFFER_SIZE 64
char uart_buffer[BUFFER_SIZE];
volatile uint8_t buffer_index = 0;

// Variables para almacenar valores de sensores
char sensor1_value[16] = "0.00";
char sensor2_value[16] = "0.00";
char sensor3_value[16] = "0.00";

// Funciones I2C/TWI
void TWI_init(void);
void TWI_start(void);
void TWI_stop(void);
void TWI_write(uint8_t data);

// Funciones LCD
void lcd_init(void);
void lcd_command(uint8_t cmd);
void lcd_data(uint8_t data);
void lcd_send(uint8_t value, uint8_t mode);
void lcd_set_cursor(uint8_t row, uint8_t col);
void lcd_print(const char* msg);
void lcd_print_string(const char* str);

// Funciones UART
void uart_init(void);
void uart_transmit(char data);
void uart_send_string(const char* str);
void uart_send_newline(void);

// Funciones para mostrar sensores
void LCD_PRINT_sensor1(void);
void LCD_PRINT_sensor2(void);
void LCD_PRINT_sensor3(void);

// Función para procesar datos recibidos
void process_sensor_data(void);

// Interrupción INT0 para cambiar pantalla
ISR(INT0_vect) {
	cont++;
	if (cont > 2) cont = 0;
}

// Interrupción UART para recepción
ISR(USART_RX_vect) {
	char received_char = UDR0;
	
	// Echo del carácter recibido
	uart_transmit(received_char);
	
	if (received_char == '\n' || received_char == '\r') {
		uart_buffer[buffer_index] = '\0';
		uart_flag = 1;
		buffer_index = 0;
		} else if (buffer_index < BUFFER_SIZE - 1) {
		uart_buffer[buffer_index++] = received_char;
	}
}
const char msg3[] PROGMEM = "     HR: ";
const char msg4[] PROGMEM = "LATIDOS x MINUTO";

const char msg1[] PROGMEM = "FLUJO(L/m): ";
const char msg2[] PROGMEM = "TASA DE VOLUMEN ";

const char msg5[] PROGMEM = " SpO2(%): ";
const char msg6[] PROGMEM = "SAT. d OXIGENO";


void lcd_print_P(const char* msg) {
	char c;
	while ((c = pgm_read_byte(msg++))) {
		lcd_data(c);
	}
}


int main(void) {
	// Configurar INT0 en flanco de subida
	EICRA = (1 << ISC01) | (1 << ISC00);
	EIMSK = (1 << INT0);
	
	// Inicializar periféricos
	uart_init();
	TWI_init();
	lcd_init();
	
	sei();  // Habilitar interrupciones globales
	
	// Mostrar pantalla inicial
	LCD_PRINT_sensor1();
	last_cont = cont;
	
	while (1) {
		// Procesar datos UART si se recibieron
		if (uart_flag) {
			uart_flag = 0;
			process_sensor_data();
			uart_send_newline();
		}
		
		// Cambiar pantalla LCD si se presionó INT0
		if (cont != last_cont) {
			last_cont = cont;
			
			lcd_command(0x01); // Limpiar pantalla
			_delay_ms(5);
			
			switch (cont) {
				case 0:
				LCD_PRINT_sensor1();
				break;
				case 1:
				LCD_PRINT_sensor2();
				break;
				case 2:
				LCD_PRINT_sensor3();
				break;
			}
		}
	}
}

void uart_init(void) {
	// Configurar velocidad de transmisión
	UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
	UBRR0L = (uint8_t)UBRR_VALUE;
	
	// Habilitar transmisor, receptor e interrupción RX
	UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0);
	
	// Formato: 8 bits de datos, 1 bit de parada
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void uart_transmit(char data) {
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = data;
}

void uart_send_string(const char* str) {
	while (*str) {
		uart_transmit(*str++);
	}
}

void uart_send_newline(void) {
	uart_transmit('\r');
	uart_transmit('\n');
}

void process_sensor_data(void) {
	char* token;
	char temp_buffer[BUFFER_SIZE];
	
	// Copiar buffer para no modificar el original
	strcpy(temp_buffer, uart_buffer);
	
	uart_send_string("Datos recibidos: ");
	uart_send_string(uart_buffer);
	uart_send_newline();
	
	// Parsear primer valor (sensor 1)
	token = strtok(temp_buffer, ";");
	if (token != NULL) {
		strncpy(sensor1_value, token, 15);
		sensor1_value[15] = '\0';
		uart_send_string("Sensor 1: ");
		uart_send_string(sensor1_value);
		uart_send_newline();
	}
	
	// Parsear segundo valor (sensor 2)
	token = strtok(NULL, ";");
	if (token != NULL) {
		strncpy(sensor2_value, token, 15);
		sensor2_value[15] = '\0';
		uart_send_string("Sensor 2: ");
		uart_send_string(sensor2_value);
		uart_send_newline();
	}
	
	// Parsear tercer valor (sensor 3)
	token = strtok(NULL, ";");
	if (token != NULL) {
		strncpy(sensor3_value, token, 15);
		sensor3_value[15] = '\0';
		uart_send_string("Sensor 3: ");
		uart_send_string(sensor3_value);
		uart_send_newline();
	}
	
	// Actualizar pantalla actual
	lcd_command(0x01); // Limpiar pantalla
	_delay_ms(5);
	
	switch (cont) {
		case 0:
		LCD_PRINT_sensor1();
		break;
		case 1:
		LCD_PRINT_sensor2();
		break;
		case 2:
		LCD_PRINT_sensor3();
		break;
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

void lcd_print_string(const char* str) {
	while (*str) {
		lcd_data(*str++);
	}
}

void LCD_PRINT_sensor1(void) {
	lcd_set_cursor(0, 0);
	lcd_print_P(msg1);           // "     HR: "
	lcd_print_string(sensor1_value); // valor

	lcd_set_cursor(1, 0);
	lcd_print_P(msg2);           // "LATIDOS x MINUTO"
}


void LCD_PRINT_sensor2(void) {
	lcd_set_cursor(0, 0);
	lcd_print_P(msg3);           // "FLUJO(L/m): "
	lcd_print_string(sensor2_value); // valor

	lcd_set_cursor(1, 0);
	lcd_print_P(msg4);           // "TASA DE VOLUMEN"
}

void LCD_PRINT_sensor3(void) {
	lcd_set_cursor(0, 0);
	lcd_print_P(msg5);           // " SpO2(%): "
	lcd_print_string(sensor3_value); // valor

	lcd_set_cursor(1, 0);
	lcd_print_P(msg6);           // "SAT. d OXIGENO"
}