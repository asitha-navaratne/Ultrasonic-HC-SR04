#define F_CPU 8000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define SENSOR_DDR 		DDRD						///< DDR of ultrasonic sensor.
#define IND_DDR			DDRB						///< DDR of indicator LED.
#define SENSOR_PORT		PORTD						///< Port of ultrasonic sensor.
#define IND_PORT		PORTB						///< Port of indicator LED.
#define TRIG_PIN		PD3							///< TRIG pin of ultrasonic sensor.
#define ECHO_PIN		PD2							///< ECHO pin of ultrasonic sensor.
#define IND_PIN			PB0							///< Pin for indicator LED.

void PORT_INIT(void);
void INT_INIT(void);
void USART_INIT(void);
void USART_SEND(unsigned char character);

volatile uint16_t count = 0;
volatile uint8_t toggle = 0;						///< Variable to determine the toggle state of ECHO pin.
uint16_t distance;

uint8_t UBRR = 51;									///< Value to be changed to specify the baudrate for the particular operating clock frequency.

/*!
 *	@brief ISR for INT0 external interrupt; start or stop timer/counter1 when ECHO pulse changes state.
 */

ISR(INT0_vect){
	if(!(toggle)){
		TCCR1B |= (1<<CS10);						///< Start timer/counter1.
		toggle = 1;
	}
	else{
		TCCR1B = 0;									///< Stop timer/counter1.
		count = TCNT1;
		TCNT1 = 0;									///< Reset timer/counter1 value to 0.
		toggle = 0;
	}
}

int main(void){
	PORT_INIT();
	USART_INIT();
	INT_INIT();
	
	sei();											///< Enable global interrupts.
	
	while(1){
		SENSOR_PORT |= (1<<TRIG_PIN);
		_delay_us(10);
		SENSOR_PORT &= ~(1<<TRIG_PIN);				///< Create a 10 us pulse at TRIG pin to transmit an ultrasonic pulse.
		
		distance = 0.034*(count*0.0625*0.5);		///< Calculate value of distance in cm.
		
		if(distance < 10){
			USART_SEND('O');
			IND_PORT |= (1<<IND_PIN);
		}
		else{
			USART_SEND('X');
			IND_PORT &= ~(1<<IND_PIN);
		}
		_delay_ms(150);
	}
}

/*!
 *	@brief Initialize Ports.
 */

void PORT_INIT(void){
	SENSOR_DDR &= ~(1<<ECHO_PIN);
	SENSOR_DDR |= (1<<TRIG_PIN);
	IND_DDR |= (1<<IND_PIN);
}

/*!
 *	@brief Initialize External Interrupt 0 at INT0.
 */

void INT_INIT(void){
	MCUCR |= (1<<ISC00);								///< Any logic change at INT0 generates an interrupt request.
	GICR |= (1<<INT0);									///< Enable INT0 external interrupt request.
}

/*!
 *	@brief Initialize USART.
 */

void USART_INIT(void){
	UCSRB |= (1<<TXEN);								///< Enable transmission over USART.
	UCSRC |= (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0);		///< Select register UCSRC and set transmission character size to 8 bits.
	UBRRL = UBRR;										///< Set UBRR value for specified baudrate at specified frequency.
}

/*!
 *	@brief Transmit a character over USART.
 *	@param Character to be transmitted (unsigned char).
 */

void USART_SEND(unsigned char character){
	while(!(UCSRA & (1<<UDRE)));						///< Wait until data register is empty.
	UDR = character;									///< Load character to be transmitted to data register.
	while(!(UCSRA & (1<<TXC)));						///< Wait until transmission is complete.
}