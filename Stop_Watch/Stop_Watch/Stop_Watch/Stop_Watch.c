/*
 * Final_pro_Inter_1.c
 *
 *  Created on: Sep 8, 2024
 *      Author: Mahmoud Ashraf
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define SET_BIT(REG,POS)     (REG |= (1<<POS))
#define CLEAR_BIT(REG,POS)   (REG &= ~(1<<POS))
#define READ_BIT(REG,POS)    ((REG>>POS)&(0X01))

unsigned char SECONDS = 0;
unsigned char MINUTES = 0;
unsigned char HOURS = 0;

unsigned char Pause_F = 0;
unsigned char Mode_F = 0;

unsigned char Sec_Inc_F = 0;
unsigned char Sec_Dec_F = 0;
unsigned char Min_Inc_F = 0;
unsigned char Min_Dec_F = 0;
unsigned char Hou_Inc_F = 0;
unsigned char Hou_Dec_F = 0;

/* -----prototype----- */
void Intialization_Application(void);
void Digital_Write(char pin, char Logic);
void Seven_Seg_Write_Num(unsigned char num_End_To_9);
void Cheak_Status(void); /*function check status of increment or decrement*/
void Configration_Bits(void);

void timer1_init(void) {
	/*set FOC1A for CTC mode No PWM*/
	TCCR1A |= (1 << FOC1A);
	/*Set Timer1 to CTC mode (WGM12 bit to 1)*/
	TCCR1B |= (1 << WGM12);
	/* Set Prescaler to 1024 (CS12 and CS10 bits to 1)*/
	TCCR1B |= (1 << CS12) | (1 << CS10);
	/*Set compare value for 1-second delay*/
	OCR1A = 15624;
	/*Enable Output Compare A Match Interrupt*/
	TIMSK |= (1 << OCIE1A);
}

/*Interrupt Service Routine for Timer1 Compare Match A*/
ISR(TIMER1_COMPA_vect) {
	/*increase SECONDS*/
	if (!(Pause_F)) {
		if (READ_BIT(PINB, 7)) {
			CLEAR_BIT(PORTD, PD0);
			SECONDS++;
			if (SECONDS == 60) {
				SECONDS = 0;
				MINUTES++;
			}
			if (MINUTES == 60) {
				MINUTES = 0;
				HOURS++;
			}
			if(HOURS == 24 ){
				HOURS = 0 ;
			}
		} else {

			if (SECONDS > 0 || MINUTES > 0 || HOURS > 0) {
				if (SECONDS == 0 && MINUTES > 0) {
					MINUTES--;
					SECONDS = 60 ;
				}
				if (MINUTES == 0 && SECONDS == 0 && HOURS > 0) {
					HOURS--;
					SECONDS = 60 ;
					MINUTES = 59 ;
				}
				SECONDS--;
			}
			if (0 == SECONDS && 0 == MINUTES && 0 == HOURS) {
				SET_BIT(PORTD, PD0);
			}

		}
	}
	/*END TIMER_ISR*/
}

/*Reset Button*/
void INT_0_reset_Init(void) {
	CLEAR_BIT(DDRD, 2); /*put pin2 in register_D as input for INT0 to reset Stopwatch*/
	SET_BIT(PORTD, PD2); /*force to use internal pull up*/
	/* set in Falling Edge*/
	MCUCR = (1 << ISC01);
	MCUCR &= ~(1 << ISC00);
	GICR |= (1 << INT0); /*Enable External Interrupt*/

}

/* External INT1 enable and configuration function */
void INT_1_Pause_Init(void) {
	CLEAR_BIT(DDRD, PD3);  // Configure INT1/PD3 as input pin
	/* Trigger INT1 with the Rising edge*/
	MCUCR |= (1 << ISC11) | (1 << ISC10);
	GICR |= (1 << INT1);    // Enable external interrupt pin INT1

}

/* External INT2 enable and configuration function */
void INT_2_Resume_Init(void) {
	CLEAR_BIT(DDRB, PB2); /*Configure INT2/PB2 as input pin*/
	SET_BIT(PORTB, PB2); /*Internal Pull Up*/

	MCUCSR &= ~(1 << ISC2); /*Trigger INT2 with the Falling edge*/
	GICR |= (1 << INT2); /*Enable external interrupt pin INT2*/
}

/* External INT0 Interrupt Service Routine */
ISR(INT0_vect) {
	SECONDS = 0;
	MINUTES = 0;
	HOURS = 0;
}

/* External INT1 Interrupt Service Routine */
ISR(INT1_vect) {
	/* Stop the clock*/
	Pause_F = 1;
}

/* External INT2 Interrupt Service Routine */
ISR(INT2_vect) {
	/*Run the clock*/
	Pause_F = 0;
}

int main(void) {
	/*Initialize Application*/
	Intialization_Application();
	/*Main loop*/
	while (1) {
		/*Seconds*/
		Seven_Seg_Write_Num((unsigned char) SECONDS % 10);
		PORTA = 0x20;
		_delay_ms(2);
		PORTA = 0x00;
		Seven_Seg_Write_Num((unsigned char) (SECONDS / 10));
		PORTA = 0x10;
		_delay_ms(2);
		PORTA = 0x00;
		/*Minutes*/
		Seven_Seg_Write_Num((unsigned char) (MINUTES % 10));
		PORTA = 0x08;
		_delay_ms(2);
		PORTA = 0x00;
		Seven_Seg_Write_Num((unsigned char) (MINUTES / 10));
		PORTA = 0x04;
		_delay_ms(2);
		PORTA = 0x00;
		/*Hours*/
		Seven_Seg_Write_Num((unsigned char) (HOURS % 10));
		PORTA = 0x02;
		_delay_ms(2);
		PORTA = 0x00;
		Seven_Seg_Write_Num((unsigned char) (HOURS / 10));
		PORTA = 0x01;
		_delay_ms(2);
		PORTA = 0x00;

		Cheak_Status();
	}
}

void Seven_Seg_Write_Num(unsigned char num_End_To_9) {
	Digital_Write(0, READ_BIT(num_End_To_9, 0));
	Digital_Write(1, READ_BIT(num_End_To_9, 1));
	Digital_Write(2, READ_BIT(num_End_To_9, 2));
	Digital_Write(3, READ_BIT(num_End_To_9, 3));
}

void Digital_Write(char pin, char Logic) {
	switch (Logic) {
	case 1:
		SET_BIT(PORTC, pin);
		break;
	case 0:
		CLEAR_BIT(PORTC, pin);
		break;
	}
}

void Configration_Bits(void) {
	/* Seconds increment*/
	CLEAR_BIT(DDRB, PB6);
	SET_BIT(PORTB, PB6);
	/* Seconds Decrement*/
	CLEAR_BIT(DDRB, PB6);
	SET_BIT(PORTB, PB5);
	/* Minutes increment*/
	CLEAR_BIT(DDRB, PB4);
	SET_BIT(PORTB, PB4);
	/* Minutes Decrement*/
	CLEAR_BIT(DDRB, PB3);
	SET_BIT(PORTB, PB3);
	/* Hours increment*/
	CLEAR_BIT(DDRB, PB1);
	SET_BIT(PORTB, PB1);
	/* Hours Decrement*/
	CLEAR_BIT(DDRB, PB0);
	SET_BIT(PORTB, PB0);
	/* LED Indicator*/
	SET_BIT(DDRD, PD4); /*RED  (Counting UP)*/
	SET_BIT(PORTD, PD4);
	SET_BIT(DDRD, PD5); /*YELLOW (Counting DOWN)*/
	CLEAR_BIT(PORTD, PD5);
	/*BUZZER Indicator*/
	SET_BIT(DDRD, PD0);
	CLEAR_BIT(PORTD, PD0);
	/* Set PORTC,PORTA as output*/
	DDRC = 0x3F;
	DDRA = 0x0F;
	DDRD &= ~(1 << PD2);
	/*Configuration BIT for Toggle Decrement OR Increment*/
	CLEAR_BIT(DDRB, PB7);
	SET_BIT(PORTB, PB7);
	/*Enable Global Interrupt*/
	SREG |= (1 << 7);
}
void Cheak_Status(void) {
	/*seconds adjustment*/
	if (READ_BIT(PINB,PB6) == 0) {
		if (Sec_Inc_F == 0) {
			SECONDS++;
			if (SECONDS == 60) {
				SECONDS = 0;
			}
			Sec_Inc_F = 1;
		}
	} else {
		Sec_Inc_F = 0;
	}
	if (READ_BIT(PINB,PB5) == 0) {
		if (Sec_Dec_F == 0) {
			if (SECONDS == 0) {
				SECONDS = 60;
			}
			SECONDS--;
			Sec_Dec_F = 1;
		}
	} else {
		Sec_Dec_F = 0;
	}

	/*minutes adjustment*/
	if (READ_BIT(PINB,PB4) == 0) {
		if (Min_Inc_F == 0) {
			MINUTES++;
			if (MINUTES == 60) {
				MINUTES = 0;
			}

			Min_Inc_F = 1;
		}
	} else {
		Min_Inc_F = 0;
	}
	if (READ_BIT(PINB,PB3) == 0) {
		if (Min_Dec_F == 0) {
			if (MINUTES == 0) {
				MINUTES = 60;
			}
			MINUTES--;
			Min_Dec_F = 1;
		}
	} else {
		Min_Dec_F = 0;
	}

	/*Hours adjustment*/
	if (READ_BIT(PINB,PB1) == 0) {
		if (Hou_Inc_F == 0) {
			HOURS++;
			if (HOURS == 24) {
				HOURS = 0;
			}
			Hou_Inc_F = 1;
		}
	} else {
		Hou_Inc_F = 0;
	}

	if (READ_BIT(PINB,PB0) == 0) {
		if (Hou_Dec_F == 0) {
			if (HOURS == 0) {
				HOURS = 24;
			}
			HOURS--;
			Hou_Dec_F = 1;
		}
	} else {
		Hou_Dec_F = 0;
	}
	/*LED Indicator (Count UP , Count DOWN)*/
	if (READ_BIT(PINB, PB7)) {
		SET_BIT(PORTD, PD4);
		CLEAR_BIT(PORTD, PD5);
	} else {
		SET_BIT(PORTD, PD5);
		CLEAR_BIT(PORTD, PD4);

	}

}
void Intialization_Application(void) {
	/*Initialize Timer1*/
	timer1_init();
	/*Initialize interrupt (INT0,INT1,INT2)*/
	INT_0_reset_Init();
	INT_1_Pause_Init();
	INT_2_Resume_Init();
	/*initialize Configration Bits*/
	Configration_Bits();

}
