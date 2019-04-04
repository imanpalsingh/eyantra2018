

#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

//UART Globals
#define RX (1<<4)
#define TX (1<<3)
#define TE (1<<5)
#define RE (1<<7)

// flag for if reached at node
int ReachedNodeflag = 0;

//Variable to hold Character received from UART
volatile unsigned char Data;

volatile unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder 
volatile unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
unsigned int Degrees; //to accept angle in degrees for turning

//Defining Thresholds

//Black line Threshold
#define THRESHOLD 47

//Black Node Threshold
#define THRESHOLDNODE 49



//Defining functions
void port_init();
void timer5_init();
void velocity(unsigned char, unsigned char);
unsigned char ADC_Conversion(unsigned char);

// Global variables
unsigned char ADC_Value;
unsigned char flag = 0;
unsigned char Left_White_Line = 0;
unsigned char Center_White_Line = 0;
unsigned char Right_White_Line = 0;

/*
 * Function Name :				magnet_pin_config
 * Input :						NONE
 * Output :						Register DDRH setting pin H0 as Output Port, Logic Low supplied to pin H0
 * Logic :						Masking of previous value of DDRH is done by bitwise 'or' operator with 0x01. This sets only Pin H0 as output Pin
 *								Masking of previous value of PORTH is done by bitwise 'and' operator with 0x01. Which sends logic LOW to H0  
 * Example call :				magnet_pin_config()
 * 
 */
void magnet_pin_config()
{   //Setting H0 as output pin 	
	DDRH = DDRH | 0x01 ;
	//Magnet is set to be turned off initially by sending logic zero
	PORTH = PORTH & 0xFE;
}

/*
 * Function Name :				magnet_on
 * Input :						NONE
 * Output :						Logic HIGH at Pin H0
 * Logic :						PORTH is assigned to 0x01 which makes Pin H0 to have a Logic HIGH
 * Example call :				magnet_on()
 * 
 */

void magnet_on()
{
	PORTH = 0x01;
}

/*
 * Function Name :				magnet_off
 * Input :						NONE
 * Output :						Logic LOW at Pin H0
 * Logic :						PORTH is assigned to 0xFE which makes Pin H0 to have a Logic LOW
 * Example call :				magnet_off()
 * 
 */
void magnet_off()
{
	PORTH = 0xFE;
}



//Servo related functions

/*
 * Function Name :			servo1_pin_config
 * Input :						NONE
 * Output :						Logic HIGH at Pin B5
 * Logic :						PORTB 5 is set as output pin .PORTB is assigned to 0x20 which makes Pin B5 to have a Logic HIGH
 * Example call :				servo1_pin_config
 * 
 */

void servo1_pin_config (void)
{
	DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
	PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
}

/*
 * Function Name :			timer1_init
 * Input :						NONE
 * Output :						NONE
 * Logic :						TIMER1 initialization in 10 bit fast PWM mode,prescale:256, PWM 10bit fast, TOP=0x03FF, actual value: 52.25Hz
 * Example call :				timer1_init
 * 
 */


void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
 TCNT1L = 0x01;	//Counter low value to which OCR1xH value is to be compared with
 OCR1AH = 0x03;	//Output compare Register high value for servo 1
 OCR1AL = 0xFF;	//Output Compare Register low Value For servo 1
 OCR1BH = 0x03;	//Output compare Register high value for servo 2
 OCR1BL = 0xFF;	//Output Compare Register low Value For servo 2
 OCR1CH = 0x03;	//Output compare Register high value for servo 3
 OCR1CL = 0xFF;	//Output Compare Register low Value For servo 3
 ICR1H  = 0x03;	
 ICR1L  = 0xFF;
 TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 					For Overriding normal port functionality to OCRnA outputs.
				  {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR1C = 0x00;
 TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}

/*
 * Function Name :			servo_1
 * Input :						NONE
 * Output :						NONE
 * Logic :						Setting the required Servo registers
 * Example call :				servo_1()
 * 
 */

void servo_1(unsigned char degrees)
{
	float PositionPanServo = 0;
	PositionPanServo = ((float)degrees / 1.86) + 35.0;
	OCR1AH = 0x00;
	OCR1AL = (unsigned char) PositionPanServo;
}

/*
 * Function Name :			servo_1_free
 * Input :						NONE
 * Output :						NONE
 * Logic :						makes servo 1 free rotating
 * Example call :				servo_1_free()
 * 
 */

void servo_1_free (void) 
{
	OCR1AH = 0x03;
	OCR1AL = 0xFF; //Servo 1 off
}

// White line Sensor , Analog to digital converter functions

/*
 * Function Name :			adc_pin_config
 * Input :					None
 * Output :					None
 * Logic :					Configures pins of port F for ADC (as input)
 * Example Call :			adc_pin_config()
 *
 */
void adc_pin_config (void)
{
	DDRF = 0x00;
	PORTF = 0x00;
}


/*
 * Function Name :			ADC_Conversion
 * Input :					ch - > channel
 * Output :					a - > Anallog value
 * Logic :					Function accepts the Channel Number and returns the corresponding Analog Value
 * Example Call :			ADC_Conversion()
 *
 */



unsigned char ADC_Conversion(unsigned char Ch)
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;
	ADMUX= 0x20| Ch;
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a; // Digital value of the analog input
}

/*
 * Function Name :			adc_init
 * Input :					None
 * Output :					None
 * Logic :					Functional resets/sets required ports/registers for ADC
 * Example Call :			adc_init()
 *
 */

void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//V ref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}






//Buzzer related functions

/*
 * Function Name :			buzzer_pin_config
 * Input :					None
 * Output :					None
 * Logic :					Sets the pin j0 as output pin and initially it isn't supplied 5v
 * Example Call :			buzzer_pin_config()
 *
 */


void buzzer_pin_config(void)
{
	DDRJ = DDRJ | 0x01;
	PORTJ = PORTJ & 0xFE;
}

/*
 * Function Name :			buzzer_on
 * Input :					None
 * Output :					None
 * Logic :					Supplies Logic High pin j0
 * Example Call :			buzzer_on()
 *
 */

void buzzer_on(void)
{
	PORTJ = 0x01;
}

/*
 * Function Name :			buzzer_off
 * Input :					None
 * Output :					None
 * Logic :					Sends Logic Low to pin j0
 * Example Call :			buzzer_off()
 *
 */
void buzzer_off(void)
{   PORTJ = 0xFE;
}


/*
 * Function Name :				uart0_init
 * Input :						NONE 
 * Output :						None
 * Logic :						Setting up UART for serial communication at baud rate 9600
 * Example call :				uart0_init()
 * 
 */


void uart0_init()
{
	UCSR0B = 0x00;							//disable while setting baud rate
	UCSR0A = 0x00;
	UCSR0C = 0x06;
	UBRR0L = 0x5F; 							//9600BPS at 14745600Hz
	UBRR0H = 0x00;
	UCSR0B = 0x98;
	//UCSR0C = 3<<1;							//setting 8-bit character and 1 stop bit
	//UCSR0B = RX | TX;
}


/*
 * Function Name :				ISR
 * Input :						NONE 
 * Output :						None
 * Logic :						Handling all sending and receiving interrupts
 * Example call :			Called automatically
 * 
 */

ISR(USART0_RX_vect)
{
	Data = UDR0;
	
	
}

/*
 * Function Name :				motor_pin_config
 * Input :						NONE
 * Output :						Register DDRA setting pins A0 to A3 as Output Port, Logic Low supplied to pins A0 to A3
 * Logic :						Masking of previous value of DDRA is done by bitwise 'or' operator with 0x0F. This sets only Pins A0 to A3  as output Pin
 *								Masking of previous value of PORTA is done by bitwise 'and' operator with 0xF0. Which sends logic LOW to pins A0 to A3
 *								Similarly DDRl and PORTL are assigned with 0x18 which sets pins L3 L4 as output pin and logic HIGH is supplied to them 
 *								DDRE is and PORTE registers are assigned with 0xF0  which makes pins from A4 to A7 as output pins and Having Logic High
 * Example call :				motor_pin_config()
 * 
 */
void motion_pin_config (void) 
{
 DDRA = DDRA | 0x0F;
 PORTA = PORTA & 0xF0;
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}



/*
 * Function Name :			left_encoder_pin_config
 * Input :					None
 * Output :					None
 * Logic :					Function to configure INT4 (PORTE 4) pin as input for the left position encoder
 * Example Call :			left_encoder_pin_config()
 *
 */
void left_encoder_pin_config (void)
{
 DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
 PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}

/*
 * Function Name :			right_encoder_pin_config
 * Input :					None
 * Output :					None
 * Logic :					Function to configure INT5 (PORTE 5) pin as input for the right position encoder
 * Example Call :			right_encoder_pin_config()
 *
 */


void right_encoder_pin_config (void)
{
 DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 5 pin as input
 PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 5 pin
}

/*
 * Function Name :				port_init
 * Input :						NONE 
 * Output :						None
 * Logic :						A function to call all the pin configuration functions related to the ports
 * Example call :				port_init()
 * 
 */

void port_init()
{
 motion_pin_config(); //robot motion pins config
 left_encoder_pin_config(); //left encoder pin config
 right_encoder_pin_config(); //right encoder pin config	
 servo1_pin_config();
 magnet_pin_config();
 
}

/*
 * Function Name :			left_position_encoder_interrupt_init
 * Input :					None
 * Output :					None
 * Logic :					Function to enable Interrupt 4
 * Example Call :			 left_position_encoder_interrupt_init()
 *
 */


void left_position_encoder_interrupt_init (void) 
{
 cli(); //Clears the global interrupt
 EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
 EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
 sei();   // Enables the global interrupt 
}

/*
 * Function Name :			right_position_encoder_interrupt_init
 * Input :					None
 * Output :					None
 * Logic :					Function to enable Interrupt 5
 * Example Call :			right_position_encoder_interrupt_init()
 *
 */

void right_position_encoder_interrupt_init (void) 
{
 cli(); //Clears the global interrupt
 EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
 EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
 sei();   // Enables the global interrupt 
}


/*
 * Function Name :				ISR
 * Input :						NONE 
 * Output :						None
 * Logic :						ISR for right position encoder
 * Example call :			Called automatically
 * 
 */

ISR(INT5_vect)  
{
 ShaftCountRight++;  //increment right shaft position count
}

/*
 * Function Name :				ISR
 * Input :						NONE 
 * Output :						None
 * Logic :						ISR for left position encoder
 * Example call :			Called automatically
 * 
 */


ISR(INT4_vect)
{
 ShaftCountLeft++;  //increment left shaft position count
}




/*
 * Function Name :				motion_set
 * Input :						Direction - > Hexadecimal values on directions according to pins
 * Output :						PORTA = Direction in which the Motors have to move
 * Logic :						The Upper nibble is removed as it is not required  from the Direction. The PORTA's original is assigned to variable PORTA_RESTORE, 
 *								Lower nibble of the PORTA_RETORE is set to 0 and direction is added to the lower nibble. The current value of the variable PORTA_RESTORE
 *								is assigned to PORTA 
 * Example call :				motion_set(0x0F)
 * 
 */

void motion_set (unsigned char Direction)
{
 unsigned char PortARestore = 0;

 Direction &= 0x0F; 		// removing upper nibbel for the protection
 PortARestore = PORTA; 		// reading the PORTA original status
 PortARestore &= 0xF0; 		// making lower direction nibbel to 0
 PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
 PORTA = PortARestore; 		// executing the command
}


/*
 * Function Name :				forward
 * Input :						NONE
 * Output :						Call to function motion_set with Direction parameters ,call to function velocity with values of velocities of left and right motor
 * Logic :						A function call to motion_set(ox06) is made passing values of directions which makes the motors move in forward direction 
 * Example call :				forward()
 * 
 */

void forward (void) //both wheels forward
{
  motion_set(0x06);
}
/*
 * Function Name :				back
 * Input :						NONE
 * Output :						Call to function motion_set with Direction parameters ,call to function velocity with values of velocities of left and right motor
 * Logic :						A function call to motion_set(ox09) is made passing values of directions which makes the motors move in backward direction 
 * Example call :				back()
 * 
 */


void back (void) //both wheels backward
{
  motion_set(0x09);
}

/*
 * Function Name :				left
 * Input :						NONE
 * Output :						Call to function motion_set with Direction parameters, call to function velocity with values of velocities of left and right motor
 * Logic :						A function call to motion_set(0x05) is made passing values of directions, which makes the left motor to move backwards and right motor to move forward
 * Example call :				left()
 * 
 */

void left (void) //Left wheel backward, Right wheel forward
{
  motion_set(0x05);
}

/*
 * Function Name :				right
 * Input :						NONE
 * Output :						Call to function motion_set with Direction parameters, call to function velocity with values of velocities of left and right motor
 * Logic :						A function call to motion_set(0x0A) is made passing values of directions, , which makes the left motor move upward and right motor in backwards direction
 * Example call :				right()
 * 
 */

void right (void) //Left wheel forward, Right wheel backward
{
  motion_set(0x0A);
}

/*
 * Function Name :			soft_left
 * Input :						NONE
 * Output :						Call to function motion_set with Direction parameters, call to function velocity with values of velocities of left and right motor
 * Logic :						A function call to motion_set(0x04) is made passing values of directions, , which makes left wheel stationary, Right wheel forward
 * Example call :				soft_left()
 * 
 */

void soft_left (void) 
{
 motion_set(0x04);
}

/*
 * Function Name :			soft_left
 * Input :						NONE
 * Output :						Call to function motion_set with Direction parameters, call to function velocity with values of velocities of left and right motor
 * Logic :						A function call to motion_set(0x02) is made passing values of directions, , which makes left wheel forward, Right wheel is stationary
 * Example call :				soft_right()
 * 
 */
void soft_right (void) 
{
 motion_set(0x02);
}


/*
 * Function Name :				stop
 * Input :						NONE 
 * Output :						Call to function motion_set with Direction parameters, call to function velocity with values of velocities of left and right motor
 * Logic :						A function call to motion_set(Direction) is made passing 0x00 to make the motors stop. 0x00 Makes all the output pins to get Logic LOW
 * Example call :				stop()
 * 
 */

void stop (void)
{
  motion_set(0x00);
}


/*
 * Function Name :			angle_rotate
 * Input :					Degrees - > The degrees to move the bot to
 * Output :					None
 * Logic :					Function used for turning robot by specified degrees
 * Example Call :			angle_rotate(90)
 *
 */
void angle_rotate(unsigned int Degrees)
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;

 ReqdShaftCount = (float) Degrees/ 0.3721; // division by resolution to get shaft count
 ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
 ShaftCountRight = 0; 
 ShaftCountLeft = 0; 

 while (1)
 {
  if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
  break;
 }
 stop(); //Stop robot
}

/*
 *
 * Function Name :				timers_init
 * Input :						NONE
 * Output :						Timer 5 initialized in PWM mode for velocity control
 * Logic :						To program PWM 4 registers are needed to be initialized. 
 *								TCCR5A - Timer counter control register A. Used to configure timer for PWM generation
 *								TCCR5B - Timer counter control register B. Used to control the timer.
 *								TCNT5  - 16 bit timer counter. Used to count up/down according to clock frequency
 *								OCR5N  - 16 bit output compare register. Compares itself to TCNT5 and sets flags acoordingly. 
 * Example call :				timers_init()
 *
 */


void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}


/*
 * Function Name :				velocity
 * Input :						unsigned char left_motor - > velocity of left motor; unsigned char right_motor - > velocity of right motor
 * Output :						Different values of voltage to the motors
 * Logic :						OCR5AL  is a 16 bit output compare register. It compares itself from TCNT counter and sets flags when match occurs, indicating overflow.
 * Example call :				velocity
 * 
 */



void velocity (unsigned char right_motor, unsigned char left_motor)
{
	OCR5AL = (unsigned char)right_motor;
	OCR5BL = (unsigned char)left_motor;
}
/*
 * Function Name :			linear_distance_mm
 * Input :					DistanceInMM - > The distance to move the bot to
 * Output :					None
 * Logic :					Function used for moving robot in a direction by specified distance
 * Example Call :			linear_distance_mm(100)
 *
 */

void linear_distance_mm(unsigned int DistanceInMM)
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;

 ReqdShaftCount = DistanceInMM / 0.3721; // division by resolution to get shaft count
 ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
  
 ShaftCountRight = 0;
 while(1)
 {
  if(ShaftCountRight > ReqdShaftCountInt)
  {
  	break;
  }
 } 
 stop(); //Stop robot
}

/*
 * Function Name :			forward_mm
 * Input :					DistanceInMM - > The distance to move the bot to
 * Output :					None
 * Logic :					Function used for moving robot forward by specified distance
 * Example Call :			forward_mm(100)
 *
 */

void forward_mm(unsigned int DistanceInMM)
{
 forward();
 linear_distance_mm(DistanceInMM);
}

/*
 * Function Name :			back_mm
 * Input :					DistanceInMM - > The distance to move the bot to
 * Output :					None
 * Logic :					Function used for moving robot Backward by specified distance
 * Example Call :			back_mm(100)
 *
 */
void back_mm(unsigned int DistanceInMM)
{
 back();
 linear_distance_mm(DistanceInMM);
}

/*
 * Function Name :			 left_degrees
 * Input :					Degrees - > The degrees to move the bot to
 * Output :					None
 * Logic :					Function used for moving robot  by some degree from the left
 * Example Call :			left_degrees(90)
 *
 */

void left_degrees(unsigned int Degrees) 
{
// 88 pulses for 360 degrees rotation 4.090 degrees per count
 left(); //Turn left
 angle_rotate(Degrees);
}




/*
 * Function Name :				WhichRotation
 * Input :						Where - > data received through the uart 
 * Output :						Bot movements according to the data received 
 * Logic :						 - 
 * Example call :				WhichRotation('r')
 * 
 */

void WhichRotation(char Where)
{	
	
	//If the instruction is to move right
	if (Where =='r')
	{
		
	// Velocity of the tires	
	velocity(150,150);
	
	//Movement of the bot
	while(1)
	{
		
		//Getting line sensor readings
		Left_White_Line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_White_Line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_White_Line = ADC_Conversion(1);	//Getting data of Right WL Sensor
		
		//Setting motors to move as right
		right(); 
		
		//Making sure it moves and stops on the black line
		
		//IF threshold is achieved at all the three sensors
		if((Left_White_Line <= THRESHOLD) && (Center_White_Line >= THRESHOLD) && (Right_White_Line <= THRESHOLD))
		{
			//Put the velocities of motor as 0
			velocity(0,0);
			
			
			
			//Break Out of the loop as Movement is successful
			break;
		}
		
		
	}
	
	}
	
	else if (Where =='b')
	{
		
		// Velocity of the tires
		velocity(150,150);
		
		//Movement of the bot
		while(1)
		{
			
			//Getting line sensor readings
			Left_White_Line = ADC_Conversion(3);	//Getting data of Left WL Sensor
			Center_White_Line = ADC_Conversion(2);	//Getting data of Center WL Sensor
			Right_White_Line = ADC_Conversion(1);	//Getting data of Right WL Sensor
			
			//Setting motors to move as backwards
			back();
			
			//Making sure it moves and stops on the black line
			
			//IF threshold is achieved at all the three sensors
			if((Left_White_Line <= THRESHOLD) && (Center_White_Line >= THRESHOLD) && (Right_White_Line <= THRESHOLD))
			{
				//Put the velocities of motor as 0
				velocity(0,0);
				
				
				
				//Break Out of the loop as Movement is successful
				break;
			}
			
			
		}
		
	}

	
	else if (Where =='l')
	{
		//Setting the velocities
		velocity(150,150);
		
		//Movement of the bot
		while(1)
		{
			//Getting line sensor readings
			Left_White_Line = ADC_Conversion(3);	//Getting data of Left WL Sensor
			Center_White_Line = ADC_Conversion(2);	//Getting data of Center WL Sensor
			Right_White_Line = ADC_Conversion(1);	//Getting data of Right WL Sensor
			
			//Moving bot left
			left(); 
			
			//till back line is at center 
			if((Left_White_Line <= THRESHOLD) && (Center_White_Line >= THRESHOLD) && (Right_White_Line <= THRESHOLD))
			{
				
				velocity(0,0);
				break;
			}
			
		}
		
	}
	
	
	
  
	  
	  
	  
	  
  }



/*
 * Function Name :				Bot_movement_common
 * Input :						NONE 
 * Output :						None
 * Logic :						When call will made the bot move on the perfect black lines according to the threshold calculated
 * Example call :				Bot_movement_common()
 * 
 */



void Bot_movement_common(void)
{
	//Setting velocities
	velocity(176,180);
	
	//Moving the bot
	while(1)
	{
		// Getting Sensor readings
		Left_White_Line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_White_Line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_White_Line = ADC_Conversion(1);	//Getting data of Right WL Sensor

		//Moving Forward
		forward();
		
		// IF reached on a node
		if((Left_White_Line >= THRESHOLDNODE) && (Center_White_Line >= THRESHOLDNODE) && (Right_White_Line >= THRESHOLDNODE))
		{
			// Waiting to go a bit ahead of the node to take successful turns
			
			//Stopping the bot and ending the moment
			
			
				_delay_ms(60);
			
			velocity(0,0);
			break;
		}
		
		//If Black line shifts towards left
		if((Left_White_Line<=THRESHOLD) && (Center_White_Line <= THRESHOLD )&& (Right_White_Line >= THRESHOLD))
		{
			velocity(180,180);
			left();
		}

		//If Black line shifts a little towards left
		if((Left_White_Line<=THRESHOLD) && (Center_White_Line >=THRESHOLD) && (Right_White_Line >= THRESHOLD))
		{
			velocity(180,180);
			soft_left();
			
		}
		//If Black line is perfectly below the center sensor
		if((Left_White_Line <= THRESHOLD) && (Center_White_Line >= THRESHOLD) && (Right_White_Line <= THRESHOLD))
		{	
			velocity(150,150);
		}
		//If Black line shifts towards left
		if((Left_White_Line>=THRESHOLD) && (Center_White_Line<=THRESHOLD) && (Right_White_Line<=THRESHOLD))
		{
			velocity(180,180);
			right();
		}
		//If Black line shifts a little towards left
		if(Left_White_Line>= THRESHOLD && Center_White_Line >=  THRESHOLD && (Right_White_Line <= THRESHOLD))
		{	velocity(180,180);
			soft_right();
		}
		
		
	   

	
}
	}

	/*
 * Function Name :				Bot_movement_common2
 * Input :						NONE 
 * Output :						None
 * Logic :						When call will made the bot move on the perfect black lines according to the threshold calculated (Difference is that is will stop the bot a little earlier when threshold is acheived on 3 sensors)
 * Example call :				Bot_movement_common2()
 * 
 */

void Bot_movement_common2(void)
{
	//Setting velocities
	velocity(95,108);
	
	//Moving the bot
	while(1)
	{
		// Getting Sensor readings
		Left_White_Line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_White_Line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_White_Line = ADC_Conversion(1);	//Getting data of Right WL Sensor

		//Moving Forward
		forward();
		
		// IF reached on a node
		if((Left_White_Line >= THRESHOLDNODE) && (Center_White_Line >= THRESHOLDNODE) && (Right_White_Line >= THRESHOLDNODE))
		{
			// Waiting to go a bit ahead of the node to take successful turns
			
			//Stopping the bot and ending the moment
			
			velocity(0,0);
			stop();
			break;
			
		}
		
		
		
		
		//If Black line shifts towards left
		if((Left_White_Line<=THRESHOLD) && (Center_White_Line <= THRESHOLD )&& (Right_White_Line >= THRESHOLD))
		{
			velocity(180,180);
			left();
		}

		//If Black line shifts a little towards left
		if((Left_White_Line<=THRESHOLD) && (Center_White_Line >=THRESHOLD) && (Right_White_Line >= THRESHOLD))
		{
			velocity(180,180);
			soft_left();
			
		}
		//If Black line is perfectly below the center sensor
		if((Left_White_Line <= THRESHOLD) && (Center_White_Line >= THRESHOLD) && (Right_White_Line <= THRESHOLD))
		{
			velocity(150,150);
		}
		//If Black line shifts towards left
		if((Left_White_Line>=THRESHOLD) && (Center_White_Line<=THRESHOLD) && (Right_White_Line<=THRESHOLD))
		{
			velocity(180,180);
			right();
		}
		//If Black line shifts a little towards left
		if(Left_White_Line>= THRESHOLD && Center_White_Line >=  THRESHOLD && (Right_White_Line <= THRESHOLD))
		{	velocity(180,180);
			soft_right();
		}
		
		
		

		
	} _delay_ms(1000);
}

/*
 * Function Name :				CheckPos
 * Input :						NONE 
 * Output :						None
 * Logic :						On taking U turns if a U turn (from left side) is not successfull, The bot will movw until it is on the line again
 * Example call :				CheckPos()
 * 
 */

void CheckPos(){
	
		// Velocity of the tires
		velocity(150,150);
		
		//Movement of the bot
		while(1)
		{
			
			//Getting line sensor readings
			Left_White_Line = ADC_Conversion(3);	//Getting data of Left WL Sensor
			Center_White_Line = ADC_Conversion(2);	//Getting data of Center WL Sensor
			Right_White_Line = ADC_Conversion(1);	//Getting data of Right WL Sensor
			
			
			
			//Setting motors to move as left
			left();
			
			//Making sure it moves and stops on the black line
			
			//IF threshold is achieved at all the three sensors
			if((Left_White_Line >= THRESHOLD) || (Center_White_Line >= THRESHOLD) || (Right_White_Line >= THRESHOLD))
			{
				//Put the velocities of motor as 0
				velocity(0,0);
				
				
				
				//Break Out of the loop as Movement is successful
				break;
			}
			
			
		}
		
	}

/*
 * Function Name :				init_devices
 * Input :						NONE 
 * Output :						None
 * Logic :						Initialising all the ports and devices
 * Example call :			init_devices()
 * 
 */

void init_devices()
{
 cli(); //Clears the global interrupt
 port_init(); 
 timer1_init();
 timer5_init(); 
 buzzer_pin_config();
 left_position_encoder_interrupt_init();
 right_position_encoder_interrupt_init();
 uart0_init();
 adc_init();
 sei();   // Enables the global interrupt 
}

/*
 * Function Name :				main
 * Input :						NONE 
 * Output :						None
 * Logic :						performs required operations after receiving Data
 * Example call :			called automatically
 * 
 */

int main(void)
{
	//Variable for servo movement
	unsigned char i = 0;

	//Initializing the ports/devices
	init_devices();

	//Sending Signal to Python script stating bot has been turned on.
	UDR0 = 'M';
	
	//infinite loop
	while(1)
	{
		
	//If data received is 'p' i.e. bot has reached the first pebble
	if(Data == 'p')
	{ 
		
		//Upon reaching

		//power the electromagnet
		 magnet_on();
		_delay_ms(100);	
		
		//Set velocities and move bot ahead 4 cm to reach inside Ar_Object
		velocity(180,180);
		forward_mm(40); 

		//Stopping the pot for pebble pickup
		velocity(0,0);
		_delay_ms(2000);
		
		//Performing pebble pickup with servo
		 //Bringing the Electromagnet down
		 for (i = 33; i <53; i++)
 {
  	servo_1(i);
  	_delay_ms(30);

 }

 		_delay_ms(2000);

 		servo_1_free(); 
 		_delay_ms(1000);
 
//Bringing the electromagnet back up
 for (i = 52; i >34; i--)
 {
	 servo_1(i);
	 _delay_ms(30);

 }
 
	 servo_1_free(); 
 	_delay_ms(1000);

		//Sending signal of pebble pickup to python script
		UDR0 = 'q';
		_delay_ms(500);

		//Buzzer set to on for one second
		buzzer_on();
		_delay_ms(1000);
		buzzer_off();
		_delay_ms(1000);

		//Bringing bot Back on the line			
		velocity(180,180);
		back_mm(40);

		//Stoping the bot
		velocity(0,0);
		_delay_ms(100);

		//Resetting the data to avoid error
		Data = 0x00;
		_delay_ms(200);

		//Sending complete signal and requresting for new data
		UDR0 = 'p';
	}

	//If the bot picked up a pebble and has reached pitcher
	
	if(Data == 'P')
	{
		
		//Move bot ahead under the pitcher
		velocity(180,180);
		forward_mm(40);

		//Stoping the bot
		velocity(0,0);
		_delay_ms(2000);


		// Bringing electromagnet down
		 for (i = 33; i <53; i++)
 {
  servo_1(i);
  _delay_ms(30);

 }

 _delay_ms(2000);
  
 servo_1_free(); 

 //Releasing the pebble picked up
 magnet_off();
 _delay_ms(1000);
 
 //Bringing Electromagnet back up
 
 for (i = 52; i >34; i--)
 {
	 servo_1(i);
	 _delay_ms(30);

 }
 
 servo_1_free(); 
 _delay_ms(1000);

		//Sending signal of pebble drop to python script
		UDR0 = 'w';
		_delay_ms(500);

		//Buzzer on for 1 second
		buzzer_on();
		_delay_ms(1000);
		buzzer_off();
		_delay_ms(1000);

		//Moving bot back on the line
		velocity(180,180);
		back_mm(40);  //Moves robot backward 100mm

		//Stopping the bot
		velocity(0,0);
		_delay_ms(100);

		//Clearing the data to avoid error
		Data = 0x00;
		_delay_ms(200);

		//Sending complete signal and requesting for Data
		UDR0 = 'p';
		
		
		
	}

	//If the bot dropped the pebble in the pitcher and is going for the next pebble
	
	if (Data == 'O')
	{

		//upon reaching

		//Magnet is set to on
		magnet_on();
		_delay_ms(100);

		//Move bot ahead 4cm to bring the electromagnet under the Ar_Object
		velocity(180,180);
		forward_mm(40); 

		//Stopping the bot
		velocity(0,0);
		_delay_ms(2000);


		//Bringing the electromagnet down for pebble pickup
		 for (i = 33; i <53; i++)
 {
  servo_1(i);
  _delay_ms(30);

 }

 _delay_ms(2000);
  
 servo_1_free(); 
 _delay_ms(1000);
 
 //Bring the electromagnet back up
 
 for (i = 52; i >34; i--)
 {
	 servo_1(i);
	 _delay_ms(30);

 }
 
 servo_1_free(); 
 _delay_ms(1000);

		//Sending signal of Pebble pickup to python script
		UDR0 = 'e';
		_delay_ms(500);

		//Buzzer on for 1 second
		buzzer_on();
		_delay_ms(1000);
		buzzer_off();
		_delay_ms(1000);

		// Bringing bot back on the line
		velocity(180,180);
		back_mm(40);  

		// Stopping the bot
		velocity(0,0);
		_delay_ms(100);

		//Clearing the data for avoiding error
		Data = 0x00;
		_delay_ms(200);

		//Sending complete signal and requesting for new Data
		UDR0 = 'O';
		
		
		
		
		
		
	}
	
	//If the Command to move forward is received
	if (Data == 'f')
	{	
			//Move forward function call
			Bot_movement_common();

			//Clearing data for any error
			Data = 0x00;
			_delay_ms(200);

			//Sending complete signal and requesting for new Data
			UDR0 = 'f';
			
			
			
		
	}

	//If the Command to move right is received
	
	if (Data == 'r')
	{
		//Move right 
		WhichRotation('r');

		//Move forward to reach next node
		Bot_movement_common();
		    _delay_ms(100);

			//Ceearing data for avoiding erros	
			Data = 0x00;
			_delay_ms(200);

			//Sending Complete signal and requesting for new Data
			UDR0 = 'r';
		
	}

	//If the command to move left is received
	 if (Data == 'l')
	{
		//Move left
		WhichRotation('l');

		//Forward till next node
	Bot_movement_common();
	_delay_ms(100);

		//Clearing Data to avoid any error
		Data = 0x00;
		_delay_ms(200);

		//Sending Complete signal and requesting for new Data
		UDR0 = 'l';
	}

	//If Command to take a U turn is received
	
	if (Data == 'U')
	{	
		//Moving Bot 180 degrees
		velocity(250,250);
		//180 is achieved when 210 is given
		left_degrees(210);

		//Stopping the bot
		velocity(0,0);
		_delay_ms(1000);

		//Making sure that after rotating the bot is currently now on a black line
		CheckPos();

		//Moving bot till next node
		Bot_movement_common();

		//Clearing Data for error correction
		Data = 0x00;
		_delay_ms(200);

		//Sending complete signal and requesting for new data
		UDR0 = 'U';
	}
	
	//If bot has reached its final destination
	if (Data == 'S')
	{
		//Stop the bot
		stop();
		velocity(0,0);

		//Buzzer on for 5 seconds
		buzzer_on();
		_delay_ms(5000);
		buzzer_off();

		//resetting data to avoid re running
		Data = 0x00;
	}
	
	//If Command to go right (also if this Movement leads to a pebble or a pitcher)
	if(Data == 'R')
	{
		//Go right
		WhichRotation('r');

		//Move forward and stop early
		Bot_movement_common2();

		//Resetting Data to avoid errors
		Data = 0x00;
		_delay_ms(200);

		//Sending Complete signal and requesting for new Data
		UDR0 = 'R';
	}

//If Command to go left(also if this Movement leads to a pebble or a pitcher)	
	if(Data == 'L')
	{
		//Go left
		WhichRotation('l');

		//Move forward and stop early
		Bot_movement_common2();

		//Resetting Data to avoid errors
		Data = 0x00;
		_delay_ms(200);

		//Sending Complete signal and requesting for new Data
		UDR0 = 'L';
	}
	
	//If Command to go 180 degrees(also if this Movement leads to a pebble or a pitcher)	
	 if(Data == 'u')
	{
		//Turn 180 degrees
		velocity(250,250);
		left_degrees(210);

		//Stop the bot
		velocity(0,0);
		_delay_ms(1000);

		//Checking if after rotation the bot is on a back line or not
		 CheckPos();

		//Moving bot to the next node and stopping early
		Bot_movement_common2();

		//Resetting data to avoid errors
		Data = 0x00;
		_delay_ms(200);

		//Sending complete signal and requesting for new Data
		UDR0 = 'u';
	}
	
	
	//If signal of left is received but the next signal to be received is also left (Determined by python script)
	if(Data == 'k')
	{
		//Move left
		WhichRotation('l');

		//Move bot forward till next node
		Bot_movement_common();

		//resetting data to avoid error
		Data = 0x00;
		_delay_ms(200);

		//Sending Complete signal and requesting for new data
		UDR0 = 'k';
	}
	
	//If signal of right is received but the next signal to be received is also right (Determined by python script)
	if(Data == 't')
	{
		//Move right
		WhichRotation('r');

		//Move forward till next node
		Bot_movement_common();

		//Resetting data to avoid error
		Data = 0x00;
		_delay_ms(200);

		//Sending Complete signal and requesting for new data
		UDR0 = 't';
	}
	
	//If signal of movement of 180 degrees is received but the next signal to be received is also movement of 180 degrees (Determined by python script)
	if(Data == 'i')
	{

		//Move 180 degrees
		velocity(250,250);
		left_degrees(210);

		//Stop the bot
		velocity(0,0);
		_delay_ms(1000);

		//Check if bot after turn is currently on a back line
		 CheckPos();

		//Move bot forward till next node
		Bot_movement_common();

		//Resetting data to avoid error
		Data = 0x00;
		_delay_ms(200);

		//Sending Complete signal and requesting for new Data
		UDR0 = 'i';
	}
}

}
