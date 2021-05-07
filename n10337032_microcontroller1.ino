#include <avr/io.h>
#include <stdio.h>
#include <string.h>
#include <util/delay.h>
#include <inttypes.h>
#include <avr/interrupt.h>

/* CITATIONS 

This code uses codes from the lecture notes and AMS exercises 
Week 9 through 11 


LCD DEFINITION, (TOPIC 11: LECTUE NOTES)
SETTING_CLEARING REGISTERS, (TOPIC 7 : INTRO TO MICROCONTROLLERS)
INITIALIZATION ADC , (TOPIC 10 : lECTURE NOTES)
DEBOUNCING, (AMS WEEK 9 ) 
AUXILARY FUNCTIONS, (TOPIC 11, 8 : LECTUE NOTES)

  


*/ 






////////////////////////LCD DEFINITIONS //////////////////


// --== WIRING ==--
// LCD GND  -> GND
// LCD VCC  -> 5V
// LCD V0   -> GND
// LCD RW   -> GND
// LCD LED Anode    -> 220 Ohm -> 5V
// LCD LED Cathode  -> GND


#define LCD_USING_4PIN_MODE (1)

// #define LCD_DATA0_DDR (DDRD)
// #define LCD_DATA1_DDR (DDRD)
// #define LCD_DATA2_DDR (DDRD)
// #define LCD_DATA3_DDR (DDRD)
#define LCD_DATA4_DDR (DDRD)
#define LCD_DATA5_DDR (DDRD)
#define LCD_DATA6_DDR (DDRD)
#define LCD_DATA7_DDR (DDRD)


// #define LCD_DATA0_PORT (PORTD)
// #define LCD_DATA1_PORT (PORTD)
// #define LCD_DATA2_PORT (PORTD)
// #define LCD_DATA3_PORT (PORTD)
#define LCD_DATA4_PORT (PORTD)
#define LCD_DATA5_PORT (PORTD)
#define LCD_DATA6_PORT (PORTD)
#define LCD_DATA7_PORT (PORTD)

// #define LCD_DATA0_PIN (0)
// #define LCD_DATA1_PIN (1)
// #define LCD_DATA2_PIN (2)
// #define LCD_DATA3_PIN (3)
#define LCD_DATA4_PIN (4)
#define LCD_DATA5_PIN (5)
#define LCD_DATA6_PIN (6)
#define LCD_DATA7_PIN (7)


#define LCD_RS_DDR (DDRB)
#define LCD_ENABLE_DDR (DDRB)

#define LCD_RS_PORT (PORTB)
#define LCD_ENABLE_PORT (PORTB)

#define LCD_RS_PIN (1)
#define LCD_ENABLE_PIN (0)



// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

void lcd_init(void);
void lcd_write_string(uint8_t x, uint8_t y, char string[]);
void lcd_write_char(uint8_t x, uint8_t y, char val);


void lcd_clear(void);
void lcd_home(void);

void lcd_createChar(uint8_t, uint8_t[]);
void lcd_setCursor(uint8_t, uint8_t); 

void lcd_noDisplay(void);
void lcd_display(void);
void lcd_noBlink(void);
void lcd_blink(void);
void lcd_noCursor(void);
void lcd_cursor(void);
void lcd_leftToRight(void);
void lcd_rightToLeft(void);
void lcd_autoscroll(void);
void lcd_noAutoscroll(void);
void scrollDisplayLeft(void);
void scrollDisplayRight(void);

size_t lcd_write(uint8_t);
void lcd_command(uint8_t);


void lcd_send(uint8_t, uint8_t);
void lcd_write4bits(uint8_t);
void lcd_write8bits(uint8_t);
void lcd_pulseEnable(void);

uint8_t _lcd_displayfunction;
uint8_t _lcd_displaycontrol;
uint8_t _lcd_displaymode;

// END Definitions


/* Setting_Clearing Registers 

 *  Setting data directions in a data direction register (DDR)
 *
 *
 *  Setting, clearing, and reading bits in registers.
 *	reg is the name of a register; pin is the index (0..7)
 *  of the bit to set, clear or read.
 *  (WRITE_BIT is a combination of CLEAR_BIT & SET_BIT)
 */

#define SET_BIT(reg, pin)		    (reg) |= (1 << (pin))
#define CLEAR_BIT(reg, pin)		  (reg) &= ~(1 << (pin))
#define WRITE_BIT(reg, pin, value)   (reg) = (((reg) & ~(1 << (pin))) | ((value) << (pin)))
#define BIT_VALUE(reg, pin)		  (((reg) >> (pin)) & 1)
#define BIT_IS_SET(reg, pin)	     (BIT_VALUE((reg),(pin))==1)

//uart definitions
#define BAUD (9600)
#define MYUBRR (F_CPU/16/BAUD-1)
#define PRESCALE (256.0)
#define FREQ (16000000.0)

// These buffers may be any size from 2 to 256 bytes.
#define  RX_BUFFER_SIZE  64
#define  TX_BUFFER_SIZE  64


//uart definitions
char rx_buf[64];char temp_buf[64];char buffer[64];


static volatile uint8_t tx_buffer[TX_BUFFER_SIZE];
static volatile uint8_t tx_buffer_head;
static volatile uint8_t tx_buffer_tail;
static volatile uint8_t rx_buffer[RX_BUFFER_SIZE];
static volatile uint8_t rx_buffer_head;
static volatile uint8_t rx_buffer_tail;


//Functions declaration
void setup(void);
int process(void);
void loop(void);
void get_current_time(void);
void blare_alarm(void);
int get_ADC(void);

void uart_init(unsigned int ubrr);
//uart functions
void uart_putchar(uint8_t c);
uint8_t uart_getchar(void);
uint8_t uart_available(void);
void uart_putstring(unsigned char* s);
void uart_getLine(unsigned char* buf, uint8_t n);
//ADC functions
uint16_t adc_read(uint8_t channel);
void adc_init();
//string convertion functions 
void ftoa(float n, char* res, int afterpoint);
int intToStr(int x, char str[], int d);
void reverse(char* str, int len);


// END function declarations


 // Create a volatile global variable called switch_counter.
//  The variable should be an 8-bit unsigned integer. 
//  Initialise the variable to zero.
// INSERT GLOBAL VARIABLE HERE
volatile uint8_t switch_counter= 0; 
volatile uint32_t  counter= 0 ; 
//  (g) Define a volatile 8-bit unsigned global variable 
//  called is_pressed which will store the current state of the switch.
//  Initialise it to zero.

// INSERT GLOBAL VARIABLE HERE
volatile uint8_t is_pressed = 0; 
unsigned char mask = 0b00000111;
uint8_t stays_closed = 0 ;





//main loop
int main() {
	setup();

	for ( ;; ) {
 	 process();
    
     loop();
       
   
      

	}
}

  void blare_alarm(void)
    
  {
    lcd_clear();
   	
      int value = get_ADC();
   
    while(BIT_IS_SET(DDRD,3))
    {
     
      lcd_write_string(5,1,"ALERT!");
       
      OCR2B=180;
     _delay_ms(100);
      OCR2B=128;
      _delay_ms(100);
      OCR2B = 0;
        _delay_ms(50);
      lcd_clear();
      
      
    }
      
     
      while(value>=268)
      
      {
      
        value=get_ADC();
        
      }
   
		  is_pressed=0;
		  lcd_clear();
		setup();
    
		  loop();
    }
      
  
  	
  
     

void setup(void)
{
		 /*~~~~~~~~~~~~~~~~~~~~~~~~  SETTING LCD, UART AND INPUT/ OUTPUT OINS FOR THE LED  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
	
	lcd_init();
  	
    uart_init(MYUBRR);

	// Enable White LED
    
	SET_BIT(DDRB,4);
  	
 
  // Enable Green LED
     SET_BIT(DDRB,3);
	 
    // Enable Red LED
  	SET_BIT(DDRB,5);
  
  
  
    /*~~~~~~~~~~~~~~~~~~~~~~~~  INITIALIZATION OF ADC ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
	
	// ADC Enable and pre-scaler of 128

	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

    // Setting the channels for ADC 
	
    ADMUX = (1 << REFS0);
  	ADMUX|=(1<<1);
	
    
    /*~~~~~~~~~~~~~~~~~~~~~~~~  SETTING PWM  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
	
   
   // Set up D3 for output and setting pwm with usage of TIMER 2

	DDRD |= (1 << PD3);
    

    OCR2B = 0; // setting PWM initial duty cycle
    

    TCCR2A |= (1 << COM2B1); // set non-inverting mode
 
    TCCR2B = (1 << CS21); // set prescaler to 8 and starts PWM  
  
    TCCR2A |= (1 << WGM21) | (1 << WGM20); // set fast PWM Mode
    

    /*~~~~~~~~~~~~~~~~~~~~~~~~ DEBOUNCING   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
  
    //  Initialise Timer 0 in normal mode so that it overflows 
    //  with a period of approximately 0.004 seconds.
   
		TCCR0B|=(1<<CS02);
		TCCR0B&=~(1<<WGM02);
		
		
    //   Enable timer overflow interrupt for Timer 0.
		TIMSK0|=(1<<TOIE0);
	
	
  
    
    //   Enable the I/O pin labelled A2 for digital input.
		DDRC&=~(1<<5);
		
 
  
  
	 
}

// This function gets the current value of the ADC 

int get_ADC(void)
{
    
	
   // Start single conversion
   
	ADCSRA |= (1 << ADSC);

	// Wait for ADSC bit to clear.
	
	 while ( ADCSRA & (1 << ADSC) ) ;

	// storing the result in the an intvarial to be ued through out the code
	
	int adc_value = ADC;
  
   // Return the value 
  
 return adc_value;
 
}


// This function 
int process(void) 

{
	

   int adc_value= get_ADC();

    
  
	 //when converted value is above a threshold, perform an action 
     
  // When temp sensor is between (-40 and 0) ;
  // that means the adc_value (ADC) value is 20 and 104
  
  if ((adc_value>=20)&(adc_value<=104))
  {
    
    
    
     CLEAR_BIT(PORTB,PB5);
    CLEAR_BIT(PORTB,PB3);
    
    SET_BIT(PORTB,PB4);
   
    
    
   
  }

 
  
  
  // When temp sensor is between (0 and 20) ;
  // that means the adc_value (ADC) value is 104 and 166V
  
  
  
  else if ((adc_value>=104)&(adc_value<=166))
  { 
    CLEAR_BIT(PORTB,PB4);   
   
     CLEAR_BIT(PORTB,PB5);
   
    SET_BIT(PORTB,PB3);    
  	 
    
   
  }


 
  // When temp sensor is between (80 and 125)
  // that means the adc_value (ADC) value is 268 and 358

  else if ((adc_value>=166)&(adc_value<=358))
  { 
    
     CLEAR_BIT(PORTB,PB4);   
    
    CLEAR_BIT(PORTB,PB3);
    SET_BIT(PORTB,PB5);
    
    if(adc_value>=268)
	{
      
    /*~~~~~~~~~~~~~~~~~~~~~~~~  SETTING  TIMER   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
	
  //	 Initialise Timer 1 in normal mode so that it overflows 
    //	with a period of approximately 0.004 seconds.
 
 
	 TCCR1B|=(1<<CS12);
	 TCCR1B&=~(1<<WGM12);
	TIMSK1|= (1<<TOIE1);
      //  Turn on interrupts.
		sei();
  		 blare_alarm();
	}

  }

	

	


 return adc_value;
   
}
 
//////////////////// Serial and LCD ////////////
 void loop(void)
 {
      
		unsigned char c = uart_getchar(); 
     
      
         int adc_value = process();

   
        float voltage = adc_value*5.0;
        voltage=(voltage/1024.0);
         float temp = (voltage-0.5)*(100.0);
   
      ftoa(voltage,rx_buf,2);
				 lcd_write_char(3,0,'0');
     			lcd_write_string(5,0,rx_buf);
				  lcd_write_char(9,0,' ');
     			lcd_write_char(10,0,'V');
               
      float temp_far = temp;
     
      if(temp<0)
               {
                    temp=(-1)*temp;
                    lcd_write_char(7,1,'-');
      
                 }
               else
               {
                 lcd_write_char(7,1,' ');
               }
                   
       
      
      ftoa(temp,rx_buf,2);

                 lcd_write_string(2,1,"temp:");
                 lcd_write_string(8,1,rx_buf);
				   
                  lcd_write_char(14,1,'C');
       			
     
          if (c=='F')
          {
			    temp_far = ((temp_far/5.0)*9)+32.0;
             if(temp_far<0)
                  {
                    temp_far=(-1)*temp_far;
                  uart_putchar('-');
                  }
               else
               {
                 lcd_write_char(7,1,' ');
               }
                   ftoa(temp_far,buffer,2);
          
       			uart_putstring((unsigned char*)buffer);
            	uart_putchar(' ');
            	uart_putchar('F');
            	uart_putchar('\n');
           		
           
           }

      
 }  
     
     
   
   
void get_current_time()
{
    //compute elapsed time
	float time = ( counter * 65536.0 + TCNT1 ) * (PRESCALE/FREQ);
	ftoa(time,temp_buf,2);
  	uart_putstring((unsigned char *)temp_buf);
   uart_putchar('\n');
    
  	

	
	
}

   
     ////////// SETTING UP interrupts/////////
  /////////// Detecting switch ///////


ISR(TIMER0_OVF_vect)
{
	
	switch_counter=(((switch_counter << 1)& mask)|(BIT_VALUE(PINC,5)));
	if (switch_counter==0)
	{
		is_pressed=0;
      	
      
	}
	else if (switch_counter==mask)
	{
		is_pressed=1;
     
       DDRD&=~(1<<PD3);
	   TCCR1B&=~(1<<CS12);
	   TCCR1B&=~(1<<CS10); 
	   TCCR1B&=~(1<<CS11);
        counter=0;
      
      	
       get_current_time();
 
    }
 

   
}


ISR(TIMER1_OVF_vect)
{
	counter++;
 
}

     









/********** auxiliary functions *************/


// Reverses a string 'str' of length 'len' 
void reverse(char* str, int len) 
{ 
    int i = 0, j = len - 1, temp; 
    while (i < j) { 
        temp = str[i]; 
        str[i] = str[j]; 
        str[j] = temp; 
        i++; 
        j--; 
    } 
} 

// d is the number of digits required in the output.  
// If d is more than the number of digits in x,  
// then 0s are added at the beginning. 
int intToStr(int x, char str[], int d) 
{ 
    int i = 0; 
    while (x) { 
        str[i++] = (x % 10) + '0'; 
        x = x / 10; 
    } 
  
    // If number of digits required is more, then 
    // add 0s at the beginning 
    while (i < d) 
        str[i++] = '0'; 
  
    reverse(str, i); 
    str[i] = '\0'; 
    return i; 
} 
  

// Converts a floating-point/double number to a string. 
void ftoa(float n, char* res, int afterpoint) 
{ 
    // Extract integer part 
    int ipart = (int)n; 
  
    // Extract floating part 
    float fpart = n - (float)ipart; 
  
    // convert integer part to string 
    int i = intToStr(ipart, res, 0); 
  
    // check for display option after point 
    if (afterpoint != 0) { 
        res[i] = '.'; // add dot 
  
        // Get the value of fraction part upto given no. 
        // of points after dot. The third parameter  
        // is needed to handle cases like 233.007 
        fpart = fpart * pow(10, afterpoint); 
  
        intToStr((int)fpart, res + i + 1, afterpoint); 
    } 
} 

// Initialize the UART
void uart_init(unsigned int ubrr) {
	
	UBRR0H = (unsigned char)(ubrr>>8);
    UBRR0L = (unsigned char)(ubrr);
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	UCSR0C = ( 1 << UCSZ00)|( 1 << UCSZ01)|( 1 << UCSZ02);
	
}

// Transmit data
void uart_putchar(unsigned char data) {
	
  
   while (!( UCSR0A & (1<<UDRE0))); /* Wait for empty transmit buffer*/
          
    UDR0 = data;            /* Put data into buffer, sends the data */
        	
}

// Receive data
unsigned char uart_getchar(void) {
  

/* Get and return received data from buffer */
      		return UDR0;
     
 
}
     
           //// recieve string 
void uart_getLine(unsigned char* buf, uint8_t n)
{
  
    uint8_t bufIdx = 0;
    unsigned char c;

    // while received character is not carriage return
    // and end of buffer has not been reached
    do
    {
        // receive character
        c = uart_getchar();

        // store character in buffer
        buf[ bufIdx++] = c;
     
    }
    while((bufIdx < n) && (c != '\n'));

    // ensure buffer is null terminated
    buf[bufIdx] = 0;
 // return buf;
}


void uart_putstring(unsigned char* s)
{
    // transmit character until NULL is reached
    while(*s != 0)
    {uart_putchar(*s++);}
}




///////////////// LCD ///////////////////////////////
/* ********************************************/
// START LIBRARY FUNCTIOMNS

void lcd_init(void){
  //dotsize

  if (LCD_USING_4PIN_MODE){
    _lcd_displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
  } else {
    _lcd_displayfunction = LCD_8BITMODE | LCD_1LINE | LCD_5x8DOTS;
  }
  
  _lcd_displayfunction |= LCD_2LINE;

  // RS Pin
  LCD_RS_DDR |= (1 << LCD_RS_PIN);
  // Enable Pin
  LCD_ENABLE_DDR |= (1 << LCD_ENABLE_PIN);
  
  #if LCD_USING_4PIN_MODE
    //Set DDR for all the data pins
    LCD_DATA4_DDR |= (1 << 4);
    LCD_DATA5_DDR |= (1 << 5);
    LCD_DATA6_DDR |= (1 << 6);    
    LCD_DATA7_DDR |= (1 << 7);

  #else
    //Set DDR for all the data pins
    LCD_DATA0_DDR |= (1 << LCD_DATA0_PIN);
    LCD_DATA1_DDR |= (1 << LCD_DATA1_PIN);
    LCD_DATA2_DDR |= (1 << LCD_DATA2_PIN);
    LCD_DATA3_DDR |= (1 << LCD_DATA3_PIN);
    LCD_DATA4_DDR |= (1 << LCD_DATA4_PIN);
    LCD_DATA5_DDR |= (1 << LCD_DATA5_PIN);
    LCD_DATA6_DDR |= (1 << LCD_DATA6_PIN);
    LCD_DATA7_DDR |= (1 << LCD_DATA7_PIN);
  #endif 

  // SEE PAGE 45/46 OF Hitachi HD44780 DATASHEET FOR INITIALIZATION SPECIFICATION!

  // according to datasheet, we need at least 40ms after power rises above 2.7V
  // before sending commands. Arduino can turn on way before 4.5V so we'll wait 50

  _delay_us(50000); 
  // Now we pull both RS and Enable low to begin commands (R/W is wired to ground)
  LCD_RS_PORT &= ~(1 << LCD_RS_PIN);
  LCD_ENABLE_PORT &= ~(1 << LCD_ENABLE_PIN);
  
  //put the LCD into 4 bit or 8 bit mode
  if (LCD_USING_4PIN_MODE) {
    // this is according to the hitachi HD44780 datasheet
    // figure 24, pg 46

    // we start in 8bit mode, try to set 4 bit mode
    lcd_write4bits(0b0111);
    _delay_us(4500); // wait min 4.1ms

    // second try
    lcd_write4bits(0b0111);
    _delay_us(4500); // wait min 4.1ms
    
    // third go!
    lcd_write4bits(0b0111); 
    _delay_us(150);

    // finally, set to 4-bit interface
    lcd_write4bits(0b0010); 
  } else {
    // this is according to the hitachi HD44780 datasheet
    // page 45 figure 23

    // Send function set command sequence
    lcd_command(LCD_FUNCTIONSET | _lcd_displayfunction);
    _delay_us(4500);  // wait more than 4.1ms

    // second try
    lcd_command(LCD_FUNCTIONSET | _lcd_displayfunction);
    _delay_us(150);

    // third go
    lcd_command(LCD_FUNCTIONSET | _lcd_displayfunction);
  }

  // finally, set # lines, font size, etc.
  lcd_command(LCD_FUNCTIONSET | _lcd_displayfunction);  

  // turn the display on with no cursor or blinking default
  _lcd_displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;  
  lcd_display();

  // clear it off
  lcd_clear();

  // Initialize to default text direction (for romance languages)
  _lcd_displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
  // set the entry mode
  lcd_command(LCD_ENTRYMODESET | _lcd_displaymode);
  
  
}


/********** high level commands, for the user! */
void lcd_write_string(uint8_t x, uint8_t y, char string[]){
  lcd_setCursor(x,y);
  for(int i=0; string[i]!='\0'; ++i){
    lcd_write(string[i]);
  }
}

void lcd_write_char(uint8_t x, uint8_t y, char val){
  lcd_setCursor(x,y);
  lcd_write(val);
}

void lcd_clear(void){
  lcd_command(LCD_CLEARDISPLAY);  // clear display, set cursor position to zero
  _delay_us(2000);  // this command takes a long time!
}

void lcd_home(void){
  lcd_command(LCD_RETURNHOME);  // set cursor position to zero
  _delay_us(2000);  // this command takes a long time!
}


// Allows us to fill the first 8 CGRAM locations
// with custom characters
void lcd_createChar(uint8_t location, uint8_t charmap[]) {
  location &= 0x7; // we only have 8 locations 0-7
  lcd_command(LCD_SETCGRAMADDR | (location << 3));
  for (int i=0; i<8; i++) {
    lcd_write(charmap[i]);
  }
}


void lcd_setCursor(uint8_t col, uint8_t row){
  if ( row >= 2 ) {
    row = 1;
  }
  
  lcd_command(LCD_SETDDRAMADDR | (col + row*0x40));
}

// Turn the display on/off (quickly)
void lcd_noDisplay(void) {
  _lcd_displaycontrol &= ~LCD_DISPLAYON;
  lcd_command(LCD_DISPLAYCONTROL | _lcd_displaycontrol);
}
void lcd_display(void) {
  _lcd_displaycontrol |= LCD_DISPLAYON;
  lcd_command(LCD_DISPLAYCONTROL | _lcd_displaycontrol);
}

// Turns the underline cursor on/off
void lcd_noCursor(void) {
  _lcd_displaycontrol &= ~LCD_CURSORON;
  lcd_command(LCD_DISPLAYCONTROL | _lcd_displaycontrol);
}
void lcd_cursor(void) {
  _lcd_displaycontrol |= LCD_CURSORON;
  lcd_command(LCD_DISPLAYCONTROL | _lcd_displaycontrol);
}

// Turn on and off the blinking cursor
void lcd_noBlink(void) {
  _lcd_displaycontrol &= ~LCD_BLINKON;
  lcd_command(LCD_DISPLAYCONTROL | _lcd_displaycontrol);
}
void lcd_blink(void) {
  _lcd_displaycontrol |= LCD_BLINKON;
  lcd_command(LCD_DISPLAYCONTROL | _lcd_displaycontrol);
}

// These commands scroll the display without changing the RAM
void scrollDisplayLeft(void) {
  lcd_command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}
void scrollDisplayRight(void) {
  lcd_command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

// This is for text that flows Left to Right
void lcd_leftToRight(void) {
  _lcd_displaymode |= LCD_ENTRYLEFT;
  lcd_command(LCD_ENTRYMODESET | _lcd_displaymode);
}

// This is for text that flows Right to Left
void lcd_rightToLeft(void) {
  _lcd_displaymode &= ~LCD_ENTRYLEFT;
  lcd_command(LCD_ENTRYMODESET | _lcd_displaymode);
}

// This will 'right justify' text from the cursor
void lcd_autoscroll(void) {
  _lcd_displaymode |= LCD_ENTRYSHIFTINCREMENT;
  lcd_command(LCD_ENTRYMODESET | _lcd_displaymode);
}

// This will 'left justify' text from the cursor
void lcd_noAutoscroll(void) {
  _lcd_displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
  lcd_command(LCD_ENTRYMODESET | _lcd_displaymode);
}

/*********** mid level commands, for sending data/cmds */

inline void lcd_command(uint8_t value) {
  //
  lcd_send(value, 0);
}

inline size_t lcd_write(uint8_t value) {
  lcd_send(value, 1);
  return 1; // assume sucess
}

/************ low level data pushing commands **********/

// write either command or data, with automatic 4/8-bit selection
void lcd_send(uint8_t value, uint8_t mode) {
  //RS Pin
  LCD_RS_PORT &= ~(1 << LCD_RS_PIN);
  LCD_RS_PORT |= (!!mode << LCD_RS_PIN);

  if (LCD_USING_4PIN_MODE) {
    lcd_write4bits(value>>4);
    lcd_write4bits(value);
  } else {
    lcd_write8bits(value); 
  } 
}

void lcd_pulseEnable(void) {
  //Enable Pin
  LCD_ENABLE_PORT &= ~(1 << LCD_ENABLE_PIN);
  _delay_us(1);    
  LCD_ENABLE_PORT |= (1 << LCD_ENABLE_PIN);
  _delay_us(1);    // enable pulse must be >450ns
  LCD_ENABLE_PORT &= ~(1 << LCD_ENABLE_PIN);
  _delay_us(100);   // commands need > 37us to settle
}

void lcd_write4bits(uint8_t value) {
  //Set each wire one at a time

  LCD_DATA4_PORT &= ~(1 << LCD_DATA4_PIN);
  LCD_DATA4_PORT |= ((value & 1) << LCD_DATA4_PIN);
  value >>= 1;

  LCD_DATA5_PORT &= ~(1 << LCD_DATA5_PIN);
  LCD_DATA5_PORT |= ((value & 1) << LCD_DATA5_PIN);
  value >>= 1;

  LCD_DATA6_PORT &= ~(1 << LCD_DATA6_PIN);
  LCD_DATA6_PORT |= ((value & 1) << LCD_DATA6_PIN);
  value >>= 1;

  LCD_DATA7_PORT &= ~(1 << LCD_DATA7_PIN);
  LCD_DATA7_PORT |= ((value & 1) << LCD_DATA7_PIN);

  lcd_pulseEnable();
}

void lcd_write8bits(uint8_t value) {
  //Set each wire one at a time

  #if !LCD_USING_4PIN_MODE
    LCD_DATA0_PORT &= ~(1 << LCD_DATA0_PIN);
    LCD_DATA0_PORT |= ((value & 1) << LCD_DATA0_PIN);
    value >>= 1;

    LCD_DATA1_PORT &= ~(1 << LCD_DATA1_PIN);
    LCD_DATA1_PORT |= ((value & 1) << LCD_DATA1_PIN);
    value >>= 1;

    LCD_DATA2_PORT &= ~(1 << LCD_DATA2_PIN);
    LCD_DATA2_PORT |= ((value & 1) << LCD_DATA2_PIN);
    value >>= 1;

    LCD_DATA3_PORT &= ~(1 << LCD_DATA3_PIN);
    LCD_DATA3_PORT |= ((value & 1) << LCD_DATA3_PIN);
    value >>= 1;

    LCD_DATA4_PORT &= ~(1 << LCD_DATA4_PIN);
    LCD_DATA4_PORT |= ((value & 1) << LCD_DATA4_PIN);
    value >>= 1;

    LCD_DATA5_PORT &= ~(1 << LCD_DATA5_PIN);
    LCD_DATA5_PORT |= ((value & 1) << LCD_DATA5_PIN);
    value >>= 1;

    LCD_DATA6_PORT &= ~(1 << LCD_DATA6_PIN);
    LCD_DATA6_PORT |= ((value & 1) << LCD_DATA6_PIN);
    value >>= 1;

    LCD_DATA7_PORT &= ~(1 << LCD_DATA7_PIN);
    LCD_DATA7_PORT |= ((value & 1) << LCD_DATA7_PIN);
    
    lcd_pulseEnable();
  #endif
}




