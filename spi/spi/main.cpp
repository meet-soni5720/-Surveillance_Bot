/*
 * spi.cpp
 *
 * Created: 23-07-2019 18:26:29
 * Author : Meet
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include <math.h>
#include <stdlib.h>

#define F_CPU 8000000UL

#define select 0
#define l3 1
#define r3 2
#define start 3
#define up 4
#define rig 5
#define down 6
#define lef 7
#define l2 0
#define r2 1
#define l1 2
#define r1 3
#define triangle 4    //up
#define circle 5      //right
#define cross 6      //down
#define squ 7        //left

int duty=255;
int cliflag = 0;
void forward();
void backward();
void right();
void left();
void sright();
void sleft();

int btod(int data)
{
	int i;
	int k =1,bit,ans=0;
	while(data>0)
	{
		bit = data%10;
		ans += (k*bit);
		k = k*2;
		data = data/10;
	}
	return ans;
}
 void PWM_init()
 {
	 TCCR1B |=(0<<WGM13)|(1<<WGM12)|(1<<CS10);  // fast pwm is activate
	 DDRB |= (1<<PINB6)|(1<<PINB5);
 }

 ISR(INT1_vect)
 {
	 PORTC |= (1<<PINC0)|(1<<PINC1);
	 _delay_ms(100);
	 cliflag=1;
 }

void SPI_master_init(void)
{
	//set MOSI,sck and ss output all other input
	//enable spi,master ,set clock rate fck/64,MSB is sent first
	SPCR |= (1<<SPE) | (1<<MSTR) | (1<<SPR1) | (1<<DORD) | (1<<CPOL) | (1<<CPHA) | (1<<SPR0);
	SPSR |= (1<<SPI2X);   // Enabling speed doubler
	
}
unsigned char SPI_write(unsigned char data)
{
	
	//char flush_buffer;
	SPDR = data;                      // write data to spi data register
	while(!(SPSR & (1<<SPIF)));  // wait till transmission is complete
	//flush_buffer = SPDR;    //flush received data
	//spi flag is cleared by first reading SPSR(with SPIF set) and then accessing SPDR hence buffer used here to access SPDR after SPSR read
	_delay_ms(30);
	return SPDR;
}
// writing dummy data in spdr register

unsigned char SPI_Read()
{
	SPDR = 0xFF;
	while(!(SPSR & (1<<SPIF)));  // wait until data is received
	return(SPDR);
}

int main(void) 
{
   
  int data0,data1,data2,data3,data4,data5,a,b,c,d=0;
  float rx,ry,lx,ly;
  int cir_rx,cir_ry,cir_lx,cir_ly; 
  
  
  // do the initialization of the ports according to the use
  DDRB = 0b00000111;
  DDRC = 0b11111111;
  DDRA = 0b11111111;
  DDRD = 0b11111111;
  PORTC = 0x00;
  
  SPI_master_init();
  EIMSK |= (1<<INT1);
  EICRA |= (1<<ISC01); //falling edge generates interrupt request

  PORTD |=(1 << PIND1);
  
  
  PWM_init();
 
  while(d!= 0x73)
  {    
	   PORTB|=(1<<PINB2)|(1<<PINB1); //set
	   PORTB&=~(1<<PINB0); //clr   // slave select setting to low
	   
	   
	   SPI_write(0x01); //entering config mode
	   SPI_write(0x43);
	   SPI_write(0x00);
	   SPI_write(0x01);
	   SPI_write(0x00);
	   
	   PORTB|=(1<<PINB2);
	   _delay_ms(1);
	   PORTB|=(1<<PINB0);

	   _delay_ms(10);

	   
	   PORTB|=(1<<PINB2)|(1<<PINB1); //setting
	   
	   PORTB&=~(1<<PINB0); // clearing

	   SPI_write(0x01); //setting analog mode
	   SPI_write(0x44);
	   SPI_write(0x00);
	   SPI_write(0x01);   // 0x01 for analog and 0x00 for digital mode
	   SPI_write(0x03);   // controller mode is locked
	   SPI_write(0x00);
	   SPI_write(0x00);
	   SPI_write(0x00);
	   SPI_write(0x00);

	   PORTB|=(1<<PINB2);
	   _delay_ms(1);
	   PORTB|=(1<<PINB0);

	   _delay_ms(10);

	   
	   PORTB|=(1<<PINB2)|(1<<PINB1);
	   PORTB&=~(1<<PINB0);

	   SPI_write(0x01);/*was using this to poll.The code will work fine even without this*/
	   SPI_write(0x43);
	   SPI_write(0x00);
	   SPI_write(0x00);
	   SPI_write(0x5A);
	   SPI_write(0x5A);
	   SPI_write(0x5A);
	   SPI_write(0x5A);
	   SPI_write(0x5A);

	   PORTB|=(1<<PINB2);
	   _delay_ms(1);
	   PORTB|=(1<<PINB0);
	   _delay_ms(10);


	   
	   
	   PORTB|=(1<<PINB2)|(1<<PINB1);
	   PORTB&=~(1<<PINB0);
	    SPI_write(0x01);
	   d= SPI_write(0x42); //making sure we're in the analog mode the value of d will be 0x73
	   SPI_write(0x00);   // if d is not equal to 0x73 this loop will repeat
	   SPI_write(0x00);
	   SPI_write(0x00);
	   SPI_write(0x00);
	   SPI_write(0x00);
	   SPI_write(0x00);
	   SPI_write(0x00);
	   PORTB|=(1<<PINB2);
	   _delay_ms(1);
	   PORTB|=(1<<PINB0);
	   _delay_ms(10);
	   
	   
	   _delay_ms(1000);
	   
   } 
    PORTC = 0x00;
    
      while(d==0x73)
      {
	      while (1)
	      {
		     
		      PORTB|=(1<<PINB2) | (1<<PINB1);
		      PORTB&=~(1<<PINB0);
		      a=SPI_write(0x01);
		      b=SPI_write(0x42);
		      c=SPI_write(0x00);
		      

		      data0 = SPI_write(0x00); //buttons set 1 8
		      data1 = SPI_write(0x00); //button set 2  8
		      data2 = SPI_write(0x00); //  rx
		      data3 = SPI_write(0x00); //  ry
		      data4 = SPI_write(0x00); //  lx
		      data5 = SPI_write(0x00); //  ly
			  
			   
		      _delay_us(1);
		      PORTB|=(1<<PINB2);
		      _delay_us(1);
		      PORTB|=(1<<PINB0);
			  
			  //INTERUPT SETUP
			  if (cliflag==0)
			  {
				  sei();
			  }
			  else
			  {
				  cli();
				  _delay_ms(1000);
				  cliflag=0;
			  }
		  } 
			  
			  rx = (float)((btod(data2)-127)/127);
			  _delay_ms(1);
			  ry = (float)((btod(data3)-127)/127);
			  _delay_ms(1);
			  lx = (float)((btod(data4)-127)/127);
			  _delay_ms(1);
			  ly = (float)((btod(data5)-127)/127);
			  _delay_ms(1);
			  
			  
			  // circular mapping
			  
			  cir_rx = (int)(rx*sqrt(1-(ry*ry/2))*127);
			  cir_ry = (int)(ry*sqrt(1-(rx*rx/2))*127);
			  cir_lx = (int)(lx*sqrt(1-(ly*ly/2))*127);
			  cir_ly = (int)(ly*sqrt(1-(lx*lx/2))*127);  
			 
			 
		
		
		if(~data1 & (1<<triangle))  //up 
		{
			forward();
			PORTC |= (1 << PINC0);
			_delay_ms(1);
			PORTC &= ~(1 << PINC0);
			
		}
		 if(~data1 & (1<<circle))   //right 
		{
			sright();
			PORTC |= (1 << PINC1);
			_delay_ms(1);
			PORTC &= ~(1 << PINC1);
			
		}
		 if(~data1 & (1 << cross) )   //down
		{    backward();
			 PORTC |= (1 << PINC2);
			_delay_ms(1);
			 PORTC &= ~(1 << PINC2);
			
		}
	    if(~data1 & (1 << squ))   //left
		{   sleft();
			PORTC |= (1 << PINC1);
			_delay_ms(1);
			PORTC &= ~(1 << PINC1);
			
		}
		 if((~data1 & (1 << triangle)) && (~data1 & (1 << circle) ))  //up-right
		{   right();
			PORTC |= (1 << PINC0)|(1 << PINC1);
			_delay_ms(1);
			PORTC &= ~(1 << PINC0) & ~(1 << PINC1);
			
		}
		 if((~data1 & (1 << triangle)) && (~data1 & (1 << squ))) //up-left
		{   left();
			PORTC |= (1 << PINC0)|(1 << PINC3);
			_delay_ms(1);
			PORTC &= ~(1 << PINC0) & (1 << PINC3);
		}
		/*else
		{
			PORTC |= (1 << PINC0) | (1 << PINC1) | (1 << PINC2) | (1 << PINC3);
		} */
		
		/*else
		{
			PORTC &= ~(1 << PINC0) & ~(1 << PINC1) & ~(1 << PINC2) & ~(1 << PINC3);
		}*/
		
		/*int V = ((128 - abs(data2))*(data5/100)) + data5;
		int W = ((128 - abs(data5))*(data2/100)) + data2;
		
		int r_value = (V+W)/2;
		int l_value = (V-W)/2;  */ // we will directly give the value of it to the pwm pins so that we can easily run our bot
		
		if(cir_rx>=0 && cir_rx <=130)
		{
			PORTC |= (1 << PINC0);
			_delay_ms(1);
			PORTC &= ~(1 << PINC0);
		}
		 if(cir_rx >= -128 && cir_rx < 0)
		{
			PORTC |= (1 << PINC2);
			_delay_ms(1);
			PORTC &= ~(1 << PINC2);
		}
		 if(cir_ly >= 0  && cir_ly <=130)
		{
			PORTC |= (1 << PINC1);
			_delay_ms(1);
			PORTC &= ~(1 << PINC1);
		}
		 if(cir_ly >= -128 && cir_ly < 0)
		{
			PORTC |= (1 << PINC0);
			_delay_ms(1);
			PORTC &= ~(1 << PINC0);
		}
			/*else
			{
				PORTC |= (1 << PINC0) | (1 << PINC1) | (1 << PINC2) | (1 << PINC3);
			}*/
		  
			
			}
			 }
			 
		  
		  void forward()
		  {
			  PORTA=0b00000101;
			  OCR1A=(int)(duty); //setting PWM to 100% duty cycle
			  OCR1B=(int)(duty);
			  
		  }
		  void backward()
		  {
			  PORTA=0b00001010;
			  OCR1A=(int)(duty); //setting PWM to 100% duty cycle
			  OCR1B=(int)(duty);
			  
		  }
		  // OCR1A FOR LEFT AND OCR1B FOR RIGHT
		  
		  void left()
		  {
			  PORTA=0b00000001;
			  OCR1A=(int)(duty*0.2); //setting PWM to 50% duty cycle
			  OCR1B=(int)(duty*0.8);
			  
		  }

		  void right()
		  {
			  PORTA=0b0000100;
			  OCR1A=(int)(duty*0.8); //setting PWM to 50% duty cycle
			  OCR1B=(int)(duty*0.2);
			  
		  }
		  void sleft()
		  {
			  PORTA=0b00001001;
			  OCR1A=(int)(duty*0.5); //setting PWM to 50% duty cycle
			  OCR1B=(int)(duty*0.8);
			  
		  }
		  void sright()
		  {
			  PORTA=0b00000110;
			  OCR1A=(int)(duty*0.8); //setting PWM to 50% duty cycle
			  OCR1B=(int)(duty*0.5);
			  
		  }
		 



