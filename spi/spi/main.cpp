/*
 * spi.cpp
 *
 * Created: 23-07-2019 18:26:29
 * Author : Meet
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <math.h>

#define F_CPU 16000000UL

void SPI_master_init(void)
{
	//set MOSI,sck and ss output all other input
	//DDRB = 0b00000111;  // MISO is input
	
	//PORTB |= (1<<SS);   // setting ss port to high initially
	//enable spi,master ,set clock rate fck/16,MSB is sent first
	SPCR = (1<<SPE) | (1<<MSTR) | (1<<SPR1) | (1<<DORD) | (1<<CPOL) | (1<<CPHA);
	SPSR |= (1<<SPI2X);   // Enabling speed doubler
	
}
unsigned char SPI_write(unsigned char data)
{
	
	//char flush_buffer;
	SPDR = data;                      // write data to spi data register
	while(!(SPSR & (1<<SPIF)));  // wait till transmisssion is complete
	//flush_buffer = SPDR;    //flush received data
	//spi flag is cleared by first reading SPSR(with SPIF set) and then accessing SPDR hence buffer used here to access SPDR after SPSR read
	_delay_ms(30);
	return SPDR;
}
// writing dummy data in spdr register

char SPI_Read()
{
	
	SPDR = 0xFF;
	while(!(SPSR & (1<<SPIF)));  // wait until data is received
	return(SPDR);
}

 enum {select, leftStick, rightStick, start, up, right, down, left}; //3rd byte
 enum {leftFront2, rightFront2, leftFront1, rightFront1, triangle_up, circle_right, cross_down, square_left}; // 4th byte
 
 int isPressed(uint16_t databyte,uint16_t databit)
 {
	 return((~(databyte & (1<<databit))) ? 1 : 0);
 }
int main(void)
{
   
  int data0,data1,data2,data3,data4,data5,a,b,c,d=0;
  int cir_rx,cir_ry,cir_lx,cir_ly;
  int x,y,z;
  
  
  
  // do the initialistion of the ports according to the use
  PORTB = 0x00;
  DDRB = 0x0D;
  DDRC = 0b11111111;
  SPI_master_init();
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

	   SPI_write(0x01); //seting analog mode
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
	   
   } 
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
			  
			 /* USART_Transmit(data0);
			  _delay_ms(1000);
			  USART_Transmit(data1);
			  _delay_ms(1000);
			  USART_Transmit(data2);
			  _delay_ms(1000);
			  USART_Transmit(data3);
			  _delay_ms(1000);
			  USART_Transmit(data4);
			  _delay_ms(1000);
			  USART_Transmit(data5);
			  _delay_ms(1000); */

		      _delay_us(1);
		      PORTB|=(1<<PINB2);
		      _delay_us(1);
		      PORTB|=(1<<PINB0); 
			  
			  
			  
			 /* cir_rx = data2*sqrt(1 - (0.5*(data3)*(data3)));
			  cir_ry = data3*sqrt(1 - (0.5*(data2)*(data2)));
			  cir_lx = data4*sqrt(1 - (0.5*(data5)*(data5)));
			  cir_ly = data5*sqrt(1 - (0.5*(data5)*(data5)));*/
			 
			 if(isPressed(data0,right))
			 {
				 PORTC |= (1 << PINC0);
				 _delay_ms(1000);
				 PORTC &= ~(1 << PINC0);
			 }
			 
			 else if(isPressed(data0,left))
			 {
				 PORTC |= (1 << PINC1);
				 _delay_ms(1000);
				 PORTC &= ~(1 << PINC1);
				 
			 }
			 else if(isPressed(data1,triangle_up))
			 {
				 PORTC |= (1 << PINC2);
				_delay_ms(1000);
				PORTC &= ~(1 << PINC2);
			 }
			 else if(isPressed(data1,cross_down))
			 {
				 PORTC |= (1 << PINC3);
				 _delay_ms(1000);
				 PORTC &= ~(1 << PINC3);
			 } 
			 else
			  PORTC &= ~(1 << PINC0) & ~(1 << PINC1) & ~(1 << PINC2) & ~(1 << PINC3);
			  
			/*DDRC = 0xFF;
			while(1)
			{
				PORTC |= (1 << PINC0) | (1 << PINC1) | (1<<PINC2);
			} */
			}
			 }
			 
		  }


