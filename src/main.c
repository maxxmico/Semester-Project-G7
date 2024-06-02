/*
Hello World MULTIMETER
Author: SPRO2 GROUP 7
*/

//Include necessary libraries
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "usart.h"
#include <stdint.h>

// include LCD libraries
#include "ssd1306.h"

//Definitions & Variables
#define chargePin PD5 
#define dischargePin PD3    
#define analogPin PC0       
#define resistorValue 10000.0F //10k resistor

float microFarads;
volatile uint32_t ms_counter = 0; //millisecond counter
uint16_t elapsedTime;

#define ADC_PIN 1 //ADC channel we’ll use

float buffer = 0.00;
int Vin = 5.00; //5V applied by Arduinno
float Vout = 0.00; //VOut in Volts
float Rref= 10.00; // Set this values to the value of the used resistor in K ohms
float R2 = 0.00; //Unknown resistor set to 0
uint16_t analogValue;

//Function prototypes
void initADC();
uint16_t readADC(uint8_t channel);
void initUSART();
void USART_Transmit(char data);
void initTimer1(void);
uint32_t millis(void);
void DisplayMicroFarads(float capacitance);

uint16_t adc_read(uint8_t adc_channel); //func prototype
void DisplayFloatResistance(float resistance2);
void DisplayFloatVoltage(float voltage);

void setup(){

  PORTC = 0b00111101; //set Analog pins C0,C2,C3 as INPUTS & C1 (A1) as OUTPUT
             //0011 for C5 & C4 for LCD

  //ADC registers
  ADMUX = 0b00000001; //last 0001 for A1 pin for analog input (ADC1)
            //first 00 REFS1 REFS0 for Voltage reference; AREF internal Vrot turned ofF
  ADCSRA = 0b10000000;
            //first 1 to ENABLE ADC
              //second 0 is set to 1 to start conversion

}

int main(void) {

    initUSART();
    initADC();
    initTimer1();
    sei(); 

    // LCD INIT
    SSD1306_Init (SSD1306_ADDR); // 0X3C

    DDRD = 0b01000100; //set D2 AND D6 as OUTPUT
    PORTD = 0b00000100; //set D2 as HIGH; D6 as LOW
    DDRD |= (1 << chargePin); //chargePin OUTPUT
    DDRD &= ~(1 << dischargePin); //dischargePin INPUT

    while (1) {

        PORTD |= (1 << chargePin); //chargePin HIGH

        uint32_t startTime = millis();
        while (readADC(analogPin) < 648); //wait until capacitor reaches value
        elapsedTime = millis() - startTime;
        microFarads = ((float)elapsedTime / resistorValue) * 1000.0;

        //printLong(elapsedTime);
        // printString(" mS    ");

        analogValue = adc_read(ADC_PIN); // Read analog Voltage

        //convert to Volts
        buffer = analogValue * Vin;
        Vout = (buffer)/ 1024.00;

        buffer = (Vin/Vout) -1;
        R2 = (Rref * buffer*1000) - 30; //*1000 because we express it in ohms / -30 due to tolerances


        if (microFarads == 0){

          SSD1306_ClearScreen(); 
          SSD1306_SetPosition (15, 3);   // center position TITLE (yellow)
          SSD1306_DrawString ("CAPACITANCE");
      
          SSD1306_SetPosition (0, 0); 
          SSD1306_DrawString ("Insert");
          SSD1306_SetPosition (0, 1); 
          SSD1306_DrawString ("Capacitor");

          SSD1306_UpdateScreen (SSD1306_ADDR);  // update


        } else{

          SSD1306_ClearScreen(); 
          SSD1306_SetPosition (15, 3);   // center position TITLE (yellow)
          SSD1306_DrawString ("CAPACITANCE");
      
          SSD1306_SetPosition (0, 1); 
          DisplayMicroFarads(microFarads);

          SSD1306_SetPosition (0, 2); 
          DisplayFloatResistance(R2);

          SSD1306_SetPosition (0, 3); 
          DisplayFloatVoltage(Vout);

          SSD1306_UpdateScreen (SSD1306_ADDR);  // update
        }

        PORTD &= ~(1 << chargePin); //chargePin LOW
        DDRD |= (1 << dischargePin); //dischargePin OUTPUT
        PORTD &= ~(1 << dischargePin); //dischargePin LOW

        while (readADC(analogPin) > 0);//wait until capacitor discharges

        DDRD &= ~(1 << dischargePin); //dischargePin INPUT
        _delay_ms(500);
    }
}

void initADC() {
  ADMUX = (1 << REFS0); // AVcc with external capacitor at AREF pin
  ADCSRA = (1 << ADEN) | (1 << ADPS1) | (1 << ADPS0); // Enable ADC and set prescaler to 8
}

uint16_t readADC(uint8_t channel) {
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
    ADCSRA |= (1 << ADSC); // Start conversion
    while (ADCSRA & (1 << ADSC)); // Wait until conversion is complete
    return ADC;
}

void initUSART() {
    //set baud rate
    uint16_t ubrr = 103; //9600 baud for 16MHz
    UBRR0H = (unsigned char)(ubrr>>8);
    UBRR0L = (unsigned char)ubrr;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0); //enable receiver and transmitter
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); //set frame format: 8 data bits, 1 stop bit
}

void USART_Transmit(char data) {
    while (!(UCSR0A & (1 << UDRE0))); //wait for empty transmit buffer
    UDR0 = data; //put data into buffer, sends the data
}

void initTimer1(void) {
    TCCR1B |= (1 << WGM12); //configure timer 1 for CTC mode
    TIMSK1 |= (1 << OCIE1A); //enable CTC interrupt
    OCR1A = 249; //16MHz / 64 prescaler / 1000 - 1 = 249
    TCCR1B |= (1 << CS11) | (1 << CS10); //start timer at Fcpu/64
}

ISR(TIMER1_COMPA_vect) {
    ms_counter++; //increment ms counter
}

uint32_t millis(void) {
    uint32_t millis_copy;
    cli(); 
    millis_copy = ms_counter; 
    sei(); 
    return millis_copy;
}

void DisplayMicroFarads(float capacitance) {

  char buffer[20]; //buffer to hold the formatted string
  sprintf(buffer, "%.3f uF", capacitance); // to format str of the capacitance value

  SSD1306_DrawString (buffer); //print capacitance value in OLED 
}

//ADC function
uint16_t adc_read(uint8_t adc_channel){
  ADMUX &= 0xf0; // clear previously used channel, but keep internal reference
  ADMUX |= adc_channel; // set the desired channel 
  ADCSRA |= (1<<ADSC);  //start a conversion
  while ( (ADCSRA & (1<<ADSC)) ); //wait for conversion to complete

  return ADC; //return to the calling function as a 16 bit unsigned int
}

//function iused to print variables in the OLED with units, not only the number
void DisplayFloatResistance(float resistance2) {

  char buffer[20]; //buffer to hold the formatted string
  sprintf(buffer, "%.3f Ohms", resistance2); // to format str of the R2 value

  SSD1306_DrawString (buffer); //print R2 value in OLED 
}

