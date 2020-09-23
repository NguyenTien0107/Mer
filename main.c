//
//  Transmit ADC value as UART data
//
//  Transmit data [header1], [header2], [y-upper], [y-lower], [x-upper], [x-lower], [x & y checksum]
//
#include <iostm8s103f3.h>
#include <intrinsics.h>
#include "nRF24L01.h"
#include "RF24.h"

// global variables
char anch = 3;    // analog channel
char txreq = 0;   // transmission request
char txbuf[4] = {0};  // serial data transmission buffer
//
unsigned long elpstime;   // elapse timer (mS)
unsigned long elpstend;   // elapse timer check end
//
RF24 radio(0);         // nRF24L01 module instance

// delay wait (dly mS)
void delay(unsigned long dly){
  elpstend = elpstime + dly;
  while(elpstend > elpstime);
}

// report system clock (mS)
unsigned long millis(void){
  return elpstime;
}

//--------------------------------------------------------------------------------
//
//  Setup the system clock to run at 16MHz using the internal oscillator.
//
void InitialiseSystemClock()
{
    CLK_ICKR = 0;                       //  Reset the Internal Clock Register.
    CLK_ICKR_HSIEN = 1;                 //  Enable the HSI.
    CLK_ECKR = 0;                       //  Disable the external clock.
    while (CLK_ICKR_HSIRDY == 0);       //  Wait for the HSI to be ready for use.
    CLK_CKDIVR = 0;                     //  Ensure the clocks are running at full speed.
    CLK_PCKENR1 = 0xff;                 //  Enable all peripheral clocks.
    CLK_PCKENR2 = 0xff;                 //  Ditto.
    CLK_CCOR = 0;                       //  Turn off CCO.
    CLK_HSITRIMR = 0;                   //  Turn off any HSIU trimming.
    CLK_SWIMCCR = 0;                    //  Set SWIM to run at clock / 2.
    CLK_SWR = 0xe1;                     //  Use HSI as the clock source.
    CLK_SWCR = 0;                       //  Reset the clock switch control register.
    CLK_SWCR_SWEN = 1;                  //  Enable switching.
    while (CLK_SWCR_SWBSY != 0);        //  Pause while the clock switch is busy.
}

//--------------------------------------------------------------------------------
//
//  Timer 2 interrupt handler.
#pragma vector = TIM2_OVR_UIF_vector
__interrupt void TIM2_UPD_OVF_IRQHandler(void)
{
    elpstime++;             //  timer increment
    TIM2_SR1_UIF = 0;       //  Reset the interrupt
    if((elpstime % 50) == 0) ADC_CR1_ADON = 1;   // next ADC start
}

//  Setup Timer 2 to generate an interrupt every 1mS based on a 16MHz clock.
void SetupTimer2()
{
    TIM2_PSCR = 0x05;       //  Prescaler = 32.
    TIM2_ARRH = 0x01;       //  High byte of 500.
    TIM2_ARRL = 0xf4;       //  Low byte of 500.
    TIM2_IER_UIE = 1;       //  Enable the update interrupts.
    TIM2_CR1_CEN = 1;       //  Finally enable the timer.
}

//--------------------------------------------------------------------------------
//
//  ADC Conversion completed interrupt handler.
//
#pragma vector = ADC1_EOC_vector
__interrupt void ADC1_EOC_IRQHandler()
{
    int reading;
    static short xaxis = 0;
    static short yaxis = 0;
 
    ADC_CR1_ADON = 0;       //  Disable the ADC.
    ADC_CSR_EOC = 0;	    // 	Indicate that ADC conversion is complete.

    reading = (int)(ADC_DRL) & 0xff;
    reading += (int)(ADC_DRH & 0x03) << 8; // Extract the ADC reading

    // set read data
    if(anch == 3){
      yaxis = reading;
      anch = 4;
    } else {
      xaxis = reading;
      anch = 3;
    }

    // check next ADC     
    ADC_CSR_CH = anch;      //  analog input channel

    // uart TX request & restart timer
    if(anch == 3){
      txbuf[0] = (yaxis >> 8) & 0xff;
      txbuf[1] = yaxis & 0xff;
      txbuf[2] = (xaxis >> 8) & 0xff;
      txbuf[3] = xaxis & 0xff;
      txreq = 1;            //  set tx request
    }

    PA_ODR_ODR3 = !PA_ODR_ODR3;           //  Indicate that the ADC has completed.
}

//--------------------------------------------------------------------------------
//
//  Setup the ADC to perform a single conversion and then generate an interrupt.
//
void SetupADC()
{
    ADC_CR1_ADON = 1;       //  Turn ADC on, note a second set is required to start the conversion.

    ADC_CSR_CH = anch;

    ADC_CR3_DBUF = 0;
    ADC_CR2_ALIGN = 1;      //  Data is right aligned.
    ADC_CSR_EOCIE = 1;      //  Enable the interrupt after conversion completed.
}

//--------------------------------------------------------------------------------
//
//  Now set up the output ports.
//
//  Setup the port used to signal to the outside world that a timer event has
//  been generated.
//
void SetupOutputPorts()
{
    //
    //  PB5 indicates uart tramsmited
    //
    PB_DDR_DDR5 = 1;
    PB_CR1_C15 = 1;
    PB_CR2_C25 = 1;
    //
    //  PA3 indicate that the ADC has completed.
    //
    PA_DDR_DDR3 = 1;
    PA_CR1_C13 = 1;
    PA_CR2_C23 = 0;
}

//--------------------------------------------------------------------------------
//  set up SPI ports
//
// SPI control port assign
#define CEOUT  PD_ODR_ODR1    // SPI ce port
#define CSNOUT PD_ODR_ODR4    // SPI csn port

// Assign output ce & csn port for SPI port control
void SPICtrlPortInitial(void){
    // set PD1 for SPI ce port
    PD_DDR_DDR1 = 1;   // PD1 set output mode
    PD_CR1_C11 = 1;    // push pull mode
    PD_CR2_C21 = 1;    // high speed mode
    CEOUT = 1;         // PD1 output set 1;
    // set PD4 for SPI csn port
    PD_DDR_DDR4 = 1;   // PD4 set output mode
    PD_CR1_C14 = 1;    // push pull mode
    PD_CR2_C24 = 1;    // high speed mode
    CSNOUT = 1;        // PD4 output set 1;
}

// Set SPI ce control port
void CEportSet(unsigned char val){ // val 0:Low level, !=0:High level
    CEOUT = (val == 0) ? 0 : 1;
}

// Set SPI csn control port
void CSNportSet(unsigned char val){ // val 0:Low level, !=0:High level
    CSNOUT = (val == 0) ? 0 : 1;
}

// TX one byte
unsigned char TxSPIbyte(unsigned char data)
{
//    SPI_CR1_LSBFIRST = (lsbfirst ? 1 : 0);
    SPI_CR1_SPE = 1;
    SPI_DR = data;
    while(!SPI_SR_TXE);  // wait until tx end
    SPI_CR1_SPE = 0;
    while(!SPI_SR_RXNE);
    return SPI_DR;
}

//  Initialize SPI setting
void SetupSPImode()
{
    SPI_CR1_SPE = 0;                    //  Disable SPI.
    SPI_CR1_BR = 1;                     //  fmaster / 4 (4M baud)  16MHz/value  value 0:2, 1:4, 2:8, 3:16 ...
    SPI_CR1_MSTR = 1;                   //  Master device.
}

//--------------------------------------------------------------------------------
//
//  Main program loop.
//
void main()
{
    // variables
     const char addresses[][6] = {"1Node","2Node"};
     unsigned long rxdevtime = 0;
     unsigned long errcnt = 0;

    //
    //  Initialise the system.
    //
    InitialiseSystemClock();
    SetupTimer2();
    SetupOutputPorts();
    SetupADC();
    SetupSPImode();                     // set SPI mode
    SPICtrlPortInitial();               // set SPI control ports
    __enable_interrupt();
    delay(500);                         // wait module unit power up

    // set nRF214L01 initial condition
    radio.begin();                      // Start up the radio
    radio.setAutoAck(1);                // Ensure autoACK is enabled
    radio.setRetries(15, 15);           // Max delay between retries & number of retries
    radio.openWritingPipe((const unsigned char *)addresses[0]);
    radio.openReadingPipe(1, (const unsigned char *)addresses[1]);
    radio.startListening();             // Start listening

    // loop process
    while (1)
    {
       // check ADC completed
       if(txreq){
          txreq = 0;
          // set ADC data block
          unsigned char tx[7];
          tx[0] = 0xaa;                 // set tx data header
          tx[1] = 0x55;
          unsigned short chksum = 0;
          for(int i = 0 ; i < 4 ; i++){
            tx[i + 2] = txbuf[i];
            chksum += txbuf[i];
          }
          tx[6] = chksum & 0xff;
          // send data to nRF24L01 module
          radio.stopListening();        // First, stop listening so we can talk
          bool errflg = false;
          if(!radio.write(tx, sizeof(tx))) errflg = true;
          if(!errflg || ((errcnt++ % 4) == 0))  //  Indicate status with LED blink
            PB_ODR_ODR5 = !PB_ODR_ODR5;
          radio.startListening();
       }

       // check data received
       if(radio.available())
         radio.read(&rxdevtime, sizeof(unsigned long));
   }
}
