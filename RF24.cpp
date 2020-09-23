/*
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */
#include <string.h>
#include "nRF24L01.h"
#include "RF24.h"

/****************************************************************************/

void RF24::csn(bool mode)
{
  CSNportSet(mode);
}

/****************************************************************************/

void RF24::ce(bool level)
{
  CEportSet(level);
}

/****************************************************************************/

unsigned char RF24::read_register(unsigned char reg, unsigned char* buf, unsigned char len)
{
  unsigned char status;
  csn(LOW);
  status = TxSPIbyte(R_REGISTER | (REGISTER_MASK & reg));
  while ( len-- ) *buf++ = TxSPIbyte(0xff);
  csn(HIGH);

  return status;
}

/****************************************************************************/

unsigned char RF24::read_register(unsigned char reg)
{
  csn(LOW);
  TxSPIbyte(R_REGISTER | (REGISTER_MASK & reg));
  unsigned char result = TxSPIbyte(0xff);
  csn(HIGH);

  return result;
}

/****************************************************************************/

unsigned char RF24::write_register(unsigned char reg, const unsigned char* buf, unsigned char len)
{
  unsigned char status;

  csn(LOW);
  status = TxSPIbyte( W_REGISTER | ( REGISTER_MASK & reg ) );
  while ( len-- ) TxSPIbyte(*buf++);

  csn(HIGH);

  return status;
}

/****************************************************************************/

unsigned char RF24::write_register(unsigned char reg, unsigned char value)
{
  unsigned char status;

  csn(LOW);
  status = TxSPIbyte( W_REGISTER | ( REGISTER_MASK & reg ) );
  TxSPIbyte(value);
  csn(HIGH);

  return status;
}

/****************************************************************************/

unsigned char RF24::write_payload(const void* buf, unsigned char data_len, const unsigned char writeType)
{
  unsigned char status;
  const unsigned char* current = reinterpret_cast<const unsigned char*>(buf);

  if(data_len > 32) data_len = 32;
  unsigned char blank_len = dynamic_payloads_enabled ? 0 : 32 - data_len;

  csn(LOW);
  status =TxSPIbyte( writeType );
  while ( data_len-- ) TxSPIbyte(*current++);
  while ( blank_len-- ) TxSPIbyte(0);
  csn(HIGH);

  return status;
}

/****************************************************************************/

unsigned char RF24::read_payload(void* buf, unsigned char data_len)
{
  unsigned char status;
  unsigned char* current = reinterpret_cast<unsigned char*>(buf);

  if(data_len > payload_size) data_len = payload_size;
  unsigned char blank_len = dynamic_payloads_enabled ? 0 : 32 - data_len;

  csn(LOW);
  status = TxSPIbyte( R_RX_PAYLOAD );
  while ( data_len-- ) *current++ = TxSPIbyte(0xFF);
  while ( blank_len-- ) TxSPIbyte(0xff);
  csn(HIGH);

  return status;
}

/****************************************************************************/

unsigned char RF24::flush_rx(void)
{
  return spiTrans( FLUSH_RX );
}

/****************************************************************************/

unsigned char RF24::flush_tx(void)
{
  return spiTrans( FLUSH_TX );
}

/****************************************************************************/

unsigned char RF24::spiTrans(unsigned char cmd){

  unsigned char status;

  csn(LOW);
  status = TxSPIbyte( cmd );
  csn(HIGH);

  return status;
}

/****************************************************************************/

unsigned char RF24::get_status(void)
{
  return spiTrans(NOP);
}

/****************************************************************************/

RF24::RF24(unsigned char id):
  unit_id(id), p_variant(false), payload_size(32), dynamic_payloads_enabled(false), addr_width(5)
{
}

/****************************************************************************/

void RF24::setChannel(unsigned char channel)
{
  const unsigned char max_channel = 127;
  write_register(RF_CH, (channel < max_channel) ? channel : max_channel);
}

/****************************************************************************/

void RF24::setPayloadSize(unsigned char size)
{
  payload_size = /*min(size,32)*/ (size < 32) ? size : 32;
}

/****************************************************************************/

unsigned char RF24::getPayloadSize(void)
{
  return payload_size;
}

/****************************************************************************/

void RF24::begin(void)
{
    ce(LOW);
  	csn(HIGH);

  // Must allow the radio time to settle else configuration bits will not necessarily stick.
  // This is actually only required following power up but some settling time also appears to
  // be required after resets too. For full coverage, we'll always assume the worst.
  // Enabling 16b CRC is by far the most obvious case if the wrong timing is used - or skipped.
  // Technically we require 4.5ms + 14us as a worst case. We'll just call it 5ms for good measure.
  // WARNING: Delay is based on P-variant whereby non-P *may* require different timing.
  delay( 5 ) ;

  // Set 1500uS (minimum for 32B payload in ESB@250KBPS) timeouts, to make testing a little easier
  // WARNING: If this is ever lowered, either 250KBS mode with AA is broken or maximum packet
  // sizes must never be used. See documentation for a more complete explanation.
  setRetries(5,15);

  // Reset value is MAX
  //setPALevel( RF24_PA_MAX ) ;

  // Determine if this is a p or non-p RF24 module and then
  // reset our data rate back to default value. This works
  // because a non-P variant won't allow the data rate to
  // be set to 250Kbps.
  if( setDataRate( RF24_250KBPS ) )
  {
    p_variant = true ;
  }

  // Then set the data rate to the slowest (and most reliable) speed supported by all
  // hardware.
  setDataRate( RF24_1MBPS ) ;

  // Initialize CRC and request 2-byte (16bit) CRC
  setCRCLength( RF24_CRC_16 ) ;

  // Disable dynamic payloads, to match dynamic_payloads_enabled setting - Reset value is 0
  //write_register(DYNPD,0);

  // Reset current status
  // Notice reset and flush is the last thing we do
  write_register(STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

  // Set up default configuration.  Callers can always change it later.
  // This channel should be universally safe and not bleed over into adjacent
  // spectrum.
  setChannel(76);

  // Flush buffers
  flush_rx();
  flush_tx();

  powerUp(); //Power up by default when begin() is called

  // Enable PTX, do not write CE high so radio will remain in standby I mode ( 130us max to transition to RX or TX instead of 1500us from powerUp )
  // PTX should use only 22uA of power
  write_register(CONFIG, ( read_register(CONFIG) ) & ~_BV(PRIM_RX) );

}

/****************************************************************************/

void RF24::startListening(void)
{
  powerUp();
  write_register(CONFIG, read_register(CONFIG) | _BV(PRIM_RX));
  write_register(STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

  // Restore the pipe0 adddress, if exists
  if (pipe0_reading_address[0] > 0){
    write_register(RX_ADDR_P0, pipe0_reading_address, addr_width);
  }

  // Flush buffers
  //flush_rx();
  flush_tx();

  // Go!
  ce(HIGH);

}

/****************************************************************************/

void RF24::stopListening(void)
{

  ce(LOW);
  DELAYMICRO(130);
  flush_tx();
  //flush_rx();

  write_register(CONFIG, ( read_register(CONFIG) ) & ~_BV(PRIM_RX) );
  DELAYMICRO/*delayMicroseconds*/(130); //Found that adding this delay back actually increases response time
}

/****************************************************************************/

void RF24::powerDown(void)
{
  ce(LOW); // Guarantee CE is low on powerDown
  write_register(CONFIG,read_register(CONFIG) & ~_BV(PWR_UP));
}

/****************************************************************************/

//Power up now. Radio will not power down unless instructed by MCU for config changes etc.
void RF24::powerUp(void)
{
   unsigned char cfg = read_register(CONFIG);

   // if not powered up then power up and wait for the radio to initialize
   if (!(cfg & _BV(PWR_UP))){
      write_register(CONFIG,read_register(CONFIG) | _BV(PWR_UP));

      // For nRF24L01+ to go from power down mode to TX or RX mode it must first pass through stand-by mode.
	  // There must be a delay of Tpd2stby (see Table 16.) after the nRF24L01+ leaves power down mode before
	  // the CEis set high. - Tpd2stby can be up to 5ms per the 1.0 datasheet
      delay(5);

   }
}

//Similar to the previous write, clears the interrupt flags
bool RF24::write( const void* buf, unsigned char len, const bool multicast )
{
	//Start Writing
	startFastWrite(buf,len,multicast);

	//Wait until complete or failed
	while( ! ( get_status()  & ( _BV(TX_DS) | _BV(MAX_RT) ))); 
	ce(LOW);

	unsigned char status = write_register(STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

  //Max retries exceeded
  if( status & _BV(MAX_RT)){
  	flush_tx(); //Only going to be 1 packet int the FIFO at a time using this method, so just flush
  	return 0;
  }
	//TX OK 1 or 0
  return 1;
}

bool RF24::write( const void* buf, unsigned char len ){
	return write(buf,len,0);
}
/****************************************************************************/

//For general use, the interrupt flags are not important to clear
bool RF24::writeBlocking( const void* buf, unsigned char len, unsigned long timeout )
{
	//Block until the FIFO is NOT full.
	//Keep track of the MAX retries and set auto-retry if seeing failures
	//This way the FIFO will fill up and allow blocking until packets go through
	//The radio will auto-clear everything in the FIFO as long as CE remains high

	unsigned long timer = millis();						  //Get the time that the payload transmission started

	while( ( get_status()  & ( _BV(TX_FULL) ))) {		  //Blocking only if FIFO is full. This will loop and block until TX is successful or timeout

		if( get_status() & _BV(MAX_RT)){					  //If MAX Retries have been reached
			reUseTX();										  //Set re-transmit and clear the MAX_RT interrupt flag
			if(millis() - timer > timeout){ return 0; }		  //If this payload has exceeded the user-defined timeout, exit and return 0
		}
  	}

  	//Start Writing
	startFastWrite(buf,len,0);								  //Write the payload if a buffer is clear

	return 1;												  //Return 1 to indicate successful transmission
}

/****************************************************************************/

void RF24::reUseTX(){
		write_register(STATUS,_BV(MAX_RT) );			  //Clear max retry flag
		spiTrans( REUSE_TX_PL );
		ce(LOW);										  //Re-Transfer packet
		ce(HIGH);
}

/****************************************************************************/

bool RF24::writeFast( const void* buf, unsigned char len, const bool multicast )
{
	//Block until the FIFO is NOT full.
	//Keep track of the MAX retries and set auto-retry if seeing failures
	//Return 0 so the user can control the retrys and set a timer or failure counter if required
	//The radio will auto-clear everything in the FIFO as long as CE remains high

	while( ( get_status()  & ( _BV(TX_FULL) ))) {			  //Blocking only if FIFO is full. This will loop and block until TX is successful or fail

		if( get_status() & _BV(MAX_RT)){
			//reUseTX();										  //Set re-transmit
			write_register(STATUS,_BV(MAX_RT) );			  //Clear max retry flag
			return 0;										  //Return 0. The previous payload has been retransmitted
															  //From the user perspective, if you get a 0, just keep trying to send the same payload
		}
  	}
		     //Start Writing
	startFastWrite(buf,len,multicast);

	return 1;
}

bool RF24::writeFast( const void* buf, unsigned char len ){
	return writeFast(buf,len,0);
}

/****************************************************************************/

//Per the documentation, we want to set PTX Mode when not listening. Then all we do is write data and set CE high
//In this mode, if we can keep the FIFO buffers loaded, packets will transmit immediately (no 130us delay)
//Otherwise we enter Standby-II mode, which is still faster than standby mode
//Also, we remove the need to keep writing the config register over and over and delaying for 150 us each time if sending a stream of data

void RF24::startFastWrite( const void* buf, unsigned char len, const bool multicast){ //TMRh20

	//write_payload( buf,len);
	write_payload( buf, len,multicast ? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD ) ;
	ce(HIGH);

}


//Added the original startWrite back in so users can still use interrupts, ack payloads, etc
//Allows the library to pass all tests
void RF24::startWrite( const void* buf, unsigned char len, const bool multicast ){

  // Send the payload

  //write_payload( buf, len );
  write_payload( buf, len,multicast? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD ) ;
  ce(HIGH);
  DELAYMICRO(10);
  ce(LOW);


}

bool RF24::txStandBy(){

	while( ! (read_register(FIFO_STATUS) & _BV(TX_EMPTY)) ){
		if( get_status() & _BV(MAX_RT)){
			write_register(STATUS,_BV(MAX_RT) );
			ce(LOW);
			flush_tx();    //Non blocking, flush the data
			return 0;
		}
	}

	ce(LOW);			   //Set STANDBY-I mode
	return 1;
}

bool RF24::txStandBy(unsigned long timeout){

	unsigned long start = millis();

	while( ! (read_register(FIFO_STATUS) & _BV(TX_EMPTY)) ){
		if( get_status() & _BV(MAX_RT)){
			write_register(STATUS,_BV(MAX_RT) );
				ce(LOW);										  //Set re-transmit
				ce(HIGH);
				if(millis() - start >= timeout){
					ce(LOW); flush_tx(); return 0;
				}
		}
	}

	
	ce(LOW);				   //Set STANDBY-I mode
	return 1;

}
/****************************************************************************/

void RF24::maskIRQ(bool tx, bool fail, bool rx){

	write_register(CONFIG, ( read_register(CONFIG) ) | fail << MASK_MAX_RT | tx << MASK_TX_DS | rx << MASK_RX_DR  );
}

/****************************************************************************/

unsigned char RF24::getDynamicPayloadSize(void)
{
  unsigned char result = 0;

  csn(LOW);
  TxSPIbyte( R_RX_PL_WID );
  result = TxSPIbyte(0xff);
  csn(HIGH);

  if(result > 32) { flush_rx(); return 0; }
  return result;
}

/****************************************************************************/

bool RF24::available(void)
{
  return available(0);
}

/****************************************************************************/

bool RF24::available(unsigned char* pipe_num)
{

  //Check the FIFO buffer to see if data is waitng to be read

  if (!( read_register(FIFO_STATUS) & _BV(RX_EMPTY) )){

    // If the caller wants the pipe number, include that
    if ( pipe_num ){
	  unsigned char status = get_status();
      *pipe_num = ( status >> RX_P_NO ) & /*B111*/7;
  	}
  	return 1;
  }

  return 0;


}

/****************************************************************************/

void RF24::read( void* buf, unsigned char len ){

  // Fetch the payload
  read_payload( buf, len );

  //Clear the two possible interrupt flags with one command
  write_register(STATUS,_BV(RX_DR) | _BV(MAX_RT) | _BV(TX_DS) );

}

/****************************************************************************/

void RF24::whatHappened(bool& tx_ok,bool& tx_fail,bool& rx_ready)
{
  // Read the status & reset the status in one easy call
  // Or is that such a good idea?
  unsigned char status = write_register(STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

  // Report to the user what happened
  tx_ok = status & _BV(TX_DS);
  tx_fail = status & _BV(MAX_RT);
  rx_ready = status & _BV(RX_DR);
}

/****************************************************************************/

void RF24::openWritingPipe(unsigned long long value)
{
  // Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
  // expects it LSB first too, so we're good.

  write_register(RX_ADDR_P0, reinterpret_cast<unsigned char*>(&value), addr_width);
  write_register(TX_ADDR, reinterpret_cast<unsigned char*>(&value), addr_width);

  //const unsigned char max_payload_size = 32;
  //write_register(RX_PW_P0,min(payload_size,max_payload_size));
  write_register(RX_PW_P0,payload_size);
}

/****************************************************************************/

void RF24::openWritingPipe(const unsigned char *address)
{
  // Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
  // expects it LSB first too, so we're good.

  write_register(RX_ADDR_P0,address, addr_width);
  write_register(TX_ADDR, address, addr_width);

  //const unsigned char max_payload_size = 32;
  //write_register(RX_PW_P0,min(payload_size,max_payload_size));
  write_register(RX_PW_P0,payload_size);
}

/****************************************************************************/

static const unsigned char child_pipe[] /*PROGMEM*/ =
{
  RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5
};
static const unsigned char child_payload_size[] /*PROGMEM*/ =
{
  RX_PW_P0, RX_PW_P1, RX_PW_P2, RX_PW_P3, RX_PW_P4, RX_PW_P5
};
static const unsigned char child_pipe_enable[] /*PROGMEM*/ =
{
  ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4, ERX_P5
};

void RF24::openReadingPipe(unsigned char child, unsigned long long address)
{
  // If this is pipe 0, cache the address.  This is needed because
  // openWritingPipe() will overwrite the pipe 0 address, so
  // startListening() will have to restore it.
  if (child == 0){
    memcpy(pipe0_reading_address,&address,addr_width);
  }

  if (child <= 6)
  {
    // For pipes 2-5, only write the LSB
    if ( child < 2 )
      write_register(pgm_read_byte(&child_pipe[child]), reinterpret_cast<const unsigned char*>(&address), addr_width);
    else
      write_register(pgm_read_byte(&child_pipe[child]), reinterpret_cast<const unsigned char*>(&address), 1);

    write_register(pgm_read_byte(&child_payload_size[child]),payload_size);

    // Note it would be more efficient to set all of the bits for all open
    // pipes at once.  However, I thought it would make the calling code
    // more simple to do it this way.
    write_register(EN_RXADDR,read_register(EN_RXADDR) | _BV(pgm_read_byte(&child_pipe_enable[child])));
  }
}

/****************************************************************************/
void RF24::setAddressWidth(unsigned char a_width){

	if(a_width -= 2){
		write_register(SETUP_AW,a_width%4);
		addr_width = (a_width%4) + 2;
	}

}

/****************************************************************************/

void RF24::openReadingPipe(unsigned char child, const unsigned char *address)
{
  // If this is pipe 0, cache the address.  This is needed because
  // openWritingPipe() will overwrite the pipe 0 address, so
  // startListening() will have to restore it.
  if (child == 0){
    memcpy(pipe0_reading_address,address,addr_width);
  }
  if (child <= 6)
  {
    // For pipes 2-5, only write the LSB
    if ( child < 2 ){
      write_register(pgm_read_byte(&child_pipe[child]), address, addr_width);
    }else{
      write_register(pgm_read_byte(&child_pipe[child]), address, 1);
	}
    write_register(pgm_read_byte(&child_payload_size[child]),payload_size);

    // Note it would be more efficient to set all of the bits for all open
    // pipes at once.  However, I thought it would make the calling code
    // more simple to do it this way.
    write_register(EN_RXADDR,read_register(EN_RXADDR) | _BV(pgm_read_byte(&child_pipe_enable[child])));

  }

}

/****************************************************************************/

void RF24::closeReadingPipe( unsigned char pipe )
{
  write_register(EN_RXADDR,read_register(EN_RXADDR) & ~_BV(pgm_read_byte(&child_pipe_enable[pipe])));
}

/****************************************************************************/

void RF24::toggle_features(void)
{

  csn(LOW);
  TxSPIbyte( ACTIVATE );
  TxSPIbyte( 0x73 );
  csn(HIGH);
}

/****************************************************************************/

void RF24::enableDynamicPayloads(void)
{
  // Enable dynamic payload throughout the system

    toggle_features();
    write_register(FEATURE,read_register(FEATURE) | _BV(EN_DPL) );


//  IF_SERIAL_DEBUG(printf("FEATURE=%i\r\n",read_register(FEATURE)));

  // Enable dynamic payload on all pipes
  //
  // Not sure the use case of only having dynamic payload on certain
  // pipes, so the library does not support it.
  write_register(DYNPD,read_register(DYNPD) | _BV(DPL_P5) | _BV(DPL_P4) | _BV(DPL_P3) | _BV(DPL_P2) | _BV(DPL_P1) | _BV(DPL_P0));

  dynamic_payloads_enabled = true;
}

/****************************************************************************/

void RF24::enableAckPayload(void)
{
  //
  // enable ack payload and dynamic payload features
  //

    toggle_features();
    write_register(FEATURE,read_register(FEATURE) | _BV(EN_ACK_PAY) | _BV(EN_DPL) );

  //
  // Enable dynamic payload on pipes 0 & 1
  //

  write_register(DYNPD,read_register(DYNPD) | _BV(DPL_P1) | _BV(DPL_P0));
  dynamic_payloads_enabled = true;
}

/****************************************************************************/

void RF24::enableDynamicAck(void){
  //
  // enable dynamic ack features
  //
    toggle_features();
    write_register(FEATURE,read_register(FEATURE) | _BV(EN_DYN_ACK) );

}

/****************************************************************************/

void RF24::writeAckPayload(unsigned char pipe, const void* buf, unsigned char len)
{
  const unsigned char* current = reinterpret_cast<const unsigned char*>(buf);

  unsigned char data_len = (len < 32) ? len : 32; // min(len,32);

  csn(LOW);
  TxSPIbyte(W_ACK_PAYLOAD | ( pipe & /*B111*/7 ) );
  while ( data_len-- ) TxSPIbyte(*current++);
  csn(HIGH);

}

/****************************************************************************/

bool RF24::isAckPayloadAvailable(void)
{
  return ! read_register(FIFO_STATUS) & _BV(RX_EMPTY);
}

/****************************************************************************/

bool RF24::isPVariant(void)
{
  return p_variant ;
}

/****************************************************************************/

void RF24::setAutoAck(bool enable)
{
  if ( enable )
    write_register(EN_AA, /*B111111*/0x3f);
  else
    write_register(EN_AA, 0);
}

/****************************************************************************/

void RF24::setAutoAck( unsigned char pipe, bool enable )
{
  if ( pipe <= 6 )
  {
    unsigned char en_aa = read_register( EN_AA ) ;
    if( enable )
    {
      en_aa |= _BV(pipe) ;
    }
    else
    {
      en_aa &= ~_BV(pipe) ;
    }
    write_register( EN_AA, en_aa ) ;
  }
}

/****************************************************************************/

bool RF24::testCarrier(void)
{
  return ( read_register(CD) & 1 );
}

/****************************************************************************/

bool RF24::testRPD(void)
{
  return ( read_register(RPD) & 1 ) ;
}

/****************************************************************************/

void RF24::setPALevel(unsigned char level)
{

  unsigned char setup = read_register(RF_SETUP) & /*0b11111000*/0xf8;

  if(level > 3){  						// If invalid level, go to max PA
	  level = (RF24_PA_MAX << 1) + 1;		// +1 to support the SI24R1 chip extra bit
  }else{
	  level = (level << 1) + 1;	 		// Else set level as requested
  }


  write_register( RF_SETUP, setup |= level ) ;	// Write it to the chip
}

/****************************************************************************/

unsigned char RF24::getPALevel(void)
{

  return (read_register(RF_SETUP) & (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH))) >> 1 ;
}

/****************************************************************************/

bool RF24::setDataRate(rf24_datarate_e speed)
{
  bool result = false;
  unsigned char setup = read_register(RF_SETUP) ;

  // HIGH and LOW '00' is 1Mbs - our default
  setup &= ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH)) ;
  if( speed == RF24_250KBPS )
  {
    // Must set the RF_DR_LOW to 1; RF_DR_HIGH (used to be RF_DR) is already 0
    // Making it '10'.
    setup |= _BV( RF_DR_LOW ) ;
  }
  else
  {
    // Set 2Mbs, RF_DR (RF_DR_HIGH) is set 1
    // Making it '01'
    if ( speed == RF24_2MBPS )
    {
      setup |= _BV(RF_DR_HIGH);
    }
  }
  write_register(RF_SETUP,setup);

  // Verify our result
  if ( read_register(RF_SETUP) == setup )
  {
    result = true;
  }

  return result;
}

/****************************************************************************/

rf24_datarate_e RF24::getDataRate( void )
{
  rf24_datarate_e result ;
  unsigned char dr = read_register(RF_SETUP) & (_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));

  // switch uses RAM (evil!)
  // Order matters in our case below
  if ( dr == _BV(RF_DR_LOW) )
  {
    // '10' = 250KBPS
    result = RF24_250KBPS ;
  }
  else if ( dr == _BV(RF_DR_HIGH) )
  {
    // '01' = 2MBPS
    result = RF24_2MBPS ;
  }
  else
  {
    // '00' = 1MBPS
    result = RF24_1MBPS ;
  }
  return result ;
}

/****************************************************************************/

void RF24::setCRCLength(rf24_crclength_e length)
{
  unsigned char config = read_register(CONFIG) & ~( _BV(CRCO) | _BV(EN_CRC)) ;

  // switch uses RAM (evil!)
  if ( length == RF24_CRC_DISABLED )
  {
    // Do nothing, we turned it off above.
  }
  else if ( length == RF24_CRC_8 )
  {
    config |= _BV(EN_CRC);
  }
  else
  {
    config |= _BV(EN_CRC);
    config |= _BV( CRCO );
  }
  write_register( CONFIG, config ) ;
}

/****************************************************************************/

rf24_crclength_e RF24::getCRCLength(void)
{
  rf24_crclength_e result = RF24_CRC_DISABLED;
  unsigned char config = read_register(CONFIG) & ( _BV(CRCO) | _BV(EN_CRC)) ;

  if ( config & _BV(EN_CRC ) )
  {
    if ( config & _BV(CRCO) )
      result = RF24_CRC_16;
    else
      result = RF24_CRC_8;
  }

  return result;
}

/****************************************************************************/

void RF24::disableCRC( void )
{
  unsigned char disable = read_register(CONFIG) & ~_BV(EN_CRC) ;
  write_register( CONFIG, disable ) ;
}

/****************************************************************************/
void RF24::setRetries(unsigned char delay, unsigned char count)
{
 write_register(SETUP_RETR,(delay&0xf)<<ARD | (count&0xf)<<ARC);
}

