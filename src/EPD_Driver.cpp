/*
	EPD_Driver.cpp
	
	--COPYRIGHT--
  * Brief The drivers and functions of development board
  * Copyright (c) 2012-2021 Pervasive Displays Inc. All rights reserved.
  *  Authors: Pervasive Displays Inc.
  *  Redistribution and use in source and binary forms, with or without
  *  modification, are permitted provided that the following conditions
  *  are met:
  *  1. Redistributions of source code must retain the above copyright
  *     notice, this list of conditions and the following disclaimer.
  *  2. Redistributions in binary form must reproduce the above copyright
  *     notice, this list of conditions and the following disclaimer in
  *     the documentation and/or other materials provided with the
  *     distribution.
  *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#if defined(ENERGIA)
#include "Energia.h"
#else
#include "Arduino.h"
#endif

#include "EPD_Driver.h"

// ---------- PUBLIC FUNCTIONS -----------

EPD_Driver::EPD_Driver(eScreen_EPD_t eScreen_EPD, pins_t board)
{
	spi_basic = board;
	
	// Type
	pdi_cp = (uint16_t) eScreen_EPD;
	pdi_size = (uint16_t) (eScreen_EPD >> 8);

	for ( uint8_t i = 0; i < sizeof( EPD_idx )/sizeof(* EPD_idx ); i++ )
	{
		if (pdi_size == EPD_idx[i])
		{
			_size = i;
			break;
		}
	}

	// Config registers according to screen size
	memcpy(register_data, register_data_mid, sizeof(register_data_mid));
}

// CoG initialization function
//		Implements Tcon (COG) power-on and temperature input to COG
//		- INPUT:
//			- none but requires global variables on SPI pinout and config register data
void EPD_Driver::COG_initial()
{	
	pinMode( spi_basic.panelBusy, INPUT );     //All Pins 0
	
	pinMode( spi_basic.panelDC, OUTPUT );
	digitalWrite(spi_basic.panelDC, HIGH);
	pinMode( spi_basic.panelReset, OUTPUT );
	digitalWrite(spi_basic.panelReset, HIGH);
	pinMode( spi_basic.panelCS, OUTPUT );
	digitalWrite(spi_basic.panelCS, HIGH);


	if (spi_basic.panelON_EXT2 != 0xff)
	{
		pinMode( spi_basic.panelON_EXT2, OUTPUT );
		pinMode( spi_basic.panelSPI43_EXT2, OUTPUT );
		digitalWrite( spi_basic.panelON_EXT2, HIGH );    //PANEL_ON# = 1
		digitalWrite( spi_basic.panelSPI43_EXT2, LOW );
	}

	// pinMode(spi_basic.flashCS, OUTPUT);
    // digitalWrite(spi_basic.flashCS, HIGH);
	
	// SPI init
	// #ifndef SPI_CLOCK_MAX
		// #define SPI_CLOCK_MAX 16000000
	// #endif

	SPI.begin();

	#if defined(ENERGIA)
	{
		SPI.setDataMode(SPI_MODE0);
		SPI.setClockDivider(SPI_CLOCK_DIV32);
		// SPI.setClockDivider(16000000 / min(16000000, 4000000));
		SPI.setBitOrder(MSBFIRST);
	}
	#else
	{
		SPISettings _settingScreen;
		_settingScreen = {8000000, MSBFIRST, SPI_MODE0};
		SPI.begin();
		SPI.beginTransaction(_settingScreen);
	}
	#endif
}

// CoG shutdown function
//		Shuts down the CoG and DC/DC circuit after all update functions
//		- INPUT:
//			- none but requires global variables on SPI pinout and config register data
void EPD_Driver::COG_powerOff()
{
	uint8_t register_turnOff[] = {0x7f, 0x7d, 0x00};
	_sendIndexData(0x09, register_turnOff, 3);
	delay(200);

	while ( digitalRead( spi_basic.panelBusy ) != HIGH );
	
	digitalWrite( spi_basic.panelDC, LOW );
	digitalWrite( spi_basic.panelCS, LOW );
	digitalWrite( spi_basic.panelBusy, LOW );
	delay( 150 );
	digitalWrite( spi_basic.panelReset, LOW );
}

// Global Update function
//		Implements global update functionality on either small/mid EPD
//		- INPUT:
//			- two image data (either BW and 0x00 or BW and BWR types)
void EPD_Driver::globalUpdate(const uint8_t * data1s, const uint8_t * data2s)
{
	_reset(200, 20, 200, 50, 5);

	uint8_t dtcl = 0x08; // 0=IST7232, 8=IST7236
	_sendIndexData(0x01, &dtcl, 1); //DCTL 0x10 of MTP

	// Send image data
	if (pdi_size == 0x58)
	{
		uint8_t data1[] = {0x00, 0x1f, 0x50, 0x00, 0x1f, 0x03}; // DUW
		_sendIndexData(0x13, data1, 6); // DUW
		uint8_t data2[] = {0x00, 0x1f, 0x00, 0xc9}; // DRFW
		_sendIndexData(0x90, data2, 4); // DRFW
		uint8_t data3[] = {0x1f, 0x50, 0x14}; // RAM_RW
		_sendIndexData(0x12, data3, 3); // RAM_RW
	}
	else // 7.41"
	{
		uint8_t data1[] = {0x00, 0x3b, 0x00, 0x00, 0x1f, 0x03}; // DUW
		_sendIndexData(0x13, data1, 6); // DUW
		uint8_t data2[] = {0x00, 0x3b, 0x00, 0xc9}; // DRFW
		_sendIndexData(0x90, data2, 4); // DRFW
		uint8_t data3[] = {0x3b, 0x00, 0x14}; // RAM_RW
		_sendIndexData(0x12, data3, 3); // RAM_RW
	}

	// send first frame
    _sendIndexData(0x10, data1s, image_data_size[_size]); // First frame

	if (pdi_size == 0x58)
	{
		uint8_t data33[] = {0x1f, 0x50, 0x14}; // RAM_RW
		_sendIndexData(0x12, data33, 3); // RAM_RW
	}
	else
	{
		uint8_t data34[] = {0x3b, 0x00, 0x14}; // RAM_RW
		_sendIndexData(0x12, data34, 3); // RAM_RW
	}
	// send second frame
    _sendIndexData(0x11, data2s, image_data_size[_size]); // Second frame

	_DCDC_softStart_Mid();

	_displayRefresh();

	_DCDC_softShutdown_Mid();
	// digitalWrite(spi_basic.panelCS, HIGH);
}

// ---------- PROTECTED FUNCTIONS -----------

// SPI transfer function
//		Implements SPI transfer of index and data (consult user manual for EPD SPI process)
//		- INPUT:
//			- register address
//			- pointer to data char array
//			- length/size of data
void EPD_Driver::_sendIndexData( uint8_t index, const uint8_t *data, uint32_t len )
{
	uint32_t tempp = 50;
		
	digitalWrite( spi_basic.panelDC, LOW );      //DC Low
	digitalWrite( spi_basic.panelCS, LOW );      //CS Low
	delayMicroseconds(tempp);
	SPI.transfer(index);
	delayMicroseconds(tempp);
	digitalWrite( spi_basic.panelCS, HIGH );     //CS High
	digitalWrite( spi_basic.panelDC, HIGH );     //DC High
	digitalWrite( spi_basic.panelCS, LOW );      //CS Low
	delayMicroseconds(tempp);
	for ( uint32_t i = 0; i < len; i++ )
	{
		SPI.transfer(data[ i ]);
	}
	delayMicroseconds(tempp);
	digitalWrite( spi_basic.panelCS, HIGH );     //CS High
}

// CoG soft-reset function
//		- INPUT:
//			- none but requires global variables on SPI pinout and config register data
void EPD_Driver::_softReset()
{
	_sendIndexData( 0x00, &register_data[1], 1 );	//Soft-reset, will reset to run the internal LUT for global update
	while( digitalRead( spi_basic.panelBusy ) != HIGH );
}

// EPD Screen refresh function
//		- INPUT:
//			- none but requires global variables on SPI pinout and config register data
void EPD_Driver::_displayRefresh()
{
	while (digitalRead(spi_basic.panelBusy) != HIGH)
	{
		delay(100);
	}
	uint8_t data18[] = {0x3c};
	_sendIndexData(0x15, data18, 1); //Display Refresh
	delay(5);
}

// CoG driver power-on hard reset
//		- INPUT:
//			- none but requires global variables on SPI pinout and config register data
void EPD_Driver::_reset(uint32_t ms1, uint32_t ms2, uint32_t ms3, uint32_t ms4, uint32_t ms5)
{
	// note: group delays into one array
	delay(ms1);
    digitalWrite(spi_basic.panelReset, HIGH); // RES# = 1
    delay(ms2);
    digitalWrite(spi_basic.panelReset, LOW);
    delay(ms3);
    digitalWrite(spi_basic.panelReset, HIGH);
    delay(ms4);
    digitalWrite(spi_basic.panelCS, HIGH); // CS# = 1
    delay(ms5);
}

// DC-DC power-on command
//		Implemented after image data are uploaded to CoG
//		Specific to small-sized EPDs only
//		- INPUT:
//			- none but requires global variables on SPI pinout and config register data
void EPD_Driver::_DCDC_powerOn()
{
	_sendIndexData( 0x04, &register_data[0], 1 );  //Power on
	while( digitalRead( spi_basic.panelBusy ) != HIGH );
}

// DC-DC soft-start command
//		Implemented after image data are uploaded to CoG
//		Specific to mid-sized EPDs only
void EPD_Driver::_DCDC_softStart_Mid()
{
	// COG init
    uint8_t data4[] = {0x7d};
    _sendIndexData(0x05, data4, 1);
    delay(200);
    uint8_t data5[] = {0x00};
    _sendIndexData(0x05, data5, 1);
    delay(10);
    uint8_t data6[] = {0x3f};
    _sendIndexData(0xc2, data6, 1);
    delay(1);
    uint8_t data7[] = {0x00};
    _sendIndexData(0xd8, data7, 1); // MS_SYNC mtp_0x1d
    uint8_t data8[] = {0x00};
    _sendIndexData(0xd6, data8, 1); // BVSS mtp_0x1e
    uint8_t data9[] = {0x10};
    _sendIndexData(0xa7, data9 , 1);
    delay(100);
    _sendIndexData(0xa7, data5, 1);
    delay(100);
    // uint8_t data10[] = {0x00, 0x02 };

	if (pdi_size == 0x58)
	{
		uint8_t data10[] = {0x00, 0x01}; // OSC
		_sendIndexData(0x03, data10, 2); // OSC mtp_0x12
	}
	else
	{
		uint8_t data10[] = {0x00, 0x01}; // OSC
		_sendIndexData(0x03, data10, 2); // OSC mtp_0x12
	}

    _sendIndexData(0x44, data5, 1);
    uint8_t data11[] = {0x80};
    _sendIndexData(0x45, data11, 1);
    _sendIndexData(0xa7, data9, 1);
    delay(100);
    _sendIndexData(0xa7, data7, 1);
    delay(100);
    uint8_t data12[] = {0x06};
    _sendIndexData(0x44, data12, 1);
    uint8_t data13[] = {0x82};
    _sendIndexData(0x45, data13, 1); // Temperature 0x82@25C
    _sendIndexData(0xa7, data9, 1);
    delay(100);
    _sendIndexData(0xa7, data7, 1);
    delay(100);
    uint8_t data14[] = {0x25};
    _sendIndexData(0x60, data14, 1); // TCON mtp_0x0b
    // uint8_t data15[] = {0x01 };

	if (pdi_size == 0x58)
	{
		uint8_t data15[] = {0x00}; // STV_DIR
		_sendIndexData(0x61, data15, 1); // STV_DIR mtp_0x1c
	}
	else
	{
		uint8_t data15[] = {0x00}; // STV_DIR
		_sendIndexData(0x61, data15, 1); // STV_DIR mtp_0x1c
	}

    uint8_t data16[] = {0x00};
    _sendIndexData(0x01, data16, 1); // DCTL mtp_0x10
    uint8_t data17[] = {0x00};
    _sendIndexData(0x02, data17, 1); // VCOM mtp_0x11

    // DC-DC soft-start
    uint8_t index51[] = {0x50, 0x01, 0x0a, 0x01};
    _sendIndexData(0x51, &index51[0], 2);
    uint8_t index09[] = {0x1f, 0x9f, 0x7f, 0xff};

    for (int value = 1; value <= 4; value++)
    {
        _sendIndexData(0x09, &index09[0], 1);
        index51[1] = value;
        _sendIndexData(0x51, &index51[0], 2);
        _sendIndexData(0x09, &index09[1], 1);
        delay(2);
    }
    for (int value = 1; value <= 10; value++)
    {
        _sendIndexData(0x09, &index09[0], 1);
        index51[3] = value;
        _sendIndexData(0x51, &index51[2], 2);
        _sendIndexData(0x09, &index09[1], 1);
        delay(2);
    }
    for (int value = 3; value <= 10; value++)
    {
        _sendIndexData(0x09, &index09[2], 1);
        index51[3] = value;
        _sendIndexData(0x51, &index51[2], 2);
        _sendIndexData(0x09, &index09[3], 1);
        delay(2);
    }
    for (int value = 9; value >= 2; value--)
    {
        _sendIndexData(0x09, &index09[2], 1);
        index51[2] = value;
        _sendIndexData(0x51, &index51[2], 2);
        _sendIndexData(0x09, &index09[3], 1);
        delay(2);
    }
    _sendIndexData(0x09, &index09[3], 1);
    delay(10);
}

// DC-DC soft-shutdown command
//		Implemented after image data are uploaded to CoG
//		Specific to mid-sized EPDs only
void EPD_Driver::_DCDC_softShutdown_Mid()
{
	// DC-DC off
    while (digitalRead(spi_basic.panelBusy) != HIGH)
    {
        delay(100);
    }
    uint8_t data19[] = {0x7f};
    _sendIndexData(0x09, data19, 1);
    uint8_t data20[] = {0x7d};
    _sendIndexData(0x05, data20, 1);
	uint8_t data55[] = {0x00};
    _sendIndexData(0x09, data55, 1);
    delay(200);

    while (digitalRead(spi_basic.panelBusy) != HIGH)
    {
        delay(100);
    }
    digitalWrite(spi_basic.panelDC, LOW);
    digitalWrite(spi_basic.panelCS, LOW);
    digitalWrite(spi_basic.panelReset, LOW);
    // digitalWrite(panelON_PIN, LOW); // PANEL_OFF# = 0

    digitalWrite(spi_basic.panelCS, HIGH); // CS# = 1
}