/*=============================================================================
 * Copyright (c) 2020, Francisco Timez <franciscotimez@gmail.com>
 * All rights reserved.
 * License: mit (see LICENSE.txt)
 * Date: 2020/09/26
 *===========================================================================*/

/*=====[Inclusions of function dependencies]=================================*/

#include "MCP23017.h"
#include "sapi.h"

//INICIO del testeo (Eliminar para convertir el archivo en una libreria)

// Tipo de datos que contiene una copia de registros importantes del Expansor
mcp23017_t expander1;

// Arreglo de pines utilizado en el ejemplo de testeo.
mcpPinMap_t PINS[16] = {
		GPA0,
		GPA1,
		GPA2,
		GPA3,
		GPA4,
		GPA5,
		GPA6,
		GPA7,
		GPB0,
		GPB1,
		GPB2,
		GPB3,
		GPB4,
		GPB5,
		GPB6,
		GPB7
};

/*=====[Main function, program entry point after power on or reset]==========*/

int main( void )
{
	// ----- Setup -----------------------------------
	boardInit();

	// Configuro la direccion del expansor. (ver MCP23017.h)
	expander1.address = MCP_ADDRESS1;

	MCP23017_init(&expander1, I2C_400KHZ);

	MCP23017_portMode(&expander1, PORTA, 0x00);

	MCP23017_portMode(&expander1, PORTB, 0x00);

	MCP23017_pinMode(&expander1, GPB0, INPUT);
	MCP23017_pinMode(&expander1, GPB1, INPUT);
	MCP23017_pinMode(&expander1, GPB2, INPUT);
	MCP23017_pinMode(&expander1, GPB3, INPUT);

	delay(1);

	// ----- Repeat for ever -------------------------
	while( true ) {
		gpioToggle(LEDB);
		for(int i = 0 ; i < 8 ; i++){
			MCP23017_pinWrite(&expander1, PINS[i], ON);

			if(MCP23017_pinRead(&expander1, GPB0))
				MCP23017_pinWrite(&expander1, GPB4, ON);
			else
				MCP23017_pinWrite(&expander1, GPB4, OFF);

			if(MCP23017_pinRead(&expander1, GPB1))
				MCP23017_pinWrite(&expander1, GPB5, ON);
			else
				MCP23017_pinWrite(&expander1, GPB5, OFF);

			if(MCP23017_pinRead(&expander1, GPB2))
				MCP23017_pinWrite(&expander1, GPB6, ON);
			else
				MCP23017_pinWrite(&expander1, GPB6, OFF);

			if(MCP23017_pinRead(&expander1, GPB3))
				MCP23017_pinWrite(&expander1, GPB7, ON);
			else
				MCP23017_pinWrite(&expander1, GPB7, OFF);

			delay(100);
			MCP23017_pinWrite(&expander1, PINS[i], OFF);
			//delay(500);
		}

	}

	// YOU NEVER REACH HERE, because this program runs directly or on a
	// microcontroller and is not called by any Operating System, as in the
	// case of a PC program.
	return 0;
}
//FIN del testeo

bool_t 	MCP23017_init(mcp23017_t *mcp, mcpSpeed_t speed){
	bool_t state_init;
	uint8_t init_pins[13] = {MCP23017_IODIRA, // sub address IODIRA + IODIRB
			0xFF, // inicia pines como entradas
			0xFF, // inicia pines como entradas
			0x00,
			0x00,
			0x00,
			0x00,
			0x00,
			0x00,
			0x00,
			0x00,
			0x00,
			0x00
	};

	mcp->DIRportA = 0xFF;
	mcp->DIRportB = 0xFF;
	mcp->PUportA = 0x00;
	mcp->PUportB = 0x00;

	i2cInit( I2C0 , speed );

	state_init = i2cWrite(I2C0 , mcp->address, init_pins, 13, TRUE);

	return state_init;
}

// Funciones orientadas a pines
bool_t 	MCP23017_pinMode(mcp23017_t *mcp, mcpPinMap_t pin, mcpPinMode_t mode){
	bool_t state;
	uint8_t buffer1[2];
	uint8_t buffer2[2];

	switch(mode){
	case OUTPUT:
		if(pin < GPB0){ // port A
			mcp->DIRportA &= 0 << pin;
			mcp->PUportB &= 0 << pin;
		}
		else{			// port B
			mcp->DIRportB &= 0 << (pin-GPB0);
			mcp->PUportB &= 0 << (pin-GPB0);
		}
		break;
	case INPUT:
		if(pin < GPB0){ // port A
			mcp->DIRportA |= 1 << pin;
			mcp->PUportA &= 0 << pin;
		}
		else{			// port B
			mcp->DIRportB |= 1 << (pin-GPB0);
			mcp->PUportB &= 0 << (pin-GPB0);
		}
		break;
	case INPUT_PULLUP:
		if(pin < GPB0){ // port A
			mcp->DIRportA |= 1 << pin;
			mcp->PUportB |= 1 << pin;
		}
		else{			// port B
			mcp->DIRportB |= 1 << (pin-GPB0);
			mcp->PUportB |= 1 << (pin-GPB0);
		}
		break;
	}

	if(pin < GPB0){ // port A
		buffer1[0] = MCP23017_IODIRA;
		buffer1[1] = mcp->DIRportA;
		buffer2[0] = MCP23017_GPPUA;
		buffer2[1] = mcp->PUportA;
	}
	else{			// port B
		buffer1[0] = MCP23017_IODIRB;
		buffer1[1] = mcp->DIRportB;
		buffer2[0] = MCP23017_GPPUB;
		buffer2[1] = mcp->PUportB;
	}

	state = i2cWrite(I2C0 , mcp->address, buffer1, 2, TRUE);
	state &= i2cWrite(I2C0 , mcp->address, buffer2, 2, TRUE);

	return state;
}

bool_t 	MCP23017_pinWrite(mcp23017_t *mcp, mcpPinMap_t pin, bool_t pinState){
	bool_t state;

	if(pin < GPB0){ // port A
		mcp->STAportA = pinState == ON ? (mcp->STAportA |= 1 << pin) : (mcp->STAportA &= ~(1 << pin));
		state = MCP23017_portWrite(mcp, PORTA, mcp->STAportA);
	}
	else{			// port B
		mcp->STAportB = pinState == ON ? (mcp->STAportB |= 1 << (pin - GPB0)) : (mcp->STAportB &= ~(1 << (pin - GPB0) ));
		state = MCP23017_portWrite(mcp, PORTB, mcp->STAportB);
	}
	return state;
}

bool_t 	MCP23017_pinRead(mcp23017_t *mcp, mcpPinMap_t pin){
	bool_t state;
	uint8_t buffer1, subaddress;

	if(pin < GPB0) // port A
		state = ( MCP23017_portRead(mcp, PORTA) & 1 << pin );
	else			// port B
		state = ( MCP23017_portRead(mcp, PORTB) & 1 << (pin - GPB0) );
	return state;
}

// Funciones orientadas a puertos
// Configura el puerto como entrada o salida.
bool_t 	MCP23017_portMode(mcp23017_t *mcp, mcpPortMap_t port, uint8_t mode){
	bool_t state;
	uint8_t buffer[2];
	switch(port){
	case PORTA:
		buffer[0] = MCP23017_IODIRA;
		mcp->DIRportA = mode;
		buffer[1] = mcp->DIRportA;
		break;
	case PORTB:
		buffer[0] = MCP23017_IODIRB;
		mcp->DIRportB = mode;
		buffer[1] = mcp->DIRportB;
		break;
	}

	state = i2cWrite(I2C0 , mcp->address, buffer, 2, TRUE);

	return state;
}

bool_t 	MCP23017_portWrite(mcp23017_t *mcp, mcpPortMap_t port, uint8_t state_port){
	bool_t state;
	uint8_t buffer[2];

	buffer[1] = state_port;

	switch(port){
	case PORTA:
		buffer[0] = MCP23017_OLATA;
		state = i2cWrite(I2C0 , mcp->address, buffer, 2, TRUE);
		break;
	case PORTB:
		buffer[0] = MCP23017_OLATB;
		state = i2cWrite(I2C0 , mcp->address, buffer, 2, TRUE);
		break;
	}
	return state;
}

uint8_t MCP23017_portRead(mcp23017_t *mcp, mcpPortMap_t port){
	bool_t state;
	uint8_t buffer, subaddress;

	switch(port){
	case PORTA:
		subaddress = MCP23017_GPIOA;
		i2cWriteRead(I2C0, mcp->address, &subaddress, 1, TRUE, &buffer, 1, TRUE);
		break;
	case PORTB:
		subaddress = MCP23017_GPIOB;
		i2cWriteRead(I2C0, mcp->address, &subaddress, 1, TRUE, &buffer, 1, TRUE);
		break;
	}
	return buffer;
}
