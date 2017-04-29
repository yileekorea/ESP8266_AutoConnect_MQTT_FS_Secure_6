//#include <stdio.h>

//#include <inttypes.h>

//#include <Arduino.h>

#include "gpio_MCP23S17.h"
#include <SPI.h>//this chip needs SPI



gpio_MCP23S17::gpio_MCP23S17(){

}



//return 255 if the choosed pin has no INT, otherwise return INT number
//if there's support for SPI transactions it will use SPI.usingInterrupt(intNum);
//to prevent problems from interrupt
/*USE:
  int intNumber = mcp.getInterruptNumber(gpio_int_pin);
  if (intNumber < 255){
    attachInterrupt(intNumber, keypress, FALLING);//attack interrupt
  } else {
    Serial.println("sorry, pin has no INT capabilities!");
  }
 */

int gpio_MCP23S17::getInterruptNumber(byte pin) {
	int intNum = digitalPinToInterrupt(pin);
	if (intNum != NOT_AN_INTERRUPT) {
		#if defined (SPI_HAS_TRANSACTION) && !defined(ESP8266)
			SPI.usingInterrupt(intNum);
		#endif
		return intNum;
	}
	return 255;
}





gpio_MCP23S17::gpio_MCP23S17(const uint8_t csPin,const uint8_t haenAdrs){
	postSetup(csPin,haenAdrs);
}

void gpio_MCP23S17::postSetup(const uint8_t csPin,const uint8_t haenAdrs){
	_cs = csPin;
	if (haenAdrs > 0x19 && haenAdrs < 0x28){//HAEN works between 0x20...0x27
//		_adrs = 0;
		_adrs = haenAdrs;
		_useHaen = 1;
	} else {
		_adrs = 0;
		_useHaen = 0;
	}

	_readCmd =  (_adrs << 1) | 1;
	DEBUG_MCP("_readCmd");
	DEBUG_MCP(_readCmd);
	_writeCmd = _adrs << 1;
	DEBUG_MCP("_writeCmd");
	DEBUG_MCP(_writeCmd);
}



void gpio_MCP23S17::begin(bool protocolInitOverride) {
	if (!protocolInitOverride){
	SPI.begin();
	SPI.setFrequency(10000000); //MCP23S08 Max 10MHz
	SPI.setDataMode(SPI_MODE0);

	}
	pinMode(_cs, OUTPUT);
	#if defined(ESP8266)
		digitalWrite(_cs, HIGH);
//		GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, _pinRegister(_cs));//H
		DEBUG_MCP("gpio_MCP23S17::_cs");
		DEBUG_MCP(_cs);
	#endif
	delay(100);

//	_useHaen == 1 ? _GPIOwriteByte(MCP23S17_IOCON,0b00101000) : _GPIOwriteByte(MCP23S17_IOCON,0b00100000);
	_useHaen == 1 ? _GPIOwriteByte(MCP23S17_IOCON,0b00100000) : _GPIOwriteByte(MCP23S17_IOCON,0b00100000);

	_gpioDirection = 0x00;//all out
	_gpioState = 0xFF;//all low 

}



uint16_t gpio_MCP23S17::gpioReadAddress(byte addr){
	_GPIOstartSend(1);
	SPI.transfer(addr);
	#if !defined(__SAM3X8E__) && ((ARDUINO >= 160) || (TEENSYDUINO > 121))
//		uint16_t temp = SPI.transfer16(0x0);
		uint8_t temp = SPI.transfer(0x0);
		_GPIOendSend();
		//DEBUG_MCP("gpioReadAddress-1");
		//DEBUG_MCP(temp);
		return temp;

	#else
		byte low_byte  = SPI.transfer(0x0);
		byte high_byte = SPI.transfer(0x0);
		_GPIOendSend();
		uint16_t temp = low_byte | (high_byte << 8);

		return temp;
	#endif
}


void gpio_MCP23S17::gpioPinMode(uint16_t mode){
	if (mode == INPUT){
		_gpioDirection = 0xFF;
	} else if (mode == OUTPUT){	
		_gpioDirection = 0x00;
		_gpioState = 0x00;
	} else {
		_gpioDirection = mode;
	}
	DEBUG_MCP("gpioPinMode");
	DEBUG_MCP(MCP23S17_IODIR);
	DEBUG_MCP(_gpioDirection);
//	_GPIOwriteWord(MCP23S17_IODIR,_gpioDirection);
	_GPIOwriteByte(MCP23S17_IODIR,_gpioDirection);
}

void gpio_MCP23S17::gpioPinMode(uint8_t pin, bool mode){
	if (pin < 16){//0...15
		mode == INPUT ? _gpioDirection |= (1 << pin) :_gpioDirection &= ~(1 << pin);
		_GPIOwriteWord(MCP23S17_IODIR,_gpioDirection);
	}
}


void gpio_MCP23S17::gpioPort(uint16_t value){
	if (value == HIGH){
		_gpioState = 0xFF;
	} else if (value == LOW){	
		_gpioState = 0x00;
	} else {
		_gpioState = value;
	}
	DEBUG_MCP("gpioPort");
	DEBUG_MCP(MCP23S17_GPIO);
	DEBUG_MCP(_gpioState);
	//	_GPIOwriteWord(MCP23S17_GPIO,_gpioState);
	_GPIOwriteByte(MCP23S17_OLAT,_gpioState);
}

void gpio_MCP23S17::gpioPort(byte lowByte, byte highByte){
	_gpioState = highByte | (lowByte << 8);
	_GPIOwriteWord(MCP23S17_GPIO,_gpioState);
}


uint16_t gpio_MCP23S17::readGpioPort(){
	return gpioReadAddress(MCP23S17_GPIO);
}

uint16_t gpio_MCP23S17::readGpioPortFast(){
	return _gpioState;
}

void gpio_MCP23S17::portPullup(uint16_t data) {
	if (data == HIGH){
		_gpioState = 0xFF;
	} else if (data == LOW){	
		_gpioState = 0x00;
	} else {
		_gpioState = data;
	}
//	_GPIOwriteWord(MCP23S17_GPPU, _gpioState);
	_GPIOwriteByte(MCP23S17_GPPU, _gpioState);
}


void gpio_MCP23S17::gpioDigitalWrite(uint8_t value){
		_gpioState = value;
		_GPIOwriteByte(MCP23S17_OLAT,_gpioState);
}
void gpio_MCP23S17::gpioDigitalWrite(uint8_t pin, bool value){
	if (pin < 16){//0...15
		value == HIGH ? _gpioState |= (1 << pin) : _gpioState &= ~(1 << pin);
//		_GPIOwriteWord(MCP23S17_GPIO,_gpioState);
		_GPIOwriteByte(MCP23S17_OLAT,_gpioState);
	}
}

void gpio_MCP23S17::gpioDigitalWriteFast(uint8_t pin, bool value){
	if (pin < 16){//0...15
		value == HIGH ? _gpioState |= (1 << pin) : _gpioState &= ~(1 << pin);
	}
}

void gpio_MCP23S17::gpioPortUpdate(){
//	_GPIOwriteWord(MCP23S17_GPIO,_gpioState);
	_GPIOwriteByte(MCP23S17_GPIO,_gpioState);
}

int gpio_MCP23S17::gpioDigitalRead(uint8_t pin){
	if (pin < 16) return (int)(gpioReadAddress(MCP23S17_GPIO) & 1 << pin);
	return 0;
}


int gpio_MCP23S17::gpioDigitalReadFast(uint8_t pin){
	int temp = 0;
	if (pin < 16) temp = bitRead(_gpioState,pin);
	return temp;
}

uint8_t gpio_MCP23S17::gpioRegisterReadByte(byte reg){
  uint8_t data = 0;
    _GPIOstartSend(1);
    SPI.transfer(reg);
    data = SPI.transfer(0);
    _GPIOendSend();
  return data;
}

uint16_t gpio_MCP23S17::gpioRegisterReadWord(byte reg){
  uint16_t data = 0;
    _GPIOstartSend(1);
    SPI.transfer(reg);
	#if !defined(__SAM3X8E__) && ((ARDUINO >= 160) || (TEENSYDUINO > 121))
		data = SPI.transfer16(0);
	#else
		data = SPI.transfer(0);
		data = SPI.transfer(0) << 8;
	#endif
    _GPIOendSend();
  return data;
}

void gpio_MCP23S17::gpioRegisterWriteByte(byte reg,byte data,bool both){
	if (!both){
		_GPIOwriteByte(reg,(byte)data);
		DEBUG_MCP("!both::_GPIOwriteByte-1");
	} else {
		_GPIOstartSend(0);
		SPI.transfer(reg);
		SPI.transfer(data);
		SPI.transfer(data);
		_GPIOendSend();
		DEBUG_MCP("!both::_GPIOwriteByte-2");
	}
}



void gpio_MCP23S17::gpioRegisterWriteWord(byte reg,word data){
	_GPIOwriteWord(reg,(word)data);
}

template <typename Generic>
void gpio_MCP23S17::DEBUG_MCP(Generic text) {
  if (_debug) {
    Serial.print("*MCP: ");
    Serial.println(text);
  }
}


