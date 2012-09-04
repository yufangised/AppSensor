/* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * AppSensor.cpp - AppSensor library
 * Copyright (c) 2012 Copenhagen Institute of Interaction Design. 
 * All right reserved.
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser Public License for more details.
 *
 * You should have received a copy of the GNU Lesser Public License
 * along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * + author: yufan + dviid
 * + contact: w.yufan@edu.ciid.dk + dviid@labs.ciid.dk
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 */

#include "AppSensor.h"

AppSensor::AppSensor(HardwareSerial &serialPort) {
	
	serial = &serialPort;
	//serial->begin(9600);
	connected = false;
	blinking = false;
	numberOfBytes = 0;
	NUM_OF_SENDING_BYTES = 0;
	
	for (int i = 0; i< sizeof(DIGITAL_CONFIG_BUF); i++) {
        DIGITAL_CONFIG_BUF[i] = '0';
        DIGITAL_PWM[i] = 0;
    }
    
    for (int j = 0; j< sizeof(ANALOG_CONFIG_BUF); j++) {
        ANALOG_CONFIG_BUF[j] = '0';
		ANALOG_INPUT[j] = 0;
    }

}

void AppSensor::setup(){
		
	if (connected == false) {
	
		delay(100);
	
wait:
	
		if(serial->available() <= 0){
			delay(100);
			goto wait;
		}else{
			incomingByte = serial->read();
			if(incomingByte == '['){
			
				serial->readBytes(DIGITAL_CONFIG_BUF, 14);
				serial->readBytes(ANALOG_CONFIG_BUF, 6);
			
				incomingByte = serial->read();
				if(incomingByte != ']'){
					while(serial->read() >= 0);
					serial->write('<');
					delay(100);
					goto wait;
				} 
			}
		}
	
		for(int i = 2; i < 14; i++){
			if(DIGITAL_CONFIG_BUF[i] == 'd'){
				pinMode(i, INPUT);
			} 
			
			if (DIGITAL_CONFIG_BUF[i] == 'D' || DIGITAL_CONFIG_BUF[i] == 'A') {
				pinMode(i, OUTPUT);
			}	
		}
		
			
		serial->write('>');
		connected = true;
	
	}
}

void AppSensor::update(){
	
	if (connected) {
		reporting();
		listening();
	}
	
}

void AppSensor::listening(){
	
	if(serial->available() >0){
		incomingByte = serial->read();
		if (incomingByte == '\0') {
			numberOfBytes = serial->read();
			memset(rxBuffer, 0, sizeof(rxBuffer));//clear the rxBuffer
			serial->readBytes(rxBuffer, numberOfBytes);
		} else{
			while(serial->read()>=0);
		}
	}
		
	
	for (int i = 2; i<sizeof(DIGITAL_CONFIG_BUF); i++) {
		if (DIGITAL_CONFIG_BUF[i] == 'D') {
			if (i<8) {
				bitRead(rxBuffer[0],(i-2)) ? digitalWrite(i, HIGH) : digitalWrite(i, LOW);
				bitRead(rxBuffer[0],(i-2)) ? DIGITAL_PWM[i] = 1 : DIGITAL_PWM[i] = 0;
			}else{
				bitRead(rxBuffer[1],(i-8)) ? digitalWrite(i, HIGH) : digitalWrite(i, LOW);
				bitRead(rxBuffer[1],(i-8)) ? DIGITAL_PWM[i] = 1 : DIGITAL_PWM[i] = 0;
			}
		}
	}
					
				
	if(numberOfBytes > 2){
		for (int j = 2; j < numberOfBytes; j++) {
					
			receivedByte = rxBuffer[j];
					
			bool byteFlag = bitRead(receivedByte, 7);
			
			unsigned int pinIndex = (receivedByte & 127) >> 2;
			
			unsigned int lowerValue = receivedByte & 3;
			unsigned int higherValue = 0;
			
			if (DIGITAL_CONFIG_BUF[pinIndex] == 'A') {
						
				if (byteFlag == true) {
					
					j++;
					receivedByte = rxBuffer[j];
					higherValue = receivedByte << 2;
				}
					
				DIGITAL_PWM[pinIndex] = lowerValue + higherValue;
				analogWrite(pinIndex, DIGITAL_PWM[pinIndex]);
			}
		}
	}
	
	/*
	for (int k = 0; k < sizeof(DIGITAL_CONFIG_BUF); k ++) {
		if (DIGITAL_CONFIG_BUF[k] =='A') {
			analogWrite(k, DIGITAL_PWM[k]);
		}
		
		if (DIGITAL_CONFIG_BUF[k] == 'D') {
			DIGITAL_PWM[k] ? digitalWrite(k, HIGH) : digitalWrite(k, LOW); 
		}
	}
	
	*/
}
							
void AppSensor::reporting(){
							
	NUM_OF_SENDING_BYTES = 4;
	memset(txBuffer, 0, sizeof(txBuffer));//clear the txBuffer

	unsigned char digitalPinsLower = 192; //1100 0000
	unsigned char digitalPinshigher = 192; //1100 0000
	
	for (int i = 2; i < sizeof(DIGITAL_CONFIG_BUF); i++) {
		
		if(DIGITAL_CONFIG_BUF[i] == 'd'){//digital input 
			DIGITAL_PWM[i] = digitalRead(i);
			if (i<8) {
				DIGITAL_PWM[i] ? bitSet(digitalPinsLower, i-2):bitClear(digitalPinsLower, i-2);
			}else {
				DIGITAL_PWM[i] ? bitSet(digitalPinshigher, i-8):bitClear(digitalPinshigher, i-8);
			}
		}
	}
	
	for (int j = 0; j < sizeof(ANALOG_CONFIG_BUF); j++) {
		if (ANALOG_CONFIG_BUF[j] == 'a') {
				
			ANALOG_INPUT[j]=analogRead(j);

			if (ANALOG_INPUT[j] <= 3) {
				txBuffer[NUM_OF_SENDING_BYTES++] = analogFormatter_PIN(j, ANALOG_INPUT[j]);
			}else {
				txBuffer[NUM_OF_SENDING_BYTES++] = analogFormatter_PIN(j, ANALOG_INPUT[j]);
				txBuffer[NUM_OF_SENDING_BYTES++] = analogFormatter_VALUE(ANALOG_INPUT[j]);
			}
		}
	}
            
            
	if (NUM_OF_SENDING_BYTES >= 4) {
		txBuffer[0] = '\0';
		txBuffer[1] = NUM_OF_SENDING_BYTES-2; //do not count the first 2 byte '\0' and numberOfBytes;
		txBuffer[2] = digitalPinsLower;
		txBuffer[3] = digitalPinshigher;
		
		for (int k = 0; k < NUM_OF_SENDING_BYTES; k++) {
			serial->write(txBuffer[k]);
		}
	}
}

									
unsigned char AppSensor::analogFormatter_PIN(unsigned int pinNumber,  unsigned int analogValue){

	unsigned char result;
	unsigned char pin = (pinNumber + 14) << 2;
	unsigned char value = analogValue & 3;
	
	if (analogValue > 3) bitSet(pin, 7);
	
	result = pin | value;
	
	return result;
}

									
unsigned char AppSensor::analogFormatter_VALUE(unsigned int analogValue){
										
	unsigned char result;
	result = (analogValue >> 2);

	return result;
}
									
void AppSensor::test() {
	//

}
