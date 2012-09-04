/* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * AppSensor.h - AppSensor library
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


#define BUFFER_LEN 32

#include "HardwareSerial.h"
#include "Arduino.h"


class AppSensor {
public:
    //pass a reference to a HardwareSerial object
    AppSensor(HardwareSerial &serialPort);
	
	void					test();
	
	void					setup();
	void					update();
	
private:
    HardwareSerial			*serial;
	bool					blinking;
	
	bool					connected;
	char					incomingByte;
	char					receivedByte;
	
	unsigned int			numberOfBytes;
	
	
	unsigned char			txBuffer[BUFFER_LEN];
    char					rxBuffer[BUFFER_LEN];
	
	unsigned int			NUM_OF_SENDING_BYTES;
	
	char			        DIGITAL_CONFIG_BUF[14];
	char			        ANALOG_CONFIG_BUF[6];
	
	unsigned int		    ANALOG_INPUT[6];
	unsigned int			DIGITAL_PWM[14];
	
	//private methods
	void					listening();
	void					reporting();
	unsigned char			analogFormatter_PIN(unsigned int pinNumber, unsigned int analogValue);
	unsigned char           analogFormatter_VALUE(unsigned int analogValue);
	
};

