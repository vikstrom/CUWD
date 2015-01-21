
#include "IRremote.h"
#include <utility/twi.h>
#include <Wire.h>

// RTC
const int MCP7940 = 0x6F; // Address of MCP7940 see data sheets
const int TIME_REG = 0;	// Sets the register to timekeeping register
const int CONTROL_REG = 0x7;	// Control register
const int ALARM0_REG = 0xA; 	// Alarm 0 register
//int MCP7940_REG = 0;	// Active register
 
// Initializes all values: 
byte second = 0;
byte minute = 0;
byte hour = 0;
byte weekday = 0;
byte date = 0;
byte month = 0;
byte year = 0;
byte controlRegister = 0;



//Defining pins
#define EN_pin_SMPS A2	//Enable pin for 5V reg. Keep low when sleeping
#define MFP_RTC A3		//Alarm pin from RTC-chip, active LOW, require PULL UP
#define SDA_pin A4		//I2C SDA pin, require PULL UP
#define SCL_pin A5		//I2C SCL pin, require PULL UP
#define RECV_PIN 0		//IR signal pin, input
//#define IR_PWR 3		//Power for IR module, this allowes the uC to easily switch this on and off 
						//during sleep to save some power.
#define Motor_PWM_pin 5 //PWM signal pin for motorcontroller
#define VCC_MEAS_PIN A1	//Pin to monitor level of RTC_VCC net. For enabling the SMPS if it goes to low.

IRrecv irrecv(RECV_PIN);	//Initiate IR Receiver
decode_results results;		//For storing received codes
uint16_t lastCode = 0;		//Storing last code received

#define BUTTON_POWER 0xD827	//Codes for buttons on IR-remote
#define BUTTON_A 0xF807
#define BUTTON_B 0x7887
#define BUTTON_C 0x58A7
#define BUTTON_UP 0xA05F
#define BUTTON_DOWN 0x00FF
#define BUTTON_LEFT 0x10EF
#define BUTTON_RIGHT 0x807F
#define BUTTON_CIRCLE 0x20DF

//// Menu items ////

// alarmSet menu items
uint16_t alarmIsSet = 0;
uint16_t alarmSetMenuActive = 0;	// Zero indicates the menu is not active
uint16_t alarmSetHoursActive = 1;
uint16_t alarmSetMinutesActive = 0;
uint16_t alarmSetDaysActive = 0;
uint16_t alarmDate = 0;
uint16_t alarmDay = 0;
uint16_t alarmMonth = 0;
uint16_t alarmHours = 0;
uint16_t alarmMinutes = 0;

const uint16_t MINUTE_CHANGE = 1;	//Number of minutes to change for each click
const uint16_t MAX_MINUTE = 60 - MINUTE_CHANGE;
const uint16_t MAX_HOUR = 23;

// Calibration
uint16_t calibrationMenuActive = 0;
uint16_t calibrationPullActive = 0;
unsigned long calibrationStartTime = 0;
unsigned long calibrationTime = 1000;	// Standard 1s

// Motor
uint16_t motorRunning = 0;
unsigned long motorStartTime = 0;
uint16_t motorAlarmTest = 0;


void resetMenu()
{
	alarmSetMenuActive = 0;
	calibrationMenuActive = 0;
	calibrationPullActive = 0;
	stopMotor();
}


void handleIr()
{
	uint16_t resultCode = (results.value & 0xFFFF);	//Get value and mask

	switch (resultCode) 
	{
	    case BUTTON_POWER:
	    	Serial.println("POWER button");
	    	alarmOnOff();	//Switches the alarm
	      break;

	    case BUTTON_A:
	     	Serial.println("A button");
	     	if(alarmSetMenuActive)	//If the user currently is setting the alarm
	     	{
	     		writeToRTC(ALARM0_REG);
	     		Serial.println("Write complete!");
	     		readFromRTC(ALARM0_REG);
	     		Serial.print("Current Alarm: ");
	     		printTime();
	     	}
	     	else if(!alarmSetMenuActive)	//Else the user wants to start setting the alarm
	     	{
		     	readFromRTC(TIME_REG);
		     	setAlarmMenu();
		    }
	      break;

	    case BUTTON_B:
	     	Serial.println("B button");
	     	calibration();	//Starts the calibration
	      break;

	    case BUTTON_C:
	     	Serial.println("C button");
	     	Alarm(1);	//Test button
	      break;

	    case BUTTON_UP:
	     	Serial.println("UP button");
	     	if(alarmSetMenuActive)	//If user is setting the alarm
	     	{
	     		if(alarmSetHoursActive && alarmHours < MAX_HOUR)
	     			alarmHours++;
	     			else if(alarmSetHoursActive && alarmHours == MAX_HOUR)
	     			{
	     				alarmHours = 0;
	     				alarmDate++;
	     			}

	     		if(alarmSetMinutesActive && alarmMinutes < (MAX_MINUTE))
	     			alarmMinutes = alarmMinutes + MINUTE_CHANGE;
	     			else if(alarmSetMinutesActive && alarmMinutes == MAX_MINUTE)	
	     				alarmMinutes = 0;
	     		
	     		setAlarmMenu();
	     	}
	      break;

	     case BUTTON_DOWN:
	     	Serial.println("DOWN button");
	     	if(alarmSetMenuActive)
	     	{
	     		if(alarmSetHoursActive && alarmHours > 0)
	     			alarmHours--;
	     			else if(alarmSetHoursActive && alarmHours == 0)
	     			{
	     				alarmHours = MAX_HOUR;
	     				alarmDate--;
	     			}

	     		if(alarmSetMinutesActive && alarmMinutes > 0)
	     			alarmMinutes = alarmMinutes - MINUTE_CHANGE;
	     			else if(alarmSetMinutesActive && alarmMinutes == 0)
	     				alarmMinutes = MAX_MINUTE;
	     		
	     		setAlarmMenu();
	     	}
	      break;

	    case BUTTON_LEFT:
	     	Serial.println("LEFT button");
		     if(alarmSetMenuActive)	//If user is setting alarm, change value to be altered
		     {	
		     	if(alarmSetMinutesActive)
		     	{
		     		alarmSetMinutesActive = 0;
		     		alarmSetHoursActive = 1;
		     	}
		     	else if(alarmSetHoursActive)
		     	{
		     		alarmSetHoursActive = 0;
		     		alarmSetDaysActive = 1;
		     	}
		    }
	      break;

	    case BUTTON_RIGHT:
	     	Serial.println("RIGHT button");
	     	if(alarmSetMenuActive)	//Same thing as above
		     {	
		     	if(alarmSetHoursActive)
		     	{
		     		alarmSetMinutesActive = 1;
		     		alarmSetHoursActive = 0;
		     	}
		    }
	      break;

	    case BUTTON_CIRCLE:
	     	Serial.println("CIRCLE button");
	     	if(calibrationMenuActive && !calibrationPullActive)
		    {
		    	startMotor();
		    	calibrationStartTime = millis();
		    	calibrationPullActive = 1;
		    	Serial.println("Calibration in progress...");
		    }
	     	else if(calibrationMenuActive && calibrationPullActive)
	     	{
	     		stopMotor();
	     		calibrationTime = millis() - calibrationStartTime;
	     		calibrationPullActive = 0;
	     		Serial.print("Calibration complete, time was:");
	     		Serial.println(calibrationTime);
	     		resetMenu();
	     	}
	      break;

	    case 0xFFFF:
	    	break;

	    default:
	      	Serial.println("Uknown code");
	      	break;
	}
	irrecv.resume();	//Receive next value
}

void startMotor()
{
	motorStartTime = millis();
/*	for(int i = 0; i < 13; i++)
	{
		analogWrite(Motor_PWM_pin, i);
		delay(10);
	}
/*	for(int i = 0; i < 100; i++)
	{
		analogWrite(Motor_PWM_pin, i);
		delay(2);
	}
*/	for(int i = 0; i < 200; i++)
	{
		analogWrite(Motor_PWM_pin, i);
		delay(1);
	}

	motorRunning = 1;
}

void stopMotor()
{
	analogWrite(Motor_PWM_pin, 0);
	motorRunning = 0;
}


void alarmOnOff()	// Sets alarm in RTC register
{
	resetMenu();
	readFromRTC(CONTROL_REG);
	byte controlRegisterTemp = controlRegister;
	Serial.println(controlRegister);
	if(controlRegister & B00010000)	// If bit 4 (ALM0) is set
	{	
		controlRegister = (controlRegister & B11101111);	// Mask all bits but bit 4 with "AND 1". Sets bit 4 = 0.
		writeToRTC(CONTROL_REG);
		Serial.println("Alarm turned off");
		Serial.println(controlRegister);
		alarmIsSet = 0;
	}
	else if ( ((controlRegister & B00010000) == 0))
	{
		controlRegister = (controlRegister | B00010000);	// Mask all bits but bit 4 with "OR 0". Sets bit 4 = 1.
		Serial.println(controlRegister);
		writeToRTC(CONTROL_REG);
		int checkMFP = 0;						//It's important to check if the MFP goes high when setting the register
		unsigned long timer = millis();			//And to stop it triggering the motor when we don't want to
		while(((millis() - timer) < 50) && (checkMFP == 0)) {
		    checkMFP = digitalRead(MFP_RTC);
			if(checkMFP) {
				controlRegister = controlRegisterTemp;
				writeToRTC(CONTROL_REG);
				Serial.println("Error, set new alarm-time before turning on alarm");
			}
		}
		if (checkMFP == 0) {
		Serial.println("Alarm turned on");
		Serial.println(controlRegister);
		readFromRTC(ALARM0_REG);
		printTime();
		alarmIsSet = 1;
		}
	}

}

void setAlarmMenu()
{
	if(!alarmSetMenuActive)
	{
		resetMenu();
		alarmMinutes = minute;
		alarmHours = hour;
		alarmDate = date;
		alarmDay = weekday;
		alarmMonth = month;
	}
	alarmSetMenuActive = 1;	// Set to active menu
	Serial.println("Setting alarm...: ");
	Serial.print("Alarm day: ");
	Serial.println(alarmDate);
	Serial.print("Alarm time: ");
	Serial.print(alarmHours);
	Serial.print(":");
	if(alarmMinutes <= 9)
	{
		Serial.print("0");
		Serial.println(alarmMinutes);
	}
	else
		Serial.println(alarmMinutes);

	Serial.println();
}

void calibration()
{
	resetMenu();
	calibrationMenuActive = 1;
	Serial.println("Let's calibrate!");
	Serial.println("Start and stop with CENTER");
}

void testAlarm()
{
	motorAlarmTest = 1;
	startMotor();
}

void readFromRTC(int MCP7940_REG) 
{
	
	switch (MCP7940_REG) {

	    case TIME_REG:
			Wire.beginTransmission(MCP7940);
			Wire.write(MCP7940_REG);
			Wire.endTransmission();
			Wire.requestFrom(MCP7940, 7);
			second = bcdToDec(Wire.read());
			minute = bcdToDec(Wire.read());
			hour = bcdToDec(Wire.read());
			weekday = bcdToDec(Wire.read());
			date = bcdToDec(Wire.read());
			month = bcdToDec(Wire.read());
			year = bcdToDec(Wire.read());
     	break;

	    case CONTROL_REG:
			Wire.beginTransmission(MCP7940);
			Wire.write(MCP7940_REG);
			Wire.endTransmission();
			Wire.requestFrom(MCP7940, 1);
			controlRegister = Wire.read();
    	break;

	    case ALARM0_REG:
			Wire.beginTransmission(MCP7940);
			Wire.write(MCP7940_REG);
			Wire.endTransmission();	
			Wire.requestFrom(MCP7940, 6);
			second = bcdToDec(Wire.read());
			minute = bcdToDec(Wire.read());
			hour = bcdToDec(Wire.read());
			weekday = bcdToDec(Wire.read());
			date = bcdToDec(Wire.read());
			month = bcdToDec(Wire.read());  
		break; 	

	   	default:
		break;
	}
}

void writeToRTC(int MCP7940_REG)
{
	switch (MCP7940_REG) {

	    case CONTROL_REG:
			Wire.beginTransmission(MCP7940);
			Wire.write(MCP7940_REG);
			Wire.write(controlRegister);
			Wire.endTransmission();
		break;

	    case ALARM0_REG:
			Wire.beginTransmission(MCP7940);
			Wire.write(MCP7940_REG);
			Wire.write(decToBcd(0));	//Always write 0 to seconds when setting the alarm
			Wire.write(decToBcd(alarmMinutes));
			Wire.write(decToBcd(alarmHours));
			Wire.write(decToBcd(alarmDay) | B11110000);	//Also sets alarm comparator bits
			Wire.write(decToBcd(alarmDate));
			Wire.write(decToBcd(alarmMonth));
			Wire.endTransmission();
	    break;

	    default:
	    break;
	      // do something
	}
}

void printTime() 
{	
	Serial.print(year);
	Serial.print("-");
	Serial.print(month);
	Serial.print("-");
	Serial.print(date);
	Serial.print("	");
	Serial.print(hour);
	Serial.print(":");
	if(minute <= 9)
	{
		Serial.print("0");
		Serial.println(minute);
	}
	else
		Serial.println(minute);

}

byte decToBcd(byte val)	
{
 	return ((val/10*16) + (val%10));
}
byte bcdToDec(byte val) 
{
	return ((val/16*10) + (val%16));
}

void resetControlRegister()
{
	readFromRTC(CONTROL_REG);
	if(controlRegister != B00000000)	//Resetting control register so that the alarm output is not triggering the motor
	{
		controlRegister = B00000000;
		writeToRTC(CONTROL_REG);
		Serial.println("controlRegister byte is set to 0");
	}

}

void Alarm(int alarmTrig)
{	
	if(alarmTrig) {
		Serial.println("ALARM!");
		resetControlRegister();
		startMotor();
	}
}

void motorControl(int motorStatus)
{
	if(((millis() - motorStartTime) >= calibrationTime) && !calibrationPullActive)
	{	
		stopMotor();
	}

}


void setup() 
{
	Serial.begin(9600);
	Wire.begin();
	irrecv.enableIRIn();	//Start receiver
	pinMode(EN_pin_SMPS, OUTPUT);
	pinMode(MFP_RTC, INPUT);
	pinMode(Motor_PWM_pin, OUTPUT);
	pinMode(RECV_PIN, INPUT_PULLUP);
	pinMode(VCC_MEAS_PIN, INPUT);
	//digitalWrite(EN_pin_SMPS, HIGH);	//Enable SMPS 5V
	resetControlRegister();
	readFromRTC(TIME_REG);
	printTime();
  }

void loop() 	//Polls the ir-input, runs the motorControl and checks if the alarm pin is high.
{
	
	if(irrecv.decode(&results))	//If there's something on the input
		handleIr();

	motorControl(motorRunning);	

	Alarm(digitalRead(MFP_RTC));

}

