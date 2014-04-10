/*
  RGBTest
  This is logic code for the v1. Shape Clip board.
	John Hardy
	Christian Weichel
	Matthias Schitthelm
  
  Features include:
	  - LDR calibration
 */

// Extending the touch screen 	--- http://portfolio.spike5000.com/?p=67
// Touch Sensing Help 			--- http://electronics.stackexchange.com/questions/63950/arduino-trigger-an-iphones-touch-screen-with-no-human-interaction
// Human Body Capacitance 		--- The Human body is modeled by a 100 pF capacitor and a 1500 ohm discharging resistance. http://en.wikipedia.org/wiki/Human-body_model
// LED Blinking library			--- https://code.google.com/p/arduino-library-syncled/
// Threading 					--- http://forum.arduino.cc/index.php/topic,5686.0.html
 
 
/* Which version of the device are we compiling for. */
//#define DEVICE_MKI
#define DEVICE_MKIII

#define MODE_PROFILING 		// Is the clip profiling itself via serial.


/* External and 3rd party libraries. */ 
#include <Math.h>
#include <EEPROM.h>
#include "CoolStepper.h" 
#include "LDR.h"
#include "Adafruit_NeoPixel.h"
#include "RunningVariance.h"
#include "WindowVariance.h"
#include "Kalman.h"

/* IO Pins */
#define PIN_LED   A5		// The on-board LED.
#define PIN_SWTOP 2			// The top switch (digital).
#define PIN_SWBOT 3			// The bottom switch (digital).
#define PIN_FORCE A0		// The force input (analog).
#define PIN_TOUCH 8  		// The touch output (digital).
#define PIN_RGB_OUT 9		// The RGB LED (digital).
#define PIN_MIN11 4			// Motor Coil 1 + (digital).
#define PIN_MIN12 5			// Motor Coil 1 - (digital).
#define PIN_MIN21 6			// Motor Coil 2 + (digital).
#define PIN_MIN22 7			// Motor Coil 2 - (digital).

#ifdef DEVICE_MKI			// ** Specific to mkII and mkII.
#define PIN_LDR1  A1		// The LDR1 input (analog).
#define PIN_LDR2  A2		// The LDR2 input (analog).
#endif
#ifdef DEVICE_MKIII			// ** Specific to mkIII.
#define PIN_RND   A4
#define PIN_LDR1  A6		// The LDR1 input (analog).
#define PIN_LDR2  A7		// The LDR2 input (analog).
#endif

/* Motor characteristics */
#define MOTOR_ANGLE 20		// The degrees turned with each motor step.
#define MOTOR_SPEED 1500	// The motor speed in rotations per minute (RPMs).
#define MOTOR_TRAVEL 470	// The number of steps to move the nut from the top to the bottom.

/* LDR Settings and Sync Pulse Constants */
const float SYNC_SIGNAL_DROP_THRESHOLD = 3.0f; 				// Based on the behaviour of the LDR with a 10k resistor on a Dell LCD monitor.
const int PULSE_COUNT = 5;									// There are 5 pulses: high, low, r, g, b
const int PULSE_WIDTH = 200;								// The amount of time each pulse last.
const int PULSE_WIDTH_ERR = 10;								// The amount of error in each pulse width start and end (~10ms due to 4ms timers in JS).
const int DATA_WIDTH  = PULSE_WIDTH * PULSE_COUNT;			// The total amount of time for an entire data pulse.
const int DATA_WIDTH_ERR = PULSE_WIDTH_ERR * PULSE_COUNT; 	// The total amount of error for an entire data pulse.

const int SAMPLE_OFFSET = 50;								// The amount of time (ms) to offset the LDR sampling start (relative to the sync drop).
const int SAMPLE_WINDOW = 90;								// The amount of time (ms) to sample the LDR for.

/* Application Behaviours */

//#define SERIAL_INFO			// Should device values be streamed via serial? csv: ms, ldr1, ldr2, rgb, height, frc, haspulse, swbot
//#define SERIAL_SYNC_DEBUG		// Should sync pulse debug data be streamed via serial? csv: ms, r, g, b
//#define SERIAL_SYNC_DEBUG2	// Should the verbose sync pulse debug data be streamed via serial? csv: ms, slope, max, r, g, b, min, raw

#define RGBMODE_NONE     0		// The RGB LED displays nothing.
#define RGBMODE_SCREEN   1		// The RGB LED displays the last colour the pulses detected.
#define RGBMODE_NOSCREEN 2		// The RGB LED displays the "not on screen" pattern (i.e. no pulse detected).
#define RGBMODE_STARTUP  3		// The RGB LED displays the "setting up / calibrating" pattern.

#define EEMODEADDR 0			// The address of the mode stored in EEPROM.
#define EEMODE_CHANGETIME 5000  // The number of ms to hold the switch before the clip mode is changed.
#define EEMODE_SYNCPULSE  45	// The Clip looks for a sync pulse on one of the LDRs.
#define EEMODE_HEIGHTONLY 37	// The Clip uses both LDRs to estimate height, no colour involved.
#define EEMODE_SERIAL     89	// The Clip can send / recieve serial data with a simple protocol.

#define PULSE_STATE__NOT_CHECKED  -1	// The sync-pulse routine did not bother checking, too soon since previous check.
#define PULSE_STATE__NO_PULSE      0	// The sync-pulse check returned no pulse.. Probably off a screen.
#define PULSE_STATE__ACTIVE_PULSE  1	// The sync-pulse check returned a pulse! It's probably on a screen.

/* Colour manipulation macros. */
#define RGB(r,g,b) 	(((r & 0xFF) << 16) | ((g & 0xFF) << 8)  | ((b & 0xFF) << 0))
#define RED(hex) 	hex >> 16 & 0xFF
#define GREEN(hex) 	hex >>  8 & 0xFF
#define BLUE(hex) 	hex >>  0 & 0xFF


/**
 * @brief Check to see if code should be called based on the timer.
 * This can be used to create a rudimentary timer.  It is called once per logic-loop.
 * If it returns true, then the timer handler code should be run.  If false, then it should not.
 * Based on: http://forum.arduino.cc/index.php/topic,5686.0.html
 * Usage: ulong timer = 0; if (cycleCheck(&timer, 10u) { log("called every 10 ms"); }
 * @param *iLastMs Pointer to a value which contains the last time this was checked.
 * @param iCycle The number of ms since the last time required to trigger.
 * @return True if the time since the last call has elasped and the code should run. False if not.
 */
boolean cycleCheck(unsigned long *iLastMs, unsigned int iCycle)  {
	unsigned long ms = millis();
	if(ms - *iLastMs >= iCycle)
	{
		*iLastMs = ms;
		return true;
	}
	else
	{
		return false;
	}
}

/**
 * @brief Reset a timer that is used with the cycleCheck function.
 * @param *iLastMs Pointer to a value which contains the last time this was checked.
 * This will update the iLastMs value to the current time.
 */
void cycleReset(unsigned long *iLastMs) {
	*iLastMs = millis();
}

/**
 * @brief Work out if a time value falls within a sample window. 
 * @param iRelativeTime The time to check.
 * @param iSampleOffset The sampling offset start (relative to 0 in iRelativeTime).
 * @param iWindowSize The sampling window size.
 * @return True if (iSampleOffset+iWindowSize) > iRelativeTime > iSampleOffset.
 */
boolean checkSample(int iRelativeTime, int iSampleOffset, int iWindowSize) {
	return (iRelativeTime > iSampleOffset) && ( iRelativeTime < (iSampleOffset + iWindowSize));
}


// ShapeClip v1 controls and live values.
Stepper motor(MOTOR_ANGLE, PIN_MIN11, PIN_MIN12, PIN_MIN21, PIN_MIN22);
Adafruit_NeoPixel pRGB = Adafruit_NeoPixel(1, PIN_RGB_OUT, NEO_GRB + NEO_KHZ800);

boolean bMotorCalibrated = false;	// Is the motor position value accurate?
int iMotorPosition = 0;				// The current number of steps in the motor.

int eClipMode = EEMODE_SYNCPULSE;
int eRGBMode = RGBMODE_NONE;		// Current RGB mode.
bool bOnScreen = false;				// Is the shape clip device on the screen (i.e. does it have a valid sync pulse)?
bool bLastOnScreen = false;			// As above, 1 state change behind.

int iTargetR = 0;					// Target RED component.
int iTargetG = 0;					// Target GREEN component.
int iTargetB = 0;					// Target BLUE component.
int iTargetPos = 0;					// Target MOTOR height (steps).

int iLDR1Min = 0;					// Current estimated LDR minimum.
int iLDR1Max = 0;					// Current estimated LDR maximum.

//int iLDR2Min = 0;					// Current estimated LDR minimum.
//int iLDR2Max = 0;					// Current estimated LDR maximum.

LDR ldr1(PIN_LDR1);
LDR ldr2(PIN_LDR2);


/**
 * @brief Configure the chip pins to the proper states.
 */
void configure() {
	
	// On-board LED (output, off).
	pinMode(PIN_LED, OUTPUT);
	digitalWrite(PIN_LED, LOW);
	
	// LDR1 and LDR2 (input). NOT REQUIRED FOR ANALOG PINS - MAYBE HARMFUL
	//pinMode(PIN_LDR1, INPUT);
	//pinMode(PIN_LDR2, INPUT);
	
	// Top and bottom switches (input, active).
	pinMode(PIN_SWTOP, INPUT);
	digitalWrite(PIN_SWTOP, HIGH);
	pinMode(PIN_SWBOT, INPUT);
	digitalWrite(PIN_SWBOT, HIGH);
	
	// Touch sensor (output, low).
	pinMode(PIN_TOUCH, OUTPUT);
	digitalWrite(PIN_TOUCH, LOW);
	
	// Force sensor (input).
	pinMode(PIN_FORCE, INPUT);
	
	// Set up the motor.
	motor.setSpeed(MOTOR_SPEED);
	
	// Set up the RGB LED.
	pRGB.begin();
	pRGB.show();
	
	// On-board LED (on).
	digitalWrite(PIN_LED, HIGH);
}
 
/**
 * @brief Print out information from the various sensors and estimated values.
 */
void debugSerial() {
	
	// Write out values.
	Serial.print(millis()); Serial.print(","); 									// ms
	Serial.print(analogRead(PIN_LDR1)); Serial.print(","); 						// ldr1
	Serial.print(ldr2.sample()); Serial.print(","); 							// ldr2
	Serial.print(RGB(iTargetR, iTargetG, iTargetB), HEX); Serial.print(" "); 	// RGB
	Serial.print(pRGB.getPixelColor(0), HEX); Serial.print(","); 				// RGB

	Serial.print(iMotorPosition); Serial.print(","); 							// current height
	//Serial.print(analogRead(PIN_FORCE)); Serial.print(","); 					// force
	Serial.print(iTargetPos); Serial.print(","); 								// target height
	Serial.print(bOnScreen ? "Y" : "N"); Serial.print(","); 					// has sync pulse
	Serial.print(digitalRead(PIN_SWBOT) == 0 ? "Y" : "N"); Serial.print("\n");	// swbot
}

/**
 * @brief This will send the motor DOWN until the SWBOT is triggered OR the full MOTOR_TRAVEL is used.
 * It will set the iMotorPosition value to 0 and not allow calls to moveMotor while operating.
 */
void zeroMotor() {
	
	// Zero flags.
	bMotorCalibrated = false;
	
	// Put us into setup mode (RGB LED).
	int eLastMode = eRGBMode;
	eRGBMode = RGBMODE_STARTUP;
	updateColour();
	
	// Until the motor is calibrated, step down each time.
	for(int iAvailableSteps = MOTOR_TRAVEL; iAvailableSteps >= 0; iAvailableSteps--) {
		motor.step(-1);

		//#ifdef SWBOT_MOTOR
		//if (digitalRead(PIN_SWBOT) == LOW) break;
		//#endif
	}
	
	// Set the motor position to 0 (known location).
	bMotorCalibrated = true;
	iMotorPosition = 0;
	
	// Shut the motor down.
	motor.shutdown();
	
	// Take us out of setup mode.
	eRGBMode = eLastMode;
	updateColour();
}

/**
 * @brief Read new height data from the screen.
 * It will update the iTargetPos value with the height between 0 and MOTOR_TRAVEL.
 */
void sampleHeight() {
	
	// Sample a value from the LDR.
	uint16_t analogValue = ldr2.sample();
	
	// Map it to the motor travel.
	iTargetPos = ldr2.mapMinMax(analogValue, 0, MOTOR_TRAVEL);
}

/**
 * @brief Move the motor one step towards the target position.
 */
void moveMotor() {
	
	// Only do this if the motor is calibrated.
	if (!bMotorCalibrated)
		return;
	
	// Compute the distance to travel.
	int iDelta = iTargetPos - iMotorPosition;
	
	// If we have to move over 1 step in either direction.
	if (abs(iDelta) > 1)
	{
		// Move one step in the correct direction and update the motor position.
		int step = iDelta >= 0 ? 1 : -1;
		motor.step(step);
		iMotorPosition += step;
	}
	
	// Otherwise shut the motor down.
	else
	{
		motor.shutdown();
	}
}


/**
 * @brief Display the correct colour on the RGB led based on the mode it is in.
 */
void updateColour() {
	
	// If we are in the normal mode.
	switch (eRGBMode)
	{
		case RGBMODE_NONE:
			pRGB.setPixelColor(0,  0, 0, 0);
			break;
		case RGBMODE_SCREEN:
			{
				// Go immediately to the new colour.
				// pRGB.setPixelColor(0,  iTargetR, iTargetG, iTargetB);
				
				// Get 1% closer to the target colour each iteration.
				uint32_t currentColor = pRGB.getPixelColor(0);
				int stepRed   = constrain(map(10, 0, 1000, RED(currentColor),   iTargetR), 0, iTargetR);
				int stepGreen = constrain(map(10, 0, 1000, GREEN(currentColor), iTargetG), 0, iTargetG);
				int stepBlue  = constrain(map(10, 0, 1000, BLUE(currentColor),  iTargetB), 0, iTargetB);
				
				// Set the step value.
				pRGB.setPixelColor(0,  stepRed, stepGreen, stepBlue);
			}
			break;
		case RGBMODE_NOSCREEN:
			pRGB.setPixelColor(0,  255, 0, 0);
			break;
		case RGBMODE_STARTUP:
			if (eClipMode == EEMODE_HEIGHTONLY)
				pRGB.setPixelColor(0,  0, 0, 255);
			else
				pRGB.setPixelColor(0,  0, 255, 0);
			break;
	}
	
	// Push the new colour.
	pRGB.show();
}




/**
 * Setup the board and configure the pins.
 */
void setup() { 
	
	// Configure the board and pins.
	configure();
	
	// Start the serial port.
	Serial.begin(115200);
	
	// Wait two seconds.
	Serial.println("Boot v2.1");
	
	// Select a default clip mode from the set of possible modes, if one is not set.
	eClipMode = EEPROM.read(EEMODEADDR);
	switch( eClipMode )
	{
	  case EEMODE_SYNCPULSE:
		break;
	  case EEMODE_HEIGHTONLY:
		break;
	  case EEMODE_SERIAL:
		break;
	  default:
		eClipMode = EEMODE_HEIGHTONLY;
		EEPROM.write(EEMODEADDR, EEMODE_HEIGHTONLY);
	}

	// Debug Mode
	eClipMode = EEMODE_SERIAL;
	detectModeChange();

	// Wait for a random time so that not all clips zero their motor at the same time.
	randomSeed(analogRead(PIN_RND));
	uint16_t randomWait = random(1000);
	delay(randomWait);
	
	// Bring the motor down until it hits the bottom.
	zeroMotor();
	
	// make sure we all start at the same time
	delay(1000 - randomWait);
}

/**
 * @brief Detect a mode change by bridging SWBOT.
 * Holding down SWBOT for 5s will change the clip mode from sync pulse (green startup)
 * to height only (blue startup).
 */
void detectModeChange() {
	
	// Throttle this to 10hz.
	static unsigned long tmrModeCheck = 0;
	if (!cycleCheck(&tmrModeCheck, 10U))
		return;
	
        if( digitalRead(PIN_SWBOT) == HIGH )
          return;

        while( digitalRead(PIN_SWBOT) == LOW )
        {
          switch( eClipMode )
          {
            case EEMODE_HEIGHTONLY:
              pRGB.setPixelColor(0,  0, 255, 0);
            
              eRGBMode  = RGBMODE_SCREEN;
              eClipMode = EEMODE_SYNCPULSE;
              break;
              
            case EEMODE_SYNCPULSE:
              pRGB.setPixelColor(0,  0, 0, 255);
              
              eRGBMode  = RGBMODE_NONE;
              eClipMode = EEMODE_SERIAL;
              break;
            
            case EEMODE_SERIAL:
              pRGB.setPixelColor(0,  255, 0, 0);
              
              eRGBMode  = RGBMODE_NONE;
              eClipMode = EEMODE_HEIGHTONLY;
              break;
            
            default:
              pRGB.setPixelColor(0,  255, 0, 0);
              eRGBMode  = RGBMODE_NONE;
              eClipMode = EEMODE_HEIGHTONLY;
          }
          
          pRGB.show();
          delay( 1000 );
        }
        
        for( int i=0; i<3; i++ )
        {
          switch( eClipMode )
          {
            case EEMODE_HEIGHTONLY:
              pRGB.setPixelColor(0,  255, 0, 0);
              break;
            case EEMODE_SYNCPULSE:
              pRGB.setPixelColor(0,  0, 255, 0);
              break;
            case EEMODE_SERIAL:
              pRGB.setPixelColor(0,  0, 0, 255);
              break;
          }
          pRGB.show();
          delay( 100 );
          pRGB.setPixelColor(0,  0, 0, 0);
          pRGB.show();
          delay( 100 );
        }
		
		// If we are in profiling mode, clear all target values on mode switch.
		#ifdef MODE_PROFILING
		iTargetR = 0;
		iTargetG = 0;
		iTargetB = 0;
		iTargetPos = 0;
		#endif
		
		// Store to EEPROM.
        EEPROM.write( EEMODEADDR, eClipMode );
}

/**
 * @brief Check the sync pulse and update relevant global variables.
 * This is automatically rate limited to 5ms samples.
 * It will update the iTargetR,G,B, iLDR1Max and iLDR1Min values if valid pulses are detected.
 * @return -1 if check not performed, 1 if we received an active pulse, 0 if we did not.
 */
int8_t checkPulse() {

	// Only allow this function to be called once every 5ms.
	static unsigned long tmr1 = 0;
	if (!cycleCheck(&tmr1, 5U))
		return PULSE_STATE__NOT_CHECKED;

	// Define static variables.
	static WindowVariance* pEstimatedMin = new WindowVariance(5);	// Running 
	static WindowVariance* pEstimatedMax = new WindowVariance(5);
	static int r=0; static int g=0; static int b=0;
	static float x1,x2,y1,y2 = 0;	// Required for slope differentials.
	static float fSlope = 0.0f;		// Required for slope differentials.
	static long iLastMinima = 0;	// The time of the last minimal (ms).

	// Read the LDR and reset the serial debug colour values.
	int iLDRValue = analogRead(PIN_LDR1);
	r=0; g=0; b=0;

	// Compute the slope (dLDR/dTime) by comparing it to the previous data.
	long iCurrentTime = millis();
	x1 = x2; y1 = y2;
	x2 = (float)iCurrentTime; y2 = (float)iLDRValue;
	fSlope = (y1-y2) / (x1-x2);

	// If we are hitting a big dip AND it has been over 50ms since the last one.
	int iDeltaTime = iCurrentTime - iLastMinima;
	boolean bSlopeDetected = fSlope < -SYNC_SIGNAL_DROP_THRESHOLD;
	boolean bWithinPulseRange = iDeltaTime > (DATA_WIDTH - DATA_WIDTH_ERR); // &&  delta < (DATA_WIDTH + DATA_WIDTH_ERR + PULSE_WIDTH + PULSE_WIDTH_ERR);

	// TODO: Reject pulses after a recent slope detection.
	if (bSlopeDetected && bWithinPulseRange)
	{
		// Log the start of the second pulse of the data-frame.
		iLastMinima = iCurrentTime;

		// If another minma happened BEFORE 500ms  (i.e. the amount of time between a high and a red/green)
		//  then we know that it is likely attached to the wrong pulse.
	}

	// Determine if this device has a sync pulse.
	boolean bActive = iDeltaTime < ( DATA_WIDTH + DATA_WIDTH_ERR ) ;

	// If an active sync pulse, sample the data.
	if (bActive)
	{
		// Make it relative to the start of the minima spike.
		int iRelativeDelta = iCurrentTime - iLastMinima;

		// Sample the min and max.  Min = immediately after the spike, Max = 4th pulse after the spike.
		if (checkSample(iRelativeDelta, (PULSE_WIDTH * 0) + SAMPLE_OFFSET, SAMPLE_WINDOW)) {		// Min.
			pEstimatedMin->push(iLDRValue);
			iLDR1Min = pEstimatedMin->mean();
			ldr2.updateMin(iLDR1Min);
		}
		if (checkSample(iRelativeDelta, (PULSE_WIDTH * 4) + SAMPLE_OFFSET, SAMPLE_WINDOW)) {		// Max.
			pEstimatedMax->push(iLDRValue);
			iLDR1Max = pEstimatedMax->mean();
			ldr2.updateMax(iLDR1Max);
		}

		// Sample the RGB pulses. R = 1st pulse after the spike, B = 3rd pulse after the spike.
		if (checkSample(iRelativeDelta, (PULSE_WIDTH * 1) + SAMPLE_OFFSET, SAMPLE_WINDOW)) {		// Red.
			r = iLDRValue;
			iTargetR = constrain(map(iLDRValue, iLDR1Min, iLDR1Max, 0, 255), 0, 255);
		}
		if (checkSample(iRelativeDelta, (PULSE_WIDTH * 2) + SAMPLE_OFFSET, SAMPLE_WINDOW)) {		// Green.
			g = iLDRValue;
			iTargetG = constrain(map(iLDRValue, iLDR1Min, iLDR1Max, 0, 255), 0, 255);
		}
		if (checkSample(iRelativeDelta, (PULSE_WIDTH * 3) + SAMPLE_OFFSET, SAMPLE_WINDOW)) {		// Blue.
			b = iLDRValue;
			iTargetB = constrain(map(iLDRValue, iLDR1Min, iLDR1Max, 0, 255), 0, 255);
		}

		// Ensure we are in a mode that displays a screen colour.
		//changeMode(SCREEN);
	}
	else
	{
		// Turn the LED blue (not on screen).
		//changeMode(NO_SCREEN);
	}

	// Debug graph. Used with SerialChart. 
	#ifdef SERIAL_SYNC_DEBUG
	Serial.print(millis()); Serial.print(","); 		// ms
	Serial.print(iTargetR); Serial.print(","); 		// Red
	Serial.print(iTargetG); Serial.print(","); 		// Green
	Serial.print(iTargetB); Serial.print("\n"); 	// Blue
	#endif
	
	#ifdef SERIAL_SYNC_DEBUG2
	// Print out the verbose serial debug as csv.
	Serial.print(millis()); Serial.print(","); 		// ms
	Serial.print(fSlope); Serial.print(","); 		// slope
	Serial.print(iLDR1Max); Serial.print(","); 		// Estimated maximum (white + backlight).
	Serial.print(r); Serial.print(","); 			// Red (estimated)
	Serial.print(g); Serial.print(","); 			// Green (estimated) 
	Serial.print(b); Serial.print(","); 			// Blue (estimated)
	Serial.print(iLDR1Min); Serial.print(","); 		// Estimated minimum (black + backlight).
	Serial.println(iLDRValue);						// ldr raw
	#endif

	// Return if we are active or not.
	return bActive ? PULSE_STATE__ACTIVE_PULSE : PULSE_STATE__NO_PULSE;
}

/**
 * @brief Handle the clip behaviour for sync pulse mode.
 * This will wait for a pulse, and then display colour and height changes
 * as appropriate.
 */
void loopSyncPulseMode() {
	
	// Sync with the RGB pulse.
	int8_t ePulseState = checkPulse();
	
	// If we have a valid sync pulse, but were previously off screen...
	if (ePulseState == PULSE_STATE__ACTIVE_PULSE && bOnScreen == false)
	{
		// Update flags.
		bOnScreen = true;
		
		// Normal LED indicating it has a pulse.
		digitalWrite(PIN_LED, HIGH);
		
		// Interpolate the colours and update the RGB LED to match.
		eRGBMode = RGBMODE_SCREEN;
	}
	
	// Otherwise if we are not on the screen, but previously were...
	else if (ePulseState == PULSE_STATE__NO_PULSE && bOnScreen == true)
	{			
		// Update flags.
		bOnScreen = false;
		
		// Normal LED indicating it has no pulse.
		digitalWrite(PIN_LED, LOW);
		
		// No pulse - reset max and min for LDR2.
		ldr2.resetLimits();
	}
	
	
	// If our last known state was on the screen, do all the sampling.
	if (bOnScreen)
	{
		// Update the target height.
		sampleHeight();
		
		// Move the motor in the direction of the target position.
		moveMotor();
	}
	
	//  Update the colour of the RGB LED.
	updateColour();
	
	// Read from the force sensor and serial stream all the outputs every 5ms.
	#ifdef SERIAL_INFO
	static unsigned long tmrSerial = 0;
	if (cycleCheck(&tmrSerial, 500U))
		debugSerial();
	#endif
}


/**
 * @brief Handle the clip behaviour for sync pulse mode.
 * This will wait for a pulse, and then display colour and height changes
 * as appropriate.
 * Note - For some reason this sampling routine seems to run SUPER SLOW.
 *      -  as a fix, I just run it at 10hz... ?!
 */
void loopHeightMode() {
	
	// Determine historic variables for height sampling.
	static WindowVariance* pMinAvg = new WindowVariance(5);
	static WindowVariance* pMaxAvg = new WindowVariance(5);
	
	static unsigned long tmrSample = 0;
	if (cycleCheck(&tmrSample, 100U))
	{
		// Push new data.
		int iSample1 = ldr1.sample();
		int iSample2 = ldr2.sample();
		
		// Compute the difference between the two means.
		uint16_t iDelta = abs(iSample1 - iSample2);
		
		// If they are quite different, we are probably not on the screen.
		boolean bOnScreen = iDelta <= 60;
		
		// If we are on screen for sure, update the max and mins and height.
		if (bOnScreen)
		{
			// Say we are on the screen.
			eRGBMode = RGBMODE_NONE;
			
			// Sample.
			uint16_t iMean = (iSample1 + iSample2) / 2;
			
			// Snap to means when in region.
			int iMin = pMinAvg->mean();
			int iMax = pMaxAvg->mean();
			if (pMinAvg->count() == 0 || iMin + 20 > iMean) pMinAvg->push(iMean);
			if (pMaxAvg->count() == 0 || iMax - 20 < iMean) pMaxAvg->push(iMean);
			
			// Target the motor position.
			iTargetPos = constrain(map(iMean, iMin, iMax, 0, MOTOR_TRAVEL), 0, MOTOR_TRAVEL);
			
			/*
			Serial.print(bOnScreen ? 100 : 0); Serial.print(",");	
			Serial.print(iMin); Serial.print(",");
			Serial.print(iMax); Serial.print(",");
			Serial.print(iMean); Serial.print("\n");
			*/
		}
		else
		{
			// Say we are not on the screen.
			eRGBMode = RGBMODE_NOSCREEN;
			
			// Deflate max and min incase they got pushed too high or low?
			//iMin = 0xFFF;
			//iMax = 0x000;
		}
	}
	
	// Update the colour and move the motor.
	moveMotor();
	updateColour();
	
}

static const unsigned char BitsSetTable256[256] = 
{
#   define B2(n) n,     n+1,     n+1,     n+2
#   define B4(n) B2(n), B2(n+1), B2(n+1), B2(n+2)
#   define B6(n) B4(n), B4(n+1), B4(n+1), B4(n+2)
    B6(0), B6(1), B6(1), B6(2)
};



WindowVariance * runningVariance = new WindowVariance( 20 );
int oldState = 0;
uint16_t bSerialBuffer = 0x00;
int inputIndex = 0;
int readState = 0;

int bitsSeen = 0;
bool flippedRead = false;
void loopSerialMode()
{
  // Only allow this function to be called once every 5ms.
  static unsigned long tmr1 = 0;
//if (cycleCheck(&tmr1, 1U))
  {
    int iLDRValue[] = { analogRead( PIN_LDR1 ), analogRead( PIN_LDR2 ) };
    int iInVal = iLDRValue[0] - iLDRValue[1];
	
	if( flippedRead )
		iInVal = iLDRValue[1] - iLDRValue[0];
		
	if( bitsSeen > 40 )
	{
		//Serial.println( "Inverted read mode!" );
		//flippedRead = !flippedRead;
		bitsSeen = 0;
	}
	
    runningVariance->push( iInVal );
    
    //iInVal = constrain( (int)runningVariance->mean(), -512, 512 ) + 512;
    int variance = runningVariance->variance();
    
    int state = constrain( map( iInVal, -450, 500, 0, 4 ), 0, 4 );
    
    //Serial.print( variance, DEC );
    //Serial.print( "\t" );
    //Serial.print( iInVal, DEC );
    //Serial.print( "\t" );
    //Serial.println( state, DEC );
    
    
    if( state != oldState )
    {
	  bitsSeen++;
	
      switch( state )
      {
        case 0:
          bSerialBuffer = (bSerialBuffer << 1) | 1;
		  Serial.print( "^" );
          break;
          
        case 1:
			inputIndex++;
			Serial.print( "~" );
			break;
        
        case 3:
          bSerialBuffer = (bSerialBuffer << 1);
		  Serial.print( "_" );
          break;
      }
      
      byte eFrame = (byte)(bSerialBuffer & 0x01);
      byte parity = (bSerialBuffer >> 1) & 0x01;
      byte frame  = (byte)((bSerialBuffer >> 2) & 0xFF);
      byte startBit = (bSerialBuffer >> 10) & 0x01;
      byte sFrame    = (bSerialBuffer >> 11) & 0x01;

      oldState = state;
      
      if( sFrame == 0 && eFrame == 0 && startBit == 1 )
      {
        if( (BitsSetTable256[frame] % 2 == 0 && parity == 1) || (BitsSetTable256[frame] % 2 != 0 && parity == 0) )
        {
		  bitsSeen = 0;
          Serial.print( "[" );
          Serial.print( frame, BIN );
          Serial.print( " " );
          Serial.print( frame, HEX );
		  Serial.print( " " );
          Serial.print( (char)frame );
          Serial.print( "]\t" );
          Serial.println( bSerialBuffer, BIN );
          
          //iTargetPos = map( frame & 0x7F, 0, 0x7F, 0, MOTOR_TRAVEL );
		  switch( frame )
		  {
		    case 'R': pRGB.setPixelColor(0,  255, 0, 0); break;
			case 'G': pRGB.setPixelColor(0,  0, 255, 0); break;
			case 'B': pRGB.setPixelColor(0,  0, 0, 255); break;
		  }
          pRGB.show();
          
          bSerialBuffer = 0;
        }
      }
    }

  }

  moveMotor();
  //updateColour();
}

/**
 * Logic loop.
 * The function performs the following tasks:
 *   - 
 */
void loop() {
	
	// Listen for a change in mode.
	detectModeChange();
	
	// Detect which mode we are in.
	switch (eClipMode)
	{
		// If we are in EEMODE_HEIGHTONLY mode, just use both LDRs for height.
		case EEMODE_HEIGHTONLY:
			loopHeightMode();
			break;
			
		// If we are in EEMODE_SYNCPULSE mode, use the height and colour.
		case EEMODE_SYNCPULSE:
			loopSyncPulseMode();
			break;
			
		// If we are in EEMODE_SERIAL, use the simple binary protocol.
		case EEMODE_SERIAL:
			loopSerialMode();
			break;
			
		// Unknown mode.. should not get here.
		default:
			break;
	}
	
	// If we are in profiling mode.
	#ifdef MODE_PROFILING
	
	static bool bMeasuring = false;
	static unsigned int iMeasureCount = 0;
	
	// Process new commands.
	if (Serial.available())
	{
		byte data = Serial.read();
		switch (data)
		{
			case 'p':
				Serial.println("pong");
				break;
			case 's':
				Serial.print("start "); 
				Serial.println(iMeasureCount);
				bMeasuring = true;
				break;
			case 'f':
				Serial.print("stop ");
				Serial.println(iMeasureCount);
				bMeasuring = false;
				iMeasureCount ++;
				break;
		}
	}
	
	// Print out if measuring.
	if (bMeasuring)
	{
		static unsigned long tmrSample = 0;
		if (cycleCheck(&tmrSample, 50U))
		{
			Serial.print(iMotorPosition); Serial.print(","); 	// Current MOTOR HEIGHT.
			Serial.print(iTargetPos); Serial.print(","); 		// Target MOTOR HEIGHT.
			Serial.print(iTargetR); Serial.print(","); 			// Target RED
			Serial.print(iTargetG); Serial.print(","); 			// Target GREEN
			Serial.print(iTargetB); Serial.print("\n"); 		// Target BLUE.
			
			//#define HRGB(h,r,g,b) 	(((h & 0xFF) << 24) | ((r & 0xFF) << 16) | ((g & 0xFF) << 8)  | ((b & 0xFF) << 0))
			//Serial.println(HRGB(iTargetPos, iTargetR, iTargetG, iTargetB), HEX); // Target HRGB.
		}
	}
	#endif
	
}









