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
#include "LDR.h"
#define DEVICE_MKIII


/* External and 3rd party libraries. */ 
#include <Math.h>
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

#define USE_SWBOT
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
#define MOTOR_TRAVEL 450	// The number of steps to move the nut from the top to the bottom.

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

#define NO_SYNC_PULSE_COLOUR RGB(255, 0, 0)

#define RGBMODE_NONE     0		// The RGB LED displays nothing.
#define RGBMODE_SCREEN   1		// The RGB LED displays the last colour the pulses detected.
#define RGBMODE_NOSCREEN 2		// The RGB LED displays the "not on screen" pattern (i.e. no pulse detected).
#define RGBMODE_SETUP    3		// The RGB LED displays the "setting up / calibrating" pattern.

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

int eRGBMode = RGBMODE_NONE;	// Current RGB mode.
bool bOnScreen = false;			// Is the shape clip device on the screen (i.e. does it have a valid sync pulse)?
bool bLastOnScreen = false;		// As above, 1 state change behind.

int iTargetR = 0;		// Target RED component.
int iTargetG = 0;		// Target GREEN component.
int iTargetB = 0;		// Target BLUE component.
int iTargetPos = 0;		// Target MOTOR height (steps).

int iLDR1Min = 0;	// Current estimated LDR minimum.
int iLDR1Max = 0;	// Current estimated LDR maximum.

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
 * @brief Process the input signals.

void processInputs() {
	
	// LDRs.
	ldr1.update();
	ldr2.update();
	
	// Top and Bottom switches and conditions.
	boolean bTopTmp    	= digitalRead(PIN_SWTOP) == LOW;
	boolean bBottomTmp 	= digitalRead(PIN_SWBOT) == LOW;
	bTopChange			= bTopTmp == bTop;
	bBottomChange 		= bBottomTmp == bBottom;
	bTop 				= bTopTmp;
	bBottom 			= bBottomTmp;
	
	// Serial messages?
}
 */
 
/**
 * @brief Print out information from the various sensors and estimated values.
 */
void debugSerial() {
	
	// Write out values.
	Serial.print(millis()); Serial.print(","); 									// ms
	Serial.print(analogRead(PIN_LDR1)); Serial.print(","); 						// ldr1
	Serial.print(ldr2.sample()); Serial.print(","); 						// ldr2
	Serial.print(RGB(iTargetR, iTargetG, iTargetB), HEX); Serial.print(" "); 		// RGB
	Serial.print(pRGB.getPixelColor(0), HEX); Serial.print(","); 		// RGB

	Serial.print(iTargetPos); Serial.print(","); 								// height
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
	eRGBMode = RGBMODE_SETUP;
	updateColour();
	
	// Until the motor is calibrated, step down each time.
	for(int iAvailableSteps = MOTOR_TRAVEL; iAvailableSteps >= 0; iAvailableSteps--) {
		motor.step(-1);

		#ifdef USE_SWBOT
		if (digitalRead(PIN_SWBOT) == LOW) break;
		#endif
	}
	
	// Set the motor position to 0 (known location).
	bMotorCalibrated = true;
	iMotorPosition = 0;
	
	// Take us out of setup mode.
	eRGBMode = eLastMode;
	updateColour();
}

/**
 * @brief Check the sync pulse and update relevant global variables.
 * This is automatically rate limited to 5ms samples.
 * It will update the iTargetR,G,B, iLDR1Max and iLDR1Min values if valid pulses are detected.
 * @return -1 if check not performed, 1 if we received an active pulse, 0 if we did not.
 */
#define PULSE_STATE__NOT_CHECKED  -1
#define PULSE_STATE__NO_PULSE      0
#define PULSE_STATE__ACTIVE_PULSE  1
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
 * @brief Read new height data from the screen.
 * It will update the iTargetPos value with the height between 0 and MOTOR_TRAVEL.
 */
void sampleHeight() {
	uint16_t analogValue = ldr2.sample();
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
	/*
	Serial.print(iTargetPos);
	Serial.print(",");
	Serial.print(iMotorPosition);
	Serial.print(",");
	Serial.println(iDelta);
	*/
	
	/*
	// Move upwards one step.
	if (iDelta > 20)
	{
		motor.step(1);
		iMotorPosition += 1;
	}
	*/
	
	// Move downwards one step. <-- THIS COMMENT IS WRONG!!!!
	if (abs(iDelta) > 20)
	{
		int step = iDelta >= 0 ? 1 : -1;
		motor.step(step);
		iMotorPosition += step;
	} else {
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
				uint32_t currentColor = pRGB.getPixelColor(0);
				
				if(currentColor != RGB(iTargetR, iTargetG, iTargetB)) {
					int stepRed   = constrain(map(10, 0, 100, RED(currentColor), iTargetR), 0, iTargetR);
					int stepGreen = constrain(map(10, 0, 100, RED(currentColor), iTargetG), 0, iTargetG);
					int stepBlue  = constrain(map(10, 0, 100, RED(currentColor), iTargetB), 0, iTargetB);

					pRGB.setPixelColor(0,  stepRed, stepGreen, stepBlue);
					//pRGB.setPixelColor(0,  iTargetR, iTargetG, iTargetB);
				}
			}
			break;
		case RGBMODE_NOSCREEN:
			pRGB.setPixelColor(0,  0, 0, 0);
			break;
		case RGBMODE_SETUP:
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
	Serial.begin(9600);
	
	// Wait two seconds.
	Serial.println("hello world");
	
		// wait for a random time so that not all clips zero their motor at the same time
		randomSeed(analogRead(PIN_RND));
		uint16_t randomWait = random(1000);
		delay(randomWait);

	// Bring the motor down until it hits the bottom.
	zeroMotor();
	
		// make sure we all start at the same time
		delay(1000 - randomWait);
}







/**
 * Logic loop.
 * The function performs the following tasks:
 *   - 
 */
void loop() {
	
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
		
		// If it has been:
		//	3s with no pulse:  flash no screen.
		//  10s with no pulse: move motor to 0.
		/*
		if (10s has passed)
		{
			// Move motor to 0.
			zeroMotor();
		}
		 else if (3s has passed)
		{
			// Flash red for 10s before going black.
			eRGBMode = RGBMODE_NOSCREEN;
		}
		*/
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
	
	/*
	// Stress the motor.
	digitalWrite(PIN_LED, HIGH);
	motor.step(-MOTOR_TRAVEL);
	delay(1000);
	digitalWrite(PIN_LED, LOW);
	motor.step(MOTOR_TRAVEL);
	delay(1000);
	*/
	
	/*
	// Get the target LDR value.
	int iTargetPosition = ldr1.percent * MOTOR_TRAVEL;
	int iDelta = iTargetPosition - iMotorPosition;
	
	// Remove noise. 
	if ( abs(iDelta) > 10 )
	{
		if (iDelta > 0)
		{
			motor.step(-1);
			iMotorPosition += 1;
			digitalWrite(PIN_LED, HIGH);
		}
		else
		{
			motor.step(1);
			iMotorPosition -= 1;
			digitalWrite(PIN_LED, HIGH);
		}
	}
	*/
}









