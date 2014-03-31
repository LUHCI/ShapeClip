/*
  Kalman.h -- Compute a 1d Kalman filtered number. - Version 0.1
  
  
*/

#include "Arduino.h"

// Ensure this library description is only included once.
#ifndef Kalman_h
#define Kalman_h


class Kalman
{
	private:
		double fLast;		// The previous estimated value.
		double fLP;			// fLast gain.
		double fStarting;	// New starting value.
	
	public:
		double fValue;		// The current smoothed value.
		double A;			// The factor of the real value to the previous real value.
		double B;			// The factor of the real value to the real control signal.
		double H;			// The factor of the measured value to the real value.
		double Q;			// Process noise variance.
		double fNoise;		// The environment noise.

		
		/**
		 * @brief A constructor which sets up with a starting value.
		 */
		Kalman(double fStartingValue) 
		{ 
			fLast = 0.0;		// The previous estimated value.
			fLP = 0.0;			// fLast gain.
			fStarting = fStartingValue;	// New starting value.
			
			fValue = 0.0;		// The current smoothed value.
			A = 1.0;			// The factor of the real value to the previous real value.
			B = 0.0;			// The factor of the real value to the real control signal.
			H = 1.0;			// The factor of the measured value to the real value.
			Q = 0.01;			// Process noise variance.
			fNoise = 0.23;		// The environment noise.
			
			reset();
		}
		
		/**
		 * @brief Reset this so that it is ready for new data.
		 */
		void reset()
		{
			fValue = fStarting;
			fLast = fStarting;
			fLP = 0.1;
		}
		
		/**
		 * @brief Push a new value to be filtered.
		 * @param fValue The value to push. 
		 */
		void push(double fValue)
		{
			// time update - prediction
			fLast = A * fLast;
			fLP = A * fLP * A + Q;
			
			// measurement update - correction
			double K = fLP * H / (H * fLP * H + fNoise);
			fLast = fLast + K * (fValue - H * fLast);
			fLP = (1.0 - K * H) * fLP;
			
			// Store the update.
			fValue = fLast;
		}
		
		/**
		 * @brief Return the filtered value.
		 * @return A double precision floating point value.
		 */
		double valued() const
		{
			return fValue;
		}
		
		/**
		 * @brief Return the filtered value.
		 * Warning: precision loss likely.
		 * @return A single precision floating point value.
		 */
		float valuef() const
		{
			return (float)fValue;
		}
		
		/**
		 * @brief Return the filtered value.
		 * Warning: precision loss very likely.
		 * @return An integer value coerced from the floating point.
		 */
		int valuei() const
		{
			return (int)fValue;
		}
};

#endif

