/*
  RunningVariance.h -- Compute a running variance of a set of numbers. - Version 0.1
  
  Based on: http://www.johndcook.com/standard_deviation.html
  
*/

#include "Arduino.h"

// Ensure this library description is only included once.
#ifndef RunningVariance_h
#define RunningVariance_h

class RunningVariance
{
	private:
		int iItems;										// The number of values in the set.
		double iOldMean, iNewMean, iOldSD, iNewSD;		// The values required to store running variance.
	
	public:
		
		/**
		 * @brief A constructor which resets the number of items to 0.
		 */
		RunningVariance() : iItems(0) {}
		
		/**
		 * @brief Reset this so that it is ready for new data.
		 */
		void reset()
		{
			iItems = 0;
		}
		
		/**
		 * @brief Push a new value into this running variance of items.
		 * @param x The value to push. 
		 */
		void push(double x)
		{
			// Increment the item count.
			iItems++;
			
			// If we have one item, set up the values.
			if (iItems == 1)
			{
				iOldMean = iNewMean = x;
				iOldSD = 0.0;
			}
			
			// Otherwise, update the mean and sd values.
			else
			{
				// See Knuth TAOCP vol 2, 3rd edition, page 232
				iNewMean = iOldMean + (x - iOldMean) / iItems;
				iNewSD = iOldSD + (x - iOldMean)*(x - iNewMean);
				
				// set up for next iteration
				iOldMean = iNewMean; 
				iOldSD = iNewSD;
			}
		}
		
		/**
		 * @brief Return the number of items in the set.
		 * @return A positive integer with the number of items accounted for in the set.
		 */
		int count() const
		{
			return iItems;
		}
		
		/**
		 * @brief Return the mean of all the items in the set.
		 * @return A floating point value.
		 */
		double mean() const
		{
			return (iItems > 0) ? iNewMean : 0.0f;
		}
		
		/**
		 * @brief Return the variance of all the items in the set.
		 * This is the sd squared.
		 * @return A floating point value.
		 */
		double variance() const
		{
			return ( (iItems > 1) ? iNewSD / (iItems - 1) : 0.0f );
		}
		
		/**
		 * @brief Return the standard deviation of all the items in the set.
		 * @return A floating point value.
		 */
		double sd() const
		{
			return sqrt( variance() );
		}
};

#endif

