/*
  WindowVariance.h -- Compute a windowed variance of a set of numbers. - Version 0.1
  
  
*/

#include "Arduino.h"

// Ensure this library description is only included once.
#ifndef WindowVariance_h
#define WindowVariance_h




class WindowVariance
{
	private:
		float* tItems;					// The array of items in the window.
		int iIndex; 					// The item index.
		int iWindowCount;				// The number of items in the window.
		int iWindowMaxSize; 			// The maximum number of items in the window.
	public:
		
		/**
		 * @brief A constructor which sets up this window variance with a custom window size.
		 */
		WindowVariance(int iWindowMaxSize_p) 
		{ 
			iWindowMaxSize = iWindowMaxSize_p;
			tItems = new float[iWindowMaxSize];
			iWindowCount = 0;
			iIndex = -1;
			reset();
		}
		
		/**
		 * @brief A destructor which removes the assigned memory.
		 */
		~WindowVariance()
		{
			delete[] tItems;
			reset();
		}
		
		/**
		 * @brief Reset this so that it is ready for new data.
		 */
		void reset()
		{
			iWindowCount = 0;
			iIndex = -1;
		}
		
		/**
		 * @brief Push a new value into this running variance of items.
		 * @param x The value to push. 
		 */
		void push(double x)
		{
			// Compute the new array position.
			iIndex = (iIndex + 1) % iWindowMaxSize;
			
			// Increment the item count.
			if (iWindowCount < iWindowMaxSize)
				iWindowCount++;
			
			// Add it into the array at the correct index.
			tItems[iIndex] = (float)x;
		}
		
		/**
		 * @brief Add a given amount to all the items.
		 * This can be used to tend up or down with samples.
		 */
		void inflate(float fAmount) const
		{
			for (int i = 0; i < iWindowCount; ++i)
				tItems[i] += fAmount;
		}
		
		/**
		 * @brief Return the number of items in the window.
		 * @return A positive integer with the number of items in the window.
		 */
		int count() const
		{
			return iWindowCount;
		}
		
		/**
		 * @brief Return the maximum size of this window.
		 * @return A positive integer with the number of items the window can contain.
		 */
		int size() const
		{
			return iWindowMaxSize;
		}
		
		/**
		 * @brief Return the mean of all the items in the window.
		 * If there are no values pushed, this will return 0.
		 * @return A floating point value.
		 */
		double mean() const
		{
			if (iWindowCount == 0) return 0.0f;
			
			float fSum = 0.0f;
			for (int i = 0; i < iWindowCount; ++i)
				fSum += tItems[i];
			return fSum / (float)iWindowCount;
		}
		
		/**
		 * @brief Return the variance of all the items in the window.
		 * If there are no values pushed, this will return 0.
		 * This is the sd squared.
		 * @return A floating point value.
		 */
		double variance() const
		{
			if (iWindowCount == 0) return 0.0f;
			
			float fMean = mean();
			float fSum = 0.0f;
			for (int i = 0; i < iWindowCount; ++i)
				fSum += (tItems[i] - fMean) * (tItems[i] - fMean);
			return fSum / (float)(iWindowCount - 1);
		}
		
		/**
		 * @brief Return the standard deviation of all the items in the window.
		 * @return A floating point value.
		 */
		double sd() const
		{
			return sqrt( variance() );
		}
};

#endif

