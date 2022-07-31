/*
  BiQuadFilter.cpp
  Copyright (c) 2016 Phillip Schmidt.  All right reserved.

*/

#include "BiQuadFilter.h"



void BiQuadFilterInit(struct BiQuadFilter* biquad, int16_t cutOffFreq, int16_t sampleFreq, float Q) // initialize filter
{
   float Fc = cutOffFreq;  // cutoff freq must not be above half of sample freq
   float Fs = sampleFreq;

   // generate fixed point coefficients
   float R = tan(3.1415927f * Fc / Fs);
   float Normal = 1.0f / (1.0f + R / Q + R * R);

   biquad->a0 = (int16_t)(R * R * Normal * 16384.0f); // offset by 2^14 (max expected coefficient is +/-2.0)
   biquad->a1 = 2 * biquad->a0;
   biquad->a2 = biquad->a0;

   biquad->b1 = (int16_t)( 2.0f * (R * R - 1.0f) * Normal * 16384.0f);
   biquad->b2 = (int16_t)((1.0f - R / Q + R * R) * Normal * 16384.0f);

}


int CalcBiQuad(struct BiQuadFilter* biquad, int16_t value) // data input function
{

   // multiply samples by coefficients -- accumulate data in a Long to reduce rounding errors
	biquad->result.L =	(int32_t)((int32_t)value * (int32_t)biquad->a0 +
			(int32_t)biquad->IN_1 *  (int32_t)biquad->a1 +
			(int32_t)biquad->IN_2 *  (int32_t)biquad->a2 -
			(int32_t)biquad->OUT_1 * (int32_t)biquad->b1 -
			(int32_t)biquad->OUT_2 * (int32_t)biquad->b2);

	biquad->result.L = biquad->result.L << 2; // bitshift to make up for non 16bit coefficient offset

   // shift data
	biquad->IN_2  = biquad->IN_1;
	biquad->IN_1  = value;
	biquad->OUT_2 = biquad->OUT_1;
	biquad->OUT_1 = biquad->result.I[1];

   return biquad->result.I[1]; // the H16 part of the Long is the result
}


int OutBiQuad(struct BiQuadFilter* biquad) // return the value of the most recent result without submitting new data
{
   return biquad->result.I[1];
}


