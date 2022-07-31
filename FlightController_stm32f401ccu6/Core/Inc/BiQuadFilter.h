/*
  BiQuadFilter.h
  Copyright (c) 2016 Phillip Schmidt.  All right reserved.

 */

#ifndef BiQuadFilter_h

   #define BiQuadFilter_h
#include "main.h"
#include <math.h>

   struct BiQuadFilter
   {





	   int16_t a0, a1, a2, b1, b2;	// coefficients
	   int16_t IN_1, IN_2, OUT_1, OUT_2; // delayed data

         union TypeConverter
         {
            long L;
            int16_t  I[2];
         } result;

   };

   void BiQuadFilterInit(struct BiQuadFilter* biquad, int16_t cutOffFreq, int16_t sampleFreq, float Q);
   int CalcBiQuad(struct BiQuadFilter* biquad, int16_t value);
   int OutBiQuad(struct BiQuadFilter* biquad);

#endif
