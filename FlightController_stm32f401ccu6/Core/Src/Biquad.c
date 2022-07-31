//
//  Biquad.cpp
//
//  Created by Nigel Redmon on 11/24/12
//  EarLevel Engineering: earlevel.com
//  Copyright 2012 Nigel Redmon
//
//  For a complete explanation of the Biquad code:
//  http://www.earlevel.com/main/2012/11/26/biquad-c-source-code/
//
//  License:
//
//  This source code is provided as is, without warranty.
//  You may copy and distribute verbatim copies of this document.
//  You may modify and use this source code to create binary code
//  for your own purposes, free or commercial.
//

#include <math.h>
#include "Biquad.h"


void BiquadInit(struct Biquad* biquad, int type, double Fc, double Q, double peakGainDB) {
    setBiquad(biquad, type, Fc, Q, peakGainDB);
    biquad->z1 = biquad->z2 = 0.0;
}

float process(struct Biquad* biquad, float in) {
    double out = in * biquad->a0 + biquad->z1;
    biquad->z1 = in * biquad->a1 + biquad->z2 - biquad->b1 * out;
    biquad->z2 = in * biquad->a2 - biquad->b2 * out;
    return out;
}

void setType(struct Biquad* biquad, int type) {
	biquad->type = type;
    calcBiquad(biquad);
}

void setQ(struct Biquad* biquad, double Q) {
	biquad->Q = Q;
    calcBiquad(biquad);
}

void setFc(struct Biquad* biquad, double Fc) {
	biquad->Fc = Fc;
    calcBiquad(biquad);
}

void setPeakGain(struct Biquad* biquad, double peakGainDB) {
	biquad->peakGain = peakGainDB;
    calcBiquad(biquad);
}
    
void setBiquad(struct Biquad* biquad, int type, double Fc, double Q, double peakGainDB) {
	biquad->type = type;
	biquad->Q = Q;
	biquad->Fc = Fc;
    setPeakGain(biquad, peakGainDB);
}

void calcBiquad(struct Biquad* biquad) {
    double norm;
    double V = pow(10, fabs(biquad->peakGain) / 20.0);
    double K = tan(M_PI * biquad->Fc);
    switch (biquad->type) {
        case bq_type_lowpass:
            norm = 1 / (1 + K / biquad->Q + K * K);
            biquad->a0 = K * K * norm;
            biquad->a1 = 2 * biquad->a0;
            biquad->a2 = biquad->a0;
            biquad->b1 = 2 * (K * K - 1) * norm;
            biquad->b2 = (1 - K / biquad->Q + K * K) * norm;
            break;
            
        /*case bq_type_highpass:
            norm = 1 / (1 + K / Q + K * K);
            a0 = 1 * norm;
            a1 = -2 * a0;
            a2 = a0;
            b1 = 2 * (K * K - 1) * norm;
            b2 = (1 - K / Q + K * K) * norm;
            break;
            
        case bq_type_bandpass:
            norm = 1 / (1 + K / Q + K * K);
            a0 = K / Q * norm;
            a1 = 0;
            a2 = -a0;
            b1 = 2 * (K * K - 1) * norm;
            b2 = (1 - K / Q + K * K) * norm;
            break;*/
            
        case bq_type_notch:
            norm = 1 / (1 + K / biquad->Q + K * K);
            biquad->a0 = (1 + K * K) * norm;
            biquad->a1 = 2 * (K * K - 1) * norm;
            biquad->a2 = biquad->a0;
            biquad->b1 = biquad->a1;
            biquad->b2 = (1 - K / biquad->Q + K * K) * norm;
            break;
            
       /* case bq_type_peak:
            if (peakGain >= 0) {    // boost
                norm = 1 / (1 + 1/Q * K + K * K);
                a0 = (1 + V/Q * K + K * K) * norm;
                a1 = 2 * (K * K - 1) * norm;
                a2 = (1 - V/Q * K + K * K) * norm;
                b1 = a1;
                b2 = (1 - 1/Q * K + K * K) * norm;
            }
            else {    // cut
                norm = 1 / (1 + V/Q * K + K * K);
                a0 = (1 + 1/Q * K + K * K) * norm;
                a1 = 2 * (K * K - 1) * norm;
                a2 = (1 - 1/Q * K + K * K) * norm;
                b1 = a1;
                b2 = (1 - V/Q * K + K * K) * norm;
            }
            break;
        case bq_type_lowshelf:
            if (peakGain >= 0) {    // boost
                norm = 1 / (1 + sqrt(2) * K + K * K);
                a0 = (1 + sqrt(2*V) * K + V * K * K) * norm;
                a1 = 2 * (V * K * K - 1) * norm;
                a2 = (1 - sqrt(2*V) * K + V * K * K) * norm;
                b1 = 2 * (K * K - 1) * norm;
                b2 = (1 - sqrt(2) * K + K * K) * norm;
            }
            else {    // cut
                norm = 1 / (1 + sqrt(2*V) * K + V * K * K);
                a0 = (1 + sqrt(2) * K + K * K) * norm;
                a1 = 2 * (K * K - 1) * norm;
                a2 = (1 - sqrt(2) * K + K * K) * norm;
                b1 = 2 * (V * K * K - 1) * norm;
                b2 = (1 - sqrt(2*V) * K + V * K * K) * norm;
            }
            break;
        case bq_type_highshelf:
            if (peakGain >= 0) {    // boost
                norm = 1 / (1 + sqrt(2) * K + K * K);
                a0 = (V + sqrt(2*V) * K + K * K) * norm;
                a1 = 2 * (K * K - V) * norm;
                a2 = (V - sqrt(2*V) * K + K * K) * norm;
                b1 = 2 * (K * K - 1) * norm;
                b2 = (1 - sqrt(2) * K + K * K) * norm;
            }
            else {    // cut
                norm = 1 / (V + sqrt(2*V) * K + K * K);
                a0 = (1 + sqrt(2) * K + K * K) * norm;
                a1 = 2 * (K * K - 1) * norm;
                a2 = (1 - sqrt(2) * K + K * K) * norm;
                b1 = 2 * (K * K - V) * norm;
                b2 = (V - sqrt(2*V) * K + K * K) * norm;
            }
            break;*/
    }
    
    return;
}
