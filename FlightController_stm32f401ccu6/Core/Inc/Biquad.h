//
//  Biquad.h
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

#ifndef Biquad_h
#define Biquad_h

#include <main.h>

enum {
    bq_type_lowpass = 0,
    bq_type_highpass,
    bq_type_bandpass,
    bq_type_notch,
    bq_type_peak,
    bq_type_lowshelf,
    bq_type_highshelf
};

 struct Biquad {

    int type;
    double a0, a1, a2, b1, b2;
    double Fc, Q, peakGain;
    double z1, z2;
};

void BiquadInit(struct Biquad* biquad, int type, double Fc, double Q, double peakGainDB);
    void setType(struct Biquad* biquad, int type);
    void setQ(struct Biquad* biquad, double Q);
    void setFc(struct Biquad* biquad, double Fc);
    void setPeakGain(struct Biquad* biquad, double peakGainDB);
    void setBiquad(struct Biquad* biquad, int type, double Fc, double Q, double peakGain);
    float process(struct Biquad* biquad, float in);

    void calcBiquad(struct Biquad* biquad);



#endif // Biquad_h
