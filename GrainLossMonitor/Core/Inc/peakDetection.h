/*
 * peakDetection.h
 *
 *  Created on: 31 de mar de 2021
 *      Author: ipizf
 */

#ifndef INC_PEAKDETECTION_H_
#define INC_PEAKDETECTION_H_

#define NUMBER_OF_CONVERSION 1
#define SAMPLE_LENGTH 200

#define lag 8
#define threshold  4
#define influence 0.6


void thresholding(float y[], int signals[]);
float stddev(float data[], int len);
float mean(float data[], int len);

int contPeak(int signal[]);

#endif /* INC_PEAKDETECTION_H_ */
