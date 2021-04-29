/*
 * nextion.h
 *
 *  Created on: 31 de mar de 2021
 *      Author: ipizf
 */

#ifndef INC_NEXTION_H_
#define INC_NEXTION_H_

void SendToProgressBar(char *obj, uint16_t value);
void SendToGauge(char *obj, uint16_t value);
void SendToWave(char *obj, uint16_t value);
void SendText(char *obj, uint16_t value);
void SendToSlider(char *obj, uint16_t value);
void GetVal(char *obj);
void GetPage();

#endif /* INC_NEXTION_H_ */
