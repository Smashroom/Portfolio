#include "Arduino.h"

#ifndef Lcd_h_
#define Lcd_h_



class Lcd
{	
	public:
		void LcdWrite(uint8_t,uint8_t); //
		void LcdClear(void); //
		void LcdInitialise(void);
		void Spacer();	//
		void DisplayTime(uint8_t,uint8_t,uint8_t);	//
		void DisplaySecondIndicator(uint8_t,bool);
		void DrawSecondsBar(uint8_t);
		void InitializeDisplay();
		void LcdBitmap(char*);
};


#endif
