#include "Arduino.h"

#ifndef buzzer_h_
#define buzzer_h_

class buzzer
{
	public:
		void sing();
		void buzz(int,long,long);
		void cutButton();
		
};

#endif
