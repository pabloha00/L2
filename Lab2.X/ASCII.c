#include "ASCII.h"

char num_ascii(uint8_t num){
	switch(num){    
		default:
		return 0x30;
		break;
        
        case 0:
		return 0x30;
		break;
		
		case 1:
		return 0x31;
		break;

		case 2:
		return 0x32;
		break;
		
		case 3:
		return 0x33;
		break;

		case 4:
		return 0x34;
		break;
		
		case 5:
		return 0x35;
		break;

		case 6:
		return 0x36;
		break;
		
		case 7:
		return 0x37;
		break;

		case 8:
		return 0x38;
		break;
		
		case 9:
		return 0x39;
		break;
	}
}
