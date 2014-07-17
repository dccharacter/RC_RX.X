#include "htc.h"
#include "usart.h"

void serialPutCh (char txByte)
{
    while (!TXIF);
    TXREG = txByte;
}

char serialGetCh (void)
{
	if (!RCIF) return -1;
	if (FERR) return -2;
	return 	RCREG;
}

void serialPutS (const char * txstr)
{
	while (*txstr)
	{
		 serialPutCh(*txstr++);
	}
}
