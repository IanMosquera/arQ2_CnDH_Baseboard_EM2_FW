/*
 * DateTime.c
 *
 *  Created on: Jul 7, 2025
 *      Author: IanMo
 */

#include "DateTime.h"

#include "string.h"
#include "stdlib.h"
#include "stdio.h"


void Extract_DateTime_From_String(char *pStr, dateTime_t *pDt)
{
	uint8_t cntx = 0;

	char YY[3] = {};
	char MM[3] = {};
	char DD[3] = {};
	char hh[3] = {};
	char mm[3] = {};
	char ss[3] = {};

	char *ptr;
	char TOKEN[10]	= "/,:\"";

	ptr = strtok(pStr, TOKEN);
	strcpy(YY, ptr);

	while (ptr != 0)
	{
		ptr = strtok(0, TOKEN);
		cntx++;

		if (cntx == 1) strcpy(MM, ptr);
		if (cntx == 2) strcpy(DD, ptr);
		if (cntx == 3) strcpy(hh, ptr);
		if (cntx == 4) strcpy(mm, ptr);
		if (cntx == 5) strcpy(ss, ptr);
	}

	pDt->Year		= atoi(YY);
	pDt->Month		= atoi(MM);
	pDt->Days		= atoi(DD);
	pDt->Hour		= atoi(hh);
	pDt->Min		= atoi(mm);
	pDt->Sec		= atoi(ss);
}
