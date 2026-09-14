#ifndef COMMON_LIBRARY_H
#define COMMON_LIBRARY_H

#include <ArduinoUniqueID.h>

void get_unique_id(HardwareSerial &ser)
{
	UniqueIDdump(ser);
	ser.print("UniqueID: ");
	for (size_t i = 0; i < UniqueIDsize; i++)
	{
		if (UniqueID[i] < 0x10)
			ser.print("0");
		ser.print(UniqueID[i], HEX);
		ser.print(" ");
	}
	ser.println();
}

#endif