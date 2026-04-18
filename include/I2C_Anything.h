// Written by Nick Gammon
// May 2012

#ifndef _I2C_ANYTHING_
#define _I2C_ANYTHING_
#include <Arduino.h>
#include <Wire.h>

template <typename T> unsigned int I2C_writeAnything (const T& value)
  {
  Wire.write((byte *) &value, sizeof (value));
  return sizeof (value);
  }  // end of I2C_writeAnything

template <typename T> unsigned int I2C_readAnything(T& value)
  {
    byte * p = (byte*) &value;
    unsigned int i;
    for (i = 0; i < sizeof value; i++)
    {
      if(Wire.available()) {
        *p++ = Wire.read();
      } else {
        return i; // Stop if we run out of data to avoid garbage values
      }
    }
    return i;
  }  // end of I2C_readAnything

#endif
