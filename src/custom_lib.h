#ifndef CUSTOM_LIB_H
#define CUSTOM_LIB_H

#include "definitions.h"

int m_atoc(char* p);
char* m_ctoa(char value, char* result, int base);
char* m_itoa(int value, char* result, int base);
char* m_ultoa(unsigned long value, char* result, int base);
void pln(char* msg);
void pln2(char* msg);

/// Experimental
void m_strcpy(char* a, char* b); // copies a into b

#endif
