#ifndef CPP_LEARN_MY_LIB_H
#define CPP_LEARN_MY_LIB_H
#include "dlib_export.h"
#include "stdint.h"
// #define EXPORT_DLL __declspec(dllexport)
// extern "C" EXPORT_DLL uint32_t add(uint16_t a, uint16_t b); // int add(int a,int b)

// DLIB_EXPORT int add(int a, int b); // int add(int a,int b)
extern DLIB_EXPORT uint32_t multiply(uint16_t a, uint16_t b); // int add(int a,int b)

#endif //CPP_LEARN_MY_DLL_H
