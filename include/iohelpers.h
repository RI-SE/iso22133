#pragma once

#include <stdint.h>
#include <stdio.h>

typedef void (*DebugPrinter_t)(const void*);
// ************************* Debug printout helper data **********************************************************
/*! Debug formatting functions*/
void printU8(const void* val)  { printf("%u",  *(const uint8_t*)val); }
void printU16(const void* val) { printf("%u",  *(const uint16_t*)val); }
void printU32(const void* val) { printf("%u",  *(const uint32_t*)val); }
void printU64(const void* val) { printf("%lu", *(const uint64_t*)val); }
void printI8(const void* val)  { printf("%d",  *(const int8_t*)val); }
void printI16(const void* val) { printf("%d",  *(const int16_t*)val); }
void printI32(const void* val) { printf("%d",  *(const int32_t*)val); }
void printI64(const void* val) { printf("%ld", *(const int64_t*)val); }
void printString(const void* val) { printf("%s", (const char*)val); }

/*! Debug struct */
typedef struct {
	char* name;
	char* unit;
	DebugPrinter_t printer;
} DebugStrings_t;

int encodeContent(uint16_t valueID, const void* src, char** dest, const size_t contentSize, size_t* bufferSpace, DebugStrings_t *debugStruct, const char debug);

void printContent(const uint16_t valueID, const uint16_t contentLength, const void* value, DebugStrings_t* deb);
