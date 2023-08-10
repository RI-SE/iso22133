#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdint.h>
#include "iso22133.h"

enum ISOMessageReturnValue decodeISOHeader(const char *MessageBuffer, const size_t length,
											 HeaderType * HeaderData, const char debug);

HeaderType buildISOHeader(const HeaderType *input, const char debug);

#ifdef __cplusplus
}
#endif