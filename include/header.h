#pragma once
#include <stdlib.h>
#include <stdint.h>
#include "iso22133.h"


enum ISOMessageReturnValue decodeISOHeader(const char *MessageBuffer, const size_t length,
											 HeaderType * HeaderData, const char debug);

HeaderType buildISOHeader(enum ISOMessageID id, uint32_t messageLength, const char debug);
