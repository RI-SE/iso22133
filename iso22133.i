/* File : iso22133.i */
%module iso22133

%{
#include "iso22133.h"
#include "positioning.h"
#define SWIG_PYTHON_STRICT_BYTE_CHAR
%}
/*
%apply int { ControlCenterStatusType controlCenterStatus }
%inline %{
struct HeabMessageDataType {
	uint32_t transmitterID;
	struct timeval dataTimestamp;
	ControlCenterStatusType controlCenterStatus;
};
%}
*/
%apply int *OUTPUT {ObjectCommandType* command};
%inline %{
	extern ssize_t decodeOSTMMessage(const char* ostmDataBuffer, const size_t bufferLength, ObjectCommandType* command, const char debug);
%}

%apply int *OUTPUT {uint32_t *senderID}
%inline %{
extern ssize_t decodeOSEMMessage(ObjectSettingsType *objectSettingsData, const char * osemDataBuffer, const size_t bufferLength, uint32_t *senderID, const char debug);
%}

typedef double double_t;
typedef long int ssize_t;

struct timeval {
long int tv_sec;
long int tv_usec;
};

%include "typemaps.i"
%include "stdint.i"
%include "cpointer.i"
%include "iso22133.h"
%include "positioning.h"
%pointer_functions(uint32_t, uint32ptr);

