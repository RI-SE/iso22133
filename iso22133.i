/* File : iso22133.i */
%module iso22133

%{
#include "iso22133.h"
#include "positioning.h"
#define SWIG_PYTHON_STRICT_BYTE_CHAR
%}

%apply int *OUTPUT {ObjectCommandType* command};
%inline %{
	extern ssize_t decodeOSTMMessage(const char* ostmDataBuffer, const size_t bufferLength, enum ObjectCommandType* command, const char debug);
%}

%apply int *OUTPUT {uint32_t *senderID}
%inline %{
extern ssize_t decodeOSEMMessage(ObjectSettingsType *objectSettingsData, const char * osemDataBuffer, const size_t bufferLength, const char debug);
%}


#%javaconst(1);

typedef double double_t;
typedef long int ssize_t;

%include "typemaps.i"
%include "stdint.i"
%include "cpointer.i"
%include "positioning.i"
%include "iso22133.h"
