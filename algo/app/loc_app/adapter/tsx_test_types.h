#ifndef TEST_TYPES_H_
#define TEST_TYPES_H_

#include "cmn_def.h"

BEGIN_DECL

#if defined(APP_HEXAGON_SSR)

#include "tsxadapter_types.h"
#include "tsxadapter.h"

typedef struct WeekSecMicrosec
{
    uint32_t week;
    uint32_t seconds;
    float microseconds;
} WeekSecMicrosec;

typedef struct CorrectionsStructure
{
    WeekSecMicrosec time;
    uint32_t ulBufferSize;
    uint8_t aucBuffer[5000];
} CorrectionsStructure;

typedef struct SubFrameStructure
{
    WeekSecMicrosec time;
    TSXConstellationType eCType;
    TSXSignalType eSType;
    uint8_t ucSVID;
    uint8_t ucFreq;
    uint32_t ulBufferSize;
    uint8_t aucBuffer[256];
} SubFrameStructure;

#endif // APP_HEXAGON_SSR

END_DECL
#endif // TEST_TYPES_H_
