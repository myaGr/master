/**
* Role of macro.h:
* 1. Defines macros that are used to enable or disable certain functions
*     in Windows/Linux playback mode. It plays the role of the counterpart of Makefiles.
* 2. Defines some global macros used for HOST environment integration.
*/

#ifndef _AGGNSS_MACRO_H_
#define _AGGNSS_MACRO_H_

#define SYSAPI_UBX            1
#define SYSAPI_RTK_SFREQ      2

#define PLT_GENERIC_LINUX     1
#define PLT_OCEANA            2
#define PLT_BANMA             3
#define PLT_HTC               4
#define PLT_XM                5
#define PLT_SAMSUNG           6
#define PLT_QUALCOM           7
#define PLT_HUAWEI_UBX        8
#define PLT_THROUGH_SDK       9
#define PLT_WINDOWS_UBX       10

/* current valid interface version: 140/141. Comment out by default. */
//#define AGGNSS_API        141
#ifdef AGGNSS_API
#define API_VERSION_NUM   AGGNSS_API
#else
#define API_VERSION_NUM   130
#endif

#define AGGNSS_API_V140   (API_VERSION_NUM >= 140 && API_VERSION_NUM < 150)

/**
* ENABLE_ASSERT macro controls the enabler of assert function.
* Don't open up this macro unless you want to terminate program as soon as the assert you 
* add is true.
*/
//#define ENABLE_ASSERT

/**
* AG_GNSS_RTD_FUNCTION_IMPL: enable RTD functionality in playback mode.
* AG_GNSS_RTK_FUNCTION_IMPL: enable RTD functionality in playback mode.
* The above two macros are mutually exclusive.
*
* _RTCM32_ENABLE_: enable rtcm3 related functions. Its precondition is AG_GNSS_RTK_FUNCTION_IMPL
*/
#if defined(_WIN32) || defined(__GNUC__)
//#define AG_GNSS_RTD_FUNCTION_IMPL
#define AG_GNSS_RTK_FUNCTION_IMPL
#else
//#define AG_GNSS_RTD_FUNCTION_IMPL
#define AG_GNSS_RTK_FUNCTION_IMPL
#endif

#if defined(AG_GNSS_RTK_FUNCTION_IMPL)
#define _RTCM32_ENABLE_
#endif

#define HOST_API       SYSAPI_UBX
#define USE_MC_POS_STRATEGY
//#define AGGNSS_LOG_OUT_CB

/* Enable this definition if you need to enable some platform-dependent feature, like log filter. */
#ifndef PLATFORM
//#define PLATFORM       PLT_WINDOWS_UBX
#endif

//#define ENABLE_PPK_MODE

#define __RTK_LITE_MODE__
//#define ASG_USE_RTK_ALGO
//#define ASG_ALGO_EXPIRE_TIME (324000)

#ifdef _WIN32
#define ASG_LOG_FULL_OUTPUT
#define PLAYBACK_MODE
#define PE_DEBUG_LOG
#elif defined(__linux__)
//#define ASG_LOG_FULL_OUTPUT
//#define PLAYBACK_MODE
#endif

#define FREQ_L1L5

#endif
