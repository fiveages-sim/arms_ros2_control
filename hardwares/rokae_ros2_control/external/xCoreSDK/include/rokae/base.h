/**
 * @file base.h
 * @copyright Copyright (C) 2024 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#ifndef ROKAEAPI_BASE_H
#define ROKAEAPI_BASE_H

#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif // defined(_MSC_VER) && (_MSC_VER >= 1200)

#include <memory>
#include <chrono>
#include <functional>
#include <system_error>
#include <set>

// *********************           Macros          ************************
// *********************           宏定义           ************************

/// @cond DO_NOT_DOCUMENT

#if defined(_MSC_VER)
    #if defined(XCORESDK_DLL_BUILD)
        #define XCORE_API __declspec(dllexport)
        #define XCORESDK_SUPPRESS_DLL_WARNING
    #elif defined(XCORESDK_DLL)
        #define XCORE_API __declspec(dllimport)
        #define XCORESDK_SUPPRESS_DLL_WARNING
    #endif
#else // if defined(_MSC_VER)
    #define XCORE_API __attribute__((visibility("default")))
#endif  // if defined(_MSC_VER)

#if !defined(XCORE_API)
#define XCORE_API
#endif

namespace rokae {
template<class T> struct Base { };

}

typedef std::error_code error_code;

#ifndef XCORESDK_DECLARE_IMPL
#define XCORESDK_DECLARE_IMPL \
protected: \
    class Impl; \
    Impl* impl_;
#endif

#ifndef XCORESDK_DECLARE_IMPLD
#define XCORESDK_DECLARE_IMPLD \
protected: \
    class Impld; \
    Impld* impld_;
#endif
/// @endcond

#endif //ROKAEAPI_BASE_H
