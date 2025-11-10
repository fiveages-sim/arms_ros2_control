/**
 * @file exception.h
 * @brief 异常类
 * @copyright Copyright (C) 2024 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#ifndef ROKAEAPI_INCLUDE_ROKAE_EXCEPTION_H_
#define ROKAEAPI_INCLUDE_ROKAE_EXCEPTION_H_

#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif // defined(_MSC_VER) && (_MSC_VER >= 1200)

#include <string>
#include <stdexcept>

namespace rokae {

/**
 * @class Exception
 * @brief 运行中异常基类
 */
 class Exception : public std::exception {
  public:
   /**
    * @brief constructor
    * @param detail detail message.
    */
   explicit Exception(const std::string &detail);
   /**
    * @brief 异常信息
    */
   const char* what() const noexcept override;

  protected:
   const std::string detail_; ///< detail msg.
 };

/**
 * @class NetworkException
 * @brief 网络异常
 */
 class NetworkException final : public Exception {
  public:
   /**
    * @brief constructor
    * @param detail detail message
    */
   explicit NetworkException(const std::string &detail);
 };

/**
 * @class ArgumentException
 * @brief 参数错误异常
 */
 class ArgumentException : public Exception {
  public:
   /**
    * @brief constructor
    * @param detail detail message
    */
   explicit ArgumentException(const std::string &detail);
 };
/**
 * @class ExecutionException
 * @brief 操作执行失败异常
 */
 class ExecutionException : public Exception {
  public:
   /**
    * @brief constructor
    * @param detail detail message
    */
   explicit ExecutionException (const std::string &detail);
 };

/**
 * @class ProtocolException
 * @brief 解析控制器消息失败异常, 可能由于SDK版本与控制器版本不匹配
 */
 class ProtocolException final : public ExecutionException {
  public:
   /**
    * @brief constructor
    * @param detail detail message
    */
   explicit ProtocolException(const std::string &detail);
 };

/**
 * @class InvalidOperationException
 * @brief 操作被控制器拒绝
 */
 class InvalidOperationException final : public ExecutionException {
  public:
   /**
    * @brief constructor
    * @param detail detail message
    */
   explicit InvalidOperationException(const std::string &detail);
 };

/**
 * @class RealtimeControlException
 * @brief 实时模式错误
 */
 class RealtimeControlException : public Exception {
  public:
   /**
    * @brief constructor
    * @param detail detail message
    */
   explicit RealtimeControlException(const std::string &detail);
 };

/**
 * @class RealtimeMotionException
 * @brief 实时模式运动错误
 */
 class RealtimeMotionException final : public RealtimeControlException {
  public:
   /**
    * @brief constructor
    * @param detail detail message
    */
   explicit RealtimeMotionException(const std::string &detail);
 };

/**
 * @class RealtimeStateException
 * @brief 实时模式状态错误
 */
 class RealtimeStateException final : public RealtimeControlException {
  public:
   /**
    * @brief constructor
    * @param detail detail message
    */
   explicit RealtimeStateException(const std::string &detail);
 };
/**
 * @class RealtimeParameterException
 * @brief 实时模式参数错误
 */
 class RealtimeParameterException final : public RealtimeControlException {
  public:
   /**
    * @brief constructor
    * @param detail detail message
    */
   explicit RealtimeParameterException(const std::string &detail);
 };
}// namespace rokae

#endif //ROKAEAPI_INCLUDE_ROKAE_EXCEPTION_H_
