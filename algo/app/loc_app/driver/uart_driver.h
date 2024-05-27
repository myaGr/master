/**@file        uart_driver.h
 * @brief
 * @version     V0.1
 * @copyright   Copyright (c) 2022-2022  Guangzhou Asensing Techology Co.,Ltd.
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/02/17  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#include <stdint.h>
#include <iostream>
#include <thread>
#include <functional>

/**
 * Uart Driver parent class 
 */
class UartDriver
{
public:

  /**
   * @brief Construct a Uart Driver object
   * @param[in] uart - com port number string
   * @param[in] baudrate - com port bps
   */
  UartDriver(char* uart = NULL, uint32_t baudrate = 115200);

  /**
   * @brief Destructor a Uart Driver object
   */
  ~UartDriver();

  /**
   * @brief Set the uart port name, the method will override the Construct
   *        function setting,
   * @param[in] uart - com port number string
   */
  void set_Uart(char* uart);

  /**
   * @brief Set the uart port name, the method will override the Construct
   *        function setting.
   * @param[in] baudrate - com port bps
   */
  void set_Baudrate(uint32_t baudrate);
  
  /**
   * @brief Uart task thread start run
   */
  void start_Run(std::function<void(uint8_t* data, uint32_t len)> receive_data_cb);

  /**
  * @brief Uart task thread stop run
  */
  void stop_Run();

private:

  /**
   * @brief Uart running task
   */
  void task();

protected:
  /** member variable as uart name */
  char m_uart[128];

  /** member variable as uart bps */
  uint32_t m_baudrate;

  /** flag to control uart running task start/stop */
  bool b_running;

  /** uart running task id */
  std::thread m_thread;

  /** function to deal with received data */
  std::function<void(uint8_t* data, uint32_t len)> receive_data_func;
};

#if 0

class GnssUartDriver : public UartDriver
{
public:
  GnssUartDriver(char* uart = NULL, uint32_t baudrate = 115200);

  ~GnssUartDriver();

  virtual void receive(uint8_t* data, uint32_t length);

private:

};

class ImuUartDriver : public UartDriver
{
public:
  ImuUartDriver(char* uart = NULL, uint32_t baudrate = 115200);

  ~ImuUartDriver();

  virtual void receive(uint8_t* data, uint32_t length);

private:
};

#endif