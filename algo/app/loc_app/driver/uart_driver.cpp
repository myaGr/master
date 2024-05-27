/**@file        uart_driver.cpp
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

#include "uart_driver.h"

/* C Language header file */
#include <string.h>
#include <stdio.h>
#include <stdint.h>

/* Cpp Language header file */
#include <iostream>
#include <string>

/* project internal header file */
#include "serial.h"
#include "loguru.h"

#include "loc_core_api.h"

using namespace std;

/**
 * @brief Construct a Uart Driver object
 * @param[in] uart - com port number string
 * @param[in] baudrate - com port bps.
 */
UartDriver::UartDriver(char* uart, uint32_t baudrate)
{
  set_Uart(uart);
  set_Baudrate(baudrate);
  b_running = true;
  receive_data_func = nullptr;
}

/**
 * @brief Destructor a Uart Driver object
 */
UartDriver::~UartDriver()
{

}

/**
 * @brief Set the uart port name, the method will override the Construct
 *        function setting,
 * @param[in] uart - com port number string
 */
void UartDriver::set_Uart(char* uart)
{
  if (uart == NULL)
  {
    return;
  }

  if (strlen(uart) > sizeof(m_uart))
  {
    return;
  }

  strcpy(m_uart, uart);
}

/**
 * @brief Set the uart port name, the method will override the Construct
 *        function setting.
 * @param[in] baudrate - com port bps
 */
void UartDriver::set_Baudrate(uint32_t baudrate)
{
  m_baudrate = baudrate;
}

/**
 * @brief Uart running task
 */
void UartDriver::task()
{
  std::string port(m_uart);
  serial::Timeout to(1, 100, 1, 0, 0);
  serial::Serial thisSerialObj(port, m_baudrate, to);
  if (thisSerialObj.isOpen())
  {
    LOG_F(INFO, "Serial port %s baudrate %d open succ", m_uart, m_baudrate);
  }
  else
  {
    LOG_F(ERROR, "Serial port %s baudrate %d open fail", m_uart, m_baudrate);
    return;
  }

  uint8_t recv_buf[1024];
  size_t recv_len = 0;
  while (b_running) {

    recv_len = thisSerialObj.read(recv_buf, 1024);
    if (nullptr != receive_data_func)
    {
      receive_data_func(recv_buf, recv_len);
    }
  }
  
  LOG_F(INFO, "Serial port %s is closing", m_uart);
  thisSerialObj.close();
}

void UartDriver::start_Run(std::function<void(uint8_t* data, uint32_t len)> receive_data_cb)
{
  receive_data_func = receive_data_cb;

  m_thread = std::thread(&UartDriver::task, this);
  return;
}

void UartDriver::stop_Run()
{
  this->b_running = false;
  this->m_thread.join();
  return;
}

#if 0
/****************************************************************************/

GnssUartDriver::GnssUartDriver(char* uart, uint32_t baudrate)
  : UartDriver(uart, baudrate)
{
  return;
}

GnssUartDriver::~GnssUartDriver()
{
  stop_Run();
  return;
}

void GnssUartDriver::receive(uint8_t* data, uint32_t length)
{
  LOG_F(DEBUG, "Receive GNSS Uart data length %d", length);
  loc_api_InjectRcvMeasRTCM(data, length);
  return;
}

/****************************************************************************/

ImuUartDriver::ImuUartDriver(char* uart, uint32_t baudrate)
  : UartDriver(uart, baudrate)
{

}

ImuUartDriver::~ImuUartDriver()
{
  stop_Run();
  return;
}

void ImuUartDriver::receive(uint8_t* data, uint32_t length)
{
  LOG_F(DEBUG, "Receive IMU Uart data length %d", length);
  return;
}

#endif
