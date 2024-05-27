/**@file        nwstream_client.h
 * @brief
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/11/13  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#ifndef __NWSTREAM_CLIENT_H_
#define __NWSTREAM_CLIENT_H_

#include "ntrip_client.h"

#include <stdarg.h>
#include <iostream>
#include <thread>
#include <chrono>

class NwStreamClient
{
public:
  NwStreamClient(char* url, uint32_t len, uint32_t port);
  ~NwStreamClient();

  void set_callback_receive_data(std::function<void(uint8_t* data, uint32_t len)> receive_data_cb);

  void send(uint8_t* data, uint32_t length);
  bool connect();
  void release();

private:
  void nw_stream_task();
  void socket_report(int32_t status, char* msg);
  void socket_receive(uint8_t* data, uint32_t len);

  std::function<void(uint8_t* data, uint32_t len)> m_receive_data_cb;
  std::unique_ptr<NonblockingSocket> m_NwStreamSocket;

  bool     m_bThreadEnable;
  std::thread m_Thread;
  uint8_t  m_SocketStatus;

  char     m_server_url[256] = { 0 };
  uint32_t m_server_url_length = 0;
  uint32_t m_server_port = 0;
};


#endif
