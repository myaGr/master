/**@file        nwstream_client.cpp
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

#include "nwstream_client.h"

NwStreamClient::NwStreamClient(char* url, uint32_t len, uint32_t port)
{
  if (len > sizeof(m_server_url))
  {
    return;
  }

  m_receive_data_cb = nullptr;
  m_SocketStatus = SOCKET_NONE;

  memcpy(m_server_url, url, len);
  m_server_url_length = len;
  m_server_port = port;

  std::function<void(uint8_t status, char* msg)> socket_report_func = std::bind(&NwStreamClient::socket_report,
    this, std::placeholders::_1, std::placeholders::_2);
  std::function<void(uint8_t* data, uint32_t len)> socket_receive_func = std::bind(&NwStreamClient::socket_receive,
    this, std::placeholders::_1, std::placeholders::_2);

#if defined(_MSC_VER)
  m_NwStreamSocket = std::unique_ptr<NonblockingSocket>(new NonblockingSocketWin(socket_receive_func, socket_report_func));
#elif defined(__gnu_linux__)
  m_NwStreamSocket = std::unique_ptr<NonblockingSocket>(new NtripSocketUnix(socket_receive_func, socket_report_func));
#endif
}

void NwStreamClient::socket_report(int32_t status, char* msg)
{
  m_SocketStatus = status;
  return;
}

void NwStreamClient::socket_receive(uint8_t* data, uint32_t len)
{
  if (nullptr != m_receive_data_cb)
  {
    m_receive_data_cb(data, len);
  }

  return;
}


NwStreamClient::~NwStreamClient()
{

}


void NwStreamClient::set_callback_receive_data(std::function<void(uint8_t* data, uint32_t len)> receive_data_cb)
{
  m_receive_data_cb = receive_data_cb;
  return;
}

void NwStreamClient::send(uint8_t* data, uint32_t length)
{

}

void NwStreamClient::nw_stream_task()
{
  m_bThreadEnable = true;

  while (m_bThreadEnable)
  {
    m_SocketStatus = SOCKET_NONE;
    if (m_SocketStatus != SOCKET_CONNECTED)
    {
      if (m_NwStreamSocket->connectable())
      {
        m_NwStreamSocket->sk_connect((char*)m_server_url, NULL, m_server_port);
      }
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  return;
}

bool NwStreamClient::connect()
{
  m_Thread = std::thread(&NwStreamClient::nw_stream_task, this);

  return false;
}

void NwStreamClient::release()
{
  m_bThreadEnable = false;
  m_SocketStatus = NTRIP_NONE;
  m_Thread.join();
}