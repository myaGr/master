/**@file        ntrip_client.cpp
 * @brief
 * @version     V0.1
 * @copyright   Copyright (c) 2022-2022  Guangzhou Asensing Techology Co.,Ltd.
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/02/19  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#include "ntrip_client.h"

#include <stdarg.h>
#include <iostream>
#include <thread>
#include <chrono>


template<class T>
inline static bool bit_is_set(T& x, T bit)
{
  return ((x & bit) == bit);
}

template<class T>
inline static void set_bit(T& x, T bit)
{
  x |= bit;
}

template<class T>
inline static void clear_bit(T& x, T bit)
{
  x &= ~bit;
}

void NonblockingSocket::report_message(uint8_t status, const char* fmt, ...)
{
#define REPORT_MESSAGE_MAX (128)

  uint32_t n = 0;
  char buffer[REPORT_MESSAGE_MAX] = { 0 };

  va_list ap;
  va_start(ap, fmt);
  if (n < REPORT_MESSAGE_MAX)
  {
    n += vsnprintf(buffer + n, REPORT_MESSAGE_MAX - 1 - n, fmt, ap);
  }
  va_end(ap);

  if (this->m_report_func != nullptr)
  {
    this->m_report_func(status, buffer);
  }
}

void NonblockingSocket::resetConnectable()
{
  m_ReConnectCount = 0;
  m_ReConnectMax = 0;
}

bool NonblockingSocket::connectable()
{
  if (m_sk_status == SOCKET_CONNECTING ||
    m_sk_status == SOCKET_CONNECTED ||
    m_sk_status == SOCKET_RECEIVING)
  {
    return false;
  }

  m_ReConnectCount++;
  if (m_ReConnectCount < m_ReConnectMax)
  {
    return false;
  }

  m_ReConnectMax = 2 * m_ReConnectCount;
  return true;
}

void NonblockingSocket::release()
{
  m_sk_status = SOCKET_INITIALZED;
  b_receive_enable = false;
  m_connect_thread.join();
}

/*********************************************************************************/
#if defined(_MSC_VER)

#include <winsock2.h>

#pragma comment(lib, "ws2_32.lib")

NonblockingSocketWin::NonblockingSocketWin(std::function<void(uint8_t* data, uint32_t len)> recv_func,
  std::function<void(uint8_t status, char* msg)> report_func) :
  NonblockingSocket(recv_func, report_func)
{
  m_sk_status = SOCKET_NONE;

  WSADATA ws;
  if (ERROR_SUCCESS != WSAStartup(MAKEWORD(2, 2), &ws))
  {
    report_message(m_sk_status, "socket init fail, error %d", GetLastError());
    return;
  }

  m_sk_status = SOCKET_INITIALZED;

  report_message(m_sk_status, "socket init");
}


void NonblockingSocketWin::connect_thread()
{
  int ret = -1;
  int error_code = ERROR_SUCCESS;

  if (m_sk_status != SOCKET_CONNECTING)
  {
    report_message(m_sk_status, "try to connect fail as to socket is not connecting status");
    return;
  }
  
  m_socket_fd = socket(AF_INET, SOCK_STREAM, 0);
  if (INVALID_SOCKET == m_socket_fd)
  {
    m_sk_status = SOCKET_INITIALZED;
    report_message(m_sk_status, "try to connect but socket create faild, error %d", 
      GetLastError());
    return;
  }

  unsigned long cmd = 1;
  ret = ioctlsocket((SOCKET)m_socket_fd, FIONBIO, &cmd);
  if (INVALID_SOCKET == ret)
  {
    m_sk_status = SOCKET_INITIALZED;
    report_message(m_sk_status, "try to connect but socket create faild, error %d", 
      GetLastError());
    return;
  }

  m_sk_status = SOCKET_CREATED;

  if (strlen(m_url) > 0)
  {
    struct hostent* hptr;
    hptr = gethostbyname(m_url);
    if (NULL == hptr)
    {
      m_sk_status = SOCKET_INITIALZED;
      report_message(m_sk_status, "try to connect but gethostbyname(%s) faild, error %d",
        m_url, GetLastError());
      return;
    }

    char* ip = inet_ntoa(*(struct in_addr*)hptr->h_addr_list[0]);
    memcpy(m_ip, ip, strlen(ip));
    
    report_message(m_sk_status, "try to connect url %s ip %s port %d", 
      m_url, m_ip, m_port);
  }

  if (strlen(m_ip) > 0)
  {
    LOG_F(INFO,"Ntrip/Info use ip %s port %d", m_ip, m_port);
  }
  else
  {
    m_sk_status = SOCKET_INITIALZED;
    report_message(m_sk_status, "try to connect url and ip are invalid");
    return;
  }

  uint32_t connect_count = 0;

  struct sockaddr_in addr;
  addr.sin_family = AF_INET;
  addr.sin_port = htons(m_port);
  addr.sin_addr.s_addr = inet_addr(m_ip);
  while (true)
  {
    ret = connect((SOCKET)m_socket_fd, (sockaddr*)&addr, sizeof(addr));
    error_code = GetLastError();
    if (WSAEWOULDBLOCK == error_code || WSAEINVAL == error_code)
    {
      connect_count++;
      if (connect_count == 100)
      {
        m_sk_status = SOCKET_INITIALZED;
        report_message(m_sk_status, "try to connect fail as for connect %d time, error %d", 
          connect_count, error_code);
        break;
      }
      report_message(m_sk_status, "try to connect server %d time", connect_count);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));

      continue;
    }
    else if (WSAEISCONN == error_code)
    {
      m_sk_status = SOCKET_CONNECTED;
      report_message(m_sk_status, "Connect Success, ready connect");
      break;
    }
    else
    {
      m_sk_status = SOCKET_INITIALZED;
      report_message(m_sk_status, "Connect fail, error code %d", error_code);
      break;
    }
  }

  if (SOCKET_CONNECTED == m_sk_status)
  {
    m_sk_status = SOCKET_CONNECTED;
    report_message(m_sk_status, "Connect to server succ, Start to receive data");
    
    resetConnectable();

    receive_proc();
  }
  else
  {
    m_sk_status = SOCKET_INITIALZED;
    report_message(m_sk_status, "Connect to server fail, exit");
  }

  return;
}

void NonblockingSocketWin::receive_proc()
{
  char receive_buffer[SOCKET_RECEIVE_BUFFER_SIZE] = { 0 };
  int32_t length = 0;
  int error_code = ERROR_SUCCESS;

  b_receive_enable = true;
  int32_t recv_miss_count = 0;

  m_sk_status = SOCKET_RECEIVING;
 
  while (b_receive_enable)
  {
    length = recv((SOCKET)m_socket_fd, (char*)receive_buffer, sizeof(receive_buffer), 0);

    if (length > 0)
    {
      recv_miss_count = 0;
      this->m_recv_func((uint8_t*)receive_buffer, length);
      continue;
    }

    error_code = GetLastError();
    if (WSAEWOULDBLOCK == error_code)
    {
      recv_miss_count++;
      if (recv_miss_count < 120) /* can not receive data for 2min */
      {
        report_message(SOCKET_NB_RECV, "Socket nonblocking receive miss %d times", recv_miss_count);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        continue;
      }
      else
      {
        report_message(SOCKET_NB_RECV, "Socket nonblocking receive miss too much and close socket");
        b_receive_enable = false;
      }
    }
    else
    {
      report_message(SOCKET_NB_RECV, "Socket nonblocking receive fail error %d", error_code);
      b_receive_enable = false;
    }
  }

  std::this_thread::sleep_for(std::chrono::seconds(1));
  sk_close();

  if (m_sk_status == SOCKET_RECEIVING)
  {
    m_sk_status = SOCKET_DISCONNECT;
    report_message(m_sk_status, "Socket disconnect as to receive fail");
  }
}

bool NonblockingSocketWin::sk_connect(char* url, char* ip, int16_t port)
{
  if (m_sk_status == SOCKET_NONE)
  {
    report_message(m_sk_status, "socket is un-initialzed");
  }

  if (NULL != url && strlen(url) < sizeof(m_url))
  {
    memcpy(m_url, url, strlen(url));
  }

  if (NULL != ip && strlen(ip) < sizeof(m_ip))
  {
    memcpy(m_ip, ip, strlen(ip));
  }

  m_port = port;

  if (m_connect_thread.joinable())
  {
    report_message(m_sk_status, "socket connect thread is joinable");
    m_connect_thread.join();
  }

  report_message(m_sk_status, "start to nonblocking connect..");

  m_sk_status = SOCKET_CONNECTING;
  m_connect_thread = std::thread(&NonblockingSocketWin::connect_thread, this);

  return true;
}

bool NonblockingSocketWin::sk_close()
{
  if (0 > closesocket((SOCKET)m_socket_fd))
  {
    report_message(m_sk_status, "Socket close fail, error %d", GetLastError());
    return false;
  }

  b_receive_enable = false;
  return true;
}

int32_t NonblockingSocketWin::sk_send(uint8_t* data, uint32_t length)
{
  if (SOCKET_RECEIVING != m_sk_status && SOCKET_CONNECTED != m_sk_status)
  {
    return 0;
  }

  int send_len = send((SOCKET)m_socket_fd, (char*)data, length, 0);
  report_message(SOCKET_NONE, "Socket send %d", length);

  return send_len;
}


#endif

#if defined(__gnu_linux__)

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <netdb.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <sys/epoll.h>


NtripSocketUnix::NtripSocketUnix(std::function<void(uint8_t* data, uint32_t len)> recv_func,
  std::function<void(uint8_t status, char* msg)> report_func) :
  NonblockingSocket(recv_func, report_func)
{
  m_sk_status = SOCKET_INITIALZED;
  report_message(m_sk_status, "socket init");
}


void NtripSocketUnix::connect_thread()
{
  int ret = -1;
  if (m_sk_status != SOCKET_CONNECTING)
  {
    report_message(m_sk_status, "try to connect fail as to socket is not connecting status");
    return;
  }

  m_socket_fd = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (m_socket_fd < 0)
  {
    m_sk_status = SOCKET_INITIALZED;
    report_message(m_sk_status, "try to connect but socket create faild, errno %d %s", errno, strerror(errno));
    return;
  }

  /* set socket as non-blocking */
  int flags = fcntl(m_socket_fd, F_GETFL, 0);
  fcntl(m_socket_fd, F_SETFL, flags | O_NONBLOCK);

  m_sk_status = SOCKET_CREATED;

  if (strlen(m_url) > 0)
  {
    struct hostent* hptr;
    hptr = gethostbyname(m_url);
    if (NULL == hptr)
    {
      m_sk_status = SOCKET_INITIALZED;
      report_message(m_sk_status, "try to connect but gethostbyname(%s) faild, errno %d %s",
        m_url, errno, strerror(errno));
      return;
    }

    char* ip = inet_ntoa(*(struct in_addr*)hptr->h_addr_list[0]);
    memcpy(m_ip, ip, strlen(ip));

    report_message(m_sk_status, "try to connect url %s ip %s port %d",
      m_url, m_ip, m_port);
  }

  if (strlen(m_ip) > 0)
  {
    LOG_F(INFO,"Ntrip/Info use ip %s port %d\n", m_ip, m_port);
  }
  else
  {
    m_sk_status = SOCKET_INITIALZED;
    report_message(m_sk_status, "try to connect url and ip are invalid");
    return;
  }

  uint32_t connect_count = 0;
  struct sockaddr_in addr;
  addr.sin_family = AF_INET;
  addr.sin_port = htons(m_port);
  addr.sin_addr.s_addr = inet_addr(m_ip);

  while (true)
  {
    ret = connect(m_socket_fd, (sockaddr*)&addr, sizeof(addr));
    if (ret == 0)
    {
      m_sk_status = SOCKET_CONNECTED;
      report_message(m_sk_status, "try to connect success, errno %d %s", errno, strerror(errno));
      break;
    }

    if (EINPROGRESS == errno || EALREADY == errno)
    {
      connect_count++;
      if (connect_count == 100)
      {
        m_sk_status = SOCKET_INITIALZED;
        report_message(m_sk_status, "try to connect fail as for connect %d time, errno %d %s",
          errno, strerror(errno));
        break;
      }
      report_message(m_sk_status, "try to connect server %d time", connect_count);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));

      continue;
    }
    else if (EISCONN == errno)
    {
      m_sk_status = SOCKET_CONNECTED;
      report_message(m_sk_status, "Connect Success, ready connect");
      break;
    }
    else
    {
      m_sk_status = SOCKET_INITIALZED;
      report_message(m_sk_status, "Connect fail, error code errno %d %s", errno, strerror(errno));
      break;
    }
  }

  if (SOCKET_CONNECTED == m_sk_status)
  {
    m_sk_status = SOCKET_CONNECTED;
    report_message(m_sk_status, "Connect to server succ, Start to receive data");

    resetConnectable();

    receive_proc();
  }
  else
  {
    m_sk_status = SOCKET_INITIALZED;
    report_message(m_sk_status, "Connect to server fail, exit");
  }

  return;
}


void NtripSocketUnix::receive_proc()
{
  char receive_buffer[SOCKET_RECEIVE_BUFFER_SIZE] = { 0 };
  int32_t length = 0;

  b_receive_enable = true;
  int32_t recv_miss_count = 0;

  m_sk_status = SOCKET_RECEIVING;

  while (b_receive_enable)
  {
    length = recv(m_socket_fd, (char*)receive_buffer, sizeof(receive_buffer), 0);

    if (length > 0)
    {
      recv_miss_count = 0;
      report_message(m_sk_status, "Socket receive %d", length);
      this->m_recv_func((uint8_t*)receive_buffer, length);
      continue;
    }

    if (EAGAIN == errno)
    {
      recv_miss_count++;
      if (recv_miss_count < 10) /* can not receive data for 2min */
      {
        report_message(SOCKET_NB_RECV, "Socket nonblocking receive miss %d times", recv_miss_count);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        continue;
      }
      else
      {
        report_message(SOCKET_NB_RECV, "Socket nonblocking receive miss too much and close socket");
        b_receive_enable = false;
      }
    }
    else
    {
      report_message(SOCKET_NB_RECV, "Socket nonblocking receive fail errno %d %s\n", errno, strerror(errno));
      b_receive_enable = false;
    }
  }

  std::this_thread::sleep_for(std::chrono::seconds(1));
  sk_close();

  m_sk_status = SOCKET_DISCONNECT;
  report_message(m_sk_status, "Socket disconnect as to receive fail");
}

bool NtripSocketUnix::sk_connect(char* url, char* ip, int16_t port)
{
  if (m_sk_status == SOCKET_NONE)
  {
    report_message(m_sk_status, "socket is un-initialzed");
  }

  if (NULL != url && strlen(url) < sizeof(m_url))
  {
    memcpy(m_url, url, strlen(url));
  }

  if (NULL != ip && strlen(ip) < sizeof(m_ip))
  {
    memcpy(m_ip, ip, strlen(ip));
  }

  m_port = port;

  if (m_connect_thread.joinable())
  {
    report_message(m_sk_status, "socket connect thread is joinable");
    m_connect_thread.join();
  }

  report_message(m_sk_status, "start to nonblocking connect..");

  m_sk_status = SOCKET_CONNECTING;
  m_connect_thread = std::thread(&NtripSocketUnix::connect_thread, this);

  return true;
}

bool NtripSocketUnix::sk_close()
{
  if (0 > close(m_socket_fd))
  {
    report_message(m_sk_status, "Socket close fail, errno %d %s", errno, strerror(errno));
    return false;
  }

  b_receive_enable = false;
  return true;
}

int32_t NtripSocketUnix::sk_send(uint8_t* data, uint32_t length)
{
  if (SOCKET_RECEIVING != m_sk_status && SOCKET_CONNECTED != m_sk_status)
  {
    return 0;
  }

  int send_len = send(m_socket_fd, (char*)data, length, 0);
  report_message(SOCKET_NONE, "Socket send %d", length);

  return send_len;
}



#endif

/*********************************************************************************/

NtripClient::NtripClient(NtripAccount_t* acc)
{
  m_bTaskEnable = false;
  m_receive_data_cb = nullptr;
  m_report_status_cb = nullptr;

  set_bit<uint32_t>(m_statusMask, NTRIP_INIT);
  memcpy(&m_Account, acc, sizeof(m_Account));
  
  if (strlen(m_Account.url) > 0 && strlen(m_Account.url) < sizeof(m_Account.url)) {
    set_bit<uint32_t>(m_statusMask, NTRIP_SERVER_SET);
  }
  if (strlen(m_Account.mountpoint) > 0 && strlen(m_Account.mountpoint) < sizeof(m_Account.mountpoint)) {
    set_bit<uint32_t>(m_statusMask, NTRIP_MOUNT_SET);
  }
  if ((strlen(m_Account.user) > 0 && strlen(m_Account.user) < sizeof(m_Account.user)) &&
    (strlen(m_Account.passwd) > 0 && strlen(m_Account.passwd) < sizeof(m_Account.passwd))) {
    set_bit<uint32_t>(m_statusMask, NTRIP_ACCOUNT_SET);
  }

  std::function<void(uint8_t status, char* msg)> report_func = std::bind(&NtripClient::socket_report,
    this, std::placeholders::_1, std::placeholders::_2);
  std::function<void(uint8_t* data, uint32_t len)> recv_func = std::bind(&NtripClient::socket_receive,
    this, std::placeholders::_1, std::placeholders::_2);

#if defined(_MSC_VER)
  ntripSocket = std::unique_ptr<NonblockingSocket>(new NonblockingSocketWin(recv_func, report_func));
#elif defined(__gnu_linux__)
  ntripSocket = std::unique_ptr<NonblockingSocket>(new NtripSocketUnix(recv_func, report_func));
#endif
}

NtripClient::~NtripClient()
{
  m_bTaskEnable = false;
  m_receive_data_cb = nullptr;
  set_bit<uint32_t>(m_statusMask, NTRIP_INIT);
  ntripSocket->release();
}

void NtripClient::set_callback_receive_data(std::function<void(uint8_t* data, uint32_t len)> receive_data_cb)
{
  m_receive_data_cb = receive_data_cb;
}

void NtripClient::set_callback_report_status(std::function<void(uint32_t status)> report_status_cb)
{
  m_report_status_cb = report_status_cb;
}

void NtripClient::release()
{
  m_bTaskEnable = false;
  m_statusMask = NTRIP_NONE;
  m_ntripThread.join();
}

void NtripClient::socket_receive(uint8_t* data, uint32_t len)
{
  if (bit_is_set<uint32_t>(m_statusMask, NTRIP_INIT))
  {
    // Nothing to do
  }

  if (bit_is_set<uint32_t>(m_statusMask, NTRIP_ALL_ACC_SET))
  {
    // Nothing to do
  }

  if (bit_is_set<uint32_t>(m_statusMask, NTRIP_CONNECTED))
  {
    if (NULL != strstr((char*)data, "ICY 200 OK"))
    {
      set_bit<uint32_t>(m_statusMask, NTRIP_AUTHORIZED);
      if (m_report_status_cb != nullptr)
      {
        m_report_status_cb(m_statusMask);
      }
    }
  }

  if ((bit_is_set<uint32_t>(m_statusMask, NTRIP_AUTHORIZED)) ||
    (bit_is_set<uint32_t>(m_statusMask, NTRIP_GGA_REPORT)))
  {
    if (m_receive_data_cb != nullptr)
    {
      m_receive_data_cb(data, len);
    }
  }

  return;
}

void NtripClient::socket_report(int32_t status, char* msg)
{
  if (SOCKET_NB_RECV != status)
  {
    LOG_F(INFO,"%s", msg);
  }

  if (SOCKET_CONNECTED == status)
  {
    set_bit<uint32_t>(m_statusMask, NTRIP_CONNECTED);
  }

  if (SOCKET_CONNECTED == status || SOCKET_RECEIVING == status)
  {
    uint8_t buffer[4 * 1024] = { 0 };
    uint32_t buffer_len = 0;

    uint8_t user_buffer[256] = { 0 };
    uint32_t user_buffer_len = 0;

    uint8_t base64_buffer[1024] = { 0 };
    uint32_t base64_buffer_len = 0;

    user_buffer_len += sprintf((char*)user_buffer, "%s:%s", m_Account.user, m_Account.passwd);
    base64_buffer_len += encbase64(base64_buffer, user_buffer, strlen((char*)user_buffer));
    buffer_len += sprintf((char*)buffer + buffer_len, "GET /%s HTTP/1.1\r\n", m_Account.mountpoint);
    buffer_len += sprintf((char*)buffer + buffer_len, "User-Agent: NTRIP %s\r\n", "VLT");
    buffer_len += sprintf((char*)buffer + buffer_len, "Accept: */*\r\n");
    buffer_len += sprintf((char*)buffer + buffer_len, "Connection: close\r\n");
    buffer_len += sprintf((char*)buffer + buffer_len, "Authorization: Basic %s\r\n\r\n", base64_buffer);

    ntripSocket->sk_send(buffer, buffer_len);
  }
  else if (SOCKET_DISCONNECT == status || SOCKET_INITIALZED == status)
  {
    clear_bit<uint32_t>(m_statusMask, NTRIP_CONNECTED);
    clear_bit<uint32_t>(m_statusMask, NTRIP_AUTHORIZED);
    clear_bit<uint32_t>(m_statusMask, NTRIP_GGA_REPORT);
  }

  return;
}

void NtripClient::set_server(char* url, uint32_t len, uint32_t port)
{
  if ((url == NULL) || (strlen(url) > len))
  {
    return;
  }

  memcpy(m_Account.url, url, len);
  m_Account.port = port;
  set_bit<uint32_t>(m_statusMask, NTRIP_SERVER_SET);
}

void NtripClient::set_mountpoint(char* mountpoint, uint32_t len)
{
  if ((mountpoint == NULL) || (strlen(mountpoint) > len))
  {
    return;
  }

  memcpy(m_Account.mountpoint, mountpoint, len);
  set_bit<uint32_t>(m_statusMask, NTRIP_MOUNT_SET);
}

void NtripClient::set_account(char* user, uint32_t user_len, char* passwd, uint32_t passwd_len)
{
  if ((user == NULL) || (strlen(user) > user_len))
  {
    return;
  }

  if ((passwd == NULL) || (strlen(passwd) > passwd_len))
  {
    return;
  }

  memcpy(m_Account.user, user, user_len);
  memcpy(m_Account.passwd, passwd, passwd_len);
  
  set_bit<uint32_t>(m_statusMask, NTRIP_ACCOUNT_SET);
}

void NtripClient::ntrip_task()
{
  m_bTaskEnable = true;

  LOG_F(INFO,"Start ntrip task to connecting");

  while (m_bTaskEnable)
  {
    if (!bit_is_set<uint32_t>(m_statusMask, NTRIP_CONNECTED))
    {
      clear_bit<uint32_t>(m_statusMask, NTRIP_AUTHORIZED);
      clear_bit<uint32_t>(m_statusMask, NTRIP_GGA_REPORT);

      if (ntripSocket->connectable())
      {
        ntripSocket->sk_connect((char*)m_Account.url, NULL, m_Account.port);
      }
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  return;
}

bool NtripClient::get_enable()
{
  return m_bTaskEnable;
}

bool NtripClient::connect()
{
  if (!bit_is_set<uint32_t>(m_statusMask, NTRIP_ALL_ACC_SET))
  {
    LOG_F(INFO, "Ntrip/Error connect is fail as to account is error 0x%08x", m_statusMask);
    return false;
  }

  m_ntripThread = std::thread(&NtripClient::ntrip_task, this);

  return true;
}

void NtripClient::send_nmea(char* nmea, uint32_t len)
{
  const char* str = "\r\n\r\n";
  char buffer[256] = { 0 };
  char* p = NULL;
  uint32_t sz = (uint32_t)sizeof(buffer);

  if (!bit_is_set<uint32_t>(m_statusMask, NTRIP_AUTHORIZED))
  {
    LOG_F(INFO, "Ntrip/Error authorized is fail. status 0x%x", m_statusMask);
    return;
  }

  if (len > sz)
  {
    LOG_F(INFO, "Ntrip/Error report nmea fail, len %d > size %d", len, sz);
    return;
  }

  memcpy(buffer, nmea, len);

  if (NULL == strstr(buffer, "GGA"))
  {
    LOG_F(INFO, "Ntrip/Error report nmea fail, nmea no GGA, %s", buffer);
    return;
  }

  p = strstr(buffer, "*");
  if (NULL == p)
  {
    LOG_F(INFO, "Ntrip/Error report nmea fail, nmea no checksum, %s", buffer);
    return;
  }

  /* Append '\r\n\r\n' to meet Ntrip server requirement */
  memcpy(p + 3, str, strlen(str));
  
  ntripSocket->sk_send((uint8_t*)buffer, strlen(buffer));

  set_bit<uint32_t>(m_statusMask, NTRIP_GGA_REPORT);
}


uint32_t NtripClient::encbase64(uint8_t* str, const uint8_t* byte, uint32_t n)
{
  const char table[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  uint32_t i, j, k, b;

  for (i = j = 0; i / 8 < n;) {
    for (k = b = 0; k < 6; k++, i++) {
      b <<= 1; if (i / 8 < n) b |= (byte[i / 8] >> (7 - i % 8)) & 0x1;
    }
    str[j++] = table[b];
  }
  while (j & 0x3) str[j++] = '=';
  str[j] = '\0';
  return j;
}

bool NtripClient::authorized()
{
  return bit_is_set<uint32_t>(m_statusMask, NTRIP_AUTHORIZED);
}
