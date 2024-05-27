/**@file        ntrip_client.h
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

#include <stdint.h>
#include <string.h>

#include <thread>
#include <functional>

#include "loguru.h"

#ifndef __NTRIP_CLIENT_H__
#define __NTRIP_CLIENT_H__

typedef struct {
  char url[128];
  char mountpoint[128];
  char user[128];
  char passwd[128];
  uint32_t port;
} Ntrip_Account_t;

#define SOCKET_RECEIVE_BUFFER_SIZE (50*1024)

/** Socket status */
typedef enum {
  SOCKET_NONE = 0x00,
  SOCKET_INITIALZED,
  SOCKET_CONNECTING,
  SOCKET_CREATED,
  SOCKET_CONNECTED,
  SOCKET_RECEIVING,
  SOCKET_DISCONNECT,
  SOCKET_NB_RECV,
  SOCKET_MAX
} SockerStatus_EnumVal;
typedef uint8_t SocketStatus_e;

class NonblockingSocket
{
public:
  NonblockingSocket(std::function<void(uint8_t* data, uint32_t len)> recv_func = nullptr,
    std::function<void(uint8_t status, char* msg)> report_func = nullptr)
  {
    m_recv_func = recv_func;
    m_report_func = report_func;
    b_receive_enable = false;

    memset(m_url, 0, sizeof(m_url));
    memset(m_ip, 0, sizeof(m_ip));
    m_port = 0;
  }
  ~NonblockingSocket() { }

  virtual bool sk_connect(char* url, char* ip, int16_t port) = 0;
  virtual bool sk_close() = 0;
  virtual int32_t sk_send(uint8_t* data, uint32_t length) = 0;
  
  bool connectable();
  void resetConnectable();
  void release();

protected:
  virtual void connect_thread() = 0;
  virtual void receive_proc() = 0;
  
  void report_message(uint8_t status, const char* fmt, ...);

  char m_url[128];
  char m_ip[128];
  int16_t m_port;

  int64_t m_socket_fd;
  std::thread m_connect_thread;
  uint32_t m_ReConnectCount = 0;
  uint32_t m_ReConnectMax = 0;

  bool b_receive_enable;

  std::function<void(uint8_t* data, uint32_t len)> m_recv_func;
  std::function<void(uint8_t status, char* msg)> m_report_func;

  SocketStatus_e m_sk_status;
};


class NonblockingSocketWin : public NonblockingSocket
{
public:
  NonblockingSocketWin(std::function<void(uint8_t* data, uint32_t len)> recv_func,
    std::function<void(uint8_t status, char* msg)> report_func);
  ~NonblockingSocketWin() {};

  virtual bool sk_connect(char* url, char* ip, int16_t port);
  virtual bool sk_close();
  virtual int32_t sk_send(uint8_t* data, uint32_t length);

protected:

  virtual void connect_thread();
  virtual void receive_proc();
};

class NtripSocketUnix : public NonblockingSocket
{
public:
  NtripSocketUnix(std::function<void(uint8_t* data, uint32_t len)> recv_func,
    std::function<void(uint8_t status, char* msg)> report_func);
  ~NtripSocketUnix() {};

  virtual bool sk_connect(char* url, char* ip, int16_t port);
  virtual bool sk_close();
  virtual int32_t sk_send(uint8_t* data, uint32_t length);

protected:

  virtual void connect_thread();
  virtual void receive_proc();
};

/** Socket status */
typedef enum {
  NTRIP_NONE          = (uint32_t)0x00000000,
  NTRIP_INIT          = (uint32_t)0x00000001,
  NTRIP_SERVER_SET    = (uint32_t)0x00000002,
  NTRIP_MOUNT_SET     = (uint32_t)0x00000004,
  NTRIP_ACCOUNT_SET   = (uint32_t)0x00000008,
  NTRIP_ALL_ACC_SET   = (uint32_t)0x0000000E,
  NTRIP_CONNECTED     = (uint32_t)0x00000010,
  NTRIP_AUTHORIZED    = (uint32_t)0x00000020,
  NTRIP_GGA_REPORT    = (uint32_t)0x00000040,
} NtripStatus_EnumVal;
typedef uint32_t NtripStatus_e;

typedef struct {
  uint8_t enable;
  char url[256];
  char mountpoint[256];
  char user[256];
  char passwd[256];
  uint32_t port;
} NtripAccount_t;

class NtripClient
{
public:
  NtripClient(NtripAccount_t* account = NULL);
  ~NtripClient();

  void set_callback_receive_data(std::function<void(uint8_t* data, uint32_t len)> receive_data_cb);
  void set_callback_report_status(std::function<void(uint32_t status)> report_status_cb);

  void set_server(char* url, uint32_t len, uint32_t port);
  void set_mountpoint(char* mountpoint, uint32_t len);
  void set_account(char* user, uint32_t user_len, char* passwd, uint32_t passwd_len);
  bool get_enable();
  bool connect();
  bool authorized();
  void send_nmea(char* nmea, uint32_t len);
  void release();
  
private:

  void ntrip_task();
  uint32_t encbase64(uint8_t* str, const uint8_t* byte, uint32_t n);
  void socket_report(int32_t status, char* msg);
  void socket_receive(uint8_t* data, uint32_t len);

  std::function<void(uint8_t* data, uint32_t len)> m_receive_data_cb;
  std::function<void(uint32_t status)> m_report_status_cb;

  std::unique_ptr<NonblockingSocket> ntripSocket;

  NtripAccount_t m_Account;
  NtripStatus_e m_statusMask;

  bool m_bTaskEnable;
  std::thread m_ntripThread;
};

#endif
