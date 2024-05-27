/**@file        qx_ssr_client.h
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
#include "qxids_sdk.h"

class QXSSRClient
{
public:
  QXSSRClient(qxids_config_t* cfg = NULL, qxids_callbacks_t* cbs = NULL);
  ~QXSSRClient();
  void start_qx_ssr_service(void);
  void stop_qx_ssr_service(void);
  void inject_nmea_to_qx_ssr(qxids_char_t* buffer, qxids_size_t len);
  bool m_acc_auth_succ;
  bool m_cap_start_succ;
private:

  void qx_ssr_sdk_task();
  qxids_config_t m_sdkcfg;
  qxids_callbacks_t m_sdkcbs;
  const qxids_interface_t* sdk_hl;
  std::thread m_ssrsdkThread;
};

