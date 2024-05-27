/**@file        qx_ssr_client.cpp
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

#include "qx_ssr_client.h"
#include "loguru.h"
#include <stdarg.h>
#include <iostream>
#include <thread>

/*********************************************************************************/
using namespace std;

static QXSSRClient* s_client = NULL;

void qx_sdk_status_response (int status)
{
  LOG_F(INFO, "QX SDK status %d\n",status);
  switch (status) {
    case QXIDS_STATUS_CAP_START_SUCCESS:
      s_client->m_cap_start_succ = true;
      break;
    case QXIDS_STATUS_AUTH_SUCCESS:
      s_client->m_acc_auth_succ = true;
      break;
	default:
      break;
  }
  return;
}
 
QXSSRClient::QXSSRClient(qxids_config_t* cfg, qxids_callbacks_t* cbs)
{
  m_acc_auth_succ = false;
  m_cap_start_succ = false;
  sdk_hl = nullptr;
  memset(&m_sdkcfg,0,sizeof(qxids_config_t));
  memset(&m_sdkcbs,0,sizeof(qxids_callbacks_t));
  m_sdkcbs.size = sizeof(qxids_callbacks_t);
  m_sdkcbs.status_response = qx_sdk_status_response;
  if((cfg == NULL)||(cbs == NULL)){
	  return;
  }
  m_sdkcbs.fill_nssr_data = cbs->fill_nssr_data;
  memcpy(&m_sdkcfg, cfg, sizeof(qxids_config_t));
}

QXSSRClient::~QXSSRClient()
{
  m_acc_auth_succ = false;
  m_cap_start_succ = false;
}

void QXSSRClient::qx_ssr_sdk_task()
{
	int ret = -1;
	unsigned int i = 0;
    s_client = this;
    qxids_caps_info_t caps_info = { 0 };
	ret = sdk_hl->init(&m_sdkcbs, &m_sdkcfg);
	if (ret != 0) {
		LOG_F(INFO, "QX SDK init failed\n");
		return;
	}
	LOG_F(INFO, "QX SDK version: %s\n", sdk_hl->get_ver_info());
	while (m_acc_auth_succ == 0) {
		LOG_F(INFO, "waiting for auth succ...\n");
		  std::this_thread::sleep_for(std::chrono::seconds(1));
	}

	ret = sdk_hl->get_caps_info(&caps_info);
	if (ret == QXIDS_OK) {
		for (i = 0; i < caps_info.caps_num; i++) {
			LOG_F(INFO, "caps id: %04x\tcaps state: %d\texpire time: %lld\n", caps_info.caps[i].id,
				   caps_info.caps[i].state, caps_info.caps[i].expire_time);
		}
	}
	ret = sdk_hl->start();
	if (ret != 0) {
		LOG_F(INFO, "QX SDK start failed\n");
	}
	while (m_cap_start_succ != 1) {
        LOG_F(INFO, "waiting for capacities starting ...\n");
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
	LOG_F(INFO, "QX SDK start ok\n");
    return;
}

void QXSSRClient::start_qx_ssr_service()
{
#ifdef FEATURE_USE_QXWZ_SSR
  sdk_hl = get_qxids_interface(sizeof(qxids_interface_t));
#endif
  if (sdk_hl == NULL) {
		LOG_F(INFO, "Please update qxids_sdk.h file!\n");
		return;
  }
  m_ssrsdkThread = std::thread(&QXSSRClient::qx_ssr_sdk_task, this);
  return;
}

void QXSSRClient::inject_nmea_to_qx_ssr(qxids_char_t* buffer, qxids_size_t len)
{
  if(sdk_hl != NULL){
      sdk_hl->inject_gnss_data(buffer,len);
  }
}

void QXSSRClient::stop_qx_ssr_service()
{
  int ret = -1;
  s_client = NULL;

  m_ssrsdkThread.join();
  ret = sdk_hl->stop();
  if (ret == 0) {
    sdk_hl->cleanup();
    if (ret != 0) {
	  LOG_F(INFO, "QX SDK cleanup failed\n");
    }
  } else {
    LOG_F(INFO, "QX SDK stop failed\n");
  }
  sdk_hl = nullptr;
  return;
}
