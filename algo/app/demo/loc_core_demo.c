/**@file        loc_core_demo.c
 * @brief       Location engine usage demo file
 * @version     V1.0
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/04/19  <td>1.0      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#include "loc_core_api.h"

/**
 * @brief Report loc core output log
 * @param[in] data - log buffer
 * @param[in] length - log length
 * @return      None
 */
static void loc_core_callback_report_log_handler(uint8_t* data, uint32_t length)
{
  /* transmit log output */
  return;
}

/**
 * @brief Report loc core navigation solution
 * @param[in] location - navigation solution result
 * @return      None
 */
static void loc_core_callback_report_location_handler(loc_api_location_report_t* location)
{
  /* handle GNSS fix report */
  return;
}

/**
 * @brief location core memory free callback
 * @param[in]
 * @return      None
 */
static uint64_t loc_core_callback_get_tick_ms()
{
  uint64_t tick_ms = 0;
  /* return system tick ms */
  return tick_ms;
}

int main(int argc, char* argv[])
{
  /** Register callback functions */
  loc_api_callback_t loc_api_callback = { 0 };

  loc_api_callback.report_log = loc_core_callback_report_log_handler;
  loc_api_callback.report_location = loc_core_callback_report_location_handler;
  loc_api_callback.get_tick_ms = loc_core_callback_get_tick_ms;
  loc_api_Register_Callback(&loc_api_callback);

  /** Register memory pool */
#define LOC_CORE_MEMORY_SIZE (1024 * 1024 * 2)

  loc_api_MemoryRegister_t z_MemoryRegister = { 0 };
  static uint8_t memory_pool0[LOC_CORE_MEMORY_SIZE] = { 0 };
  z_MemoryRegister.u_type = LOC_API_MEMORY_EXT_API;
  z_MemoryRegister.pool_addr[0] = &(memory_pool0[0]);
  z_MemoryRegister.pool_size[0] = sizeof(memory_pool0);
  z_MemoryRegister.pool_count = 1;
  z_MemoryRegister.alloc = NULL;
  z_MemoryRegister.free = NULL;
  loc_api_Register_MemoryPool(&z_MemoryRegister);

  /** Register configuration */
  loc_api_config_para_group_t z_loc_api_config = { 0 };
  z_loc_api_config.para[0].t_mask = 0x0027; /* 1Hz */
  z_loc_api_config.para[1].t_mask = 0x0007;
  z_loc_api_config.para[2].t_mask = 0x0000;
  z_loc_api_config.para[3].t_mask = 0x0001;
  z_loc_api_config.para[4].t_mask = 0x0001;
  z_loc_api_config.para[5].t_mask = 0x0000;
  z_loc_api_config.para[6].t_mask = 0x01a5;
  z_loc_api_config.para[7].t_mask = 0x0002;
  loc_api_Register_Config(&z_loc_api_config);

  /** loc core init */
  loc_api_Initialize();

  uint8_t data[2048] = { 0 };
  uint32_t data_length = 0;

  while (1)
  {
    /* GNSS Device RTCM data receive */
    if (1)
    {
      loc_api_InjectRcvMeasRTCM(data, data_length);
    }

    /* GNSS Reference RTCM data receive */
    if (1)
    {
      loc_api_InjectRefCorrRTCM(data, data_length);
    }
  }

  return 0;
}
