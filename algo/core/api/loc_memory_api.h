/**@file        loc_memory_api.h
 * @brief       Location Memory Pool API Header File
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/10/30  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#ifndef __LOC_MEMORY_API_H__
#define __LOC_MEMORY_API_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/********************Memory Pool Manager Module API***************************/

/** Memory Pool Manager Structure */
typedef struct {
  uint8_t u_Init;
  void* pool_ctrl;
  uint8_t* pool_addr;
  uint32_t pool_size;
} loc_api_MemoryPoolManager_t;

/**
 * @brief Create a memory pool instance
 * @param[in] pointer to loc_api_MemoryPoolManager_t
 * @return TRUE - Success FALSE - Fail
 */
void loc_api_MemoryPoolCreate(loc_api_MemoryPoolManager_t* pz_MemoryPoolManager);

/**
 * @brief Destroy the memory pool instance
 * @param[in] pointer to loc_api_MemoryPoolManager_t
 * @return None
 */
void loc_api_MemoryPool_Destroy(loc_api_MemoryPoolManager_t* pz_MemoryPoolManager);

/**
 * @brief Malloc of a memory pool
 * @param[in] pointer to loc_api_MemoryPoolManager_t
 * @param[in] memory size
 * @return Pointer to memory
 */
void* loc_api_MemoryPool_Malloc(loc_api_MemoryPoolManager_t* pz_MemoryPoolManager, uint32_t size);

/**
 * @brief Free of a memory pool
 * @param[in] pointer to loc_api_MemoryPoolManager_t
 * @param[in] pointer need to free
 * @return
 */
void loc_api_MemoryPool_Free(loc_api_MemoryPoolManager_t* pz_MemoryPoolManager, void* ptr);

/********************End Memory Pool Manager Module API***********************/

#ifdef __cplusplus
}
#endif

#endif
