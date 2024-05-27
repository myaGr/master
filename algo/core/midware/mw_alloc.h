#ifndef INCLUDED_tlsf
#define INCLUDED_tlsf

/*
** Two Level Segregated Fit memory allocator, version 3.1.
** Written by Matthew Conte
**	http://tlsf.baisoku.org
**
** Based on the original documentation by Miguel Masmano:
**	http://www.gii.upv.es/tlsf/main/docs
**
** This implementation was written to the specification
** of the document, therefore no GPL restrictions apply.
** 
** Copyright (c) 2006-2016, Matthew Conte
** All rights reserved.
** 
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
**     * Redistributions of source code must retain the above copyright
**       notice, this list of conditions and the following disclaimer.
**     * Redistributions in binary form must reproduce the above copyright
**       notice, this list of conditions and the following disclaimer in the
**       documentation and/or other materials provided with the distribution.
**     * Neither the name of the copyright holder nor the
**       names of its contributors may be used to endorse or promote products
**       derived from this software without specific prior written permission.
** 
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
** ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
** WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
** DISCLAIMED. IN NO EVENT SHALL MATTHEW CONTE BE LIABLE FOR ANY
** DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
** (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
** LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
** ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
** SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <stddef.h>
#include <stdint.h>
#include "cmn_def.h"

#if defined(__cplusplus)
extern "C" {
#endif

#if 0

/* tlsf_t: a TLSF structure. Can contain 1 to N pools. */
/* pool_t: a block of memory that TLSF can manage. */
typedef void* tlsf_t;
typedef void* pool_t;

/* Create/destroy a memory pool. */
tlsf_t tlsf_create(void* mem);
tlsf_t tlsf_create_with_pool(void* mem, size_t bytes);
void tlsf_destroy(tlsf_t tlsf);
pool_t tlsf_get_pool(tlsf_t tlsf);

/* Add/remove memory pools. */
pool_t tlsf_add_pool(tlsf_t tlsf, void* mem, size_t bytes);
void tlsf_remove_pool(tlsf_t tlsf, pool_t pool);

/* malloc/memalign/realloc/free replacements. */
void* tlsf_malloc(tlsf_t tlsf, size_t bytes);
void* tlsf_memalign(tlsf_t tlsf, size_t align, size_t bytes);
void* tlsf_realloc(tlsf_t tlsf, void* ptr, size_t size);
void tlsf_free(tlsf_t tlsf, void* ptr);

/* Returns internal block size, not original request size */
size_t tlsf_block_size(void* ptr);

/* Overheads/limits of internal structures. */
size_t tlsf_size(void);
size_t tlsf_align_size(void);
size_t tlsf_block_size_min(void);
size_t tlsf_block_size_max(void);
size_t tlsf_pool_overhead(void);
size_t tlsf_alloc_overhead(void);

/* Debugging. */
typedef void (*tlsf_walker)(void* ptr, size_t size, int used, void* user);
void tlsf_walk_pool(pool_t pool, tlsf_walker walker, void* user);
/* Returns nonzero if any internal consistency check fails. */
int tlsf_check(tlsf_t tlsf);
int tlsf_check_pool(pool_t pool);

#endif

/*************************************************************************/

#include "loc_core_api.h"
#include "loc_memory_api.h"

#define NORMAL_MODE 0
#define FAST_MODE   1

typedef void* (*mw_alloc_func)(uint32_t sz, 
  const char* func, uint32_t line, char* file);
typedef void  (*mw_free_func)(void* ptr);

/**
 * @brief Initialize the alloc module
 * @return None
 */
BOOL mw_os_alloc_init(loc_api_MemoryRegister_t* pz_MemoryRegister);

/**
 * @brief Implement the malloc function
 * @param[in] size - alloc size
 * @return None
 */
void* mw_os_malloc(uint32_t size, uint32_t type, const char* func, uint32_t line, const char* file);

/**
 * @brief Implement the memory free function
 * @param[in] ptr
 * @return None
 */
void mw_os_free(void* ptr);

#define OS_MALLOC_FAST(sz)   mw_os_malloc(sz, FAST_MODE, __FUNCTION__, __LINE__, __FILE__)
#define OS_MALLOC(sz)        mw_os_malloc(sz, NORMAL_MODE, __FUNCTION__, __LINE__, __FILE__)
#define OS_FREE(ptr)         do { mw_os_free(ptr); ptr = NULL;} while(0)
#define OS_CALLOC(n,size)    mw_os_malloc(n*size, NORMAL_MODE, __FUNCTION__, __LINE__, __FILE__)

#define IPC_MALLOC(sz, IpcId)  mw_os_malloc(sz, NORMAL_MODE, __FUNCTION__, IpcId, __FILE__)
#define IPC_FREE(ptr)    do { mw_os_free(ptr); ptr = NULL;} while(0)


/********************Memory Pool Manager Module API***************************/

/**
 * @brief Create a memory pool instance
 * @param[in] pointer to loc_api_MemoryPoolManager_t
 * @return TRUE - Success FALSE - Fail
 */
uint8_t mw_api_MemoryPoolCreate(loc_api_MemoryPoolManager_t* pz_MemoryPoolManager);

/**
 * @brief Destroy the memory pool instance
 * @param[in] pointer to loc_api_MemoryPoolManager_t
 * @return None
 */
void mw_api_MemoryPool_Destroy(loc_api_MemoryPoolManager_t* pz_MemoryPoolManager);

/**
 * @brief Malloc of a memory pool
 * @param[in] pointer to loc_api_MemoryPoolManager_t
 * @param[in] memory size
 * @return Pointer to memory
 */
void* mw_api_MemoryPool_Malloc(loc_api_MemoryPoolManager_t* pz_MemoryPoolManager, uint32_t size);

/**
 * @brief Free of a memory pool
 * @param[in] pointer to loc_api_MemoryPoolManager_t
 * @param[in] pointer need to free
 * @return
 */
void mw_api_MemoryPool_Free(loc_api_MemoryPoolManager_t* pz_MemoryPoolManager, void* ptr);

/********************End Memory Pool Manager Module API***********************/

#if defined(__cplusplus)
};
#endif

#endif
