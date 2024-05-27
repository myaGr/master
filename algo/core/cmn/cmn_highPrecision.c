
#include "cmn_highPrecision.h"
#include "gnss_common.h"
#include "mw_alloc.h"

#ifdef HPREF

#define EPOCH_PER_NODE 100
#define MAX_TIME_DIFF_HPREF 0.05


cmn_highPrecision_oneNode* gpz_highPrecision_beginNode = NULL;
cmn_highPrecision_oneNode* gpz_highPrecision_endNode = NULL;
cmn_highPrecision_oneNode* gpz_highPrecision_searchNode = NULL;

/**
  * @brief init the high precision structure
  * @return TRUE represent success and FALSE for fail
  */
uint8_t cmn_highPrecision_init()
{
  uint8_t u_status = FALSE;
  cmn_highPrecision_oneNode* pz_oneNode;

  if (gpz_highPrecision_beginNode != NULL || gpz_highPrecision_endNode != NULL || gpz_highPrecision_searchNode != NULL)
  {
    return u_status;
  }

  pz_oneNode = (cmn_highPrecision_oneNode*)OS_MALLOC(sizeof(cmn_highPrecision_oneNode));
    
  if (pz_oneNode != NULL)
  {
    u_status = TRUE;
    pz_oneNode->u_valid = FALSE;
    pz_oneNode->u_epochNum = 0;
    pz_oneNode->d_beginTod = 0.0;
    pz_oneNode->d_endTod = 0.0;
    pz_oneNode->pz_data = NULL;
    pz_oneNode->pz_next = NULL;
    gpz_highPrecision_beginNode = pz_oneNode;
    gpz_highPrecision_endNode = pz_oneNode;
    gpz_highPrecision_searchNode = pz_oneNode;
  }

  return u_status;
}

/**
  * @brief deinit the high precision structure
  * @return TRUE represent success and FALSE for fail
  */
uint8_t cmn_highPrecision_deinit()
{
  uint8_t u_status = TRUE;
  cmn_highPrecision_oneNode* pz_oneNode;

  if (NULL != gpz_highPrecision_searchNode)
  {
    gpz_highPrecision_searchNode = NULL;
  }
  if (NULL != gpz_highPrecision_endNode)
  {
    gpz_highPrecision_endNode = NULL;
  }

  if (NULL != gpz_highPrecision_beginNode)
  {
    do
    {
      pz_oneNode = gpz_highPrecision_beginNode;
      if (NULL != gpz_highPrecision_beginNode->pz_data)
      {
        OS_FREE(gpz_highPrecision_beginNode->pz_data);
      }
      gpz_highPrecision_beginNode = gpz_highPrecision_beginNode->pz_next;
      OS_FREE(pz_oneNode);
    } while (NULL != gpz_highPrecision_beginNode->pz_next);
  }

  return u_status;
}


/**
  * @brief  inject the high precision data
  * @param[in]      u_num the inject high precision epoch num
  * @param[in]      pz_highPrecision_array the injected data
  * @return TRUE represent success and FALSE for fail
  */
uint8_t cmn_highPrecision_inject(uint8_t u_num, cmn_highPrecision_oneEpoch* pz_highPrecision_array)
{
  uint8_t u_status = FALSE;
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  int8_t w_degree;
  double d_minute;
  double pd_blh[3] = { 0 };
  cmn_highPrecision_oneEpoch* pz_data = NULL;
  cmn_highPrecision_oneNode* pz_oneNode = NULL;

  if ((NULL != pz_highPrecision_array) && (0 != u_num)
    && (NULL != gpz_highPrecision_endNode) && (FALSE == gpz_highPrecision_endNode->u_valid))
  {
    pz_data = (cmn_highPrecision_oneEpoch*)OS_MALLOC(u_num * sizeof(cmn_highPrecision_oneEpoch));
    if (NULL != pz_data)
    {
      memcpy(pz_data, pz_highPrecision_array, u_num * sizeof(cmn_highPrecision_oneEpoch));
      gpz_highPrecision_endNode->u_valid = TRUE;
      gpz_highPrecision_endNode->pz_data = pz_data;
      gpz_highPrecision_endNode->u_epochNum = u_num;
      gpz_highPrecision_endNode->d_beginTod = pz_highPrecision_array[0].d_tod;
      gpz_highPrecision_endNode->d_endTod = pz_highPrecision_array[u_num - 1].d_tod;

      for (u_i = 0; u_i < u_num; u_i++)
      {
        for (u_j = 0; u_j < 2; u_j++)
        {
          w_degree = (int8_t)(gpz_highPrecision_endNode->pz_data[u_i].pd_pos[u_j] / 100);
          d_minute = gpz_highPrecision_endNode->pz_data[u_i].pd_pos[u_j] - w_degree * 100.0;
          pd_blh[u_j] = (w_degree + d_minute / 60.0) * DEG2RAD;
        }
        pd_blh[2] = gpz_highPrecision_endNode->pz_data[u_i].pd_pos[2];
        gnss_Lla2Ecef(pd_blh, gpz_highPrecision_endNode->pz_data[u_i].pd_pos);
      }

      pz_oneNode = (cmn_highPrecision_oneNode*)OS_MALLOC(sizeof(cmn_highPrecision_oneNode));
      if (pz_oneNode != NULL)
      {
        u_status = TRUE;
        pz_oneNode->u_valid = FALSE;
        pz_oneNode->u_epochNum = 0;
        pz_oneNode->d_beginTod = 0.0;
        pz_oneNode->d_endTod = 0.0;
        pz_oneNode->pz_data = NULL;
        pz_oneNode->pz_next = NULL;
        gpz_highPrecision_endNode->pz_next = pz_oneNode;
        gpz_highPrecision_endNode = gpz_highPrecision_endNode->pz_next;
      }
    }
  }
  return u_status;
}


/**
  * @brief  search the high precision data
  * @param[in]      z_tor the target time
  * @param[out]     pd_pos the high precision reference
  * @return TRUE represent success and FALSE for fail
  */
uint8_t cmn_highPrecision_search(GpsTime_t z_tor, double* pd_pos)
{
  uint8_t u_status = FALSE;
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  double d_tod;
  cmn_highPrecision_oneEpoch* pz_data = NULL;
  cmn_highPrecision_oneNode* pz_oneNode = NULL;
  EpochTime_t z_epochT;

  tm_cvt_GpstToEpoch(&z_tor, &z_epochT);
  d_tod = z_epochT.hour * 3600.0 + z_epochT.min * 60.0 + z_epochT.second;

  if ((NULL != pd_pos) && (NULL != gpz_highPrecision_searchNode))
  {
    while ((FALSE != gpz_highPrecision_searchNode->u_valid) && (NULL != gpz_highPrecision_searchNode->pz_next) &&
      (d_tod - gpz_highPrecision_searchNode->d_endTod > MAX_TIME_DIFF_HPREF))
    {
      gpz_highPrecision_searchNode = gpz_highPrecision_searchNode->pz_next;
    }

    if ((FALSE != gpz_highPrecision_searchNode->u_valid)
      && (NULL != gpz_highPrecision_searchNode->pz_data)
      && (d_tod - gpz_highPrecision_searchNode->d_beginTod > MAX_TIME_DIFF_HPREF)
      && (d_tod - gpz_highPrecision_searchNode->d_endTod < MAX_TIME_DIFF_HPREF))
    {
      for (u_i = 0; u_i < gpz_highPrecision_searchNode->u_epochNum; u_i++)
      {
        if (fabs(d_tod - gpz_highPrecision_searchNode->pz_data[u_i].d_tod) < MAX_TIME_DIFF_HPREF)
        {
          u_status = TRUE;
          break;
        }
      }
      if (u_status)
      {
        for (u_j = 0; u_j < 3; u_j++)
        {
          pd_pos[u_j] = gpz_highPrecision_searchNode->pz_data[u_i].pd_pos[u_j];
        }
      }
    }

  }
  return u_status;
}
#else

/**
  * @brief init the high precision structure
  * @return TRUE represent success and FALSE for fail
  */
uint8_t cmn_highPrecision_init()
{
  return FALSE;
}

/**
  * @brief deinit the high precision structure
  * @return TRUE represent success and FALSE for fail
  */
uint8_t cmn_highPrecision_deinit()
{
  return FALSE;
}

/**
  * @brief  inject the high precision data
  * @param[in]      u_num the inject high precision epoch num
  * @param[in]      pz_highPrecision_array the injected data
  * @return TRUE represent success and FALSE for fail
  */
uint8_t cmn_highPrecision_inject(uint8_t u_num, cmn_highPrecision_oneEpoch* pz_highPrecision_array)
{
    return FALSE;
}

/**
  * @brief  search the high precision data
  * @param[in]      z_tor the target time
  * @param[out]     pd_pos the high precision reference
  * @return TRUE represent success and FALSE for fail
  */
uint8_t cmn_highPrecision_search(GpsTime_t z_tor, double* pd_pos)
{
  return FALSE;
}

#endif