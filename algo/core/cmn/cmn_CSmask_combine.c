#include "cmn_CSmask_combine.h"
#include "gnss_type.h"

/**
  * @brief combine the mask of cycle slip
  * @param[in] u_tdcpMethodValid is the field mask the TDCP method whether valid
  * @param[in] u_FreqNum is frequency number for one satellite
  * @param[in] u_slipMask is result of cycle slip detect
  * @param[in] u_LLI LLI
  * @param[in] d_deltaTime the delta time between current epoch and previous epoch
  * @param[in] u_isCurrentCloseSky whether the station located in close sky for current epoch
  * @param[in] u_slipMethodValid is the valid of detecting cycle slip method
  * @param[out] pu_slipFlag the cycle slip type
  * @return 1 represent cycle slip and 0 represent non cycle slip happpen
  */
uint8_t cmn_combineCycleSlipMask(uint8_t u_tdcpMethodValid, uint8_t u_FreqNum, uint8_t u_slipMask,
  uint8_t u_LLI, double d_deltaTime, uint8_t u_isCurrentCloseSky, uint8_t u_slipMethodValid, uint8_t* pu_slipFlag)
{
  uint8_t u_isCycleSlipHappen = 0;
  uint8_t u_slipFlag = 0;
  uint8_t u_detectMethodNum = 0;
  if ((GF_DETECT_VALID & u_slipMethodValid))
  {
    ++u_detectMethodNum;
  }
  if ((MULTIFREQUENCY_DETECT_VALID & u_slipMethodValid))
  {
    ++u_detectMethodNum;
  }
  if ((TDCP_DETECT_VALID & u_slipMethodValid))
  {
    ++u_detectMethodNum;
  }
  if ((DOPPLER_DETECT_VALID & u_slipMethodValid))
  {
    ++u_detectMethodNum;
  }
  if (CYCLE_SLIP_DEFAULT == u_slipMask || 0 != u_LLI) // cycle slip
  {
    u_slipFlag |= SLIP_FLAG_LLI;
    u_isCycleSlipHappen = 1;
  }
  if (u_detectMethodNum <= 1)
  {
    u_slipFlag |= SLIP_FLAG_LACK;
    u_isCycleSlipHappen = 1;
  }
  if (u_FreqNum > 1)/*for the dual-frequecny or multi-frequency data*/
  {
    if (!(NON_CYCLE_SLIP_BY_GF & u_slipMask) && (GF_DETECT_VALID & u_slipMethodValid))
    {
      u_slipFlag |= SLIP_FLAG_GF;
      u_isCycleSlipHappen = 1;
    }
   /* if (1 == u_isCurrentCloseSky && !(NON_CYCLE_SLIP_BY_MW & u_slipMask))
    {
      u_slipFlag |= SLIP_FLAG_MW;
      u_isCycleSlipHappen = 1;
    }*/
  }
  if ((MULTIFREQUENCY_DETECT_VALID & u_slipMethodValid) && (GF_DETECT_VALID & u_slipMethodValid)) /*for the multi-frequency data*/
  {
    if (!(NON_CYCLE_SLIP_BY_MULTIFREQUENCY & u_slipMask) && (!(NON_CYCLE_SLIP_BY_GF & u_slipMask)))
    {
      u_isCycleSlipHappen = 1;
    }
  }
  if (1 == u_tdcpMethodValid)/*the cycle slip detect method of TDCP is valid*/
  {
    if (!(NON_CYCLE_SLIP_BY_TDCP & u_slipMask) && (TDCP_DETECT_VALID & u_slipMethodValid))
    {
      u_slipFlag |= SLIP_FLAG_TDCP;
      u_isCycleSlipHappen = 1;
    }
    if (1 == u_isCurrentCloseSky && u_FreqNum <= 1)
    {
      if (!(NON_1CYCLE_SLIP_BY_DOPPLER & u_slipMask) && (DOPPLER_DETECT_VALID & u_slipMethodValid))
      {
        u_slipFlag |= SLIP_FLAG_DOP;
        u_isCycleSlipHappen = 1;
      }
    }
    else
    {
      if (!(NON_5CYCLE_SLIP_BY_DOPPLER & u_slipMask) && (DOPPLER_DETECT_VALID & u_slipMethodValid))
      {
        u_slipFlag |= SLIP_FLAG_DOP;
        u_isCycleSlipHappen = 1;
      }
    }
  }
  else                      /*the cycle slip detect method of TDCP is invalid*/
  {
    if (!(NON_1CYCLE_SLIP_BY_DOPPLER & u_slipMask) && (DOPPLER_DETECT_VALID & u_slipMethodValid))
    {
      u_slipFlag |= SLIP_FLAG_DOP;
      u_isCycleSlipHappen = 1;
    }
  }
  if (pu_slipFlag != NULL)
  {
    *pu_slipFlag = u_slipFlag;
  }
  return u_isCycleSlipHappen;
}