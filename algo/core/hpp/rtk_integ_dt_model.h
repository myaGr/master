/**@file        rtk_integ_dt_model.h
 * @brief       
 * @details     
 * @author      houxiaowei
 * @date        2024/2/19
 * @version     V0.1
 **********************************************************************************
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2024/2/19  <td>0.1      <td>houxiaowei  <td>Init version
 * </table>
 *
 **********************************************************************************
 */
#ifndef AG_LOC_ENGINE_CORE_HPP_RTK_INTEG_DT_MODEL_H_
#define AG_LOC_ENGINE_CORE_HPP_RTK_INTEG_DT_MODEL_H_
#include "rtk_type.h"

/**
 * @brief RTK Decision Tree predict STD
 * @param[in] pz_RTKfilterInfo RTK filer information
 */
extern float rtk_dt_STDPredict(const rtk_filterInfo_t* pz_RTKfilterInfo);

#endif //AG_LOC_ENGINE_CORE_HPP_RTK_INTEG_DT_MODEL_H_


