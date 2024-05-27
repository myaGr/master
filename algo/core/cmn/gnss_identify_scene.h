/**@file        gnss_identify_scene.h
 * @brief       Identify the scene using GNSS observations 
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/12/30  <td>0.1      <td>chenjinhe   <td>Init version
 * </table>
 *
 **********************************************************************************
 */
#ifndef __GNSS_IDENTIFY_SCENE_H__
#define __GNSS_IDENTIFY_SCENE_H__
#include "cmn_def.h"
#include "gnss_type.h"
BEGIN_DECL

typedef uint8_t gnss_SceneType;
#define OPEN_SKY_SCENE   (0x01)
#define SEMI_SKY_SCENE   (0x02)
#define CLOSE_SKY_SCENE  (0x04)

/**
 * @brief using the GNSS observations to identify the scene of the current epoch
 * @param[in] pz_satSigMeasCollect - the struct of satellite measure clooect
 * @return OPEN_SKY_SCENE represent the open sky, SEMI_SKY_SCENE represent the semi sky and CLOSE_SKY_SCENE is the complex scene
 */
gnss_SceneType gnss_usingObsIdentifyScene(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect);

END_DECL
#endif