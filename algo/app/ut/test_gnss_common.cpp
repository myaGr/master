#include "gtest/gtest.h"
#include <emock/emock.hpp>
#include "loc_core_api.h"
#include "gnss_common.h"

TEST(test_gnss_common, tm_01) 
{
  GpsTime_t gps_time = { 0 };
  tm_cvt_SetGpst(&gps_time, 2201, 103649.580);

  BdsTime_t bds_time = { 0 };
  tm_cvt_SetBdt(&bds_time, 845, 103635.580);

  GalTime_t gal_time = { 0 };
  tm_cvt_SetGst(&gal_time, 1177, 103649.580);

  EXPECT_EQ(gps_time.t_fullMsec, bds_time.t_fullMsec);
  EXPECT_EQ(gps_time.t_fullMsec, gal_time.t_fullMsec);
}

TEST(test_gnss_common, sat_signal_idx_convert)
{
  /* sat_Id2Index */
  EXPECT_EQ(gnss_cvt_Svid2SvIndex(1, C_GNSS_GPS), 0);
  EXPECT_EQ(gnss_cvt_Svid2SvIndex(38, C_GNSS_GAL), ALL_GNSS_SYS_SV_NUMBER);

  /*signal_Id2Index */
  EXPECT_EQ(gnss_Signal2FreqIdx(C_GNSS_SIG_BDS_B1I), 0);
  EXPECT_EQ(gnss_Signal2FreqIdx(C_GNSS_SIG_BDS_B2I), 1);
  EXPECT_EQ(gnss_Signal2FreqIdx(C_GNSS_SIG_BDS_B2A), 2);

  /*signal_Index2Id */
  EXPECT_EQ(gnss_FreqIdx2Signal(0, C_GNSS_BDS3), C_GNSS_SIG_BDS_B1I);
  EXPECT_EQ(gnss_FreqIdx2Signal(0, C_GNSS_GPS), C_GNSS_SIG_GPS_L1C);
  EXPECT_EQ(gnss_FreqIdx2Signal(3, C_GNSS_GLO), C_GNSS_SIG_MAX);

}

TEST(test_gnss_common, check_meas_second_align)
{
  BOOL b_expected;
  BOOL b_ret;
  uint64_t q_preSecondAlignMsec = 0;

  q_preSecondAlignMsec = 0;
  for (uint64_t q_tow_ms = 1310; q_tow_ms <= 5600; q_tow_ms +=100)
  {
    if (q_tow_ms % 1000 == 910)
    {
      b_expected = TRUE;
    }
    else
    {
      b_expected = FALSE;
    }

    b_ret = gnss_CheckMeasSecondAlign(q_tow_ms, &q_preSecondAlignMsec, 1.0f);
    if (b_ret != b_expected)
    {
      printf("gnss_CheckMeasSecondAlign fail ret %d expected %d tow %llu\n",
        b_ret, b_expected, q_tow_ms);
    }
    EXPECT_EQ(b_ret, b_expected);
  }

  q_preSecondAlignMsec = 0;
  for (uint32_t q_tow_ms = 1410; q_tow_ms <= 5600; q_tow_ms += 200)
  {
    if (q_tow_ms % 1000 == 10)
    {
      b_expected = TRUE;
    }
    else
    {
      b_expected = FALSE;
    }

    b_ret = gnss_CheckMeasSecondAlign(q_tow_ms, &q_preSecondAlignMsec, 1.0f);
    if (b_ret != b_expected)
    {
      printf("gnss_CheckMeasSecondAlign fail ret %d expected %d tow %d\n",
        b_ret, b_expected, q_tow_ms);
    }
    EXPECT_EQ(b_ret, b_expected);
  }

  q_preSecondAlignMsec = 1000;
  for (uint64_t q_tow_ms = 1990; q_tow_ms <= 10600; q_tow_ms += 500)
  {
    if (q_tow_ms % 1000 == 990)
    {
      b_expected = TRUE;
    }
    else
    {
      b_expected = FALSE;
    }

    b_ret = gnss_CheckMeasSecondAlign(q_tow_ms, &q_preSecondAlignMsec, 1.0f);
    if (b_ret != b_expected)
    {
      printf("gnss_CheckMeasSecondAlign fail ret %d expected %d tow %llu\n",
        b_ret, b_expected, q_tow_ms);
    }
    EXPECT_EQ(b_ret, b_expected);
  }

  q_preSecondAlignMsec = 0;
  for (uint64_t q_tow_ms = 1999; q_tow_ms <= 10600; q_tow_ms += 50)
  {
    if (q_tow_ms % 100 == 99)
    {
      b_expected = TRUE;
    }
    else
    {
      b_expected = FALSE;
    }
    
    uint64_t temp = q_preSecondAlignMsec;
    b_ret = gnss_CheckMeasSecondAlign(q_tow_ms, &q_preSecondAlignMsec, 0.1f);
    if (temp != 0)
    {
      if (b_ret != b_expected)
      {
        printf("gnss_CheckMeasSecondAlign fail ret %d expected %d tow %llu\n",
          b_ret, b_expected, q_tow_ms);
      }
      EXPECT_EQ(b_ret, b_expected);
    }
  }
}

