#include "gtest/gtest.h"
#include <emock/emock.hpp>

#include "loc_core_api.h"

TEST(test_Loc_Core_API, loc_api_01) 
{
  EXPECT_EQ(1, loc_api_Initialize());
}

TEST(test_Loc_Core_API, loc_api_02)
{
  EMOCK((int32_t(*)())loc_api_Initialize)
    .stubs()
    .will(returnValue(0));
  EXPECT_EQ(0, loc_api_Initialize());
}