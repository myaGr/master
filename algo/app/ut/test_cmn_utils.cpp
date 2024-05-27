#include "gtest/gtest.h"
#include <emock/emock.hpp>
#include "cmn_utils.h"


BOOL compare(void* a, void* b)
{
  uint32_t* _a = (uint32_t*)a;
  uint32_t* _b = (uint32_t*)b;
  return (*_a) < (*_b);
}

TEST(test_cmn_util, math_sort_01)
{
  uint32_t num[10] = { 1,3,10,100,4,7,19,0,2,22 };
  loc_sort((uint8_t*)num, sizeof(uint32_t), 10, compare);
  
  for (int i = 1; i < sizeof(num)/sizeof(num[0]); i++)
  {
    EXPECT_EQ(compare(&num[i - 1], &num[i]), TRUE);
  }
}

TEST(test_cmn_util, matrix_inverse_01)
{
  double data[5][5] = {
    {1.0, 2.0, 3.0, 4.0, 5.0},
    {2.0, 1.0, 1.0,-2.0, 5.0},
    {0.0, 2.0, 3.0, 4.0, 3.0},
    {4.0, 2.0, 2.0, 4.0, 5.0},
    {5.0, 0.0, 3.0, 4.0, 7.0}
  };
  
  matrix_t* m = matrix_new_from_array(5, 5, (const double*)data);
  matrix_t* m_inv = matrix_new(5, 5);
  matrix_t* m_rst = matrix_new(5, 5);

  matrix_inverse(m, m_inv);
  matrix_mul(m, m_inv, m_rst);
  
  for (int i = 0; i < 5; i++)
  {
    EXPECT_EQ(FLT_EQU(MAT(m_rst, i, i), 1.0), TRUE);
  }
}


TEST(test_cmn_util, matrix_inverse_02)
{
  double data[5][5] = {
    {1.0, 2.0, 3.0, 4.0,-9.0},
    {0.0, 1.0, 1.0,-2.0, 5.0},
    {0.0, 0.0, 3.0, 4.0, 3.0},
    {0.0, 0.0, 0.0, 1.0, 5.0},
    {0.0, 0.0, 0.0, 0.0, 7.0}
  };

  matrix_t* m = matrix_new_from_array(5, 5, (const double*)data);
  matrix_t* m_inv = matrix_new(5, 5);
  matrix_t* m_rst = matrix_new(5, 5);

  matrix_rightup_inverse(m, m_inv);
  matrix_mul(m_inv, m, m_rst);

  for (int i = 0; i < 5; i++)
  {
    for (int j = 0; j < 5; j++)
    {
      double val = MAT(m_rst, i, j);
      if (i == j)
      {
        EXPECT_EQ(FLT_EQU(val, 1.0), TRUE);
      }
      else
      {
        EXPECT_EQ(FLT_EQU(val, 0.0), TRUE);
      }
    }
  }
}

TEST(test_cmn_util, matrix_qr_01)
{
  double data[5][3] = {
          {1.0, 2.0, 3.0},
          {2.0, 1.0, 1.0},
          {0.0, 2.0, 3.0},
          {4.0, 2.0, 2.0},
          {5.0, 0.0, 3.0}
  };
  
  double q[3][3] = {0};
  
  matrix_t* A = matrix_new_from_buffer(5, 3, (double*)data);
  matrix_t* Q = matrix_new(5, 3);
  matrix_t* R = matrix_new(3, 3);

  matrix_qr_decompose(A, Q, R);
  matrix_t* Qt = matrix_clone(Q);
  matrix_transpose(Qt);
  
  matrix_t* QtQ = matrix_new(Qt->row, Qt->row);
  matrix_mul(Qt, Q, QtQ);
  
  for (uint32_t i = 0; i < QtQ->row; i++)
  {
    double v = MAT(QtQ, i, i);
    EXPECT_EQ(TRUE, FLT_EQU(v, 1.0));
  }
}

