



<center><font color=black size=8><b>C/C++编程规范说明</b></font></center>



[TOC]



# 1 命名



## 1.1 文件命名

- 文件名要全部小写。并用下划线方式对名称连接
- C++源文件后缀.cpp，而不是.cxx或者.cc, 头文件后缀.h, C源文件后缀.c.  一般情况下源文件需有对应头文件
- [option] 文件名称描述名字一般不要超过**4**个
- 不推荐CMake, 因为我们代码不开源

不推荐的命名

| 不推荐的命名                     | 推荐的命名                      |
| -------------------------------- | ------------------------------- |
| SLAM_Location.cpp                | slam_location.c  or slm_loc.cpp |
| slam_location_sync_frame_sub.cpp | slmloc_syncfrm.cpp              |
| slmloc_syncfrm.cxx               |                                 |
| slmloc_syncfrm.cc                |                                 |
| slmloc-syncfrm.cc                |                                 |
| SlamLocation.cpp                 |                                 |

## 1.1 变量命名

### 1.1.1 基本变量及前缀

采用**stdint.h** 对基本变量的定义

 ```c
 typedef signed char        int8_t;
 typedef short              int16_t;
 typedef int                int32_t;
 typedef long long          int64_t;
 typedef unsigned char      uint8_t;
 typedef unsigned short     uint16_t;
 typedef unsigned int       uint32_t;
 typedef unsigned long long uint64_t;
 // 可使用float 和 double
 
 #define INT8_MIN         (-127i8 - 1)
 #define INT16_MIN        (-32767i16 - 1)
 #define INT32_MIN        (-2147483647i32 - 1)
 #define INT64_MIN        (-9223372036854775807i64 - 1)
 #define INT8_MAX         127i8
 #define INT16_MAX        32767i16
 #define INT32_MAX        2147483647i32
 #define INT64_MAX        9223372036854775807i64
 #define UINT8_MAX        0xffui8
 #define UINT16_MAX       0xffffui16
 #define UINT32_MAX       0xffffffffui32
 #define UINT64_MAX       0xffffffffffffffffui64
 ```

### 1.1.2 变量命名

变量名按照小写和下划线连接的方式进行命名

| 变量           | 格式                                                         | 例子                                                         |
| -------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| int8_t         | s_xxx                                                        | int8_t s_value;                                              |
| uint8_t        | u_xxx                                                        | uint8_t u_value;                                             |
| int16_t        | w_xxx                                                        | int16_t w_value;                                             |
| uint16_t       | w_xxx                                                        | uint16_t w_value;                                            |
| int32_t        | q_xxx                                                        | int32_t q_value;                                             |
| uint32_t       | q_xxx                                                        | uint32_t q_value;                                            |
| int64_t        | t_xxx                                                        | int64_t t_value;                                             |
| uint64_t       | t_xxx                                                        | uint64_t t_value;                                            |
| float          | f_xxx                                                        | float f_value;                                               |
| double         | d_xxx                                                        | double d_value;                                              |
| 结构体名       | XxxYyy_t                                                     | typedef struct {<br />    uint8_t a;<br />    uint8_t b;<br />} TestValue_t; |
| 结构体变量     | z_Xxx                                                        | TestValue_t z_Value;                                         |
| 枚举名         | typedef enum<br/>{<br/>  XXX = 0,<br/>  YYY,<br />   ... <br />} XxxYyy_EnumVal;<br/>typedef uint8_t/uint16_t XxxYyy_Enum; | typedef enum<br/>{<br/>   C_VALUE1 = 0,<br/>   C_VALUE2,<br /> } TestValue_EnumVal;<br/>typedef uint8_t TestValue_Enum; |
| 枚举           | e_xxx                                                        | TestValue_Enum e_value;                                      |
| 数组           | <类型> p<类型缩写>_aXxx[YYY];                                | uint8_t pu_aValue_test[10];<br />value_t pz_aValue_test[10]; |
| 指针变量       | <类型>   *p<类型缩写>_xxx;                                   | uint8_t   *pu_value;<br />value_t   *pz_Value;<br />uint8_t   *pu_aValue[10]; |
| 对象           | xxYyZz                                                       | CTestValue testValue;                                        |
| const 类型     | 不需特别注明是否为const类型                                  |                                                              |
| 全局变量       | g_xx                                                         | uint8_t gu_value;                                            |
| 全局静态变量   | gs_xx                                                        | uint8_t gsu_value;                                           |
| 局部变量       | x_xx                                                         | uint8_t u_value;                                             |
| 局部静态变量   | s_xx                                                         | uint8_t su_value;                                            |
| 全局结构体变量 | gz_xx                                                        | struct {uint8_t a;uint8_t b;} gz_value;                      |
| 结构体变量     | z_xx                                                         | struct {uint8_t a;uint8_t b;} z_value;                       |

一般情况下，不推荐使用**局部静态变量**

## 1.3 函数命名

为区别变量，函数类型大小写方加下划线式进行命名; 

```c++
// <模块>_XxxYyyZzz; 或者 <模块>_<子模块>_XxxYyyZzz;
void slam_FindKeyFrame();
// or 增加子模块
void slam_loc_FindKeyFrame();
// or 去掉部分元音字母
void slam_loc_FindKyFrm();

// 不推荐的函数命名
void SLAMLocFindKeyFrame();
```

函数命名比较灵活，可以参考以下几点进行命名

- 文件中区分static函数和非static函数，将只会在该文件中调用的函数声明为static函数。非static函数放在文件后面部分，并且需要在头文件中有对应的声明
- 对于不是特别重要的名词或者比较常见的名词，可以采用取消元音字母的方式缩短长度
- 命名可按照动词+名词的方式

## 1.4 宏命名

- 特定文件使用的宏定义，放在特定文件中，宏的可调用范围尽量靠近宏的使用范围。
- 宏命名需为大写字母和下划线组成。
- 带有参数的宏，对参数需增加括号

## 1.5 类命名

为区别变量和函数，类名称采用大写字母且不带下划线的方式。类型的，结构体和枚举也采用该方式。

并且为区分结构体成员变量和成员函数，与其他函数与变量时，需增加**m**进行注明

```c++
// 结构体
typedef struct {
    <类型> <类型缩写>_xxx;
    ...
} XxxYyy_t;

// 枚举
typedef enum {
    <value0>，
    <value1>，
    ...
} XxxYyy_EnumVal;
typedef uint8_t XxxYyy_Enum;

// 类
class CXXXyyy
{
public:
  void CXXXyyy();
  void ~CXXXyyy();
    
  /* 变量 */
  <类型> <类型缩写>_mXxx_yyy;
      
  /* 数组 */
  <类型> <类型缩写>_maXxx_yyy[NUMBER];

  /* 一般成员函数 */
  <返回值类型> m_XxYy();

  /* 静态成员函数 */
  static <返回值类型> ms_XxYy();
}
```



**例如**

```c++
// 结构体
typedef struct {
    uint8_t value;
} TestValue_t;

// 枚举
typedef enum {
    value1，
    value2，
    ...
} TestValue_EnumVal;
typedef uint8_t TestValue_Enum;

// 类
class CTestValue
{
public:
  void CTestValue();
  void ~CTestValue();
    
  /* 变量 */
  uint8_t u_mTest_value;
  TestValue_t z_MTest_value2;
    
  /* 数组 */
  uint8_t u_maValues[10]  

  /* 一般成员函数 */
  void ms_DoTest();
      
  /* 静态成员函数 */
  static void ms_DoTest();
}
```



# 2 文件

## 2.1 保存目录

对于源文件的头文件放在于源文件一个目录，外部依赖或者接口的头文件放在include目录或者其他目录

```
.
├── include
│   └── module_api.h
└── source
    ├── module1.cpp
    ├── module1.h
    ├── module2.cpp
    └── module2.h
```

## 2.2 头文件#define

1. 以双下划线开头和结尾
2. 最后一个#endif增加说明

```c
#ifndef __FILENAME_H__
#define __FILENAME_H__
...
#endif /* __FILENAME_H__ */
```

## 2.3 引用顺序

```
// 该源代码直接相关的头文件 和 不稳定的头文件
#include “test_demo.h”

// C 系统头文件
#include <stdio.h>
#include <sdtint.h>
...
// C++ 系统头文件
#include <string>
#include <iostream>
// 其他系统文件
#include <sys/time.h> // linux
#include <windows.h>  // windows
...
// 需要的项目内其他稳定的文件
#include "log.h"
#include “matrix.h”
...
```

## 2.4 C++调用C头文件

 为了使C函数能被C++调用，在定义头文件时需增加extern "C"

```c++
#ifdef __cplusplus
extern "C" {
#endif

xxxxx

#ifdef __cplusplus
}
#endif
```



# 3 注释

参考格式doxygen注释规范

- 注释不要描述从阅读代码就能轻易获取的信息，要描写一些**即使阅读该部分代码也无法获取**的内容, 例如背景原因，使用限制，原理介绍或相关连接地址.
- 注释必须时英文，不要中文，因为中文注释会在文件转码后容易变成乱码

## 3.1 文件头


```c++
/**@file        main.cpp
 * @brief       
 * @version     V1.0
 * @copyright   Copyright (c) 2022-2022  Guangzhou Asensing Techology Co.,Ltd.
 **********************************************************************************
 * @note        Hardware rk3399pro
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/01/21  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */
```

## 3.2 函数头

```c++

/**
 * @brief xxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
          xxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
 * @param[in] para1
 ....
 * @param[out] para2
 ....
 * @return 
*/
```

## 3.3 枚举

```c++
/** Test enum vals */
typedef enum {
  /** comment for AA */
  AA = 0, 
  /** comment for BB */
  BB,   
  /** comment for CC */
  CC,  
} Test_EnumVal;

/** Test enum vals */
typedef enum {
  AA = 0, //!< comment for AA
  BB,     //!< comment for BB
  CC,     //!< comment for CC
} Test_EnumVal;
```

## 3.4 结构体

```c++
/** Test struct vals */
typedef struct {
  /** comment for a */
  uint8_t a; 
  /** comment for b */
  uint8_t b;
} Test_t;

/** Test struct vals */
typedef struct {
  uint8_t a; //!< comment for a
  uint8_t b; //!< comment for b
} Test_t;
```



# 4 格式

- 每行最大值80个字符，代码不要太长，可以适当放宽。
- 不要写中文
- 代码中，不要使用Tab，用空格代替. 用缩进**2** 空格代码一个Tab
- 文件采用UTF-8 编码
- 命名空间内容不缩进

## 4.1 关于if的格式

- if 后的 { 需换行， 并且if 条件的执行代码只有一行，也要加{}

  ```c++
  int main()
  {
  	int* x = NULL;
  	int y = 0;
      // 不推荐
      if (NULL == x) {
          y = 1;
      }
      
      // 不推荐
      if (NULL == x)
          y = 1;
      
  	// 推荐
      if (NULL == x) 
  	{
  	  y = 1;
          return 0;
  	}
  	return 0;
  }
  ```
  
- 函数的圈复杂度控制在10以内，简单理解就是 一个函数内条件分支的个数。这个是从单元测试的角度触发考虑的，一个函数条件分支太多，对于单元测试来说要完全覆盖所有分支就越困难。

- 需要对一些列条件进行判断时，尽量把条件拆分以下，单个if的判断条件不要超过4个，并且各个条件的逻辑关系相同

  ```
  // 不好的if
  if (((x > y) || (a < b)) && (c != b) && (ptr = getfunc()))
  {
    ..
  }
  
  // 好的if
  if ((x > y) || (a < b))
  {	
      if (c != b)
      {
          ptr = getfunc();
          if (NULL != ptr)
          {
          ...
  
          }
      }
  }
  ```

## 4.2 关于for的格式



# 5 类



# 6 性能优化

影响性能的因素有很多，硬件层面有存储层次体系，流水线执行顺序，并行和并发，软件层面有系统调用开销，编译器优化和语言抽象性。

## 6.1 算术优化

### 6.1.1 乘法代替除法

```c
#define PI      (3.14159265358979323846)
#define INV_PI  (0.3183098861837907) /* 1/PI */
#define RAD2DEG (57.29577951308232)  /* 180/PI */

float deg = 90.0f;
float rad = 1.0f;

// 直接使用除法，可优化
float target_deg = rad * 180 / PI;

// 换为乘法
float target_deg = rad * 180 * INV_PI;

// 换为乘法
float target_deg = rad * RAD2DEG;
```

### 6.1.2 常量归并计算

```
int x = 0;
int y = 1;
int c = 0;

// 不好的操作
c = 10*x*10*y;

// 较好的操作
c = 10*10*x*y

// 更好的操作
c = 100*x*y
```

### 6.1.3 FPU

- 在带有单精度FPU的设备上，**float的计算快于double**

- 在带有双精度FPU的设备上，**double的计算快于float**

  ***程序设计时，需考虑到未来在不同平台上使用的可能性，对于计算量大的部分代码，浮点数类型可以考虑为可配置***。

## 6.2 避免使用不对齐的结构

网络传输数据往往时按照紧凑且不对齐的方式信息传输的，但是CPU处理不对齐数据时，需额外的操作。所以接收到网络数据后，尽量转为结构化数据来处理。

- 结构对齐有利于提高空间利用率

```c++
// 推荐使用
typedef struct {
  uint8_t a1;
  uint8_t a2;
  uint8_t a3;
  uint8_t a4;
  uint32_t a5;
  uint32_t a6;
  uint32_t a7;
  uint64_t a8;
  uint64_t a9;
  uint64_t a10;
  uint64_t a11;
  uint64_t a12;
  uint64_t a13;
} Type1_t;

// 不推荐使用,错误的
typedef struct {
  uint8_t a1;
  uint64_t a8;
  uint8_t a2;
  uint64_t a9;
  uint8_t a3;
  uint64_t a10;
  uint8_t a4;
  uint64_t a11;
  uint32_t a5;
  uint64_t a12;
  uint32_t a6;
  uint64_t a13;
  uint32_t a7;
} Type2_t;

sizeof(Type1_t)=64 sizeof(Type2_t)=104
```

- 结构对齐有利于代码执行效率. 对不对齐的数据访问，CPU有额外的操作. 

```
// 结构对齐
typedef struct {
  uint8_t a1;
  uint32_t a4;
  uint8_t a2;
} Type1_t;

// 结构不对齐
#pragma pack (push, 1)
typedef struct {
  uint8_t a1;
  uint32_t a4;
  uint8_t a2;
} Type2_t;
#pragma pack (pop)

sizeof(Type1_t)=12 sizeof(Type2_t)=6
```

## 6.3 Cache

数据访问时可以考虑**数据局部性**的原则，避免使用不紧凑的数据访问，提高CPU cache的命中率。cache命中失败时，会从内存获取数据。访问cache的速度远大于访问内存的速度。

松散的结构提不仅浪费内存，而且访问的效率也降低。

```c++
uint8_t data[10][10];

// 较好的访问方式
for (size_t i = 0; i < 10; ++i)
{
    for (size_t j = 0; j < 10; ++j)
    {
        data[i][j] = 0;
    }
}

// 不好的隔行访问
for (size_t i = 0; i < 10; ++i)
{
    for (size_t j = 0; j < 10; ++j)
    {
        data[j][i] = 0;
    }
}

```

- 使用内存，尽量保证是32N或64N的大小，这有利总线和内存的使用效率。但是有一种额外情况是，开辟内存大小尽量避免**size=2^(2N)  (n>=7)**, 这和cache的设计机制相关，这些内存大小会出现**缓存竞争**的问题

  ```c++
  uint8_t data1[32*32]; // 合理size
  uint8_t data1[64*64]; // 合理size
  uint8_t data1[128*128]; // 不合理size
  uint8_t data1[256*256]; // 不合理size
  uint8_t data1[256*257]; // 合理size
  uint8_t data1[512*512]; // 不合理size
  uint8_t data1[512*511]; // 合理size
  uint8_t data1[1024*1024]; // 不合理size
  ```
  
  

## 6.4 避免做重复的操作

```c++
// exmaple-01
uint8_t* str = "hello world";
// 假设这里编译器没有优化的话，这里strlen会不断调用
for (int i = 0; i < strlen(str); ++i)
{
    str[i] = 0;
}
// 较好的代码
for (int i = 0, count = strlen(str); i < count; i++)
{
    str[i] = 0;
}
```



```c++
// example-02
for (int i = 0; i < count; ++i)
{
    // 重复调用构造函数
    std::string s;
    s = "Hi";
    process(s);
}

std::string s;
for (int i = 0; i < count; ++i)
{
    s.clear();
    s = "Hi";
    process(s);
}
```



## 6.5 inline

对小且经常调用的函数增加inline描述 



## 6.6 函数参数

- 函数参数为不修改的对象，使用const Obj& xxx
- 函数参数为需要修改的对象，使用 Obj& xxx
- 函数参数为需要拷贝的对象，使用Obj

```c++
void print(const std::string& s);
void modify(std::string& s);
void clone(std::string s);// c++ 14
```



## 6.7 空间与时间的平衡

- 在内存空间富裕的情况下，可以采用多使用空间来换取减少内存分配的调用

```c++
// 空间换时间
uint8_t data[size];
for (int i = 0; i < count; i++)
{
	get_data(data);
    process_data(data);
}

// 时间换空间
for (int i = 0; i < count; i++)
{
    uint8_t* data = malloc(size);
    get_data(data);
    process_data(data);
    free(data);
}

// 非常理想，两者兼得
uint8_t* data = NULL;
for (int i = 0; i < count; i++)
{
    if (NULL == data)
    {
        data = malloc(size);
    }
    get_data(data);
    process_data(data);
}
if (NULL != data)
{
	free(data);   
}
```

- 编译器优化选项： **-O3**：要求性能， **-Os**: 要求空间，可以考虑对工程中不同文件或者模块使用不同的优化选项。

## 6.8 多线程竞争

**没有竞争** > **原子操作** > **读写锁** > **锁**



# 7 《重构》笔记



- 重复代码： 重复代码归并

- 函数过长: 这个看情况，主要功能函数保持在500行以内

- 参数过长: 函数参数尽量最多为4个参数，操作4个就会涉及到LR指针的空间问题.

- switch-case: 各个case的statement尽量简短，复杂的操作包装为一个函数

- 冗余的代码，陈旧的代码和为未来设计的代码，要删除

- 中间人工作不要太频繁，要是经常需要中间人来传递信息，考虑把中间人删除

  

# 8 并发编程





# 参考

1. Google Style Guide: https://zh-google-styleguide.readthedocs.io 
2. https://www.cnblogs.com/flyinggod/p/8343478.html
