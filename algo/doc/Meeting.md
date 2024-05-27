# Todo 07/13:

1. 嵌入式 STM32H7
   - freertos工程 @zhanglei
   - 串口数据 @caizhijie

2. PPP-RTK
   - 模糊度固定 @liuguo
   - 浮点方程
   - 原理设计文档 @chenliang

3. 反馈
   - 已完成

4. DCP

   - 数据分享 @houxiaowei

     

# Meeting 07/13/2022



## 1 模块

| 模块            | 人员                         | 时间 |
| --------------- | ---------------------------- | ---- |
| 软件架构        | caizhijie                    |      |
| 回放和日志      | caizhijie                    |      |
| PVT(LS+KF)      | xuning, caizhijie            |      |
| SD              | caizhijie                    |      |
| RTK             | xuning                       |      |
| PPP             | chenjinhe, liuguo, fanweijie |      |
| DCP             | chenjinhe                    |      |
| Dual-Rcv Orient |                              |      |



## 2 PVT

**Requirement:** 

1. PVT输出1Hz，输出卫星质量控制接口

**2022/04/13:**  

1. [已完成] 复用当前算法模块，优化设计API接口，采用新的观测量数据

**2022/04/20:** 

1. [进行中] PVT复用和接口优化已完成，待合入project

**2022/04/27:**

1. [已完成] Ver1.0 PVT复用和接口优化已完成，已合入
2. [进行中] 10Hz PVT性能评估

**2022/05/05:**

1. [进行中] 10Hz PVT性能评估

2. [进行中] PVT 2 RTK 和 PVT 2 PPP 的质量控制结果，输出结构体内容

   

**2022/05/11:**

1. [进行中] 10Hz PVT性能评估

2. [进行中] PVT 2 RTK 和 PVT 2 PPP 的质量控制结果，输出结构体内容

3. [进行中] 时间转化优化, a) leap second 维护 b) Full Gps Time构建 @caizhijie

   

**2022/05/18:**

1. [Hold] 10Hz PVT性能评估

2. [进行中] PVT 2 RTK 和 PVT 2 PPP 的质量控制结果，输出结构体内容

3. [已完成] 时间转化优化, a) leap second 维护 b) Full Gps Time构建 @caizhijie

   

**2022/05/25:**

1. [Hold] 10Hz PVT性能评估

2. [进行中] PVT 2 RTK 和 PVT 2 PPP 的质量控制结果，输出结构体内容



**2022/06/01:**

1. [进行中] PVT2RTK 和 PVT2PPP 的质量控制结果，输出结构体内容



**2022/06/08:**

1. [进行中] PVT2RTK 和 PVT2PPP 的质量控制结果，输出结构体内容



**2022/06/29:**

1. [进行中] PVT Version 2.0 @caizhijie

   


## 3 软件架构和回放

**Requirement:** 

1. 实时日志的回放
2. 支持离线文件的回放注入，RINEX, RTCM, PPP offline. etc.
3. 二进制日志解析工具

**2022/04/13:** a. 软件log记录和回放(caizhijie), b. rtcm decode 优化，注入meas和差分(caizhijie) 

**2022/04/20:** 

1. [进行中] 软件log记录和回放
2. [进行中] rtcm decode 优化
2. [进行中] 增加定向的接口功能

**2022/04/27:**

1. [已完成] 软件log记录和回放
2. [已完成] rtcm decode 优化
3. [进行中] 增加定向的接口功能

**2022/05/05:**

1. [进行中] 增加定向的接口功能
2. [进行中] 千寻Linux SSR SDK集成, @zhanglei

**2022/05/11:**

1. [进行中] 增加定向的接口功能 @caizhijie
2. [进行中] 千寻Linux SSR SDK集成, , @zhanglei
   - SSR RTCM数据库已调通，a) 断网重连，b)码流大小
   - SSR Adapter 在调试中
3. [已完成] 回放调试界面优化v1
4. [已完成] 支持模块独立回放
5. [进行中] 双天线的模块划分 下周Deliver
6. [进行中] 增加GTest, GMock支持



**2022/05/18:**

1. [进行中] 增加定向的接口功能 @caizhijie
2. [已完成] 千寻Linux SSR SDK集成, , @zhanglei
   - [已完成] SSR RTCM数据库已调通，a) 断网重连，b)码流大小 
   - SSR Adapter 在调试中
3. [已完成] 回放调试界面优化v1
4. [已完成] 支持模块独立回放
5. [进行中] 双天线的模块划分  继续 delay @caizhijie
6. [已完成] 增加GTest, GMock支持



**2022/05/25**

1. [已完成] 增加定向的接口功能 @caizhijie
2. [已完成] 千寻Linux SSR SDK集成, , @zhanglei
   - [已完成] SSR RTCM数据库已调通，a) 断网重连，b)码流大小 
   - [已完成] SSR Adapter 在调试中
3. [已完成] 回放调试界面优化v1
4. [已完成] 支持模块独立回放
5. [已完成] 双天线的模块划分  @caizhijie
6. [已完成] 增加GTest, GMock支持
7. [进行中] ARMCC 嵌入式开发板环境调试 @zhang lei



**2022/05/25**

1. [进行中] ARMCC嵌入式开发板环境调试 @zhang lei
2. [进行中] STM32开发板购买 @caizhijie
3. [进行中] 回放数据格式定义和命名规则制定 @caizhijie





**2022/06/08**

1. [进行中] ARMCC嵌入式开发板环境调试 @zhang lei
2. [进行中] 回放数据格式定义和命名规则制定文档编写 @caizhijie
3. [进行中] STM32开发板一直FreeRTOS @zheng lei



**2022/06/15**

1. [进行中] ARMCC嵌入式开发板环境调试: 开发板已到货 @zhang lei
2. [进行中] STM32开发板一直FreeRTOS @zheng lei
3. [进行中] 回放数据格式定义和命名规则制定文档编写 @caizhijie



**2022/06/22**

1. [已完成]] ARMCC嵌入式开发板环境调试: 开发板已到货 @zhang lei
2. [已完成]] 回放数据格式定义和命名规则制定文档编写 @caizhijie
3. [进行中] STM32开发板一直FreeRTOS @zheng lei



**2022/06/29**

1. [已完成] OS层 mutex,signal已移植 @zhang lei

2. [进行中] 系统绝对时间模块开发 @caizhijie

3. [进行中] 实时数据调试 @zhang lei

4. [进行中] 动态内存占用大小统计 @zhang lei



**2022/07/06**

1. [进行中] 实时数据调试 @zhang lei
2. [进行中] 动态内存占用大小统计 @caizhijie
3. [进行中] STM32 中断式串口 @caizhijie



## 4 SD

**Requirement:** 

1. 卫星位置计算
2. SSR2OSR
3. 卫星有效性管理
4. Health检测

**2022/04/13:** 未启动

**2022/04/20:** 未启动

**2022/04/27:** 未启动

**2022/05/05:** 

1. [进行中] 移植卫星位置计算代码，和定义卫星数据结构体，暂时不支持GLONASS

**2022/05/11:**

1. [进行中] 初步编写GPS卫星位置计算，Cubic拟合计算方法

**2022/05/18:**

1. [进行中] 初步编写GPS卫星位置计算，Cubic拟合计算方法 @caizhijie

**2022/05/25:**

1. [进行中] GPS Ephemeris计算已实现，GAL/BDS未实现，Cubic拟合计算方法 @caizhijie
2. [进行中] Ephemeris 缓存维护和更新



**2022/06/01:**

1. [已完成] GPS/GAL/BDS Ephemeris计算已实现 @caizhijie
2. [已完成] Ephemeris 缓存维护和更新 
3. [进行中] Cubic拟合计算方法 @caizhijie
4. [进行中] 卫星位置多项式数据消息分发至HPP/PPP/DCP @caizhijie



**2022/06/01:**

1. [已完成] GPS/GAL/BDS Ephemeris计算已实现 @caizhijie
2. [已完成] Ephemeris 缓存维护和更新 
3. [已完成] Cubic拟合计算方法 @caizhijie
4. [进行中] 卫星位置多项式分发采用HPP/DCP/PPP可重入读取方式 @caizhijie



**2022/06/15:**

1. [已完成] 卫星位置多项式分发采用HPP/DCP/PPP可重入读取方式 @caizhijie

   

## 5 RTK

**Requirement:** 

1. Multiband/Triband measurement
2. 采用序贯滤波器进行固定解滤波
3. 三差探周跳, GF-MW周跳
4. 1Hz输出频率，嵌入式
5. 浮点解或固定解支持1Hz，5Hz, 10Hz
6. RTK需重新开发

**Note:**

- 6月中旬开始写Ver2.0



**2022/04/13:**  

1. 定义PVT2HPP report QoS data(xuning) b. 序贯滤波器(chenjinhe)

   

**2022/04/20:** 

1. [进行中] PVT2HPP report QoS data @xuning

2. [已完成] 序贯滤波器 @chenjinhe

3. [进行中] 观测量预处理 @xuning

   

**2022/04/27:** 

1. [已完成] 沿用当前Ver1.0 RTK算法库，合入新框架。

2. [进行中] 当前Ver1.0 RTK算法存在固定困难的问题. @chenjinhe 

3. [进行中] 观测量预处理 @xuning

   

**2022/05/05:**

1. [进行中] 当前Ver1.0 RTK算法存在固定困难的问题， 继续排查. @chenjinhe @caizhijie

2. [进行中] 观测量预处理 @xuning

   

**2022/05/11:**

  1. [进行中] 无法固定解，继续采集一组楼顶数据用PC的loc_app程序. @chenjinhe @caizhijie
 2. [进行中] 观测量预处理 @xuning



**2022/05/18:**

  1. [已完成] 无法固定解，继续采集一组楼顶数据用PC的loc_app程序. @caizhijie
 2. [进行中] 观测量预处理 @xuning



**2022/05/25:**

  1. [进行中] Ratio计算结果有时为异常值. @caizhijie @chenjinhe
  2. [进行中] 观测量预处理 @xuning



**2022/06/01:**

  1. [进行中] Ratio计算结果有时为异常值. @caizhijie @chenjinhe
 2. [进行中] 观测量预处理，文档说明当前Qos的大致功能 @xuning



**2022/06/08:**

  1. [进行中] Ratio计算结果有时为异常值. @caizhijie @chenjinhe
  2. [进行中] 观测量预处理，文档说明当前Qos的大致功能 @xuning



**2022/06/15:**

  1. [进行中] Ratio计算结果有时为异常值. @caizhijie @chenjinhe

  2. [进行中] 观测量预处理，文档说明当前Qos的大致功能 @xuning

     

## 6 PPP

**Requirement:** 

1. 嵌入式平台，CY平台，dfp，RAM<1M,
2. Multiband/Triband measurement
3. 兼容Qianxun, GeeSpace
4. 浮点解或固定解支持1Hz，5Hz, 10Hz



**2022/04/13:** 

 1. 统一的服务端输入接口设计(chenjinhe) 
 2. Qianxun SDK和账号(chenjinhe) 
 3. PVT2PPP report QoS data(chenjinhe)

**2022/04/20:** 

1. [进行中] 统一的服务端输入接口设计，Geespace和Qianxun整合为统一接口 @liuguo @chenjinhe
2. [进行中] Qianxun PPP 服务SDK和账号 @chenjinhe. 
3. [进行中] PPP离线文件回放支持, 服务数据和观测量数据的时间对齐 @chenjinhe
4. [进行中] Qianxun Requirement:  @liuguo 
   - Windows&Linux(x86) PPP SDK
   - SDK RAM 1M size, 占用RAM太大
5. [进行中] GNSS 时间转换程序
6. [进行中] GNSS 位置转换程序，太阳位置，月亮位置
7. [进行中] mLamdba移植

**2022/04/27:** 

1. [进行中] 统一的服务端输入接口设计，Geespace和Qianxun整合为统一接口, 代码未合入 @liuguo @chenjinhe
2. [进行中] Qianxun PPP 服务SDK和账号, 已和千寻沟通，尚未回复。 @chenjinhe. 
3. [进行中] PPP离线文件回放支持, 服务数据和观测量数据的时间对齐 @chenjinhe
4. [进行中] Qianxun Requirement:  @liuguo 
   - Windows&Linux(x86) PPP SDK
   - SDK RAM 1M size, 占用RAM太大
5. [进行中] GNSS时间转换程序 @fanweijie @liuguo
6. [进行中] GNSS位置转换程序，太阳位置，月亮位置 @fanweijie @liuguo
7. [进行中] mLamdba移植 @fanweijie @liuguo
8. [进行中] 准备未加密的开发环境 @liuguo

**2022/05/05:** 

1. [已完成] 统一的服务端输入接口设计，Qianxun整合为统一接口, 代码已合入 @liuguo @chenjinhe
2. [已完成] Qianxun PPP 服务SDK和账号, 已和千寻沟通，千寻已提供Linux SDK。 @chenjinhe. 
3. [进行中] PPP离线文件回放支持, 服务数据和观测量数据的时间对齐 @chenjinhe
4. [进行中] GNSS时间转换程序，待更新 @fanweijie @liuguo
5. [进行中] GNSS位置转换程序，太阳位置，月亮位置 @fanweijie @liuguo
6. [进行中] PPP算法：校正和质量控制相关(fanweijie)
7. [进行中] PPP算法：浮点滤波方程(chenjinhe)
8. [进行中] PPP算法：固定解计算(liuguo)



**2022/05/11:**

1. [进行中] PPP离线文件回放支持, 服务数据和观测量数据的时间对齐 @chenjinhe
2. [进行中] Qianxun Adapter增加到回放流程中
   - 增加Qianxun SSR码流注入算法接口，计算将Qianxun Adapter 集成到SM模块中
   - 增加Linux平台支持Qianxun Adapter 回放
3. [进行中] GNSS时间转换程序，待优化 @caizhijie
4. [已完成] GNSS位置转换程序，太阳位置，月亮位置，@fanweijie
5. [进行中] PPP算法-校正和质量控制(fanweijie)：
   - 潮汐改正(海潮，固体潮，极潮)，已增加
   - 相位缠绕，待增加
6. [进行中] PPP算法-滤波方程 (chenjinhe)
   - 未进展
7. [进行中] PPP算法-固定解计算 (liuguo)
   - 预研



**2022/05/18:**

1. [进行中] PPP离线文件回放支持, 服务数据和观测量数据的时间对齐 @chenjinhe
2. [进行中] Qianxun Adapter增加到回放流程中 @zhange lei
   - 增加Qianxun SSR码流注入算法接口，计算将Qianxun Adapter 集成到SM模块中
   - 增加Linux平台支持Qianxun Adapter 回放 
3. [已完成] GNSS时间转换程序，待优化 @caizhijie
4. [已完成] GNSS位置转换程序，太阳位置，月亮位置，@fanweijie
5. [进行中] 卫星位置 @caizhijie
6. [进行中] PPP算法-校正和质量控制(fanweijie)：
   - 潮汐改正(海潮，固体潮，极潮)，已增加
   - 相位缠绕，涉及卫星姿态，待增加
   - 卫星状态保存和索引的实现 @caizhijie
   - 周跳探测
7. [进行中] PPP算法-滤波方程 (chenjinhe)
   - 未进展
8. [进行中] PPP算法-固定解计算 (liuguo)
   - lamdba
   - 浮点解 到 固定解 接口，浮点解的输入 @liuguo



**2022/05/25:**

1. [进行中] PPP离线文件回放支持, 服务数据和观测量数据的时间对齐 @chenjinhe
2. [已完成] Qianxun Adapter增加到回放流程中 @zhange lei
   - [已完成] 增加Qianxun SSR码流注入算法接口，计算将Qianxun Adapter 集成到SM模块中
   - [已完成] 增加Linux平台支持Qianxun Adapter 回放 
3. [进行中 50%] 卫星位置 @caizhijie
4. [进行中] PPP算法-校正和质量控制(fanweijie)：
   - 潮汐改正(海潮，固体潮，极潮)，已增加
   - 相位缠绕，涉及卫星姿态，已增加
   - [已完成] 卫星状态保存和索引的实现 @caizhijie
   - [进行中] 卫星数据池的复制和维护 @caizhijie
   - 周跳探测
5. [进行中] PPP算法-滤波方程 (chenjinhe)
   - [进行中] 接口定义 1) PPP Task对其他task的接口 2) 内部子模块划分
6. [进行中] PPP算法-固定解计算 (liuguo)
   - 浮点解 到 固定解 接口，浮点解的输入 @liuguo
   - 新拉分支增加结构体 @chenjinhe



**2022/06/01:**

1. [已完成] Qianxun Adapter增加到回放流程中 @zhange lei
   - [已完成] 增加Qianxun SSR码流注入算法接口，计算将Qianxun Adapter 集成到SM模块中
   - [已完成] 增加Linux平台支持Qianxun Adapter 回放 
2. [进行中] PPP算法-校正和质量控制(fanweijie)：
   - [已完成] 潮汐改正(海潮，固体潮，极潮)，@houxiaowei
   - [已完成] 相位缠绕，涉及卫星姿态，@houxiaowei
   - [已完成] 卫星状态保存和索引的实现 @caizhijie
   - [已完成] 周跳探测 @houxiaowei
   - [进行中] 卫星数据池的复制和维护 @caizhijie
3. [进行中] PPP算法-滤波方程 (chenjinhe)
   - [进行中] 接口定义, PPP Task对其他task的接口 
   - [进行中] 内部子模块划分 @chenjinhe
4. [进行中] PPP算法-固定解计算 (liuguo)
   - 浮点解到固定解接口，浮点解的输入 @liuguo
   - 新拉分支增加结构体 @chenjinhe
   - LAMDBA @liuguo

5. [进行中] 矩阵数学库形式 @caizhijie @houxiaowei @liuguo 



**2022/06/08:**

1. [进行中] PPP算法-校正和质量控制(fanweijie)：
   - [已完成] 潮汐改正(海潮，固体潮，极潮)，@houxiaowei
   - [已完成] 相位缠绕，涉及卫星姿态，@houxiaowei
   - [已完成] 卫星状态保存和索引的实现 @caizhijie
   - [已完成] 周跳探测 @houxiaowei
   - [已完成] 卫星数据池的复制和维护 @caizhijie
2. [进行中] PPP算法-滤波方程 (chenjinhe)
   - [进行中] 接口定义, PPP Task对其他task的接口 
   - [进行中] 内部子模块划分 @chenjinhe
3. [进行中] PPP算法-固定解计算 (liuguo)
   - [进行中] 浮点解到固定解接口，浮点解的输入 @liuguo
   - [进行中] 新拉分支增加结构体 @chenjinhe
   - [已完成] LAMDBA @liuguo
4. [进行中] 矩阵定义和基本加减已增加，QR分解，LU分解和矩阵求逆待增加 @caizhijie @houxiaowei @liuguo 



**2022/06/15:**

1. [进行中] PPP算法-滤波方程 (chenjinhe)
   - [已完成] 接口定义, PPP Task对其他task的接口 @caizhijie
   - [进行中] 内部子模块划分，接口已定义，待代码实现 @chenjinhe
2. [进行中] PPP算法-固定解计算 (liuguo)
   - [进行中] 浮点解到固定解接口已定好，浮点解的输入 @liuguo
   - [进行中] 新拉分支增加结构体 @chenjinhe
3. [进行中] 矩阵定义和基本加减已增加，QR分解，LU分解和矩阵求逆待增加 @caizhijie @houxiaowei @liuguo 



**2022/06/22:**

1. [已完成] 矩阵定义和基本加减已增加，QR分解，LU分解和矩阵求逆待增加 @caizhijie @houxiaowei @liuguo 
2. [进行中] PPP算法-滤波方程 (chenjinhe)

   - [已完成] 内部子模块划分，接口已定义，待代码实现 @chenjinhe	

   - [进行中] 非差非组合模型组建和KF滤波 @chenjinhe
3. [进行中] PPP算法-固定解计算 (liuguo)

   - [已完成] 浮点解到固定解接口已定好，浮点解的输入 @liuguo
- [进行中] 固定解代码实现 @liuguo



**2022/06/29:**

1. [进行中] PPP算法-滤波方程 (chenjinhe)
   - [进行中] 非差非组合模型组建和KF滤波 @chenjinhe @houxiaowei
2. [进行中] PPP算法-固定解计算 (liuguo)
   - [已完成] 浮点解到固定解接口已定好，浮点解的输入 @liuguo



**2022/07/06:**

1. [进行中] PPP算法-滤波方程 (chenjinhe)
   - [进行中] 非差非组合模型组建和KF滤波 @chenjinhe @houxiaowei





## 7 DCP

**Requirement:** 

1. 支持10Hz
2. 采用历元间递推方式更新位置

**2022/04/13:**  未启动

**2022/04/20:**  未启动

**2022/04/27:**

1. [已完成] tdcp观测量定义已增加
2. [已完成] 10Hz观测量的整秒选取策略，并将整秒观测量传至HPP/PPP
3. [进行中] HPP传递数据至DCP @caizhijie
4. [进行中] DCP代码移植 @chenjinhe @caizhijie

**2022/05/05:**

1. [进行中] HPP传递数据至DCP @caizhijie
2. [进行中] DCP代码移植 @chenjinhe @caizhijie



**2022/05/11:**

1. [进行中] HPP传递数据至DCP @caizhijie
2. [进行中] DCP代码移植 @chenjinhe @caizhijie



**2022/05/18:**

1. [进行中] HPP传递数据 gnss_TdcpMeasBlock_t 至DCP @caizhijie

2. [进行中] DCP代码移植 @chenjinhe @caizhijie

   

**2022/05/25:**

1. [进行中] HPP传递数据 gnss_TdcpMeasBlock_t 至DCP @caizhijie
2. [进行中] DCP代码模块化和部分重写 @chenjinhe 



**2022/06/01:**

1. [进行中] HPP传递数据 gnss_TdcpMeasBlock_t 至DCP @caizhijie
2. [进行中] DCP代码模块化和部分重写 @chenjinhe 



**2022/06/08:**

1. [进行中] HPP传递数据 gnss_TdcpMeasBlock_t 至DCP @caizhijie
2. [进行中] DCP代码模块化和部分重写 @chenjinhe 
3. [待定中] 历元间载波差分和Doppler KF 



**2022/06/15:**

1. [已完成] HPP传递数据 gnss_TdcpMeasBlock_t 至DCP @caizhijie
2. [进行中] DCP代码模块化和部分重写 @chenjinhe 
3. [待定中] 历元间载波差分和Doppler KF



**2022/06/22:**

1. [进行中] DCP代码模块化和部分重写 @chenjinhe 

2. [进行中] 历元间载波差分和Doppler 10Hz KF (CY平台) @fanweijie

   

**2022/06/29:**

1. [进行中] DCP代码模块化和部分重写 @chenjinhe 
2. [进行中] 历元间载波差分和Doppler 10Hz KF (CY平台)，10Hz Doppler速度计算已实现，PVA 和 PV模型协方差存在发散 @fanweijie



**2022/07/06:**

1. [进行中] Doppler计算得到10Hz速度，速度的计算结果精度不高，位置输出滤波发散问题已解决，但位置精度需再优化. @chenjinhe @fanweijie






# 8 TC

**Requirement:** 

1. DR算法集成
2. 定义GNSS紧组合DR的数据信息

**2022/05/05:**

1. [进行中] DR算法集成

2. [进行中] GNSS紧组合信息定义@xuning

   

**2022/06/01:**

1. [进行中] PC端Start up IMU+GNSS的数据注入 @zheng lei @caizhijie
2. [进行中] INS <=> GNSS 接口文档设计 @wangjianhui 



**2022/06/08:**

1. [进行中] PC端Start up IMU+GNSS的数据注入 @zheng lei @caizhijie
2. [进行中] a) INS码流demo.  b) 码流定义/解码代码(c).  c)接口的结构体定义. @chenjinhe @wangjianhui



**2022/06/15:**

1. [进行中] PC端Start up IMU+GNSS的数据注入 @zheng lei @caizhijie
2. [进行中] a) INS码流demo.  b) 码流定义/解码代码(c语言).  c)接口的结构体定义. @chenjinhe @wangjianhui



**2022/06/22:**

1. [进行中] a) INS码流demo.  b) 码流定义/解码代码(c语言).  c)接口的结构体定义. @chenjinhe @caizhijie



**2022/06/29:**

1. [进行中] 解码已完成，集成到大通分支 @wangjianhui
   - 加速度缺失调试。



# 9 定向
