DFRobot_MuVisionSensor V1.1.7
==============================
* 1.更新格式，添加文件头，中文readme。

DFRobot_MuVisionSensor V1.1.6
==============================
* 1.添加了球算法Label类型：`MU_BALL_TABLE_TENNIS,MU_BALL_TENNIS`
* 2.减少了`begin`函数因通讯异常而导致的程序卡死时间
* 3.弃用了函数`uint8_t begin(void* communicationPort, muVsMode mode)`，改为`uint8_t begin(MuVsUart* communicationPort);`和`uint8_t begin(MuVsI2C* communicationPort);`
* 4.新增函数`muVsVisionState* getVisionState(muVisionType visionType)`，获取算法结果缓存地址
* 5.添加了全新的例程、测试程序与相关文档

DFRobot_MuVisionSensor V1.1.5
==============================
* 1.修复了部分Arduino IDE无法找到头文件和examples的问题
* 2.修复了无法从.ZIP加载库文件的问题
* 3.增加了常用函数的注释
* 4.修复了无法设置算法参数的问题
* 5.添加了默认构造函数`DFRobot_MuVisionSensor()`
* 6.将用户常量添加至`DFRobot_MuVisionSensor.h`头文件
* 7.修复了锁定白平衡后一段时间算法结果不稳定的问题

DFRobot_MuVisionSensor V1.1.4 20190117
==============================
* 1.修复了lock位锁定失败而导致的模块死机问题
* 2.修改了LED库函数
* 3.修改了部分枚举变量名


