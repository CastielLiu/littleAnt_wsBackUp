1. FRESET 
//恢复出厂设置，清除所有配置信息。
2. INTERFACEMODE COM3 AUTO NOVATEL OFF 
//以COM2口为例，指定COM3口为RTK差分接收口，并配置该口的收发模式为AUTO，可以接收任意格式的差分改正数。可以根据实际需求指定COMx为差分接收口。
# the defaut baudrate is 9600
# so we should config the suitable baudrate 
3. SERIALCONFIG COM1 115200 N 8 1 N OFF 
4. SERIALCONFIG COM2 115200 N 8 1 N OFF 
5. SERIALCONFIG COM3 115200 N 8 1 N OFF 
//设置COM1的波特率为115200。可以根据实际需要进行波特率设置。
6. LOG COM3 GPGGA ONTIME 1 
//设置COM3的输出GPGGA 1Hz，接千寻时需要配置该指令。
7. SAVECONFIG 
//保存设置。

1. CONNECTIMU SPI EPSON_G320
//卫导通过SPI接口与IMU连接
2. SETINSPROFILE LAND_PLUS
//开启LAND_PLUS功能，失锁情况的长时间精度保持
# SETINSROTATION RBV X Y Z [XSTD] [YSTD] [ZSTD]
3. SETINSROTATION RBV 0 0 0
//由IMU坐标系旋转到载体坐标系的旋转角度，按照Z、X、Y顺序依据右手定制依次旋转，绕X轴旋转角度值填写到X上，绕Y轴旋转角度值填写到Y上，绕Z轴旋转角度值填写到Z上，[XSTD] [YSTD] [ZSTD]为旋转后与对应的载体坐标轴之间的角度偏差值。
# SETINSTRANSLATION ANT1 X Y Z [XSTD] [YSTD] [ZSTD]
4. SETINSTRANSLATION ANT1 0 0 0.8
// X、Y、Z分别表示IMU中心到ANT1天线相位中心在IMU的X、Y、Z三个轴向上的距离，有正负之分，单位是m，[XSTD] [YSTD] [ZSTD]为在IMU的X、Y、Z三个轴向上的估计偏差值。
#SETINSTRANSLATION ANT2 X Y Z [XSTD] [YSTD] [ZSTD]
5. SETINSTRANSLATION ANT2 0 1.1 0.8
// X、Y、Z分别表示IMU中心到ANT2天线相位中心在IMU的X、Y、Z三个轴向上的距离，有正负之分，单位是m，[XSTD] [YSTD] [ZSTD]为在IMU的X、Y、Z三个轴向上的估计偏差值。
6. ALIGNMENTMODE AIDED_TRANSFER
//对准方式为双天线对准
7. SAVECONFIG             
//保存


