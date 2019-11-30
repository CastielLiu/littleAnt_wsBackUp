#README
**commit**
[blue]


1. [max_steering_speed_] **通过限制前后帧转角命令差值 控制转向最大速度**
2. **无人驾驶按键按下，开启发送进入无人模式指令，一定时间后，发送换挡指令**
3. **应先启动各控制程序节点，然后按下无人驾驶模式**
4. **所有控制节点程序关闭时，无控制指令，汽车将自动退出无人驾驶模式**
5. **急停开关按下后，汽车退出无人驾驶模式并进入机械急停（此时方向盘可操纵）**
6. **当系统自动退出无人驾驶模式后，需要关闭无人驾驶开关再重新启动**


 ID_CMD_1 0x2C5
 ID_CMD_2 0x1C5

 ID_STATE1 0x151
 ID_STATE2 0x300
 ID_STATE3 0x4D1
 ID_STATE4 0x1D5
 
 gear  0:initial
 	   1:drive 
 	   9:Reverse
 	   A:Neutral
 	   B:Parking
