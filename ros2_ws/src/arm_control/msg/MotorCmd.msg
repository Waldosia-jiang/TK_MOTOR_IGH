uint8 mode  #运动模式 1、位置控制；3、速度控制；4、力位混控；8、csp模式；10、cst模式
float32 q	#期望弧度位置
float32 dq	#期望速度rad/s
float32 tau	#前馈力矩，Nm
float32 kp	#位置刚度系数	
float32 kd	#速度阻尼系数	
