# ist-examine-4
包含一个mpu6050.c文件和一个mpu6050.h文件 只有两个函数

mpu_update(mpu6050 *mpudata,float acc[3],float groy[3],float K,float dt) 处理传入的陀螺仪参数换算成姿态角
    
    mpu6050 *mpudata：处理数据的中间变量
    acc[3]：传入的加速度计数据 依次为 acc_x,acc_y,acc_z
    groy[3]:传入的陀螺仪数据 依次为groy_x,groy_y,groy_z
    K:互补时加速度计数据比例 angle = (1-K)*groy_angle+K*acc_angle[0]
    dt:积分时间，与定时中断周期相同(例如定时周期为0.1秒，则dt=0.1)

void mpu_angle(mpu6050 *mpudata,float angles[3]) :将换算出的姿态角存入数组angle中  
    
    mpu6050 *mpudata：与mpu_update中的一致
    angles[3]：依次为 roll,pitch,yaw

开始时创建一个mpu6050类型的结构体和一个float型的数组存放角度数据 

    mpu6050 mpu_0；
    float angle[3];

将mpu_update函数放入定时中断中周期执行
需要读取姿态角时调用mpu_angle函数将数据存入angle[3]中


