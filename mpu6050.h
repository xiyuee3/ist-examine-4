/*
 * mpu6050.h
 *
 *  Created on: Nov 3, 2023
 *      Author: Asus
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

typedef struct
{
	float acc_angle[2];   //加速度计数据得到姿态角1
	float d_angle[3];     //陀螺仪数据换算出的角速度
	float groy_angle[3];  //陀螺仪数据得出的姿态角2
	float angle[3];       //互补姿态角 依次存入roll,pitch,yaw
}mpu6050;

void mpu_init(mpu6050 *mpudata,float k,float dt);
void mpu_update(mpu6050 *mpudata,float acc[3],float groy[3]);
void mpu_angle(mpu6050 *mpudata,float angles[3]);

#endif /* INC_MPU6050_H_ */
