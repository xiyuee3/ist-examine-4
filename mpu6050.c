/*
 * mpu6050.c
 *
 *  Created on: Nov 3, 2023
 *      Author: Asus
 */

#include "mpu6050.h"
#include <math.h>

#define PI 3.1415926

//更新互补姿态角 放在定时中断里固定周期执行 （周期默认0.01s，dt=0.01）定时器里设置周期
void mpu_update(mpu6050 *mpudata,float acc[3],float groy[3]) 
{
    //只通过加速度计数据得到姿态角1
	mpudata->acc_angle[0] = atan2(acc[1],acc[2])*180/PI;
	mpudata->acc_angle[1] = atan2( -acc[0],sqrt( acc[2]*acc[2] + acc[1]*acc[1] ) )*180/PI;
	//通过陀螺仪数据和姿态角1得到角速度
	mpudata->d_angle[0] = 1*groy[0]+groy[1]*sin(mpudata->acc_angle[0])*tan(mpudata->acc_angle[1])+groy[2]*cos(mpudata->acc_angle[0])*tan(mpudata->acc_angle[1]);
	mpudata->d_angle[1] = 0*groy[0]+groy[1]*cos(mpudata->acc_angle[0])+groy[2]*(-sin(mpudata->acc_angle[0]));
	mpudata->d_angle[2] = 0*groy[0]+groy[1]*sin((mpudata->acc_angle[0])/cos(mpudata->acc_angle[1]))+groy[2]*cos(mpudata->acc_angle[0])/cos(mpudata->acc_angle[1]);
	//角速度积分得到姿态角2
	mpudata->groy_angle[0] = mpudata->groy_angle[0] + (mpudata->d_angle[0])*dt;
	mpudata->groy_angle[1] = mpudata->groy_angle[1] + (mpudata->d_angle[1])*dt;
	mpudata->groy_angle[2] = mpudata->groy_angle[2] + (mpudata->d_angle[2])*dt;
    //姿态角1和姿态角2互补得到互补姿态角
	mpudata->angle[0] = (1-K)*(mpudata->groy_angle[0])+K*(mpudata->acc_angle[0]);
	mpudata->angle[1] = (1-K)*(mpudata->groy_angle[1])+K*(mpudata->acc_angle[1]);
	mpudata->angle[2] = 1*(mpudata->groy_angle[2]);
}

void mpu_angle(mpu6050 *mpudata,float angles[3])
{
	angles[0] = mpudata->angle[0];
	angles[1] = mpudata->angle[1];
	angles[2] = mpudata->angle[2];
}

void mpu_init(mpu6050 *mpudata,float k,float dt)// K-互补时加速度计比例 dt-积分时间
{
	mpudata->K = k;
	mpudata->dt = dt;

	for (int i = 0; i < 2; i++)
	{
	    mpudata->acc_angle[i] = 0.0f;
	}

	for (int i = 0; i < 3; i++) 
	{
		mpudata->d_angle[i] = 0.0f;
	}

	for (int i = 0; i < 3; i++)
	{
		mpudata->groy_angle[i] = 0.0f;
	}

	for (int i = 0; i < 3; i++) 
	{
		mpudata->angle[i] = 0.0f;
	}
}

