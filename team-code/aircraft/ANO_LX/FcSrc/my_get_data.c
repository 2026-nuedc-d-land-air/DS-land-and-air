#include "my_get_data.h"

double rol, pit, yaw, yaw_part, yaw_last, yaw_round;	// imu输出的欧拉角，单位度
int16_t vel_x, vel_y, vel_z;							// 速度，单位cm/s
int32_t hight;											// 高度,单位cm
int32_t dis_x, dis_y, dis_x_slam, dis_y_slam, yaw_slam; // 累积的距离,单位cm

void get_data(void)
{
	// 读取数据
	rol = (fc_att.st_data.rol_x100) / 100.0;
	pit = (fc_att.st_data.pit_x100) / 100.0;

	yaw_part = fc_att.st_data.yaw_x100;
	if ((yaw_last > 9000) && (yaw_part < -9000))
		yaw_round++;
	if ((yaw_last < -9000) && (yaw_part > 9000))
		yaw_round--;
	yaw = yaw_round * 360 + yaw_part / 100.0;
	// yaw = (int16_t)(yaw_round * 360 + yaw_part / 100.0);
	yaw_last = yaw_part;
	yaw_fix = yaw - yaw_zero; // 归一化角度

	////数据来自光流数据融合后
	//	vel_x = ano_of.of2_dx_fix;
	//	vel_y = ano_of.of2_dy_fix;
	dis_x = ano_of.intergral_x;
	dis_y = ano_of.intergral_y;
	//	hight = ano_of.of_alt_cm;

	// 数据来自ANO_LX.h凌霄接收IMU数据
	vel_x = fc_vel.st_data.vel_x;
	vel_y = fc_vel.st_data.vel_y;
	//	dis_x = -fc_dis.st_data.pix_x;
	//	dis_y = -fc_dis.st_data.pix_y;
	hight = fc_hight.st_data.alt_add;
}
