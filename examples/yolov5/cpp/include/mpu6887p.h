#ifndef _MPU_6887P_H
#define _MPU_6887P_H

#ifdef __cplusplus
extern "C" {
#endif

enum sensor
{
	ACC = 0,
	GYRO,
};

enum parameter
{
	FSR = 0,
	BW,
};

int get_input_number();
int gsensor_init();
int gsensor_deinit();
int gsensor_capture_init();
int gsensor_capture_deinit();
void mpu_set_angle(uint8_t angle);
void mpu_set_rotation(uint8_t rot);
void mpu_set_enable(uint8_t enable);
int get_angle();

#ifdef __cplusplus
}
#endif

#endif