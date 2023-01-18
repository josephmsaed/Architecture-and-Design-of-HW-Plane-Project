#include <stdio.h>
#include "platform.h"
#include "xil_printf.h"
#include <math.h>

// Registers to send data to the slave PWM IP
#define PWM0 0x43C00000
#define PWM1 0x43C00004

#define Pi 3.14159265
#define LENGTH 6

int main(){
    unsigned long* reg_pwm0 = (unsigned long*)(PWM0);
	unsigned long* reg_pwm1 = (unsigned long*)(PWM1);

    float x_val_offset, y_val_offset, z_val_offset, 
        x_mean, y_mean, z_mean;
	char x, y, z;
	float x_c, y_c, z_c;
    float theta_0, psi_0;
	int j, calib_iter;

	float roll = 0;
	float pitch = 0;
	float JSTK_rx[3] = {0};

    printf("init program\n");
    init_platform();
    SPI_ADXL_Init();
    ADXL362_Init();
    ADXL362_SetRange();
    printf("init complete\n");
    ADXL362_PrintID();

    // set initial position of plane
    *reg_pwm0 = 270; // roll
    *reg_pwm1 = 270; // pitch

    delay_ms(2000);

    //---- Accelerometer Calibration ----
	x_val_offset = 0;
	y_val_offset = 0;
	z_val_offset = 0;
    calib_iter = 500;

    for(int i=0; i<calib_iter;i++){
		while(!ADXL362_IsDataReady()){}

		x = ADXL362_ReadReg(ADXL362_XDATA);
		x_c = ADXL362_Convert(x);
		x_val_offset += x_c;

		y = ADXL362_ReadReg(ADXL362_YDATA);
		y_c = ADXL362_Convert(y);
		y_val_offset += y_c;

		z = ADXL362_ReadReg(ADXL362_ZDATA);
		z_c = ADXL362_Convert(z);
		z_val_offset += z_c;
		printf("%d \n", i);
	}

	x_val_offset = x_val_offset/calib_iter;
	y_val_offset = y_val_offset/calib_iter;
	z_val_offset = z_val_offset/calib_iter;

	printf("------------------\n\r");
	printf("x offset: %f\n", x_val_offset);
	printf("y offset: %f\n", y_val_offset);
	printf("z offset: %f\n\r", z_val_offset);
	printf("------------------\n\r");

	delay_ms(3000);

	while(1){
	    JSTK_readval(&JSTK_rx);
        // Automatic Mode (under joystick button is not pressed)
	    if (JSTK_rx[2] == 0.0f){
		    SPI_ADXL_Init();

		    x_mean = 0;
		    y_mean = 0;
		    z_mean = 0;

		    j = 10;
		    for(int i=0; i<j;i++){
			    while(!ADXL362_IsDataReady()){}
			    x = ADXL362_ReadReg(ADXL362_XDATA);
			    x_c = ADXL362_Convert(x);
			    x_mean += x_c;

	            y = ADXL362_ReadReg(ADXL362_YDATA);
			    y_c = ADXL362_Convert(y);
			    y_mean += y_c;

			    z = ADXL362_ReadReg(ADXL362_ZDATA);
			    z_c = ADXL362_Convert(z);
			    z_mean += z_c;
            }
		    x_mean = x_mean/j;
		    y_mean = y_mean/j;
		    z_mean = z_mean/j;

		    x_mean -= x_val_offset;
		    y_mean -= y_val_offset;
		    z_mean = z_mean - z_val_offset + 1;

		    theta_0 = atan(x_mean/sqrt(y_mean*y_mean +
                            z_mean*z_mean))*180.0/Pi;
		    psi_0 = atan(y_mean/sqrt(x_mean*x_mean +
                            z_mean*z_mean))*180.0/Pi;

		    printf("------------------\n\r");
		    printf("x : %f\n", x_mean);
		    printf("y : %f\n", y_mean );
		    printf("z : %f\n\r", z_mean);
		    printf("------------------\n\r");
		    printf("theta : %f\n", theta_0);
		    printf("ksi : %f\n", psi_0);
		    printf("------------------\n\r");

		    roll -= theta_0;
		    pitch += psi_0;
		    *reg_pwm0 = 270 + (int)roll; // roll
		    *reg_pwm1 = 270 + (int)pitch; // pitch
		    }

        // Manual Mode (under joystick button is pressed)
	    else if (JSTK_rx[2] == 2.0f){
            // roll
            *reg_pwm0 = 180 + (int)((1000 - JSTK_rx[0])*180.0/1000.0); 
		    // pitch
            *reg_pwm1 = 180 + (int)(JSTK_rx[1]*180.0/1000.0); 
        }

	}

    cleanup_platform();
    return 0;
}


