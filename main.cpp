/*#################################################################################

Program Name    : MPU6050 Hello World 
Author          : Crispin Mukalay
Date Modified   : 17/10/2018
Compiler        : ARMmbed
Tested On       : NUCLEO-F446RE

Description     : Demonstrates the use of the MPU6050 gryroscope/accelerometer/temperature
                  sensor to read gyroscope 3-axis angular velocities(°/s) and accelerometer
                  3-axis accelerations (°).
                  
Requirements    : * NUCLEO-F446RE Board
                  * MPU6050 Module
              
Circuit         : * The MPU6050 module is connected as follows:
                    VCC                      -   3.3V
                    GND                      -   GND
                    SCL                      -   PB10 (I2C2_SCL pin)
                    SDA                      -   PB3 (I2C2_SDA pin)

####################################################################################*/

#include "mbed.h"
#include "MPU6050.h"
#include <math.h>
#define pi 3.141592654
Serial bluetooth(D10, D2);//Tx, Rx
MPU6050 AccGyro(D14, D15); // Create an MPU object called AccGyro

Serial pc(SERIAL_TX, SERIAL_RX);    //To use the PC as a console (display output)

int16_t Ax, Ay, Az, Gx, Gy, Gz;
float Ax_f, Ay_f, Az_f;
double Gx_f, Gy_f, Gz_f;
float Ax_f_sum, Ay_f_sum, Az_f_sum, Gx_f_sum, Gy_f_sum, Gz_f_sum;
float roll, pitch, yaw;

int main() {
    
    uint16_t AccelReadings[3] = {0, 0, 0};
    uint16_t GyroReadings[3] = {0, 0, 0};
    uint8_t DevId;

    pc.printf("Starting MPU6050 test...\n");
    DevId = AccGyro.getWhoAmI();
    
    if(DevId == 0x68){
        pc.printf("\n");
        pc.printf("MPU6050 detected...\n");
        pc.printf("Device ID is: 0x%02x\n", DevId);
        pc.printf("\n");
    }else{
        pc.printf("\n");
        pc.printf("MPU6050 not found...\n");
        while(1);
    }
    
    // The device will come up in sleep mode upon power-up.
    AccGyro.setPowerCtl_1(0x00, 0x00, 0x00, 0x00, INT_8MHz_OSC);    // Disable sleep mode
    wait(.001);
    
    // Full scale, +/-2000°/s, 16.4LSB°/s.
    AccGyro.setGyroConfig(GYRO_ST_OFF, GFS_2000dps); // Accelerometer elf-test trigger off.
    wait(.001);
    
    // Full scale, +/-16g, 2048LSB/g.
    AccGyro.setAccelConfig(ACC_ST_OFF, AFS_16g);    // Gyroscope self-test trigger off.
    wait(.001);
        
    while (true) {
        
       wait(0.2);
       
       Ax_f_sum = 0;
       Ay_f_sum = 0;
       Az_f_sum = 0;
       Gx_f_sum = 0;
       Gy_f_sum = 0;
       Gz_f_sum = 0;
       
       for(int i = 0; i < 10; i = i + 1)    // Take ten analog input readings
       {
            AccGyro.readAccel(AccelReadings);   // Extract accelerometer measurements
            AccGyro.readGyro(GyroReadings);     // Extract gyroscope measurements
            
            // 2s complement acclerometer and gyroscope values 
            Ax = AccelReadings[0];
            Ay = AccelReadings[1];
            Az = AccelReadings[2]; 
            Gx = GyroReadings[0];
            Gy = GyroReadings[1];
            Gz = GyroReadings[2];
                        
            // Add every reading to the sum variables     
            Ax_f_sum = Ax_f_sum + (float)Ax;
            Ay_f_sum = Ay_f_sum + (float)Ay;
            Az_f_sum = Az_f_sum + (float)Az;
            Gx_f_sum = Gx_f_sum + (float)Gx;
            Gy_f_sum = Gy_f_sum + (float)Gy;
            Gz_f_sum = Gz_f_sum + (float)Gz;
        }
        
        // Divide by 10 to get the averaged value
        Ax_f = Ax_f_sum / 10;
        Ay_f = Ay_f_sum / 10;
        Az_f = Az_f_sum / 10;
        Gx_f = Gx_f_sum / 10;
        Gy_f = Gy_f_sum / 10;
        Gz_f = Gz_f_sum / 10;
         
        // 1. Calculate actual roll, pitch and yaw angles in degrees
        // 2. Calibrate readings by adding or substracting the off-set
        roll = (180/pi)*(atan(Ax_f/(sqrt((Ay_f*Ay_f)+(Az_f*Az_f))))) - 4.36;
        pitch = (180/pi)*(atan(Ay_f/(sqrt((Ax_f*Ax_f)+(Az_f*Az_f))))) - 0.063;
        yaw = (180/pi)*(atan((sqrt((Ax_f*Ax_f)+(Ay_f*Ay_f)))/Az_f)) - 3.93;
        
        // Convert gyroscope readings into degrees/s
        Gx_f = Gx_f / 131.0;
        Gy_f = Gy_f / 131.0;
        Gz_f = Gz_f / 131.0;
        pc.printf("Gyro(deg/s) X: %.3f Y: %.3f Z: %.3f || Accel(deg) Roll: %.3f, Pitch: %.3f, Yaw: %.3f \r\n", Gx_f, Gy_f, Gz_f, roll, pitch, yaw);
        bluetooth.printf("Gyro(deg/s) X: %.3f Y: %.3f Z: %.3f || Accel(deg) Roll: %.3f, Pitch: %.3f, Yaw: %.3f \n", Gx_f, Gy_f, Gz_f, roll, pitch, yaw);
        
    }
}

