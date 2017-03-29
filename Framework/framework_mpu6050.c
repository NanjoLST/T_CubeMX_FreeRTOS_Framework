#include "framework_mpu6050.h"
#include "framework_mpu6050address.h"

#include "cmsis_os.h"
#include "i2c.h"
#include "framework_debug.h"
#include "framework_iopool.h"

/*****Begin define ioPool*****/
#define IOPoolName0 mpuI2CIOPool 
#define DataType struct{uint8_t ch[20];}
#define DataPoolInit {0}
#define ReadPoolSize 1
#define ReadPoolMap {0}
#define GetIdFunc 0 
#define ReadPoolInit {0, Empty, 1}

DefineIOPool(IOPoolName0, DataType, DataPoolInit, ReadPoolSize, ReadPoolMap, GetIdFunc, ReadPoolInit);
	
#undef DataType
#undef DataPoolInit 
#undef ReadPoolSize 
#undef ReadPoolMap
#undef GetIdFunc
#undef ReadPoolInit
/*****End define ioPool*****/

#define mpuI2C hi2c1

void I2C_Error_Handler(I2C_HandleTypeDef *hi2c){
//	fw_printfln("I2C_Error_Handler");
//	HAL_I2C_DeInit(hi2c);//�ͷ�IO��ΪGPIO,��λ���״̬��־
//	HAL_I2C_Init(hi2c);//���³�ʼ��I2C������
}

void IIC_WriteData(uint8_t dev_addr, uint8_t reg_addr, uint8_t data){
	if(HAL_I2C_Mem_Write(&mpuI2C, dev_addr, reg_addr, 1, &data, 1, 5) != HAL_OK){
		//fw_printfln("WriteData_Error");
		I2C_Error_Handler(&mpuI2C);
		if(HAL_I2C_Mem_Write(&mpuI2C, dev_addr, reg_addr, 1, &data, 1, 5) != HAL_OK){
			Error_Handler();
		}
	}
}	

void mpu6050Init(void){
	//MPU6050 Init
	//fw_printfln("MPU6050 init");
	uint8_t readBuff = 0;
	if(HAL_I2C_Mem_Read(&mpuI2C, MPU6050_DEVICE_ADDRESS, WHO_AM_I, 1, &readBuff, 1, 5) != HAL_OK){
		//fw_printfln("MPU6050_WHO_AM_I");
		I2C_Error_Handler(&mpuI2C);
		if(HAL_I2C_Mem_Read(&mpuI2C, MPU6050_DEVICE_ADDRESS, WHO_AM_I, 1, &readBuff, 1, 5) != HAL_OK){
			Error_Handler();
		}
	}
	//fw_printfln("MPU6050_WHO_AM_I success");
	if(readBuff != MPU6050_ID){
		Error_Handler();
	}
	//fw_printfln("MPU6050_wrtieData");
	IIC_WriteData(MPU6050_DEVICE_ADDRESS,PWR_MGMT_1,0x01);
	IIC_WriteData(MPU6050_DEVICE_ADDRESS,CONFIG,0x03);
	IIC_WriteData(MPU6050_DEVICE_ADDRESS,GYRO_CONFIG,0x10);
	IIC_WriteData(MPU6050_DEVICE_ADDRESS,ACCEL_CONFIG,0x00);
	IIC_WriteData(MPU6050_DEVICE_ADDRESS,INT_PIN_CFG,0x02);
	IIC_WriteData(MPU6050_DEVICE_ADDRESS,INT_ENABLE,0x00);
	IIC_WriteData(MPU6050_DEVICE_ADDRESS,MPU6050_RA_USER_CTRL,0x00);
	
	IIC_WriteData(MPU6050_DEVICE_ADDRESS,SMPLRT_DIV,0x01);
	IIC_WriteData(MPU6050_DEVICE_ADDRESS,INT_ENABLE,0x01);
	//fw_printfln("MPU6050_wrtieData success");
	
	//HMC5883 Init
	//IIC_ReadData(MPU6050_DEVICE_ADDRESS,WHO_AM_I,&temp_data,1)
	//MPU6050_ReadData(uint8_t Slave_Addr, uint8_t Reg_Addr, uint8_t * Data, uint8_t Num)
	//IIC_ReadData(Slave_Addr,Reg_Addr,Data,Num)
	//MPU6050_ReadData(HMC5883_ADDRESS, HMC58X3_R_IDA, &tmp_ch, 1)
	//IIC_ReadData(HMC5883_ADDRESS,HMC58X3_R_IDA,tmp_ch,1)
	//fw_printfln("HMC5883_WHO_AM_I");
  if(HAL_I2C_Mem_Read(&mpuI2C, HMC5883_ADDRESS, HMC58X3_R_IDA, 1, &readBuff, 1, 5) != HAL_OK){
		//fw_printfln("HMC5883_WHO_AM_I_Error");
		I2C_Error_Handler(&mpuI2C);
		if(HAL_I2C_Mem_Read(&mpuI2C, HMC5883_ADDRESS, HMC58X3_R_IDA, 1, &readBuff, 1, 5) != HAL_OK){
			Error_Handler();
		}
	}
	//fw_printfln("HMC5883_WHO_AM_I success");
	if(readBuff != HMC5883_DEVICE_ID_A){
		Error_Handler();
	}
	//fw_printfln("HMC5883_WHO_AM_I device success");
	IIC_WriteData(HMC5883_ADDRESS, HMC58X3_R_CONFA,0x70);
	//osDelay(5);
	IIC_WriteData(HMC5883_ADDRESS, HMC58X3_R_CONFB,0xA0);
	//osDelay(5);
	IIC_WriteData(HMC5883_ADDRESS, HMC58X3_R_MODE,0x00);    //�����ʼ��Ϊ0x00 ����ģʽ
	//wait the response of the hmc5883 stabalizes, 6 milliseconds 
	//osDelay(6);
	//set DOR
	IIC_WriteData(HMC5883_ADDRESS, HMC58X3_R_CONFA,6<<2);   //75HZ����
	
	fw_printfln("success MPU6050 init");
}


float q0 = 1.0f;
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;
float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
//��ʼ��IMU����
#define BOARD_DOWN 1   //�������泯�°ڷ�

#include "math.h"
void Init_Quaternion()//���ݲ������ݣ���ʼ��q0,q1,q2.q3���Ӷ��ӿ������ٶ�
{
	int16_t hx,hy,hz;
	//fw_printfln("wait for mpuI2CIOPool");
	if(HAL_I2C_Mem_Read(&mpuI2C, HMC5883_ADDRESS, HMC58X3_R_XM, 1, 
			IOPool_pGetWriteData(mpuI2CIOPool)->ch + 14, 6, 100) != HAL_OK){
				fw_Error_Handler();
//			I2C_Error_Handler(&mpuI2C);
//				
//			if(HAL_I2C_Mem_Read_DMA(&mpuI2C, HMC5883_ADDRESS, HMC58X3_R_XM, 1, 
//			IOPool_pGetWriteData(mpuI2CIOPool)->ch + 14, 6) != HAL_OK){
//				Error_Handler();
//			}
		}
	IOPool_getNextWrite(mpuI2CIOPool);
	//fw_printfln("??get mpuI2CIOPool??");
	//fw_printfln("begin mpuI2CIOPool getnextread");
	IOPool_getNextRead(mpuI2CIOPool, 0);
	//fw_printfln("IOPool_getNextRead");
	uint8_t *pData = IOPool_pGetReadData(mpuI2CIOPool, 0)->ch;
	//	fw_printfln("pData =");
	hx = (int16_t)(((int16_t)pData[14]) << 8) | pData[15];
	hy = (int16_t)(((int16_t)pData[16]) << 8) | pData[17];
	hz = (int16_t)(((int16_t)pData[18]) << 8) | pData[19];
		//fw_printfln("hz");
	if(hz){}
	
	//fw_printfln("b");
	#ifdef BOARD_DOWN
	if(hx<0 && hy <0)   //OK
	{
		if(fabs((float)hx/hy)>=1)
		{
			q0 = -0.005;
			q1 = -0.199;
			q2 = 0.979;
			q3 = -0.0089;
		}
		else
		{
			q0 = -0.008;
			q1 = -0.555;
			q2 = 0.83;
			q3 = -0.002;
		}
		
	}
	else if (hx<0 && hy > 0) //OK
	{
		if(fabs((float)hx/hy)>=1)   
		{
			q0 = 0.005;
			q1 = -0.199;
			q2 = -0.978;
			q3 = 0.012;
		}
		else
		{
			q0 = 0.005;
			q1 = -0.553;
			q2 = -0.83;
			q3 = -0.0023;
		}
		
	}
	else if (hx > 0 && hy > 0)   //OK
	{
		if(fabs((float)hx/hy)>=1)
		{
			q0 = 0.0012;
			q1 = -0.978;
			q2 = -0.199;
			q3 = -0.005;
		}
		else
		{
			q0 = 0.0023;
			q1 = -0.83;
			q2 = -0.553;
			q3 = 0.0023;
		}
		
	}
	else if (hx > 0 && hy < 0)     //OK
	{
		if(fabs((float)hx/hy)>=1)
		{
			q0 = 0.0025;
			q1 = 0.978;
			q2 = -0.199;
			q3 = 0.008;			
		}
		else
		{
			q0 = 0.0025;
			q1 = 0.83;
			q2 = -0.56;
			q3 = 0.0045;
		}		
	}
	#else
		if(hx<0 && hy <0)
	{
		if(fabs((float)hx/hy)>=1)
		{
			q0 = 0.195;
			q1 = -0.015;
			q2 = 0.0043;
			q3 = 0.979;
		}
		else
		{
			q0 = 0.555;
			q1 = -0.015;
			q2 = 0.006;
			q3 = 0.829;
		}
		
	}
	else if (hx<0 && hy > 0)
	{
		if(fabs((float)hx/hy)>=1)
		{
			q0 = -0.193;
			q1 = -0.009;
			q2 = -0.006;
			q3 = 0.979;
		}
		else
		{
			q0 = -0.552;
			q1 = -0.0048;
			q2 = -0.0115;
			q3 = 0.8313;
		}
		
	}
	else if (hx>0 && hy > 0)
	{
		if(fabs((float)hx/hy)>=1)
		{
			q0 = -0.9785;
			q1 = 0.008;
			q2 = -0.02;
			q3 = 0.195;
		}
		else
		{
			q0 = -0.9828;
			q1 = 0.002;
			q2 = -0.0167;
			q3 = 0.5557;
		}
		
	}
	else if (hx > 0 && hy < 0)
	{
		if(fabs((float)hx/hy)>=1)
		{
			q0 = -0.979;
			q1 = 0.0116;
			q2 = -0.0167;
			q3 = -0.195;			
		}
		else
		{
			q0 = -0.83;
			q1 = 0.014;
			q2 = -0.012;
			q3 = -0.556;
		}		
	}
	#endif
	
	//����hx hy hz���ж�q��ֵ��ȡ�ĸ������ֵ���ƽ�����,��ʼֵ������ŷ����ת������Ԫ������õ�
	 //fw_printfln("Init_Quaternion finish");
}
uint32_t Get_Time_Micros(void);
float gx, gy, gz, ax, ay, az, mx, my, mz;
float gYroX, gYroY, gYroZ;
void printMPU6050Task(void const * argument){
	while(1){
		if(IOPool_hasNextRead(mpuI2CIOPool, 0)){
			IOPool_getNextRead(mpuI2CIOPool, 0);
			uint8_t *pData = IOPool_pGetReadData(mpuI2CIOPool, 0)->ch;
			
			int16_t myax = (int16_t)(((int16_t)pData[0]) << 8) | pData[1];
			int16_t myay = (int16_t)(((int16_t)pData[2]) << 8) | pData[3];
			int16_t myaz = (int16_t)(((int16_t)pData[4]) << 8) | pData[5];

			int16_t mygx = (int16_t)(((int16_t)pData[8]) << 8) | pData[9];
			int16_t mygy = (int16_t)(((int16_t)pData[10]) << 8) | pData[11];
			int16_t mygz = (int16_t)(((int16_t)pData[12]) << 8) | pData[13];
			
			int16_t mymy = (int16_t)(((int16_t)pData[14]) << 8) | pData[15];
			int16_t mymz = (int16_t)(((int16_t)pData[16]) << 8) | pData[17];
			int16_t mymx = (int16_t)(((int16_t)pData[18]) << 8) | pData[19];
			
//			static int16_t mymaxmx = -30000, myminmx = 30000;
//			static int16_t mymaxmy = -30000, myminmy = 30000;
//			static int16_t mymaxmz = -30000, myminmz = 30000;
//			
//			if(mymx > mymaxmx)mymaxmx = mymx;
//			if(mymx < myminmx)myminmx = mymx;
//			if(mymy > mymaxmy)mymaxmy = mymy;
//			if(mymy < myminmy)myminmy = mymy;
//			if(mymz > mymaxmz)mymaxmz = mymz;
//			if(mymz < myminmz)myminmz = mymz;

			
			float mygetqval[9];
			mygetqval[0] = (float)myax;
			mygetqval[1] = (float)myay;
			mygetqval[2] = (float)myaz;
			
			mygetqval[3] = (float)mygx / 32.8f;
			mygetqval[4] = (float)mygy / 32.8f;
			mygetqval[5] = (float)mygz / 32.8f;
			
			mygetqval[6] = (float)mymx - 38.0;
			mygetqval[7] = (float)mymy - 102.5;
			mygetqval[8] = (float)mymz - 15.5;
			
			gYroX = mygetqval[3];
			gYroY = mygetqval[4] - (-0.5);
			gYroZ = mygetqval[5];
			
#define Kp 2.0f
#define Ki 0.01f 
#define M_PI  (float)3.1415926535
			static uint32_t lastUpdate, now;
			static float exInt, eyInt, ezInt;

			float norm;
			float hx, hy, hz, bx, bz;
			float vx, vy, vz, wx, wy, wz;
			float ex, ey, ez, halfT;
			float tempq0,tempq1,tempq2,tempq3;

			float q0q0 = q0*q0;
			float q0q1 = q0*q1;
			float q0q2 = q0*q2;
			float q0q3 = q0*q3;
			float q1q1 = q1*q1;
			float q1q2 = q1*q2;
			float q1q3 = q1*q3;
			float q2q2 = q2*q2;   
			float q2q3 = q2*q3;
			float q3q3 = q3*q3;   

			gx = mygetqval[3] * M_PI/180;
			gy = mygetqval[4] * M_PI/180;
			gz = mygetqval[5] * M_PI/180;
			ax = mygetqval[0];
			ay = mygetqval[1];
			az = mygetqval[2];
			mx = mygetqval[6];
			my = mygetqval[7];
			mz = mygetqval[8];		

			now = Get_Time_Micros();  //��ȡʱ�� ��λ��us   
			if(now<lastUpdate)
			{
			//halfT =  ((float)(now + (0xffffffff- lastUpdate)) / 2000000.0f);   //  uint 0.5s
			}
			else	
			{
					halfT =  ((float)(now - lastUpdate) / 2000000.0f);
			}
			lastUpdate = now;	//����ʱ��
			//������ƽ�����㷨
			norm = invSqrt(ax*ax + ay*ay + az*az);       
			ax = ax * norm;
			ay = ay * norm;
			az = az * norm;
			//�ѼӼƵ���ά����ת�ɵ�λ������
			norm = invSqrt(mx*mx + my*my + mz*mz);          
			mx = mx * norm;
			my = my * norm;
			mz = mz * norm; 
			// compute reference direction of flux
			hx = 2.0f*mx*(0.5f - q2q2 - q3q3) + 2.0f*my*(q1q2 - q0q3) + 2.0f*mz*(q1q3 + q0q2);
			hy = 2.0f*mx*(q1q2 + q0q3) + 2.0f*my*(0.5f - q1q1 - q3q3) + 2.0f*mz*(q2q3 - q0q1);
			hz = 2.0f*mx*(q1q3 - q0q2) + 2.0f*my*(q2q3 + q0q1) + 2.0f*mz*(0.5f - q1q1 - q2q2);         
			bx = sqrt((hx*hx) + (hy*hy));
			bz = hz; 
			// estimated direction of gravity and flux (v and w)
			vx = 2.0f*(q1q3 - q0q2);
			vy = 2.0f*(q0q1 + q2q3);
			vz = q0q0 - q1q1 - q2q2 + q3q3;
			wx = 2.0f*bx*(0.5f - q2q2 - q3q3) + 2.0f*bz*(q1q3 - q0q2);
			wy = 2.0f*bx*(q1q2 - q0q3) + 2.0f*bz*(q0q1 + q2q3);
			wz = 2.0f*bx*(q0q2 + q1q3) + 2.0f*bz*(0.5f - q1q1 - q2q2);  
			// error is sum of cross product between reference direction of fields and direction measured by sensors
			ex = (ay*vz - az*vy) + (my*wz - mz*wy);
			ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
			ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

			if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
			{
					exInt = exInt + ex * Ki * halfT;
					eyInt = eyInt + ey * Ki * halfT;	
					ezInt = ezInt + ez * Ki * halfT;
					// �ò���������PI����������ƫ
					gx = gx + Kp*ex + exInt;
					gy = gy + Kp*ey + eyInt;
					gz = gz + Kp*ez + ezInt;
			}
			// ��Ԫ��΢�ַ���
			tempq0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
			tempq1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
			tempq2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
			tempq3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  

			// ��Ԫ���淶��
			norm = invSqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
			q0 = tempq0 * norm;
			q1 = tempq1 * norm;
			q2 = tempq2 * norm;
			q3 = tempq3 * norm;
			
			float q[4];
			q[0] = q0; //���ص�ǰֵ
			q[1] = q1;
			q[2] = q2;
			q[3] = q3;
			float angles[3];
			angles[0] = -atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2]*q[2] - 2 * q[3] * q[3] + 1)* 180/M_PI; // yaw        -pi----pi
			angles[1] = -asin(-2 * q[1] * q[3] + 2 * q[0] * q[2])* 180/M_PI; // pitch    -pi/2    --- pi/2 
			angles[2] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1)* 180/M_PI; // roll       -pi-----pi  

			static int countPrint = 0;
			if(countPrint > 50){
				countPrint = 0;
				
//				fw_printf("mx max = %d | min = %d\r\n", mymaxmx, myminmx);
//				fw_printf("my max = %d | min = %d\r\n", mymaxmy, myminmy);
//				fw_printf("mz max = %d | min = %d\r\n", mymaxmz, myminmz);
//				fw_printf("========================\r\n");
				
//				fw_printf("now = %d \r\n", now);
//				fw_printf("xxx = %d \r\n", 2147483647);
//				fw_printf("halfT = %f \r\n", halfT);
				static float last_yaw_temp, yaw_temp;
				static int yaw_count = 0;
				last_yaw_temp = yaw_temp;
				yaw_temp = angles[0]; 
				if(yaw_temp-last_yaw_temp>=330)  //yaw��ǶȾ����������������
				{
					yaw_count--;
				}
				else if (yaw_temp-last_yaw_temp<=-330)
				{
					yaw_count++;
				}
				float yaw_angle = yaw_temp + yaw_count*360;
				
//				fw_printf("yaw_angle = %f | ", yaw_angle);
//				fw_printf("angles0 = %f | ", angles[0]);
//				fw_printf("angles1 = %f | ", angles[1]);
//				fw_printf("angles2 = %f\r\n", angles[2]);
//				fw_printf("========================\r\n");
				
//				fw_printf("ax = %d | ", myax);
//				fw_printf("ay = %d | ", myay);
//				fw_printf("az = %d\r\n", myaz);
//				
//				fw_printf("gx = %d | ", mygx);
//				fw_printf("gy = %d | ", mygy);
//				fw_printf("gz = %d\r\n", mygz);
			
//				fw_printf("mx = %d | ", mymx);
//				fw_printf("my = %d | ", mymy);
//				fw_printf("mz = %d\r\n", mymz);
//				fw_printf("========================\r\n");
			}else{
				countPrint++;
			}

//			fw_printf("ax = %d | ", ax);
//			fw_printf("ay = %d | ", ay);
//			fw_printf("az = %d\r\n", az);
//			
//			fw_printf("gx = %d | ", gx);
//			fw_printf("gy = %d | ", gy);
//			fw_printf("gz = %d\r\n", gz);
//			
//			fw_printf("mx = %d | ", mx);
//			fw_printf("my = %d | ", my);
//			fw_printf("mz = %d\r\n", mz);
//			fw_printf("========================\r\n");
		}
		//osDelay(500);
	}
}

extern osSemaphoreId readMPU6050SemaphoreHandle;
extern osSemaphoreId refreshMPU6050SemaphoreHandle;
void readMPU6050Task(void const * argument){
	while(1){
		//fw_printfln("wait refresh");
		osSemaphoreWait(refreshMPU6050SemaphoreHandle, osWaitForever);
		//fw_printfln("wait read");
		osSemaphoreWait(readMPU6050SemaphoreHandle, osWaitForever);
		//fw_printfln("begin read dma");
		if(HAL_I2C_Mem_Read_DMA(&mpuI2C, MPU6050_DEVICE_ADDRESS, MPU6050_DATA_START, 1, 
			IOPool_pGetWriteData(mpuI2CIOPool)->ch, 14) != HAL_OK){
			I2C_Error_Handler(&mpuI2C);
				
			if(HAL_I2C_Mem_Read_DMA(&mpuI2C, MPU6050_DEVICE_ADDRESS, MPU6050_DATA_START, 1, 
			IOPool_pGetWriteData(mpuI2CIOPool)->ch, 14) != HAL_OK){
				Error_Handler();
			}
		}
		//fw_printfln("wait read2");
		osSemaphoreWait(readMPU6050SemaphoreHandle, osWaitForever);
		//fw_printfln("begin read dma2");
		if(HAL_I2C_Mem_Read_DMA(&mpuI2C, HMC5883_ADDRESS, HMC58X3_R_XM, 1, 
			IOPool_pGetWriteData(mpuI2CIOPool)->ch + 14, 6) != HAL_OK){
			I2C_Error_Handler(&mpuI2C);
				
			if(HAL_I2C_Mem_Read_DMA(&mpuI2C, HMC5883_ADDRESS, HMC58X3_R_XM, 1, 
			IOPool_pGetWriteData(mpuI2CIOPool)->ch + 14, 6) != HAL_OK){
				Error_Handler();
			}
		}
		IOPool_getNextWrite(mpuI2CIOPool);
	}
}


void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
	//IOPool_getNextWrite(mpuI2CIOPool);
	//fw_printfln("release read");
	osSemaphoreRelease(readMPU6050SemaphoreHandle);
//	if(HAL_I2C_Mem_Read_DMA(hi2c, MPU6050_DEVICE_ADDRESS, MPU6050_DATA_START, 1, 
//		IOPool_pGetWriteData(mpuI2CIOPool)->ch, 14) != HAL_OK){
//		Error_Handler();
//	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_4){
		//fw_printfln("release refresh");
		osSemaphoreRelease(refreshMPU6050SemaphoreHandle);
	}
}
