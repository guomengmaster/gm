/* IMU.c file
��д��:
���뻷��:MDK ��Version: 5.17
����ʱ��:
����:����������I9DOF����ɲ���
����:��̬����IMU�������������ֵ������̬���㣬�õ�Ŀ������ĸ����Ǻͺ���Ǻͺ����
------------------------------------
*/

#include "IMU.h"

float  pitch ,roll ,yaw;
volatile float exInt, eyInt, ezInt;			//������
volatile float q0, q1, q2, q3,w1,w2,w3; //ȫ����Ԫ����������������
volatile float bx,by,bz;								//������ָ��ʱx�ᡢy�ᡢz��Ĵų�����
volatile float integralFBhand,handdiff;
volatile uint32_t lastUpdate, now;			//�������ڼ�������λ us
float mygetqval[9];											//���ڴ�Ŵ�����ת�����������
extern float Accel_X,  									//���ٶ�X��, ��λg [9.8m/S^2]
             Accel_Y,										//���ٶ�Y��, ��λg [9.8m/S^2]
             Accel_Z;										//���ٶ�Z��, ��λg [9.8m/S^2]

float P[49]={
						0.0001,0,0,0,0,0,0,
						0,0.0001,0,0,0,0,0,
						0,0,0.0001,0,0,0,0,
						0,0,0,0.0001,0,0,0,
						0,0,0,0,0.0002,0,0,
						0,0,0,0,0,0.0002,0,
						0,0,0,0,0,0,0.0002
						};

float Q[49]={
						0.0001,0,0,0,0,0,0,
            0,0.0001,0,0,0,0,0,
						0,0,0.0001,0,0,0,0,
						0,0,0,0.0001,0,0,0,
						0,0,0,0,0.0001,0,0,
						0,0,0,0,0,0.0001,0,
						0,0,0,0,0,0,0.0001
						};
			    
float R[36]={
						0.0002,0,0,0,0,0,
            0,0.0002,0,0,0,0,
						0,0,0.0002,0,0,0,
						0,0,0,0.0001,0,0,
						0,0,0,0,0.0001,0,
						0,0,0,0,0,0.0001
						};
			   		
float A[49],B[49],E[42],F1[36],X[49],Z[49],Ht[42],Ft[49],K[42],O[49],T[6],F[49],Y[7],P1[49],U1[36],U1t[36],D1[36],X1[36],X2[36];
float H[42]={
						0,0,0,0,0,0,0,
						0,0,0,0,0,0,0,
						0,0,0,0,0,0,0,
						0,0,0,0,0,0,0,
						0,0,0,0,0,0,0,													   												  
						0,0,0,0,0,0,0,
						};
float I[49]={
						1,0,0,0,0,0,0,
						0,1,0,0,0,0,0,
						0,0,1,0,0,0,0,
						0,0,0,1,0,0,0,
						0,0,0,0,1,0,0,
						0,0,0,0,0,1,0,
						0,0,0,0,0,0,1
						};

/**************************ʵ�ֺ���********************************************
*����ԭ��:	void IMU_init(void)
*��������:	��ʼ��IMU���	
					��ʼ������������
					��ʼ����Ԫ��
					����������
					����ϵͳʱ��
*�������:	��
*�������:	��
*******************************************************************************/
void IMU_init(void)
{	 
	int i;
	for(i=0;i<100;i++)
	Dof6_Update();
	HMC5983L_SetUp();
	now = micros();		//��ȡʱ��
  lastUpdate = now;	//����ʱ��
	//��ʼ����Ԫ��
	q0 = 1.0f;
	q1 = 0.0f;
	q2 = 0.0f;
	q3 = 0.0f;
	exInt = 0.0;
	eyInt = 0.0;
	ezInt = 0.0;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	void AHRS_init(void)
*��������:	��ʼ��IMU���	
					��ʼ������������
					��ʼ����Ԫ��
					����������
					����ϵͳʱ��
*�������:	��
*�������:	��
*******************************************************************************/
void AHRS_init(void)
{
	int i;
	for(i=0;i<100;i++)
	Dof6_Update();
	HMC5983L_SetUp();
	now = micros();		//��ȡʱ��
  lastUpdate = now;	//����ʱ��
	//��ʼ����Ԫ��
	q0=1.0f;
	q1=0.0f;
	q2=0.0f;
	q3=0.0f;
	//������ƫ��
	w1=0.0;
	w2=0.0;
	w3=0.0;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	void AHRS_getValues(float * values)
*��������:	��ȡ���ٶȼơ������ǡ������Ƶĵ�ǰֵ
*�������:	�������ŵ������׵�ַ
*�������:	��
*******************************************************************************/
void AHRS_getValues(float * values)
{  
	//��ȡ���ٶȺ������ǵĵ�ǰADC
	Dof6_Update();
	
	values[0] = Accel_Xint;
	values[1] = Accel_Yint;
	values[2] = Accel_Zint;
	
	values[3] = Gyro_X;							//���ٶ�X�ᣬ��λdps [��ÿ��]
	values[4] = Gyro_Y;
	values[5] = Gyro_Z;
  HMC5983_mgetValues(&values[6]);	//��ȡ�����Ƶ�ADCֵ
}

// Fast inverse square-root
/**************************ʵ�ֺ���********************************************
*����ԭ��:	float invSqrt(float x)
*��������:	���ټ��� 1/Sqrt(x) 	
*�������:	Ҫ�����ֵ
*�������:	���
*******************************************************************************/
float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	void IMU_AHRSupdate
*��������:	����AHRS ������Ԫ�� 
*�������:	��ǰ�Ĳ���ֵ
*�������:	��
*******************************************************************************/
#define Kp 2.0f   // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.01f   // integral gain governs rate of convergence of gyroscope biases

void IMU_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
  float norm;
  float hx, hy, hz, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez,halfT;
  float tempq0,tempq1,tempq2,tempq3;
	
  // �Ȱ���Щ�õõ���ֵ���
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
  
  now = micros();	//��ȡʱ��
  if(now<lastUpdate)
	{	//��ʱ���������
		halfT =  ((float)(now + (0xffff- lastUpdate)) / 2000000.0f);
  }
  else
	{
		halfT =  ((float)(now - lastUpdate) / 2000000.0f);
  }
  lastUpdate = now;	//����ʱ��
	
  norm = invSqrt(ax*ax + ay*ay + az*az);
  ax = ax * norm;
  ay = ay * norm;
  az = az * norm;
  //�ѼӼƵ���ά����ת�ɵ�λ����
  norm = invSqrt(mx*mx + my*my + mz*mz);
  mx = mx * norm;
  my = my * norm;
  mz = mz * norm;
	
  /*
  ���ǰ���Ԫ������ɡ��������Ҿ����еĵ����е�����Ԫ�ء�
	�������Ҿ����ŷ���ǵĶ��壬��������ϵ������������ת����������ϵ��������������Ԫ�ء�
	���������vx\y\z����ʵ���ǵ�ǰ��ŷ���ǣ�����Ԫ�����Ļ����������ϵ�ϣ����������������λ������
  */
  // compute reference direction of flux
  hx = 2*mx*(0.5f - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
  hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5f - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
  hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5f - q1q1 - q2q2);         
  bx = sqrt((hx*hx) + (hy*hy));
  bz = hz;     
  
  // estimated direction of gravity and flux (v and w)
  vx = 2*(q1q3 - q0q2);
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;
  wx = 2*bx*(0.5f - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
  wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
  wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5f - q1q1 - q2q2);  
  
  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay*vz - az*vy) + (my*wz - mz*wy);
  ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
  ez = (ax*vy - ay*vx) + (mx*wy - my*wx);
	
  /*
  axyz�ǻ����������ϵ�ϣ����ٶȼƲ����������������Ҳ����ʵ�ʲ����������������
	axyz�ǲ����õ�������������vxyz�����ݻ��ֺ����̬����������������������Ƕ��ǻ����������ϵ�ϵ�����������
	������֮�������������������ݻ��ֺ����̬�ͼӼƲ��������̬֮�����
	������������������������Ҳ�������������ˣ�����ʾ��exyz�����������������Ĳ����
	�����������Ծ���λ�ڻ�������ϵ�ϵģ������ݻ������Ҳ���ڻ�������ϵ�����Ҳ���Ĵ�С�����ݻ����������ȣ����������������ݡ���������Լ��ö�������һ�£����������ǶԻ���ֱ�ӻ��֣����Զ����ݵľ�������ֱ�������ڶԻ�������ϵ�ľ�����
  */
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
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	void AHRS_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
*��������:	����AHRS ������Ԫ�� 
*�������:	��ǰ�Ĳ���ֵ
*�������:	��
*******************************************************************************/
void AHRS_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
  float norm;
  float hx, hy, hz;												//��ת��nϵ�ų�����ͶӰ
	
  float vx, vy, vz, wx, wy, wz;						//��������ʹų�����
	
  float g=9.79973;
  float Ha1,Ha2,Ha3,Ha4,Hb1,Hb2,Hb3,Hb4;	//H����Ԫ��
  float e1,e2,e3,e4,e5,e6;								//��Ϣ
  float halfT;														//����ʱ������һ��
	float Accel_Quadratic_Sum;							//������ٶ�ֵ��ƽ����
	
  //�Ȱ���Щ�õõ���ֵ���
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
	
/******************************************************************************/
	
  now = micros();		//��ȡʱ��
  if(now<lastUpdate)
	{	//��ʱ���������
		halfT =  ((float)(now + (0xffff - lastUpdate)) / 2000000.0f);
	}
  else
	{
		halfT =  ((float)(now - lastUpdate) / 2000000.0f);
	}
  lastUpdate = now;	//����ʱ��
	
	Accel_Quadratic_Sum = Accel_X*Accel_X+Accel_Y*Accel_Y+Accel_Z*Accel_Z;
	
  norm = invSqrt(ax*ax + ay*ay + az*az);
  ax = ax * norm*g;
  ay = ay * norm*g;
  az = az * norm*g;
	
  norm = invSqrt(mx*mx + my*my + mz*mz);
  mx = mx * norm;
  my = my * norm;
  mz = mz * norm;
	
	hx=2*(0.5f - q2q2 - q3q3)*mx + 2*(q1q2 - q0q3)*my + 2*(q1q3 + q0q2)*mz;
	hy=2*(q1q2 + q0q3)*mx + 2*(0.5f - q1q1 - q3q3)*my + 2*(q2q3 - q0q1)*mz;
	hz=2*(q1q3 - q0q2)*mx + 2*(q2q3 + q0q1)*my + 2*(0.5f - q1q1 - q2q2)*mz;
	
//	bx=1/invSqrt(hx*hx+hy*hy);
//	bz=hz;
	by=1/invSqrt(hx*hx + hy*hy);
	bz=hz;
  
  gx=gx - w1;
  gy=gy - w2;
  gz=gz - w3;
	
  //�������ֵ
  Ha1=(-q2)*g;
  Ha2=q3*g;
  Ha3=-q0*g;
  Ha4=q1*g;
	
  Hb1=by*q3 - bz*q2;
  Hb2=by*q2 + bz*q3;
  Hb3=by*q1 - bz*q0;
  Hb4=by*q0 + bz*q1;
	
  H[0]= Ha1;H[1]= Ha2;H[2]= Ha3;H[3]= Ha4;
  H[7]= Ha4;H[8]=-Ha3;H[9]= Ha2;H[10]=-Ha1;
  H[14]=-Ha3;H[15]=-Ha4;H[16]= Ha1;H[17]= Ha2;
	
  H[21]= Hb1;H[22]= Hb2;H[23]= Hb3;H[24]= Hb4;
  H[28]= Hb4;H[29]=-Hb3;H[30]= Hb2;H[31]=-Hb1;
  H[35]=-Hb3;H[36]=-Hb4;H[37]= Hb1;H[38]= Hb2;
  
  //״̬���£���Ԫ��΢�ַ��̣�
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;
  //��Ԫ����һ��
  norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 * norm;
  q1 = q1 * norm;
  q2 = q2 * norm;
  q3 = q3 * norm;
	
/******************************************************************************/
	
  //F��ֵ���ſ˱�/һ��ת�ƾ���ո�ֵ��
  F[0]=1;F[8]=1;F[16]=1;F[24]=1;F[32]=1;F[40]=1;F[48]=1;
  F[1]=-gx*halfT;F[2]=-gy*halfT;F[3]=-gz*halfT;
	F[4]=q1*halfT; F[5]=q2*halfT; F[6]=q3*halfT;
  F[7]=gx*halfT;F[9]=gz*halfT;F[10]=-gy*halfT;
	F[11]=-q0*halfT; F[12]=q3*halfT; F[13]=-q2*halfT;
  F[14]=gy*halfT;F[15]=-gz*halfT;F[17]=gx*halfT;
	F[18]=-q3*halfT; F[19]=-q0*halfT;F[20]=q1*halfT;
  F[21]=gz*halfT;F[22]=gy*halfT;F[23]=-gx*halfT;
	F[25]=q2*halfT; F[26]=-q1*halfT; F[27]=-q0*halfT;
  F[28]=0;F[29]=0;F[30]=0;F[31]=0;F[33]=0;F[34]=0;
  F[35]=0;F[36]=0;F[37]=0;F[38]=0;F[39]=0;F[41]=0;
  F[42]=0;F[43]=0;F[44]=0;F[45]=0;F[46]=0;F[47]=0;
  //�������˲�
  MatrixMultiply(F,7,7,P,7,7,A );		//A=F*P
  MatrixTranspose(F,7,7,Ft);				//Fת��  F'
  MatrixMultiply(A,7,7,Ft,7,7,B);		//B=F*P*F'
  MatrixAdd( B,Q,P1,7,7 );
  MatrixTranspose(H,6,7,Ht);				//Hת��  H'
  MatrixMultiply(P1,7,7,Ht,7,6,E );	//E=P*H'
  MatrixMultiply(H,6,7,E,7,6,F1 );	//F1=H*P*H'	6*6
  MatrixAdd(F1,R,X,6,6 );						//X=F1+R	   6*6
  UD(X,6,U1,D1);										//X��UD�ֽ�
  MatrixTranspose(U1,6,6,U1t);			//U1��ת��
  MatrixMultiply(U1,6,6,D1,6,6,X1);	//X1=U1*D1
  MatrixMultiply(X1,6,6,U1t,6,6,X2);//X2=U1*D1*U1t
  MatrixInverse(X2,6,0);						//X2�� 
  MatrixMultiply(E,7,6,X2,6,6,K );	//����K   7*6
  //������������
  vx = 2*(q1q3 - q0q2)*g;
  vy = 2*(q0q1 + q2q3)*g;
  vz = (q0q0 - q1q1 - q2q2 + q3q3)*g;
  //�����������         
//  wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
//  wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
//  wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);
  wx = 2.0f*by*(q1q2 + q0q3) + 2.0f*bz*(q1q3 - q0q2);
  wy = 2.0f*by*(0.5f - q1q1 - q3q3) + 2.0f*bz*(q0q1 + q2q3);
  wz = 2.0f*by*(q2q3 - q0q1) + 2.0f*bz*(0.5f - q1q1 - q2q2);
	
  e1=ax-vx;e2=ay-vy;e3=az-vz;				//����ٶȸ������
  e4=mx-wx;e5=my-wy;e6=mz-wz;				//��شżƸ������
  T[0]=e1;T[1]=e2;T[2]=e3;T[3]=e4;T[4]=e5;T[5]=e6;//��������T[]
  MatrixMultiply(K,7,6,T,6,1,Y );		//Y=K*(Z-H*X)=K*T	7*1
	
	if(1/invSqrt(Accel_Quadratic_Sum)<1.12f)
	{
		q0= q0+Y[0];
		q1= q1+Y[1];
		q2= q2+Y[2];
		q3= q3+Y[3];
		w1= w1+Y[4];
		w2= w2+Y[5];
		w3= w3+Y[6];	
	}
  
  MatrixMultiply(K,7,6,H,6,7,Z);		//Z= K*H		7*7
  MatrixSub(I,Z,O,7,7 );						//O=I-K*H      7*7
  MatrixMultiply(O,7,7,P1,7,7,P);		//P=(I-K*H)*P1 7*7
	
  //��Ԫ����һ��
  norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 * norm;
  q1 = q1 * norm;
  q2 = q2 * norm;
  q3 = q3 * norm;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	void AHRS_getQ(float * q)
*��������:	������Ԫ�� ���ص�ǰ����Ԫ����ֵ
*�������:	��Ҫ�����Ԫ���������׵�ַ
*�������:	��
*******************************************************************************/
void AHRS_getQ(float * q) 
{
	AHRS_getValues(mygetqval);	 
	
	//�������ǵĲ���ֵת�ɻ���ÿ��
	//���ٶȺʹ����Ʊ���ADCֵ������Ҫת��
	AHRS_AHRSupdate(mygetqval[3] * M_PI/180, mygetqval[4] * M_PI/180, mygetqval[5] * M_PI/180,
	mygetqval[0], mygetqval[1], mygetqval[2], mygetqval[6], mygetqval[7], mygetqval[8]);
  
	q[0] = q0;	//���ص�ǰֵ
	q[1] = -q2;
	q[2] = q1;
	q[3] = q3;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	float safe_asin(float v)
*��������:	��ȫ��ⷴ����ֵ
*�������:	Ҫ���ĸ�����
*�������:	���ػ���ֵ
*******************************************************************************/
// a varient of asin() that checks the input ranges and ensures a
// valid angle as output. If nan is given as input then zero is
// returned.
float safe_asin(float v)
{
	if (isnan(v))
	{
		return 0.0f;
	}
	if (v >= 1.0f)
	{
		return M_PI/2;
	}
	if (v <= -1.0f)
	{
		return -M_PI/2;
	}
	return asin(v);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	void AHRS_getYawPitchRoll(float * angles)
*��������:	������Ԫ�� ���ص�ǰ��������̬����
*�������:	��Ҫ�����̬�ǵ������׵�ַ
*�������:	��
*******************************************************************************/
void AHRS_getYawPitchRoll(float * angles)
{
  float q[4];												//��Ԫ��
  
  AHRS_getQ(q);											//����ȫ����Ԫ��
  angles[0] = -atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2]*q[2] - 2 * q[3] * q[3] + 1)* 180/M_PI;		// yaw
  angles[1] = safe_asin(-2 * q[1] * q[3] + 2 * q[0] * q[2])* 180/M_PI;																			// pitch
  angles[2] = -atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1)* 180/M_PI;	// roll
  if(angles[0] < 0)angles[0] += 360.0f;	//�� -+180��  ת��0-360��
  yaw = angles[0];
  pitch = angles[1];
  roll = angles[2];
}

//------------------End of File----------------------------
