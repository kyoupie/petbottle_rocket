#include <Wire.h>

// MPU-6050のアドレス、レジスタ設定値
#define MPU6050_WHO_AM_I     0x75  // Read Only
#define MPU6050_PWR_MGMT_1   0x6B  // Read and Write
#define MPU_ADDRESS  0x68

int16_t axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw, Temperature;//生値たち
uint32_t timer;//時間図るやつ

// デバイス初期化時に実行される
void setup() {//まだ±16にできてないかもです
  Wire.begin();

  // PCとの通信を開始
  Serial.begin(115200); //115200bps
 
  // 初回の読み出し
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(MPU6050_WHO_AM_I);  //MPU6050_PWR_MGMT_1
  Wire.write(0x00);
  Wire.endTransmission();

  // 動作モードの読み出し
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(MPU6050_PWR_MGMT_1);  //MPU6050_PWR_MGMT_1レジスタの設定
  Wire.write(0x00);
  Wire.endTransmission();
  timer=micros();
}

double gravX=0,gravY=0,gravZ=0;//重力の成分
const double k=0.8;//ローパス用の定数
double velx=0,vely=0,velz=0;//成分ごとの速度
double X=0,Y=0,Z=0;//成分ごとの変位
bool flag=false;//初期の加速度が落ち着いたかどうか(初期化フラグ)
double prevaccX=0,prevaccY=0,prevaccZ=0;//前回の加速度

//繰り返し
void loop() {
  //while (Wire.available() < 14);
  double sumAx=0,sumAy=0,sumAz=0;//加速度の和
  double aveAx=0,aveAy=0,aveAz=0;//加速度の平均
  double axRaw2=0,ayRaw2=0,azRaw2=0;//加速度の生値(double)
  int tmp=0;//カウント用変数
  
  while(tmp<10){//10回サンプルをとる
  
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 14, true);
    axRaw = Wire.read() << 8 | Wire.read();
    ayRaw = Wire.read() << 8 | Wire.read();
    azRaw = Wire.read() << 8 | Wire.read();

    axRaw2=axRaw/16384.0,ayRaw2=ayRaw/16384.0,azRaw2=azRaw/16384.0;
    sumAx+=axRaw2*9.8,sumAy+=ayRaw2*9.8,sumAz+=azRaw2*9.8;//かける9.8する？    
    tmp++;
    delay(2);
  }

  double dt=(double)(micros()-timer)/1000000;//dtを設定(0.04だった)
  timer=micros();
  
  aveAx=sumAx/10.0,aveAy=sumAy/10.0,aveAz=sumAz/10.0;//平均

  gravX=(1-k)*aveAx+k*gravX;//ローパスフィルタ
  gravY=(1-k)*aveAy+k*gravY;
  gravZ=(1-k)*aveAz+k*gravZ;
  
  double accX=aveAx-gravX,accY=aveAy-gravY,accZ=aveAz-gravZ;//gを取り除く
  
  if(abs(accX)<0.1&&abs(accY)<0.1&&abs(accZ)<0.1){//初期ブレ防止
    flag=true;
  }
  
  if(flag){//初期ブレしなくなったら必ず通す

    velx=velx+(accX+prevaccX)/2*dt;//台形公式
    vely=vely+(accY+prevaccY)/2*dt;
    velz=velz+(accZ+prevaccZ)/2*dt;

  }//いずれも、x軸の速度がずっと負になるバグ?が発生してしまう...(解決済)
  
  prevaccX=accX;//前回の値を更新
  prevaccY=accY;
  prevaccZ=accZ;
  
  Serial.print(velx);  Serial.print(",");
  Serial.print(vely);  Serial.print(",");
  Serial.print(velz);  Serial.println(",");

  delay(2);
}