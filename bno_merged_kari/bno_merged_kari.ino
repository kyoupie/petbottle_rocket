#include <Wire.h>
#include <Adafruit_BNO055.h>
Adafruit_BNO055 bno=Adafruit_BNO055(55,0x28);

struct sensorXYZ{
  double x;
  double y;
  double z;
};

void setup() {
  // put your setup code here, to run once:
  pinMode(20, INPUT_PULLUP); //SDA 21番ピンのプルアップ(念のため)
  pinMode(21, INPUT_PULLUP); //SDA 22番ピンのプルアップ(念のため)
  Serial.begin(115200);
  if(!bno.begin()){
    Serial.print("Something went wrong! Please RESET it...");Serial.println("");
    while(true);
  }
  delay(2000);
}

double gravX=0,gravY=0,gravZ=0;
double global_accX=0,global_accY=0,global_accZ=0;
double velx=0,vely=0,velz=0;
double x=0,y=0,z=0;
const double k=0.85;
double dt=10;

sensorXYZ get_fixed_local_acceleration(){

  //double sumAx=0,sumAy=0,sumAz=0;//加速度の和
  //double aveAx=0,aveAy=0,aveAz=0;//加速度の平均
  double axRaw=0,ayRaw=0,azRaw=0;//加速度の生値(double)
  double local_accX,local_accY,local_accZ;//返り値用の変数
  //int tmp=0;//カウント用変数

  //while(tmp<10){//10回サンプルをとる
    imu::Vector<3> raw_acceleration = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    axRaw=raw_acceleration.x(),ayRaw=raw_acceleration.y(),azRaw=raw_acceleration.z();
  //  sumAx+=axRaw,sumAy+=ayRaw,sumAz+=azRaw;
    //tmp++;
  //  delay(2);
  //}

  //aveAx=sumAx/10.0,aveAy=sumAy/10.0,aveAz=sumAz/10.0;

  gravX=(1-k)*axRaw+k*gravX;
  gravY=(1-k)*ayRaw+k*gravY;
  gravZ=(1-k)*azRaw+k*gravZ;
  
  local_accX=axRaw-gravX,local_accY=ayRaw-gravY,local_accZ=azRaw-gravZ;
  if(abs(local_accX)<0.02)local_accX=0;
  if(abs(local_accY)<0.02)local_accY=0;
  if(abs(local_accZ)<0.02)local_accZ=0;
  //velx+=local_accX*dt;//試運転
  return sensorXYZ({local_accX,local_accY,local_accZ});
}

sensorXYZ get_bno055_data(){
  // センサフュージョンの方向推定値のクオータニオン
  double q0=0,q1=0,q2=0,q3=0;
  imu::Quaternion quat = bno.getQuat();
  q0=quat.w(),q1=quat.x(),q2=quat.y(),q3=quat.z();
  
  double t0=2*(q0*q1+q2*q3),t1=1-2*(q1*q1+q2*q2);
  double roll=atan2(t0,t1);
  
  double t2=2*(q0*q2-q3*q1);
  t2=(t2>1.0?1.0:t2);
  t2=(t2<-1.0?-1.0:t2);
  double pitch=asin(t2);

  double t3=2*(q0*q3+q1*q2),t4=1-2*(q2*q2+q3*q3);
  double yaw=atan2(t3,t4);
  //roll*=180/acos(-1),pitch*=180/acos(-1),yaw*=180/acos(-1);<-度数法にしない！！！！！！

  return sensorXYZ({roll,pitch,yaw});
}

sensorXYZ get_fixed_global_acceleration(double ax,double ay,double az,double roll,double pitch,double yaw){
  //ここに回転行列を書く
  
}

void loop() {
  // put your main code here, to run repeatedly:
  sensorXYZ local_accels,rads,world_accels;
  local_accels = get_fixed_local_acceleration();
  rads=get_bno055_data();
  world_accels=get_fixed_global_acceleration(local_accels.x,local_accels.y,local_accels.z,rads.x,rads.y,rads.z);
  Serial.print(rads.x,2);
  Serial.print(", ");
  Serial.print(rads.y,2);
  Serial.print(", ");
  Serial.print(rads.z,2);
  Serial.println();
  delay(10);
}