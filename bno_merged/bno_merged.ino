#include <Wire.h>
#include <Adafruit_BNO055.h>
Adafruit_BNO055 bno=Adafruit_BNO055(55,0x28);

struct sensorXYZ{
  double x;
  double y;
  double z;
};

//生ローカル＝＞生グローバル＝＞修正グローバル

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

const double g=9.8;
double gravX=0,gravY=0,gravZ=0;
double global_accX=0,global_accY=0,global_accZ=0;
double velx=0,vely=0,velz=0;
double x=0,y=0,z=0;
const double k=0.85;
double dt=10;

sensorXYZ get_fixed_global_acceleration(double garx,double gary,double garz){

  double accX,accY,accZ;//返り値用の変数
  
  gravX=(1-k)*garx+k*gravX;
  gravY=(1-k)*gary+k*gravY;
  gravZ=(1-k)*garz+k*gravZ;
  
  accX=garx-gravX,accY=gary-gravY,accZ=garz-gravZ;

  if(abs(accX)<0.02)accX=0;
  if(abs(accY)<0.02)accY=0;
  if(abs(accZ)<0.02)accZ=0;
  
  return sensorXYZ({accX,accY,accZ});
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

sensorXYZ get_raw_global_acceleration(double ax,double ay,double az,double roll,double pitch,double yaw){//いい感じの回転行列募集中！
  //ここに回転行列を書く
  // double grax=-sin(pitch)*g;
  // double gary=sin(roll)*cos(pitch)*g;
  // double garz=cos(roll)*cos(pitch)*g;
  // return sensorXYZ({ax,ay,az});
}

void loop() {
  // put your main code here, to run repeatedly:
  sensorXYZ rads,raw_world_accels,fixed_global_accels;
  imu::Vector<3> raw_acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  rads=get_bno055_data();
  raw_world_accels=get_raw_global_acceleration(raw_acc.x(),raw_acc.y(),raw_acc.z(),rads.x,rads.y,rads.z);
  fixed_global_accels=get_fixed_global_acceleration(raw_world_accels.x,raw_world_accels.y,raw_world_accels.z);
   Serial.print(raw_acc.x(),2);
   Serial.print(", ");
   Serial.print(raw_acc.y(),2);
   Serial.print(", ");
   Serial.print(raw_acc.z(),2);
   Serial.println();
  delay(10);
}