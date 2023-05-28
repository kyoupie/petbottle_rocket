#include <Wire.h>
#include <Adafruit_BNO055.h>
Adafruit_BNO055 bno=Adafruit_BNO055(55,0x28);
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
double local_accX=0,local_accY=0,local_accZ=0;
double global_accX=0,global_accY=0,global_accZ=0;
double velx=0,vely=0,velz=0;
double x=0,y=0,z=0;
const double k=0.85;
bool flag=false;
int waiting=0;

void get_fixed_local_acceleration(){
  double sumAx=0,sumAy=0,sumAz=0;//加速度の和
  double aveAx=0,aveAy=0,aveAz=0;//加速度の平均
  double axRaw=0,ayRaw=0,azRaw=0;//加速度の生値(double)
  int tmp=0;//カウント用変数

  //while(tmp<10){//10回サンプルをとる
    imu::Vector<3> raw_acceleration = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    axRaw=raw_acceleration.x(),ayRaw=raw_acceleration.y(),azRaw=raw_acceleration.z();
    //sumAx+=axRaw,sumAy+=ayRaw,sumAz+=azRaw;
    //tmp++;
    //delay(2);
  //}

  //aveAx=sumAx/10.0,aveAy=sumAy/10.0,aveAz=sumAz/10.0;

  gravX=(1-k)*axRaw+k*gravX;
  gravY=(1-k)*ayRaw+k*gravY;
  gravZ=(1-k)*azRaw+k*gravZ;
  
  local_accX=axRaw-gravX,local_accY=ayRaw-gravY,local_accZ=azRaw-gravZ;
  velx+=local_accX*0.01;
  x+=velx*0.01;
  //if(waiting<50){
  //  local_accX=0,local_accY=0,local_accZ=0;
  //  delay(2);
  //  waiting++;
  //}
}

void get_fixed_global_acceleration(){
  
  
}

void loop() {
  // put your main code here, to run repeatedly:  
  get_fixed_local_acceleration();
  //get_fixed_global_acceleration();
  Serial.print(x,3);
  Serial.print(", ");
  //Serial.print(local_accY,1);
  //Serial.print(", ");
  //Serial.print(local_accZ,1);
  //Serial.print(", ");
  Serial.println();
  delay(10);
}
