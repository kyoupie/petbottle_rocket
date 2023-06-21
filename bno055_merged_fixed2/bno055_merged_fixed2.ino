#include <Wire.h>
#include <Adafruit_BNO055.h>
Adafruit_BNO055 bno=Adafruit_BNO055(55,0x28);

/*
  このプログラムの簡単な説明:
  1.get_fixed_local_acceleration()で生の加速度を重力加速度の成分を抜いた加速度に変換。
  2.get_bno055_data()でセンサのクオータニオンを取り出す
  3.get_fixed_global_acceleration()でクオータニオンによる回転行列を用いて、ローカルの加速度->グローバルの加速度 に変換。
  4.3で変換した加速度をdtについて積分して速度を計算(開発途中)
  
  メインファイルはこれです。上記の関数はこのフォルダの別のファイルにあります。
  
  現状:
  鉛直上向き(z軸)の加速度を取り出すことはできた(かなり正確かも)
  
  課題:
  速度と変位を計算しようとするとどうしても誤差が生じてしまう(改善案募集中)
  記録をSDカードに保存することはまだできてない
  これはハードについてだけど、ArduinoMegaじゃ大きすぎると思うので、arduino nanoなどを買うべき？ロケットに入らない可能性が...
*/

struct sensorXYZ{//センサーの構造体
  double x;
  double y;
  double z;
};

struct quaternions{//クオータニオンの構造体
  double i;
  double j;
  double k;
  double l;
};

void setup() {
  // put your setup code here, to run once:
  pinMode(20, INPUT_PULLUP); //SDA 21番ピンのプルアップ(念のため)
  pinMode(21, INPUT_PULLUP); //SDA 22番ピンのプルアップ(念のため)
  Serial.begin(115200);
  if(!bno.begin()){//センサの接続不良テスト
    Serial.print("Something went wrong! Please RESET it...");Serial.println("");
    while(true);
  }
  delay(2000);
}

double gravX = 0,gravY = 0,gravZ = 0;
double global_accX = 0,global_accY = 0,global_accZ = 0;
double velx=0,vely=0,velz=0;
double vel_offsetx=0,vel_offsety=0,vel_offsetz=0;//速度補正用
double x=0,y=0,z=0;
const double k = 0.85;//ローパスの定数
//double dt = 0.01;//時間(ms)
bool flag = false;
long double dt;
long time;
long time1;
long time2;

void loop() {
  time=millis();
  sensorXYZ local_accels,world_accels,vels,location;
  quaternions qua;
  
  local_accels = get_fixed_local_acceleration();

  qua = get_bno055_data();

  world_accels = get_fixed_global_acceleration(local_accels.x,local_accels.y,local_accels.z,qua.i,qua.j,qua.k,qua.l);
  
  global_accX = world_accels.x,global_accY = world_accels.y;global_accZ = world_accels.z;
  
  if(flag){//最初、z軸加速度にずれが生じるので0付近になるまで待つ
    get_velocities(global_accX,global_accY,global_accZ);
    get_locations(velx,vely,velz);      
  } 
  else if(time>=2500&&abs(global_accZ)<0.01){
    flag = true;
  }
  else {
    global_accZ=0;
  }
  //Serial.print(global_accX,3);//出力,シリアルプロッタで閲覧可能
  //Serial.print(", ");
  //Serial.print(global_accY,3);
  //Serial.print(", ");
  Serial.print(velz,3);
  Serial.println();
}