#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Ticker.h>

#define BNO055interval 10 //何ms間隔でデータを取得するか

//Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire); //ICSの名前, デフォルトアドレス, 謎
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup(void)
{
  pinMode(21, INPUT_PULLUP); //SDA 21番ピンのプルアップ(念のため)
  pinMode(22, INPUT_PULLUP); //SDA 22番ピンのプルアップ(念のため)

  Serial.begin(115200);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  if (!bno.begin()) // センサの初期化
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(false);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
}

double prevyaw=0;

void get_bno055_data(void)
{
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  
  /*
  // キャリブレーションのステータスの取得と表示
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("CALIB Sys:");
  Serial.print(system, DEC);
  Serial.print(", Gy");
  Serial.print(gyro, DEC);
  Serial.print(", Ac");
  Serial.print(accel, DEC);
  Serial.print(", Mg");
  Serial.print(mag, DEC);
  */
  
  /*
  // ジャイロセンサ値の取得と表示
  imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  Serial.print(" 　Gy_xyz:");
  Serial.print(gyroscope.x());
  Serial.print(", ");
  Serial.print(gyroscope.y());
  Serial.print(", ");
  Serial.print(gyroscope.z());
  */
  
  /*
  // 加速度センサ値の取得と表示
  imu::Vector<3> accelermetor = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  Serial.print(" 　Ac_xyz:");
  Serial.print(accelermetor.x());
  Serial.print(", ");
  Serial.print(accelermetor.y());
  Serial.print(", ");
  Serial.print(accelermetor.z());
  */
  
  /*
  // 磁力センサ値の取得と表示
  imu::Vector<3> magnetmetor = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  Serial.print(" 　Mg_xyz:");
  Serial.print(magnetmetor .x());
  Serial.print(", ");
  Serial.print(magnetmetor .y());
  Serial.print(", ");
  Serial.print(magnetmetor .z());
  */
/*
  // センサフュージョンによる方向推定値の取得と表示
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  //Serial.print(" 　DIR_xyz:");
  Serial.print(euler.x());
  Serial.print(", ");
  Serial.print(euler.y());
  Serial.print(", ");
  Serial.print(euler.z());
*/
    // センサフュージョンの方向推定値のクオータニオン
    imu::Quaternion quat = bno.getQuat();
    double q0=quat.w(),q1=quat.x(),q2=quat.y(),q3=quat.z();

    double t0=2*(q0*q1+q2*q3),t1=1-2*(q1*q1+q2*q2);
    double roll=atan2(t0,t1);

    double t2=2*(q0*q2-q3*q1);
    t2=(t2>1.0?1.0:t2);
    t2=(t2<-1.0?-1.0:t2);
    double pitch=asin(t2);

    double t3=2*(q0*q3+q1*q2),t4=1-2*(q2*q2+q3*q3);
    double yaw=atan2(t3,t4);
    roll*=180/acos(-1),pitch*=180/acos(-1),yaw*=180/acos(-1);
    if(abs(yaw)<0.01&&abs(yaw-prevyaw)>5)yaw=prevyaw;
    Serial.print(yaw);Serial.print(", ");
    prevyaw=yaw;//波形がきもい、何とかする
    //Serial.print(t0,4);Serial.print(" ");Serial.print(t1,4);
    //Serial.print("roll: ");
    //Serial.print(roll,4);Serial.print(", ");
    //Serial.print("pitch: ");
    //Serial.print(pitch,4);Serial.print(", ");
    //Serial.print("yaw: ");
    //Serial.print(yaw,4);
  /*
    Serial.print("qW: ");
    Serial.print(quat.w(), 4);
    Serial.print(" qX: ");
    Serial.print(quat.x(), 4);
    Serial.print(" qY: ");
    Serial.print(quat.y(), 4);
    Serial.print(" qZ: ");
    Serial.print(quat.z(), 4);
    Serial.print("\t\t");
  */
  Serial.println();
}

void loop(void)
{
  get_bno055_data();
  delay(10);
}