sensorXYZ get_fixed_local_acceleration(){//生の加速度を出し、それをローパスフィルタで修正。
  
  double axRaw=0,ayRaw=0,azRaw=0;//加速度の生値(double)
  double local_accX,local_accY,local_accZ;//返り値用の変数
  imu::Vector<3> raw_acceleration = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  axRaw=raw_acceleration.x(),ayRaw=raw_acceleration.y(),azRaw=raw_acceleration.z();

  gravX=(1-k)*axRaw+k*gravX;//ローパスフィルタ
  gravY=(1-k)*ayRaw+k*gravY;
  gravZ=(1-k)*azRaw+k*gravZ;
  
  local_accX=axRaw-gravX,local_accY=ayRaw-gravY,local_accZ=azRaw-gravZ;//重力加速度成分を取り除く
  if(abs(local_accX)<=0.04)local_accX=0;//あまりにも小さい加速度は誤差なので0にする
  if(abs(local_accY)<=0.04)local_accY=0;
  if(abs(local_accZ)<=0.04)local_accZ=0;

  return sensorXYZ({local_accX,local_accY,local_accZ});
}