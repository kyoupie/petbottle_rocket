quaternions get_bno055_data(){//クオータニオンを取り出す

  double q0=0,q1=0,q2=0,q3=0;
  
  imu::Quaternion quat = bno.getQuat();
  q0=quat.w(),q1=quat.x(),q2=quat.y(),q3=quat.z();

  return quaternions({q0,q1,q2,q3});
}