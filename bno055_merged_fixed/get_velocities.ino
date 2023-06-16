sensorXYZ get_velocities(double ax,double ay,double az){//速度を計算する

  velx+=ax*dt;//まずは単純な積分から,
  vely+=ay*dt;
  velz+=az*dt;
  
  return sensorXYZ({velx,vely,velz});
}
