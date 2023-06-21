void get_velocities(double ax,double ay,double az){//速度を計算する
  time1=millis();
  velx+=ax*(time1-time)/1000;//まずは単純な積分から,
  vely+=ay*(time1-time)/1000;
  velz+=az*(time1-time)/1000;
  //平均とって同じ値ばかりだったらvelを0に補正するとか？vel_offsetを使う
}
