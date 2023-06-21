sensorXYZ get_fixed_global_acceleration(double ax,double ay,double az,double q0,double q1,double q2,double q3){//回転行列で座標変換する関数
  //ここに回転行列を書く
  double rotation_matrix[3][3]={//クオータニオンの回転行列.いかつい
    {q0*q0+q1*q1-q2*q2-q3*q3,2*(q1*q2-q0*q3),2*(q1*q3+q0*q2)},
    {2*(q1*q2+q0*q3),q0*q0-q1*q1+q2*q2-q3*q3,2*(q2*q3-q0*q1)},
    {2*(q1*q3-q0*q2),2*(q2*q3+q0*q1),q0*q0-q1*q1-q2*q2+q3*q3}
  };
  
  double accel_matrix[3][1]={{ax},{ay},{az}};//ローカル加速度の配列
  double global_accel_matrix[3][1];//グローバル加速度の配列
  double inner_product=0;
  double vecA[3],vecB[3];
  
  for(int a=0;a<3;a++){//ここで行列積の計算
    for(int b=0;b<1;b++){
      for(int c=0;c<3;c++){
        vecA[c]=rotation_matrix[a][c];
        vecB[c]=accel_matrix[c][b];
      }
      inner_product=0;
      for(int c=0;c<3;c++){
        inner_product+=vecA[c]*vecB[c];
      }
      global_accel_matrix[a][b]=inner_product;
    }
  }

  return sensorXYZ({global_accel_matrix[0][0],global_accel_matrix[1][0],global_accel_matrix[2][0]});
}