void get_locations(double velx,double vely,double velz){
  time2=millis();
  x+=velx*(time2-time)/1000;
  y+=vely*(time2-time)/1000;
  z+=velz*(time2-time)/1000;
}