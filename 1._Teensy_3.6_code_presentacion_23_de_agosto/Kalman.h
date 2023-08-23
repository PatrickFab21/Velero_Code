#ifndef _Kalman_h
#define _Kalman_h

class SensorKalman {
  public:
    SensorKalman(){
        Q_distance=1;
        R_measure=1;

        distance=0; //reset the distance
        P=0;  //initial covariance matrix     
    }
   double getDistance(double newDistance, double dt){
       //distance=distance; //priori distance
       P+=Q_distance*dt;  //estimation error covariance
       //Kalman gain
       S=P+R_measure;
       K=P/S;
       //Update whith measurement 
       y=newDistance-distance;
       //Calculate distance
       distance+=K*y;
       //Update the error covariance
       P*=(1-K);
       return distance;
   };
   void setDistance(double newDistance){ distance = newDistance;};
   double getQdistance(){return Q_distance;};
   void setQdistance(double newQ_distance){Q_distance=newQ_distance;};
   double getRmeasure(){return R_measure;};
   void setRmeasure(double newR_measure){R_measure=newR_measure;}; 
  private:
   double Q_distance;  
   double R_measure; 
   double distance;
   double P;
   double K;
   double y;
   double S;
};

#endif

