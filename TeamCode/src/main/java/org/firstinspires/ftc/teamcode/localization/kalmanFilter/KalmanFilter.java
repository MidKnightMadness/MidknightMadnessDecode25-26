package org.firstinspires.ftc.teamcode.localization.kalmanFilter;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;


public class KalmanFilter {
    double Q;//pinpoint noise
    double R ;//april tags noise

    double x;//estimated value
    double p; //estimated value uncertainty
    double p_prev;

    double Kt;//kalman gain(0 - 1)

    public KalmanFilter(double Q, double R){
        this.Q = Q;
        this.R = R;
    }
    public double update(double pinpointVal, double aprilTagVal, boolean tagDetected){
        x = pinpointVal;

        p = p_prev + Q;

        if(tagDetected){
            Kt = p / (p + R);//Kalman gain, closer to 1 is more trust in sensor, closer to 0 is less
            x = x + Kt * (aprilTagVal - x);//update value prediction
            p = (1- Kt) * p;//update estimated value uncertainty
        }
        p_prev = p;
        return x;
    }


    public double updateAngle(double pinpointTheta, double aprilTagTheta, boolean tagDetected){//input as radians
        if(!tagDetected){
            return wrapAngleRad(pinpointTheta);
        }
        pinpointTheta = wrapAngleRad(pinpointTheta); aprilTagTheta = wrapAngleRad(aprilTagTheta);
        double dTheta = pinpointTheta - aprilTagTheta;
        double smallestDifference = ((dTheta + Math.PI) % (2 * Math.PI)) - Math.PI;
        return wrapAngleRad(pinpointTheta +  Kt * smallestDifference);
    }

    double wrapAngleRad(double value){
        value = value % (2*Math.PI);
        if(value < 0) value += 2*Math.PI;
        return value;
    }


}
