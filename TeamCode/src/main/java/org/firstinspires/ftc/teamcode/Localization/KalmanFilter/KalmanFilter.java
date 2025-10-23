package org.firstinspires.ftc.teamcode.Localization.KalmanFilter;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
@Config
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
        double angularKt = tagDetected == true ? Kt : 0;
        double dTheta = wrapAngleRad(aprilTagTheta - pinpointTheta);
        double correctAngle = tagDetected ? dTheta * angularKt + pinpointTheta : pinpointTheta;
        return wrapAngleRad(correctAngle);
    }

    double wrapAngleRad(double value){
        while(value <= - Math.PI){
            value += 2 * Math.PI;
        }
        while(value >=  Math.PI){
            value -=  2 * Math.PI;
        }
        return value;
    }

    double warpAngleDeg(double value){
        while (value <= -180) {
            value += 360;
        }
        while(value >= 180){
            value -= 360;
        }
        return value;
    }

}
