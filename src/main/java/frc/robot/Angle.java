package frc.robot;

public class Angle {
    static double difference(double angle1,double angle2){
        return ((Math.abs(angle1-angle2 + 180) % 360) + 360) % 360 - 180;
    }
}
