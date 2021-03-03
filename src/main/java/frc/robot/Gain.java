package frc.robot;

public class Gain {
    public double Kp, Ki, Kd;
    private double tilt_Kp =0, tilt_Ki = 0, tilt_Kd = 0;
    private double constantTerm_Kp = 0, constantTerm_Ki = 0, constantTerm_Kd = 0;

    public Gain(double _Kp, double _Ki, double _Kd){
        Kp = _Kp;
        Ki = _Ki;
        Kd = _Kd;
    }

    public Gain(){
    }

    public void setGyroGainApproximate(double power1, double Kp1, double Ki1, double Kd1, double power2, double Kp2, double Ki2, double Kd2){
        double amount_power = power2-power1;
        tilt_Kp = (Kp2-Kp1) / amount_power;
        tilt_Ki = (Ki2-Ki1) / amount_power;
        tilt_Kd = (Kd2-Kd1) / amount_power;
        constantTerm_Kp = Kp1 - tilt_Kp *  power1;
        constantTerm_Ki = Ki1 - tilt_Ki * power1;
        constantTerm_Kd = Kd1 - tilt_Kd * power1;
    }

    public void doGyroGainApproximate(double power){
        Kp = tilt_Kp * power + constantTerm_Kp; Kp = Kp > 0 ? Kp : 0;
        Ki = tilt_Ki * power + constantTerm_Ki; Ki = Ki > 0 ? Ki : 0;
        Kd = tilt_Kd * power + constantTerm_Kd; Kd = Kd > 0 ? Kd : 0;
    }

}
