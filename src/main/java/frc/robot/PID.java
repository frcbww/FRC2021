package frc.robot;

public class PID {
    double p, i, d;
    double Kp, Ki, Kd;
    double tar, now, diff, last, inte;

    public void setGain(double gain1, double gain2, double gain3) {
        Kp = gain1;
        Ki = gain2;
        Kd = gain3;
    }

    public void setTarget(double Target) {
        tar = Target;
    }

    public void reset() {
        tar = 0;
        now = 0;
        diff = 0;
        inte = 0;
        last = 0;
    }

    public double getCalculation(double now) {
        last = diff;
        diff = tar - now;
        inte += (diff + last) / 2;

        p = diff * Kp;
        i = inte * Ki;
        d = (diff - last) * Kd;
        return p + i + d;
    }
}
