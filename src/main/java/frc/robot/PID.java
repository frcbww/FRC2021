package frc.robot;

import java.util.List;

public class PID {
    double p, i, d;
    double tar, diff, last, inte;

    Gain gain;

    public PID(Gain _gain){
        gain = _gain;
    }

    public void setGain(double gain1, double gain2, double gain3) {
        gain.Kp = gain1;
        gain.Ki = gain2;
        gain.Kd = gain3;
    }

    public void setTarget(double Target) {
        tar = Target;
    }

    public void reset() {
        tar = 0;
        diff = 0;
        inte = 0;
        last = 0;
    }

    public double getCalculation(double now) {
        last = diff;
        diff = tar - now;
        inte += (diff + last) / 2;

        p = diff * gain.Kp;
        i = inte * gain.Ki;
        d = (diff - last) * gain.Kd;
        return p + i + d;
    }
}
