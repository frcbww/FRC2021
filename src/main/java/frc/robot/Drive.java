package frc.robot;

import edu.wpi.first.wpilibj.*;

public class Drive extends edu.wpi.first.wpilibj.drive.DifferentialDrive {
    public Gain gyroGain = new Gain();
    public Gain encoderGain = new Gain();
    private final Print print = new Print();
    private final PID gyroPID = new PID(gyroGain);
    private final PID encoderPID = new PID(encoderGain);
    private final Timer driveTimer = new Timer();
    private final Init init = new Init();
    double gyroKp = 0, gyroKi = 0, gyroKd = 0;
    public Encoder encoderL, encoderR;
    public ADXRS450_Gyro gyro;
    public SpeedController mL, mR;

    private int gyroSmoothStraightState = 0;
    private int gyroSmoothPivotTurnState = 0;
    private double firstGyro = 0;

    public Drive(SpeedController sp1, SpeedController sp2) {
        super(sp1, sp2);
        mL = sp1;
        mR = sp2;
    }

    public Drive(SpeedController sp1, SpeedController sp2, Encoder ec1, Encoder ec2, ADXRS450_Gyro gy) {
        super(sp1, sp2);
        mL = sp1;
        mR = sp2;
        encoderL = ec1;
        encoderR = ec2;
        gyro = gy;
        gyroGain.setGyroGainApproximate(0.4, 0.04, 0.0005, 0.6, 0.6, 0.06, 0.0008, 1.2);
    }

    public void init() {
        encoderL.reset();
        encoderR.reset();
        gyro.reset();
        init.resetAll();
    }


    public void setGyroGain(double Kp, double Ki, double Kd) {
        gyroKp = Kp;
        gyroKi = Ki;
        gyroKd = Kd;
        gyroPID.setGain(Kp, Ki, Kd);
    }

    public void setEncoderGain(double Kp, double Ki, double Kd) {
        encoderPID.setGain(Kp, Ki, Kd);
    }

    public int gyroStraight(double power, double stop, double tar, boolean sud) {
        final String KEY = "gyroStraight";
        if (init.isNotInit(KEY)) {
            encoderL.reset();
            encoderR.reset();
            if (!init.isNotInit("gyroSmoothStraight")) gyroPID.reset();
            gyroGain.doGyroGainApproximate(Math.abs(power));
        }

        boolean pos_or_neg = power > 0 && stop > 0;
        int i = pos_or_neg ? 1 : -1;

        if (Math.abs((encoderR.get() - encoderL.get())) < Math.abs(stop) * 2) {
            arcadeDrive(i * Math.abs(power), gyroPID.getCalculation(gyro.getAngle() - tar));
            print.print(gyro.getAngle());
            return 0;
        } else {
            if (sud) {
                suddenly_stop(pos_or_neg);
            } else {
                stopMotor();
            }
            init.resetInit(KEY);
            return 1;
        }
    }

    public int gyroSmoothStraight(double min_power, double max_power, double stop, double tar, boolean sud) {
        final String KEY = "gyroSmoothStraight";
        if (init.isNotInit(KEY)) {
            encoderL.reset();
            encoderR.reset();
            gyroPID.reset();
            gyroSmoothStraightState = 0;
            System.out.println("initStraight");
        }
        switch (gyroSmoothStraightState) {
            case 0:
                gyroSmoothStraightState += gyroStraight_ChangeSpeed(min_power, max_power, stop / 3, tar, false);
                break;
            case 1:
                gyroSmoothStraightState += gyroStraight(max_power, stop / 3, tar, false);
                break;
            case 2:
                gyroSmoothStraightState += gyroStraight_ChangeSpeed(max_power, min_power, stop / 3, tar, sud);
                break;
            case 3:
                init.resetInit(KEY);
                return 1;
        }
        return 0;
    }

    public int gyroStraight_ChangeSpeed(double first_power, double last_power, double stop, double tar, boolean sud) {
        final String KEY = "gyroStraight_ChangeSpeed";
        if (init.isNotInit(KEY)) {
            encoderL.reset();
            encoderR.reset();
            if (init.isNotInit("gyroSmoothStraight")) gyroPID.reset();
        }
        boolean pos_or_neg = first_power > 0 && last_power > 0 && stop > 0;
        int i = pos_or_neg ? 1 : -1;
        double K = (Math.abs(last_power) * i - Math.abs(first_power) * i) / Math.abs(stop);

        if (Math.abs(encoderR.get() - encoderL.get()) < 2 * Math.abs(stop)) {
            double power = Math.abs(first_power) * i + Math.abs(encoderR.get() - encoderL.get()) * K / 2;
            gyroGain.doGyroGainApproximate(Math.abs(power));
            arcadeDrive(power, gyroPID.getCalculation(gyro.getAngle() - tar));
            return 0;
        } else {
            if (sud) {
                suddenly_stop(pos_or_neg);
            } else {
                stopMotor();
            }
            init.resetInit(KEY);
            return 1;
        }
    }

    public int gyroPivotTurn(char motor, double power, double angle, boolean sud) {
        final String KEY = "gyroPivotTurn";
        if (init.isNotInit(KEY)) {
            encoderL.reset();
            encoderR.reset();
            firstGyro = gyro.getAngle();
            System.out.println("init");
        }
        boolean pos_or_neg = angle > 0 && power > 0;
        int i = pos_or_neg ? 1 : -1;

        if (Math.abs(gyro.getAngle() - firstGyro) < Math.abs(angle)) {
            if (motor == 'L') {
                tankDrive(Math.abs(power) * i, encoderR.get() * 0.002);
            } else if (motor == 'R') {
                tankDrive(encoderL.get() * 0.002, Math.abs(power) * i);
            }
            return 0;
        } else {
            if (sud) {
                suddenly_stop_one(motor, pos_or_neg);
            } else {
                stopMotor();
            }
            init.resetInit(KEY);
            System.out.println("gyroPivotEnd");
            return 1;
        }
    }

    public int gyroSmoothPivotTurn(char motor, double min_power, double max_power, double angle, boolean sud) {
        final String KEY = "gyroSmoothPivotTurn";
        if (init.isNotInit(KEY)) {
            encoderL.reset();
            encoderR.reset();
            gyroSmoothPivotTurnState = 0;
        }
        switch (gyroSmoothPivotTurnState) {
            case 0:
                gyroSmoothPivotTurnState += gyroPivotTurn_ChangeSpeed(motor, min_power, max_power, angle / 3, false);
                break;
            case 1:
                gyroSmoothPivotTurnState += gyroPivotTurn(motor, max_power, angle / 3, false);
                break;
            case 2:
                gyroSmoothPivotTurnState += gyroPivotTurn_ChangeSpeed(motor, max_power, min_power, angle / 3, sud);
                break;
            case 3:
                init.resetInit(KEY);
                return 1;

        }
        return 0;
    }

    public int gyroPivotTurn_ChangeSpeed(char motor, double first_power, double last_power, double angle, boolean sud) {
        final String KEY = "gyroPivotTurn_ChangeSpeed";
        if (init.isNotInit(KEY)) {
            encoderL.reset();
            encoderR.reset();
            firstGyro = gyro.getAngle();
            System.out.println("initPivot");
        }

        boolean pos_or_neg = first_power > 0 && last_power > 0 && angle > 0;
        int i = pos_or_neg ? 1 : -1;
        double K = (Math.abs(last_power) * i - Math.abs(first_power) * i) / Math.abs(angle);

        if (Math.abs(gyro.getAngle()) < Math.abs(angle)) {
            double power = Math.abs(first_power) * i + Math.abs(gyro.getAngle() - firstGyro) * K;
            print.print(power);
            if (motor == 'L') {
                tankDrive(power, -encoderR.get() * 0.002);
            } else if (motor == 'R') {
                tankDrive(encoderL.get() * 0.002, power);
            }
            return 0;
        } else {
            if (sud) {
                suddenly_stop_one(motor, pos_or_neg);
            } else {
                stopMotor();
            }
            init.resetInit(KEY);
            return 1;
        }
    }


//    public void arcDrive (double power, double radius, double rotation_angle, boolean sud){
//        gyro.reset();
//        encoderR.reset();
//        encoderL.reset();
//        driveTimer.reset();
//        driveTimer.start();
//        double el = 0, er = 0, el_last = 0, er_last = 0;
//        int i;
//        boolean pos_or_neg = rotation_angle > 0;
//        if(pos_or_neg){i = 1;}else{i = -1;}
//        double control = 0;
//        while (gyro.getAngle() < Math.abs(rotation_angle)){
//            if(driveTimer.get() > 0.1){
//                el = encoderL.get() - el_last;
//                er = encoderR.get() - er_last;
//                el_last = encoderL.get();
//                er_last = encoderR.get();
//                driveTimer.reset();
//            }
//            control = encoderPID.getCalculation( (er + el) + 27 * (er - el) / (radius + 27));
//            print.print(control + ",  "+(el * (radius - 27) + er * (radius + 27)));
//            arcadeDrive(Math.abs(power)*i,control);
//        }
//        if(sud){suddenly_stop(pos_or_neg);}
//    }

    public void suddenly_stop(boolean pos_or_neg) {
        driveTimer.reset();
        driveTimer.start();
        while (driveTimer.get() < 0.1) {
            if (pos_or_neg) {
                arcadeDrive(-0.4, 0);
            } else {
                arcadeDrive(0.4, 0);
            }
        }
        stopMotor();
    }

    public void suddenly_stop_one(char motor, boolean pos_or_neg) {
        double i;
        if (pos_or_neg) {
            i = 1;
        } else {
            i = -1;
        }
        driveTimer.reset();
        driveTimer.start();
        if (motor == 'L') {
            while (driveTimer.get() < 0.1) {
                mL.set(-0.4 * i);
            }
            mL.stopMotor();
        } else if (motor == 'R') {
            while (driveTimer.get() < 0.1) {
                mR.set(0.4 * i);
            }
            mR.stopMotor();
        }
    }

}