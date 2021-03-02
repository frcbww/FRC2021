package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive extends edu.wpi.first.wpilibj.drive.DifferentialDrive {
    private final Print print = new Print();
    private final PID gyroPID = new PID();
    private final PID encoderPID = new PID();
    private final Timer driveTimer = new Timer();
    double left, right;
    double tar, val, dif;
    double gyroKp = 0, gyroKi = 0, gyroKd = 0;
    public Encoder encoderL, encoderR;
    public ADXRS450_Gyro gyro;
    public SpeedController mL, mR;

    public int smoothStraightState = 0;

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
    }

//    public void setGain(double Kp, double Ki, double Kd){
//        gyroKp = Kp;
//        gyroKi = Ki;
//        gyroKd = Kd;
//        encoderKp = Kp;
//        encoderKi = Ki;
//        encoderKd = Kd;
//    }

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
        if (!Init.isInit(KEY)) {
            encoderL.reset();
            encoderR.reset();
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
            Init.resetInit(KEY);
            return 1;
        }
    }

    public void gyroSmoothStraight(double min_power, double max_power, double stop, double tar, boolean sud) {
        if (!Init.isInit("gyroSmoothStraight")) {
            encoderL.reset();
            encoderR.reset();
            gyroPID.setGain(gyroKp, gyroKi, gyroKd);
            System.out.println("init");
        }
        switch (smoothStraightState) {
            case 0:
                smoothStraightState += gyroStraight_ChangeSpeed(min_power, max_power, stop / 3, tar, false);
                System.out.println("smooth0");
                break;
            case 1:
                smoothStraightState += gyroStraight(max_power, stop / 3, tar, false);
                System.out.println("smooth1");
                break;
            case 2:
                smoothStraightState += gyroStraight_ChangeSpeed(max_power, min_power, stop / 3, tar, sud);
                break;
        }

    }

    public void gyroSmoothPivotTurn(char motor, double min_power, double max_power, double angle, boolean sud) {
        int i;
        gyro.reset();
        encoderL.reset();
        encoderR.reset();
        boolean pos_or_neg = angle > 0 && min_power > 0 && max_power > 0;
        double K = Math.abs((max_power - min_power) / (angle / 3));
        double last_power = min_power;
        if (pos_or_neg) {
            i = 1;
        } else {
            i = -1;
        }
        if (motor == 'L') {
            for (int count = 1; count <= 3; count++) {
                while (Math.abs(gyro.getAngle()) < Math.abs(angle) * count / 3) {
                    mL.set(i * (Math.abs(last_power) - Math.abs((gyro.getAngle() - (count - 1) * angle / 3) * K) * (count - 2)));
                    mR.set(encoderR.get() * 0.002);
                }
                last_power = max_power;
            }
            if (sud) {
                driveTimer.reset();
                driveTimer.start();
                while (driveTimer.get() < 0.1) {
                    mL.set(-i * 0.3);
                }
            }
            stopMotor();
        } else if (motor == 'R') {
            for (int count = 1; count <= 3; count++) {
                while (Math.abs(gyro.getAngle()) < Math.abs(angle) * count / 3) {
                    mR.set(-i * (Math.abs(last_power) - Math.abs((gyro.getAngle() + (count - 1) * angle / 3) * K) * (count - 2)));
                    mL.set(encoderL.get() * 0.002);
                }
                last_power = max_power;
            }
            if (sud) {
                driveTimer.reset();
                driveTimer.start();
                while (driveTimer.get() < 0.1) {
                    mR.set(i * 0.3);
                }
            }
            stopMotor();
        }
        driveTimer.reset();
        driveTimer.start();
        while (driveTimer.get() < 0.2) ;
    }

    public int gyroStraight_ChangeSpeed(double first_power, double last_power, double stop, double tar, boolean sud) {
        final String KEY = "gyroStraight_ChangeSpeed";
        if (!Init.isInit(KEY)) {
            encoderL.reset();
            encoderR.reset();
        }
        boolean pos_or_neg = first_power > 0 && last_power > 0 && stop > 0;
        int i = pos_or_neg ? 1 : -1;
        double K = (Math.abs(last_power) * i - Math.abs(first_power) * i) / Math.abs(stop);

        if (Math.abs(encoderR.get() - encoderL.get()) < 2 * Math.abs(stop)) {
            double power = Math.abs(first_power) * i + Math.abs(encoderR.get() - encoderL.get()) * K / 2;
            gyroKp = Math.abs(power) * 0.05 + 0.01;
            gyroKi = Math.abs(power) * 0.0000175 - 0.000006;
            if (gyroKi < 0) {
                gyroKi = 0;
            }
            gyroKd = Math.abs(power) * 1.5 - 0.3;
            if (gyroKd < 0) {
                gyroKd = 0;
            }
            gyroPID.setGain(gyroKp, gyroKi, gyroKd);
            arcadeDrive(power, gyroPID.getCalculation(gyro.getAngle() - tar));
            return 0;
        } else {
            if (sud) {
                suddenly_stop(pos_or_neg);
            } else {
                stopMotor();
            }
            Init.resetInit(KEY);
            return 1;
        }
    }

    public void gyroPivotTurn_ChangeSpeed(char motor, double first_power, double last_power, double angle, boolean sud) {
        encoderL.reset();
        encoderR.reset();
        double first_gyro = gyro.getAngle();
        double i;
        double power;
        boolean pos_or_neg = first_power > 0 && last_power > 0 && angle > 0;
        if (pos_or_neg) {
            i = 1;
        } else {
            i = -1;
        }
        double K = (Math.abs(last_power) * i - Math.abs(first_power) * i) / Math.abs(angle);
        while (Math.abs(gyro.getAngle()) < Math.abs(angle)) {
            power = Math.abs(first_power) * i + Math.abs(gyro.getAngle() - first_gyro) * K;
            print.print(power);
            if (motor == 'L') {
                tankDrive(power, encoderR.get() * 0.002);
            } else if (motor == 'R') {
                tankDrive(encoderL.get() * 0.002, power);
            }
        }
        if (sud) {
            suddenly_stop_one(motor, pos_or_neg);
        }
        stopMotor();
    }

    public void gyroPivotTurn(char motor, double power, double angle, boolean sud) {
        encoderL.reset();
        encoderR.reset();
        double first_gyro = gyro.getAngle();
        double i;
        boolean pos_or_neg = angle > 0 && power > 0;
        if (pos_or_neg) {
            i = 1;
        } else {
            i = -1;
        }
        while (Math.abs(gyro.getAngle() - first_gyro) < Math.abs(angle)) {
            if (motor == 'L') {
                tankDrive(Math.abs(power) * i, encoderR.get() * 0.002);
            } else if (motor == 'R') {
                tankDrive(encoderL.get() * 0.002, Math.abs(power) * i);
            }
        }
        if (sud) {
            suddenly_stop_one(motor, pos_or_neg);
        }
        stopMotor();
    }


//    public void encoderStraight (double power, double stop, boolean sud){
//        encoderL.reset();
//        encoderR.reset();
//        if (stop > 0){
//            while ((encoderR.get() - encoderL.get()) < stop * 2){
//                arcadeDrive(power,encoderPID.getCalculation(encoderR.get()+encoderL.get()));
//            }
//        } else {
//            while ((encoderL.get() - encoderR.get()) < stop * 2){
//                arcadeDrive(power,encoderPID.getCalculation(encoderL.get()+encoderR.get()));
//            }
//        }
//
//        if (sud){suddenly_stop(stop>0);}
//        stopMotor();
//    }

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