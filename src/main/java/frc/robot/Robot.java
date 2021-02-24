// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * VMは、このクラスを自動的に実行し、TimedRobotのドキュメントに記載されている各モードに対応するメソッドです。
 * このプロジェクトの作成後にこのクラスまたはパッケージの名前を変更する場合は、プロジェクトのbuild.gradleファイルも更新する必要があります。
 */
public class Robot extends TimedRobot {
    private static final String DEFAULT_AUTO = "Default";
    private static final String BARREL_RACING_AUTO = "Barrel Racing Path";
    private static final String SLALOM_AUTO = "Slalom Path";
    private static final String BOUNCE_AUTO = "Bounce Path";
    private String autoSelected;
    private final SendableChooser<String> chooser = new SendableChooser<>();
    private final Timer timer = new Timer();
    private final PID pid = new PID();

    private final DifferentialDrive robotDrive = new DifferentialDrive(new PWMVictorSPX(0), new PWMVictorSPX(1));
    private final PWMVictorSPX collect = new PWMVictorSPX(2);
    private final Joystick stick = new Joystick(0);
    Encoder encoderL = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
    Encoder encoderR = new Encoder(2, 3, false, Encoder.EncodingType.k4X);
    private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

    /**
     * このメソッドはロボットが最初に起動されたときに実行され、初期化コードを書くことができます。
     */
    @Override
    public void robotInit() {
        chooser.setDefaultOption("Default Auto", DEFAULT_AUTO);
        chooser.addOption("Barrel Racing Path", BARREL_RACING_AUTO);
        chooser.addOption("Slalom Path", SLALOM_AUTO);
        chooser.addOption("Bounce Path", BOUNCE_AUTO);
        SmartDashboard.putData("Auto choices", chooser);
//        gyro.calibrate();
    }

    /**
     * このメソッドは、モードに関係なく、すべてのロボットパケットと呼ばれます。
     * 無効、自律、遠隔操作、およびテスト中に実行する診断などの項目にこれを使用します。
     * <p>
     * これは、モード固有の定期的な方法の後、LiveWindowとSmartDashboardの統合更新の前に実行されます。
     */
    @Override
    public void robotPeriodic() {
    }

    /**
     * この自律型（上記の選択コード）は、ダッシュボードを使用して異なる自律型モードを選択するための方法を示しています。
     * また、送信可能な選択コードは、Java SmartDashboardで機能します。
     * <p>
     * LabVIEWダッシュボードを使用する場合は、すべての選択コードを削除し getString コードのコメントを解除して、
     * ジャイロの下のテキストボックスから自動名(auto name)を取得します。
     * <p>
     * 上記のセレクターコードに追加コマンド（コメントに載っているの例など）を追加するか、
     * 以下のスイッチ構造に追加の文字列とコマンドを追加して比較することにより、自動モードを追加できます。
     */
    @Override
    public void autonomousInit() {
        autoSelected = chooser.getSelected();
        // autoSelected = SmartDashboard.getString("Auto Selector", defaultAuto);
        System.out.println("Auto selected: " + autoSelected);
        init();
    }

    /**
     * このメソッドは自律走行中に定期的に呼び出されます。
     */
    @Override
    public void autonomousPeriodic() {
        switch (autoSelected) {
            // バレルレーシング経路
            case BARREL_RACING_AUTO:
                break;

            // スラローム経路
            case SLALOM_AUTO:
                break;

            // バウンド経路
            case BOUNCE_AUTO:
                break;

            // 前進プログラムテスト
            case DEFAULT_AUTO:
                if (timer.get() < 2.0) {
//                    double zRotation = convertStraightEncoder(encoderL.get(), encoderR.get());
                    double zRotation = convertStraightGyro(0);
//                    robotDrive.arcadeDrive(0.6, 0.4, true);
                    robotDrive.arcadeDrive(0.6, zRotation, true);
                } else {
                    robotDrive.stopMotor();
                }
                break;
        }
    }

    /**
     * この関数は操作制御が有効になっているときに一度だけ呼び出されます。
     */
    @Override
    public void teleopInit() {
        init();
    }

    /**
     * このメソッドは操作制御中に定期的に呼び出されます。
     */
    @Override
    public void teleopPeriodic() {
        //コントローラーデータ
        double stickLX = stick.getX();
        double stickLY = stick.getY();
        double stickRX = stick.getRawAxis(4);

        double stickLR = 0.35 * stickLX + 0.4 * stickRX;

        //スピード制限
        if (Math.abs(stickLR) >= 0.5) {
            stickLR = 0.5 * Math.signum(stickLR);
        }

        // 押下ボタンのテスト
//        for (int i = 0; i < 9; i++) if (stick.getRawButton(i)) System.out.println(i);

        // 回収機構
        if (stick.getRawButton(5)) {
            collect.set(0.7);
        } else {
            collect.set(0);
        }

//        System.out.println(encoderL.getDistance() + ", " + encoderR.getDistance());

        //足回りモーター
        double xSpeed = -0.7 * convertStickSigmoid(stickLY);
        double zRotation = convertStickSigmoid(stickLR);
        robotDrive.arcadeDrive(xSpeed, zRotation, true);
    }

    /**
     * この関数はロボットが無効化されたときに一度だけ呼び出されます。
     */
    @Override
    public void disabledInit() {
    }

    /**
     * この関数はロボットが無効化されたときに定期的に呼び出されます。
     */
    @Override
    public void disabledPeriodic() {
    }

    /**
     * この関数はテストモードが有効になっているときに一度だけ呼び出されます。
     */
    @Override
    public void testInit() {
    }

    /**
     * このメソッドはテストモード中に定期的に呼び出されます。
     */
    @Override
    public void testPeriodic() {
    }

    private void init() {
        timer.reset();
        timer.start();
        encoderL.reset();
        encoderR.reset();
        gyro.reset();
    }

    // スティックの値をシグモイド関数で変換
    private double convertStickSigmoid(double zRotation) {
        return 2 / (1 + Math.exp(-3 * zRotation)) - 1;
    }

    // エンコーダーを使用した直進
    private double convertStraightEncoder(double leftEncoder, double rightEncoder) {
        double kP = 0.004;
        double diff = rightEncoder + leftEncoder;
        return kP * diff;
    }

    // ジャイロを使用した直進
    private double convertStraightGyro(double targetAngle) {
        pid.setGain(0.05, 0.00002, 0.3);
        pid.setTarget(targetAngle);

        return pid.getCalculation(gyro.getAngle());
    }
}
