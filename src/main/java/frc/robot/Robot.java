// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;

/**
 * VMは、このクラスを自動的に実行し、TimedRobotのドキュメントに記載されている各モードに対応するメソッドです。
 * このプロジェクトの作成後にこのクラスまたはパッケージの名前を変更する場合は、プロジェクトのbuild.gradleファイルも更新する必要があります。
 */
public class Robot extends TimedRobot {
    private static final String DEFAULT_AUTO = "Default";
    private static final String BARREL_RACING_AUTO = "Barrel Racing Path";
    private static final String SLALOM_AUTO = "Slalom Path";
    private static final String BOUNCE_AUTO = "Bounce Path";
    private  static final String TUNING = "Tuning";
    private String autoSelected;
    private final SendableChooser<String> chooser = new SendableChooser<>();
    public final Timer timer = new Timer();
    public final Timer compressor_timer = new Timer();
    private final PID gyroPid = new PID();
    private final Print print = new Print();
    public final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
    Encoder encoderL = new Encoder(0, 1, false, Encoder.EncodingType.k2X);
    Encoder encoderR = new Encoder(2, 3, false, Encoder.EncodingType.k2X);
    private final Drive drive = new Drive(new PWMVictorSPX(0), new PWMVictorSPX(1), encoderL, encoderR, gyro);

    public final XboxController controller = new XboxController(0);

    Compressor c = new Compressor();
    VictorSP victor = new VictorSP(2);

    final double gyroKp = 0.05;
    final double gyroKi = 0.00002;
    final double gyroKd = 0.3;

    int loop_count;


    /**
     * このメソッドはロボットが最初に起動されたときに実行され、初期化コードを書くことができます。
     */
    @Override
    public void robotInit() {
        CameraServer.getInstance().startAutomaticCapture(0);
        chooser.setDefaultOption("Default Auto", DEFAULT_AUTO);
        chooser.addOption("Barrel Racing Path", BARREL_RACING_AUTO);
        chooser.addOption("Slalom Path", SLALOM_AUTO);
        chooser.addOption("Bounce Path", BOUNCE_AUTO);
        chooser.addOption("Tuning", TUNING);
        SmartDashboard.putData("Auto choices", chooser);
        gyro.calibrate();
        c.stop();
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
        encoderL.reset();
        encoderR.reset();
        victor.set(0);
        c.setClosedLoopControl(true);
        c.stop();
        compressor_timer.reset();
        compressor_timer.start();
        drive.setGyroGain(0.09,0.0000007,0.9);
//        drive.setGyroGain(0,0,0);
        drive.setEncoderGain(0.002,0,0);
        loop_count = 1;
    }

    /**
     * このメソッドは自律走行中に定期的に呼び出されます。
     */
    @Override
    public void autonomousPeriodic() {

        if (loop_count == 1){
            int i = 0;
            System.out.println(encoderL.get() + ", " + encoderR.get());
            switch (autoSelected) {
                // バレルレーシング経路
                case BARREL_RACING_AUTO:
                    break;

                // スラローム経路
                case SLALOM_AUTO:
                    victor.set(0.5);
                    break;

                // バウンド経路
                case BOUNCE_AUTO:
                    timer.reset();timer.start();
                    while (timer.get()<0.2);
//                    drive.gyroSmoothStraight(0.5,0.7,3000,false);
//                    drive.gyroSmoothPivotTurn('R',0.2,0.4,50,true);
//                    drive.gyroSmoothStraight(0.4,0.7,4500,false);
//                    drive.gyroSmoothPivotTurn('L',0.2,0.4,50,true);
//                    drive.gyroSmoothStraight(0.6,0.8,12000,false);
//                    drive.gyroSmoothPivotTurn('L',0.2,0.2,60,true);
//                    drive.gyroSmoothStraight(0.4,0.7,7000,true);

                    drive.gyroStraight(0.6,3000,0,false);
                    drive.gyroSmoothPivotTurn('R',0.2,0.4,50,true);
                    drive.gyroStraight(0.6,4500,50,false);
                    drive.gyroSmoothPivotTurn('L',0.2,0.4,50,true);
                    drive.gyroSmoothStraight(0.6,0.8,12000,0,false);
                    drive.gyroSmoothPivotTurn('L',0.2,0.2,60,true);
                    drive.gyroStraight(0.6,7000,0,false);


                    break;

                // 前進プログラムテスト
                case DEFAULT_AUTO:
                    timer.reset();timer.start();
                    while (timer.get()<5);
                    drive.gyroSmoothStraight(0.6,0.8,10000,0,true);
                    break;

                case TUNING :
                    timer.reset();timer.start();
                    drive.gyroSmoothPivotTurn('R',0.3,0.3,15,true);
                    while (timer.get()<2);
                    drive.setGyroGain(0.03,0.000001,0.3);
                    drive.gyroStraight(0.6,20000,0,true);
                    break;

            }
        } else {
            drive.stopMotor();

        }
        loop_count++;
    }

    /**
     * この関数は操作制御が有効になっているときに一度だけ呼び出されます。
     */
    @Override
    public void teleopInit() {
        init();
        compressor_timer.reset();
        compressor_timer.start();
    }

    /**
     * このメソッドは操作制御中に定期的に呼び出されます。
     */
    @Override
    public void teleopPeriodic() {
        //コントローラーデータ
        double stickLX = controller.getX(GenericHID.Hand.kLeft);
        double stickLY = controller.getY(GenericHID.Hand.kLeft);
        double stickRX = controller.getX(GenericHID.Hand.kRight);

        double stickLR = 0.35 * stickLX + 0.4 * stickRX;

        //スピード制
        if (Math.abs(stickLR) >= 0.5) {
            stickLR = 0.5 * Math.signum(stickLR);
        }
        if (controller.getAButton()){
            victor.set(0.5);
        } else{
            victor.stopMotor();
        }

//        System.out.println(controller.getTriggerAxis(GenericHID.Hand.kLeft));


        print.print(encoderL.get() + ", " + encoderR.get());
//        System.out.println(gyro.getAngle());


        //足回りモーター
        double xSpeed = -0.7 * convertStickSigmoid(stickLY);
        double zRotation = convertStickSigmoid(stickLR);
        drive.arcadeDrive(xSpeed, zRotation, true);

        if (compressor_timer.get() < 0.7){
//            c.start();
        } else {
            c.stop();
        }
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
        init();
    }

    /**
     * このメソッドはテストモード中に定期的に呼び出されます。
     */
    @Override
    public void testPeriodic() {
        System.out.println(gyro.getAngle());
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
    private double convertAngleGyro(double targetAngle, double Kp, double Ki, double Kd) {
        gyroPid.setGain(Kp, Ki, Kd);
        gyroPid.setTarget(targetAngle);

        return gyroPid.getCalculation(gyro.getAngle());
    }

    private void straightDrive_Gyro(double speed, int distance, double Kp, double Ki, double Kd){

    }
}
