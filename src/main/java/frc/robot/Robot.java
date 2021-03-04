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
    private static final String TUNING = "Tuning";
    private String autoSelected;
    private final SendableChooser<String> chooser = new SendableChooser<>();
    public final Timer compressor_timer = new Timer();
    private final Print print = new Print();
    public final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
    Encoder encoderL = new Encoder(0, 1, false, Encoder.EncodingType.k2X);
    Encoder encoderR = new Encoder(2, 3, false, Encoder.EncodingType.k2X);
    private final Drive drive = new Drive(new PWMVictorSPX(0), new PWMVictorSPX(1), encoderL, encoderR, gyro);

    public final XboxController controller = new XboxController(0);
    private String controllerMode = "DefaultMode";

    Compressor c = new Compressor();
    VictorSP victor = new VictorSP(2);

    final double gyroKp = 0.05;
    final double gyroKi = 0.00002;
    final double gyroKd = 0.3;

    private int defaultAutoState = 0, barrelRacingAutoState = 0, slalomAutoState = 0, bounceAtoState = 0, tuningState = 0;

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
        System.out.println("Auto selected: " + autoSelected);
        drive.init();
        victor.set(0);
        c.setClosedLoopControl(true);
        c.stop();
        drive.setEncoderGain(0.002, 0, 0);
        loop_count = 1;
        defaultAutoState = 0;
        barrelRacingAutoState = 0;
        slalomAutoState = 0;
        bounceAtoState = 0;
        tuningState = 0;
    }

    /**
     * このメソッドは自律走行中に定期的に呼び出されます。
     */
    @Override
    public void autonomousPeriodic() {

//        if (loop_count == 1) {
        int i = 0;
//        System.out.println(encoderL.get() + ", " + encoderR.get());

        switch (autoSelected) {
            // バレルレーシング経路
            case BARREL_RACING_AUTO:
                break;

            // スラローム経路
            case SLALOM_AUTO:
//                System.out.println(gyro.getAngle());
                switch (slalomAutoState) {
                    case 0:
                        slalomAutoState += drive.gyroStraight_ChangeSpeed(0.6,0.7,2000,0,false);
                        break;
                    case 1:
                        slalomAutoState += drive.gyroArcTurn_ChangeSpeed(0.7,0.8,-0.7,70,false);
                        break;
                    case 2:
                        slalomAutoState += drive.gyroArcTurn_ChangeSpeed(0.8,0.9,0.7,70,false);
                        break;
                    case 3:
                        slalomAutoState += drive.gyroSmoothStraight(0.8,1,10700,0,false);
                        break;
                    case 4:
                        slalomAutoState += drive.gyroArcTurn_ChangeSpeed(0.8,0.7,0.7,60,false);
                        break;
                    case 5:
                        slalomAutoState += drive.gyroSmoothStraight(0.7,0.8,2000,60,false);
                        break;
                    case 6:
                        slalomAutoState += drive.gyroArcTurn(0.8,-1,300,false);
                        break;
                    case 7:
                        slalomAutoState += drive.gyroStraight_ChangeSpeed(0.8,0.8,1000,-240,false);
                        break;
                    case 8:
                        slalomAutoState += drive.gyroArcTurn_ChangeSpeed(0.8,0.9,0.7,60,false);
                        System.out.println("turn");
                        break;
                    case 9:
                        slalomAutoState += drive.gyroSmoothStraight(0.8,1,10700,-180,true);
                        System.out.println("Straight");
                        break;
                    case 10:
                        slalomAutoState += drive.gyroArcTurn_ChangeSpeed(0.8,0.7,0.8,60,false);
                        break;
                    case 11:
                        slalomAutoState += drive.gyroSmoothStraight(0.7,0.9,1500,-120,false);
                        break;
                    case 12:
                        slalomAutoState += drive.gyroArcTurn_ChangeSpeed(0.8,0.7,-0.8,60,true);
                }
                break;

            // バウンド経路
            case BOUNCE_AUTO:

                // 前進プログラムテスト
            case DEFAULT_AUTO:
                switch (defaultAutoState){
                    case 0:
                        defaultAutoState += drive.gyroSmoothStraight(0.5,0.7,5000,0,true);
                        System.out.println("first");
                        break;
                    case 1:
                        drive.gyroSmoothStraight(0.5,0.7,10000,0,true);
                        System.out.println("second");
                        break;
                }
                break;

            case TUNING:
                break;

        }
//        } else {
//            drive.stopMotor();
//
//        }

//        loop_count++;
    }

    /**
     * この関数は操作制御が有効になっているときに一度だけ呼び出されます。
     */
    @Override
    public void teleopInit() {
        drive.init();
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
        double stickRY = controller.getY(GenericHID.Hand.kRight);

        double stickLR = 0.35 * stickLX + 0.4 * stickRX;

        //スピード制
        if (Math.abs(stickLR) >= 0.5) {
            stickLR = 0.6 * Math.signum(stickLR);
        }


        //足回りモーター
        double xSpeed = -0.8 * convertStickSigmoid(stickLY);
        if(controller.getXButton()){
            System.out.println("slow");
        }
        double zRotation = convertStickSigmoid(stickLR);
        drive.arcadeDrive(xSpeed, zRotation, true);


//        if (controller.getAButton()) {
//            victor.set(0.5);
//        } else {
//            victor.stopMotor();
//        }

//        if (compressor_timer.get() < 0.7) {
//            c.start();
//        } else {
//            c.stop();
//        }
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
        drive.init();
    }

    /**
     * このメソッドはテストモード中に定期的に呼び出されます。
     */
    @Override
    public void testPeriodic() {
        System.out.println(gyro.getAngle());
    }

    // スティックの値をシグモイド関数で変換
    private double convertStickSigmoid(double zRotation) {
        return 2 / (1 + Math.exp(-3 * zRotation)) - 1;
    }

}
