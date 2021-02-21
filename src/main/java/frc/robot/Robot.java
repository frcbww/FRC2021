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
public class Robot extends TimedRobot
{
    private static final String DEFAULT_AUTO = "Default";
    private static final String CUSTOM_AUTO = "My Auto";
    private String autoSelected;
    private final SendableChooser<String> chooser = new SendableChooser<>();
    private final Timer timer = new Timer();

    private final DifferentialDrive robotDrive = new DifferentialDrive(new PWMVictorSPX(0), new PWMVictorSPX(1));
    private final Joystick stick = new Joystick(0);
    Encoder encoderL = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
    Encoder encoderR = new Encoder(2, 3, false, Encoder.EncodingType.k4X);

    /**
     * このメソッドはロボットが最初に起動されたときに実行され、初期化コードを書くことができます。
     */
    @Override
    public void robotInit()
    {
        chooser.setDefaultOption("Default Auto", DEFAULT_AUTO);
        chooser.addOption("My Auto", CUSTOM_AUTO);
        SmartDashboard.putData("Auto choices", chooser);
    }

    /**
     *  このメソッドは、モードに関係なく、すべてのロボットパケットと呼ばれます。
     *  無効、自律、遠隔操作、およびテスト中に実行する診断などの項目にこれを使用します。
     *
     *  これは、モード固有の定期的な方法の後、LiveWindowとSmartDashboardの統合更新の前に実行されます。
     */
    @Override
    public void robotPeriodic() {}

    /**
     * この自律型（上記の選択コード）は、ダッシュボードを使用して異なる自律型モードを選択するための方法を示しています。
     * また、送信可能な選択コードは、Java SmartDashboardで機能します。
     *
     * LabVIEWダッシュボードを使用する場合は、すべての選択コードを削除し getString コードのコメントを解除して、
     * ジャイロの下のテキストボックスから自動名(auto name)を取得します。
     *
     * 上記のセレクターコードに追加コマンド（コメントに載っているの例など）を追加するか、
     * 以下のスイッチ構造に追加の文字列とコマンドを追加して比較することにより、自動モードを追加できます。
     */
    @Override
    public void autonomousInit()
    {
        autoSelected = chooser.getSelected();
        // autoSelected = SmartDashboard.getString("Auto Selector", defaultAuto);
        System.out.println("Auto selected: " + autoSelected);
        timer.reset();
        timer.start();
        encoderL.reset();
        encoderR.reset();
    }

    /** このメソッドは自律走行中に定期的に呼び出されます。 */
    @Override
    public void autonomousPeriodic()
    {
        // 前進プログラムテスト
        if (timer.get() < 1.0){
            double zRotation = convertStraightEncoder(encoderL.get(), encoderR.get());
            robotDrive.arcadeDrive(0.5, zRotation, true);
        } else {
            robotDrive.stopMotor();
        }

        switch (autoSelected)
        {
            case CUSTOM_AUTO:
                // Put custom auto code here
                break;
            case DEFAULT_AUTO:
            default:
                // Put default auto code here
                break;
        }
    }

    /** この関数は操作制御が有効になっているときに一度だけ呼び出されます。 */
    @Override
    public void teleopInit() {
        encoderL.reset();
        encoderR.reset();
    }

    /** このメソッドは操作制御中に定期的に呼び出されます。 */
    @Override
    public void teleopPeriodic() {
        //コントローラーデータ
        double stickLX = stick.getX();
        double stickLY = stick.getY();
        double stickRX = stick.getRawAxis(4);

        double stickLR = 0.3 * stickLX + 0.35 * stickRX;

        //スピード制限
        if (Math.abs(stickLR) >= 0.5) {
            stickLR = 0.5 * Math.signum(stickLR);
        }

        System.out.println(encoderL.get() + ", " + encoderR.get());

        //足回りモーター
        double xSpeed = -0.65 * convertStickSigmoid(stickLY);
        double zRotation = convertStickSigmoid(stickLR);
        robotDrive.arcadeDrive(xSpeed, zRotation, true);
    }

    /** この関数はロボットが無効化されたときに一度だけ呼び出されます。 */
    @Override
    public void disabledInit() {}

    /** この関数はロボットが無効化されたときに定期的に呼び出されます。 */
    @Override
    public void disabledPeriodic() {}

    /** この関数はテストモードが有効になっているときに一度だけ呼び出されます。 */
    @Override
    public void testInit() {}

    /** このメソッドはテストモード中に定期的に呼び出されます。 */
    @Override
    public void testPeriodic() {}

    // スティックの値をシグモイド関数で変換
    private double convertStickSigmoid(double value) {
        return 2 / (1 + Math.exp(-3 * value)) - 1;
    }

    // エンコーダーを使用した直進
    private double convertStraightEncoder(double leftEncoder, double rightEncoder) {
        double kP = 0.004;
        double diff = rightEncoder + leftEncoder;
        return kP * diff;
    }
}
