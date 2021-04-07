// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.cscore.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

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
    private static final String GALACTIC_AUTO = "Galactic Auto";
    private static final String PATH_A = "Path_A";
    private static final String PATH_B = "Path_B";
    private static String RED_OR_BLUE = "";
    private static String TELEOP_MODE = "Arcade";
    //    private static String Route = "A_red";
    private String autoSelected;
    private final SendableChooser<String> chooser = new SendableChooser<>();
    private final Timer timer = new Timer();
    public final Timer compressor_timer = new Timer();
    private final Print print = new Print();
    public final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
    Encoder encoderL = new Encoder(0, 1, false, Encoder.EncodingType.k2X);
    Encoder encoderR = new Encoder(2, 3, false, Encoder.EncodingType.k2X);
    private final Drive drive = new Drive(new PWMVictorSPX(0), new PWMVictorSPX(1), encoderL, encoderR, gyro);

    public final XboxController controller = new XboxController(0);
    public final Joystick stick = new Joystick(0);

    Compressor c = new Compressor();
    VictorSP victor = new VictorSP(2);

    Rect contourRect = null;
    double length = 10000;
    boolean pos_or_neg;

    double stickAngle_deg = 0;

    private int autoState = 0, pathA_State = 0, pathA_Red_State = 0, pathA_Blue_State = 0, pathB_State = 0, pathB_Red_State = 0, pathB_Blue_State = 0, testState = 0;

    int loop_count;

    /**
     * このメソッドはロボットが最初に起動されたときに実行され、初期化コードを書くことができます。
     */
    @Override
    public void robotInit() {
        UsbCamera camera = CameraServer.getInstance().startAutomaticCapture("USBCamera", 0);
        camera.setResolution(320, 240);
        camera.setFPS(30);
        chooser.setDefaultOption("Default Auto", DEFAULT_AUTO);
        chooser.addOption("Barrel Racing Path", BARREL_RACING_AUTO);
        chooser.addOption("Slalom Path", SLALOM_AUTO);
        chooser.addOption("Bounce Path", BOUNCE_AUTO);
        chooser.addOption("Galactic Auto", GALACTIC_AUTO);
        chooser.addOption("Tuning", TUNING);
//        chooser.addOption("PathA", PATH_A);
        SmartDashboard.putData("Auto choices", chooser);
        gyro.calibrate();
        c.stop();

        new Thread(() -> {
            CvSink cvSink = CameraServer.getInstance().getVideo();
            CvSource outputStream = CameraServer.getInstance().putVideo("OpenCV_USBCamera", 320, 240);

            Mat source = new Mat();
            Mat output = new Mat();
            List<MatOfPoint> contours = new ArrayList<>();//輪郭
            Mat hierarchy = new Mat();
            double viewAngle = 68.5;

            while (!Thread.interrupted()) {
                if (cvSink.grabFrame(source) == 0) {
                    continue;
                }
                contours.clear();
                Core.inRange(source, new Scalar(0, 150, 150), new Scalar(100, 255, 255), output);
                Imgproc.findContours(output, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_NONE);

                String text = (RED_OR_BLUE.isEmpty()) ? "Searching..." : autoSelected + " | " + RED_OR_BLUE;
                Imgproc.putText(source, text, new Point(8, 230), Core.FONT_HERSHEY_DUPLEX, 0.6, new Scalar(255, 255, 255));

                double maxArea = 0;
                contourRect = null;
                for (MatOfPoint contour : contours) {
                    double area = Imgproc.contourArea(contour);
                    if (area > maxArea) {
                        maxArea = area;
                        contourRect = Imgproc.boundingRect(contour);
                    }
                }

                if (contourRect != null) {
                    double angle = Math.toRadians(viewAngle * (contourRect.width / 320.0));
                    length = 17 / (2 * Math.tan(angle / 2));
                    Imgproc.rectangle(source, contourRect.tl(), contourRect.br(), new Scalar(255, 0, 0), 1);
                } else {
                    length = 10000;
                }
                outputStream.putFrame(source);
            }
        }).start();
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
        compressor_timer.reset();
        compressor_timer.start();
        timer.reset();
        timer.start();
        autoState = 0;
        pathA_State = 0;
        pathA_Red_State = 0;
        pathA_Blue_State = 0;
        pathB_State = 0;
        pathB_Red_State = 0;
        pathB_Blue_State = 0;
    }

    /**
     * このメソッドは自律走行中に定期的に呼び出されます。
     */
    @Override
    public void autonomousPeriodic() {
        System.out.println(autoSelected);
        System.out.println(RED_OR_BLUE);

        switch (autoSelected) {
            // バレルレーシング経路
            case BARREL_RACING_AUTO:
                System.out.println(gyro.getAngle());
                switch (autoState) {
                    case 0:
                        autoState += drive.gyroStraight_ChangeSpeed(0.7, 0.9, 10000, 0, false);
                        break;
                    case 1:
                        autoState += drive.gyroArcTurn(0.8, 0.8, 360, false);
                        break;
                    case 2:
                        autoState += drive.gyroSmoothStraight(0.8, 1, 7000, 360, false);
                        break;
                    case 3:
                        autoState += drive.gyroArcTurn(0.8, -0.77, 315, false);
                        break;
                    case 4:
                        autoState += drive.gyroSmoothStraight(0.8, 1, 6000, 45, false);
                        break;
                    case 5:
                        autoState += drive.gyroArcTurn(0.8, -0.82, 225, false);
                        break;
                    case 6:
                        autoState += drive.gyroSmoothStraight(0.8, 1, 21500, -180, true);
                        break;
                }
                break;

            // スラローム経路
            case SLALOM_AUTO:
//                System.out.println(gyro.getAngle());
                switch (autoState) {
                    case 0:
                        autoState += drive.gyroStraight_ChangeSpeed(0.6, 0.7, 2000, 0, false);
                        break;
                    case 1:
                        autoState += drive.gyroArcTurn_ChangeSpeed(0.7, 0.8, -0.7, 70, false);
                        break;
                    case 2:
                        autoState += drive.gyroArcTurn_ChangeSpeed(0.8, 0.9, 0.7, 70, false);
                        break;
                    case 3:
                        autoState += drive.gyroSmoothStraight(0.8, 1, 8000, 0, false);
                        break;
                    case 4:
                        autoState += drive.gyroArcTurn_ChangeSpeed(0.8, 0.7, 0.7, 60, false);
                        break;
                    case 5:
                        autoState += drive.gyroSmoothStraight(0.7, 0.8, 2000, 60, false);
                        break;
                    case 6:
                        autoState += drive.gyroArcTurn(0.8, -1, 300, false);
                        break;
                    case 7:
                        autoState += drive.gyroStraight_ChangeSpeed(0.8, 0.8, 1000, -240, false);
                        break;
                    case 8:
                        autoState += drive.gyroArcTurn_ChangeSpeed(0.8, 0.9, 0.7, 60, false);
                        System.out.println("turn");
                        break;
                    case 9:
                        autoState += drive.gyroSmoothStraight(0.8, 1, 9500, -180, false);
                        System.out.println("Straight");
                        break;
                    case 10:
                        autoState += drive.gyroArcTurn_ChangeSpeed(0.8, 0.7, 0.8, 60, false);
                        break;
                    case 11:
                        autoState += drive.gyroSmoothStraight(0.7, 0.9, 1500, -120, false);
                        break;
                    case 12:
                        autoState += drive.gyroArcTurn_ChangeSpeed(0.8, 0.7, -0.8, 60, true);
                }
                break;

            // バウンド経路
            case BOUNCE_AUTO:
                System.out.println(gyro.getAngle());
                switch (autoState) {
                    case 0:
                        autoState += drive.gyroStraight_ChangeSpeed(0.8, 0.9, 250, 0, false);
                        break;
                    case 1:
                        autoState += drive.gyroArcTurn_ChangeSpeed(0.9, 0.6, -0.64, 90, true);
                        break;
                    case 2:
                        autoState += drive.gyroArcTurn_ChangeSpeed(0.6,0.9,-0.7,-20,false);
                        break;
                    case 3:
                        autoState += drive.gyroStraight(0.9,-5000,-110,false);
                        break;
                    case 4:
                        autoState += drive.gyroArcTurn(0.8,-0.88,-160,false);
                        break;
                    case 5:
                        autoState += drive.gyroStraight_ChangeSpeed(0.9,0.6,-5500,-270,true);
                        break;
                    case 6:
                        autoState += drive.gyroStraight_ChangeSpeed(0.8, 0.9, 5500, -270, false);
                        break;
                    case 7:
                        autoState += drive.gyroArcTurn(0.9, -0.61, 180, false);
                        break;
                    case 8:
                        autoState += drive.gyroStraight_ChangeSpeed(0.9, 0.5, 5200, -450, true);
                        break;
                    case 9:
                        autoState += drive.gyroStraight_ChangeSpeed(0.8, 0.9, -700, -450, false);
                        break;
                    case 10:
                        autoState += drive.gyroArcTurn(0.8, -0.65, -90, true);
                        break;
                }
                break;

            // 前進プログラムテスト
            case DEFAULT_AUTO:
                drive.arcadeDrive(0.4,0);
                break;

            case GALACTIC_AUTO:
                if (compressor_timer.get() < 0.7) {
                    c.start();
                } else {
                    c.stop();
                }
                switch (autoState) {
                    case 0:
                        if (timer.get() > 0.2) {
                            autoState += 1;
                        }
                        break;
                    case 1:
                        victor.set(0.5);
                        autoState += drive.gyroStraight_ChangeSpeed(0.6, 0.8, 1000, 0, false);
                        break;
                    case 2:
                        if (length < 200) {
                            RED_OR_BLUE = "red";
                            autoState = 0;
                            if (contourRect.x < 50) {
                                autoSelected = PATH_B;
                            } else {
                                autoSelected = PATH_A;
                            }
                        } else {
                            RED_OR_BLUE = "blue";
                            autoState += 1;
                        }
                        break;
                    case 3:
                        autoState += drive.gyroArcTurn(0.8, 0.75, 40, false);
                        break;
                    case 4:
                        autoState += drive.gyroSmoothStraight(0.8,1,3500,40,false);
                        break;
                    case 5:
                        autoState += drive.gyroArcTurn_ChangeSpeed(0.8, 0.65, -0.75, 40, false);
                        break;
                    case 6:
                        autoState += drive.gyroStraight_ChangeSpeed(0.65, 0.5, 1000, 0, true);
                        break;
                    case 7:
                        if(contourRect.x < 100){
                            autoSelected = PATH_B;
                        } else {
                            autoSelected = PATH_A;
                        }
                        autoState = 0;
                        break;
                }
                break;

            case PATH_A:
                if(Objects.equals(RED_OR_BLUE,"red")){
                    switch (autoState){
                        case 0:
                            autoState += drive.gyroStraight_ChangeSpeed(0.8,0.9,2000,0,false);
                            break;
                        case 1:
                            autoState += drive.gyroArcTurn(0.9,0.8,20,false);
                            break;
                        case 2:
                            autoState += drive.gyroArcTurn_ChangeSpeed(0.7,0.9,-1.1,120,false);
                            break;
                        case 3:
                            autoState += drive.gyroArcTurn(0.9,1.5,180,false);
                            break;
                        case 4:
                            autoState += drive.gyroStraight_ChangeSpeed(0.9,0.6,8000,60,true);
                            break;
                    }

                } else if(Objects.equals(RED_OR_BLUE,"blue")){
                    switch (autoState){
                        case 0:
                            break;
                    }
                }
                break;

            case PATH_B:
                if(Objects.equals(RED_OR_BLUE,"red")){
                    switch (autoState){
                        case 0:
                            autoState += drive.gyroSmoothArcTurn(0.6,0.8,-1.2,60,true);
                            break;
                        case 1:
                            autoState += drive.gyroArcTurn(0.8,1.2,180,true);
                            break;
                    }

                } else if(Objects.equals(RED_OR_BLUE,"blue")){
                    switch (autoState){
                        case 1:
                            break;
                    }
                }
                break;

            case TUNING:
                break;

        }
    }

    /**
     * この関数は操作制御が有効になっているときに一度だけ呼び出されます。
     */
    @Override
    public void teleopInit() {
        drive.init();
        compressor_timer.reset();
        compressor_timer.start();
        TELEOP_MODE = "Arcade";
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

        //操作モード切替
        if(controller.getStartButtonPressed()){
            TELEOP_MODE = Objects.equals(TELEOP_MODE, "Arcade") ? "Tank" : "Arcade";
        }

        //操作モード
        if (Objects.equals(TELEOP_MODE, "Arcade")){
            final double K_LX = 0.3;
            final double K_RX = 0.4;

            //右スティックと左スティックの中から大きい値を採用
            double stickLR;
            if (Math.signum(stickLX) == Math.signum(stickRX)) {
                stickLR = Math.abs(stickLX) > Math.abs(stickRX) ? K_LX * stickLX : K_RX * stickRX;
            } else {
                stickLR = K_LX * stickLX + K_RX * stickRX;
            }

            //スピード制限
            if (Math.abs(stickLR) >= 0.5) {
                stickLR = 0.6 * Math.signum(stickLR);
            }

            //RB: 低速モード切り替え
            double highSpeed = controller.getBumper(GenericHID.Hand.kRight) ? 0.6 : 0.8;

            //足回りモーター
            double xSpeed = -highSpeed * convertStickSigmoid(stickLY);
            double zRotation = convertStickSigmoid(stickLR);

            xSpeed *= controller.getTriggerAxis(GenericHID.Hand.kLeft) * 0.25 + 1;
            zRotation *= controller.getTriggerAxis(GenericHID.Hand.kLeft) * 0.25 + 1;

            xSpeed /= controller.getTriggerAxis(GenericHID.Hand.kRight) + 1;
            zRotation /= controller.getTriggerAxis(GenericHID.Hand.kRight) + 1;

            drive.arcadeDrive(xSpeed, zRotation, true);

        } else if(Objects.equals(TELEOP_MODE, "Tank")){
            drive.tankDrive(-0.8*stickLY,-0.8*stickRY);

        }

        //LB: 回収機構
        if (controller.getBumper(GenericHID.Hand.kLeft)) {
            victor.set(0.5);
        } else {
            victor.stopMotor();
        }

//        double targetAngle, progressAngle, error;
//        double isForward;
//        double stick_z = stick.getZ();
//        double stick_X = Math.abs(stick.getX()) < 0.1 ? 0 : stick.getX();
//        double stick_Y = Math.abs(stick.getY()) < 0.1 ? 0 : -stick.getY();
//        double stickAngle_rad = Math.atan2(stick_X,stick_Y);
//        double stickMagnitude = Math.min(1,Math.sqrt(Math.pow(stick_X,2) + Math.pow(stick_Y,2)))*((stick.getThrottle()-1)/-2);
////        double stickMagnitude = Math.max(Math.abs(Math.sin(stickAngle_rad)),Math.abs(Math.cos(stickAngle_rad))) * Math.sqrt(Math.pow(stick_X,2) + Math.pow(stick_Y,2));
//        stickAngle_deg = stickMagnitude==0 ? stickAngle_deg :stickAngle_rad*180/Math.PI;
//        double[] direction = getDirection(gyro.getAngle(),stickAngle_deg);
//
//        if(stickMagnitude == 0 && Math.abs(stick_z) > 0.05){
//            drive.arcadeDrive(0,stick_z*0.45);
//        } else {
//            drive.stickGyro(stickMagnitude * direction[0], direction[1]);
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
    }

    // スティックの値をシグモイド関数で変換
    private double convertStickSigmoid(double zRotation) {
        return 2 / (1 + Math.exp(-3 * zRotation)) - 1;
    }

    private double[] getDirection(double robot_angle,double target){
        double error = Math.floorMod((long)(robot_angle-target),360);
        double[] data = new double[2];
        data[0] = error < 90 || error > 270 ? 1:-1;
        data[1] = (error+90) % 180 -90;
        return data;
    }
}
