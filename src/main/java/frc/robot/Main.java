// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * このクラスに静的変数を追加したり、初期化を行ったりしないでください。
 * 実行内容がわからない場合は、パラメータークラスをstartRobot呼び出しに変更する場合を除いて、
 * このファイルを変更しないでください。
 */
public final class Main
{
    private Main() {}

    /**
     * メイン関数の初期化メソッド。 ここでは初期化を追加しないでください。
     * メインのRobotクラス（名前）を変更する場合は、パラメーターの種類を変更します。
     */
    public static void main(String... args)
    {
        RobotBase.startRobot(Robot::new);
    }
}
