// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
    public static final int flyWheelID = 16;
    public static final int rightFlyWheelID = 17;
    //right flywheel only needed depending on if it will be on the final shooter

    public static final int feedWheelID = 0;

    public static final int joystick = 0;

    public final static int feedButton = 0;
    public final static int speedUpButton = 3;
    public final static int stopButton = 4;
    public final static int lowSpeedButton = 8;
    public final static int midSpeedButton = 10;
    public final static int highSpeedButton = 12;

    public static final int encoderTicks = 4096;

    public static final double lowSpeed = 400;
    public static final double midSpeed = 1000;
    public static final double highSpeed = 2000;
    //placeholder numbers
    public static final double fullSpeedInRpm = 2400;

    public static final double kS = 0.41733;
    public static final double kV = 0.4025;
    public static final double kA = 0.046839;
}