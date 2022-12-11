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
    public static final int flyWheelID = 2;
    public static final int rightFlyWheelID = 17;
    //right flywheel only needed depending on if it will be on the final shooter

    public static final int feedWheelID = 3;

    public static final int joystick = 0;

    public final static int feedButton = 1;
    public final static int speedUpButton = 3;
    public final static int stopButton = 4;
    public final static int lowSpeedButton = 8;
    public final static int midSpeedButton = 10;
    public final static int highSpeedButton = 12;

    public static final int encoderTicks = 4096;
    //Placeholder speeds. each is meant to be in RPM
    public static final double lowSpeed = 400;
    public static final double midSpeed = 1000;
    public static final double highSpeed = 2000;
    public static final double fullSpeedInRpm = 2400;

    //Placeholder speed. Speed is in the range of [-1,1]
    public static final double feedSpeed = 0.5;
    
    //Placeholder numbers. We will probably have 1 flywheel on the final product.
    public static final class rightFlywheelFF {
        public static final double kS = 0.53709;
        public static final double kV = 0.28844;
        public static final double kA = 0.015011;
    }
        
    public static final class leftFlywheelFF {
        public static final double kS = 0.57443;
        public static final double kV = 0.29162;
        public static final double kA = 0.011557;
    }        
    public static final class PIDConstants {
        public static final double kP = 0.0006;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
      }
}