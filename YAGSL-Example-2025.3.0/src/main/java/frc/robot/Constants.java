// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static final class ElevatorConstants{
    
    public static final int leftElevatorID = 11;
    public static final int rightElevatorID = 12;
    public static final int limitSwitchPort = 1;

   //countsper inch = (360 * gear ratio)/(diamiter of sprocket * pi) 
    public static final double countsPerInch = (360 * 45 )/(2* 3.14159265359);
    
    public static final double downPos = .1;
    public static final double L1 = 7.25;
    public static final double L2 = 23;
    public static final double L3 = 20;
    public static final double L4 = 20;
    public static final double bottomPos = 10;
   
    public static final double minPos = 0;
    public static final double maxPos = 24;
    public static final double posTolerance = .25;

    public static final double maxVelocity = 0.5;
    public static final double maxAcceleration = 0.5;

    public static final int kP = 0;
    public static final int kI = 0;
    public static final int kD = 0;

    public static final int kS = 0;
    public static final int kG = 0;
    public static final int kV = 0;

    public static final double max_output = 5;
  
  }
// Shooter constants
  public static final class Shooter{
    public static final int RightMotorId = 21;
    public static final int LeftMotorId = 22;

    public static final int ForBeamID = 3;
    public static final int BackBeamID = 4;

    public static final double IntakeSpeed = 20;
    public static final double ReverseSpeed = 20;
    
    public static final double IndexSpeed = 20;
    public static final double L1Speed = 20;
    public static final double L24Speed = 20;



  }
}
