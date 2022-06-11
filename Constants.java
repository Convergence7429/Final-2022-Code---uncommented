package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class Constants {

  // Index
  public static int flMotorIndex = 6;
  public static int blMotorIndex = 21;
  public static int frMotorIndex = 20;
  public static int brMotorIndex = 13;

  public static int masterShooterMotorIndex = 10;
  public static int slaveShooterMotorIndex = 11;
  public static int hoodMotorIndex = 3;

  public static int intakeMotorIndex = 24;
  public static int indexerMotorIndex = 1;
  public static int intakeAngleMotorIndex = 8;

  public static int centerClimberHeightMotorIndex = 23;

  public static int stickIndex = 0;
  public static int xboxIndex = 1;

  //////////////////////////////////////////////////////
  // Controllers

  public static Joystick stick = new Joystick(stickIndex);
  public static Joystick xbox = new Joystick(xboxIndex);

  //////////////////////////////////////////////////////
  // Functions

  public static double quadraticPositionAndSpeed(double minimumMotorSpeed, double maximumMotorSpeed,
      double positionGoal, double currentPosition) {

    double a = ((positionGoal * maximumMotorSpeed - minimumMotorSpeed * positionGoal
        - minimumMotorSpeed * (positionGoal / 2) + minimumMotorSpeed * (positionGoal / 2))
        / (positionGoal * (positionGoal / 2) * ((positionGoal / 2) - positionGoal)));

    double b = ((maximumMotorSpeed - a * (positionGoal / 2) * (positionGoal / 2) - minimumMotorSpeed)
        / (positionGoal / 2));

    double speed = a * currentPosition * currentPosition + b * currentPosition + minimumMotorSpeed;

    return speed;
  }

  
}