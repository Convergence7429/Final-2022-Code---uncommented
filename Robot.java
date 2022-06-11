package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticHub;
import com.kauailabs.navx.frc.*;

public class Robot extends TimedRobot {

  static Shooter shooter = new Shooter();
  static DriveTrain drive = new DriveTrain();
  static Intake intake = new Intake();
  static Climber climber = new Climber();
  static PowerDistribution PDP = new PowerDistribution(6, ModuleType.kRev);
  static PneumaticHub ph = new PneumaticHub(15);

  static AHRS gyro = new AHRS();

  public void clearStickyFaults() {
    PDP.clearStickyFaults();
    ph.clearStickyFaults();

    shooter.masterShooterMotor.clearStickyFaults();
    shooter.slaveShooterMotor.clearStickyFaults();
    shooter.hoodMotor.clearFaults();

    drive.flMotor.clearFaults();
    drive.frMotor.clearFaults();
    drive.blMotor.clearFaults();
    drive.brMotor.clearFaults();

    intake.intakeMotor.clearFaults();
    intake.indexerMotor.clearFaults();
  }

  static boolean isRed;

  int autoStage = 0;
  Timer autonomousTimer = new Timer();

  ///////////////////////////////////////////////////////
  // Robot

  @Override
  public void robotInit() {
    clearStickyFaults();
    drive.driveTrainInit();
    shooter.shooterRobotInit();
    climber.climberInit();
    intake.intakeInit();

    if (DriverStation.getAlliance().equals(Alliance.Red)) {
      isRed = true;
    } else {
      isRed = false;
    }

  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putString("Pressure", (ph.getPressure(0)) + " psi");
  }

  ////////////////////////////////////////////////////////
  // Autonomous

  @Override
  public void autonomousInit() {
    clearStickyFaults();
    autonomousTimer.reset();
    autonomousTimer.start();
    autoStage = 0;
    shooter.shooterInit();
    shooter.resetHoodEncoders();
    intake.isIntakeDown = false;
    climber.isLevel2ClimberDown = true;
    ph.disableCompressor();
    gyro.calibrate();
    gyro.zeroYaw();
    gyro.calibrate();
    gyro.zeroYaw();
  }

  // @Override
  // public void autonomousPeriodic(){
  // System.out.println(gyro.getYaw());
  // // drive.driveTrainByControls(0.0, 0.0, , false);
  // drive.mecanumDrive.driveCartesian(0.0, 0.0,
  // Constants.quadraticPositionAndSpeed(0.1, 0.5, 90.0, gyro.getYaw()));

  // }

  Timer turningTimer = new Timer();

  @Override
  public void autonomousPeriodic(){
    shooter.dumpShot();
    shooter.shooterIdle(0.20);
    System.out.println("timer: " + autonomousTimer.get());
    System.out.println("yaw: " + gyro.getYaw());

    switch(autoStage){

      case 0 : {
        autoStage = 1;
        drive.stopMotors();
        drive.resetDriveTrainEncoders();
        intake.intakeDown();
        shooter.shootInit();
        break;
      }


      case 1 : {
        shooter.dumpShot = true;
        if(autonomousTimer.get() > 1.0){
          autoStage = 2;
          shooter.dumpShot = false;
          drive.stopMotors();
          drive.resetDriveTrainEncoders();
          intake.intakeMotor.set(-0.65);
        }
        break;
      }

      case 2 : {
        drive.driveTrainByInches(40.0, 0);
        if(autonomousTimer.get() > 5.0){
          autoStage = 4;
          drive.stopMotors();
          drive.resetDriveTrainEncoders();
        }
        break;
      }

      case 4 : {
        drive.driveTrainByInches(35.0, 1);
        if(autonomousTimer.get() > 9.0){
          autoStage = 5;
          drive.stopMotors();
          drive.resetDriveTrainEncoders();
        }
        break;
      }

      case 5 : {
        if(shooter.centerRobotOnTarget()){
          shooter.dumpShot = true;
        }
        break;
      }
    }

  }

  ///////////////////////////////////////////////////////////
  // Tele-operated

  Timer teleopTimer = new Timer();

  @Override
  public void teleopInit() {
    clearStickyFaults();
    shooter.shooterInit();
    intake.isIntakeDown = true;
    climber.isLevel2ClimberDown = true;
    climber.isTraverseClimberActivated = false;
    ph.enableCompressorAnalog(115, 120);
    teleopTimer.reset();
    teleopTimer.start();
    intake.intakingByColor = true;
  }

  @Override
  public void teleopPeriodic() {

    SmartDashboard.putNumber("Timer", DriverStation.getMatchTime());

    if (Constants.stick.getRawButton(2) || (Constants.xbox.getRawAxis(2) > 0.5)) {
      shooter.centerRobotOnTarget();
    } 
    // else if (Constants.stick.getRawButton(3) || Constants.xbox.getRawButton(6)) {
    //   shooter.distanceRobotFromTargetCorrectly();
    // } 
    else {
      drive.driveTrainByControls(Constants.stick.getRawAxis(1), Constants.stick.getRawAxis(0),
          Constants.stick.getRawAxis(2));
    }

    if(Constants.xbox.getRawButtonPressed(5)){
      shooter.hoodMotor.getEncoder().setPosition(10.5);
    }
    
    intake.intakeTeleop();
    //shooter.shoot();
    shooter.dumpShot();
    shooter.lowDumpShot();
    // shooter.hangarShot();
    shooter.shooterIdle(0.0);
    shooter.hoodControl();
    climber.climberTeleop();
  }

  /////////////////////////////////////////////////////////////
  // Test

  @Override
  public void testInit() {

    shooter.shooterInit();
    autonomousTimer.reset();
    autonomousTimer.start();
    ph.enableCompressorAnalog(115, 120);
  }

  @Override
  public void testPeriodic() {
    shooter.masterShooterMotor.set(0.0);
    intake.indexerMotor.set(0.0);
    // if(Constants.stick.getRawButton(1)){
    // canSparkMax.set(0.2);
    // } else if(Constants.stick.getRawButton(2)){
    // canSparkMax.set(-0.2);
    // } else {
    // canSparkMax.set(0.0);
    // canSparkMax.getPIDController().setReference(canSparkMax.getEncoder().getPosition(),
    // ControlType.kPosition);
    // }

    // System.out.println("intakeUpSolenoid: " + Robot.intake.intakeUp.get());
    // System.out.println("intakeDownSolenoid: " + Robot.intake.intakeDown.get());

    // if(Constants.stick.getRawButtonPressed(1)){
    // Robot.intake.intakeUp.set(false);
    // Robot.intake.intakeDown.set(true);
    // }

    // if(Constants.stick.getRawButtonPressed(2)){
    // Robot.intake.intakeDown.set(false);
    // Robot.intake.intakeUp.set(true);
    // }

    // intake.intakeAngleMotor.set(0.2);
    // System.out.println(intake.intakeAngleMotor.get());
    // shooter.masterShooterMotor.set(ControlMode.Velocity, 0.2,
    // DemandType.ArbitraryFeedForward, 0.5);
    // shooter.masterShooterMotor.pid
    // shooter.masterShooterMotor.set(ControlMode.Velocity, 1500.0);
    // System.out.println(shooter.masterShooterMotor.getSelectedSensorVelocity(1));

    // intake.indexByColor();

    Color detectedColor = intake.indexerColorSensor.getColor();
    System.out.println(
    "red: " + detectedColor.red + ", blue: " + detectedColor.blue + ", green: " +
    detectedColor.green);

    // // if (!Robot.shooter.shooting && !Robot.shooter.dumpShot) {
    // // if ((Math.abs(detectedColor.red - 0.32) < 0.02) ||
    // (Math.abs(detectedColor.blue - 0.305) < 0.02)) {
    // // intake.indexerMotor.set(0.0);
    // // } else {
    // // intake.indexerMotor.set(-0.2);
    // // }
    // // }

    // intake.indexByColor();

    // System.out.println(doubleSolenoid.get());

    // if(Constants.stick.getRawButton(1)){
    // intake.intakeAngleMotor.set(-0.5);
    // } else if (Constants.stick.getRawButton(2)) {
    // intake.intakeAngleMotor.set(0.5);
    // } else {
    // intake.intakeAngleMotor.set(0.0);
    // }

    // System.out.println(solenoid0.get());
    // System.out.println(solenoid1.get());
    // System.out.println("*******************************");

    // if(Constants.xbox.getRawButtonPressed(7)){
    // solenoid1.set(true);
    // // if(solenoid0.get()){
    // // solenoid0.set(false);
    // // solenoid1.set(true);
    // // } else if(solenoid1.get()){
    // // solenoid1.set(false);
    // // solenoid0.set(true);
    // // }
    // //solenoid1.set(true);
    // //doubleSolenoid.toggle();
    // }

    // if(Constants.xbox.getRawButtonPressed(8)){
    // solenoid0.set(true);
    // }

    // if(Constants.xbox.getRawButtonPressed(2)){
    // ph.fireOneShot(1);
    // }
    // ph.getCompressorConfigType()
    // ph.
    // System.out.println(ph.get);

    // if(autonomousTimer.get() > 10.0){
    // doubleSolenoid.set(Value.kReverse);
    // autonomousTimer.reset();
    // autonomousTimer.start();
    // }

    // System.out.println("intake angle motor encoders: " +
    // intake.intakeAngleMotor.getEncoder().getPosition());

    // still have the intake up and down stuff. Move at 0.5 speed until get within a
    // certain number of encoder counts of the goal
    // then PID
    // have buttons that go up and down at 0.35 or something and if you press the
    // buttons, you still switch between intake up and down
    // that finish the way

    // could double check distance calculation

    //////////////////////////////////////////////////////////////////////////
    // Color detectedColor = intake.indexerColorSensor.getColor();
    // System.out.println("blue: " + new Color(0.0, 0.290, 0.0).blue);
    // //double difference = Math.abs(detectedColor.blue - 0.2975); // or 0.295. <
    ////////////////////////////////////////////////////////////////////////// 0.015
    // double difference = Math.abs(detectedColor.red - 0.295); // 0.2932 // 0.2976
    // System.out.println("difference: " + difference);

    // if (difference < 0.025){
    // intake.indexerMotor.set(0.0);
    // } else {
    // intake.indexerMotor.set(-0.2);
    // }

    // ColorMatchResult match = intake.colorMatch.matchClosestColor(detectedColor);
    // System.out.println(match.color.red + " " + match.color.blue + " " +
    // match.color.green);

    // if(Constants.stick.getRawButtonPressed(1)){
    // shooter.shooting = !shooter.shooting;
    // }

    // if ((Math.abs(detectedColor.red - new Color(0.195, 0.374, 0.432).red) < 0.05)
    // && detectingBall) {
    // ballPresent = true;
    // intakeTimer.reset();
    // intakeTimer.start();
    // } else {
    // if(!ballPresent){
    // intake.indexerMotor.set(-0.2);
    // }
    // }

    // if(ballPresent){
    // detectingBall = false;
    // intake.indexerMotor.set(-0.2);
    // if(intakeTimer.get() > 0.6){
    // intake.indexerMotor.set(0.0);
    // if(shooter.shooting || shooter.dumpShot || (Constants.xbox.getPOV() == 180)){
    // detectingBall = true;
    // ballPresent = false;
    // }
    // }
    // }

    // // if(ballPresent){

    // // } else {

    // // }

    // // else {
    // // ballPresent = false;
    // // intake.indexerMotor.set(-0.2);
    // // }

    // if(ballPresent){

    // }
    /////////////////////////////////////////////////////////////////////

    // shooter.masterShooterMotor.set(ControlMode.PercentOutput, 0.87);
    // System.out.println("calculated " +
    // (shooter.shooterWheelLinearVelocityToMotorVelocity(shooter.calculatedVelocity)));
    // System.out.println("actual speed " +
    // (shooter.masterShooterMotor.getSelectedSensorVelocity()));
    // System.out.println("calculated angle: " + (90.0 - (shooter.calculatedAngle *
    // 180.0 / Math.PI)));
    // System.out.println("actual angle: " +
    // shooter.hoodMotor.getEncoder().getPosition());

    // if(Constants.stick.getRawButtonPressed(5)){
    // shooter.angleError -= 0.05;
    // } else if(Constants.stick.getRawButton(6)){
    // shooter.angleError += 0.05;
    // } else if(Constants.stick.getRawButton(3)){
    // shooter.angleError -= 0.01;
    // } else if(Constants.stick.getRawButtonPressed(4)){
    // shooter.angleError += 0.01;
    // }

    // //System.out.println(shooter.getXDistanceFromCenterOfHub(NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0)));
    // System.out.println(shooter.getXDistanceFromFrontOfRobotToFender(NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0)));
    // System.out.println(shooter.angleError);

    // public static float getXDistanceFromCenterOfHub(double verticalAngle) { // x
    // // distance from front of robot to fender. better for testing
    // return (float) (((y - cameraHeight) / Math.tan(Math.toRadians(verticalAngle +
    // cameraAngle - angleError))) - horizontalDistanceFromLimelightToFrontOfRobot -
    // distanceFromFenderToTape);
    // }

    // if(Constants.stick.getRawButtonPressed(7)){
    // shooterActivated = !shooterActivated;
    // }
    // if(Constants.stick.getRawButtonPressed(11)){ // adjust shooter speed
    // shooterActivated = true;
    // shooterSpeed -= 0.05;
    // }
    // if(Constants.stick.getRawButtonPressed(12)){
    // shooterActivated = true;
    // shooterSpeed += 0.05;
    // }
    // if(shooterActivated){
    // shooter.masterShooterMotor.set(ControlMode.PercentOutput, shooterSpeed);
    // } else {
    // shooter.masterShooterMotor.set(0.0);
    // }
  }

  //////////////////////////////////////////////////////////////
  // Disabled

  @Override
  public void disabledInit() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setDouble(1.0);
  }

  @Override
  public void disabledPeriodic() {
  }

}