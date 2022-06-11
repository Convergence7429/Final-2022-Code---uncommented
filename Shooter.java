package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter {

    WPI_TalonFX masterShooterMotor = new WPI_TalonFX(Constants.masterShooterMotorIndex);
    WPI_TalonFX slaveShooterMotor = new WPI_TalonFX(Constants.slaveShooterMotorIndex);
    CANSparkMax hoodMotor = new CANSparkMax(Constants.hoodMotorIndex, MotorType.kBrushless);
    PWMSparkMax LEDsSparkMax = new PWMSparkMax(0);

    boolean shooting = false;
    boolean dumpShot = false;
    boolean lowDumpShot = false;
    boolean hangarShot = false;

    final double saturationVoltage = 12.5;

    public void resetHoodEncoders() {
        hoodMotor.getEncoder().setPosition(10.5);
    }

    public void shooterRobotInit() {
        slaveShooterMotor.follow(masterShooterMotor);
        hoodMotor.setInverted(true);
        hoodMotor.getEncoder().setPositionConversionFactor(0.65222);
        masterShooterMotor.configVoltageCompSaturation(saturationVoltage);
        slaveShooterMotor.configVoltageCompSaturation(saturationVoltage);
        masterShooterMotor.enableVoltageCompensation(true);
        slaveShooterMotor.enableVoltageCompensation(true);

        masterShooterMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
        masterShooterMotor.setSensorPhase(true);

        masterShooterMotor.configNominalOutputForward(0, 30);
        masterShooterMotor.configNominalOutputForward(0, 30);
        masterShooterMotor.configPeakOutputForward(1, 30);
        masterShooterMotor.configPeakOutputReverse(-1, 30);

        masterShooterMotor.config_kF(0, 0.34, 30);
        masterShooterMotor.config_kP(0, 0.2, 30);
        masterShooterMotor.config_kI(0, 0, 30);
        masterShooterMotor.config_kD(0, 0, 30);

        resetHoodEncoders();
    }

    public void shooterInit() {
        shooting = false;
        dumpShot = false;
        lowDumpShot = false;
        hangarShot = false;
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setDouble(3.0);
    }

    final static float distanceFromTapeToCenterOfHub = 26.6875f;
    final static float distanceFromFenderToCenterOfHub = 33.875f;
    final static float distanceFromFenderToTape = distanceFromFenderToCenterOfHub - distanceFromTapeToCenterOfHub;
    // final static float horizontalDistanceFromLimeLightToShooter = 5.375f;
    // final static float horizontalDistanceFromShooterToFrontOfRobot = 7.25f;
    // final static float horizontalDistanceFromLimelightToFrontOfRobot =
    // horizontalDistanceFromShooterToFrontOfRobot
    // - horizontalDistanceFromLimeLightToShooter;
    final static float horizontalDistanceFromFrontOfRobotToLimelight = 15.0f;
    final static float horizontalDistanceFromFrontOfRobotToShooter = 7.25f;
    final static float horizontalDistanceFromLimelightToShooter = horizontalDistanceFromFrontOfRobotToLimelight
            - horizontalDistanceFromFrontOfRobotToShooter;

    final static float gravity = 386.103f;
    final static float y = 104.0f;
    final static float y0 = 22.5f;
    final static float minimumVelocity = 267.7f;
    final static float maximumVelocity = 405.5f;
    final static float minimumAngle = 0.804f;
    final static float maximumAngle = 1.387f;

    private static float cameraHeight = 40.0f;
    private static float cameraAngle = 45.5f;
    static float angleError = 20.25f;

    // public static float getXDistanceFromCenterOfHub(double verticalAngle) {
    // return (float) (((y - cameraHeight) / Math.tan(Math.toRadians(verticalAngle +
    // cameraAngle - angleError)))
    // + (distanceFromTapeToCenterOfHub + 24.0f) -
    // horizontalDistanceFromLimelightToShooter);
    // }

    public static float getXDistanceToGoal(double verticalAngle) {
        return (float) (((y - cameraHeight) / Math.tan(Math.toRadians(verticalAngle + cameraAngle - angleError)))
                - horizontalDistanceFromLimelightToShooter + distanceFromTapeToCenterOfHub + 18.0f);
    }

    public static float getXDistanceFromFrontOfRobotToFender(double verticalAngle) {
        return (float) (((y - cameraHeight) / Math.tan(Math.toRadians(verticalAngle + cameraAngle - angleError)))
                - distanceFromFenderToTape - horizontalDistanceFromFrontOfRobotToLimelight);
    }

    // public static float getXDistanceFromCenterOfHub(double verticalAngle) { // x
    // // distance from front of robot to fender. better for testing
    // return (float) (((y - cameraHeight) / Math.tan(Math.toRadians(verticalAngle +
    // cameraAngle - angleError))) - horizontalDistanceFromFrontOfRobotToLimelight -
    // distanceFromFenderToTape);
    // }

    static float calculatedVelocity = 0.0f;
    static float calculatedAngle = 0.000f;
    public static float shootingCoefficient = 2.765f;

    static boolean setHoodYet = false;
    static boolean goUp = false;
    static boolean goDown = false;
    static boolean calculationBoolean = false;
    public double dumpShotSpeed = 0.4472;
    public double lowDumpShotSpeed = 0.22;
    public double hangarShotSpeed = 0.50;

    static float tempAngle = 0.000f;
    static float calculatedDistance = 0.000f;

    public static void testVelocity(float x) {

        float velocity = 0.0f;
        for (float i = minimumVelocity; i <= maximumVelocity; i += 1.0f) {
            if ((testAngle(i, x) > minimumAngle) && (testAngle(i, x) < maximumAngle)) {
                velocity = i;
                break;
            }
        }
        calculatedVelocity = velocity;
        calculatedAngle = testAngle(velocity, x);
    }

    public static float testAngle(float velocity, float x) {

        float angle = minimumAngle;

        float[] possibleAngles = new float[(int) ((maximumAngle * 1000.0f) - (minimumAngle * 1000.0f) + 1)];
        for (int i = 0; i < possibleAngles.length; i++) {
            possibleAngles[i] = angle;
            angle = angle + 0.001f;
        }

        for (float possibleAngle : possibleAngles) {
            float slope = slopeOfLine(velocity, possibleAngle, x);
            if (!Double.isNaN(slope)) {
                if (slope > 0.00) {
                    angle = possibleAngle;
                    break;
                }
            }
        }

        while (slopeOfLine(velocity, angle, x) > 0.000f) {
            angle += 0.005f;
        }
        return angle;
    }

    public static float slopeOfLine(float velocity, float possibleAngle, float x) {

        return (float) (((y - x)
                / (((-1.0f) * ((-1.0f) * velocity * Math.sin(possibleAngle) - Math.sqrt(Math.pow(velocity, 2)
                        * Math.pow(Math.sin(possibleAngle), 2) + ((2.0f * gravity) * (y0 - y)))) / gravity)
                        - (x / (velocity * Math.cos(possibleAngle))))));
    }

    // public static float getXDistanceFromCenterOfHub(double verticalAngle) { // x
    // // distance from front of robot to fender. better for testing
    // return (float) (((y - cameraHeight) / Math.tan(Math.toRadians(verticalAngle +
    // cameraAngle - angleError))) - horizontalDistanceFromLimelightToFrontOfRobot -
    // distanceFromFenderToTape);
    // }

    public static float shooterWheelLinearVelocityToMotorVelocity(double projectileVelocity) {
        return (float) ((shootingCoefficient
                * (projectileVelocity * (1.0f / (4.0f * Math.PI)) * (1.0f / 1.21) * 2048.0f * (1.0f / 10.0f))));
    }

    public static float shooterWheelLinearVelocityToMotorPercentOutput(double projectileVelocity) {
        // same method except dividing. could change to return above method / 22068.97f
        return (float) ((shootingCoefficient
                * (projectileVelocity * (1.0f / (4.0f * Math.PI)) * (1.0f / 1.21f) * 2048.0f * (1.0f / 10.0f)))
                / 22068.97f);
    }

    double centeringA = 0.0002;

    public boolean centerRobotOnTarget() {
        double xAngle = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0) - 4.0;

        double ySpeed = Constants.stick.getRawAxis(1);
        double xSpeed = Constants.stick.getRawAxis(0);
        double yDirectionMaintainer = 1.0;
        double xDirectionMaintainer = 1.0;
        if (ySpeed < 0.0) {
            yDirectionMaintainer = -1.0;
        }
        if (xSpeed < 0.0) {
            xDirectionMaintainer = -1.0;
        }
        if ((ySpeed > -0.2) && (ySpeed < 0.2)) {
            ySpeed = 0.00;
        }
        if ((xSpeed > -0.2) && (xSpeed < 0.2)) {
            xSpeed = 0.00;
        }

        if (Constants.stick.getRawButton(4)) {
            Robot.drive.driveSpeedCoefficient = 0.85;
        } else {
            Robot.drive.driveSpeedCoefficient = 0.7;
        }

        if (xAngle < -15.0) {
            Robot.drive.mecanumDrive.driveCartesian(
                    Robot.drive.driveSpeedCoefficient * -ySpeed * ySpeed * yDirectionMaintainer,
                    (Robot.drive.driveSpeedCoefficient + 0.1) * xSpeed * xSpeed * xDirectionMaintainer, -0.2);
            return false;
        } else if (xAngle > 15.0) {
            Robot.drive.mecanumDrive.driveCartesian(
                    Robot.drive.driveSpeedCoefficient * -ySpeed * ySpeed * yDirectionMaintainer,
                    (Robot.drive.driveSpeedCoefficient + 0.1) * xSpeed * xSpeed * xDirectionMaintainer, 0.2);
            return false;
        } else if (xAngle < -4.0) {
            Robot.drive.mecanumDrive.driveCartesian(
                    Robot.drive.driveSpeedCoefficient * -ySpeed * ySpeed * yDirectionMaintainer,
                    (Robot.drive.driveSpeedCoefficient + 0.1) * xSpeed * xSpeed * xDirectionMaintainer, -0.12);
            return false;
        } else if (xAngle > 4.0) {
            Robot.drive.mecanumDrive.driveCartesian(
                    Robot.drive.driveSpeedCoefficient * -ySpeed * ySpeed * yDirectionMaintainer,
                    (Robot.drive.driveSpeedCoefficient + 0.1) * xSpeed * xSpeed * xDirectionMaintainer, 0.12);
            return false;
        } else if (xAngle < -1.0) {
            Robot.drive.mecanumDrive.driveCartesian(
                    Robot.drive.driveSpeedCoefficient * -ySpeed * ySpeed * yDirectionMaintainer,
                    (Robot.drive.driveSpeedCoefficient + 0.1) * xSpeed * xSpeed * xDirectionMaintainer, -0.06);
            return false;
        } else if (xAngle > 1.0) {
            Robot.drive.mecanumDrive.driveCartesian(
                    Robot.drive.driveSpeedCoefficient * -ySpeed * ySpeed * yDirectionMaintainer,
                    (Robot.drive.driveSpeedCoefficient + 0.1) * xSpeed * xSpeed * xDirectionMaintainer, 0.06);
            return false;
        } else {
            Robot.drive.mecanumDrive.driveCartesian(
                    Robot.drive.driveSpeedCoefficient * -ySpeed * ySpeed * yDirectionMaintainer,
                    (Robot.drive.driveSpeedCoefficient + 0.1) * xSpeed * xSpeed * xDirectionMaintainer, 0.0);
            return true;
        }
    }

    public boolean distanceRobotFromTargetCorrectly() {
        double yAngle = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);
        double distance = getXDistanceFromFrontOfRobotToFender(yAngle);

        if ((distance < 33.0) || (distance > 300.0)) {
            Robot.drive.mecanumDrive.driveCartesian(0.175, 0.0, 0.0);
            return false;
        } else if (distance > 55.0) {
            Robot.drive.mecanumDrive.driveCartesian(-0.175, 0.0, 0.0);
            return false;
        } else {
            Robot.drive.mecanumDrive.driveCartesian(0.0, 0.0, 0.0);
            return true;
        }
    }

    public void shooterIdle(double shooterSpeed) { // does this need to be in the shooter methods or just in
                                                   // teleopPeriodic
        if (!shooting && !dumpShot && !lowDumpShot && !hangarShot && Constants.xbox.getPOV() != 0 && Constants.xbox.getPOV() != 180) {
            masterShooterMotor.set(shooterSpeed);
            if (!DriverStation.isTeleop()) {
                Robot.intake.indexerMotor.set(0.0);
            }
        }
    }

    Timer shootingTimer = new Timer();

    public void shootInit() {
        setHoodYet = false;
        tempAngle = 0.000f;
        goUp = false;
        goDown = false;
        calculationBoolean = false;
    }

    public void shoot() {

        if (Constants.xbox.getRawButtonPressed(5)) {
            if (!dumpShot && !lowDumpShot && !hangarShot) {
                shooting = !shooting;
                if (shooting) {
                    shootInit();
                }
            }
        }

        if (shooting && !dumpShot && !lowDumpShot && !hangarShot) {

            if (!calculationBoolean) {
                testVelocity(getXDistanceToGoal(
                        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0)));
                calculationBoolean = true;
                shootingTimer.reset();
                shootingTimer.start();
            }

            if (Math.abs(calculatedAngle - tempAngle) > 0.004) {

                if (DriverStation.isTeleop()) {
                    if (shootingTimer.get() > 10.0) {
                        shooting = false;
                    }
                }

                // masterShooterMotor.set(ControlMode.PercentOutput,
                // (shooterWheelLinearVelocityToMotorPercentOutput(calculatedVelocity)));
                masterShooterMotor.set(ControlMode.Velocity, calculatedVelocity * shootingCoefficient);

                if ((hoodMotor.getEncoder().getPosition() < (90.0 - Math.toDegrees(calculatedAngle)))
                        && (goDown == false)) {
                    goUp = true;
                }

                if (goUp) {
                    hoodMotor.set(0.2);
                    if ((hoodMotor.getEncoder().getPosition() > (90.0 - Math.toDegrees(calculatedAngle)))) {
                        hoodMotor.set(0.0);
                        hoodMotor.getPIDController().setReference(hoodMotor.getEncoder().getPosition(),
                                ControlType.kPosition);
                        setHoodYet = true;
                    }
                }

                if ((hoodMotor.getEncoder().getPosition() > (90.0 - Math.toDegrees(calculatedAngle)))
                        && (goUp == false)) {
                    goDown = true;
                }

                if (goDown) {
                    hoodMotor.set(-0.2);
                    if ((hoodMotor.getEncoder().getPosition() < (90.0 - Math.toDegrees(calculatedAngle)))) {
                        hoodMotor.set(0.0);
                        hoodMotor.getPIDController().setReference(hoodMotor.getEncoder().getPosition(),
                                ControlType.kPosition);
                        setHoodYet = true;
                    }
                }

                double distance = getXDistanceToGoal(
                        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0));
                if (distance > 200.0) {
                    shootingCoefficient = 2.825f;
                } else {
                    shootingCoefficient = 2.765f;
                }
                double thresholdVelocity = calculatedVelocity;
                System.out.println("calculated: " + thresholdVelocity);
                System.out.println("real:" + masterShooterMotor.getSelectedSensorVelocity(1));
                System.out.println("distance: " + distance);
                if ((masterShooterMotor.getSelectedSensorVelocity(1) > thresholdVelocity) && setHoodYet) {
                    if (DriverStation.isTeleop()) {
                        if (Constants.xbox.getRawAxis(3) > 0.8) {
                            Robot.intake.indexerMotor.set(-0.8);
                        } else {
                            Robot.intake.indexerMotor.set(0.0);
                        }
                    } else {
                        Robot.intake.indexerMotor.set(-0.8);
                    }
                } else {
                    Robot.intake.indexerMotor.set(0.0);
                }
            }
        }
    }

    public void dumpShotLEDs() {
        double distance = getXDistanceFromFrontOfRobotToFender(
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0));
        System.out.println(distance);

        if ((distance > 33.0) && (distance < 55.0)) {
            SmartDashboard.putBoolean("Dumpshot Distance", true);
            LEDsSparkMax.set(0.15);
        } else {
            SmartDashboard.putBoolean("Dumpshot Distance", false);
            LEDsSparkMax.set(0.57);
        }
    }

    public void dumpShot() { // 92 inches according to distance method
        // hood all the way down. there's code to put the hood down that could take away
        // later if we locked hood in place

        dumpShotLEDs();

        if (Constants.xbox.getRawButtonPressed(4)) {
            if (!shooting && !lowDumpShot && !hangarShot) {
                dumpShot = !dumpShot;
                if (dumpShot) {
                    shootInit();
                    shootingTimer.reset();
                    shootingTimer.start();
                }
            }
        }

        if (dumpShot && !shooting && !lowDumpShot && !hangarShot) {

            if (shootingTimer.get() > 10.0) {
                dumpShot = false;
            }

            // masterShooterMotor.set(ControlMode.PercentOutput, dumpShotSpeed);
            masterShooterMotor.set(ControlMode.Velocity, dumpShotSpeed * 1900.0);

            if ((hoodMotor.getEncoder().getPosition() > 11.0)) { // only goes down to bottom angle of 10.5 degrees if
                                                                 // it's above.
                goDown = true;
            } else {
                setHoodYet = true;
            }

            if (goDown) {
                hoodMotor.set(-0.2);
                if ((hoodMotor.getEncoder().getPosition() < 11.0)) {
                    hoodMotor.set(0.0);
                    hoodMotor.getPIDController().setReference(hoodMotor.getEncoder().getPosition(), ControlType.kPosition);
                    setHoodYet = true;
                }
            }

            double thresholdVelocity = 9200.0;
            System.out.println("calculated: " + thresholdVelocity);
            System.out.println("real:" + masterShooterMotor.getSelectedSensorVelocity(1));
            if ((masterShooterMotor.getSelectedSensorVelocity(1) > thresholdVelocity) && setHoodYet) {
                if (DriverStation.isTeleop()) {
                    if (Constants.xbox.getRawAxis(3) > 0.8) {
                        Robot.intake.indexerMotor.set(-0.8);
                    } else {
                        Robot.intake.indexerMotor.set(0.0);
                    }
                } else {
                    Robot.intake.indexerMotor.set(-0.8);
                }
            } else {
                Robot.intake.indexerMotor.set(0.0);
            }
        }
    }

    public void lowDumpShot() {
        if (Constants.xbox.getRawButtonPressed(1)) {
            if (!shooting && !dumpShot && !hangarShot) {
                lowDumpShot = !lowDumpShot;
                if (lowDumpShot) {
                    shootInit();
                    shootingTimer.reset();
                    shootingTimer.start();
                }
            }
        }

        if (lowDumpShot && !shooting && !dumpShot && !hangarShot) {

            if (shootingTimer.get() > 10.0) {
                lowDumpShot = false;
            }

            // masterShooterMotor.set(ControlMode.PercentOutput, dumpShotSpeed);
            masterShooterMotor.set(ControlMode.Velocity, lowDumpShotSpeed * 1900.0);

            if ((hoodMotor.getEncoder().getPosition() > 13.0)) { // only goes down to bottom angle of 10.5 degrees if
                                                                 // it's above.
                goDown = true;
            } else {
                setHoodYet = true;
            }

            if (goDown) {
                hoodMotor.set(-0.1);
                if ((hoodMotor.getEncoder().getPosition() < 13.0)) {
                    hoodMotor.set(0.0);
                    setHoodYet = true;
                }
            }

            double thresholdVelocity = 4000.0;
            System.out.println("calculated: " + thresholdVelocity);
            System.out.println("real:" + masterShooterMotor.getSelectedSensorVelocity(1));
            if ((masterShooterMotor.getSelectedSensorVelocity(1) > thresholdVelocity) && setHoodYet) {
                if (DriverStation.isTeleop()) {
                    if (Constants.xbox.getRawAxis(3) > 0.8) {
                        Robot.intake.indexerMotor.set(-0.8);
                    } else {
                        Robot.intake.indexerMotor.set(0.0);
                    }
                } else {
                    Robot.intake.indexerMotor.set(-0.8);
                }
            } else {
                Robot.intake.indexerMotor.set(0.0);
            }
        }
    }

    public void hangarShot() {
        if (Constants.xbox.getRawButtonPressed(3)) {
            if (!shooting && !dumpShot && !lowDumpShot) {
                hangarShot = !hangarShot;
                if (hangarShot) {
                    shootInit();
                    shootingTimer.reset();
                    shootingTimer.start();
                }
            }
        }

        if (hangarShot && !shooting && !dumpShot && !lowDumpShot) {

            if (shootingTimer.get() > 10.0) {
                hangarShot = false;
            }

            masterShooterMotor.set(ControlMode.Velocity, hangarShotSpeed * 1900.0);

            if ((hoodMotor.getEncoder().getPosition() < 34.5)
                    && (goDown == false)) {
                goUp = true;
            }

            if (goUp) {
                hoodMotor.set(0.2);
                if ((hoodMotor.getEncoder().getPosition() > 34.5)) {
                    hoodMotor.set(0.0);
                    hoodMotor.getPIDController().setReference(hoodMotor.getEncoder().getPosition(),
                            ControlType.kPosition);
                    setHoodYet = true;
                }
            }

            if ((hoodMotor.getEncoder().getPosition() > 34.5)
                    && (goUp == false)) {
                goDown = true;
            }

            if (goDown) {
                hoodMotor.set(-0.2);
                if ((hoodMotor.getEncoder().getPosition() < 34.5)) {
                    hoodMotor.set(0.0);
                    hoodMotor.getPIDController().setReference(hoodMotor.getEncoder().getPosition(),
                            ControlType.kPosition);
                    setHoodYet = true;
                }
            }

            double thresholdVelocity = 9200.0 * (hangarShotSpeed / dumpShotSpeed);
            System.out.println("calculated: " + thresholdVelocity);
            System.out.println("real:" + masterShooterMotor.getSelectedSensorVelocity(1));
            if ((masterShooterMotor.getSelectedSensorVelocity(1) > thresholdVelocity) && setHoodYet) {
                if (Constants.xbox.getRawAxis(3) > 0.8) {
                    Robot.intake.indexerMotor.set(-0.8);
                } else {
                    Robot.intake.indexerMotor.set(0.0);
                }
            } else {
                Robot.intake.indexerMotor.set(0.0);
            }
        }
    }

    public void hoodControl() {
        if (!shooting && !dumpShot && !lowDumpShot && !hangarShot) {
            if (Constants.xbox.getPOV() == 180) {
                hoodMotor.set(-0.1);
            } else if (Constants.xbox.getPOV() == 0) {
                hoodMotor.set(0.1);
            } else {
                hoodMotor.set(0.0);
                hoodMotor.getPIDController().setReference(hoodMotor.getEncoder().getPosition(), ControlType.kPosition);
            }
        }
    }

    public void moveHood(double degree) {

        if ((hoodMotor.getEncoder().getPosition() < degree) && (goDown == false)) {
            goUp = true;
        }

        if (goUp) {
            hoodMotor.set(0.1);
            if ((hoodMotor.getEncoder().getPosition() > degree)) {
                hoodMotor.set(0.0);
                hoodMotor.getPIDController().setReference(hoodMotor.getEncoder().getPosition(),
                        ControlType.kPosition);
                setHoodYet = true;
            }
        }

        if ((hoodMotor.getEncoder().getPosition() > degree) && (goUp == false)) {
            goDown = true;
        }

        if (goDown) {
            hoodMotor.set(-0.1);
            if ((hoodMotor.getEncoder().getPosition() < degree)) {
                hoodMotor.set(0.0);
                hoodMotor.getPIDController().setReference(hoodMotor.getEncoder().getPosition(),
                        ControlType.kPosition);
                setHoodYet = true;
            }
        }
    }

}