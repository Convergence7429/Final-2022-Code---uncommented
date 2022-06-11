package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake {

    CANSparkMax intakeMotor = new CANSparkMax(Constants.intakeMotorIndex, MotorType.kBrushed);
    CANSparkMax indexerMotor = new CANSparkMax(Constants.indexerMotorIndex, MotorType.kBrushless);

    Solenoid intakeUpSolenoid;
    Solenoid intakeDownSolenoid;

    static boolean isIntakeDown = false;

    I2C.Port i2cPort = I2C.Port.kOnboard;
    ColorSensorV3 indexerColorSensor = new ColorSensorV3(i2cPort);

    static boolean intaking = false;

    static boolean intakingByColor = true;

    public void intakeInit() {
        intaking = false;

        intakeMotor.enableVoltageCompensation(12.5);
        indexerMotor.enableVoltageCompensation(12.5);

        intakeMotor.setSmartCurrentLimit(35, 40);

        intakeUpSolenoid = Robot.ph.makeSolenoid(12);
        intakeDownSolenoid = Robot.ph.makeSolenoid(10);
    }

    static boolean ballPresent = false;
    static Timer indexingTimer = new Timer();

    public void indexByColor() {
        Color detectedColor = indexerColorSensor.getColor();
        // System.out.println("red: " + detectedColor.red + ", blue: " + detectedColor.blue + ", green: " + detectedColor.green);

        if (!Robot.shooter.shooting && !Robot.shooter.dumpShot && !Robot.shooter.lowDumpShot && !Robot.shooter.hangarShot) {
            if(((detectedColor.red > 0.30) && (detectedColor.red < 0.40)) || ((detectedColor.blue > 0.30) && (detectedColor.blue < 0.50))){
                if(!ballPresent){
                    ballPresent = true;
                    indexingTimer.reset();
                    indexingTimer.start();
                }
            } else {
                indexerMotor.set(-0.2);
            }

            if(ballPresent && (indexingTimer.get() > 0.65)){
                indexerMotor.set(0.0);
            }
        }
    }

    public void intakeUp(){
        intakeDownSolenoid.set(false);
        intakeUpSolenoid.set(true);
        isIntakeDown = false;
    }

    public void intakeDown(){
        intakeUpSolenoid.set(false);
        intakeDownSolenoid.set(true);
        isIntakeDown = true;
    }

    public void intakeTeleop() {

        if(Constants.xbox.getRawButtonPressed(9)){
            intakingByColor = false;
        }

        if(Constants.xbox.getRawAxis(3) > 0.8){
            ballPresent = false;
        }

        SmartDashboard.putBoolean("Intake", intaking);

        if (isIntakeDown) {
            if (Constants.stick.getRawButton(10)) {
                intakeMotor.set(0.6);
                intaking = false;
            } else if (Constants.stick.getRawButtonPressed(1)) {
                intaking = !intaking;
                if (intaking) {
                    intakeMotor.set(-0.8);
                }
            } else {
                if (!intaking) {
                    intakeMotor.set(0.0);
                }
            }
        } else {
            intakeMotor.set(0.0);
        }

        if (!Robot.shooter.shooting && !Robot.shooter.dumpShot && !Robot.shooter.lowDumpShot && !Robot.shooter.hangarShot) {
            if (Constants.xbox.getPOV() == 90) {
                indexerMotor.set(-0.3);
            } else if (Constants.xbox.getPOV() == 270) {
                indexerMotor.set(0.3);
            } else {
                if(intakingByColor){
                    indexByColor();
                } else {
                    indexerMotor.set(0.0);
                }
            }
        }

        if(Constants.xbox.getRawButtonPressed(2)){
            if(isIntakeDown){
                intakeUp();
            } else {
                intakeDown();
            }
        }
    }
    
}