package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;

public class Climber {

  Solenoid level2ClimberUp;
  Solenoid level2ClimberDown;

  Solenoid traverseClimberActivate;
  Solenoid traverseClimberDeactivate;

  CANSparkMax traverseClimberMotor = new CANSparkMax(8, MotorType.kBrushless);

  boolean isLevel2ClimberDown = true;
  boolean isTraverseClimberActivated = false;

  public void climberInit() {
    level2ClimberUp = Robot.ph.makeSolenoid(13);
    level2ClimberDown = Robot.ph.makeSolenoid(11);
    traverseClimberActivate = Robot.ph.makeSolenoid(8);
    traverseClimberDeactivate = Robot.ph.makeSolenoid(9);
  }

  public void putLevel2ClimberDown() {
    level2ClimberUp.set(false);
    level2ClimberDown.set(true);
    isLevel2ClimberDown = true;
  }

  public void putLevel2ClimberUp() {
    level2ClimberDown.set(false);
    level2ClimberUp.set(true);
    isLevel2ClimberDown = false;
  }

  public void activateTraverseClimber() {
    traverseClimberDeactivate.set(false);
    traverseClimberActivate.set(true);
    isTraverseClimberActivated = true;
  }

  public void deactivateTraverseClimber() {
    traverseClimberActivate.set(false);
    traverseClimberDeactivate.set(true);
    isTraverseClimberActivated = false;
  }

  public void climberTeleop() {

    if (Constants.xbox.getRawButtonPressed(7)) {
      if (isLevel2ClimberDown) {
        putLevel2ClimberUp();
      } else {
        putLevel2ClimberDown();
      }
    }

    if(Constants.xbox.getRawButtonPressed(8)){
      if(isTraverseClimberActivated){
        deactivateTraverseClimber();
      } else {
        activateTraverseClimber();
      }
    }

    double traverseClimberArmSpeed = Constants.xbox.getRawAxis(5);
    boolean isTraverseSpeedNegative = false;

    if(traverseClimberArmSpeed < 0.0){
      isTraverseSpeedNegative = true;
    }
    if((traverseClimberArmSpeed > -0.2) && (traverseClimberArmSpeed < 0.2)){
      traverseClimberArmSpeed = 0.0;
    }

    if(isTraverseSpeedNegative){
      traverseClimberMotor.set(-1.0 * traverseClimberArmSpeed*traverseClimberArmSpeed);
    } else {
      traverseClimberMotor.set(traverseClimberArmSpeed*traverseClimberArmSpeed);
    }
    
  }

}