package frc.robot.Subsystems.Drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

public class ModuleIOSim implements ModuleIO {
  private SwerveModuleSimulation module;

  private SimulatedMotorController.GenericMotorController driveMotor;
  private int moduleNum;

  private SimulatedMotorController.GenericMotorController turnMotor;

  public ModuleIOSim(SwerveModuleSimulation module, int moduleNum) {
    this.module = module;

    this.moduleNum = moduleNum;
    driveMotor =
        module.useGenericMotorControllerForDrive().withCurrentLimit(Current.ofBaseUnits(40, Amps));

    turnMotor =
        module.useGenericControllerForSteer().withCurrentLimit(Current.ofBaseUnits(60, Amps));
  }

  @Override
  public void setDriveVoltage(double voltage) {
    driveMotor.requestVoltage(Voltage.ofBaseUnits(voltage, Volts));
  }

  @Override
  public void setTurnVoltage(double voltage) {
    turnMotor.requestVoltage(Voltage.ofBaseUnits(voltage, Volts));
  }

  @Override
  public void setDriveSpeed(double speed) {
    driveMotor.requestVoltage(
        module
            .getDriveMotorConfigs()
            .calculateVoltage(
                module.getDriveMotorSupplyCurrent(),
                RadiansPerSecond.of(
                    speed
                        * SwerveConstants.DRIVETRAIN_MAX_SPEED
                        * SwerveConstants.WHEEL_DIAMETER
                        / 2)));
  }

  @Override
  public void setTurnSpeed(double speed) {
    turnMotor.requestVoltage(
        module
            .getSteerMotorConfigs()
            .calculateVoltage(
                module.getSteerMotorSupplyCurrent(),
                RadiansPerSecond.of(
                    speed
                        * SwerveConstants.DRIVETRAIN_MAX_ANGULAR_SPEED
                        * SwerveConstants.WHEEL_DIAMETER
                        / 2)));
  }

  @Override
  public void resetDriveEncoder() {
    // Figure out later
  }

  @Override
  public void resetTurnEncoder() {
    // Figure out later
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.driveVoltage = module.getDriveMotorAppliedVoltage().abs(Volts);
    inputs.turnVoltage = module.getSteerMotorAppliedVoltage().abs(Volts);

    inputs.driveCurrent = module.getDriveMotorSupplyCurrent().abs(Amps);
    inputs.turnCurrent = module.getSteerMotorSupplyCurrent().abs(Amps);

    inputs.absEncoderValue = module.getSteerAbsoluteAngle().abs(Degrees);
    inputs.absEncoderConnected = true;

    inputs.driveEncoderValue =
        module.getDriveEncoderUnGearedPosition().abs(Radians)
            * SwerveConstants.DRIVE_MOTOR_PCONVERSION;
    inputs.turnEncoderValue =
        module.getSteerRelativeEncoderPosition().abs(Radians)
            * SwerveConstants.TURN_MOTOR_PCONVERSION;

    inputs.driveVelocity =
        module.getDriveEncoderUnGearedSpeed().abs(RadiansPerSecond)
            * SwerveConstants.DRIVE_MOTOR_VCONVERSION;
    inputs.turnVelocity =
        module.getSteerRelativeEncoderVelocity().abs(RadiansPerSecond)
            * SwerveConstants.TURN_MOTOR_VCONVERSION;

    inputs.moduleNumebr = moduleNum;
  }
}
