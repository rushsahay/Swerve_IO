package frc.robot.subsystems.GroundIntake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.drive.DriveConstants;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public class GroundIntakeIOSim implements GroundIntakeIO {

    private final DCMotor armMotor = DCMotor.getNEO(1);
    private final double gearing = 100; // 364;//125;//375;//75
    private final Distance armLength = Inches.of(20.544446); // 24.719);
    private final Mass armWeight = Pounds.of(3); // 8.23);//11
    private final Angle minAngle = Rotations.of(GroundIntakeConstants.ControlConstants.visualOffsetRotations);
    private final Angle maxAngle = Rotations.of(0.3);
    private final Angle startingAngle = Rotations.of(GroundIntakeConstants.ControlConstants.visualOffsetRotations);
    private final SingleJointedArmSim sim = new SingleJointedArmSim(
            armMotor,
            gearing,
            SingleJointedArmSim.estimateMOI(armLength.in(Meters), armWeight.in(Kilograms)),
            armLength.in(Meters),
            minAngle.in(Radians), // minAngle.in(Radians)
            maxAngle.in(Radians), // maxAngle.in(Radians)
            true,
            startingAngle.in(Radians));
    private MutVoltage appliedVolts = new MutVoltage(0, 0, Volts);
    private AbstractDriveTrainSimulation driveTrain;
    private final IntakeSimulation intakeSim;
    private final double metersIntakeInsideBot = 0.3;

    public GroundIntakeIOSim(AbstractDriveTrainSimulation driveTrain) {
        this.driveTrain = driveTrain;
        intakeSim = IntakeSimulation.OverTheBumperIntake(
                "Algae",
                driveTrain,
                Meters.of(DriveConstants.trackWidth),
                Meters.of(armLength.in(Meters)-metersIntakeInsideBot),
                IntakeSide.FRONT,
                1);
    }

    @Override
    public void updateInputs(GroundIntakeIOInputs inputs) {
        sim.update(0.02);
        inputs.encoderPosition = Rotations.convertFrom(sim.getAngleRads(), Radians)
                + GroundIntakeConstants.ControlConstants.visualOffsetRotations;
        inputs.algaeDetected = intakeSim.getGamePiecesAmount() != 0;
        inputs.rollerVoltage = 0;
        inputs.rollerCurrent = 0;
        inputs.pivotVoltage = appliedVolts.in(Volts);
        inputs.pivotCurrent = sim.getCurrentDrawAmps();
        inputs.rollerSpeed = 0.0;
        inputs.encoderConnected = true;
    }

    @Override
    public void setPivotVoltage(double voltage) {
        double clampedEffort = -MathUtil.clamp(voltage, -12, 12);
        appliedVolts.mut_replace(clampedEffort, Volts);
        sim.setInputVoltage(clampedEffort);
    }

    @Override
    public void setRollerSpeed(double speed) {
        if (speed != 0) {
            intakeSim.startIntake();
        } else {
            intakeSim.stopIntake();
            if (Rotations.convertFrom(sim.getAngleRads(), Radians)
                            + GroundIntakeConstants.ControlConstants.visualOffsetRotations
                    < GroundIntakeConstants.ControlConstants.algaeHoldPosition) {
                intakeSim.setGamePiecesCount(0);
            }
        }
    }
}
