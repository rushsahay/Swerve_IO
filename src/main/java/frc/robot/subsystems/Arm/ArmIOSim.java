package frc.robot.subsystems.Arm;

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

public class ArmIOSim implements ArmIO {

    private final DCMotor armMotor = DCMotor.getNEO(1);
    private final double gearing = 100; // 364;//125;//375;//75
    private final Distance armLength = Inches.of(20.544446); // 24.719);
    private final Mass armWeight = Pounds.of(9); // 8.23);//11
    private final Angle minAngle =
            Radians.of(Radians.convertFrom(-ArmConstants.ControlConstants.visualOffset.magnitude(), Rotations));
    private final Angle maxAngle = Radians.of(3 * Math.PI / 2);
    private final Angle startingAngle =
            Radians.of(Radians.convertFrom(-ArmConstants.ControlConstants.visualOffset.magnitude(), Rotations));
    private final SingleJointedArmSim sim = new SingleJointedArmSim(
            armMotor,
            gearing,
            SingleJointedArmSim.estimateMOI(armLength.in(Meters), armWeight.in(Kilograms)),
            armLength.in(Meters),
            minAngle.in(Radians),
            maxAngle.in(Radians),
            true,
            startingAngle.in(Radians));
    private MutVoltage appliedVolts = new MutVoltage(0, 0, Volts);

    public ArmIOSim() {}

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        sim.update(0.02);
        inputs.voltage = appliedVolts.in(Volts);
        inputs.current = sim.getCurrentDrawAmps();
        inputs.encoderValue = Rotations.convertFrom(sim.getAngleRads(), Radians)
                + ArmConstants.ControlConstants.visualOffset
                        .magnitude(); // Sync this value with what's written in the visualizer so gravity
        // works
        inputs.velocity = sim.getVelocityRadPerSec();
    }

    @Override
    public void setVoltage(double voltage) {
        double clampedEffort = MathUtil.clamp(voltage, -12, 12);
        appliedVolts.mut_replace(clampedEffort, Volts);
        sim.setInputVoltage(clampedEffort);
    }
}
