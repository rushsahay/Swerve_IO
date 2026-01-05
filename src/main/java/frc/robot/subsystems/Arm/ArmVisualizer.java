package frc.robot.subsystems.Arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ArmVisualizer {
    private LoggedMechanism2d panel;
    private LoggedMechanismRoot2d root;
    private LoggedMechanismLigament2d arm;

    public ArmVisualizer() {
        this.panel =
                new LoggedMechanism2d(Inches.of(100).in(Meters), Inches.of(100).in(Meters));
        this.root = panel.getRoot("Arm", Inches.of(7).in(Meters), Inches.of(7).in(Meters));
        this.arm = root.append(new LoggedMechanismLigament2d(
                "Arm",
                Inches.of(7.365).in(Meters),
                Degrees.convertFrom(0.24, Rotations),
                10,
                new Color8Bit(Color.kGreen)));

        Logger.recordOutput("Arm/Mechanism 2D/", panel);
    }

    public void updateDistance(double angle) {
        double newAngle = angle - ArmConstants.ControlConstants.visualOffset.magnitude();
        arm.setAngle(Rotation2d.fromRotations(newAngle));
        Logger.recordOutput("Arm/Mechanism 2D/", panel);
        Pose3d armPose = new Pose3d();
        armPose = rotateAround(
                armPose,
                new Translation3d(-0.2, 1, .77),
                new Rotation3d(
                        0,
                        Radians.convertFrom(newAngle, Rotations),
                        0)); // new Translation3d(-0.2,1,.77)//new Translation3d(-0.18,0.3,.76)
        Logger.recordOutput("Arm/Mechanism 3D/", armPose);
    }

    public Pose3d rotateAround(Pose3d currentPose, Translation3d point, Rotation3d rot) {
        return new Pose3d(
                currentPose.getTranslation().rotateAround(point, rot),
                currentPose.getRotation().rotateBy(rot));
    }
}
