package frc.robot.subsystems.GroundIntake;

import static edu.wpi.first.units.Units.*;

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

public class GroundIntakeVisualizer {

    LoggedMechanism2d groundIntakePanel;
    LoggedMechanismRoot2d groundIntakeRoot;
    LoggedMechanismLigament2d groundIntakeLigament;

    public GroundIntakeVisualizer() {
        groundIntakePanel = new LoggedMechanism2d(100, 100);
        groundIntakeRoot = groundIntakePanel.getRoot("Ground Intake Root", 16, 16);
        groundIntakeLigament = groundIntakeRoot.append(new LoggedMechanismLigament2d(
                "Ground Intake Mech", Meters.convertFrom(16, Inches), 0, 10, new Color8Bit(Color.kBlue)));
        Logger.recordOutput("Ground Intake/Mechanism2d/", groundIntakePanel);
    }

    public void updateAngle(double angleRotations) {
        groundIntakeLigament.setAngle(Rotation2d.fromRadians(angleRotations));
        Logger.recordOutput("Ground Intake/Mechanism2d/", groundIntakePanel);
        Pose3d groundIntakeMechPose = new Pose3d();
        Pose3d groundIntakePose = rotateAround(
                groundIntakeMechPose,
                new Translation3d(0.25, -0.35, 0.),
                new Rotation3d(0.0, Radians.convertFrom(angleRotations, Rotations), 0.0));
        Logger.recordOutput("Ground Intake/Mechanism3d/", groundIntakePose);
    }

    public Pose3d rotateAround(Pose3d currentPose, Translation3d point, Rotation3d rot) {
        return new Pose3d(
                currentPose.getTranslation().rotateAround(point, rot),
                currentPose.getRotation().rotateBy(rot));
    }
}
