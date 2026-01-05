// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.Arm.ArmPositions;
import frc.robot.subsystems.Arm.ArmIO;
import frc.robot.subsystems.Arm.ArmIOSim;
import frc.robot.subsystems.Arm.ArmIOSparkMax;
import frc.robot.subsystems.Arm.ArmVisualizer;
import frc.robot.subsystems.GroundIntake.GroundIntake;
import frc.robot.subsystems.GroundIntake.GroundIntake.GroundIntakeStates;
import frc.robot.subsystems.GroundIntake.GroundIntakeIO;
import frc.robot.subsystems.GroundIntake.GroundIntakeIOSim;
import frc.robot.subsystems.GroundIntake.GroundIntakeIOSparkMax;
import frc.robot.subsystems.GroundIntake.GroundIntakeVisualizer;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.*;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Drive drive;
    private final Vision vision;
    private final Arm arm;
    private ArmVisualizer armVisualizer = null;
    public static GroundIntake groundIntake;
    private GroundIntakeVisualizer groundIntakeVisualizer = null;
    private SwerveDriveSimulation driveSimulation = null;

    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);

    public Trigger resetHeadingTrigger = new Trigger(() -> controller.start().getAsBoolean());
    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOSpark(0),
                        new ModuleIOSpark(1),
                        new ModuleIOSpark(2),
                        new ModuleIOSpark(3),
                        (pose) -> {});

                this.vision = new Vision(
                        drive,
                        new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
                        new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation));

                this.arm = new Arm(new ArmIOSparkMax());
                this.groundIntake = new GroundIntake(new GroundIntakeIOSparkMax());
                break;
            case SIM:
                // create a maple-sim swerve drive simulation instance
                this.driveSimulation =
                        new SwerveDriveSimulation(DriveConstants.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
                // add the simulated drivetrain to the simulation field
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                // Sim robot, instantiate physics sim IO implementations
                drive = new Drive(
                        new GyroIOSim(driveSimulation.getGyroSimulation()),
                        new ModuleIOSim(driveSimulation.getModules()[0]),
                        new ModuleIOSim(driveSimulation.getModules()[1]),
                        new ModuleIOSim(driveSimulation.getModules()[2]),
                        new ModuleIOSim(driveSimulation.getModules()[3]),
                        driveSimulation::setSimulationWorldPose);

                vision = new Vision(
                        drive,
                        new VisionIOPhotonVisionSim(
                                camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
                        new VisionIOPhotonVisionSim(
                                camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose));

                arm = new Arm(new ArmIOSim());
                armVisualizer = new ArmVisualizer();

                groundIntake = new GroundIntake(new GroundIntakeIOSim());
                groundIntakeVisualizer = new GroundIntakeVisualizer();
                break;
            default:
                // Replayed robot, disable IO implementations
                drive = new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        (pose) -> {});
                vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});
                arm = new Arm(new ArmIO() {});
                groundIntake = new GroundIntake(new GroundIntakeIO() {});
                break;
        }

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(DriveCommands.joystickDrive(
                drive, () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> -controller.getRightX()));

        // Lock to 0° when A button is held
        // controller
        //         .a()
        //         .whileTrue(DriveCommands.joystickDriveAtAngle(
        //                 drive, () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> new Rotation2d()));

        controller.x().onTrue(new InstantCommand(() -> arm.setWantedPosition(ArmPositions.A2), arm));
        controller.a().onTrue(new InstantCommand(() -> arm.setWantedPosition(ArmPositions.Idle), arm));

        controller
                .y()
                .onTrue(new InstantCommand(() -> groundIntake.setWantedState(GroundIntakeStates.Hold), groundIntake));
        controller
                .b()
                .onTrue(new InstantCommand(() -> groundIntake.setWantedState(GroundIntakeStates.Rest), groundIntake));
        // Switch to X pattern when X button is pressed
        controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

        // Reset gyro / odometry
        // Look into implementing instead
        final Runnable resetGyro = Constants.currentMode == Constants.Mode.SIM
                ? () -> drive.resetOdometry(
                        driveSimulation
                                .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during simulation
                : () -> drive.resetOdometry(
                        new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro
        controller.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

        // Example Coral Placement Code
        // // TODO: delete these code for your own project Godlie scoring L4s HAHA - Rush
        // if (Constants.currentMode == Constants.Mode.SIM) {
        //     // L4 placement
        //     controller.y().onTrue(Commands.runOnce(() -> SimulatedArena.getInstance()
        //             .addGamePieceProjectile(new ReefscapeCoralOnFly(
        //                     driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
        //                     new Translation2d(0.4, 0),
        //                     driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
        //                     driveSimulation.getSimulatedDriveTrainPose().getRotation(),
        //                     Meters.of(2),
        //                     MetersPerSecond.of(1.5),
        //                     Degrees.of(-80)))));
        //     // L3 placement
        //     controller.b().onTrue(Commands.runOnce(() -> SimulatedArena.getInstance()
        //             .addGamePieceProjectile(new ReefscapeCoralOnFly(
        //                     driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
        //                     new Translation2d(0.4, 0),
        //                     driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
        //                     driveSimulation.getSimulatedDriveTrainPose().getRotation(),
        //                     Meters.of(1.35),
        //                     MetersPerSecond.of(1.5),
        //                     Degrees.of(-60)))));
        // }
        resetHeadingTrigger.onTrue(new InstantCommand(() -> {
            Pose2d currentPose = drive.getPose();
            Pose2d resetPose = new Pose2d(
                    new Translation2d(currentPose.getX(), currentPose.getY()),
                    (DriverStation.getAlliance().get() == Alliance.Red) ? Rotation2d.k180deg : Rotation2d.kZero);
            drive.resetGyro(resetPose);
        }));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public void resetSimulationField() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        drive.resetOdometry(new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void updateSimulation() {
        if (Constants.currentMode != Constants.Mode.SIM) return;
        SimulatedArena.getInstance().simulationPeriodic();
        armVisualizer.updateDistance(arm.getEncoderPosition());
        groundIntakeVisualizer.updateAngle(groundIntake.getPosition());
        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput(
                "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        Logger.recordOutput(
                "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    }
}
