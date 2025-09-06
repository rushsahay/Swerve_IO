// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.SwerveDrive;
import frc.robot.Subsystems.Drive.Drivetrain;
import frc.robot.Subsystems.Drive.GyroPidgeonIO;
import frc.robot.Subsystems.Drive.ModuleIOSparkMax;

public class RobotContainer {
  public static Drivetrain m_Drivetrain;
  public static final CommandXboxController driverController =
      new CommandXboxController(0);
  public final JoystickButton resetHeading_Start =
      new JoystickButton(driverController.getHID(), XboxController.Button.kStart.value);
  // public final SendableChooser<Command> autoChooser;
  public RobotContainer() {
    m_Drivetrain = new Drivetrain(new GyroPidgeonIO(), new ModuleIOSparkMax(1), new ModuleIOSparkMax(2), new ModuleIOSparkMax(3), new ModuleIOSparkMax(4));
    m_Drivetrain.setDefaultCommand(new SwerveDrive());
    registerNamedCommands();
    configureBindings();
    
    // autoChooser = AutoBuilder.buildAutoChooser();
  }

  private void configureBindings() {
    resetHeading_Start.onTrue(new InstantCommand(m_Drivetrain::zeroHeading, m_Drivetrain));
  }

  public Command getAutonomousCommand() {
    return null;
    // return autoChooser.getSelected();
  }
  public void registerNamedCommands() {}
}
