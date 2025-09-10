// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.MomentOfInertiaUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.SwerveDrive;
import frc.robot.Subsystems.Drive.Drivetrain;
import frc.robot.Subsystems.Drive.GyroPidgeonIO;
import frc.robot.Subsystems.Drive.GyroSim;
import frc.robot.Subsystems.Drive.ModuleIOSim;
import frc.robot.Subsystems.Drive.ModuleIOSparkMax;
import frc.robot.Subsystems.Drive.SwerveConstants;

public class RobotContainer {
  public static Drivetrain m_Drivetrain;
  public static final CommandXboxController driverController =
      new CommandXboxController(0);
  public final JoystickButton resetHeading_Start =
      new JoystickButton(driverController.getHID(), XboxController.Button.kStart.value);
      
  public SwerveDriveSimulation swerveSim = null;
  //public final SendableChooser<Command> autoChooser;
  public final LoggedDashboardChooser<Command> autoChooser;
  public RobotContainer() {
    if(Robot.isReal()){
     m_Drivetrain = new Drivetrain(new GyroPidgeonIO(), new ModuleIOSparkMax(1), new ModuleIOSparkMax(2), new ModuleIOSparkMax(3), new ModuleIOSparkMax(4));
     }
    else{
    swerveSim = new SwerveDriveSimulation(SwerveConstants.SIM_CONFIG, new Pose2d(3,3,Rotation2d.kZero));
    SimulatedArena.getInstance().addDriveTrainSimulation(swerveSim);
    m_Drivetrain = new Drivetrain(new GyroSim(swerveSim.getGyroSimulation()), new ModuleIOSim(swerveSim.getModules()[0],1),new ModuleIOSim(swerveSim.getModules()[1],2), new ModuleIOSim(swerveSim.getModules()[2],3),new ModuleIOSim(swerveSim.getModules()[3], 4));
    }
     m_Drivetrain.setDefaultCommand(new SwerveDrive());
     //autoChooser = AutoBuilder.buildAutoChooser();
     autoChooser = new LoggedDashboardChooser<>("Auto Routine", AutoBuilder.buildAutoChooser());
     SmartDashboard.putData("AutoChooser",autoChooser.getSendableChooser());
     
    registerNamedCommands();
    configureBindings();
    
  }

  private void configureBindings() {
    resetHeading_Start.onTrue(new InstantCommand(m_Drivetrain::zeroHeading, m_Drivetrain));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
  public void registerNamedCommands() {}
}
