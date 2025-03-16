// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.io.File;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoElevatorCommand;
import frc.robot.commands.Climb;
import frc.robot.commands.PositionTeleopElevator;
import frc.robot.commands.Seq_ElevatorAuto;
import frc.robot.commands.TeleopElevator;
import frc.robot.generated.MechanismConstants.ServoConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LaserSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;
import frc.robot.subsystems.ServoSubsystem;
import frc.robot.commands.ServoArm;
import frc.robot.generated.MechanismConstants.ServoConstants;;



public class RobotContainer {
    
        private final CommandXboxController driverJoystick = new CommandXboxController(0);
        private final CommandXboxController operatorJoystick = new CommandXboxController(1);

    private final SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

     SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverJoystick.getLeftY() * -1,
                                                                () -> driverJoystick.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverJoystick::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

    SwerveInputStream driveRobotOrientated = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverJoystick.getLeftY()*-0.5,
                                                                () -> driverJoystick.getLeftX()*-0.5)
                                                                .withControllerRotationAxis(
                                                                () -> driverJoystick.getRightX()*-1)
                                                                .deadband(OperatorConstants.DEADBAND)
                                                                .scaleTranslation(0.8)
                                                                .robotRelative(true)
                                                                .allianceRelativeControl(false);
    

                                                      

SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverJoystick::getRightX,
                                                                                           driverJoystick::getRightY)
                                                                                           . headingWhile(true);

Command driveFieldOrientatedDirectAngle = drivebase.driveFieldOrientated(driveDirectAngle);
Command driveFieldOrientatedDirectAngularVelocity = drivebase.driveFieldOrientated(driveAngularVelocity);
Command driveRobotOrientatedAngularVelocity = drivebase.driveFieldOrientated(driveRobotOrientated);
    /* Setting up bindings for necessary control of the swerve drive platform */

    SendableChooser<Command> m_chooser = new SendableChooser<>();
    


    public final static ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
    public final Seq_ElevatorAuto m_Seq_ElevatorAuto = new Seq_ElevatorAuto(m_elevatorSubsystem);
    public final static ClimbSubsystem m_climbSubsystem=new ClimbSubsystem();
    public final static ArmSubsystem m_armSubsystem=new ArmSubsystem();
    public final static LaserSubsystem m_laserSubsystem=new LaserSubsystem();
    public final static ServoSubsystem m_servoSubsystem = new ServoSubsystem();
    public RobotContainer() {
        configureBindings();

        drivebase.setDefaultCommand(driveFieldOrientatedDirectAngularVelocity);

        // m_chooser.addOption("Elevator Test Auto", m_Seq_ElevatorAuto);
        m_chooser.addOption( "Testing Simple", new PathPlannerAuto("testing simple"));
        // m_chooser.addOption( "Testing Complicated", new PathPlannerAuto("testing complicated"));
        SmartDashboard.putData("Auto Chooser",m_chooser);

    }

    

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

       // operatorJoystick.x().onTrue(new AutoElevatorCommand(m_elevatorSubsystem, 16.0, "up"));
       // operatorJoystick.a().onTrue(new AutoElevatorCommand(m_elevatorSubsystem, 16.0, "up"));
       // operatorJoystick.b().onTrue(new AutoElevatorCommand(m_elevatorSubsystem, 16.0, "up"));


        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverJoystick.a().onTrue(new AutoElevatorCommand(m_elevatorSubsystem, 16.0, "up"));
        driverJoystick.x().onTrue(new AutoElevatorCommand(m_elevatorSubsystem, -16.0, "down"));
        driverJoystick.rightBumper().whileTrue(new Climb(m_climbSubsystem, true,0.3));
        driverJoystick.leftBumper().whileTrue(new Climb(m_climbSubsystem, false,0.3));
        driverJoystick.y().toggleOnTrue(driveRobotOrientatedAngularVelocity);

        m_elevatorSubsystem.setDefaultCommand(new TeleopElevator(m_elevatorSubsystem, () -> operatorJoystick.getRightTriggerAxis(), ()->operatorJoystick.getLeftTriggerAxis()));
        operatorJoystick.a().onTrue(new PositionTeleopElevator(m_elevatorSubsystem,m_laserSubsystem, 0,0, null));
        operatorJoystick.b().onTrue(new PositionTeleopElevator(m_elevatorSubsystem,m_laserSubsystem, -3,3, null));
        operatorJoystick.y().onTrue(new PositionTeleopElevator(m_elevatorSubsystem,m_laserSubsystem, -1, 1,null));
        operatorJoystick.x().onTrue(new PositionTeleopElevator(m_elevatorSubsystem,m_laserSubsystem, -2,2, null));
    
        operatorJoystick.povUp().onTrue(new ServoArm(m_servoSubsystem,ServoConstants.kServoPositionOrignal));
        operatorJoystick.povRight().onTrue(new ServoArm(m_servoSubsystem,ServoConstants.kServoPositionRelease));
    }

    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }
}
