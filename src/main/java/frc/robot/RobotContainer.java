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
import frc.robot.commands.Intake;
import frc.robot.commands.IntakeOscillate;
import frc.robot.commands.PositionTeleopElevator;
import frc.robot.commands.Seq_ElevatorAuto;
import frc.robot.commands.TeleopElevator;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeWheelsSubsystem;
import frc.robot.subsystems.LaserSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;
import frc.robot.commands.IntakeWheels;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;




public class RobotContainer {

        private final UsbCamera climbCamera = CameraServer.startAutomaticCapture();
        private final UsbCamera coralCamera = CameraServer.startAutomaticCapture();

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

//SwerveInputStream driveRobotOrientated = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                               // () -> driverJoystick.getLeftY()*-0.5,
                                                               // () -> driverJoystick.getLeftX()*-0.5)
                                                               /*  .withControllerRotationAxis(
                                                                () -> driverJoystick.getRightX()*-1)
                                                                .deadband(OperatorConstants.DEADBAND)
                                                                .scaleTranslation(0.8)
                                                                .robotRelative(true)
                                                                .allianceRelativeControl(false);
    */

                                                      

SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverJoystick::getRightX,
                                                                                           driverJoystick::getRightY)
                                                                                           . headingWhile(true);

Command driveFieldOrientatedDirectAngle = drivebase.driveFieldOrientated(driveDirectAngle);
Command driveFieldOrientatedDirectAngularVelocity = drivebase.driveFieldOrientated(driveAngularVelocity);
///Command driveRobotOrientatedAngularVelocity = drivebase.driveFieldOrientated(driveRobotOrientated);
    /* Setting up bindings for necessary control of the swerve drive platform */

    SendableChooser<Command> m_chooser = new SendableChooser<>();
    


    public final static ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
    public final Seq_ElevatorAuto m_Seq_ElevatorAuto = new Seq_ElevatorAuto(m_elevatorSubsystem);
    public final static ClimbSubsystem m_climbSubsystem=new ClimbSubsystem();
    public final static LaserSubsystem m_laserSubsystem=new LaserSubsystem();
    public final static IntakeSubsystem m_intakeSubsystem=new IntakeSubsystem();
    public final static IntakeWheelsSubsystem m_intakeWheelsSubsystem=new IntakeWheelsSubsystem();

    public RobotContainer() {
        configureBindings();

        m_elevatorSubsystem.resetEncoder();
        drivebase.setDefaultCommand(driveFieldOrientatedDirectAngularVelocity);

        // m_chooser.addOption("Elevator Test Auto", m_Seq_ElevatorAuto);
        m_chooser.addOption( "Testing Simple", new PathPlannerAuto("testing simple"));
        m_chooser.addOption( "Leave", new PathPlannerAuto("Leave L4"));
        m_chooser.addOption( "Left Corner To I L2", new PathPlannerAuto("Left Corner To I L2"));


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
       // operatorJoystick.a().onTrue(new AutoElevatorCommand(m_elevatorSubsystem, -16.0, "up"));
       // operatorJoystick.x().onTrue(new AutoElevatorCommand(m_elevatorSubsystem, 16.0, "down"));
       // operatorJoystick.rightBumper().whileTrue(new Climb(m_climbSubsystem, true,0.3));
        driverJoystick.leftBumper().whileTrue(new Climb(m_climbSubsystem, false,0.3));
        //driverJoystick.y().toggleOnTrue(driveRobotOrientatedAngularVelocity);
       // driverJoystick.b().toggleOnTrue(driveFieldOrientatedDirectAngularVelocity);


        m_elevatorSubsystem.setDefaultCommand(new TeleopElevator(m_elevatorSubsystem, () -> operatorJoystick.getRightTriggerAxis(), ()->operatorJoystick.getLeftTriggerAxis()));
        operatorJoystick.a().onTrue(new PositionTeleopElevator(m_elevatorSubsystem, m_laserSubsystem,1, 0));
        operatorJoystick.b().onTrue(new PositionTeleopElevator(m_elevatorSubsystem,  m_laserSubsystem,-15, 16));
        operatorJoystick.y().onTrue(new PositionTeleopElevator(m_elevatorSubsystem,  m_laserSubsystem,-1, 1));
        operatorJoystick.x().onTrue(new PositionTeleopElevator(m_elevatorSubsystem,  m_laserSubsystem,-5, 2));
        operatorJoystick.povUp().onTrue(new Intake(m_intakeSubsystem, 0,0));
        operatorJoystick.povDown().onTrue(new Intake(m_intakeSubsystem, 0.5,0.1));
        operatorJoystick.povLeft().onTrue(new Intake(m_intakeSubsystem, 1,0.1));
        operatorJoystick.povRight().onTrue(new Intake(m_intakeSubsystem, 20,0.1));
        operatorJoystick.leftBumper().whileTrue(new IntakeWheels(m_intakeWheelsSubsystem, 0.3,true));
        operatorJoystick.rightBumper().whileTrue(new IntakeWheels(m_intakeWheelsSubsystem, 0.3,false));
        
        operatorJoystick.povCenter().whileTrue(new IntakeOscillate(m_intakeSubsystem, 0.1));
      //  operatorJoystick.povUp().onTrue(new ServoArm(m_servoSubsystem,ServoConstants.kServoPositionOrignal));
      //  operatorJoystick.povRight().onTrue(new ServoArm(m_servoSubsystem,ServoConstants.kServoPositionRelease));
    }

    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }
}
