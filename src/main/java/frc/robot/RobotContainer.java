// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.io.File;
import java.util.function.DoubleSupplier;

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
import frc.robot.commands.PositionIntake;
import frc.robot.commands.PositionTeleopElevator;
import frc.robot.commands.ResetIntake;
import frc.robot.commands.RunAuto;
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
import frc.robot.commands.L2;
import frc.robot.commands.L3;
import frc.robot.commands.L4;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;




public class RobotContainer {

       // private final UsbCamera climbCamera = CameraServer.startAutomaticCapture();
       // private final UsbCamera coralCamera = CameraServer.startAutomaticCapture();

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
       //  m_chooser.addOption( "Testing Simple", new PathPlannerAuto("testing simple"));
        m_chooser.addOption( "Leave Blue", new PathPlannerAuto("Leave Blue"));
        m_chooser.addOption( "Leave Red", new PathPlannerAuto("Leave Red"));
        m_chooser.addOption( "Score Preloaded Coral L2 Red", new PathPlannerAuto("Score Preloaded Coral L2 Red"));
        m_chooser.addOption( "Score Preloaded Coral L2 Blue", new PathPlannerAuto("Score Preloaded Coral L2 Blue"));


       //  m_chooser.addOption( "Left Corner To I L2", new PathPlannerAuto("Left Corner To I L2"));


        // m_chooser.addOption( "Testing Complicated", new PathPlannerAuto("testing complicated"));
        SmartDashboard.putData("Auto Chooser",m_chooser);

    }

    

    private void configureBindings() {
        

        DoubleSupplier elevatorDownPos = () -> 0;
        DoubleSupplier elevatorIntakePos = () -> SmartDashboard.getNumber("Elevator Intake Pos",17);
        DoubleSupplier elevatorL2Pos = () -> SmartDashboard.getNumber("Elevator L2 Pos",25);
        DoubleSupplier elevatorL3Pos = () -> SmartDashboard.getNumber("Elevator L3 Pos",10);
        DoubleSupplier elevatorL4Pos = () -> SmartDashboard.getNumber("Elevator L4 Pos",40);
        DoubleSupplier holderUpPos = () -> SmartDashboard.getNumber("Holder Up Pos",0.3);
        DoubleSupplier holderIntakePos = () -> SmartDashboard.getNumber("Holder Intake Pos",15.4);
        DoubleSupplier holderEjectPos = () -> SmartDashboard.getNumber("Holder Down Pos",30);

        SmartDashboard.putNumber("Elevator Intake Pos", elevatorIntakePos.getAsDouble());
        SmartDashboard.putNumber("Elevator L2 Pos", elevatorL2Pos.getAsDouble());
        SmartDashboard.putNumber("Elevator L3 Pos", elevatorL3Pos.getAsDouble());
        SmartDashboard.putNumber("Elevator L4 Pos", elevatorL4Pos.getAsDouble());
        SmartDashboard.putNumber("Holder Up Pos", holderUpPos.getAsDouble());
        SmartDashboard.putNumber("Holder Intake Pos", holderIntakePos.getAsDouble());
        SmartDashboard.putNumber("Holder Down Pos", holderEjectPos.getAsDouble());


        // m_elevatorSubsystem.setDefaultCommand(new TeleopElevator(m_elevatorSubsystem, () -> operatorJoystick.getRightTriggerAxis(), ()->operatorJoystick.getLeftTriggerAxis()));
        operatorJoystick.leftTrigger().whileTrue(new Intake(
            m_elevatorSubsystem,
            m_intakeSubsystem,
            m_intakeWheelsSubsystem,
            elevatorIntakePos,
            holderIntakePos
        ));
        operatorJoystick.leftTrigger().onFalse(new PositionIntake(m_intakeSubsystem, holderIntakePos));
        operatorJoystick.leftTrigger().onFalse(new PositionTeleopElevator(m_elevatorSubsystem, elevatorDownPos));

        operatorJoystick.y().whileTrue(new L2(
            m_elevatorSubsystem,
            m_intakeSubsystem,
            m_intakeWheelsSubsystem,
            elevatorL2Pos,
            holderEjectPos
        ));
        operatorJoystick.y().onFalse(new PositionIntake(m_intakeSubsystem, holderIntakePos));
       // operatorJoystick.y().onFalse(new IntakeWheels(m_intakeWheelsSubsystem, -0.2));
        operatorJoystick.y().onFalse(new PositionTeleopElevator(m_elevatorSubsystem, elevatorDownPos));

        operatorJoystick.b().whileTrue(new L3(
            m_elevatorSubsystem,
            m_intakeSubsystem,
            m_intakeWheelsSubsystem,
            elevatorL3Pos,
            holderEjectPos
        ));
        operatorJoystick.b().onFalse(new PositionIntake(m_intakeSubsystem, holderIntakePos));
        operatorJoystick.b().onFalse(new PositionTeleopElevator(m_elevatorSubsystem, elevatorDownPos));
        
        operatorJoystick.a().whileTrue(new L4(
            m_elevatorSubsystem,
            m_intakeSubsystem,
            m_intakeWheelsSubsystem,
            elevatorL3Pos,
            holderEjectPos
        ));
        operatorJoystick.a().onFalse(new PositionIntake(m_intakeSubsystem, holderIntakePos));
        operatorJoystick.a().onFalse(new PositionTeleopElevator(m_elevatorSubsystem, elevatorDownPos));

       operatorJoystick.rightTrigger().onTrue(new PositionTeleopElevator(m_elevatorSubsystem, elevatorDownPos));
        operatorJoystick.povUp().onTrue(new PositionIntake(m_intakeSubsystem, holderUpPos));
        operatorJoystick.povLeft().onTrue(new PositionIntake(m_intakeSubsystem, holderIntakePos));
        operatorJoystick.povDown().onTrue(new PositionIntake(m_intakeSubsystem, holderEjectPos));
        operatorJoystick.povRight().onTrue(new ResetIntake(m_intakeSubsystem));
        operatorJoystick.rightBumper().whileTrue(new IntakeWheels(m_intakeWheelsSubsystem, -0.4));
        operatorJoystick.leftBumper().whileTrue(new IntakeWheels(m_intakeWheelsSubsystem, 0.3));
      //  operatorJoystick.povUp().onTrue(new ServoArm(m_servoSubsystem,ServoConstants.kServoPositionOrignal));
      //  operatorJoystick.povRight().onTrue(new ServoArm(m_servoSubsystem,ServoConstants.kServoPositionRelease));
    }

    public Command getAutonomousCommand() {
        return new RunAuto(
            m_intakeSubsystem,
            () -> 0.3,
            m_chooser.getSelected()
        );
    }
}
