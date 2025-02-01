package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;

public class Seq_ElevatorAuto  extends SequentialCommandGroup {
    public Seq_ElevatorAuto(ElevatorSubsystem m_elevatorSubsystem) {
  
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand()); 
        addCommands(
        new AutoElevatorCommand(m_elevatorSubsystem, 5, "up"), //-> uncomment this when youre done testing directions
        new AutoElevatorCommand(m_elevatorSubsystem, 5, "down")
        // speed +, direction + -> drives backwards
        // speed -, distance + -> drives forwards extra far?
        // think it would do the same with speed +, distance -
        // robot just blips - makes a noise, kind of shudders, but doesn't go anywhere
        );
      }
}
