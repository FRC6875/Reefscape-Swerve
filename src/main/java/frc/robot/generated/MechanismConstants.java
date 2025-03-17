package frc.robot.generated;


public class MechanismConstants {
    
    public static class ElevatorConstants {
        public static final int kElevatorPort = 14;
        public static final int kElevatorEncoderConvFact = (int) (Math.PI*6/8.45);

    }



   public static class ClimbConstants{
    public static final int kClimbPort = 15;
    public static final int kClimbEncoderConvFact = (int) (Math.PI*6/8.45);
   }

   public static class ServoConstants{
    public static final int kServoPort = 1;
    public static final int kServoPositionOrignal = 0;
    public static final int kServoPositionRelease = 135;
   }

   public static class LaserConstants{
    public static final int kLaserPort = 0;
}
    public static class IntakeConstants{
        public static final int kIntakePort = 2;
        public static final int kIntakeEncoderConvFact = (int) (360/42);
    }

}
