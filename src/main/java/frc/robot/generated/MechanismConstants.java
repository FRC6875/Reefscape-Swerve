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

   public static class LimeLightConstants{

    public static final double kRotSetpointReefAlignment = 0;  // Rotation
	public static final double kRotToleranceReefAlignment = 1;
	public static final double kXSetpointReefAlignment = -0.34;  // Vertical pose
	public static final double kXToleranceReefAlignment = 0.02;
	public static final double kYSetpointReefAlignment = 0.16;  // Horizontal pose
	public static final double kYToleranceReefAlignment = 0.02;

    public static final double kDontSeeTagWaitTime = 1;
	public static final double kPoseValidationTime = 1;

    public static final double kXReefAlignmentP = 3.3;
	public static final double kYReefAlignmentP = 3.3;
	public static final double kRotReefAlignmentP = 0.058;

   }

}