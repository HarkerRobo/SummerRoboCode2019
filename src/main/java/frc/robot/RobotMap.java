package frc.robot;

/**
 * Stores constants including CAN IDs and solenoid indices.
 * 
 * @author Finn Frankis
 * @author Jatin Kohli
 * @author Chirag Kaushik
 * @since 6/14/19
 */
public class RobotMap {
    public static final int PRIMARY_PID_INDEX = 0;
    public static final int SECONDARY_PID_INDEX = 1;

    public static class CAN_IDS {
        public static final int DT_LEFT_MASTER = 4; 
        public static final int DT_RIGHT_MASTER = 6; 
        public static final int DT_LEFT_FOLLOWER = 2; 
        public static final int DT_RIGHT_FOLLOWER = 1;
        
        public static final int EL_MASTER = 2; 
        public static final int EL_VICTOR_LEFT_FRONT = 3; 
        public static final int EL_VICTOR_LEFT_BACK = 5; 
        public static final int EL_TALON_FOLLOWER = 3;

        public static final int WRIST_MASTER = 5; 
        public static final int WRIST_FOLLOWER = 7;

        public static final int ARM_FORWARD_CHANNEL = 4; 
        public static final int ARM_REVERSE_CHANNEL = 0;
        public static final int BALL_INTAKE_MASTER_VICTOR = 0;

        public static final int ROLLERS_TALON = 1;
        
        public static final int EXTENDER_FORWARD_CHANNEL = 6; 
        public static final int EXTENDER_REVERSE_CHANNEL = 2;

        public static final int FLOWER_FORWARD_CHANNEL = 1; 
        public static final int FLOWER_REVERSE_CHANNEL = 5;
    }
}