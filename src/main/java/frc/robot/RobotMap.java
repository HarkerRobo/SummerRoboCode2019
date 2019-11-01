package frc.robot;

/**
 * Stores constants including CAN IDs and Solenoid Indices.
 * 
 * @author Finn Frankis
 * @author Jatin Kohli
 * @author Chirag Kaushik
 * 
 * @since 6/14/19
 */
public class RobotMap {
    public static final int PRIMARY_PID_INDEX = 0;
    public static final int SECONDARY_PID_INDEX = 1;

    public static class CAN_IDS {
        
        public static int DT_LEFT_MASTER = 4; 
        public static int DT_RIGHT_MASTER = 1; 
        public static int DT_LEFT_FOLLOWER = 4; 
        public static int DT_RIGHT_FOLLOWER = 1;
        
        public static int EL_MASTER = 2; 
        public static int EL_VICTOR_LEFT = 3; 
        public static int EL_VICTOR_RIGHT = 5; 
        public static int EL_TALON_FOLLOWER = 6;

        public static int WRIST_MASTER = 9; 
        public static int WRIST_FOLLOWER = 9;

        public static int ARM_FORWARD_CHANNEL = 2; 
        public static int ARM_REVERSE_CHANNEL = 3;
        public static int BALL_INTAKE_SPARK = 1; 

        public static int ROLLERS_TALON = 7;
        
        public static int EXTENDER_FORWARD_CHANNEL = 4; 
        public static int EXTENDER_REVERSE_CHANNEL = 5;

        public static int FLOWER_FORWARD_CHANNEL = 6; 
        public static int FLOWER_REVERSE_CHANNEL = 7;
    }
}