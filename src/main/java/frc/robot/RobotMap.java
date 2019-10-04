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

    public static final boolean PRACTICE_BOT = false;

    public static class CAN_IDS {
        static {
            if (PRACTICE_BOT) {
                DT_LEFT_MASTER = 4; 
                DT_RIGHT_MASTER = 6; 
                DT_LEFT_FOLLOWER = 2; 
                DT_RIGHT_FOLLOWER = 1;
                    
                EL_MASTER = 2; 
                EL_VICTOR_LEFT = 3; 
                EL_VICTOR_RIGHT = 5; 
                EL_TALON_FOLLOWER = 3;

                WRIST_MASTER = 5; 
                WRIST_FOLLOWER = 7;

                ARM_FORWARD_CHANNEL = 1;
                ARM_REVERSE_CHANNEL = 2;

                BALL_INTAKE_VICTOR = 0; //Summer bot
                BALL_INTAKE_SPARK = 1; //Season bot

                ROLLERS_TALON = 1;
                
                EXTENDER_FORWARD_CHANNEL =  0;
                EXTENDER_REVERSE_CHANNEL = 6;

                FLOWER_FORWARD_CHANNEL = 5;
                FLOWER_REVERSE_CHANNEL = 4;
            } else {
                DT_LEFT_MASTER = 4; 
                DT_RIGHT_MASTER = 1; 
                DT_LEFT_FOLLOWER = 4; 
                DT_RIGHT_FOLLOWER = 1;
                
                EL_MASTER = 2; 
                EL_VICTOR_LEFT = 3; 
                EL_VICTOR_RIGHT = 5; 
                EL_TALON_FOLLOWER = 6;

                WRIST_MASTER = 9; 
                WRIST_FOLLOWER = 9;

                ARM_FORWARD_CHANNEL = 2; 
                ARM_REVERSE_CHANNEL = 3;
                BALL_INTAKE_VICTOR = 0; //Summer bot
                BALL_INTAKE_SPARK = 1; //Season bot

                ROLLERS_TALON = 7;
                
                EXTENDER_FORWARD_CHANNEL = 4; 
                EXTENDER_REVERSE_CHANNEL = 5;

                FLOWER_FORWARD_CHANNEL = 6; 
                FLOWER_REVERSE_CHANNEL = 7;
            }
        }
        public static int DT_LEFT_MASTER; 
        public static int DT_RIGHT_MASTER; 
        public static int DT_LEFT_FOLLOWER; 
        public static int DT_RIGHT_FOLLOWER;
        
        public static int EL_MASTER; 
        public static int EL_VICTOR_LEFT; 
        public static int EL_VICTOR_RIGHT; 
        public static int EL_TALON_FOLLOWER;

        public static int WRIST_MASTER; 
        public static int WRIST_FOLLOWER;

        public static int ARM_FORWARD_CHANNEL; 
        public static int ARM_REVERSE_CHANNEL;
        public static int BALL_INTAKE_VICTOR; //Summer bot
        public static int BALL_INTAKE_SPARK; //Season bot

        public static int ROLLERS_TALON;
        
        public static int EXTENDER_FORWARD_CHANNEL; 
        public static int EXTENDER_REVERSE_CHANNEL;

        public static int FLOWER_FORWARD_CHANNEL; 
        public static int FLOWER_REVERSE_CHANNEL;
    }
}