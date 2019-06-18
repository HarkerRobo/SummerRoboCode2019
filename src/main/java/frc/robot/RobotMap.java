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
        public static final int DT_LEFT_MASTER, DT_RIGHT_MASTER, DT_LEFT_FOLLOWER, DT_RIGHT_FOLLOWER;
        public static final int EL_MASTER, EL_VICTOR_LEFT_FRONT, EL_VICTOR_LEFT_BACK, EL_TALON_FOLLOWER;
        public static final int WRIST_MASTER, WRIST_FOLLOWER;

        public static final int BALL_INTAKE_MASTER_VICTOR, ARM_MASTER;
        public static final int ROLLERS_TALON;
        public static final int ARM_FORWARD_CHANNEL, ARM_REVERSE_CHANNEL;
        public static final int EXTENDER_FORWARD_CHANNEL, EXTENDER_REVERSE_CHANNEL;
        public static final int FLOWER_FORWARD_CHANNEL, FLOWER_REVERSE_CHANNEL;
        public static final int PCM, PIGEON;
        public static final int CLIMBER_TALON, CLIMBER_VICTOR;
        public static final int CLIMBER_ARM_FORWARD_CHANNEL, CLIMBER_ARM_REVERSE_CHANNEL;

        static
        {
            DT_LEFT_MASTER = 4;
            DT_RIGHT_MASTER = 6;
            DT_LEFT_FOLLOWER = 2;
            DT_RIGHT_FOLLOWER = 1;

            EL_MASTER = 2;
            EL_VICTOR_LEFT_FRONT = 3;
            EL_VICTOR_LEFT_BACK = 5;
            EL_TALON_FOLLOWER = 3;

            WRIST_MASTER = 5;
            WRIST_FOLLOWER = 7;

            BALL_INTAKE_MASTER_VICTOR = 0;
            ARM_MASTER = 0;
            CLIMBER_TALON = 0;
            CLIMBER_VICTOR = 4;

            ROLLERS_TALON = 1;
            
            ARM_FORWARD_CHANNEL = 4;
            ARM_REVERSE_CHANNEL = 0;

            EXTENDER_FORWARD_CHANNEL = 6; //1
            EXTENDER_REVERSE_CHANNEL = 2; //5

            FLOWER_FORWARD_CHANNEL = 1; //6
            FLOWER_REVERSE_CHANNEL = 5; //2

            CLIMBER_ARM_FORWARD_CHANNEL = 0;
            CLIMBER_ARM_REVERSE_CHANNEL = 0;

            PIGEON = 1;
            PCM = 0;
        }
    }
}