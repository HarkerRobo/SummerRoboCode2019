package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.robot.RobotMap;
import frc.robot.commands.drivetrain.DriveWithVelocity;
import harkerrobolib.subsystems.HSDrivetrain;
import harkerrobolib.util.Conversions;
import harkerrobolib.wrappers.HSTalon;

/** 
 * The drivetrain subsystem, including our two master Talons with encoders and two follower Victors 
 * 
 * @author Finn Frankis 
 * @author Jatin Kohli 
 * @author Chirag Kaushik 
 * @author Angela Jia 
 *  
 * @since 6/14/19 
 */
public class Drivetrain extends HSDrivetrain {
    private static Drivetrain instance;

    public static final int WHEEL_DIAMETER = 4;

    private static final int COMPENSATION_VOLTAGE = 10;

    private static final boolean LEFT_MASTER_INVERTED = true;
    private static final boolean LEFT_VICTOR_INVERTED = true;
    private static final boolean RIGHT_MASTER_INVERTED = false;
    private static final boolean RIGHT_VICTOR_INVERTED = false;

    public static final double FREE_VELOCITY = 18;

    //Arbitrary Feed Forward Constants
    public static final double leftkS = 0.09;
    public static final double rightkS = 0.08;
    public static final double leftkF = 0.22;
    public static final double rightkF = 0.24;
    public static final double leftkA = 0.027;
    public static final double rightkA = 0.027;

    //Velocity PID Constants
    public static final int VELOCITY_SLOT = 0;
    private static final double VELOCITY_LEFT_kF = leftkF;
    private static final double VELOCITY_LEFT_kP = 0.75;
    private static final double VELOCITY_LEFT_kI = 0;
    private static final double VELOCITY_LEFT_kD = 8;
    private static final double VELOCITY_RIGHT_kF = rightkF;
    private static final double VELOCITY_RIGHT_kP = 0.75;
    private static final double VELOCITY_RIGHT_kI = 0;
    private static final double VELOCITY_RIGHT_kD = 8;
    public static final double VELOCITY_RAMP_RATE = 0.2;

    //Position PID Constants
    public static final int POSITION_SLOT = 1;
    private static final double POSITION_LEFT_kP = 0.3;
    private static final double POSITION_LEFT_kI = 0;//0.001;
    private static final double POSITION_LEFT_kD = 60;
    private static final double POSITION_RIGHT_kP = 0.3;
    private static final double POSITION_RIGHT_kI = 0;
    private static final double POSITION_RIGHT_kD = 60;
    private static final int POSITION_IZONE = 300;
    public static final double POSITION_RAMP_RATE = 0.2;

    //Motion Profiling Constants
    public static final int MOTION_PROF_SLOT = 2;
    private static final double MOTION_PROF_LEFT_kF = leftkF;
    private static final double MOTION_PROF_LEFT_kP = 1;
    private static final double MOTION_PROF_LEFT_kI = 0;
    private static final double MOTION_PROF_LEFT_kD = 0;
    private static final double MOTION_PROF_RIGHT_kF = rightkF;
    private static final double MOTION_PROF_RIGHT_kP = 1;
    private static final double MOTION_PROF_RIGHT_kI = 0;
    private static final double MOTION_PROF_RIGHT_kD = 0;
    public static final double MOTION_PROF_RAMP_RATE = 0;

    public static final double TX_kP = 0.018;//0.014;
    public static final double TX_kI = 0;
    public static final double TX_kD = 0;

    public static final double THOR_kP = 0;
    public static final double THOR_kI = 0;
    public static final double THOR_kD = 0;

    private static final int MOTION_FRAME_PERIOD = 10;

    public static final double MAX_FORWARD_VELOCITY = 14;
    public static final double MAX_TURN_VELOCITY = 8;

    private Drivetrain() {
        super(new HSTalon(RobotMap.CAN_IDS.DT_LEFT_MASTER), new HSTalon(RobotMap.CAN_IDS.DT_RIGHT_MASTER),
                new VictorSPX(RobotMap.CAN_IDS.DT_LEFT_FOLLOWER), new VictorSPX(RobotMap.CAN_IDS.DT_RIGHT_FOLLOWER));

         talonInit();
    }

    private void talonInit() {
        resetMasters();
        invertTalons(LEFT_MASTER_INVERTED, RIGHT_MASTER_INVERTED, LEFT_VICTOR_INVERTED, RIGHT_VICTOR_INVERTED);
        setNeutralMode(NeutralMode.Brake);
        configBothFeedbackSensors(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PRIMARY_PID_INDEX);
        configVoltageComp();
        applyToMasters((talon) -> talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, MOTION_FRAME_PERIOD));

        setupVelocityPID();
        setupPositionPID();
        setupMotionProfilePID();

        Conversions.setWheelDiameter(WHEEL_DIAMETER);
        applyToMasters((talon) -> talon.setSelectedSensorPosition(0));
    }

    public void initDefaultCommand() {
        setDefaultCommand(new DriveWithVelocity());
    }

    private void setupVelocityPID() {
        getLeftMaster().config_kF(VELOCITY_SLOT, VELOCITY_LEFT_kF);
        getLeftMaster().config_kP(VELOCITY_SLOT, VELOCITY_LEFT_kP);
        getLeftMaster().config_kI(VELOCITY_SLOT, VELOCITY_LEFT_kI);
        getLeftMaster().config_kD(VELOCITY_SLOT, VELOCITY_LEFT_kD);

        getRightMaster().config_kF(VELOCITY_SLOT, VELOCITY_RIGHT_kF);
        getRightMaster().config_kP(VELOCITY_SLOT, VELOCITY_RIGHT_kP);
        getRightMaster().config_kI(VELOCITY_SLOT, VELOCITY_RIGHT_kI);
        getRightMaster().config_kD(VELOCITY_SLOT, VELOCITY_RIGHT_kD);
    }

    private void setupPositionPID() {
        getLeftMaster().config_kP(POSITION_SLOT, POSITION_LEFT_kP);
        getLeftMaster().config_kI(POSITION_SLOT, POSITION_LEFT_kI);
        getLeftMaster().config_kD(POSITION_SLOT, POSITION_LEFT_kD);

        getRightMaster().config_kP(POSITION_SLOT, POSITION_RIGHT_kP);
        getRightMaster().config_kI(POSITION_SLOT, POSITION_RIGHT_kI);
        getRightMaster().config_kD(POSITION_SLOT, POSITION_RIGHT_kD);

        applyToMasters((talon) -> talon.config_IntegralZone(POSITION_SLOT, POSITION_IZONE));
    }

    private void setupMotionProfilePID() {
        getLeftMaster().config_kF(MOTION_PROF_SLOT, MOTION_PROF_LEFT_kF);
        getLeftMaster().config_kP(MOTION_PROF_SLOT, MOTION_PROF_LEFT_kP);
        getLeftMaster().config_kI(MOTION_PROF_SLOT, MOTION_PROF_LEFT_kI);
        getLeftMaster().config_kD(MOTION_PROF_SLOT, MOTION_PROF_LEFT_kD);

        getRightMaster().config_kF(MOTION_PROF_SLOT, MOTION_PROF_RIGHT_kF);
        getRightMaster().config_kP(MOTION_PROF_SLOT, MOTION_PROF_RIGHT_kP);
        getRightMaster().config_kI(MOTION_PROF_SLOT, MOTION_PROF_RIGHT_kI);
        getRightMaster().config_kD(MOTION_PROF_SLOT, MOTION_PROF_RIGHT_kD);

        applyToMasters((talon) -> talon.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, MOTION_FRAME_PERIOD));
    }

    private void configVoltageComp() {
        applyToMasters((talon) -> talon.configVoltageCompSaturation(COMPENSATION_VOLTAGE));
        applyToMasters((talon) -> talon.enableVoltageCompensation(true));
    }
    
    public static Drivetrain getInstance() {
        if (instance == null) {
            instance = new Drivetrain();
        }
        return instance;
    }
}