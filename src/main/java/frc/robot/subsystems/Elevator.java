package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.robot.commands.elevator.MoveElevatorPercentOutput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import harkerrobolib.wrappers.HSTalon;

public class Elevator extends Subsystem {

    private static Elevator instance;

    public static final double kS = 0.1;

    public static final double GRAVITY_FF = 0.09;
    public static final int ALLOWABLE_ERROR = 200;
    private static final int LOWER_SOFT_LIMIT = 500;
    private static final int UPPER_SOFT_LIMIT = 20000;

    private static final boolean SENSOR_PHASE = true;
    private static final boolean MASTER_INVERTED = true;
    private static final boolean LEFT_VICTOR_INVERTED = true;
    private static final boolean RIGHT_VICTOR_INVERTED = true;
    private static final boolean FOLLOWER_TALON_INVERTED = true;

    public static final int MOTION_MAGIC_SLOT = 0;
    private static final double MOTION_MAGIC_KF = 0.21;
    private static final double MOTION_MAGIC_KP = 0;
    private static final double MOTION_MAGIC_KI = 0;
    private static final double MOTION_MAGIC_KD = 0;
    private static final int CRUISE_VELOCITY = 2000; //Encoder Units per 100ms
    private static final int MAX_ACCELERATION = 4000; //Encoder Units per 100ms per s

    private HSTalon master;
    private HSTalon talonFollower;
    private VictorSPX victorLeft;
    private VictorSPX victorRight;

    private Elevator() {
        master = new HSTalon(RobotMap.CAN_IDS.EL_MASTER); 
        talonFollower = new HSTalon(RobotMap.CAN_IDS.EL_TALON_FOLLOWER);
        victorLeft = new VictorSPX(RobotMap.CAN_IDS.EL_VICTOR_LEFT);
        victorRight = new VictorSPX(RobotMap.CAN_IDS.EL_VICTOR_RIGHT);
        talonInit(); 
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new MoveElevatorPercentOutput());
    }

    public void talonInit() {
        master.configFactoryDefault();
        talonFollower.configFactoryDefault();
        followMasters();
        invert();
        configSoftLimits();
        master.setNeutralMode(NeutralMode.Brake);
        master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        master.setSensorPhase(SENSOR_PHASE);

        setupMotionMagic();
    }

    private void invert() {
        master.setInverted(MASTER_INVERTED);
        victorLeft.setInverted(LEFT_VICTOR_INVERTED);
        victorRight.setInverted(RIGHT_VICTOR_INVERTED);
        talonFollower.setInverted(FOLLOWER_TALON_INVERTED);
    }

    private void followMasters() {
        victorLeft.follow(master);
        victorRight.follow(master);
        talonFollower.follow(master);
    }

    private void configSoftLimits() {
        master.configForwardSoftLimitThreshold(UPPER_SOFT_LIMIT);
        master.configReverseSoftLimitThreshold(LOWER_SOFT_LIMIT);
        master.configForwardSoftLimitEnable(true);
        master.configReverseSoftLimitEnable(true);
    }

    private void setupMotionMagic() {
        master.config_kF(MOTION_MAGIC_SLOT, MOTION_MAGIC_KF);
        master.config_kP(MOTION_MAGIC_SLOT, MOTION_MAGIC_KP);
        master.config_kI(MOTION_MAGIC_SLOT, MOTION_MAGIC_KI);
        master.config_kD(MOTION_MAGIC_SLOT, MOTION_MAGIC_KD);

        master.configMotionCruiseVelocity(CRUISE_VELOCITY);
        master.configMotionAcceleration(MAX_ACCELERATION);
    }

    public HSTalon getMaster() {
        return master;
    }

    public static Elevator getInstance() {
        if (instance == null)
            instance = new Elevator();
        return instance;
    }
}