package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.wrist.MoveWristPercentOutput;
import harkerrobolib.util.MathUtil;

/**
 * Represents the Wrist Subsystem, with one Master Talon and one follower Victor
 * 
 * @author Jatin Kohli
 * @author Angela Jia
 * 
 * @since 6/26/19
 */
public class Wrist extends Subsystem {

    private static Wrist instance;

    private TalonSRX master;
    private VictorSPX follower;

    private static int COMPENSATION_VOLTAGE = 10;

    private static final boolean SENSOR_PHASE = true;
    private static final boolean TALON_INVERTED = false;
    private static final boolean VICTOR_INVERTED = false;

    public static final int FRONTMOST_POSITION = -197;
    public static final int BACKMOST_POSITION = 2095;
    public static final int MIDDLE_POSITION = 1050;
    public static final int HORIZONTAL_FRONT = 0;
    public static final int HORIZONTAL_BACK = 2035;
    public static final int DEFENSE_POSITION = 910;

    public static final double CARGO_FF = -0.1;
    public static final double HORIZONTAL_FORWARD_GRAV_FF = 0.10; //Gravity FF required to keep the wrist level at 0 degrees
    public static final double kS = 0.03;
    public static final double kA = 0.00036;
    public static final double kF = 2;

    public static final int MOTION_MAGIC_SLOT = 0;
    public static final double MOTION_MAGIC_KF = kF; //1.3
    public static final double MOTION_MAGIC_KP = 0.9; //1.2
    public static final double MOTION_MAGIC_KI = 0.0015;
    public static final double MOTION_MAGIC_KD = 30;
    public static final int CRUISE_VELOCITY = 420; //Encoder Units per 100ms
    public static final int MAX_ACCELERATION = 640; //Encoder Units per 100ms per s
    public static final double RAMP_RATE = 0.1;

    public static final int VELOCITY_SLOT = 1;
    public static final double VELOCITY_KF = kF;

    public static final int ALLOWABLE_ERROR = 50;
    public static final int MIDDLE_VARIANCE = 200;

    private Wrist() {
        master = new TalonSRX(RobotMap.CAN_IDS.WRIST_MASTER);
        follower = new VictorSPX(RobotMap.CAN_IDS.WRIST_FOLLOWER);
        talonInit();
    }

    private void talonInit() {
        master.configFactoryDefault();
        master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        follower.follow(master);
        master.setInverted(TALON_INVERTED);
        follower.setInverted(VICTOR_INVERTED);
        master.setSensorPhase(SENSOR_PHASE);
        master.configForwardSoftLimitThreshold(BACKMOST_POSITION);
        master.configReverseSoftLimitThreshold(FRONTMOST_POSITION);
        configVoltageComp();
        setupMotionMagic();
        setupVelocity();
        master.setSelectedSensorPosition(FRONTMOST_POSITION);
        master.setNeutralMode(NeutralMode.Coast);
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new MoveWristPercentOutput());
    }

    private void setupMotionMagic() {
        master.config_kF(MOTION_MAGIC_SLOT, MOTION_MAGIC_KF);
        master.config_kP(MOTION_MAGIC_SLOT, MOTION_MAGIC_KP);
        master.config_kI(MOTION_MAGIC_SLOT, MOTION_MAGIC_KI);
        master.config_kD(MOTION_MAGIC_SLOT, MOTION_MAGIC_KD);

        master.configMotionCruiseVelocity(CRUISE_VELOCITY);
        master.configMotionAcceleration(MAX_ACCELERATION);
        master.setStatusFramePeriod(StatusFrame.Status_10_Targets, 10);
        //master.configClosedloopRamp(RAMP_RATE);
    }

    private void setupVelocity() {
        master.config_kF(VELOCITY_SLOT, VELOCITY_KF);
    }

    private void configVoltageComp() {
        master.configVoltageCompSaturation(COMPENSATION_VOLTAGE);
        master.enableVoltageCompensation(true);
    }

    public double getCurrentDegreeAngle() {
        return MathUtil.map(getMaster().getSelectedSensorPosition(), HORIZONTAL_FRONT, HORIZONTAL_BACK, 0, 180);
    }

    public double calculateGravFF() {
        return HORIZONTAL_FORWARD_GRAV_FF * Math.cos(Math.toRadians(getCurrentDegreeAngle()));
    }

    public TalonSRX getMaster() {
        return master;
    }

    public static Wrist getInstance() {
        if (instance == null)
            instance = new Wrist();
        return instance;
    }
}