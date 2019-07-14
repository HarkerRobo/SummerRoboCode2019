package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.wrist.MoveWristManual;
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
    public static final int HORIZONTAL_FRONT = 0;
    public static final int HORIZONTAL_BACK = 2035;

    public static final double HORIZONTAL_FORWARD_GRAV_FF = 0.07; //Gravity FF required to keep the wrist level at 0 degrees
    public static final double kS = 0.03;
    public static final double kA = 0.00036;

    public static final int MOTION_MAGIC_SLOT = 0;
    public static final double MOTION_MAGIC_KF = 2; //1.3
    public static final double MOTION_MAGIC_KP = 0.8; //0.3
    public static final double MOTION_MAGIC_KI = 0;//0.0001
    public static final double MOTION_MAGIC_KD = 30; //20
    public static final int CRUISE_VELOCITY = 420; //Encoder Units per 100ms
    public static final int MAX_ACCELERATION = 640; //Encoder Units per 100ms per s

    private Wrist() {
        master = new TalonSRX(RobotMap.CAN_IDS.WRIST_MASTER);
        follower = new VictorSPX(RobotMap.CAN_IDS.WRIST_FOLLOWER);
        talonInit();
    }

    private void talonInit() {
        master.configFactoryDefault();
        master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        master.setSelectedSensorPosition(FRONTMOST_POSITION);
        follower.follow(master);
        master.setInverted(TALON_INVERTED);
        follower.setInverted(VICTOR_INVERTED);
        master.setSensorPhase(SENSOR_PHASE);

        configVoltageComp();
        setupMotionMagic();
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new MoveWristManual());
    }

    private void setupMotionMagic() {
        master.config_kF(MOTION_MAGIC_SLOT, MOTION_MAGIC_KF);
        master.config_kP(MOTION_MAGIC_SLOT, MOTION_MAGIC_KP);
        master.config_kI(MOTION_MAGIC_SLOT, MOTION_MAGIC_KI);
        master.config_kD(MOTION_MAGIC_SLOT, MOTION_MAGIC_KD);

        master.configMotionCruiseVelocity(CRUISE_VELOCITY);
        master.configMotionAcceleration(MAX_ACCELERATION);
        master.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10);
    }

    private void configVoltageComp() {
        master.configVoltageCompSaturation(COMPENSATION_VOLTAGE);
        master.enableVoltageCompensation(true);
    }

    public double convertCurrentTicksToDegrees() {
        return MathUtil.map(getMaster().getSelectedSensorPosition(), HORIZONTAL_FRONT, HORIZONTAL_BACK, 0, 180);
    }

    public double calculateGravFF() {
        return HORIZONTAL_FORWARD_GRAV_FF * Math.cos(Math.toRadians(convertCurrentTicksToDegrees()));
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