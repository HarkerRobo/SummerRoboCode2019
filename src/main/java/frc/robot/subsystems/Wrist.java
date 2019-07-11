package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
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

        configVoltageComp();
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new MoveWristManual());
    }

    private void configVoltageComp() {
        master.configVoltageCompSaturation(COMPENSATION_VOLTAGE);
        master.enableVoltageCompensation(true);
    }

    public double convertTicksToDegrees() {
        return MathUtil.map(getMaster().getSelectedSensorPosition(), HORIZONTAL_FRONT, HORIZONTAL_BACK, 0, 180);
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