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

    public static final double GRAVITY_FF = 0.09;
    public static final int ALLOWABLE_ERROR = 0;
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
        followMasters();
        invert();
        master.setNeutralMode(NeutralMode.Brake);
        master.configFactoryDefault();
        talonFollower.configFactoryDefault();
        master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        master.setSensorPhase(false);
    }

    private void invert() {
        master.setInverted(false);
        victorLeft.setInverted(false);
        victorRight.setInverted(false);
        talonFollower.setInverted(false);
    }

    private void followMasters() {
        victorLeft.follow(master);
        victorRight.follow(master);
        talonFollower.follow(master);
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