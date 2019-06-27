package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.wrist.MoveWristManual;

public class Wrist extends Subsystem {

    private static Wrist instance;

    private TalonSRX master;
    private VictorSPX follower;

    private Wrist() {
        master = new TalonSRX(RobotMap.CAN_IDS.WRIST_MASTER);
        follower = new VictorSPX(RobotMap.CAN_IDS.WRIST_FOLLOWER);

        talonInit();
    }

    private void talonInit() {
        follower.follow(master);
    }

    @Override
    protected void initDefaultCommand() {
    }

    public TalonSRX getMaster() {
        return master;
    }

    public VictorSPX getFollower() {
        return follower;
    }

    public static Wrist getInstance() {
        if (instance == null)
            instance = new Wrist();
        return instance;
    }
}