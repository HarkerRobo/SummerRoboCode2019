package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * The Hatch Flower Subsystem, which includes the flower solenoid
 * 
 * @author Jatin Kohli
 * 
 * @since 6/14/19
 */
public class HatchFlower extends Subsystem {

    private static HatchFlower instance;

    private DoubleSolenoid solenoid;

    private HatchFlower() {
        solenoid = new DoubleSolenoid(RobotMap.CAN_IDS.FLOWER_FORWARD_CHANNEL, RobotMap.CAN_IDS.FLOWER_REVERSE_CHANNEL);
    }

    @Override
    protected void initDefaultCommand() {
    }

    public DoubleSolenoid getSolenoid() {
        return solenoid;
    }

    public static HatchFlower getInstance() {
        if (instance == null)
            instance = new HatchFlower();
        return instance;
    }
}