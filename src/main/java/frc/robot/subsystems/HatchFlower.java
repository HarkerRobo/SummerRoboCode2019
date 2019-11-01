package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
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

    /**
     * DoubleSolenoid.Value for when the flower is not holding a hatch panel
     */
    public static final DoubleSolenoid.Value CLOSED = Value.kReverse;

    /**
     * DoubleSolenoid.Value for when the flower is holding a hatch panel
     */
    public static final DoubleSolenoid.Value OPEN = Value.kForward;

    private HatchFlower() {
        solenoid = new DoubleSolenoid(RobotMap.CAN_IDS.FLOWER_FORWARD_CHANNEL, RobotMap.CAN_IDS.FLOWER_REVERSE_CHANNEL);
        solenoid.set(OPEN);
    }

    @Override
    protected void initDefaultCommand() {
    }

    public DoubleSolenoid getSolenoid() {
        return solenoid;
    }

    public void toggleFlower(DoubleSolenoid.Value value) {
        solenoid.set(solenoid.get() == DoubleSolenoid.Value.kReverse ? OPEN : CLOSED);
    }
    
    public static HatchFlower getInstance() {
        if (instance == null)
            instance = new HatchFlower();
        return instance;
    }
}