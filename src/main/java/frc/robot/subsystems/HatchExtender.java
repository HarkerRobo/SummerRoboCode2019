package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

public class HatchExtender extends Subsystem {

    private static HatchExtender instance;

    private DoubleSolenoid solenoid;

    public HatchExtender() {
        solenoid = new DoubleSolenoid(RobotMap.CAN_IDS.EXTENDER_FORWARD_CHANNEL, RobotMap.CAN_IDS.EXTENDER_REVERSE_CHANNEL);
    }

    @Override
    protected void initDefaultCommand() {
    }

    public DoubleSolenoid getSolenoid() {
        return solenoid;
    }

    public static HatchExtender getInstance() {
        if (instance == null)
            instance = new HatchExtender();
        return instance;
    }
}