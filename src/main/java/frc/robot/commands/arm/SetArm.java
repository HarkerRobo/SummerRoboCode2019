package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.Arm;

/**
 * Sets the HatchExtender's solenoid to a specified state
 * 
 * @author Angela Jia
 * @author Jatin Kohli
 * 
 * @since 7/27/19
 */
public class SetArm extends InstantCommand {

    private DoubleSolenoid.Value value;

    public SetArm(DoubleSolenoid.Value value) {
        requires(Arm.getInstance());
        this.value = value;
    }

    @Override
    protected void initialize() {
        Arm.getInstance().getSolenoid().set(value);    
    }
}