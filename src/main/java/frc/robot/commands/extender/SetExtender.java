package frc.robot.commands.extender;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.HatchExtender;

/**
 * Sets the HatchExtender's solenoid to a specified state
 * 
 * @author Jatin Kohli
 * 
 * @since 8/1/19
 */
public class SetExtender extends InstantCommand {

    private DoubleSolenoid.Value value;

    public SetExtender(DoubleSolenoid.Value value) {
        requires(HatchExtender.getInstance());
        this.value = value;
    }

    @Override
    protected void initialize() {
        HatchExtender.getInstance().getSolenoid().set(value);    
    }
}