package frc.robot.commands.flower;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.HatchFlower;

/**
 * Sets the HatchFlower's solenoid to a specified state
 * 
 * @author Jatin Kohli
 * 
 * @since 8/1/19
 */
public class SetFlower extends InstantCommand {

    private DoubleSolenoid.Value value;

    public SetFlower(DoubleSolenoid.Value value) {
        requires(HatchFlower.getInstance());
        this.value = value;
    }

    @Override
    protected void initialize() {
        HatchFlower.getInstance().getSolenoid().set(value);
    }
}