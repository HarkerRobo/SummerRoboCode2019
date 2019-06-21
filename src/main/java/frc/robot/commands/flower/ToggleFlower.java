package frc.robot.commands.flower;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.HatchFlower;
import harkerrobolib.util.Pneumatics;

/**
 * Toggles the state of the Hatch Flower Solenoid between Forward and Reverse
 * 
 * @author Jatin Kohli
 * 
 * @since 6/17/19
 */
public class ToggleFlower extends InstantCommand {
   
   public ToggleFlower() {
        requires(HatchFlower.getInstance());
   }
   
    @Override
    public void initialize() {
        Value value = HatchFlower.getInstance().getSolenoid().get();
        HatchFlower.getInstance().getSolenoid().set(Pneumatics.switchSolenoidValue(value));
    }
}