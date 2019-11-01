package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.Arm;
import harkerrobolib.util.Pneumatics;

/**
 * Toggles the state of the Arm Solenoid between Forward and Reverse
 * 
 * @author Jatin Kohli
 * 
 * @since 6/17/19
 */
public class ToggleArm extends InstantCommand {
   
   public ToggleArm() {
        requires(Arm.getInstance());
   }
   
    @Override
    public void initialize() {
        Value value = Arm.getInstance().getSolenoid().get();
        Arm.getInstance().getSolenoid().set(Pneumatics.switchSolenoidValue(value));
    }
}