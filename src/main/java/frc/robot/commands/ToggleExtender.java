package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.HatchExtender;

public class ToggleExtender extends InstantCommand {
   
   public ToggleExtender() {
        requires(HatchExtender.getInstance());
   }
   
    @Override
    public void initialize() {
        Value value = HatchExtender.getInstance().getSolenoid().get();

        if (value == Value.kForward)
            HatchExtender.getInstance().getSolenoid().set(Value.kReverse);
        else
            HatchExtender.getInstance().getSolenoid().set(Value.kForward);
    }
}