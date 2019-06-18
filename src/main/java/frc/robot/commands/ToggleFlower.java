package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.HatchFlower;

public class ToggleFlower extends InstantCommand {
   
   public ToggleFlower() {
        requires(HatchFlower.getInstance());
   }
   
    @Override
    public void initialize() {
        Value value = HatchFlower.getInstance().getSolenoid().get();

        if (value == Value.kForward)
            HatchFlower.getInstance().getSolenoid().set(Value.kReverse);
        else
            HatchFlower.getInstance().getSolenoid().set(Value.kForward);
    }
}