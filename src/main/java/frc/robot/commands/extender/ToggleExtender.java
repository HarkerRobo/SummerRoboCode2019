package frc.robot.commands.extender;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.HatchExtender;
import harkerrobolib.util.Pneumatics;

public class ToggleExtender extends InstantCommand {
   
   public ToggleExtender() {
        requires(HatchExtender.getInstance());
   }
   
    @Override
    public void initialize() {
        Value value = HatchExtender.getInstance().getSolenoid().get();
        HatchExtender.getInstance().getSolenoid().set(Pneumatics.switchSolenoidValue(value));
    }
}