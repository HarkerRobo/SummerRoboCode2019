package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

public class Climber extends Subsystem {

    private Climber instance;
    private TalonSRX climberTalon;
    private VictorSPX climberVictor;
    public Climber(){
        climberTalon = new TalonSRX(RobotMap.CAN_IDS.CLIMBER_MASTER);
        climberVictor = new VictorSPX(RobotMap.CAN_IDS.CLIMBER_FOLLOWER);

    }    
    @Override
    protected void initDefaultCommand() {
       // setDefaultCommand(new Yeet());
    }
    public Climber getInstance() {
        if(instance==null){
            instance = new Climber();
        }
        return instance;
    }
    public void talonInit(){

    }
    public void followMasters(){
        climberVictor.follow(climberTalon);
    }

}