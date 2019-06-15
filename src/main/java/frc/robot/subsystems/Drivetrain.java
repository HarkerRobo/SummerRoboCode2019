package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.robot.RobotMap;
import frc.robot.commands.DriveWithPercentOutput;
import harkerrobolib.subsystems.HSDrivetrain;
import harkerrobolib.wrappers.HSTalon;

/**
 * Represents the drivetrain on the robot.
 * 
 * @author Finn Frankis
 * @author Jatin Kohli
 * @author Chirag Kaushik
 * @since 6/14/19
 */
public class Drivetrain extends HSDrivetrain {
   private static Drivetrain instance;

   private static final boolean LEFT_MASTER_INVERTED = true;
   private static final boolean LEFT_VICTOR_INVERTED = true;
   private static final boolean RIGHT_MASTER_INVERTED = false;
   private static final boolean RIGHT_VICTOR_INVERTED = false;

   private Drivetrain() {
      super(new HSTalon(RobotMap.CAN_IDS.DT_LEFT_MASTER), new HSTalon(RobotMap.CAN_IDS.DT_RIGHT_MASTER),
            new VictorSPX(RobotMap.CAN_IDS.DT_LEFT_FOLLOWER), new VictorSPX(RobotMap.CAN_IDS.DT_RIGHT_FOLLOWER));
   }

   public void talonInit() {
      
      invertTalons(LEFT_MASTER_INVERTED, RIGHT_MASTER_INVERTED, LEFT_VICTOR_INVERTED, RIGHT_VICTOR_INVERTED);
   }

   public void initDefaultCommand() {
      setDefaultCommand(new DriveWithPercentOutput());
   }
   
   public static Drivetrain getInstance() {
       if (instance == null) {
          instance = new Drivetrain();
       }
       return instance;
   }
}