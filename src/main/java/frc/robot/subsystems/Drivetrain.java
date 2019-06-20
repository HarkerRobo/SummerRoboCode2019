package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.robot.RobotMap;
import frc.robot.commands.drivetrain.DriveWithVelocity;
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

   public static final int WHEEL_DIAMETER = 4;

   private static final int COMPENSATION_VOLTAGE = 10;

   private static final boolean LEFT_MASTER_INVERTED = true;
   private static final boolean LEFT_VICTOR_INVERTED = true;
   private static final boolean RIGHT_MASTER_INVERTED = false;
   private static final boolean RIGHT_VICTOR_INVERTED = false;

   public static final double kS = 0.05;
   public static final double kF = 0;//1 / Drivetrain.MAX_FORWARD_VELOCITY;
   public static final double kA = 0;

   private static final int VELOCITY_SLOT = 0;
   private static final double VELOCITY_LEFT_kF = 0.23;
   private static final double VELOCITY_LEFT_kP = 0.75;
   private static final double VELOCITY_LEFT_kI = 0.001;
   private static final double VELOCITY_LEFT_kD = 0;
   private static final double VELOCITY_RIGHT_kF = 0.275;
   private static final double VELOCITY_RIGHT_kP = 0.75;
   private static final double VELOCITY_RIGHT_kI = 0;
   private static final double VELOCITY_RIGHT_kD = 0;
   private static final double VELOCITY_RAMP_RATE = 0.2;

   private static final int POSITION_SLOT = 1;
   private static final double POSITION_LEFT_kP = 0.3;
   private static final double POSITION_LEFT_kI = 0.001;
   private static final double POSITION_LEFT_kD = 60;
   private static final double POSITION_RIGHT_kP = 0.3;
   private static final double POSITION_RIGHT_kI = 0;
   private static final double POSITION_RIGHT_kD = 60;
   private static final int POSITION_IZONE = 300;
   private static final double POSITION_RAMP_RATE = 0.2;

   public static final int MOTION_PROF_SLOT = 2;
   private static final double MOTION_PROF_LEFT_kP = 0;//0.3;
   private static final double MOTION_PROF_LEFT_kI = 0;
   private static final double MOTION_PROF_LEFT_kD = 0;//60;
   private static final double MOTION_PROF_RIGHT_kP = 0;//0.3;
   private static final double MOTION_PROF_RIGHT_kI = 0;
   private static final double MOTION_PROF_RIGHT_kD = 0;//60;
   private static final double MOTION_PROF_RAMP_RATE = 0.2;
   private static final int MOTION_PROF_IZONE = 0;

   public static final double MAX_FORWARD_VELOCITY = 14;
   public static final double MAX_TURN_VELOCITY = 8;

   private Drivetrain() {
      super(new HSTalon(RobotMap.CAN_IDS.DT_LEFT_MASTER), new HSTalon(RobotMap.CAN_IDS.DT_RIGHT_MASTER),
            new VictorSPX(RobotMap.CAN_IDS.DT_LEFT_FOLLOWER), new VictorSPX(RobotMap.CAN_IDS.DT_RIGHT_FOLLOWER));
   }

   public void talonInit() {
      resetMasters();
      invertTalons(LEFT_MASTER_INVERTED, RIGHT_MASTER_INVERTED, LEFT_VICTOR_INVERTED, RIGHT_VICTOR_INVERTED);
      setNeutralMode(NeutralMode.Brake);
      configBothFeedbackSensors(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PRIMARY_PID_INDEX);
      
      setupVoltageComp();
   }

   public void initDefaultCommand() {
      setDefaultCommand(new DriveWithVelocity());
   }

   public void setupVelocityPID() {
      getLeftMaster().config_kF(VELOCITY_SLOT, VELOCITY_LEFT_kF);
      getLeftMaster().config_kP(VELOCITY_SLOT, VELOCITY_LEFT_kP);
      getLeftMaster().config_kI(VELOCITY_SLOT, VELOCITY_LEFT_kI);
      getLeftMaster().config_kD(VELOCITY_SLOT, VELOCITY_LEFT_kD);

      getRightMaster().config_kF(VELOCITY_SLOT, VELOCITY_RIGHT_kF);
      getRightMaster().config_kP(VELOCITY_SLOT, VELOCITY_RIGHT_kP);
      getRightMaster().config_kI(VELOCITY_SLOT, VELOCITY_RIGHT_kI);
      getRightMaster().config_kD(VELOCITY_SLOT, VELOCITY_RIGHT_kD);
      
      getLeftMaster().selectProfileSlot(VELOCITY_SLOT, RobotMap.PRIMARY_PID_INDEX);
      getRightMaster().selectProfileSlot(VELOCITY_SLOT, RobotMap.PRIMARY_PID_INDEX);
   
      getLeftMaster().configClosedloopRamp(VELOCITY_RAMP_RATE);
      getRightMaster().configClosedloopRamp(VELOCITY_RAMP_RATE);
   }

   public void setupPositionPID() {
      getLeftMaster().config_kP(POSITION_SLOT, POSITION_LEFT_kP);
      getLeftMaster().config_kI(POSITION_SLOT, POSITION_LEFT_kI);
      getLeftMaster().config_kD(POSITION_SLOT, POSITION_LEFT_kD);
      getLeftMaster().config_IntegralZone(POSITION_SLOT, POSITION_IZONE);

      getRightMaster().config_kP(POSITION_SLOT, POSITION_RIGHT_kP);
      getRightMaster().config_kI(POSITION_SLOT, POSITION_RIGHT_kI);
      getRightMaster().config_kD(POSITION_SLOT, POSITION_RIGHT_kD);
      getRightMaster().config_IntegralZone(POSITION_SLOT, POSITION_IZONE);

      getLeftMaster().selectProfileSlot(POSITION_SLOT, RobotMap.PRIMARY_PID_INDEX);
      getRightMaster().selectProfileSlot(POSITION_SLOT, RobotMap.PRIMARY_PID_INDEX);
   
      getLeftMaster().configClosedloopRamp(POSITION_RAMP_RATE);
      getRightMaster().configClosedloopRamp(POSITION_RAMP_RATE);
   }

   public void setupMotionProfilePID() {
      getLeftMaster().config_kP(MOTION_PROF_SLOT, MOTION_PROF_LEFT_kP);
      getLeftMaster().config_kI(MOTION_PROF_SLOT, MOTION_PROF_LEFT_kI);
      getLeftMaster().config_kD(MOTION_PROF_SLOT, MOTION_PROF_LEFT_kD);

      getRightMaster().config_kP(MOTION_PROF_SLOT, MOTION_PROF_RIGHT_kP);
      getRightMaster().config_kI(MOTION_PROF_SLOT, MOTION_PROF_RIGHT_kI);
      getRightMaster().config_kD(MOTION_PROF_SLOT, MOTION_PROF_RIGHT_kD);

      getLeftMaster().selectProfileSlot(MOTION_PROF_SLOT, RobotMap.PRIMARY_PID_INDEX);
      getLeftMaster().selectProfileSlot(MOTION_PROF_SLOT, RobotMap.PRIMARY_PID_INDEX);

      getLeftMaster().config_IntegralZone(MOTION_PROF_SLOT, MOTION_PROF_IZONE);
      getRightMaster().config_IntegralZone(MOTION_PROF_SLOT, MOTION_PROF_IZONE);

      getLeftMaster().configClosedloopRamp(MOTION_PROF_RAMP_RATE);
      getRightMaster().configClosedloopRamp(MOTION_PROF_RAMP_RATE);

      applyToMasters((talon) -> talon.setSelectedSensorPosition(0));
   }

   public void setupVoltageComp() {
      getLeftMaster().configVoltageCompSaturation(COMPENSATION_VOLTAGE);
      getLeftMaster().enableVoltageCompensation(true);

      getRightMaster().configVoltageCompSaturation(COMPENSATION_VOLTAGE);
      getRightMaster().enableVoltageCompensation(true);
   }
   
   public static Drivetrain getInstance() {
       if (instance == null) {
          instance = new Drivetrain();
       }
       return instance;
   }
}