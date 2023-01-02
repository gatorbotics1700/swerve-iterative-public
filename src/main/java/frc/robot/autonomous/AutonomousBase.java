package frc.robot.autonomous;

//import edu.wpi.first.math.controller.PIDController; never used
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AutonomousBase extends AutonomousBasePID{
    public static final double kP= 0.004;
    public static final double kI= 0.0;
    public static final double kD= 0.0;
    DrivetrainSubsystem drivetrainSubsystem = Robot.m_drivetrainSubsystem;
  
    public static enum States{
        DRIVE,
        TURN;
    }

    public static States states = States.DRIVE;

    public void setState(States newState){
        states = newState;
    }

    public void periodic(double setpoint)
    {
        System.out.println("state: "+states);
        if (states == States.DRIVE){
            drivetrainSubsystem.setSpeed(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    distanceController.calculate(drivetrainSubsystem.getDistance() * AutonomousBasePID.x/(Math.sqrt(Math.pow(x, 2)) + (Math.pow(y, 2)))), 
                    distanceController.calculate(drivetrainSubsystem.getDistance() * AutonomousBasePID.y/(Math.sqrt(Math.pow(x, 2)) + (Math.pow(y, 2)))), 
                    0.0, 
                    drivetrainSubsystem.getGyroscopeRotation()
                )
            );

            if(distanceController.atSetpoint()){
                turnTo(setpoint, kP, kI, kD);
                setState(States.TURN);
            }
        } else if(states==States.TURN){
            drivetrainSubsystem.setSpeed(
                ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, directionController.calculate(drivetrainSubsystem.getGyroscopeRotation().getDegrees()), drivetrainSubsystem.getGyroscopeRotation())
            );
            System.out.println(directionController.getPositionError());
            if(directionController.atSetpoint()){
                directionController.close();
                //set state to next state
            }
        }
    }
}
