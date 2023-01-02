package frc.robot.autonomous;

import javax.naming.spi.DirStateFactory;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;

public class AutonomousBasePD extends AutonomousBase{
    public static final double turnKP= 0.004;
    public static final double turnKI= 0.0;
    public static final double turnKD= 0.0;
    public static final double driveKP= 0.00004;
    public static final double driveKI= 0.0;
    public static final double driveKD= 0.0;
    private final double DEADBAND = 3;
    private double xdirection;
    private double ydirection;

    private double driveSetpoint1;
    private double x1;
    private double y1;
    private double turnSetpoint1;

    DrivetrainSubsystem drivetrainSubsystem = Robot.m_drivetrainSubsystem;

    //pids
    private PIDController directionController = new PIDController(turnKP, turnKI, turnKD);
    private PIDController distanceController = new PIDController(driveKP, driveKI, driveKD);
    
    public AutonomousBasePD(double driveSetpoint1, double x1, double y1, double turnSetpoint1){
        this.driveSetpoint1 = driveSetpoint1*Constants.TICKS_PER_INCH;
        this.x1 = x1;
        this.y1 = y1;
        this.turnSetpoint1 = turnSetpoint1;
    }

    @Override
    public void init(){
        directionController.setTolerance(DEADBAND); 
        distanceController.setTolerance(DEADBAND*Constants.TICKS_PER_INCH);
        directionController.reset();
        distanceController.reset();
        states = States.FIRST;
    }

    public static enum States{
        FIRST,
        DRIVE,
        TURN,
        STOP;
    }

    private static States states = States.FIRST;

    public void setState(States newState){
        states = newState;
    }

    @Override
    public void periodic()
    {
        //System.out.println("setpoint: " + driveSetpoint);
        //System.out.println("state: "+states);
        if (states == States.FIRST){
            preDDD(driveSetpoint1);
            setState(States.DRIVE);
        }
        if (states == States.DRIVE){
            driveDesiredDistance(driveSetpoint1);
            if (distanceController.atSetpoint()){
                preTDA(turnSetpoint1);
                setState(States.STOP);
            }
        } else if(states==States.TURN){
            turnDesiredAngle(turnSetpoint1);
            if(directionController.atSetpoint()){
                setState(States.STOP);
            }
        } else {
            drivetrainSubsystem.stopDrive();
        }
    }

    //predrivedesiredistance
    public void preDDD(double setpoint){
        distanceController.reset();
        distanceController.setSetpoint(setpoint);
        //drivetrainSubsystem.zeroDriveEncoder();
        xdirection = x1/(Math.sqrt(Math.pow(x1, 2) + (Math.pow(y1, 2)))); //unit vector component x
        ydirection = y1/(Math.sqrt(Math.pow(x1, 2) + (Math.pow(y1, 2)))); //unit vector component y
        System.out.println(drivetrainSubsystem.getDistance());
    }

    @Override
    public void driveDesiredDistance(double driveSetpoint){      
        double speed = distanceController.calculate(drivetrainSubsystem.getDistance(), driveSetpoint); //calculate() takes in ticks for second argument
        if (!distanceController.atSetpoint()){
            System.out.println("pid val: " + speed);
            System.out.println("distance: " + drivetrainSubsystem.getDistance()/Constants.TICKS_PER_INCH);
            double scaledX = speed * xdirection;
            double scaledY = speed * ydirection;
            if (Math.abs(scaledX) > Constants.DRIVE_MOTOR_MAX_VOLTAGE || Math.abs(scaledY) > Constants.DRIVE_MOTOR_MAX_VOLTAGE){
                if (Math.abs(x1) > Math.abs(y1)){
                    scaledX = Constants.DRIVE_MOTOR_MAX_VOLTAGE * Math.signum(scaledX);
                    scaledY= Constants.DRIVE_MOTOR_MAX_VOLTAGE * y1/x1 * Math.signum(scaledY);
                } else {
                    scaledX = Constants.DRIVE_MOTOR_MAX_VOLTAGE * x1/y1 * Math.signum(scaledX);
                    scaledY = Constants.DRIVE_MOTOR_MAX_VOLTAGE * Math.signum(scaledY);
                }
            }
            if (Math.abs(scaledX) < Constants.DRIVE_MOTOR_MIN_VOLTAGE || Math.abs(scaledY) < Constants.DRIVE_MOTOR_MIN_VOLTAGE){
                if (Math.abs(x1) < Math.abs(y1)){
                    scaledX = Constants.DRIVE_MOTOR_MIN_VOLTAGE * Math.signum(scaledX);
                    scaledY= Constants.DRIVE_MOTOR_MIN_VOLTAGE * y1/x1 * Math.signum(scaledY);
                } else {
                    scaledX = Constants.DRIVE_MOTOR_MIN_VOLTAGE * x1/y1 * Math.signum(scaledX);
                    scaledY = Constants.DRIVE_MOTOR_MIN_VOLTAGE * Math.signum(scaledY);
                }
            }
            drivetrainSubsystem.setSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(scaledX, scaledY, 0.0, drivetrainSubsystem.getGyroscopeRotation()));
            System.out.println("speed: " + speed);
            System.out.println("x speed component: " + scaledX);
            System.out.println("y speed component: " + scaledY);
            System.out.println("error: " + distanceController.getPositionError()/Constants.TICKS_PER_INCH);
        } else {
            drivetrainSubsystem.setSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(0.0, 0.0, 0.0, drivetrainSubsystem.getGyroscopeRotation()));
        }

        drivetrainSubsystem.drive();
    }

    //pre turn desired angle
    public void preTDA(double setpoint){
        directionController.reset();
        directionController.setSetpoint(setpoint);
        drivetrainSubsystem.zeroGyroscope();
    }

    @Override
    public void turnDesiredAngle(double turnSetpoint){
        directionController.enableContinuousInput(0, 360); //so it goes shortest angle to get to correct
        double pidturnval = directionController.calculate(drivetrainSubsystem.getGyroscopeRotation().getDegrees(), turnSetpoint);
        System.out.println("pid val: " + pidturnval);
        drivetrainSubsystem.setSpeed(
            ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 
            Math.signum(pidturnval)*Math.max(Constants.DRIVE_MOTOR_MIN_VOLTAGE-0.1, Math.min(Constants.DRIVE_MOTOR_MAX_VOLTAGE, Math.abs(pidturnval))), 
            drivetrainSubsystem.getGyroscopeRotation())
        );
        System.out.println("error: " + directionController.getPositionError());
    }

}
