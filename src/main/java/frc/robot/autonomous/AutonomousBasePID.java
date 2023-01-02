package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
//import frc.robot.subsystems.DrivetrainSubsystem; never used

public class AutonomousBasePID {
    public static PIDController directionController;
    public static PIDController distanceController;
    public static double x;
    public static double y;
    private final double DEADBAND = 3;

    public void turnTo(double setpoint, double P, double I, double D)
    {
        directionController = new PIDController(P, I, D);
        directionController.reset(); //sets pid error back to 0
        directionController.setTolerance(DEADBAND); 
        directionController.setSetpoint(setpoint);
    }

    //length ratio is y/x
    public void driveTo(double setpoint, double P, double I, double D, double _x, double _y)
    {
        distanceController = new PIDController(P, I, D);
        distanceController.reset(); //sets pid error back to 0
        distanceController.setTolerance(DEADBAND); 
        distanceController.setSetpoint(setpoint);
        x= _x;
        y= _y;
    }
}
