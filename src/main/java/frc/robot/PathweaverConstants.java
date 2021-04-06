package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

public class PathweaverConstants{
    //Given values
    public static final double ksVolts = 0.688f;
    public static final double kvVoltSecondsPerMeter = 2.52f;
    public static final double kaVoltSecondsSquaredPerMeter = 0.653f;
    public static final double kPDriveVel = 2.56f;

    public static final double kTrackwidthMeters = 0.612f;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);
    
    //Our values
    public static final double kMaxSpeedMetersPerSecond = 2.00f; //Was 4.00f
    public static final double kMaxAccelerationMetersPerSecondSquared = .20f; //Was 2.00f

    //Default values
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

}
