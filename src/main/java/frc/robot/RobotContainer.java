/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.commands.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.Robot;
import frc.robot.commandGroups.GalacticSearch;
//Pathweaver stuff
import frc.robot.PathweaverConstants;

import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;
import java.nio.file.Path;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import java.io.IOException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.PIDController;

import java.text.DecimalFormat;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...

    private final SendableChooser<Command> m_chooser = new SendableChooser<>();
    public DecimalFormat decimalScale = new DecimalFormat("#,###.##");

    //Pathwever
    private String trajectoryJSON;
    private Trajectory test1Trajectory;
    private Path trajectoryPath;
    private RamseteCommand ramseteCommand;

    public final Joystick leftJoy = new Joystick(Constants.leftJoyPort);
    public final Joystick rightJoy = new Joystick(Constants.rightJoyPort);
    public final Joystick mechJoy = new Joystick(Constants.mechJoyPort);

    public JoystickButton leftTrigger = new JoystickButton(leftJoy, 1), leftButton2 = new JoystickButton(leftJoy, 2),
            leftButton3 = new JoystickButton(leftJoy, 3), leftButton4 = new JoystickButton(leftJoy, 4),
            leftButton5 = new JoystickButton(leftJoy, 5), leftButton6 = new JoystickButton(leftJoy, 6),
            leftButton7 = new JoystickButton(leftJoy, 7), leftButton8 = new JoystickButton(leftJoy, 8),
            leftButton9 = new JoystickButton(leftJoy, 9), leftButton10 = new JoystickButton(leftJoy, 10),
            leftButton11 = new JoystickButton(leftJoy, 11), leftButton12 = new JoystickButton(leftJoy, 12);

    public double getLeftJoyX() {
        return leftJoy.getRawAxis(0);
    }

    public double getLeftJoyY() {
        return leftJoy.getRawAxis(1);
    }

    public double getLeftJoyThrottle() {
        return leftJoy.getRawAxis(2);
    }

    public JoystickButton rightTrigger = new JoystickButton(rightJoy, 1),
            rightButton2 = new JoystickButton(rightJoy, 2), rightButton3 = new JoystickButton(rightJoy, 3),
            rightButton4 = new JoystickButton(rightJoy, 4), rightButton5 = new JoystickButton(rightJoy, 5),
            rightButton6 = new JoystickButton(rightJoy, 6), rightButton7 = new JoystickButton(rightJoy, 7),
            rightButton8 = new JoystickButton(rightJoy, 8), rightButton9 = new JoystickButton(rightJoy, 9),
            rightButton10 = new JoystickButton(rightJoy, 10), rightButton11 = new JoystickButton(rightJoy, 11),
            rightButton12 = new JoystickButton(rightJoy, 12);

    public double getRightJoyX() {
        return rightJoy.getRawAxis(0);
    }

    public double getRightJoyY() {
        return rightJoy.getRawAxis(1);
    }

    public double getRightJoyThrottle() {
        return rightJoy.getRawAxis(2);
    }

    public JoystickButton mechTrigger = new JoystickButton(mechJoy, 1), mechButton2 = new JoystickButton(mechJoy, 2),
            mechButton3 = new JoystickButton(mechJoy, 3), mechButton4 = new JoystickButton(mechJoy, 4),
            mechButton5 = new JoystickButton(mechJoy, 5), mechButton6 = new JoystickButton(mechJoy, 6),
            mechButton7 = new JoystickButton(mechJoy, 7), mechButton8 = new JoystickButton(mechJoy, 8),
            mechButton9 = new JoystickButton(mechJoy, 9), mechButton10 = new JoystickButton(mechJoy, 10),
            mechButton11 = new JoystickButton(mechJoy, 11), mechButton12 = new JoystickButton(mechJoy, 12);

    public double getMechJoyX() {
        return mechJoy.getRawAxis(0);
    }

    public double getMechJoyY() {
        return mechJoy.getRawAxis(1);
    }

    public double getMechJoyThrottle() {
        return mechJoy.getRawAxis(2);
    }

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        m_chooser.setDefaultOption("Barrel Racer", PathCommand("BarrelRacer"));
        m_chooser.addOption("Bounce", PathCommand("Bounce"));
        m_chooser.addOption("Slalom", PathCommand("Slalom"));
        m_chooser.addOption("Straight", PathCommand("Straight"));
        m_chooser.addOption("Straight By Code", StraightByCode());
        m_chooser.addOption("Barrel Racer By Code", PathCommandByCode("Barrel Racer"));
        m_chooser.addOption("Bounce By Code", PathCommandByCode("Bounce"));
        m_chooser.addOption("Slalom By Code", PathCommandByCode("Slalom"));
        SmartDashboard.putData(m_chooser);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Example : mechButton3.whenPressed(new IntakeBall());

        // Left Hand
        mechButton8.whileHeld(new ForwardTopConveyer());
        mechButton7.whileHeld(new ReverseTopConveyer());
        mechButton10.whileHeld(new ForwardBottomConveyer());
        mechButton9.whileHeld(new ReverseBottomConveyer());
        mechButton12.whileHeld(new IntakeBall());
        mechButton11.whileHeld(new OuttakeBall());

        // Right Hand
        mechButton5.whileHeld(new DeployIntake());
        mechButton3.whileHeld(new RetractIntake());
        //mechButton6.whileHeld(new ElevatorUp());
        //mechButton4.whileHeld(new ElevatorDown());
        //mechButton2.whileHeld(new IndexOne());

        // right stick
        rightTrigger.whileHeld(new DriveStraight());
        rightButton5.whileHeld(new ZoneOne());
        rightButton6.whileHeld(new ZoneTwo());
        rightButton3.whileHeld(new ZoneThree());
        rightButton4.whileHeld(new ZoneFour());
        //rightButton5.whenPressed(new ResetEncodersTest());
        //rightButton2.whileHeld(new WinchPull());

        //left stick
        leftTrigger.whileHeld(new LaunchBalls());
        leftButton4.whenPressed(new AlignToTarget2());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return m_chooser.getSelected();
    }

    public Command PathCommand(String trajectoryName)
    {
        trajectoryJSON = "paths/"+ trajectoryName + ".wpilib.json";
        trajectoryPath =  Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        try {
        test1Trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        }
        catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }

        ramseteCommand = new RamseteCommand(
            test1Trajectory,
            Robot.m_drivetrain::getPose,
            new RamseteController(PathweaverConstants.kRamseteB, PathweaverConstants.kRamseteZeta),
            new SimpleMotorFeedforward(PathweaverConstants.ksVolts,
                                    PathweaverConstants.kvVoltSecondsPerMeter,
                                    PathweaverConstants.kaVoltSecondsSquaredPerMeter),
            PathweaverConstants.kDriveKinematics,
            Robot.m_drivetrain::getWheelSpeeds,
            new PIDController(PathweaverConstants.kPDriveVel, 0, 0),
            new PIDController(PathweaverConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            Robot.m_drivetrain::driveByVolts,
            Robot.m_drivetrain
        );
        System.out.println("Initial Pose" + decimalScale.format(test1Trajectory.getInitialPose().getX()) + ", " + decimalScale.format(test1Trajectory.getInitialPose().getY()) + ")");
        SmartDashboard.putString("Initial Pose", "(" + decimalScale.format(test1Trajectory.getInitialPose().getX()) + ", " + decimalScale.format(test1Trajectory.getInitialPose().getY()) + ")");
        SmartDashboard.putString("Pose", test1Trajectory.getInitialPose().toString());
        System.out.println(test1Trajectory.getInitialPose().toString());

        //Robot.m_drivetrain.resetOdometry(test1Trajectory.getInitialPose());
        //return ramseteCommand.andThen(() -> Robot.m_drivetrain.driveByVolts(0, 0));

        // Reset robot odometry to initial position of path
        Command resetCommand = new InstantCommand(() -> Robot.m_drivetrain.resetOdometry(test1Trajectory.getInitialPose()));
        // Create CommandGroup of the resetCommand and our ramseteCommand
        Command returnGroup = new SequentialCommandGroup(resetCommand, ramseteCommand.andThen(() -> Robot.m_drivetrain.driveByVolts(0, 0)));
        return returnGroup;
    }

    public Command StraightByCode()
    {
        var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(PathweaverConstants.ksVolts,
                                    PathweaverConstants.kvVoltSecondsPerMeter,
                                    PathweaverConstants.kaVoltSecondsSquaredPerMeter),
            PathweaverConstants.kDriveKinematics,
            10);

        // Create config for trajectory
        TrajectoryConfig config =
            new TrajectoryConfig(PathweaverConstants.kMaxSpeedMetersPerSecond,
                                PathweaverConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(PathweaverConstants.kDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint);
                
        Trajectory straightTrajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0,0, new Rotation2d(0)), 
            List.of(new Translation2d(1,0)), 
            new Pose2d(3,0, new Rotation2d(0)), 
            config);

        RamseteCommand ramseteCommand = new RamseteCommand(
            straightTrajectory,
            Robot.m_drivetrain::getPose,
            new RamseteController(PathweaverConstants.kRamseteB, PathweaverConstants.kRamseteZeta),
            new SimpleMotorFeedforward(PathweaverConstants.ksVolts,
                    PathweaverConstants.kvVoltSecondsPerMeter,
                    PathweaverConstants.kaVoltSecondsSquaredPerMeter),
                    PathweaverConstants.kDriveKinematics,
            Robot.m_drivetrain::getWheelSpeeds,
            new PIDController(PathweaverConstants.kPDriveVel, 0, 0),
            new PIDController(PathweaverConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            Robot.m_drivetrain::driveByVolts,
            Robot.m_drivetrain
        );
    
        // Reset odometry to the starting pose of the trajectory.
        Robot.m_drivetrain.resetOdometry(straightTrajectory.getInitialPose());
    
        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> Robot.m_drivetrain.driveByVolts(0, 0));
    }

    public Command PathCommandByCode(String trajectoryName)
    {

        var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(PathweaverConstants.ksVolts,
                                    PathweaverConstants.kvVoltSecondsPerMeter,
                                    PathweaverConstants.kaVoltSecondsSquaredPerMeter),
            PathweaverConstants.kDriveKinematics,
            10);

        // Create config for trajectory
        TrajectoryConfig config =
            new TrajectoryConfig(PathweaverConstants.kMaxSpeedMetersPerSecond,
                                PathweaverConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(PathweaverConstants.kDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint);
        List<Translation2d> interiorWaypoints;
        switch (trajectoryName)
        {
            case "Barrel Racer":
                interiorWaypoints = BarrelRacerByCodeValues();
                break;

            case "Bounce":
                interiorWaypoints = BounceByCodeValues();
                break;

            case "Slalom":
                interiorWaypoints = SlalomByCodeValues();
                break;
            
            default:
                interiorWaypoints = BarrelRacerByCodeValues();
                break;

        }
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0,0, new Rotation2d(0)), 
            interiorWaypoints,
            new Pose2d(0,0, new Rotation2d(180)), 
            config);

        RamseteCommand ramseteCommand = new RamseteCommand(
            trajectory,
            Robot.m_drivetrain::getPose,
            new RamseteController(PathweaverConstants.kRamseteB, PathweaverConstants.kRamseteZeta),
            new SimpleMotorFeedforward(PathweaverConstants.ksVolts,
                    PathweaverConstants.kvVoltSecondsPerMeter,
                    PathweaverConstants.kaVoltSecondsSquaredPerMeter),
                    PathweaverConstants.kDriveKinematics,
            Robot.m_drivetrain::getWheelSpeeds,
            new PIDController(PathweaverConstants.kPDriveVel, 0, 0),
            new PIDController(PathweaverConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            Robot.m_drivetrain::driveByVolts,
            Robot.m_drivetrain
        );
    
        // Reset odometry to the starting pose of the trajectory.
        Robot.m_drivetrain.resetOdometry(trajectory.getInitialPose());
    
        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> Robot.m_drivetrain.driveByVolts(0, 0));
    }

    public List<Translation2d> BarrelRacerByCodeValues()
    {
        return List.of(
            new Translation2d(0.000, 0),
            new Translation2d(0.381,  0),
            new Translation2d(0.762,  0),
            new Translation2d(1.143,  0),
            new Translation2d(1.524,  0),
            new Translation2d(1.905,  0),
            new Translation2d(2.286,  0),
            new Translation2d(2.667,  0),
            new Translation2d(3.048,  0),
            new Translation2d(3.429,  0),
            new Translation2d(3.810,  0),
            new Translation2d(4.191, -0.381),
            new Translation2d(4.191,  -0.762),
            new Translation2d(4.191,  -1.143),
            new Translation2d(3.81,   -1.524),
            new Translation2d(3.429,  -1.524),
            new Translation2d(3.048,  -1.524),
            new Translation2d(2.667,  -1.524),
            new Translation2d(2.286,  -1.524),
            new Translation2d(1.905,  -1.143),
            new Translation2d(1.905,  -0.762),
            new Translation2d(1.905,  -0.381),
            new Translation2d(2.286,  0),
            new Translation2d(2.667,  0),
            new Translation2d(3.048,  0),
            new Translation2d(3.429,  0),
            new Translation2d(3.81,   0),
            new Translation2d(4.191,  0),
            new Translation2d(4.572,  0),
            new Translation2d(4.953,  0),
            new Translation2d(5.334,  0),
            new Translation2d(5.715,  0),
            new Translation2d(6.096,  0),
            new Translation2d(6.477,  0.381),
            new Translation2d(6.477,  0.762),
            new Translation2d(6.477,  1.143),
            new Translation2d(6.096,  1.524),
            new Translation2d(5.715,  1.524),
            new Translation2d(5.334,  1.524),
            new Translation2d(4.953,  1.524),
            new Translation2d(4.572,  1.524),
            new Translation2d(4.191,  1.143),
            new Translation2d(4.191,  0.762),
            new Translation2d(4.191,  0.381),
            new Translation2d(4.572,  0),
            new Translation2d(4.953,  -0.381),
            new Translation2d(5.334,  -0.762),
            new Translation2d(5.715,  -1.143),
            new Translation2d(6.096,  -1.524),
            new Translation2d(6.477,  -1.524),
            new Translation2d(6.858,  -1.524),
            new Translation2d(7.239,  -1.524),
            new Translation2d(7.62,   -1.524),
            new Translation2d(8.001,  -1.143),
            new Translation2d(8.001,  -0.762),
            new Translation2d(8.001,  -0.381),
            new Translation2d(7.8,   0.3),
            new Translation2d(7.4,  0.2),
            new Translation2d(7,  0.1),
            new Translation2d(6.477,  0),
            new Translation2d(6.096,  0),
            new Translation2d(5.715,  0),
            new Translation2d(5.334,  0),
            new Translation2d(4.953,  0),
            new Translation2d(4.572,  0),
            new Translation2d(4.191,  0),
            new Translation2d(3.81,   0),
            new Translation2d(3.429,  0),
            new Translation2d(3.048,  0),
            new Translation2d(2.667,  0),
            new Translation2d(2.286,  0),
            new Translation2d(1.905,  0),
            new Translation2d(1.524,  0),
            new Translation2d(1.143,  0),
            new Translation2d(0.762,  0),
            new Translation2d(0.381,  0),
            new Translation2d(0,      0)
        );
    }  
    
    public List<Translation2d> BounceByCodeValues()
    {
        return List.of(
            new Translation2d(0.000,0.000),
            new Translation2d(0.381,0.000),
            new Translation2d(0.762,0.000),
            new Translation2d(1.143,0.381),
            new Translation2d(1.524,0.762),
            new Translation2d(1.524,1.143),
            new Translation2d(1.524,1.524),
            new Translation2d(1.524,1.143),
            new Translation2d(1.524,0.762),
            new Translation2d(1.715,0.381),
            new Translation2d(1.905,0.000),
            new Translation2d(2.096,-0.381),
            new Translation2d(2.286,-0.762),
            new Translation2d(2.477,-1.143),
            new Translation2d(2.667,-1.524),
            new Translation2d(3.048,-1.524),
            new Translation2d(3.429,-1.524),
            new Translation2d(3.810,-1.143),
            new Translation2d(3.810,-0.762),
            new Translation2d(3.810,-0.381),
            new Translation2d(3.810,0.000),
            new Translation2d(3.810,0.381),
            new Translation2d(3.810,0.762),
            new Translation2d(3.810,1.143),
            new Translation2d(3.810,1.524),
            new Translation2d(3.810,1.143),
            new Translation2d(3.810,0.762),
            new Translation2d(3.810,0.381),
            new Translation2d(3.810,0.000),
            new Translation2d(3.810,-0.381),
            new Translation2d(3.810,-0.762),
            new Translation2d(3.810,-1.143),
            new Translation2d(4.191,-1.524),
            new Translation2d(4.572,-1.524),
            new Translation2d(4.953,-1.524),
            new Translation2d(5.334,-1.524),
            new Translation2d(5.715,-1.524),
            new Translation2d(6.096,-1.143),
            new Translation2d(6.096,-0.762),
            new Translation2d(6.096,-0.381),
            new Translation2d(6.096,0.000),
            new Translation2d(6.096,0.381),
            new Translation2d(6.096,0.762),
            new Translation2d(6.096,1.143),
            new Translation2d(6.096,1.524),
            new Translation2d(6.477,1.143),
            new Translation2d(6.858,0.762),
            new Translation2d(7.239,0.381),
            new Translation2d(7.620,0.000)
        );
    }  

    public List<Translation2d> SlalomByCodeValues()
    {
        return List.of(
            new Translation2d(0.000, 0.000),
            new Translation2d(0.381, 0.000),
            new Translation2d(0.762, 0.000),
            new Translation2d(1.143, 0.000),
            new Translation2d(1.524, 0.381),
            new Translation2d(1.524, 0.762),
            new Translation2d(1.524, 1.143),
            new Translation2d(1.905, 1.524),
            new Translation2d(2.286, 1.524),
            new Translation2d(2.667, 1.524),
            new Translation2d(3.048, 1.524),
            new Translation2d(3.429, 1.524),
            new Translation2d(3.810, 1.524),
            new Translation2d(4.191, 1.524),
            new Translation2d(4.572, 1.524),
            new Translation2d(4.953, 1.524),
            new Translation2d(5.334, 1.524),
            new Translation2d(5.715, 1.524),
            new Translation2d(6.096, 1.143),
            new Translation2d(6.096, 0.762),
            new Translation2d(6.096, 0.381),
            new Translation2d(6.477, 0.000),
            new Translation2d(6.858, 0.000),
            new Translation2d(7.239, 0.000),
            new Translation2d(7.620, 0.381),
            new Translation2d(7.620, 0.762),
            new Translation2d(7.620, 1.143),
            new Translation2d(7.239, 1.524),
            new Translation2d(6.858, 1.524),
            new Translation2d(6.477, 1.524),
            new Translation2d(6.096, 1.143),
            new Translation2d(6.096, 0.762),
            new Translation2d(6.096, 0.381),
            new Translation2d(5.715, 0.000),
            new Translation2d(5.334, 0.000),
            new Translation2d(4.953, 0.000),
            new Translation2d(4.572, 0.000),
            new Translation2d(4.191, 0.000),
            new Translation2d(3.810, 0.000),
            new Translation2d(3.429, 0.000),
            new Translation2d(3.048, 0.000),
            new Translation2d(2.667, 0.000),
            new Translation2d(2.286, 0.000),
            new Translation2d(1.905, 0.000),
            new Translation2d(1.524, 0.381),
            new Translation2d(1.524, 0.762),
            new Translation2d(1.524, 1.143),
            new Translation2d(1.143, 1.524),
            new Translation2d(0.762, 1.524),
            new Translation2d(0.381, 1.524),
            new Translation2d(0.000, 1.524)
        );
    }    
}
