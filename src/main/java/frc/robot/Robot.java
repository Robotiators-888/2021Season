/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import java.io.File;
import java.io.FileNotFoundException;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 * 
 * @param <Commander>
 * @param <Scenario>
 * @param <WaypointMap>
 */
public class Robot<Commander, Scenario, WaypointMap> extends TimedRobot {
    /* Here are the instantiations of everything used in the class */

    Intake intake;
    Navigation nav;
    OI oi;
    Shooter shooter;
    Turret turret;
    

    DeadReckoning location;

    IMU imu;
    WaypointTravel guidence;
    UDPSender send;
    UDPReceiver receive;
    WaypointMap map;

    UsbCamera camera0;
    UsbCamera camera1;
    
    SendableChooser autoChooser;
    Scenario autoScenario;
    Commander auto;

    // Pneumatics
    Compressor compressor;

    boolean done = false;

    int delay;
    int counter = 0;

    @Override
    public void robotInit() {

        try {
            map = new WaypointMap(new File("/home/lvuser/Waypoints2020.txt"));
            autoScenario = new Scenario(new File("/home/lvuser/TestAuto.txt"));
            SmartDashboard.putBoolean("Suicide", false);
        }
        catch (FileNotFoundException e) {
            e.printStackTrace();
            SmartDashboard.putBoolean("Suicide", true);
        }

        oi = new OI();
        DriveTrain drive = new DriveTrain();
        intake = new Intake(oi);
        index = new Indexing(oi, intake);
        climb = new Climber(oi);
        turret = new Turret(oi);

        imu = new IMU();
        send = new UDPSender();
        receive = new UDPReceiver();
        location = new DeadReckoning(drive, imu, receive, map);
        guidence = new WaypointTravel(drive, location);
        nav = new Navigation(oi, drive, guidence);
        jetsonLight = new JetsonLight(oi);

        camera0 = CameraServer.getInstance().startAutomaticCapture(0);
        camera1 = CameraServer.getInstance().startAutomaticCapture(1);


        shooter = new Shooter(oi, location, index, turret, map);

        compressor = new Compressor(Constants.PCM);

        autoChooser = new SendableChooser<>();
        autoChooser.addDefault("Default Auto", null);
        autoChooser.addObject("Auto Scenario 2", null);
        SmartDashboard.putData("Auto Scenarios", autoChooser);
        auto = new Commander(autoScenario, map, location, guidence, intake,
                index, shooter);

    }

    @Override
    public void autonomousInit() {
        location.reset();
    }

    @Override
    public void autonomousPeriodic() {
        location.updateTracker();
        location.updateDashboard();
        index.updateBallPoitions();
        auto.periodic();

    }

    @Override
    public void teleopInit() {
        compressor.setClosedLoopControl(true);
        location.reset();
        // send.sendMessage();
    }

    @Override
    public void teleopPeriodic() {
        // receive.run();
        location.updateTracker();
        location.updateDashboard();
        nav.navTeleopPeriodic();
        climb.climberTeleopPeriodic();
        intake.intakeTeleopPeriodic();
        index.indexPeriodic();
        shooter.shooterTeleopPeriodic();
        turret.turretTeleopPeriodic();
        jetsonLight.jetsonLightPeriodic();
    }

    @Override
    public void testPeriodic() {

    }

}