package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

@Autonomous(name="Vision_Auto_Red_Right", group="Training")
//@Disabled
public class Vision_Auto_Red_Right extends LinearOpMode {

    /* Declare OpMode members. */
    MaristBaseRobot2021_Quad robot   = new MaristBaseRobot2021_Quad();   
    private ElapsedTime runtime = new ElapsedTime();
    
    // Variables to control Speed
    double velocity = 0.5; // Default velocity
    
    //Camera fields
    WebcamName webcamName;
    OpenCvCamera camera;
    
    //Pipeline
    Pipeline_Target_Detect myPipeline;
    
    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        
        //Test of Webcam
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        myPipeline = new Pipeline_Target_Detect();
        
       
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
           public void onOpened() {
               //Start streaming from camera
               camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
               
                //Init Pipeline
                camera.setPipeline(myPipeline);
           } 
           @Override
           public void onError(int errorCode) {
               //Called if not working
           }
        });
        
        double xPos = -1;
        int zone = 2;
        
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < 3)) {
            xPos = myPipeline.getXPos();
            zone = myPipeline.getZone();
            telemetry.addData("XPos", xPos);
            telemetry.addData("Zone", zone);
            telemetry.update();
        }
        
        xPos = myPipeline.getXPos();
        zone = myPipeline.getZone();
        // Autonomous Finished
        telemetry.addData("XPos", xPos);
        telemetry.addData("Zone", zone);
        telemetry.update();
        
        delay(1);
        //Stop the Camera
        camera.closeCameraDevice();
        //sleep(1000);
        robot.moveDistance(-5, 0.6);
        delay(0.3);
        robot.strafeInches(0.5, -7, 5);
        delay(0.3);
        robot.turnAngle(-220, 0.5);
        delay(0.3);
        //arm decision 
        if(zone == 1) {//Works as of jan 21. slim margin of error, use full battery
            robot.leftArmMotorDeg(0.5, -500, 5);
            delay(0.5);
            robot.moveDistance(4.5, 0.7);
        }
        else if(zone == 2) {
            robot.leftArmMotorDeg(0.5, -500, 5);
            delay(0.5);
            robot.moveDistance(7, 0.7);
        }
        else {
            robot.leftArmMotorDeg(0.5, -800, 5);
            delay(0.5);
            robot.moveDistance(8, 0.7);
        }
        //park in the parking spot
        delay(0.5);
        robot.leftHand.setPosition(0.4);
        delay(1.5);
        robot.moveDistance(-10, 1.0);
        delay(0.5);
        robot.leftHand.setPosition(0);
        delay(0.3);
        //reset arm
        if(zone == 1) {
            robot.leftArmMotorDeg(0.5, 480, 5);
        }
        else if(zone == 2) {
            robot.leftArmMotorDeg(0.5, 480, 5);
        }
        else {
            robot.leftArmMotorDeg(0.5, 780, 5);
        }
        delay(0.2);
        robot.turnAngle(-70, 0.5);
        delay(0.3);
        robot.strafeInches(0.8, 25, 5);
        delay(0.3);
        robot.moveDistance(-30, 1.0);
        delay(0.3);
        robot.strafeInches(0.8, -20, 5);
        
        // Autonomous Finished
        telemetry.addData("Path", "Complete");
        telemetry.update();
        //sleep(1000);
        
    }

    // Functions for REACH 2019 based on Python Turtles
    public void forward(double inches)
    {
        robot.driveStraightInches(velocity, inches, 10);
    }
    
    public void right(double degrees)
    {
        robot.pointTurnDegrees(velocity, degrees, 10);
    }
    
    public void left(double degrees)
    {
        degrees = degrees * -1;
        robot.pointTurnDegrees(velocity, degrees, 10);
    }
    
    public void speed(int speed)
    {
        double newSpeed = (double)speed / 10.0;
        velocity = newSpeed;
    }
    
    // Sample Delay Code
    public void delay(double t) { // Imitates the Arduino delay function
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < t)) {
            // telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            // telemetry.update();
        }
    }
}
