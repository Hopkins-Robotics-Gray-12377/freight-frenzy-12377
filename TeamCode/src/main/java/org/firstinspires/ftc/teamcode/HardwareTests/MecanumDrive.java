package org.firstinspires.ftc.teamcode.HardwareTests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.ToggleMap;
import org.firstinspires.ftc.teamcode.UseMap;


@TeleOp(name="MecanumDrive", group="main")
public class MecanumDrive extends LinearOpMode {
    Bot robo = new Bot();

    //    for controller 1
    ToggleMap toggleMap1 = new ToggleMap();
    UseMap useMap1 = new UseMap();

    //    for controller 2
    ToggleMap toggleMap2 = new ToggleMap();
    UseMap useMap2 = new UseMap();
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    /* V * A * R * I * A * B * E * S *////* V * A * R * I * A * B * E * S */
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////

    private ElapsedTime runtime = new ElapsedTime();
    public ElapsedTime launchTime = new ElapsedTime();

    //    VARIABLES
    boolean launcherRunning = false;
    boolean servoMoving = false;

    //    IMU STUFF
    double newZero = 0;
    int fullRotationCount = 0;
    double previousAngle = 0;


    @Override
    public void runOpMode(){
        robo.init(hardwareMap);
        robo.imu();
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robo.runWithoutEncoderDrive();


        double startTime = 0;



        while(!opModeIsActive()) {

            telemetry.update();
        }


        waitForStart();

//        is the following necessary?
//        if (isStopRequested()) return;

        while(opModeIsActive()){
//            angleOverflow();
            roboDrive();
            


//            telemetry.addData("Back Left", robo.backLeft.getCurrentPosition());
//            telemetry.addData("Front Left", robo.frontLeft.getCurrentPosition());
//            telemetry.addData("Back Right", robo.backRight.getCurrentPosition());
//            telemetry.addData("Front Right", robo.frontRight.getCurrentPosition());

            telemetry.update();
        }
    }


    //Player 1
    public void roboDrive() {
        double stick_x = -gamepad1.left_stick_x;
        double stick_y = gamepad1.left_stick_y;
        double pX = 0;
        double pY = 0;
        double pRot = 0;
        double rotMultiplier = 0.6;
        double theta = Math.atan2(stick_y, stick_x); //Arctan2 doesn't have bad range restriction

        if (gamepad1.dpad_up || gamepad1.dpad_right || gamepad1.dpad_left || gamepad1.dpad_down) {

            double mag = 0.25;
            if (gamepad1.dpad_up) {
                pX = mag;
                pY = -mag;
            } else if (gamepad1.dpad_left) {
                pX = 2*mag;
                pY = 2*mag;
            } else if (gamepad1.dpad_down) {
                pX = -mag;
                pY = mag;
            } else if (gamepad1.dpad_right) {
                pX = -2*mag;
                pY = -2*mag;
            }

            pRot = -rotMultiplier*(gamepad1.right_trigger-gamepad1.left_trigger);
            robo.mecanumDrive(pX, pY, -pRot);
        }

        else {

            pRot = -0.6 * rotMultiplier*(gamepad1.right_stick_x);

            if (gamepad1.left_stick_y == 0 && gamepad1.left_stick_x == 0) {
                pRot = -rotMultiplier*(gamepad1.right_stick_x);
            }

            double gyroAngle = 0;
            double magnitudeMultiplier = 0;

//            if (toggleMap1.guide) {
////                try this for field centric driving
//                gyroAngle = getHeading();
//            } else {
//                gyroAngle = 0;
//            }

//            ALL TRIG IS IN RADIANS
            double modifiedTheta = theta + Math.PI / 4 - gyroAngle;

            double thetaInFirstQuad = Math.abs(Math.atan(stick_y / stick_x)); //square to circle conversion
//            arctan is same as tan inverse

            if (thetaInFirstQuad > Math.PI / 4) {
                magnitudeMultiplier = Math.sin(thetaInFirstQuad); //Works because we know y is 1 when theta > Math.pi/4
            } else if (thetaInFirstQuad <= Math.PI / 4) {
                magnitudeMultiplier = Math.cos(thetaInFirstQuad); //Works because we know x is 1 when theta < Math.pi/4
            }

            double magnitude = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * magnitudeMultiplier * (1 - Math.abs(pRot)); //Multiplied by (1-pRot) so it doesn't go over 1 with rotating
            pX = magnitude * Math.cos(modifiedTheta);
            pY = magnitude * Math.sin(modifiedTheta);

            robo.mecanumDrive(pX, pY, -pRot);


        }

    }

//    public void rrDrive(SampleMecanumDrive drive) {
//        drive.setWeightedDrivePower(
//                new Pose2d(
//                        -gamepad1.left_stick_y,
//                        -gamepad1.left_stick_x,
//                        -gamepad1.right_stick_x
//                )
//        );
//
//        drive.update();
//
//        Pose2d poseEstimate = drive.getPoseEstimate();
//        telemetry.addData("x", poseEstimate.getX());
//        telemetry.addData("y", poseEstimate.getY());
//        telemetry.addData("heading", poseEstimate.getHeading());
//    }



    public boolean cdCheck(double key, int cdTime){
        return runtime.milliseconds() - key > cdTime;
    }


    public boolean toggle(boolean variable){
        if(variable == true){
            variable = false;
        }
        else if(variable == false){
            variable = true;
        }
        return variable;

    }

//    COPIED FROM ROVER RUCKUS
//    public double getHeading(){ //Includes angle subtraction, angle to radian conversion, and 180>-180 to regular system conversion
//        Orientation angles = robo.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        double heading = angles.firstAngle;
//        heading = (Math.PI/180)*heading;
//        if(heading < 0){
//            heading = (2*Math.PI) + heading;
//        }
//
//        heading = heading - newZero;
//
//        heading += fullRotationCount*(2*Math.PI);
//        return heading;
//    }
//
//    public void angleOverflow(){ //Increase fullRotationCount when angle goes above 2*PI or below 0
//        double heading = getHeading() - fullRotationCount*(2*Math.PI);
//        //Warning: Will break if the robot does a 180 in less thank 1 tick, but that probably won't happen
//        if(heading < Math.PI/2 && previousAngle > 3*Math.PI/2){
//            fullRotationCount++;
//        }
//        if(heading > 3*Math.PI/2 && previousAngle < Math.PI/2){
//            fullRotationCount--;
//        }
//        previousAngle = heading;
//    }
}

