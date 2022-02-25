package org.firstinspires.ftc.teamcode.HardwareTests;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Bot;


@TeleOp
public class ArmCapperTest extends LinearOpMode {
    Bot robo = new Bot();


    private Servo servo = null;
    @Override
    public void runOpMode() {
        robo.init(hardwareMap);
        servo = hardwareMap.servo.get("armCapper");

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

            servoAdjust();
            servoToLimits();
            telemetry.addData("Servo Pos", servo.getPosition());
            telemetry.update();



            telemetry.update();
        }

    }

    public void servoAdjust(){
        servo.setPosition(servo.getPosition() + (-gamepad2.left_stick_y)/1000);
    }

    public void servoToLimits(){
        if(gamepad2.b){
            servo.setPosition(1);
        }
        else if(gamepad2.a){
            servo.setPosition(0);
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
}