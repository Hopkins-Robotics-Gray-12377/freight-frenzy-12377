package org.firstinspires.ftc.teamcode.HardwareTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.ToggleMap;
import org.firstinspires.ftc.teamcode.UseMap;

@TeleOp(name = "Intake & Carousel Test")
public class TestTeleOp extends LinearOpMode {
    Bot robot = new Bot();

    //    for controller 1
    ToggleMap toggleMap1 = new ToggleMap();
    UseMap useMap1 = new UseMap();

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.imu();
        robot.runWithoutEncoderDrive();

        while(!opModeIsActive()) {

            telemetry.update();
        }

        waitForStart();


        while(opModeIsActive()){

            roboDrive();
            intake();
            carousel();
            slides();

            holder();

            telemetry.addData("Holder position", robot.holder.getPosition());

            telemetry.update();
        }
    }

    public void holder() {
//
//        if (robot.slides.getCurrentPosition() < robot.topSlidePos - 800 && robot.slides.getCurrentPosition() > robot.initialSlidePos + 800) {
//            robot.holderHold();
//        }


        if (gamepad1.y) {
            robot.holderHold();
        }

        if (gamepad1.right_bumper) {
            robot.holderDeposit();
        }

        if (gamepad1.left_bumper) {
            robot.holderNormal();
        }
    }


    public void intake() {
        if (gamepad1.a) {
            robot.intake();
        } else if (gamepad1.b) {
            robot.outtake();
        } else if (gamepad1.x) {
            robot.intake.setPower(0);
        }
    }

    public void carousel() {
        if (gamepad1.dpad_right) {
            robot.moveCarousel();
        } else if (gamepad1.dpad_left) {
            robot.carousel.setPower(0);
        }
    }

    public void slides() {
        telemetry.addData("Slide Position: ", robot.slides.getCurrentPosition());
        telemetry.addData("Initial Slide Position: ", robot.initialSlidePos);

        double slidePower = 0;

        if (gamepad1.dpad_up && robot.slides.getCurrentPosition() < robot.topSlidePos) {
            slidePower = .4;
        } else if (gamepad1.dpad_down && robot.slides.getCurrentPosition() > robot.initialSlidePos) {
            slidePower = -.1;
        } else if (robot.slides.getCurrentPosition() <= robot.initialSlidePos || robot.slides.getCurrentPosition() >= robot.topSlidePos) {
            slidePower = 0;
        }

        telemetry.addData("Slide Power", slidePower);

        robot.slides.setPower(slidePower);
    }


    private void runSlidesDown() {
        int slidePosition = robot.slides.getCurrentPosition();

        if (slidePosition <= robot.initialSlidePos) {
            robot.slides.setPower(0);
        } else {
            robot.slides.setPower(.1);
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

//        if (gamepad1.dpad_up || gamepad1.dpad_right || gamepad1.dpad_left || gamepad1.dpad_down) {
//
//            double mag = 0.25;
//            if (gamepad1.dpad_up) {
//                pX = mag;
//                pY = -mag;
//            } else if (gamepad1.dpad_left) {
//                pX = 2*mag;
//                pY = 2*mag;
//            } else if (gamepad1.dpad_down) {
//                pX = -mag;
//                pY = mag;
//            } else if (gamepad1.dpad_right) {
//                pX = -2*mag;
//                pY = -2*mag;
//            }
//
//            pRot = -rotMultiplier*(gamepad1.right_trigger-gamepad1.left_trigger);
//            robot.mecanumDrive(pX, pY, -pRot);
//        }

//        else {

            pRot = -0.6 * rotMultiplier*(-gamepad1.right_stick_x);

            if (gamepad1.left_stick_y == 0 && gamepad1.left_stick_x == 0) {
                pRot = -rotMultiplier*(-gamepad1.right_stick_x);
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

            robot.mecanumDrive(pX, pY, -pRot);


//        }

    }

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
}
