package org.firstinspires.ftc.teamcode.StatesOnBotJava;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.StatesOnBotJava.StatesToggleMapOBJ;
import org.firstinspires.ftc.teamcode.StatesOnBotJava.StatesUseMapOBJ;
import org.firstinspires.ftc.teamcode.StatesOnBotJava.StatesBotOBJ;


@TeleOp(name = "AStatesTeleOp")
public class StatesTeleOpOBJ extends LinearOpMode {
    StatesBotOBJ robot = new StatesBotOBJ();

    //    for controller 1
    StatesToggleMapOBJ toggleMap1 = new StatesToggleMapOBJ();
    StatesUseMapOBJ useMap1 = new StatesUseMapOBJ();

    StatesToggleMapOBJ toggleMap2 = new StatesToggleMapOBJ();
    StatesUseMapOBJ useMap2 = new StatesUseMapOBJ();

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
            joystickMoveCapper();

            telemetry.addData("Holder position", robot.holder.getPosition());

            telemetry.update();

            updateKeys();
        }
    }

    public void joystickMoveCapper() {
        if (toggleMap2.dpad_right) {
            robot.capper.setPosition(robot.capper.getPosition() + (-gamepad2.left_stick_y)/1000);
        }
    }

    public void holder() {
        double servoPosition = robot.holder.getPosition();

        if (gamepad2.y) {
            servoPosition = .35;
        }

        if (gamepad2.x) {
            servoPosition = .75;
//            robot.holderDeposit();
        }

        if (gamepad2.b) {
            servoPosition = .07;
//            robot.holderNormal();
        }

        if (gamepad2.left_bumper) {
            servoPosition = 0;
//            robot.holderExtraLow();
        }

        robot.holder.setPosition(servoPosition);
    }


    public void intake() {
        if (toggleMap1.a) {
            robot.intake();
        } else if (toggleMap1.b) {
            robot.outtake();
        } else {
            robot.intake.setPower(0);
        }
    }

    public void carousel() {
        if (toggleMap1.x) {
            robot.moveCarousel();
        } else if (toggleMap1.y) {
            robot.reverseCarousel();
        } else {
            robot.carousel.setPower(0);
        }
    }

    public void slides() {
        telemetry.addData("Slide Position: ", robot.slides.getCurrentPosition());
        telemetry.addData("Initial Slide Position: ", robot.initialSlidePos);

        double slidePower = 0;

        if (gamepad2.dpad_up && robot.slides.getCurrentPosition() < robot.topSlidePos) {
            slidePower = .6;
        } else if (gamepad2.dpad_down && robot.slides.getCurrentPosition() > robot.initialSlidePos) {
            slidePower = -.1;
        } else if (robot.slides.getCurrentPosition() <= robot.initialSlidePos || robot.slides.getCurrentPosition() >= robot.topSlidePos) {
            slidePower = 0;
        }

        telemetry.addData("Slide Power", slidePower);

        robot.slides.setPower(slidePower);
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
            robot.mecanumDrive(pX, pY, -pRot);
        }

        else {

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


        }

    }

    public void updateKeys(){
        if(gamepad1.b && cdCheck(useMap1.b, 500)){
            toggleMap1.b = toggle(toggleMap1.b);
            useMap1.b = runtime.milliseconds();
            toggleMap1.a = false;
        }
        if(gamepad1.a && cdCheck(useMap1.a, 500)){
            toggleMap1.a = toggle(toggleMap1.a);
            useMap1.a = runtime.milliseconds();
            toggleMap1.b = false;
        }
        if(gamepad1.x && cdCheck(useMap1.x, 500)){
            toggleMap1.x = toggle(toggleMap1.x);
            useMap1.x = runtime.milliseconds();
            toggleMap1.y = false;
        }
        if(gamepad1.y && cdCheck(useMap1.y, 500)){
            toggleMap1.y = toggle(toggleMap1.y);
            useMap1.y = runtime.milliseconds();
            toggleMap1.x = false;
        }
        if(gamepad2.dpad_right && cdCheck(useMap2.dpad_right, 500)){
            toggleMap2.dpad_right = toggle(toggleMap2.dpad_right);
            useMap2.dpad_right = runtime.milliseconds();
        }
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