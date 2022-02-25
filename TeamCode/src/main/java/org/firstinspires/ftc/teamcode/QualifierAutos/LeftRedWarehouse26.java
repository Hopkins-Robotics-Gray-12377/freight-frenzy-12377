package org.firstinspires.ftc.teamcode.QualifierAutos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.ArrayList;


@Autonomous
public class LeftRedWarehouse26 extends LinearOpMode {

//    Pose2d startPose = new Pose2d(-62.0, -18.0, 0.0);


    private Pose2d startPose = new Pose2d(-62.5, 34.0, 0.0);

    private Pose2d carouselPose = new Pose2d(-55.0, 60.0, 0.0);

    private Pose2d deposit1 = new Pose2d(-35.0, 55.0, Math.toRadians(-90.0));

    private Pose2d deposit2 = new Pose2d(5.0, 0.0, Math.toRadians(-90.0));

    private Pose2d depositPose = new Pose2d(-23.0, 14.0, Math.toRadians(-90.0));

    private Pose2d storagePark = new Pose2d(-36.0, 62.5, 0.0);

    private Pose2d mineralPark1 = new Pose2d(-0.0, -10.0, Math.toRadians(-90.0));

    private Pose2d mineralPark2 = new Pose2d(-40.0, 4.0, Math.toRadians(-90.0));

    private Pose2d mineralFinalPark = new Pose2d(-50.0, -50.0, Math.toRadians(-90.0));

    public ElapsedTime launchTime = new ElapsedTime();

    enum LeftRed {
        Start,
        Duck,
        Deposit1,
        Deposit2,
        Deposit,
        StoragePark,
        MineralPark1,
        MineralPark2,
        MineralParkFinal,
        Park
    }


    LeftRed currentLeftRedState = LeftRed.Start;


    @Override
    public void runOpMode() throws InterruptedException{
        Bot robo = new Bot();
        robo.autoInit(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);


        // Left Red
        Trajectory Duck = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(carouselPose)
                .build();

        Trajectory Deposit1 = drive.trajectoryBuilder(Duck.end())
                .forward(20.0)
                .splineTo(new Vector2d(deposit2.getX(), deposit2.getY()), deposit2.getHeading())
                .build();

        Trajectory deposit = drive.trajectoryBuilder(Deposit1.end())
                .lineToSplineHeading(depositPose)
                .build();

        Trajectory parkInMineral1 = drive.trajectoryBuilder(deposit.end())
                .lineToSplineHeading(mineralPark2)
                .build();

        Trajectory finalParkInMineral = drive.trajectoryBuilder(parkInMineral1.end())
                .forward(40)
                .build();


        ElapsedTime init = new ElapsedTime();

        init.reset();

        waitForStart();

        if (isStopRequested()) return;

        if (opModeIsActive()) {
            currentLeftRedState = LeftRed.Start;
            robo.autoRaiseSlidesALittle();
            drive.followTrajectoryAsync(Duck);

            while (opModeIsActive() && !isStopRequested()) {
                // Our state machine logic
                // You can have multiple switch statements running together for multiple state machines
                // in parallel. This is the basic idea for subsystems and commands.

                // We essentially define the flow of the state machine through this switch statement
                switch (currentLeftRedState) {
                    case Start:

                        if (!drive.isBusy()) {
                            robo.reverseCarousel();
                            sleep(3000);
//                            currentLeftRedState = LeftRed.StoragePark;
                            currentLeftRedState = LeftRed.Deposit1;

                        }
                        break;

                    case Deposit1:
                        if (!drive.isBusy()) {
                            drive.followTrajectoryAsync(Deposit1);
                            robo.carousel.setPower(0);
                            currentLeftRedState = LeftRed.Deposit2;

                        }

                        break;

                    case Deposit2:

                        if (!drive.isBusy()) {
                            drive.followTrajectoryAsync(deposit);
                            currentLeftRedState = LeftRed.Deposit;

                        }

                        break;

                    case Deposit:

                        if (!drive.isBusy()) {
                            // move slides to deposit item
                            robo.autoDepositSlides();
                            sleep(1000);
                            currentLeftRedState = LeftRed.MineralPark1;

                        }

                        break;


                    case MineralPark1:

                        if (!drive.isBusy()) {
                            robo.autoLowerSlidesALittle();
                            drive.followTrajectoryAsync(parkInMineral1);
                            currentLeftRedState = LeftRed.MineralParkFinal;
                        }

                        break;

//
                    case MineralParkFinal:

                        if (!drive.isBusy()) {
                            drive.followTrajectoryAsync(finalParkInMineral);
                            currentLeftRedState = LeftRed.Park;
                        }


                        break;

                    case Park:
                        robo.autoLowerSlides();
                        break;


                }

                // Anything outside of the switch statement will run independent of the currentState
                drive.update();

                telemetry.addData("State", currentLeftRedState);
                telemetry.update();
            }


        }
    }

}