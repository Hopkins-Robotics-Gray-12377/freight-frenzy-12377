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
public class LeftRedStorage extends LinearOpMode {

//    Pose2d startPose = new Pose2d(-62.0, -18.0, 0.0);


    Pose2d startPose = new Pose2d(-62.5, 34.0, Math.toRadians(90));

    Pose2d depositPose = new Pose2d(-25.0, 28.0, Math.toRadians(90));

    Pose2d carouselPose = new Pose2d(-58.0, 58.0, 0.0);

    Pose2d storagePark = new Pose2d(-36.0, 62.5, 0.0);

    Pose2d mineralPark1 = new Pose2d(0.0, 0.0, Math.toRadians(-90));

    Pose2d mineralPark2 = new Pose2d(-50.0, 0.0, Math.toRadians(-90));

    Pose2d mineralFinalPark = new Pose2d(-50.0, -40.0, Math.toRadians(-90));

    public ElapsedTime launchTime = new ElapsedTime();

    enum LeftRed {
        Start,
        Deposit,
        StoragePark,
        MineralPark1,
        MineralPark2,
        MineralParkFinal,
        Duck,
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
        Trajectory Deposit = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(depositPose)
                .build();

        Trajectory Duck = drive.trajectoryBuilder(Deposit.end())
                .lineToSplineHeading(carouselPose)
                .build();

        Trajectory parkInStorage = drive.trajectoryBuilder(Duck.end())
                .lineToSplineHeading(storagePark)
                .build();

        Trajectory parkInMineral1 = drive.trajectoryBuilder(Duck.end())
                .splineTo(new Vector2d(mineralPark1.getX(), mineralPark1.getY()), mineralPark1.getHeading())
                .build();

        Trajectory parkInMineral2 = drive.trajectoryBuilder(parkInMineral1.end())
                .strafeRight(48)
                .build();

        Trajectory finalParkInMineral = drive.trajectoryBuilder(parkInMineral2.end())
                .forward(40)
                .build();


        ElapsedTime init = new ElapsedTime();

        init.reset();

        waitForStart();

        if (isStopRequested()) return;

        if (opModeIsActive()) {
            currentLeftRedState = LeftRed.Start;
            drive.followTrajectoryAsync(Deposit);

            while (opModeIsActive() && !isStopRequested()) {
                // Our state machine logic
                // You can have multiple switch statements running together for multiple state machines
                // in parallel. This is the basic idea for subsystems and commands.

                // We essentially define the flow of the state machine through this switch statement
                switch (currentLeftRedState) {
                    case Start:

                        if (!drive.isBusy()) {
                            // move slides to deposit item
                            robo.autoDepositSlides();
                            sleep(1000);
                            currentLeftRedState = LeftRed.Deposit;
                        }
                        break;

                    case Deposit:

                        if (!drive.isBusy()) {
                            robo.autoLowerSlides();
                            drive.followTrajectoryAsync(Duck);
                            currentLeftRedState = LeftRed.Duck;
                        }

                        break;

                    case Duck:

                        if (!drive.isBusy()) {
                            robo.moveCarousel();
                            sleep(3000);
                            currentLeftRedState = LeftRed.StoragePark;
//                            currentLeftRedState = LeftRed.MineralPark1;
                        }
                        break;

                    case StoragePark:

                        if (!drive.isBusy()) {
                            drive.followTrajectoryAsync(parkInStorage);
                            robo.carousel.setPower(0);
                            currentLeftRedState = LeftRed.Park;
                        }

                        break;

                    case MineralPark1:

                        if (!drive.isBusy()) {
                            drive.followTrajectoryAsync(parkInMineral1);
                            robo.carousel.setPower(0);
                            currentLeftRedState = LeftRed.MineralPark2;
                        }

                        break;


//
                    case MineralPark2:


                        if (!drive.isBusy()) {
                            drive.followTrajectoryAsync(parkInMineral2);
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
