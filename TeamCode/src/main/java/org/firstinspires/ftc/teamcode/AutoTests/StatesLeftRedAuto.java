package org.firstinspires.ftc.teamcode.AutoTests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.VisionTests.GobiidaeVision.Gobiidae;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="AStatesLeftRedAuto")
public class StatesLeftRedAuto extends LinearOpMode {

    Pose2d startPose = new Pose2d(-34, -62, Math.toRadians(-90));

//    Pose2d startDepositInter = new Pose2d(-25, -60, Math.toRadians(-90));
    Pose2d depositPose = new Pose2d(-15, -43, Math.toRadians(-90));

    Pose2d duck1Pose = new Pose2d(-25, -60, Math.toRadians(-90));
    Pose2d carouselPose1 = new Pose2d(-25, -60, Math.toRadians(90));
    Pose2d carouselPose2 = new Pose2d(-58, -58, Math.toRadians(90));

    Pose2d storagePark = new Pose2d(-62, -36, Math.toRadians(90));


    Pose2d topDepositPose = new Pose2d(-15, -45, Math.toRadians(-90));
    Pose2d middleDepositPose = new Pose2d(-15, -45, Math.toRadians(-90));
    Pose2d lowDepositPose = new Pose2d(-15, -45, Math.toRadians(-90));


    Pose2d startDepositInter = new Pose2d(-25, -60, Math.toRadians(-90));



    public ElapsedTime launchTime = new ElapsedTime();

    enum RedDepot {
        Start,
        StartDepositInter,
        Deposit,
        StoragePark,
        MineralPark1,
        MineralPark2,
        MineralParkFinal,
        Duck1,
        Duck2,
        Carousel,
        Park
    }


    @Override
    public void runOpMode() throws InterruptedException {
        Bot robo = new Bot();
        robo.autoInit(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);


        // Left Red
        Trajectory startDepositInter = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-25, -60, Math.toRadians(-90)))
                .build();

        Trajectory Deposit = drive.trajectoryBuilder(new Pose2d(-25, -60, Math.toRadians(-90)))
                .lineToSplineHeading(depositPose)
                .build();

        Trajectory Duck1 = drive.trajectoryBuilder(depositPose)
                .lineToSplineHeading(carouselPose1)
                .build();

        Trajectory Duck2 = drive.trajectoryBuilder(carouselPose1)
                .lineToSplineHeading(carouselPose2)
                .build();

        Trajectory StoragePark = drive.trajectoryBuilder(carouselPose2)
                .lineToSplineHeading(storagePark)
                .build();



        Gobiidae pipeline;
        pipeline = new Gobiidae();
        pipeline.setup(hardwareMap);

        sleep(2000);

        while (!opModeIsActive()) {

            telemetry.update();
        }


        ElapsedTime init = new ElapsedTime();

        init.reset();


        waitForStart();

        if (isStopRequested()) return;

        if (opModeIsActive()) {

            int results = pipeline.getAnalysis();

            sleep(1000);

            RedDepot currState = RedDepot.Start;
            drive.followTrajectoryAsync(startDepositInter);

            while (opModeIsActive() && !isStopRequested()) {
                // Our state machine logic
                // You can have multiple switch statements running together for multiple state machines
                // in parallel. This is the basic idea for subsystems and commands.

                // We essentially define the flow of the state machine through this switch statement
                switch (currState) {
                    case Start:

                        if (!drive.isBusy()) {
                            // move slides to deposit item
                            drive.followTrajectoryAsync(Deposit);

                            currState = RedDepot.StartDepositInter;
                        }
                        break;

                    case StartDepositInter:

                        if (!drive.isBusy()) {
                            if (results == 1) {
                                robo.lowDeposit();
                            } else if (results == 2) {
                                robo.middleDeposit();
                            } else {
                                robo.highDeposit();
                            }

                            sleep(3000);

                            currState = RedDepot.Duck1;
                        }

                        break;

                    case Duck1:
                        if (!drive.isBusy()) {
                            robo.autoLowerSlides();
                            drive.followTrajectoryAsync(Duck1);

                            currState = RedDepot.Duck2;
                        }

                        break;
                    case Duck2:

                        if (!drive.isBusy()) {
                            drive.followTrajectoryAsync(Duck2);

                            currState = RedDepot.Carousel;
                        }

                        break;

                    case Carousel:

                        if (!drive.isBusy()) {
                            robo.reverseCarousel();
                            sleep(3000);
                            currState = RedDepot.StoragePark;
                        }

                        break;

                    case StoragePark:

                        if (!drive.isBusy()) {
                            drive.followTrajectoryAsync(StoragePark);

                            currState = RedDepot.Park;
                        }

                        break;

                    case Park:

                        break;




                }

                // Anything outside of the switch statement will run independent of the currentState
                drive.update();

                telemetry.addData("State", currState);
                telemetry.update();
            }


        }
    }
}
