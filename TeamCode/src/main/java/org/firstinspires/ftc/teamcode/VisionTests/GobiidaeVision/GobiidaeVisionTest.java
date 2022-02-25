package org.firstinspires.ftc.teamcode.VisionTests.GobiidaeVision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.ToggleMap;
import org.firstinspires.ftc.teamcode.UseMap;
import org.firstinspires.ftc.teamcode.VisionTests.PinkExample.ContourPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name="VisionTest", group="main")
public class GobiidaeVisionTest extends LinearOpMode {
    Bot robo = new Bot();


    private ElapsedTime runtime = new ElapsedTime();
    public ElapsedTime launchTime = new ElapsedTime();


    @Override
    public void runOpMode() {

        Gobiidae pipeline;
        pipeline = new Gobiidae();
        pipeline.setup(hardwareMap);


//        OpenCvCamera webcam;
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        //OpenCV Pipeline
//
//        pipeline = new Gobiidae();
//
//        webcam.setPipeline(pipeline.pipeline);
//
//        // Webcam Streaming
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//
//            }
//
//        });
//
//
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        telemetry = dashboard.getTelemetry();
//        FtcDashboard.getInstance().startCameraStream(webcam, 10);


        sleep(2000);

        while (!opModeIsActive()) {

            telemetry.update();
        }


        waitForStart();


        while (opModeIsActive()) {


            int results = pipeline.getAnalysis();

            telemetry.addData("test", "value");

            if (results == 3)
                telemetry.addData("Position", "3");
            else if (results == 2)
                telemetry.addData("Position", "2");
            else if (results == 1)
                telemetry.addData("Position", "1");
            else
                telemetry.addData("Position", "0");

            telemetry.addData("Avg 1: ", pipeline.getAvg1());
            telemetry.addData("Avg 2: ", pipeline.getAvg2());

            telemetry.update();
        }
    }

}