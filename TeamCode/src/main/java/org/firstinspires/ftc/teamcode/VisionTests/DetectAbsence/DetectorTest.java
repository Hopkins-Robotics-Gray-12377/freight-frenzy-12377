package org.firstinspires.ftc.teamcode.VisionTests.DetectAbsence;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.VisionTests.PinkExample.ContourPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name="Detector Test", group="test")
@Disabled
public class DetectorTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Detector detector = new Detector(hardwareMap);
        telemetry.addData("initializing", "done");
        telemetry.update();

        // OpenCV webcam
        OpenCvCamera webcam;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //OpenCV Pipeline


        // Webcam Streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }

        });

        // Only if you are using ftcdashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        FtcDashboard.getInstance().startCameraStream(webcam, 10);


        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                detector.loadImage();
            }
            // Default position is NONE so make sure you click 'A' to get the actual position.
            Detector.ElementPosition pos = detector.getElementPosition();
            telemetry.addData("seen objects", pos == Detector.ElementPosition.LEFT ? "left" :
                    (pos == Detector.ElementPosition.RIGHT ? "right" : "none"));
            telemetry.addData("test", "value");
            telemetry.update();
        }
    }
}

