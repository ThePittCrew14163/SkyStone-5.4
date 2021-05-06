/*
Copyright 2019 FIRST Tech Challenge Team 14163

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
//@Disabled
@Autonomous(name = "Skystone: Everything Solo", group = "Skystone Official")

public class Skystone4StoneAuto extends LinearOpMode {
    private SkystoneRobot2 robot = new SkystoneRobot2();
    private OpenCvInternalCamera phoneCam;
    private SkystoneDetector detector;
    private String position;
    @Override
    public void runOpMode() {
        robot.init(hardwareMap, this);
        robot.odometer.x = -9;
        robot.odometer.y = 33;
        detector = new SkystoneDetector();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();

        phoneCam.setPipeline(detector);
        // supported camera resolutions (for the gold E4) are: 1280x720, 960x720, 768x432, 720x480, 640x480, 320x240, 176x144
        phoneCam.startStreaming(720, 480, OpenCvCameraRotation.UPSIDE_DOWN);

        robot.autoclaw.setPosition(0);
        robot.wrist.setPosition(0.8);
        robot.foundationGrabberL.setPosition(1);
        robot.foundationGrabberR.setPosition(0);

        while (!isStarted()) {
            position = detector.position;
            telemetry.addData("position", position);
            telemetry.update();
        }

        if (position.equals("CENTER")) {
            robot.AutoGrabStoneRedSide(2, true);
            robot.AutoPlaceStoneRedSide(24);
            GoBackUnderRedBridge();
            robot.AutoGrabStoneRedSide(5);
            robot.AutoPlaceStoneRedSide(16);
            GoBackUnderRedBridge();
            robot.AutoGrabStoneRedSide(6);
            robot.AutoPlaceStoneRedSide(8);
            GoBackUnderRedBridge(-2);
            robot.AutoGrabStoneRedSide(4);
            robot.AutoPlaceStoneRedSide(12);
        } else if (position.equals("RIGHT")) {
            robot.AutoGrabStoneRedSide(3, true);
            robot.AutoPlaceStoneRedSide(24);
            GoBackUnderRedBridge();
            robot.AutoGrabStoneRedSide(6);
            robot.AutoPlaceStoneRedSide(16);
            GoBackUnderRedBridge();
            robot.AutoGrabStoneRedSide(5);
            robot.AutoPlaceStoneRedSide(8);
            GoBackUnderRedBridge(-2);
            robot.AutoGrabStoneRedSide(4);
            robot.AutoPlaceStoneRedSide(12);
        } else {
            robot.AutoGrabStoneRedSide(1, true);
            robot.AutoPlaceStoneRedSide(24);
            GoBackUnderRedBridge();
            robot.AutoGrabStoneRedSide(4);
            robot.AutoPlaceStoneRedSide(16);
            GoBackUnderRedBridge();
            robot.AutoGrabStoneRedSide(6);
            robot.AutoPlaceStoneRedSide(8);
            GoBackUnderRedBridge(-2);
            robot.AutoGrabStoneRedSide(5);
            robot.AutoPlaceStoneRedSide(12);
        }

        // move foundation
        robot.lift2.setPower(0.6);
        robot.lift3.setPower(0.6);
        robot.odTurn(90, 0.8, 1000);
        robot.lift2.setPower(0);
        robot.lift3.setPower(0);
        robot.odStrafe(90, 0.45, -42, 116, 3, 0.01, 2500);
        robot.foundationGrabberL.setPosition(0.02);
        robot.foundationGrabberR.setPosition(0.98);
        robot.odometer.odSleep(500);

        robot.odStrafe(80, 1, -15, 110, 4, 0.001, 3500);
        robot.odTurn(-20, 2, 2500);
        robot.foundationGrabberL.setPosition(1);
        robot.foundationGrabberR.setPosition(0);
        robot.odStrafe(0, 1, -15, 125, 5, 0.02, 750);
        // park
        robot.odStrafe(0, 0.6, -20, 80, 6);
    }
    public void GoBackUnderRedBridge() {
        GoBackUnderRedBridge(0);
    }
    public void GoBackUnderRedBridge(int offset){
        robot.odStrafe(0, 1, -26, 105, 8);
        robot.odStrafe(0, 1, -29+offset, 76, 10);
    }
}