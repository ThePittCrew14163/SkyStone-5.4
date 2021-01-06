/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name="Ultimate Goal Best Solo")
public class UltimateGoalAuto4 extends LinearOpMode {
    // NOTE: THE ROBOT STARTS ON THE FAR BLUE LINE WITH ITS MIDDLE LEFT CHASSIS EXTRUSION OVER THE LINE.
    // NOTE: EXTRUSION THAT HOLDS WOBBLE GOAL GRABBER WRIST LINES UP AT THE SAME HEIGHT WITH PIECE OF YELLOW TAPE THAT JOSH PUT THERE.
    // NOTE: MOST OF THE VISION CODE WAS COPIED FROM 9794 WIZARDS.EXE.
    UltimateGoalRobot robot = new UltimateGoalRobot();
    OpenCvInternalCamera phoneCam;
    RingDeterminationPipeline pipeline;
    private RingPosition ringPosition;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, this);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new RingDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // supported camera resolutions (for the gold E4) are: 1280x720, 960x720, 768x432, 720x480, 640x480, 320x240, 176x144
                phoneCam.startStreaming(720, 480, OpenCvCameraRotation.UPSIDE_DOWN);
            }
        });

        // initialize servo positions
        robot.odometer.x = 27;
        robot.odometer.y = 9;
        robot.flicker.setPosition(robot.FLICKER_STANDBY);
        robot.wobbleRelease.setPosition(0.4);
        robot.wrist.setPosition(1);
        robot.claw1.setPosition(0.4);
        robot.claw2.setPosition(0.82);
        robot.intakeBar.setPosition(1);

        while (!isStarted() && !isStopRequested()) {
            this.ringPosition = pipeline.position;
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
        robot.odStrafe(0, 1, 22, 18, 5, 140);
        robot.turretLevel.setPosition(0.5);

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////// POSITION A (NO RINGS) 80pt /////////////////////////////////////////////////////////////////////////
        if (this.ringPosition == RingPosition.NONE) {

            /////////////// PLACE FIRST WOBBLE GOAL /////////////////////////////////////////////////
            robot.odStrafe(-5, 1, 23, 63, 6, 80);
            robot.odStrafe(0, 0.42, 23, 80, 3);
            robot.wobbleRelease.setPosition(0.8);
            robot.odometer.odSleep(350);

            //////////////////////// HIT POWER SHOTS ////////////////////////////////////////////////
            robot.flywheel.setVelocity(robot.best_flywheel_velocity);
            robot.odStrafe(0, 1, 35, 66, 8);
            robot.odStrafe(0, 0.5, 44, 66, 4);
            robot.wobbleRelease.setPosition(0.4);
            robot.wrist.setPosition(0.5);
            robot.odTurn(170, 1, 1300);

            /////// POWER SHOT 1 ///////
            robot.aim_turret(3);
            robot.odometer.odSleep(400);
            robot.flicker.setPosition(1);
            robot.odometer.odSleep(220);
            robot.flicker.setPosition(robot.FLICKER_STANDBY);

            /////// POWER SHOT 2 ///////
            robot.aim_turret(2);
            robot.odometer.odSleep(400);
            robot.flicker.setPosition(1);
            robot.odometer.odSleep(220);
            robot.flicker.setPosition(robot.FLICKER_STANDBY);

            /////// POWER SHOT 3 ///////
            robot.aim_turret(1);
            robot.odometer.odSleep(400);
            robot.flicker.setPosition(1);
            robot.odometer.odSleep(220);
            robot.flicker.setPosition(robot.FLICKER_STANDBY);

            robot.odometer.odSleep(200);
            robot.flywheel.setVelocity(0);

            //////////////////// GRAB SECOND WOBBLE GOAL //////////////////////////////////////////////
            robot.wrist.setPosition(0);
            robot.odStrafe(135, 1, 45, 60, 8, 120);
            robot.motorTurnNoReset(1, 2600, robot.wobbleLift);
            robot.claw1.setPosition(0.5);
            robot.claw2.setPosition(0.5);
            robot.odometer.odSleep(700);
            robot.odStrafe(90, 0.8, 47, 50, 8, 100);
            robot.odStrafe(90, 0.7, 48, 37, 4, 50, 2300);
            robot.claw1.setPosition(0.265);
            robot.claw2.setPosition(0.735);
            robot.odometer.odSleep(500);
            robot.motorTurnNoReset(1, 1000, robot.wobbleLift);
            robot.wrist.setPosition(0.05);

            /////////////// PLACE SECOND WOBBLE GOAL //////////////////////////////////////////////////////
            robot.odStrafe(90, 1, 48, 68, 9);
            robot.odStrafe(20, 1, 44, 87, 9, 160);
            robot.odStrafe(0, 0.5, 30, 88, 3);
            robot.claw1.setPosition(0.5);
            robot.claw2.setPosition(0.5);
            robot.wrist.setPosition(0.3);
            robot.odStrafe(-35, 0.6, 35, 73, 5);
        }
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////// POSITION B (ONE RING) 92pt /////////////////////////////////////////////////////////////////////////
        else if (this.ringPosition == RingPosition.ONE) {

            /////////////// PLACE FIRST WOBBLE GOAL /////////////////////////////////////////////////
            robot.odStrafe(-5, 1, 24, 66, 6, 80);
            robot.odStrafe(-30, 1, 36, 89, 10, 200);
            robot.odStrafe(-50, 0.5, 43, 94, 3, 150);
            robot.wobbleRelease.setPosition(0.8);
            robot.odometer.odSleep(350);

            /////////////////////////// HIT POWER SHOTS ////////////////////////////////////////////////
            robot.flywheel.setVelocity(robot.best_flywheel_velocity);
            robot.wobbleRelease.setPosition(0.4);
            robot.wrist.setPosition(0.5);
            robot.odStrafe(-60, 1, 39, 80, 8, 150);
            robot.odStrafe(-70, 0.5, 38, 66, 3, 100);
            robot.odTurn(170, 1, 1200);

            /////// POWER SHOT 1 ///////
            robot.aim_turret(3);
            robot.odometer.odSleep(400);
            robot.flicker.setPosition(1);
            robot.odometer.odSleep(220);
            robot.flicker.setPosition(robot.FLICKER_STANDBY);

            /////// POWER SHOT 2 ///////
            robot.aim_turret(2);
            robot.odometer.odSleep(400);
            robot.flicker.setPosition(1);
            robot.odometer.odSleep(220);
            robot.flicker.setPosition(robot.FLICKER_STANDBY);

            /////// POWER SHOT 3 ///////
            robot.aim_turret(1);
            robot.odometer.odSleep(400);
            robot.flicker.setPosition(1);
            robot.odometer.odSleep(220);
            robot.flicker.setPosition(robot.FLICKER_STANDBY);

            robot.odometer.odSleep(200);
            robot.flywheel.setPower(0);

            ////////////// GRAB SECOND WOBBLE GOAL ///////////////////////////////////////////////////////
            robot.wrist.setPosition(0);
            // FIRST, PICK UP STARTER RING //
            robot.set_turret_reload_position();
            robot.encoderX.setPower(1);
            robot.odStrafe(-180, 1, 35, 49, 6, 120);

            // NOW GO GRAB WOBBLE GOAL //
            robot.motorTurnNoReset(1, 2600, robot.wobbleLift);
            robot.claw1.setPosition(0.5);
            robot.claw2.setPosition(0.5);
            robot.odometer.odSleep(850);
            robot.odStrafe(125, 0.9, 40, 56, 8, 100);
            robot.odStrafe(115, 0.8, 42, 33, 5, 50, 2300);
            robot.claw1.setPosition(0.265);
            robot.claw2.setPosition(0.735);
            robot.odometer.odSleep(500);
            robot.motorTurnNoReset(1, 1000, robot.wobbleLift);
            robot.wrist.setPosition(0.05);

            /////////////// PLACE SECOND WOBBLE GOAL //////////////////////////////////////////////////////
            robot.odStrafe(90, 1, 48, 68, 8);
            // SHOOT RING //
            robot.flywheel.setVelocity(robot.best_flywheel_velocity);
            robot.odStrafe(-180, 0.7, 46, 54, 6, 200);
            robot.aim_turret(-9);
            robot.odometer.odSleep(400);
            robot.flicker.setPosition(1);
            robot.odometer.odSleep(220);
            robot.flicker.setPosition(robot.FLICKER_STANDBY);
            robot.odometer.odSleep(220);
            robot.flywheel.setVelocity(0);

            // PLACE WOBBLE GOAL
            robot.odStrafe(-140, 1, 39, 74, 6, 250);
            robot.odTurn(-100, 0.6, 1600);
            robot.claw1.setPosition(0.5);
            robot.claw2.setPosition(0.5);
            robot.odometer.odSleep(300);
            robot.odStrafe(-120, 0.8, 31, 70, 5, 120);
            robot.odStrafe(0, 0.6, 23, 73, 5);
        }
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////// POSITION C (FOUR RINGS) 116pt /////////////////////////////////////////////////////////////////////////
        else {

            /////////////// PLACE FIRST WOBBLE GOAL /////////////////////////////////////////////////
            robot.odStrafe(-10, 1, 25, 104, 9, 80);
            robot.odStrafe(-5, 0.45, 27, 122, 4);
            robot.wobbleRelease.setPosition(0.8);
            robot.odometer.odSleep(350);
            robot.odStrafe(0, 1, 32, 110, 7);

            //////////////// HIT POWER SHOTS /////////////////////////////////////////////////////////
            robot.flywheel.setVelocity(robot.best_flywheel_velocity);
            robot.wobbleRelease.setPosition(0.4);
            robot.odStrafe(0, 1, 37, 79, 9);
            robot.odStrafe(0, 0.45, 38, 66, 3);
            robot.odTurn(170, 1, 1300);

            /////// POWER SHOT 1 ///////
            robot.aim_turret(3);
            robot.odometer.odSleep(400);
            robot.flicker.setPosition(1);
            robot.odometer.odSleep(220);
            robot.flicker.setPosition(robot.FLICKER_STANDBY);

            /////// POWER SHOT 2 ///////
            robot.aim_turret(2);
            robot.odometer.odSleep(400);
            robot.flicker.setPosition(1);
            robot.odometer.odSleep(220);
            robot.flicker.setPosition(robot.FLICKER_STANDBY);

            robot.intakeBar.setPosition(0.05);

            /////// POWER SHOT 3 ///////
            robot.aim_turret(1);
            robot.odometer.odSleep(400);
            robot.flicker.setPosition(1);
            robot.odometer.odSleep(220);
            robot.flicker.setPosition(robot.FLICKER_STANDBY);

            robot.odometer.odSleep(200);
            robot.flywheel.setVelocity(0);

            ////////////// GRAB SECOND WOBBLE GOAL ///////////////////////////////////////////////////////
            robot.wrist.setPosition(0);
            // FIRST, PICK UP STARTER RINGS //
            robot.set_turret_reload_position();
            robot.encoderX.setPower(1);
            robot.odStrafe(-180, 0.45, 35, 39, 6, 120);

            // NOW GO GRAB WOBBLE GOAL //
            robot.motorTurnNoReset(1, 2600, robot.wobbleLift);
            robot.claw1.setPosition(0.5);
            robot.claw2.setPosition(0.5);
            robot.odometer.odSleep(850);
            robot.odStrafe(125, 0.9, 40, 46, 8, 100);
            robot.odStrafe(115, 0.8, 42, 33, 5, 50, 2300);
            robot.claw1.setPosition(0.265);
            robot.claw2.setPosition(0.735);
            robot.odometer.odSleep(500);
            robot.motorTurnNoReset(1, 1000, robot.wobbleLift);
            robot.wrist.setPosition(0.05);

            // SHOOT RINGS //
            robot.flywheel.setVelocity(robot.best_flywheel_velocity);
            robot.odStrafe(180, 1, 36, 60, 8, 175);
            robot.odometer.odSleep(250);
            for (int i = 0; i < 3; i++) {
                robot.odometer.odSleep(450);
                robot.aim_turret(-9);
                robot.odometer.odSleep(200);
                robot.flicker.setPosition(1);
                robot.odometer.odSleep(220);
                robot.flicker.setPosition(robot.FLICKER_STANDBY);
            }
            robot.flywheel.setVelocity(0);

            /////////////// PLACE SECOND WOBBLE GOAL /////////////////////////////////////////////////////
            robot.odTurn(0, 1, 1200);
            robot.odStrafe(0, 1, 30, 110, 9);
            robot.odStrafe(0, 0.5, 27, 129, 4);
            robot.claw1.setPosition(0.5);
            robot.claw2.setPosition(0.5);
            robot.odStrafe(0, 0.8, 36, 117, 6);
            robot.motorTurnNoReset(1, -400, robot.wobbleLift);
            robot.wrist.setPosition(0.3);
            robot.odStrafe(0, 1, 16, 82, 8);
        }


        //////////////////////// PARK /////////////////////////////////////////////////////////////////
        robot.motorTurnNoReset(1, -400, robot.wobbleLift);
        robot.wrist.setPosition(1);
        robot.claw1.setPosition(0.5);
        robot.claw2.setPosition(0.5);
        robot.odStrafe(0, 0.64, 9, 73, 1, 50, 3000);
    }



    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////// VISION SOFTWARE STUFF //////////////////////////////////////////////////////////////////////////////////////////////////
    public static class RingDeterminationPipeline extends OpenCvPipeline {
        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(243, 425);

        static final int REGION_WIDTH = 100;
        static final int REGION_HEIGHT = 100;

        final int FOUR_RING_THRESHOLD = 143;
        final int ONE_RING_THRESHOLD = 133;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile RingPosition position = RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingPosition.FOUR; // Record our analysis
            if (avg1 > FOUR_RING_THRESHOLD) {
                position = RingPosition.FOUR;
            } else if (avg1 > ONE_RING_THRESHOLD) {
                position = RingPosition.ONE;
            } else {
                position = RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis() {
            return avg1;
        }


    }

    /*
     * An enum to define the ring position
     */
    public enum RingPosition {
        FOUR,
        ONE,
        NONE
    }
}
