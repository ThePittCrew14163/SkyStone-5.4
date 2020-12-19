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

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

@Autonomous(name="Ultimate Goal Better Solo")
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
        robot.odometer.x = 27.5;
        robot.odometer.y = 9;
        robot.flicker.setPosition(robot.FLICKER_STANDBY);
        robot.wobbleRelease.setPosition(0.4);
        robot.wrist.setPosition(1);
        robot.claw1.setPosition(0.3);
        robot.claw2.setPosition(0.7);

        while (!isStarted() && !isStopRequested()) {
            this.ringPosition = pipeline.position;
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
        robot.odStrafe(15, 1, 25, 24, 5, 140);
        robot.turretLevel.setPosition(0.5);

        //////// PLACE FIRST WOBBLE GOAL ////////////
        if (this.ringPosition == RingPosition.NONE) {
            robot.odStrafe(0, 1, 22, 63, 6, 80);
            robot.odStrafe(0, 0.4, 22, 79, 3);
            robot.wobbleRelease.setPosition(0.8);
            robot.odometer.odSleep(350);
            robot.odStrafe(0, 1, 32, 81, 6);

        } else if (this.ringPosition == RingPosition.ONE) {
            robot.odStrafe(0, 1, 22, 66, 6, 80);
            robot.odStrafe(-30, 0.9, 35, 88, 8, 200);
            robot.odStrafe(-50, 0.4, 43, 94, 3, 150);
            robot.wobbleRelease.setPosition(0.8);
            robot.odometer.odSleep(350);
        } else {
            robot.odStrafe(-5, 1, 24, 104, 6, 80);
            robot.odStrafe(0, 0.4, 25, 122, 4);
            robot.wobbleRelease.setPosition(0.8);
            robot.odometer.odSleep(350);
            robot.odStrafe(0, 1, 31, 110, 7);
        }

        ////////// HIT POWER SHOTS ///////////
        robot.flywheel.setVelocity(robot.best_flywheel_velocity);
        robot.wobbleRelease.setPosition(0.4);
        robot.odStrafe(-135, 1, 42, 75, 8, 150);
        robot.odStrafe(-180, 0.4, 51, 68, 3, 100);
        robot.odTurn(180, 0.5, 600);

        robot.aim_turret(3);
        robot.odometer.odSleep(400);
        robot.flicker.setPosition(1);
        robot.odometer.odSleep(220);
        robot.flicker.setPosition(robot.FLICKER_STANDBY);

        //robot.setFlywheelRPM(4500);
        robot.aim_turret(2);
        robot.odometer.odSleep(400);
        robot.flicker.setPosition(1);
        robot.odometer.odSleep(220);
        robot.flicker.setPosition(robot.FLICKER_STANDBY);

        //robot.setFlywheelRPM(4500);
        robot.aim_turret(1);
        robot.odometer.odSleep(400);
        robot.flicker.setPosition(1);
        robot.odometer.odSleep(220);
        robot.flicker.setPosition(robot.FLICKER_STANDBY);

        robot.odometer.odSleep(200);
        robot.flywheel.setPower(0);

        ////////// GRAB SECOND WOBBLE GOAL /////////
        robot.wrist.setPosition(0);
        if (this.ringPosition == RingPosition.NONE) {
            robot.odStrafe(135, 1, 51, 60, 8, 120);
            robot.motorTurnNoReset(1, 2500, robot.wobbleLift);
            robot.claw1.setPosition(0.5);
            robot.claw2.setPosition(0.5);
            robot.odometer.odSleep(700);
            robot.odStrafe(90, 0.8, 51, 48, 8, 100);
            robot.odStrafe(90, 0.6, 51, 33, 5, 50, 2300);
            robot.claw1.setPosition(0.265);
            robot.claw2.setPosition(0.735);
            robot.odometer.odSleep(500);
            robot.motorTurnNoReset(1, 1000, robot.wobbleLift);
            robot.wrist.setPosition(0.05);

        } else if (this.ringPosition == RingPosition.ONE) {
            // FIRST, PICK UP STARTER RING //
            robot.turretBase.setPosition(0.51);
            robot.turretLevel.setPosition(0.35);
            robot.encoderX.setPower(1);
            robot.odStrafe(-180, 1, 31, 48, 8, 120);

            // NOW GO GRAB WOBBLE GOAL //
            robot.motorTurnNoReset(1, 2500, robot.wobbleLift);
            robot.claw1.setPosition(0.5);
            robot.claw2.setPosition(0.5);
            robot.odometer.odSleep(850);
            robot.odStrafe(125, 0.8, 40, 56, 8, 100);
            robot.odStrafe(115, 0.6, 42, 33, 5, 50, 2300);
            robot.claw1.setPosition(0.265);
            robot.claw2.setPosition(0.735);
            robot.odometer.odSleep(500);
            robot.motorTurnNoReset(1, 1000, robot.wobbleLift);
            robot.wrist.setPosition(0.05);

        } else {
            robot.odStrafe(135, 1, 51, 60, 8, 120);
            robot.motorTurnNoReset(1, 2500, robot.wobbleLift);
            robot.claw1.setPosition(0.5);
            robot.claw2.setPosition(0.5);
            robot.odometer.odSleep(700);
            robot.odStrafe(90, 0.8, 51, 48, 8, 100);
            robot.odStrafe(90, 0.6, 51, 33, 5, 50, 2300);
            robot.claw1.setPosition(0.265);
            robot.claw2.setPosition(0.735);
            robot.odometer.odSleep(500);
            robot.motorTurnNoReset(1, 1000, robot.wobbleLift);
            robot.wrist.setPosition(0.05);

        }

        ////////// PLACE SECOND WOBBLE GOAL /////////
        robot.odStrafe(90, 1, 48, 68, 8);
        if (this.ringPosition == RingPosition.NONE) {
            robot.odStrafe(20, 1, 44, 87, 7, 160);
            robot.odStrafe(0, 0.44, 30, 88, 3);
            robot.claw1.setPosition(0.5);
            robot.claw2.setPosition(0.5);
            robot.motorTurnNoReset(1, 0, robot.wobbleLift);
            robot.wrist.setPosition(0.3);
            robot.odStrafe(-35, 0.6, 35, 73, 5);
            robot.wrist.setPosition(1);

        } else if (this.ringPosition == RingPosition.ONE) {
            // SHOOT RING //
            robot.flywheel.setPower(-0.81);
            robot.odStrafe(-180, 0.7, 46, 54, 6, 200);
            robot.aim_turret(-9);
            robot.odometer.odSleep(400);
            robot.flicker.setPosition(1);
            robot.odometer.odSleep(220);
            robot.flicker.setPosition(robot.FLICKER_STANDBY);
            robot.odometer.odSleep(220);
            robot.flywheel.setPower(0);

            // PLACE WOBBLE GOAL
            robot.odStrafe(-140, 1, 39, 74, 6, 250);
            robot.odTurn(-100, 0.6, 1600);
            robot.claw1.setPosition(0.5);
            robot.claw2.setPosition(0.5);
            robot.motorTurnNoReset(1, 0, robot.wobbleLift);
            robot.odometer.odSleep(300);
            robot.odStrafe(-120, 0.8, 31, 70, 5, 120);
            robot.wrist.setPosition(1);
            robot.odStrafe(0, 0.6, 23, 73, 5);
        } else {
            robot.odStrafe(-10, 1, 34, 106, 6, 200);
            robot.odStrafe(0, 0.44, 25, 125, 4);
            robot.claw1.setPosition(0.5);
            robot.claw2.setPosition(0.5);
            robot.motorTurnNoReset(1, 0, robot.wobbleLift);
            robot.odStrafe(0, 0.5, 36, 117);
            robot.wrist.setPosition(1);
            robot.odStrafe(0, 1, 16, 82, 8);
        }
        ////// PARK /////
        robot.claw1.setPosition(0.265);
        robot.claw2.setPosition(0.735);
        robot.odStrafe(0, 0.64, 9, 73, 1, 50, 3000);

        //this.log_coordinates(); // doesn't work. yet.
    }

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

    public void log_coordinates() {
        // Log robot's odometry coordinates after it's done with auto.
        try {
            File new_log = new File("odometry_log.txt");
            boolean never_used = new_log.createNewFile();

            FileWriter log = new FileWriter("odometry_log.txt");
            ArrayList<Double> coordinates = robot.odometer.getCurrentCoordinates();
            log.write(coordinates.get(0).toString()); // log angle
            log.append(coordinates.get(1).toString()); // log x
            log.append(coordinates.get(2).toString()); // log y

            telemetry.addData("Success!", "");
            telemetry.update();
            sleep(2500);
        } catch (IOException e) {
            telemetry.addData("FAILURE!", "");
            telemetry.update();
            sleep(2500);
        }
    }
}
