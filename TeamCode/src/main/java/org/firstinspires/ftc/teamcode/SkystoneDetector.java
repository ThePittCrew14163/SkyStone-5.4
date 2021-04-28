package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SkystoneDetector extends OpenCvPipeline {
    private Mat workingMatrix = new Mat();
    public String position = "LEFT";
    private int leftBoxX = 70;
    private int centerBoxX = 230;
    private int rightBoxX = 370;
    private int boxWidth = 110;
    private int boxHeight = 70;
    private int boxY = 330;
    public SkystoneDetector() {
    }
    public SkystoneDetector(String side) {
    }

    @Override
    public final Mat processFrame(Mat input) {
        input.copyTo(workingMatrix);

        if (workingMatrix.empty()) {
            return input;
        }

        Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_RGB2YCrCb);

        Mat matLeft = workingMatrix.submat(boxY, boxY+boxHeight, leftBoxX, leftBoxX+boxWidth);
        Mat matCenter = workingMatrix.submat(boxY, boxY+boxHeight, centerBoxX, centerBoxX+boxWidth);
        Mat matRight = workingMatrix.submat(boxY, boxY+boxHeight, rightBoxX, rightBoxX+boxWidth);

        Imgproc.rectangle(workingMatrix, new Rect(leftBoxX, boxY, boxWidth, boxHeight), new Scalar(0,255,0));
        Imgproc.rectangle(workingMatrix, new Rect(centerBoxX, boxY, boxWidth, boxHeight), new Scalar(0,255,0));
        Imgproc.rectangle(workingMatrix, new Rect(rightBoxX, boxY, boxWidth, boxHeight), new Scalar(0,255,0));

        double leftTotal = Core.sumElems(matLeft).val[2];
        double centerTotal = Core.sumElems(matCenter).val[2];
        double rightTotal = Core.sumElems(matRight).val[2];

        if (leftTotal > centerTotal) {
            if (leftTotal > rightTotal) {
                // left is skystone
                position = "LEFT";
            } else {
                // right is skystone
                position = "RIGHT";
            }
        } else {
            if (centerTotal > rightTotal) {
                // center is skystone
                position = "CENTER";
            } else {
                // right is skystone
                position = "RIGHT";
            }
        }

        return workingMatrix;
    }
}