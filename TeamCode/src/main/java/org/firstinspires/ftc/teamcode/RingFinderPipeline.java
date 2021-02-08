package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class RingFinderPipeline extends OpenCvPipeline {
    private Mat workingMatrix = new Mat();
    private ArrayList<Mat> boxes = new ArrayList<Mat>();
    public int[] positions = {0, 0}; // up to two rings to get from a position

    private int numBoxes = 7;
    public double[] threshholds = new double[numBoxes]; // the threshholds for each box

    private int boxWidth = 68;
    private int boxHeight = 110;
    private int boxArea = boxWidth*boxHeight;
    private int startX = 2;
    private int boxY = 460;

    private double isRingThreshhold = 110;
    public String s = "nothing so far";

    //public RingFinderPipeline() {}


    @Override
    public final Mat processFrame(Mat input) {
        input.copyTo(workingMatrix);

        if (workingMatrix.empty()) {
            return input;
        }
        s = "";
        Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_RGB2YCrCb);

        // Sets up boxes for looking around the field
        boxes.clear();
        for (int i = 0; i < numBoxes; i++){
            // workingMatrix.submat(rowStart, rowEnd, colStart, colEnd) row is y col is x
            int colStart = startX+(boxWidth*i);
            int colEnd = colStart + boxWidth;
            int rowStart = boxY;
            int rowEnd = boxY + boxHeight;
            try {
                Mat newMat = workingMatrix.submat(rowStart, rowEnd, colStart, colEnd);
                boxes.add(newMat);
            } catch (Exception e) {
                this.s += "Bad "+i;
            }
            Imgproc.rectangle(workingMatrix, new Rect(colStart, rowStart, boxWidth, boxHeight), new Scalar(0, 255, 0), 2);
        }

        int count = 0, bestPosition = -1, secondBestPosition = -1;
        double bestPositionValue = 125, secondBestPositionValue = 125;
        double boxValue;

        // Loop through every box where the robot looks for rings and find the places where there are most likely rings
        // The value we're looking at is lowest in the presence of rings
        for (Mat box : this.boxes) {
            boxValue = Core.sumElems(box).val[2]/boxArea;
            threshholds[count] = boxValue;
            if (boxValue < isRingThreshhold){
                if (boxValue < bestPositionValue){
                    // If the best position has been surpassed, what used to be best is now second best
                    secondBestPositionValue = bestPositionValue;
                    secondBestPosition = bestPosition;
                    // save new best position
                    bestPositionValue = boxValue;
                    bestPosition = count;

                } else if (boxValue < secondBestPositionValue){
                    // if the probability isn't a new best, but is better than the current second best, save it as the second best.
                    secondBestPositionValue = boxValue;
                    secondBestPosition = count;
                }
                // because a ring can fill two boxes, don't allow two adjacent boxes to be selected.
                //if (Math.abs(secondBestPosition-bestPosition) <= 1) {
                //    secondBestPosition = -1;
                //    secondBestPositionValue = 125;
                //}
            }
            count++;
        }

        // determine if there are positions for the robot to go get rings
        positions[0] = bestPosition;
        positions[1] = secondBestPosition;

        return workingMatrix;
    }
}