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

    private int numBoxes = 9;
    public double[] threshholds = new double[numBoxes]; // the threshholds for each box

    private int boxWidth = 50;
    private int boxHeight = 100;
    private int boxArea = boxWidth*boxHeight;
    private int startX = 29;
    private int boxY = 470;

    private double isRingThreshhold = 105;
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

        int count = 0, bestPosition = 0, secondBestPosition = 0;
        double bestPositionValue = 100, secondBestPositionValue = 100;
        double boxValue;

        // Loop through every box where the robot looks for rings and find the places where there are most likely rings
        // The value we're looking at is lowest in the presence of rings
        for (Mat box : this.boxes) {
            boxValue = Core.sumElems(box).val[2]/boxArea;
            threshholds[count] = boxValue;
            if (boxValue < isRingThreshhold){
                if (boxValue < bestPositionValue){
                    secondBestPositionValue = bestPositionValue;
                    secondBestPosition = bestPosition;
                    bestPositionValue = boxValue;
                    bestPosition = count;
                } else if (boxValue < secondBestPositionValue){
                    secondBestPositionValue = boxValue;
                    secondBestPosition = count;
                }
            }
            count++;
        }

        // determine if there are positions for the robot to go get rings
        if (bestPositionValue > 0){
            positions[0] = bestPosition;
        }
        if (secondBestPositionValue > 0){
            positions[1] = secondBestPosition;
        }

        return workingMatrix;
    }
}