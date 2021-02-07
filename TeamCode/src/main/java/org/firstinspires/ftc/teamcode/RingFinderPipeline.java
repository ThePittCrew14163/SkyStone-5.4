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

    private int numBoxes = 15;
    public double[] threshholds = new double[numBoxes]; // the threshholds for each box

    private int boxWidth = 25;
    private int boxHeight = 80;
    private int startX = 100;
    private int boxY = 450;

    private double isRingThreshhold = 135;

    public RingFinderPipeline() {
        // Sets up boxes for looking around the field
        for (int i = 0; i < numBoxes; i++){
            // workingMatrix.submat(rowStart, rowEnd, colStart, colEnd)
            int rowStart = startX+(boxWidth*i);
            int rowEnd = rowStart + boxWidth;
            int colStart = boxY;
            int colEnd = boxY + boxHeight;
            boxes.add(workingMatrix.submat(rowStart, rowEnd, colStart, colEnd));
            Imgproc.rectangle(workingMatrix, new Rect(rowStart, rowEnd, colStart, colEnd), new Scalar(0,255,0), 2);
        }
    }

    @Override
    public final Mat processFrame(Mat input) {
        input.copyTo(workingMatrix);

        if (workingMatrix.empty()) {
            return input;
        }

        Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_RGB2YCrCb);

        //Mat matLeft = workingMatrix.submat(boxY, boxY+boxHeight, leftBoxX, leftBoxX+boxWidth);
        //Mat matCenter = workingMatrix.submat(boxY, boxY+boxHeight, centerBoxX, centerBoxX+boxWidth);
        //Mat matRight = workingMatrix.submat(boxY, boxY+boxHeight, rightBoxX, rightBoxX+boxWidth);

        //Imgproc.rectangle(workingMatrix, new Rect(leftBoxX, boxY, boxWidth, boxHeight), new Scalar(0,255,0));
        //Imgproc.rectangle(workingMatrix, new Rect(centerBoxX, boxY, boxWidth, boxHeight), new Scalar(0,255,0));
        //Imgproc.rectangle(workingMatrix, new Rect(rightBoxX, boxY, boxWidth, boxHeight), new Scalar(0,255,0));

        //double leftTotal = Core.sumElems(matLeft).val[2];
        //double centerTotal = Core.sumElems(matCenter).val[2];
        //double rightTotal = Core.sumElems(matRight).val[2];

        int count = 0, bestPosition = 0, secondBestPosition = 0;
        double bestPositionValue = 0, secondBestPositionValue = 0;
        double boxValue;

        // Loop through every box where the robot looks for rings and find the places where there are most likely rings
        for (Mat box : this.boxes) {
            boxValue = Core.sumElems(box).val[2];
            threshholds[count] = boxValue;
            if (boxValue > isRingThreshhold){
                if (boxValue > bestPositionValue){
                    secondBestPositionValue = bestPositionValue;
                    bestPositionValue = boxValue;
                    bestPosition = count;
                } else if (boxValue > secondBestPositionValue){
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