package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class WRLAuto extends LinearOpMode
{
    WRLRobot robot = new WRLRobot();

    @Override
    public void runOpMode()
    {
        robot.init(hardwareMap, this);
        robot.odometer.x = 6;
        robot.odometer.y = 6;

        telemetry.addData("Do something!!!", "This is another string. I dunno man.");
        telemetry.update();

        waitForStart();

        robot.odStrafe(0, 0.3, 8, 20.5, 1, 0.02, 4000);
        robot.odTurn(-88, 1, 1500);
        robot.odTurn(-88, 0.21, 2000);
        robot.motorTurnNoReset(0.5, -1400, robot.elbow);
        robot.intake.setPower(1);
        robot.odSleep(1500);
        robot.odTurn(-120, 0.5, 2000);

        for (int i = 1; i <= 3; i++)  {  //this is a loop remember how to use it, this is loop for getting first 3 blocks with handle
            robot.motorTurnNoReset(0.5, 0, robot.elbow);
            robot.odSleep(1500);

            if (i >= 3){break;}
            robot.odStrafe(-90, 0.3, 8 + i*15, 21, 2, 0.01, 4000);

            robot.motorTurnNoReset(0.5, -1400, robot.elbow);
            robot.intake.setPower(1);

            telemetry.addData("IF I WORK GOOD JOB!!! IF I DON'T GO FIX ME!!!", i);
            telemetry.update();

            robot.odSleep(1500);

        }

        //turns the robot so that it picks up last block with handle
        robot.odStrafe(-90,0.3,47, 24.5, 2, 0.01, 4000);
        robot.odTurn(-120,1,1500);
        robot.odTurn(-120,0.18 ,1000);
        robot.motorTurnNoReset(0.5,-1400, robot.elbow);
        robot.intake.setPower(1);
        robot.odSleep(3000);
        robot.motorTurnNoReset(0.5, 0,robot.elbow);
        robot.odSleep(3000);


        //makes robot face right wall and put HANDLE blocks through tiny tiny gap//
        robot.odTurn(-90, 1, 1500);
        robot.odTurn(-90, 0.4, 1100);
        robot.odStrafe(-90,0.3,39.7,25,2,0.02,2400);
        robot.motorTurnNoReset(0.5,-1400,robot.elbow);
        robot.odSleep(2000);
        robot.odStrafe(-90,0.3,40,26.5,1,0.025,2400);

        robot.intake.setPower(-1);
        robot.odSleep(2500);
        robot.motorTurnNoReset(1,-400,robot.elbow);
        robot.odSleep(1500);
        robot.motorTurnNoReset(1,-1400,robot.elbow);
        robot.odSleep(2000);
        robot.motorTurnNoReset(0.5,0,robot.elbow);
        robot.odSleep(2500);

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //picking up blocks without handles//
        robot.odStrafe(-90,0.3,42.5, 24, 2, 0.01, 4000);

        //twists robot to get the block in the right corner//
        robot.odTurn(-112,1,1500);
        robot.odTurn(-112,0.15,2000);
        robot.motorTurnNoReset(0.5,-1400, robot.elbow);
        robot.intake.setPower(1);
        robot.odSleep(2000);
        robot.odTurn(-160, 0.4, 2000);

        // lifts intake to turn and eat other s-blocks
        robot.motorTurnNoReset(0.5, -400,robot.elbow);
        robot.odSleep(2500);
        robot.odStrafe(-145,0.3,47.4,28,2,0.01,2500);
        robot.odTurn(-35,0.5,1000);
        robot.odTurn(-35,0.3,2000);
        //gets the rest of the blocks //
        robot.motorTurnNoReset(0.5,-1400,robot.elbow);
        robot.odSleep(1500);
        robot.odTurn(30, 0.6, 2000);

        robot.odStrafe(-15, 0.3, 30.5, 25, 2, 0.01, 4000);
        robot.odTurn(-45, 0.5, 1500);
        robot.odStrafe(-35, 0.25, 13.5, 26, 2, 0.01, 3000);
        robot.odTurn(5, 0.5, 1500);
        robot.motorTurnNoReset(0.6,-500,robot.elbow);
        robot.odSleep(3000);

        //parks and spits the last blocks//
        robot.odStrafe(-42,0.5,18,18,4,0.01,5000);
        robot.odTurn(140,1,1000);
        robot.odTurn(140,0.25,2000);
        robot.motorTurnNoReset(0.5,-700,robot.elbow);
        robot.odSleep(1500);
        for (int t = 1; t <= 1; t++) {
            robot.intake.setPower(-1);
            robot.odSleep(1500);
            robot.motorTurnNoReset(0.7,-1000,robot.elbow);
            robot.odSleep(1500);
            robot.motorTurnNoReset(0.7,-1400,robot.elbow);
        }
        robot.odSleep(1500);
        robot.motorTurnNoReset(0.5,0,robot.elbow);
        robot.odSleep(1000);
        robot.odStrafe(-90,0.6,15,15,1,0.01,3000);
        robot.odStrafe(-90,0.4,11,11,1,0.01,3000);
        robot.odStrafe(-90,0.3,8,8,1,0.01,2000);
        robot.odStrafe(-90,0.3,6,6,1,0.01,2000);
        robot.odStrafe(-90,0.4,robot.odometer.x+0.3,3,1,0.01,2000);
        robot.odStrafe(-90,0.4,3,robot.odometer.y+0.25,1,0.01,3000);
    }
}