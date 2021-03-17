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

        robot.odStrafe(0, 0.4, 10, 18, 2, 0.005, 3000);
        robot.odTurn(-90, 1, 500);
        robot.odTurn(-90, 0.15, 1000);

        for (int i = 1; i <= 3; i++)  {  //this is a loop remember how to use it, this is loop for getting first 3 blocks with handle
            robot.motorTurnNoReset(0.5, 100, robot.elbow);
            robot.intake.setPower(1);

            telemetry.addData("Running", i);
            telemetry.update();

            robot.odSleep(1000);
            robot.motorTurnNoReset(0.5, 0, robot.elbow);
            robot.odSleep(1000);

            if (i >= 3){break;}
            robot.odStrafe(-90, 0.4, 10 + i*14, 20, 2, 0.005, 4500);
        }
        //turns the robot so that it picks up last block with handle
        robot.odStrafe(-90,0.4,46, 23, 2, 0.005, 4000);
        robot.odTurn(-130,1,500);
        robot.odTurn(-130,0.15,1000);
        robot.motorTurnNoReset(0.5,100, robot.elbow);
        robot.intake.setPower(1);
        robot.odSleep(500);
        robot.motorTurnNoReset(0.5, 0,robot.elbow);
        robot.odTurn(-90, 1, 500);
        robot.odTurn(-90, 0.25, 1000);
        robot.odStrafe(-90,0.4,43,22,2,0.005,4000);

    }


}