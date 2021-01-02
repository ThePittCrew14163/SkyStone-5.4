package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Scanner;

@TeleOp(name="Ultimate Goal Drive")
public class UltimateGoalDrive2 extends LinearOpMode {
    public UltimateGoalRobot robot = new UltimateGoalRobot();
    double robotAngle = 0;  // heading (in degrees) robot is to maintain. is set by gamepad1.right_stick.
    double leftStickAngle = 0;
    double theta;  // difference between robot heading and the direction it should travel towards.
    double leftStickR = 0; // distance from 0-1 from gamepad1.left_stick center to edge. is used to set power level to drive train motors.
    double xWheelsPower; // wheel 2 and 3 power
    double yWheelsPower; // wheel 1 and 4 power
    double speed;  // speed adjustment for robot to turn towards robotAngle.
    double difference;
    double sign;

    int flicker_delay = 250; // delay in ms
    int last_flick = (int)System.currentTimeMillis();
    int intake_switch_delay = 500; // delay in ms
    int last_intake_switch = (int)System.currentTimeMillis();
    boolean power_flywheel = false; // whether or not the flywheel should be on.
    boolean turret_idle = true; // true= have the turret ready for accepting rings, false= aim the turret to shoot.
    public boolean run_intake = false;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, this);
        double adjustAngle = 0;
        // adjustAngle = this.get_coordinates(); // doesn't work. yet.

        //Start robot with front just up against launch zone line and side against blue wall, intake facing the target.
        robot.odometer.x = 9;
        robot.odometer.y = 73;

        if (Math.abs(gamepad2.right_stick_x) + Math.abs(gamepad2.right_stick_y) > 0.2) {
            adjustAngle = Math.atan2(gamepad2.right_stick_x, -gamepad2.right_stick_y) + Math.PI/2;
        }
        robotAngle = -(adjustAngle * 180) / Math.PI;
        telemetry.addData("Status", "Initialized. Please do something already.");
        telemetry.addData("adjustAngle", (adjustAngle * 180) / Math.PI);
        telemetry.update();

        // Wait for the game to start

        waitForStart();
        robot.flicker.setPosition(robot.FLICKER_STANDBY);
        robot.wrist.setPosition(1);
        robot.intakeBar.setPosition(0);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // #######################################################
            //  ###### CONTROLS TO MAKE THE DRIVE TRAIN MOVE. ######
            robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            // now use right stick input to get the robot's heading. the if statement ensures that the joystick is a distance away from the center where readings will be accurate.
            if (Math.abs(gamepad1.right_stick_x) + Math.abs(gamepad1.right_stick_y) > 0.6) {
                robotAngle = (-Math.atan2(gamepad1.right_stick_x, -gamepad1.right_stick_y) * 180 / Math.PI)-90;
                if (robotAngle <= -180) {
                    robotAngle += 360;
                }
                if (robotAngle >= 180) {
                    robotAngle -= 360;
                }
            }
            if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0) {
                leftStickAngle = -Math.atan2(gamepad1.left_stick_x, -gamepad1.left_stick_y) + (Math.PI * 5/ 4);
                if (leftStickAngle >= Math.PI) {
                    leftStickAngle -= Math.PI * 2;
                }
                theta = robotAngle / 180 * Math.PI - leftStickAngle;
                xWheelsPower = Math.cos(theta);
                yWheelsPower = Math.sin(theta);
            } else {
                xWheelsPower = 0;
                yWheelsPower = 0;
            }
            difference = robot.angles.firstAngle - robotAngle - (adjustAngle * 180) / Math.PI;
            if (Math.abs(difference) > 180) {
                if (difference < 0) {
                    sign = -1;
                } else {
                    sign = 1;
                }
                difference = sign * (360 - Math.abs(difference));
                speed = -(difference) / 80;
            } else {
                speed = (difference) / 80;
            }

            leftStickR = Math.sqrt((Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2))) * 1.42;

            // this code here ensures that the robot can turn without having to strafe or sit there making a whining noise.
            double speed1 = speed, speed2 = speed; // speed1 is for the front wheels 1 and 2, and speed2 is for the back wheels 3 and 4.

            if (0.1 <= Math.abs(speed) && Math.abs(speed) < 0.2) {
                // only the back wheels move, meaning that the robot can turn but at a lower speed.
                speed2 = sign * 0.2;
                speed1 = 0;
            } else if (Math.abs(speed) < 0.1) {
                // at a certain threshold you'll get no movement, but the motors will whine. thus, it's best to just stop them.
                speed1 = 0;
                speed2 = 0;
            }
            robot.wheel1.setPower(yWheelsPower * leftStickR + speed1);
            robot.wheel4.setPower(yWheelsPower * leftStickR - speed2);
            robot.wheel2.setPower(xWheelsPower * leftStickR - speed1);
            robot.wheel3.setPower(xWheelsPower * leftStickR + speed2);

            ArrayList<Double> list = robot.odometer.getCurrentCoordinates();
            telemetry.addData("X value:", list.get(1));
            telemetry.addData("Y value:", list.get(2));


            ///////////////// INTAKE CONTROLS gamepad 1 //////////////////
            if (gamepad1.left_bumper && (int)System.currentTimeMillis() - this.last_intake_switch >= intake_switch_delay) {
                run_intake = !run_intake;
                this.last_intake_switch = (int) System.currentTimeMillis();
            }
            if (gamepad1.right_bumper) {
                robot.encoderX.setPower(-1);
            } else if (!run_intake) {
                robot.encoderX.setPower(0);
            } else {
                robot.encoderX.setPower(1);
            }
            if (gamepad1.y) {
                robot.intakeBar.setPosition(1);
            } else if (gamepad1.a) {
                robot.intakeBar.setPosition(0);
            }

            ///////////////// RESET ODOMETRY gamepad 2 //////////////////
            if (gamepad2.left_stick_y > 0.6) {
                robot.odometer.x = 9;
            } else if (gamepad2.left_stick_y < -0.6) {
                robot.odometer.x = 87;
            } else if (gamepad2.left_stick_x > 0.6) {
                robot.odometer.y = 9;
            } else if (gamepad2.left_stick_x < -0.6) {
                robot.odometer.y = 132.5;
            }

            ///////////////// WOBBLE LIFT CONTROLS gamepad 1 //////////////////
            if (gamepad1.left_trigger > 0) {
                robot.wobbleLift.setPower(gamepad1.left_trigger);
            } else if (gamepad1.right_trigger >= 0) {
                robot.wobbleLift.setPower(-gamepad1.right_trigger);
            }
            if (gamepad1.dpad_up) {
                robot.wrist.setPosition(0);
            } else if (gamepad1.dpad_left) {
                robot.wrist.setPosition(0.16);
            } else if (gamepad1.dpad_down) {
                robot.wrist.setPosition(1);
            }
            if (gamepad1.x) {
                robot.claw1.setPosition(0.275);
                robot.claw2.setPosition(0.725);
            } else if (gamepad1.b) {
                robot.claw1.setPosition(0.5);
                robot.claw2.setPosition(0.5);
            }

            //////////////////// TURRET CONTROLS gamepad 2 /////////////////////
            if (gamepad2.right_bumper) {
                power_flywheel = false;
            } else if (gamepad2.left_bumper) {
                power_flywheel = true;
            }
            if (power_flywheel) {
                robot.flywheel.setVelocity(robot.best_flywheel_velocity);
            } else {
                robot.flywheel.setVelocity(0);
            }
            telemetry.addData("RPM", robot.getFlywheelRPM());

            if (gamepad2.left_trigger > 0) {
                robot.flicker.setPosition(1);
                this.last_flick = (int) System.currentTimeMillis();
            }
            if ((int)System.currentTimeMillis() - this.last_flick >= flicker_delay && robot.flicker.getPosition() == 1) {
                robot.flicker.setPosition(robot.FLICKER_STANDBY);
            }

            if (gamepad2.dpad_up) {
                turret_idle = true;
            } else if (gamepad2.dpad_down || gamepad2.x || gamepad2.y || gamepad2.b || gamepad2.a) {
                turret_idle = false;
            }
            if (turret_idle) {
                robot.turretBase.setPosition(0.51);
                robot.turretLevel.setPosition(0.26);
            } else if (gamepad2.x) {
                robot.aim_turret(1);
            } else if (gamepad2.y) {
                robot.aim_turret(2);
            } else if (gamepad2.b) {
                robot.aim_turret(3);
            } else if (gamepad2.a) {
                robot.aim_turret(4);
            } else {
                robot.aim_turret(0);
            }


            ////// UPDATE TELEMETRY //////
            telemetry.update();
        }
    }
    public double get_coordinates() {
        File log = new File("odometry_log.txt");
        double angle;
        try {
            Scanner log_reader = new Scanner(log);
            angle = log_reader.nextDouble();
            robot.odometer.x = log_reader.nextDouble();
            robot.odometer.y = log_reader.nextDouble();
            telemetry.addData("Success!", "");
            telemetry.update();
            sleep(2500);
        } catch (IOException e) {
            angle = 0;
            telemetry.addData("FAILURE!", "");
            telemetry.update();
            sleep(2500);
        }
        return angle;
    }
}
