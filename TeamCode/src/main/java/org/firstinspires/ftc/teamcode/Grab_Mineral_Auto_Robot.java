package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous
public class Grab_Mineral_Auto_Robot extends LinearOpMode
{
    public Odometry odometer;
    public Orientation angles;
    public BNO055IMU imu;
    public DcMotorSimple wheel1;
    public DcMotorSimple wheel4;
    public DcMotorSimple wheel2;
    public DcMotorSimple wheel3;


    SkystoneRobot2 robot = new SkystoneRobot2();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap, this);
        robot.odometer.x = 0;
        robot.odometer.y = 0;

    }
}
