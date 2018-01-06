package org.firstinspires.ftc.teamcode.Code9161_2017;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by jreynoldsstudent on 1/4/2018.
 */
@Autonomous(name="RedColorTester", group="Linear Opmode")
public class RedColorTester extends LinearOpMode{
    Scorpion r = new Scorpion();
    @Override
    public void runOpMode() {
        r.initRobot(hardwareMap, telemetry);
        waitForStart();
        //r.pushJewel("Red");
    }
}