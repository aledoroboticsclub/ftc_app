package org.firstinspires.ftc.teamcode.Code9161_2017;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by jreynoldsstudent on 1/4/2018.
 */
@Autonomous(name="SimpleAutoRed1", group="Linear Opmode")
public class SimpleAutoRed1 extends LinearOpMode{
    Scorpion r = new Scorpion();
    @Override
    public void runOpMode() {
        r.initRobot(hardwareMap, telemetry);
        waitForStart();
        r.driveRightEncoder(1,36);
        r.driveForwardEncoder(1,34);
        r.driveLeftEncoder(1,24);
        r.setTrayToPlace();
        r.waiter(1000);
        r.setTrayToIntake();
        r.driveBackwardEncoder(1,6);
    }
}
