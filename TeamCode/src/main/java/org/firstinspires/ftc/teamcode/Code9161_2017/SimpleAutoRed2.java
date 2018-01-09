package org.firstinspires.ftc.teamcode.Code9161_2017;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by jreynoldsstudent on 1/4/2018.
 */
@Autonomous(name="SimpleAutoRed2", group="Linear Opmode")
public class SimpleAutoRed2 extends LinearOpMode{
    Scorpion r = new Scorpion();
    @Override
    public void runOpMode() {
        r.initRobot(hardwareMap, telemetry);
        waitForStart();
        r.driveRightEncoder(1,36);
        r.driveForwardEncoder(1,10);
        r.setTrayToPlace();
        r.waiter(1000);
        r.setTrayToIntake();
        r.driveBackwardEncoder(1,6);
    }
}
