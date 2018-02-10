package org.firstinspires.ftc.teamcode.Code2018.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Code2018.Scorpion;

/**
 * Created by jreynoldsstudent on 1/4/2018.
 */
//@Disabled
@Autonomous(name="SimpleAutoRed1", group="Linear Opmode")
public class SimpleAutoRed1 extends LinearOpMode {
    Scorpion r = new Scorpion();
    @Override
    public void runOpMode() {
        r.initRobot(hardwareMap, telemetry);
        AutoTransitioner.transitionOnStop(this, "TeleopScorpion");

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
