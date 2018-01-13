package org.firstinspires.ftc.teamcode.Code9161_2017.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Code9161_2017.Scorpion;

/**
 * Created by jreynoldsstudent on 1/4/2018.
 */
@Autonomous(name="BlueColorTester", group="Linear Opmode")
public class BlueColorTester extends LinearOpMode{
    Scorpion r = new Scorpion();
    @Override
    public void runOpMode() {
        r.initRobot(hardwareMap, telemetry);
        waitForStart();
        r.pushJewel("Blue");
    }
}
