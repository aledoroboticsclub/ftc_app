package org.firstinspires.ftc.teamcode.Code2018.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Code2018.Scorpion;

/**
 * Created by jreynoldsstudent on 1/4/2018.
 * This is a template Autonomous, it has all basic vision setup and gyro setup
 * It has no actual driving
 */
@Disabled

@Autonomous(name="AutoTemplate", group="Linear Opmode")
public class AutoTemplate extends LinearOpMode {
    Scorpion r = new Scorpion();
    @Override
    public void runOpMode() {
        r.initRobot(hardwareMap, telemetry);
        r.initGyro();
        r.initVuforia();//starts vuforia
        AutoTransitioner.transitionOnStop(this, "TeleopScorpion");

        waitForStart();

        r.waitUntilVuMarkIsFound();
        r.deactivateVuforia();

        r.initJewelDetector();//starts dogeCV
        while(opModeIsActive()){
            telemetry.addData("Jewel Order: ", r.jewelDetector.getCurrentOrder().toString());
            telemetry.update();
        }
        r.jewelDetector.disable();
    }
}
