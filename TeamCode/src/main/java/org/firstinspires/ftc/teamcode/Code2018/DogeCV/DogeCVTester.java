package org.firstinspires.ftc.teamcode.Code2018.DogeCV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.Code2018.Scorpion;

/**
 * Created by jreynoldsstudent on 11/25/2017.
 */
@Autonomous(name="DogeCVTester", group ="Concept")
//@Disabled
public class DogeCVTester extends LinearOpMode
{
    Scorpion r=new Scorpion();
    @Override public void runOpMode() {

        r.getOpmodeVariables(hardwareMap, telemetry);
        r.initJewelDetector();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Jewel Order: ", r.jewelDetector.getCurrentOrder());
        }
        r.jewelDetector.disable();
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}


