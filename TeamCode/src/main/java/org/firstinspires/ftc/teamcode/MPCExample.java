package org.firstinspires.ftc.teamcode;

import com.horse.mpclib.lib.drivers.Motor;
import com.horse.mpclib.lib.geometry.Pose2d;
import com.horse.mpclib.lib.geometry.Rotation2d;
import com.horse.mpclib.lib.physics.MecanumDriveModel;
import com.horse.mpclib.lib.physics.MotorModel;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.ejml.simple.SimpleMatrix;

@Autonomous
public class MPCExample extends OpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private Pose2d fieldPosition = new Pose2d(0d, 0d, new Rotation2d(0d, false));
    private Pose2d desiredPose   = new Pose2d(144d, 144d, new Rotation2d(Math.toRadians(90d), false));

    private static SimpleMatrix state;
    private static SimpleMatrix input;

    private static MecanumDriveModel driveModel;

    private static MecanumDriveMPC mecanumDriveMPC;
    private MecanumRunnableMPC mecanumRunnableMPC;

    @Override
    public void init() {
        state = new SimpleMatrix(6, 1);
        input = new SimpleMatrix(4, 1);
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        driveModel = new MecanumDriveModel(
                0.001d, 18.4d, 0.315d, 0.315d * (0.1d * 0.1d + 0.032d * 0.032d) / 2d,
                0.315d * (3d * (0.1d * 0.1d + 0.032d * 0.032d) + 0.05d * 0.05d) / 12d, 0.5613d,
                0.1d / 2d, 7d * 0.0254d, 7d * 0.0254d, 6d * 0.0254d, 6d * 0.0254d,
                MotorModel.generateMotorModel(Motor.NEVEREST_20, null));
    }

    @Override
    public void loop() {

        applyInput();
    }

    private void applyInput() {
        frontLeft.setPower(input.get(0));
        frontRight.setPower(input.get(1));
        backLeft.setPower(input.get(2));
        backRight.setPower(input.get(3));
    }
}
