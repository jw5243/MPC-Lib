package com.horse.mpclib.examples;

import org.ejml.simple.SimpleMatrix;
import com.horse.mpclib.debugging.ComputerDebugger;
import com.horse.mpclib.debugging.IllegalMessageTypeException;
import com.horse.mpclib.debugging.MessageOption;
import com.horse.mpclib.lib.geometry.Line2d;
import com.horse.mpclib.lib.geometry.Pose2d;
import com.horse.mpclib.lib.geometry.Rotation2d;
import com.horse.mpclib.lib.geometry.Translation2d;
import com.horse.mpclib.lib.motion.Spline;

import java.util.function.DoubleUnaryOperator;

public class RobotGVF extends Robot {
    private Spline spline = new Spline(2, new double[] {
            9d, 9d, 144d - 18d, 0d, 0d, 2d * (144d - 18d)
    }, false);

    private static final double kF = 1d / 30d;
    private static final double kN = 0.1d;
    private static final double kOmega = 1d / Math.toRadians(360d);
    private static final DoubleUnaryOperator errorMap = (error) -> error;

    private static boolean done = false;

    @Override
    public void init_debug() {
        super.init_debug();
    }

    @Override
    public void loop_debug() {
        super.loop_debug();
        double[] parameterAndDisplacement = spline.getParameterAndMinDistanceFromPoint(getFieldPosition().getTranslation());
        Translation2d tangent = spline.getDerivative(parameterAndDisplacement[0]);
        tangent = tangent.scale(1d / tangent.norm());
        Translation2d normal = tangent.rotateBy(new Rotation2d(Math.PI / 2d, false));
        Translation2d displacementVector = spline.evaluate(parameterAndDisplacement[0]).translateBy(getFieldPosition().getTranslation().inverse());
        double orientation = Math.signum(Translation2d.cross(displacementVector, tangent));
        double error = orientation * displacementVector.norm();
        System.out.println(error);
        Translation2d vectorFieldResult = tangent.translateBy(normal.inverse().scale(kN * errorMap.applyAsDouble(error)));
        vectorFieldResult = vectorFieldResult.scale(1d / vectorFieldResult.norm());

        //GVF Following algorithm
        double desiredHeading = Math.atan2(vectorFieldResult.y(), vectorFieldResult.x());
        double headingError = desiredHeading - getFieldPosition().getRotation().getRadians();
        double angularOutput = kOmega * headingError;

        double distanceToPoint = spline.evaluate(1d).distance(getFieldPosition().getTranslation());
        double forwardOutput = kF * distanceToPoint;

        Translation2d translationalPower = /*new Translation2d(forwardOutput, 0d)*/vectorFieldResult.scale(forwardOutput).rotateBy(getFieldPosition().getRotation());

        //System.out.println(distanceToPoint);
        if(distanceToPoint > 1d && !done) {
            setInput(convertPowerToInput(new Pose2d(translationalPower, new Rotation2d(0d, false))));
        } else {
            done = true;
            setInput(new SimpleMatrix(4, 1, false, new double[] {0, 0, 0, 0}));
        }

        try {
            displacementVector = vectorFieldResult;
            if(displacementVector.norm() != 0) {
                ComputerDebugger.send(MessageOption.LINE.setSendValue(
                        new Line2d(getFieldPosition().getTranslation(), getFieldPosition().getTranslation().translateBy(
                                displacementVector.scale(20d / displacementVector.norm())
                        ))
                ));
            }
        } catch (IllegalMessageTypeException e) {
            e.printStackTrace();
        }

        int steps = 500;
        Translation2d[] values = new Translation2d[2 * steps];
        for(int i = 0; i < steps; i++) {
            values[i] = spline.evaluate((double)(i) / steps);
        }

        for(int i = 0; i < values.length - 1; i++) {
            try {
                ComputerDebugger.send(MessageOption.LINE.setSendValue(new Line2d(values[i], values[i + 1])));
            } catch (IllegalMessageTypeException e) {
                e.printStackTrace();
            }
        }
    }

    public static SimpleMatrix convertPowerToInput(Pose2d power) {
        double x = power.getTranslation().x();
        double y = power.getTranslation().y();
        double turn = power.getRotation().getRadians();

        double frontLeft  = x - y - turn;
        double frontRight = x + y + turn;
        double backLeft   = x + y - turn;
        double backRight  = x - y + turn;

        double highestPower = Math.abs(frontLeft);
        if(highestPower < Math.abs(frontRight)) {
            highestPower = Math.abs(frontRight);
        }

        if(highestPower < Math.abs(backLeft)) {
            highestPower = Math.abs(backLeft);
        }

        if(highestPower < Math.abs(backRight)) {
            highestPower = Math.abs(backRight);
        }

        double normalization = highestPower > 1d ? 1d / highestPower : 1d;

        return new SimpleMatrix(4, 1, false, new double[] {
                frontLeft * normalization,
                frontRight * normalization,
                backLeft * normalization,
                backRight * normalization
        });
    }
}
