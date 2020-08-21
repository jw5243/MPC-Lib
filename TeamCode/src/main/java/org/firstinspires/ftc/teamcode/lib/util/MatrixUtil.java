package org.firstinspires.ftc.teamcode.lib.util;

import org.ejml.simple.SimpleMatrix;

public class MatrixUtil {
    public static SimpleMatrix expm(SimpleMatrix A) {
        // constants for pade approximation
        final double c0 = 1.0;
        final double c1 = 0.5;
        final double c2 = 0.12;
        final double c3 = 0.01833333333333333;
        final double c4 = 0.0019927536231884053;
        final double c5 = 1.630434782608695E-4;
        final double c6 = 1.0351966873706E-5;
        final double c7 = 5.175983436853E-7;
        final double c8 = 2.0431513566525E-8;
        final double c9 = 6.306022705717593E-10;
        final double c10 = 1.4837700484041396E-11;
        final double c11 = 2.5291534915979653E-13;
        final double c12 = 2.8101705462199615E-15;
        final double c13 = 1.5440497506703084E-17;

        int j = Math.max(0, 1 + (int) Math.floor(Math.log(A.elementMaxAbs()) / Math.log(2)));
        SimpleMatrix As = A.divide(Math.pow(2, j)); // scaled version of A
        int n = A.numRows();

        // calculate D and N using special Horner techniques
        SimpleMatrix As_2 = As.mult(As);
        SimpleMatrix As_4 = As_2.mult(As_2);
        SimpleMatrix As_6 = As_4.mult(As_2);

        // U = c0*I + c2*A^2 + c4*A^4 + (c6*I + c8*A^2 + c10*A^4 + c12*A^6)*A^6
        SimpleMatrix U = SimpleMatrix.identity(n).scale(c0).plus(As_2.scale(c2)).plus(As_4.scale(c4)).plus(
                SimpleMatrix.identity(n).scale(c6).plus(As_2.scale(c8)).plus(As_4.scale(c10)).plus(As_6.scale(c12)).mult(As_6));
        // V = c1*I + c3*A^2 + c5*A^4 + (c7*I + c9*A^2 + c11*A^4 + c13*A^6)*A^6
        SimpleMatrix V = SimpleMatrix.identity(n).scale(c1).plus(As_2.scale(c3)).plus(As_4.scale(c5)).plus(
                SimpleMatrix.identity(n).scale(c7).plus(As_2.scale(c9)).plus(As_4.scale(c11)).plus(As_6.scale(c13)).mult(As_6));

        SimpleMatrix AV = As.mult(V);
        SimpleMatrix N = U.plus(AV);
        SimpleMatrix D = U.minus(AV);

        // solve DF = N for F
        SimpleMatrix F = D.solve(N);
        // now square j times
        for (int k = 0; k < j; k++) {
            F = F.mult(F);
        }

        return F;
    }

    public static SimpleMatrix expAt(SimpleMatrix A, double t) {
        return expm(A.scale(t));
    }
}
