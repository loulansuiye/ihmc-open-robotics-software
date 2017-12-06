package us.ihmc.commonWalkingControlModules.dynamicPlanning;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.robotics.linearAlgebra.DiagonalMatrixTools;

import static us.ihmc.commonWalkingControlModules.dynamicPlanning.SimpleLIPMDynamics.controlVectorSize;
import static us.ihmc.commonWalkingControlModules.dynamicPlanning.SimpleLIPMDynamics.stateVectorSize;

public class SimpleLIPMSimpleCostFunction implements LQCostFunction
{
   private final DenseMatrix64F Q = new DenseMatrix64F(stateVectorSize, stateVectorSize);
   private final DenseMatrix64F R = new DenseMatrix64F(controlVectorSize, controlVectorSize);

   /**
    * This is a cost of the form 0.5 (X - X_d)^T Q (X - X_d) + 0.5 (U - U_d)^T R (U - U_d)
    */

   public SimpleLIPMSimpleCostFunction()
   {
      Q.set(0, 0, 1e-5);
      Q.set(1, 1, 1e-5);
      Q.set(2, 2, 1e-5);
      Q.set(3, 3, 1e-5);

      R.set(0, 0, 1e2);
      R.set(1, 1, 1e2);
   }

   private DenseMatrix64F tempStateMatrix = new DenseMatrix64F(stateVectorSize, 1);
   private DenseMatrix64F tempControlMatrix = new DenseMatrix64F(controlVectorSize, 1);
   private DenseMatrix64F tempWX = new DenseMatrix64F(stateVectorSize, 1);
   private DenseMatrix64F tempWU = new DenseMatrix64F(controlVectorSize, 1);

   public double getCost(DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F desiredControlVector,
                  DenseMatrix64F desiredStateVector)
   {
      CommonOps.subtract(controlVector, desiredControlVector, tempControlMatrix);
      CommonOps.subtract(stateVector, desiredStateVector, tempStateMatrix);

      DiagonalMatrixTools.preMult(Q, tempStateMatrix, tempWX);
      DiagonalMatrixTools.preMult(R, tempControlMatrix, tempWU);

      return CommonOps.dot(tempControlMatrix, tempWU) + CommonOps.dot(tempStateMatrix, tempWX);
   }

   /** L_x(X_k, U_k) */
   public void getCostStateGradient(DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F desiredControlVector,
                                    DenseMatrix64F desiredStateVector, DenseMatrix64F matrixToPack)
   {
      CommonOps.subtract(stateVector, desiredStateVector, tempStateMatrix);
      DiagonalMatrixTools.preMult(Q, tempStateMatrix, matrixToPack);
   }

   /** L_u(X_k, U_k) */
   public void getCostControlGradient(DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F desiredControlVector,
                                      DenseMatrix64F desiredStateVector, DenseMatrix64F matrixToPack)
   {
      CommonOps.subtract(controlVector, desiredControlVector, tempControlMatrix);
      DiagonalMatrixTools.preMult(R, tempControlMatrix, matrixToPack);
   }

   /** L_xx(X_k, U_k) */
   public void getCostStateHessian(DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F matrixToPack)
   {
      matrixToPack.set(Q);
   }

   /** L_uu(X_k, U_k) */
   public void getCostControlHessian(DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F matrixToPack)
   {
      matrixToPack.set(R);
   }

   /** L_ux(X_k, U_k) */
   public void getCostStateGradientOfControlGradient(DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F matrixToPack)
   {
      matrixToPack.reshape(controlVectorSize, stateVectorSize);
   }
}
