package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.decoupledMotionGeneration;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commons.PrintTools;
import us.ihmc.convexOptimization.quadraticProgram.JavaQuadProgSolver;
import us.ihmc.euclid.Axis;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * Optimized linear motion along the Z axis to obtain a feasible CoM height trajectory
 * This basically involves trading off between force, velocity and position objectives to obtain a desired motion
 * @author Apoorv S
 *
 */
public class CentroidalZAxisOptimizationControlModule
{
   private final String name;
   private final Axis axis = Axis.Z;
   // Planner runtime variables
   private final ControlModuleHelper helper;

   private final DenseMatrix64F solverInput_H;
   private final DenseMatrix64F solverInput_f;
   private final DenseMatrix64F solverInput_lb;
   private final DenseMatrix64F solverInput_ub;
   private final DenseMatrix64F solverInput_Ain;
   private final DenseMatrix64F solverInput_bin;
   private final DenseMatrix64F solverInput_Aeq;
   private final DenseMatrix64F solverInput_beq;

   private final JavaQuadProgSolver qpSolver;
   private final DenseMatrix64F qpSolution;
   private final YoInteger numberOfQPIterations;
   private final YoInteger numberOfQPFailures;
   private final YoInteger numberOfPlansRequested;

   private DenseMatrix64F tempMatrixForComputation = new DenseMatrix64F(0, 1);

   public CentroidalZAxisOptimizationControlModule(ControlModuleHelper helper, CentroidalMotionPlannerParameters parameters, YoVariableRegistry registry)
   {
      this.name = getClass().getSimpleName();
      this.helper = helper;

      // Initialize the QP matrices
      solverInput_H = helper.getObjectiveHMatrix(this.axis);
      solverInput_f = helper.getObjectivefMatrix(this.axis);
      solverInput_lb = new DenseMatrix64F(0, 1);
      solverInput_ub = new DenseMatrix64F(0, 1);
      solverInput_Aeq = helper.getConstraintAeqMatrix(this.axis);
      solverInput_beq = helper.getConstraintbeqMatrix(this.axis);
      solverInput_Ain = new DenseMatrix64F(0, 1);
      solverInput_bin = new DenseMatrix64F(0, 1);

      qpSolver = new JavaQuadProgSolver();
      //qpSolver.setConvergenceThreshold(parameters.getOptimizationConvergenceThreshold());
      qpSolution = new DenseMatrix64F(0, 1);

      this.numberOfQPFailures = new YoInteger(name + "NumberOfQPFailures", registry);
      this.numberOfQPIterations = new YoInteger(name + "NumberOfQPIterations", registry);
      this.numberOfPlansRequested = new YoInteger(name + "NumberOfPlansRequested", registry);
      this.numberOfQPFailures.set(0);
      this.numberOfQPIterations.set(0);
      this.numberOfPlansRequested.set(0);
      reset();
   }

   public void reset()
   {
      qpSolver.clear();
   }

   public void compute()
   {
      numberOfPlansRequested.increment();
      processQPInputs();
      qpSolution.reshape(helper.getNumberOfDecisionVariables(axis), 1);
      qpSolver.setQuadraticCostFunction(solverInput_H, solverInput_f, 0.0);
      qpSolver.setLinearEqualityConstraints(solverInput_Aeq, solverInput_beq);
      qpSolver.setLinearInequalityConstraints(solverInput_Ain, solverInput_bin);
      qpSolver.setLowerBounds(solverInput_lb);
      qpSolver.setUpperBounds(solverInput_ub);
      try
      {
         int iterations = qpSolver.solve(qpSolution);
         numberOfQPIterations.add(iterations);
         if (solutionContainsNaN())
            numberOfQPFailures.increment();
      }
      catch (RuntimeException exception)
      {
         PrintTools.debug("Got runtime exception from QP solver");
         numberOfQPFailures.increment();
      }
      helper.setDecisionVariableValues(axis, qpSolution);
   }

   private boolean solutionContainsNaN()
   {
      for (int i = 0; i < qpSolution.numRows; i++)
         if (!Double.isFinite(qpSolution.get(i, 0)))
            return true;

      return false;
   }

   private void processQPInputs()
   {
      DenseMatrix64F upperBoundConstraints = helper.getDecisionVariableUpperBoundMatrix(axis);
      DenseMatrix64F lowerBoundConstraints = helper.getDecisionVariableLowerBoundMatrix(axis);
      solverInput_lb.set(lowerBoundConstraints);
      solverInput_ub.set(upperBoundConstraints);
      DenseMatrix64F helperInequalitiesAin = helper.getConstraintAinMatrix(axis);
      DenseMatrix64F helperInequalitiesbin = helper.getConstraintbinMatrix(axis);
      solverInput_Ain.set(helperInequalitiesAin);
      solverInput_bin.set(helperInequalitiesbin);

      //      PrintTools.debug("H:" + solverInput_H.toString());
      //      PrintTools.debug("f:" + solverInput_f.toString());
      //      PrintTools.debug("Aeq:" + solverInput_Aeq.toString());
      //      PrintTools.debug("beq:" + solverInput_beq.toString());
      //      PrintTools.debug("Ain:" + solverInput_Ain.toString());
      //      PrintTools.debug("bin:" + solverInput_bin.toString());
      //      PrintTools.debug("ub:" + solverInput_ub.toString());
      //      PrintTools.debug("lb:" + solverInput_lb.toString());

   }
}