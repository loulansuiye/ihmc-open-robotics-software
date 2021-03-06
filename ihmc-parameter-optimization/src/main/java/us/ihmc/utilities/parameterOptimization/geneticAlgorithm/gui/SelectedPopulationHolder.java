package us.ihmc.utilities.parameterOptimization.geneticAlgorithm.gui;

import us.ihmc.utilities.parameterOptimization.geneticAlgorithm.Population;

/**
 * <p>Title: Genetic Algorithm Library </p>
 *
 * <p>Description: General Purpose Genetic Algorithm Library </p>
 *
 * <p>Copyright: Copyright (c) 2003-2005 Jerry Pratt, IHMC </p>
 *
 * <p>Company: Institute for Human and Machine Cognition.
 * 40 South Alcaniz Street
 * Pensacola, FL 32502 </p>
 *
 * @author Jerry Pratt and Jim Warrenfeltz, jpratt@ihmc.us
 * @version 1.0
 */

public interface SelectedPopulationHolder
{
   public abstract Population getSelectedPopulation();

   public abstract void setSelectedPopulation(Population population);
}
