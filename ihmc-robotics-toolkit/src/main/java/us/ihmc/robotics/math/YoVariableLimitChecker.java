package us.ihmc.robotics.math;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class YoVariableLimitChecker
{
   private final YoEnum<Status> status;
   private final double lowerLimit;
   private final double upperLimit;
   private final YoDouble variableToCheck;

   public YoVariableLimitChecker(YoDouble variableToCheck, String prefix, double lowerLimit, double upperLimit, YoVariableRegistry registry)
   {
      status = new YoEnum<Status>(prefix + variableToCheck.getName() + "_Status", registry, Status.class);

      if (upperLimit < lowerLimit)
      {
         System.out.println("YoVariableLimitChecker: Disabling limits. Upper limit needs to be greater than lower limit for variable: "
               + variableToCheck.getName());
         this.lowerLimit = Double.NEGATIVE_INFINITY;
         this.upperLimit = Double.POSITIVE_INFINITY;
      }
      else
      {
         this.lowerLimit = lowerLimit;
         this.upperLimit = upperLimit;
      }

      this.variableToCheck = variableToCheck;
   }

   public Status update()
   {
      if (variableToCheck.getDoubleValue() > upperLimit)
         status.set(Status.ABOVE_LIMIT);
      else if (variableToCheck.getDoubleValue() < lowerLimit)
         status.set(Status.BELOW_LIMIT);
      else
         status.set(Status.IN_RANGE);

      return status.getEnumValue();
   }

   public Status getStatus()
   {
      return status.getEnumValue();
   }
   
   
   public double getDoubleValue()
   {
      return variableToCheck.getDoubleValue();
   }
   public String getName()
   {
      return variableToCheck.getName();
   }

   public enum Status
   {
      IN_RANGE, BELOW_LIMIT, ABOVE_LIMIT
   }
}
