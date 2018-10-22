package us.ihmc.commonWalkingControlModules.configurations;

import static org.junit.Assert.fail;

import java.lang.reflect.Method;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.PrintTools;

public class WalkingControllerParametersTest
{
   @Test
   public void testNoParameters() throws ClassNotFoundException, InstantiationException, IllegalAccessException
   {
      Class<?> c = Class.forName("us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters");
      Method[] allMethods = c.getDeclaredMethods();

      boolean foundMethodWithArgument = false;

      for (Method method : allMethods)
      {
         Class<?>[] parameterTypes = method.getParameterTypes();

         if (parameterTypes.length != 0)
         {
            PrintTools.info(method.getName() + " has arguments!");
            for (Class<?> parameter : parameterTypes)
            {
               System.out.println("   Argument type: " + parameter.getSimpleName());
            }

            foundMethodWithArgument = true;
         }
      }

      if (foundMethodWithArgument)
      {
         fail("This is a parameter class. It should not have methods that take arguments.");
      }
   }
}
