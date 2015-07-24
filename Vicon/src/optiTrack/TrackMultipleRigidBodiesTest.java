package optiTrack;

import java.util.ArrayList;

import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.time.CallFrequencyCalculator;

public class TrackMultipleRigidBodiesTest implements MocapRigidbodiesListener
{
   private YoVariableRegistry registry = new YoVariableRegistry("MOCAP");
   private CallFrequencyCalculator frequencyCalculator = new CallFrequencyCalculator(registry, "");

   public TrackMultipleRigidBodiesTest()
   {
      MocapDataClient mocapDataClient = new MocapDataClient();
      mocapDataClient.registerRigidBodiesListener(this);
   }

   @Override
   public void updateRigidbodies(ArrayList<MocapRigidBody> listOfRigidbodies)
   {
      System.out.println("\n\n>> START DATA RECEIVED: ");
      System.out.println("# of RigidBodies: " + listOfRigidbodies.size());

      for (MocapRigidBody rigidBody : listOfRigidbodies)
      {
         System.out.println(rigidBody.toString());
      }

      System.out.println("Update rate: " + frequencyCalculator.determineCallFrequency() + " Hz");
      System.out.println("<< END DATA RECEIVED ");
   }

   public static void main(String args[])
   {
      new TrackMultipleRigidBodiesTest();
   }
}
