package us.ihmc.footstepPlanning.graphSearch.listeners;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class BodyCollisionListener implements NodeFailureEventListener
{
   public PlannerHeuristicNodeSearchPolicy heuristicSearchPolicy = null;

   public void setHeuristicSearchPolicy(PlannerHeuristicNodeSearchPolicy heuristicSearchPolicy)
   {
      this.heuristicSearchPolicy = heuristicSearchPolicy;
   }

   @Override
   public void goalWasSet(RigidBodyTransform goalLeftFootPose, RigidBodyTransform goalRightFootPose)
   {
   }

   @Override
   public void startNodeWasAdded(FootstepNode startNode)
   {
   }

   @Override
   public void planarRegionsListSet(PlanarRegionsList planarRegionsList)
   {

   }

   @Override
   public void nodeIsBeingExpanded(FootstepNode nodeToExpand)
   {

   }

   @Override
   public void nodeUnderConsideration(FootstepNode nodeToExpand)
   {

   }

   @Override
   public void nodeUnderConsiderationWasRejected(FootstepNode rejectedNode, FootstepNode parentNode, BipedalFootstepPlannerNodeRejectionReason reason)
   {
      if (reason.equals(BipedalFootstepPlannerNodeRejectionReason.OBSTACLE_HITTING_BODY) && heuristicSearchPolicy != null)
      {
         boolean foundNewNode = heuristicSearchPolicy.performSearchForValidNode(rejectedNode, parentNode);
         if (foundNewNode)
            heuristicSearchPolicy.executeActionPoliciesForNewValidNode();
      }

   }

   @Override
   public void nodeUnderConsiderationWasSuccessful(FootstepNode node)
   {

   }

   @Override
   public void solutionWasFound(FootstepPlan footstepPlan)
   {

   }

   @Override
   public void solutionWasNotFound()
   {

   }
}
