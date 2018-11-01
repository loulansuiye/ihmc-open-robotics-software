package us.ihmc.footstepPlanning.graphSearch.listeners;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public interface BipedalFootstepPlannerListener
{
   public abstract void goalWasSet(RigidBodyTransform goalLeftFootPose, RigidBodyTransform goalRightFootPose);
   public abstract void startNodeWasAdded(FootstepNode startNode);
   public abstract void planarRegionsListSet(PlanarRegionsList planarRegionsList);
   
   public abstract void nodeIsBeingExpanded(FootstepNode nodeToExpand);

   public abstract void nodeUnderConsideration(FootstepNode nodeToExpand);
   public abstract void nodeUnderConsiderationWasRejected(FootstepNode rejectedNode, FootstepNode parentNode, BipedalFootstepPlannerNodeRejectionReason reason);
   public abstract void nodeUnderConsiderationWasSuccessful(FootstepNode node);

   public abstract void solutionWasFound(FootstepPlan footstepPlan);
   public abstract void solutionWasNotFound();
}
