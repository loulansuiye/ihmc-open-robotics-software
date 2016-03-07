package us.ihmc.simulationconstructionset.eventBased;

/**
 * Interface for classes that listen to events emitted by an Observable.
 *
 * @author Twan Koolen
 *
 */
public interface Observer
{
   /**
    * handles an event generated by an Observable
    * @param source event source, i.e. the observable that generated the event
    * @param event event generated by the source
    */
   public abstract void handleEvent(Observable source, Event event);
}
