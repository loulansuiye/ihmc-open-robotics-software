package controller_msgs.msg.dds;

/**
 * Topic data type of the struct "NeckDesiredAccelerationsMessage" defined in "NeckDesiredAccelerationsMessage_.idl". Use this class to provide the TopicDataType to a Participant.
 *
 * This file was automatically generated from NeckDesiredAccelerationsMessage_.idl by us.ihmc.idl.generator.IDLGenerator.
 * Do not update this file directly, edit NeckDesiredAccelerationsMessage_.idl instead.
 */
public class NeckDesiredAccelerationsMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.NeckDesiredAccelerationsMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::NeckDesiredAccelerationsMessage_";
   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   public NeckDesiredAccelerationsMessagePubSubType()
   {

   }

   public static int getMaxCdrSerializedSize()
   {
      return getMaxCdrSerializedSize(0);
   }

   public static int getMaxCdrSerializedSize(int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += controller_msgs.msg.dds.DesiredAccelerationsMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.NeckDesiredAccelerationsMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.NeckDesiredAccelerationsMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += controller_msgs.msg.dds.DesiredAccelerationsMessagePubSubType
            .getCdrSerializedSize(data.getDesiredAccelerations(), current_alignment);

      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.NeckDesiredAccelerationsMessage data, us.ihmc.idl.CDR cdr)
   {

      controller_msgs.msg.dds.DesiredAccelerationsMessagePubSubType.write(data.getDesiredAccelerations(), cdr);
   }

   public static void read(controller_msgs.msg.dds.NeckDesiredAccelerationsMessage data, us.ihmc.idl.CDR cdr)
   {

      controller_msgs.msg.dds.DesiredAccelerationsMessagePubSubType.read(data.getDesiredAccelerations(), cdr);
   }

   public static void staticCopy(controller_msgs.msg.dds.NeckDesiredAccelerationsMessage src, controller_msgs.msg.dds.NeckDesiredAccelerationsMessage dest)
   {
      dest.set(src);
   }

   @Override
   public void serialize(controller_msgs.msg.dds.NeckDesiredAccelerationsMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload)
         throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.NeckDesiredAccelerationsMessage data)
         throws java.io.IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      read(data, deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   @Override
   public final void serialize(controller_msgs.msg.dds.NeckDesiredAccelerationsMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("desired_accelerations", new controller_msgs.msg.dds.DesiredAccelerationsMessagePubSubType(), data.getDesiredAccelerations());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.NeckDesiredAccelerationsMessage data)
   {
      ser.read_type_a("desired_accelerations", new controller_msgs.msg.dds.DesiredAccelerationsMessagePubSubType(), data.getDesiredAccelerations());
   }

   @Override
   public controller_msgs.msg.dds.NeckDesiredAccelerationsMessage createData()
   {
      return new controller_msgs.msg.dds.NeckDesiredAccelerationsMessage();
   }

   @Override
   public int getTypeSize()
   {
      return us.ihmc.idl.CDR.getTypeSize(getMaxCdrSerializedSize());
   }

   @Override
   public java.lang.String getName()
   {
      return name;
   }

   public void serialize(controller_msgs.msg.dds.NeckDesiredAccelerationsMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.NeckDesiredAccelerationsMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }

   public void copy(controller_msgs.msg.dds.NeckDesiredAccelerationsMessage src, controller_msgs.msg.dds.NeckDesiredAccelerationsMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public NeckDesiredAccelerationsMessagePubSubType newInstance()
   {
      return new NeckDesiredAccelerationsMessagePubSubType();
   }
}