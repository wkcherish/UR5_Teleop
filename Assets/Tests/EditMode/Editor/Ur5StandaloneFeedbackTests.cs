using System.Reflection;
using NUnit.Framework;
using UnityEngine;

public class Ur5StandaloneFeedbackTests
{
    private static string Feedback(string session, long sequence, string joints = "0,1,2,3,4,5", string gripper = "0.5")
    {
        return "{\"protocol_version\":1,\"stream\":\"ur5_joint_state\",\"source\":\"quest_ur5_host\","
            + "\"session_id\":\"" + session + "\",\"sequence\":" + sequence
            + ",\"joint_positions_rad\":[" + joints + "],\"gripper_command\":" + gripper + "}";
    }

    private static void Accept(Ur5RtdeJointStateReceiver receiver, string json)
    {
        typeof(Ur5RtdeJointStateReceiver).GetMethod("TryAcceptPacket", BindingFlags.NonPublic | BindingFlags.Instance)
            .Invoke(receiver, new object[] { json });
    }

    [Test]
    public void FeedbackRejectsPartialInvalidUpdateAndAllowsNewHostSession()
    {
        GameObject owner = new GameObject("feedback-protocol-test");
        owner.SetActive(false);
        try
        {
            var receiver = owner.AddComponent<Ur5RtdeJointStateReceiver>();
            receiver.logStatus = false;
            Accept(receiver, Feedback("host-a", 100));
            Assert.AreEqual(100, receiver.LastSequence);
            FieldInfo pending = typeof(Ur5RtdeJointStateReceiver)
                .GetField("pendingJointDegrees", BindingFlags.NonPublic | BindingFlags.Instance);
            float[] accepted = (float[])pending.GetValue(receiver);

            Accept(receiver, Feedback("host-a", 101, "1,2,3,4,5,1e39"));
            Assert.AreSame(accepted, pending.GetValue(receiver), "A rejected packet must not partially change the pose.");
            Assert.AreEqual(100, receiver.LastSequence);
            Accept(receiver, Feedback("host-a", 102, "1,2,3,4,5,6", "1e39"));
            Assert.AreSame(accepted, pending.GetValue(receiver));
            Assert.AreEqual(100, receiver.LastSequence);

            Accept(receiver, Feedback("host-b", 0));
            Assert.AreEqual(0, receiver.LastSequence, "A new host process can restart its sequence.");
            Accept(receiver, Feedback("host-a", 103));
            Assert.AreEqual(0, receiver.LastSequence, "Delayed packets from retired sessions must be rejected.");
        }
        finally
        {
            Object.DestroyImmediate(owner);
        }
    }
}
