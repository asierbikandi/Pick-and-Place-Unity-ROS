using System;
using RosMessageTypes.Geometry;
using RosMessageTypes.UR3;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.UrdfImporter;
using UnityEngine;

public class SourceDestinationPublisher : MonoBehaviour
{
    const int k_NumRobotJoints = 6;

    public static readonly string[] LinkNames =
        { "base_link/base_link_inertia/shoulder_link", "/upper_arm_link", "/forearm_link", "/wrist_1_link", "/wrist_2_link", "/wrist_3_link" };

    // Variables required for ROS communication
    [SerializeField]
    string m_TopicName = "/UR3_joints";

    [SerializeField]
    GameObject m_UR3;
    [SerializeField]
    GameObject m_Target_BlueBall_1;
    [SerializeField]
    GameObject m_Target_BlueBall_2;
    [SerializeField]
    GameObject m_Target_RedBall_1;
    [SerializeField]
    GameObject m_Target_RedBall_2;
    [SerializeField]
    GameObject m_Placement_Basket_1;
    [SerializeField]
    GameObject m_Placement_Basket_2;


    readonly Quaternion m_PickOrientation = Quaternion.Euler(90, 90, 0);

    // Robot Joints
    UrdfJointRevolute[] m_JointArticulationBodies;

    // ROS Connector
    ROSConnection m_Ros;

    void Start()
    {
        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterPublisher<UR3_MoveitJointsMsg>(m_TopicName);

        m_JointArticulationBodies = new UrdfJointRevolute[k_NumRobotJoints];

        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            linkName += LinkNames[i];
            m_JointArticulationBodies[i] = m_UR3.transform.Find(linkName).GetComponent<UrdfJointRevolute>();
        }
    }

    public void Publish()
    {
        var sourceDestinationMessage = new UR3_MoveitJointsMsg();

        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            sourceDestinationMessage.joints[i] = m_JointArticulationBodies[i].GetPosition();
        }

        // Pick Pose Blue Ball 1
        sourceDestinationMessage.pick_pose_blueball_1 = new PoseMsg
        {
            position = m_Target_BlueBall_1.transform.position.To<FLU>(),
            orientation = Quaternion.Euler(90, m_Target_BlueBall_1.transform.eulerAngles.y, 0).To<FLU>()
        };
        // Pick Pose Blue Ball 2
        sourceDestinationMessage.pick_pose_blueball_2 = new PoseMsg
        {
            position = m_Target_BlueBall_2.transform.position.To<FLU>(),
            orientation = Quaternion.Euler(90, m_Target_BlueBall_2.transform.eulerAngles.y, 0).To<FLU>()
        };

        // Pick Pose Red Ball 1
        sourceDestinationMessage.pick_pose_redball_1 = new PoseMsg
        {
            position = m_Target_RedBall_1.transform.position.To<FLU>(),
            orientation = Quaternion.Euler(90, m_Target_RedBall_1.transform.eulerAngles.y, 0).To<FLU>()
        };
        // Pick Pose Red Ball 2
        sourceDestinationMessage.pick_pose_redball_2 = new PoseMsg
        {
            position = m_Target_RedBall_2.transform.position.To<FLU>(),
            orientation = Quaternion.Euler(90, m_Target_RedBall_2.transform.eulerAngles.y, 0).To<FLU>()
        };

        // Place Pose Basket 1
        sourceDestinationMessage.Place_basket_1 = new PoseMsg
        {
            position = m_Placement_Basket_1.transform.position.To<FLU>(),
            orientation = m_PickOrientation.To<FLU>()
        };
        // Place Pose Basket 2
        sourceDestinationMessage.Place_basket_2 = new PoseMsg
        {
            position = m_Placement_Basket_2.transform.position.To<FLU>(),
            orientation = m_PickOrientation.To<FLU>()
        };

        // Finally send the message to server_endpoint.py running in ROS
        m_Ros.Publish(m_TopicName, sourceDestinationMessage);
    }
}
