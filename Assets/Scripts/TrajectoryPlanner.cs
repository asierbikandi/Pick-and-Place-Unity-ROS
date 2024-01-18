using System;
using System.Collections;
using System.Linq;
using RosMessageTypes.Geometry;
using RosMessageTypes.UR3;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class TrajectoryPlanner : MonoBehaviour
{
    // Hardcoded variables
    const int k_NumRobotJoints = 6;
    const float k_JointAssignmentWait = 0.1f;
    const float k_PoseAssignmentWait = 0.5f;

    // Variables required for ROS communication
    [SerializeField]
    string m_RosServiceName = "ur3_moveit";
    public string RosServiceName { get => m_RosServiceName; set => m_RosServiceName = value; }

    [SerializeField]
    GameObject m_UR3;
    public GameObject UR3 { get => m_UR3; set => m_UR3 = value; }
    [SerializeField]
    GameObject m_Target_BlueBall_1;
    public GameObject Target_1 { get => m_Target_BlueBall_1; set => m_Target_BlueBall_1 = value; }
    [SerializeField]
    GameObject m_Target_BlueBall_2;
    public GameObject Target_2 { get => m_Target_BlueBall_2; set => m_Target_BlueBall_2 = value; }
    [SerializeField]
    GameObject m_Target_RedBall_1;
    public GameObject Target_3 { get => m_Target_RedBall_1; set => m_Target_RedBall_1 = value; }
    [SerializeField]
    GameObject m_Target_RedBall_2;
    public GameObject Target_4 { get => m_Target_RedBall_2; set => m_Target_RedBall_2 = value; }
    [SerializeField]
    GameObject m_Placement_Basket_1;
    public GameObject TargetPlacement_1 { get => m_Placement_Basket_1; set => m_Placement_Basket_1 = value; }
    [SerializeField]
    GameObject m_Placement_Basket_2;
    public GameObject TargetPlacement_2 { get => m_Placement_Basket_2; set => m_Placement_Basket_2 = value; }

    // Assures that the gripper is always positioned above the m_Target cube before grasping.
    readonly Quaternion m_PickOrientation = Quaternion.Euler(90, 90, 0);
    readonly Vector3 m_PickPoseOffset = Vector3.up * 0.0f ; //Vector3.up * 0.1f;

    // Articulation Bodies
    ArticulationBody[] m_JointArticulationBodies;
    ArticulationBody m_LeftGripper;
    ArticulationBody m_RightGripper;

    // ROS Connector
    ROSConnection m_Ros;

    /// <summary>
    ///     Find all robot joints in Awake() and add them to the jointArticulationBodies array.
    ///     Find left and right finger joints and assign them to their respective articulation body objects.
    /// </summary>
    void Start()
    {
        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterRosService<MoverServiceRequest, MoverServiceResponse>(m_RosServiceName);

        m_JointArticulationBodies = new ArticulationBody[k_NumRobotJoints];

        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            linkName += SourceDestinationPublisher.LinkNames[i];
            m_JointArticulationBodies[i] = m_UR3.transform.Find(linkName).GetComponent<ArticulationBody>();
        }

        // Find left and right fingers
        var rightGripper = linkName + "/flange/tool0/coupling/hand_e_link/hande_right_finger";
        var leftGripper = linkName + "/flange/tool0/coupling/hand_e_link/hande_left_finger";

        m_RightGripper = m_UR3.transform.Find(rightGripper).GetComponent<ArticulationBody>();
        m_LeftGripper = m_UR3.transform.Find(leftGripper).GetComponent<ArticulationBody>();
    }

    /// <summary>
    ///     Close the gripper
    /// </summary>
    void CloseGripper()
    {
        var leftDrive = m_LeftGripper.xDrive;
        var rightDrive = m_RightGripper.xDrive;

        leftDrive.target = 0.007f;
        rightDrive.target = 0.007f;

        m_LeftGripper.xDrive = leftDrive;
        m_RightGripper.xDrive = rightDrive;
    }

    /// <summary>
    ///     Open the gripper
    /// </summary>
    void OpenGripper()
    {
        var leftDrive = m_LeftGripper.xDrive;
        var rightDrive = m_RightGripper.xDrive;

        leftDrive.target = -0.01f;
        rightDrive.target = -0.01f;

        m_LeftGripper.xDrive = leftDrive;
        m_RightGripper.xDrive = rightDrive;
    }

    /// <summary>
    ///     Get the current values of the robot's joint angles.
    /// </summary>
    /// <returns>UR3_MoveitJoints</returns>
    UR3_MoveitJointsMsg CurrentJointConfig()
    {
        var joints = new UR3_MoveitJointsMsg();

        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            joints.joints[i] = m_JointArticulationBodies[i].jointPosition[0];
        }

        return joints;
    }

    private PointMsg Vector3ToPointMsg(Vector3 vector)
    {
        return new PointMsg
        {
            x = vector.x,
            y = vector.y,
            z = vector.z
        };
    }

    /// <summary>
    ///     Create a new MoverServiceRequest with the current values of the robot's joint angles,
    ///     the target cube's current position and rotation, and the targetPlacement position and rotation.
    ///     Call the MoverService using the ROSConnection and if a trajectory is successfully planned,
    ///     execute the trajectories in a coroutine.
    /// </summary>
    public void PublishJoints()
    {
        var Rob_offset = new Vector3(-0.4f, 1.5f, 0.8f);  // Offset values because of the robots position
        var request = new MoverServiceRequest();
        request.joints_input = CurrentJointConfig();

        // Pick Pose blueball_1
        request.pick_pose_blueball_1 = new PoseMsg
        {
            //position = (m_Target_BlueBall_1.transform.position + m_PickPoseOffset - Rob_offset).To<FLU>(),
            position = Vector3ToPointMsg(m_UR3.transform.InverseTransformPoint(m_Target_BlueBall_1.transform.position )),
 
            // The hardcoded x/z angles assure that the gripper is always positioned above the target cube before grasping.
            //orientation = Quaternion.Euler(90, m_Target_BlueBall_1.transform.eulerAngles.y, 0).To<FLU>()
            orientation = Quaternion.Euler(90, 0, 0).To<FLU>()
        };
        // Pick Pose blueball_2
        request.pick_pose_blueball_2 = new PoseMsg
        {
            //position = (m_Target_BlueBall_2.transform.position + m_PickPoseOffset - Rob_offset).To<FLU>(),
            position = Vector3ToPointMsg(m_UR3.transform.InverseTransformPoint(m_Target_BlueBall_2.transform.position + m_PickPoseOffset)),
            // The hardcoded x/z angles assure that the gripper is always positioned above the target cube before grasping.
            //orientation = Quaternion.Euler(90, m_Target_BlueBall_2.transform.eulerAngles.y, 0).To<FLU>()
            orientation = Quaternion.Euler(90, 0, 0).To<FLU>()
        };
        // Pick Pose redball_1
        request.pick_pose_redball_1 = new PoseMsg
        {
            //position = (m_Target_RedBall_1.transform.position + m_PickPoseOffset - Rob_offset).To<FLU>(),
            position = Vector3ToPointMsg(m_UR3.transform.InverseTransformPoint(m_Target_RedBall_1.transform.position + m_PickPoseOffset)),

            // The hardcoded x/z angles assure that the gripper is always positioned above the target cube before grasping.
            //orientation = Quaternion.Euler(90, m_Target_RedBall_1.transform.eulerAngles.y, 0).To<FLU>()
            orientation = Quaternion.Euler(90, 0, 0).To<FLU>()
        };
        // Pick Pose redball_2
        request.pick_pose_redball_2 = new PoseMsg
        {
            //position = (m_Target_RedBall_2.transform.position + m_PickPoseOffset - Rob_offset).To<FLU>(),
            position = Vector3ToPointMsg(m_UR3.transform.InverseTransformPoint(m_Target_RedBall_2.transform.position + m_PickPoseOffset)),

            // The hardcoded x/z angles assure that the gripper is always positioned above the target cube before grasping.
            //orientation = Quaternion.Euler(90, m_Target_RedBall_2.transform.eulerAngles.y, 0).To<FLU>()
            orientation = Quaternion.Euler(90, 0, 0).To<FLU>()
        };

        // Place Pose basket 1
        request.Place_basket_1 = new PoseMsg
        {
            //position = (m_Placement_Basket_1.transform.position + m_PickPoseOffset - Rob_offset).To<FLU>(),
            position = Vector3ToPointMsg(m_UR3.transform.InverseTransformPoint(m_Placement_Basket_1.transform.position + m_PickPoseOffset)),
            //orientation = m_PickOrientation.To<FLU>()
            orientation = Quaternion.Euler(90, 0, 0).To<FLU>()
        };
        // Place Pose basket 2
        request.Place_basket_2 = new PoseMsg
        {
            //position = (m_Placement_Basket_2.transform.position + m_PickPoseOffset - Rob_offset).To<FLU>(),
            position =Vector3ToPointMsg(m_UR3.transform.InverseTransformPoint(m_Placement_Basket_2.transform.position + m_PickPoseOffset)),
            //orientation = m_PickOrientation.To<FLU>()
            orientation = Quaternion.Euler(90, 0, 0).To<FLU>()
        };

        m_Ros.SendServiceMessage<MoverServiceResponse>(m_RosServiceName, request, TrajectoryResponse);
    }

    void TrajectoryResponse(MoverServiceResponse response)
    {
        if (response.trajectories.Length > 0)
        {
            Debug.Log("Trajectory returned.");
            StartCoroutine(ExecuteTrajectories(response));
        }
        else
        {
            Debug.LogError("No trajectory returned from MoverService.");
        }
    }

    /// <summary>
    ///     Execute the returned trajectories from the MoverService.
    ///     The expectation is that the MoverService will return four trajectory plans,
    ///     PreGrasp, Grasp, PickUp, and Place,
    ///     where each plan is an array of robot poses. A robot pose is the joint angle values
    ///     of the six robot joints.
    ///     Executing a single trajectory will iterate through every robot pose in the array while updating the
    ///     joint values on the robot.
    /// </summary>
    /// <param name="response"> MoverServiceResponse received from UR3_moveit mover service running in ROS</param>
    /// <returns></returns>
    IEnumerator ExecuteTrajectories(MoverServiceResponse response)
    {
        if (response.trajectories != null)
        {
            // For every trajectory plan returned
            for (var poseIndex = 0; poseIndex < response.trajectories.Length; poseIndex++)
            {
                // For every robot pose in trajectory plan
                foreach (var t in response.trajectories[poseIndex].joint_trajectory.points)
                {
                    var jointPositions = t.positions;
                    var result = jointPositions.Select(r => (float)r * Mathf.Rad2Deg).ToArray();

                    // Set the joint values for every joint
                    for (var joint = 0; joint < m_JointArticulationBodies.Length; joint++)
                    {
                        var joint1XDrive = m_JointArticulationBodies[joint].xDrive;
                        joint1XDrive.target = result[joint];
                        m_JointArticulationBodies[joint].xDrive = joint1XDrive;
                    }

                    // Wait for robot to achieve pose for all joint assignments
                    yield return new WaitForSeconds(k_JointAssignmentWait);
                }
                Debug.Log("Current Pose: " + ((Poses)poseIndex).ToString());
                // Close the gripper if completed executing the trajectory for the Grasp pose
                if (poseIndex == (int)Poses.Grasp)
                {
                    CloseGripper();
                    Rigidbody targetRigidbody = m_Target_BlueBall_1.GetComponent<Rigidbody>();
                    targetRigidbody.isKinematic = true;
                    m_Target_BlueBall_1.transform.parent = m_LeftGripper.transform;
                    Debug.Log("Gravity On: Gravity enabled for ball 1");
                }
                if (poseIndex == (int)Poses.Place)
                {
                    OpenGripper();
                    m_Target_BlueBall_1.transform.parent = null;
                    Rigidbody targetRigidbody = m_Target_BlueBall_1.GetComponent<Rigidbody>();
                    targetRigidbody.isKinematic = false;                   
                }
                if (poseIndex == (int)Poses.Grasp2)
                {
                    CloseGripper();
                    Rigidbody targetRigidbody = m_Target_BlueBall_2.GetComponent<Rigidbody>();
                    targetRigidbody.isKinematic = true;
                    m_Target_BlueBall_2.transform.parent = m_LeftGripper.transform;
                    Debug.Log("Gravity On: Gravity enabled for ball 2");
                }
                if (poseIndex == (int)Poses.Place2)
                {
                    OpenGripper();
                    m_Target_BlueBall_2.transform.parent = null;
                    Rigidbody targetRigidbody = m_Target_BlueBall_2.GetComponent<Rigidbody>();
                    targetRigidbody.isKinematic = false;                   
                }
                if (poseIndex == (int)Poses.Grasp3)
                {
                    CloseGripper();
                    Rigidbody targetRigidbody = m_Target_RedBall_1.GetComponent<Rigidbody>();
                    targetRigidbody.isKinematic = true;
                    m_Target_RedBall_1.transform.parent = m_LeftGripper.transform;
                    Debug.Log("Gravity On: Gravity enabled for ball 2");
                }
                if (poseIndex == (int)Poses.Place3)
                {
                    OpenGripper();
                    m_Target_RedBall_1.transform.parent = null;
                    Rigidbody targetRigidbody = m_Target_RedBall_1.GetComponent<Rigidbody>();
                    targetRigidbody.isKinematic = false;                   
                }
                 if (poseIndex == (int)Poses.Grasp4)
                {
                    CloseGripper();
                    Rigidbody targetRigidbody = m_Target_RedBall_2.GetComponent<Rigidbody>();
                    targetRigidbody.isKinematic = true;
                    m_Target_RedBall_2.transform.parent = m_LeftGripper.transform;
                    Debug.Log("Gravity On: Gravity enabled for ball 2");
                }
                if (poseIndex == (int)Poses.Place4)
                {
                    OpenGripper();
                    m_Target_RedBall_2.transform.parent = null;
                    Rigidbody targetRigidbody = m_Target_RedBall_2.GetComponent<Rigidbody>();
                    targetRigidbody.isKinematic = false;                   
                }
                
                // Wait for the robot to achieve the final pose from joint assignment
                yield return new WaitForSeconds(k_PoseAssignmentWait);
            }

            // All trajectories have been executed, open the gripper to place the target cube
            OpenGripper();
        }
    }

    enum Poses
    {
        PreGrasp,
        Grasp,
        PickUp,
        Place,

        PreGrasp2,
        Grasp2,
        PickUp2,
        Place2,

        PreGrasp3,
        Grasp3,
        PickUp3,
        Place3,

        PreGrasp4,
        Grasp4,
        PickUp4,
        Place4
    }
}
