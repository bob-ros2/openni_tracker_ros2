#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <kdl/frames.hpp>
#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>

xn::Context        g_Context;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator  g_UserGenerator;

XnBool g_bNeedPose   = FALSE;
XnChar g_strPose[20] = "";

const char* g_package_name = "openni_tracker_ros2";
std::shared_ptr<rclcpp::Node> g_node;

void XN_CALLBACK_TYPE 
User_NewUser(
    xn::UserGenerator& generator, 
    XnUserID nId, 
    void* pCookie)
{
    RCLCPP_INFO(g_node->get_logger(), "New User %d", nId);

    if (g_bNeedPose)
        g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(
            g_strPose, nId);
    else
        g_UserGenerator.GetSkeletonCap().RequestCalibration(
            nId, TRUE);
}

void XN_CALLBACK_TYPE 
User_LostUser(
    xn::UserGenerator& generator, 
    XnUserID nId, 
    void* pCookie)
{
    RCLCPP_INFO(g_node->get_logger(), 
        "Lost user %d", nId);
}

void XN_CALLBACK_TYPE 
UserCalibration_CalibrationStart(
    xn::SkeletonCapability& capability, 
    XnUserID nId, 
    void* pCookie)
{
    RCLCPP_INFO(g_node->get_logger(), 
        "Calibration started for user %d", nId);
}

void XN_CALLBACK_TYPE 
UserCalibration_CalibrationEnd(
    xn::SkeletonCapability& capability, 
    XnUserID nId, 
    XnBool bSuccess, 
    void* pCookie)
{
    if (bSuccess) {
        RCLCPP_INFO(g_node->get_logger(), 
            "Calibration complete, start tracking user %d", nId);
        g_UserGenerator.GetSkeletonCap().StartTracking(nId);
    }
    else {
        RCLCPP_INFO(g_node->get_logger(), 
            "Calibration failed for user %d", nId);
        if (g_bNeedPose)
            g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(
                g_strPose, nId);
        else
            g_UserGenerator.GetSkeletonCap().RequestCalibration(
                nId, TRUE);
    }
}

void XN_CALLBACK_TYPE 
UserPose_PoseDetected(
    xn::PoseDetectionCapability& capability, 
    XnChar const* strPose, 
    XnUserID nId, 
    void* pCookie) 
{
    RCLCPP_INFO(g_node->get_logger(), 
        "Pose %s detected for user %d", strPose, nId);
    g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
    g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}

void publishTransform(
    XnUserID const& user, 
    XnSkeletonJoint const& joint, 
    std::string const& frame_id, 
    std::string const& child_frame_id) 
{
    static std::unique_ptr<tf2_ros::TransformBroadcaster> br = 
        std::make_unique<tf2_ros::TransformBroadcaster>(g_node);

    XnSkeletonJointPosition joint_position;
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(
        user, joint, joint_position);
    double x = -joint_position.position.X / 1000.0;
    double y = joint_position.position.Y / 1000.0;
    double z = joint_position.position.Z / 1000.0;

    XnSkeletonJointOrientation joint_orientation;
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(
        user, joint, joint_orientation);

    XnFloat* m = joint_orientation.orientation.elements;
    KDL::Rotation rotation(m[0], m[1], m[2],
                           m[3], m[4], m[5],
                           m[6], m[7], m[8]);
    double qx, qy, qz, qw;
    rotation.GetQuaternion(qx, qy, qz, qw);

    char child_frame_no[128];
    snprintf(child_frame_no, sizeof(child_frame_no), 
        "%s_%d", child_frame_id.c_str(), user);

    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(x, y, z));
    transform.setRotation(tf2::Quaternion(qx, -qy, -qz, qw));

    tf2::Transform change_frame;
    change_frame.setOrigin(tf2::Vector3(0, 0, 0));
    tf2::Quaternion frame_rotation;
    frame_rotation.setRPY(1.5708, 0, 1.5708);
    change_frame.setRotation(frame_rotation);

    transform = change_frame * transform;

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = g_node->get_clock()->now();
    t.header.frame_id = frame_id;
    t.child_frame_id = child_frame_no;

    t.transform.translation.x = transform.getOrigin().getX();
    t.transform.translation.y = transform.getOrigin().getY();
    t.transform.translation.z = transform.getOrigin().getZ();

    t.transform.rotation.w = transform.getRotation().getW();
    t.transform.rotation.x = transform.getRotation().getAxis().getX();
    t.transform.rotation.y = transform.getRotation().getAxis().getY();
    t.transform.rotation.z = transform.getRotation().getAxis().getZ();

    br->sendTransform(t);
}

void publishTransforms(const std::string& frame_id)
{
    XnUserID users[15];
    XnUInt16 users_count = 15;
    g_UserGenerator.GetUsers(users, users_count);

    for (int i = 0; i < users_count; ++i)
    {
        XnUserID user = users[i];
        if (!g_UserGenerator.GetSkeletonCap().IsTracking(user))
            continue;

        publishTransform(user, XN_SKEL_HEAD,           frame_id, "head");
        publishTransform(user, XN_SKEL_NECK,           frame_id, "neck");
        publishTransform(user, XN_SKEL_TORSO,          frame_id, "torso");

        publishTransform(user, XN_SKEL_LEFT_SHOULDER,  frame_id, "left_shoulder");
        publishTransform(user, XN_SKEL_LEFT_ELBOW,     frame_id, "left_elbow");
        publishTransform(user, XN_SKEL_LEFT_HAND,      frame_id, "left_hand");

        publishTransform(user, XN_SKEL_RIGHT_SHOULDER, frame_id, "right_shoulder");
        publishTransform(user, XN_SKEL_RIGHT_ELBOW,    frame_id, "right_elbow");
        publishTransform(user, XN_SKEL_RIGHT_HAND,     frame_id, "right_hand");

        publishTransform(user, XN_SKEL_LEFT_HIP,       frame_id, "left_hip");
        publishTransform(user, XN_SKEL_LEFT_KNEE,      frame_id, "left_knee");
        publishTransform(user, XN_SKEL_LEFT_FOOT,      frame_id, "left_foot");

        publishTransform(user, XN_SKEL_RIGHT_HIP,      frame_id, "right_hip");
        publishTransform(user, XN_SKEL_RIGHT_KNEE,     frame_id, "right_knee");
        publishTransform(user, XN_SKEL_RIGHT_FOOT,     frame_id, "right_foot");
    }
}

#define CHECK_RC(nRetVal, what)                                     \
    if (nRetVal != XN_STATUS_OK)                                    \
    {                                                               \
        RCLCPP_INFO(g_node->get_logger(),                           \
            "%s failed: %s", what, xnGetStatusString(nRetVal));     \
        return nRetVal;                                             \
    }

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("openni_tracker");

    std::string package_share = 
        ament_index_cpp::get_package_share_directory(g_package_name);
    std::string configFilename = package_share + "/openni_tracker.xml";
    XnStatus nRetVal = g_Context.InitFromXmlFile(configFilename.c_str());
    CHECK_RC(nRetVal, "InitFromXml");

    nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
    CHECK_RC(nRetVal, "Find depth generator");

    nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
    if (nRetVal != XN_STATUS_OK)
    {
        nRetVal = g_UserGenerator.Create(g_Context);
        if (nRetVal != XN_STATUS_OK)
        {
            RCLCPP_INFO(g_node->get_logger(), 
                "NITE is likely missing: Please install NITE >= 1.5.2.21. "
                "Check the readme for download information. Error Info: User "
                "generator failed: %s", xnGetStatusString(nRetVal));
            return nRetVal;
        }
    }

    if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON))
    {
        RCLCPP_INFO(g_node->get_logger(), 
            "Supplied user generator doesn't support skeleton");
        return 1;
    }

    XnCallbackHandle hUserCallbacks;
    g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, 
        NULL, hUserCallbacks);

    XnCallbackHandle hCalibrationCallbacks;
    g_UserGenerator.GetSkeletonCap().RegisterCalibrationCallbacks(
        UserCalibration_CalibrationStart, UserCalibration_CalibrationEnd, 
        NULL, hCalibrationCallbacks);

    if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration())
    {
        g_bNeedPose = TRUE;
        if (!g_UserGenerator.IsCapabilitySupported(
            XN_CAPABILITY_POSE_DETECTION))
        {
            RCLCPP_INFO(g_node->get_logger(), 
                "Pose required, but not supported");
            return 1;
        }

        XnCallbackHandle hPoseCallbacks;
        g_UserGenerator.GetPoseDetectionCap().RegisterToPoseCallbacks(
            UserPose_PoseDetected, NULL, NULL, hPoseCallbacks);

        g_UserGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose);
    }

    g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

    nRetVal = g_Context.StartGeneratingAll();
    CHECK_RC(nRetVal, "StartGenerating");

    g_node->declare_parameter("frame_id", "camera_depth_frame");
    std::string frame_id = g_node->get_parameter("frame_id").as_string();

    rclcpp::Rate r(30);
                
    while (rclcpp::ok())
    {
        g_Context.WaitAndUpdateAll();
        publishTransforms(frame_id);
        r.sleep();
    }

    g_Context.Shutdown();
    rclcpp::shutdown();

    return 0;
}
