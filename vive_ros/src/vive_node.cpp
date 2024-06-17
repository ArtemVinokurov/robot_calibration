#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <std_srvs/srv/empty.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
// #include <Matrix3x3.h>
#include "vive_ros/vr_interface.h"
#include "tf2_ros/buffer.h"
#include <signal.h>
#include <iostream>
#include <cmath>
#include <memory>


std::string GetTrackedDeviceString( vr::IVRSystem *pHmd, vr::TrackedDeviceIndex_t unDevice, vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError = NULL )
{
  uint32_t unRequiredBufferLen = pHmd->GetStringTrackedDeviceProperty( unDevice, prop, NULL, 0, peError );
  if( unRequiredBufferLen == 0 )
    return "";

  char *pchBuffer = new char[ unRequiredBufferLen ];
  unRequiredBufferLen = pHmd->GetStringTrackedDeviceProperty( unDevice, prop, pchBuffer, unRequiredBufferLen, peError );
  std::string sResult = pchBuffer;
  delete [] pchBuffer;
  return sResult;
}

void mySigintHandler(int sig)
{
    rclcpp::shutdown();
}

void handleDebugMessages(const std::string &msg)
{
    //RCLCPP_DEBUG(logger_, "%s", msg.c_str());
    std::cout << msg.c_str() << std::endl;
}

void handleInfoMessages(const std::string &msg)
{
    //RCLCPP_DEBUG(logger_, "%s", msg.c_str());
    std::cout << msg.c_str() << std::endl;
}

void handleErrorMessages(const std::string &msg)
{
    //RCLCPP_DEBUG(logger_, "%s", msg.c_str());
    std::cout << msg.c_str() << std::endl;
}


class VIVEnode : public rclcpp::Node
{
    public:
        VIVEnode(int rate);
        ~VIVEnode();

        bool init();
        void run();
        void shutdown();
        void setOriginCB(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res);
        // void handleDebugMessages(const std::string &msg);
        // void handleInfoMessages(const std::string &msg);
        // void handleErrorMessages(const std::string &msg);
        //void set_feedback(sensor)
        VRInterface vr_;

    private:
        rclcpp::Rate loop_rate_;
        std::vector<double> world_offset_;
        double world_yaw_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr set_origin_srv_;
        rclcpp::Logger logger_;

        
        // rclcpp::Publisher<geometry_msgs::msg::TwistStamped> twist0_pub;
        // rclcpp::Publisher<geometry_msgs::msg::TwistStamped> twist1_pub;
        // rclcpp::Publisher<geometry_msgs::msg::TwistStamped> twist2_pub;
};


VIVEnode::VIVEnode(int rate) :
    Node("vive_node"),
    loop_rate_(rate),
    // tf_broadcaster_(),
    tf_listener_(),
    vr_(),
    world_offset_({0, 0, 0}),
    world_yaw_(0),
    logger_(this->get_logger())
{
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    world_offset_ = this->get_parameter("/vive/world_offset").as_double_array();
    world_yaw_ = this->get_parameter("/vive/world_yaw").as_double();
    RCLCPP_INFO(logger_, "WORLD OFFSET: [%2.3f, %2.3f, %2.3f] %2.3f", world_offset_[0], world_offset_[1], world_offset_[2], world_yaw_);
    set_origin_srv_ = create_service<std_srvs::srv::Empty>("/vive/set_origin", std::bind(&VIVEnode::setOriginCB, this, std::placeholders::_1, std::placeholders::_2));
    return;
}

VIVEnode::~VIVEnode()
{
    return;
}

// void VIVEnode::handleDebugMessages(const std::string &msg)
// {
//     RCLCPP_DEBUG(logger_, "%s", msg.c_str());
// }

// void VIVEnode::handleInfoMessages(const std::string &msg)
// {
//     RCLCPP_DEBUG(logger_, "%s", msg.c_str());
// }

// void VIVEnode::handleErrorMessages(const std::string &msg)
// {
//     RCLCPP_DEBUG(logger_, "%s", msg.c_str());
// }


bool VIVEnode::init()
{
    vr_.setDebugMsgCallback(handleDebugMessages);
    vr_.setInfoMsgCallback(handleInfoMessages);
    vr_.setErrorMsgCallback(handleErrorMessages);

    if (!vr_.init())
    {
        return false;
    }

    return true;

}

void VIVEnode::shutdown()
{
    vr_.shutdown();
}

void VIVEnode::setOriginCB(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
    double tf_matrix[3][4];
    int index = 1, dev_type;
    while (dev_type != 2)
    {
        dev_type = vr_.getDeviceMatrix(index++, tf_matrix);
    }

    if (dev_type == 0)
    {
        RCLCPP_WARN(logger_, "Couldn't find controller 1");
        return;
    }

    tf2::Matrix3x3 rot_matrix(tf_matrix[0][0], tf_matrix[0][1], tf_matrix[0][2],
                            tf_matrix[1][0], tf_matrix[1][1], tf_matrix[1][2],
                            tf_matrix[2][0], tf_matrix[2][1], tf_matrix[2][2]);

    tf2::Vector3 c_z = rot_matrix*tf2::Vector3(0,0,1);
    c_z[1] = 0;
    c_z.normalize();
    double new_yaw = acos(tf2::Vector3(0,0,1).dot(c_z)) + M_PI_2;

    if(c_z[0] < 0) new_yaw *= -1;
    world_yaw_ = -new_yaw;

    tf2::Vector3 new_offset;
    tf2::Matrix3x3 new_rot;
    new_rot.setRPY(0, 0, world_yaw_);
    new_offset = new_rot*tf2::Vector3(-tf_matrix[0][3], tf_matrix[2][3], -tf_matrix[1][3]);

    world_offset_[0] = new_offset[0];
    world_offset_[1] = new_offset[1];
    world_offset_[2] = new_offset[2];
    std::vector<rclcpp::Parameter> new_parameters{rclcpp::Parameter("/vive/world_offset", world_offset_), rclcpp::Parameter("/vive/world_yaw", world_yaw_)};
    this->set_parameters(new_parameters);
    RCLCPP_INFO(logger_, "NEW WORLD OFFSET: [%2.3f, %2.3f, %2.3f] %2.3f", world_offset_[0], world_offset_[1], world_offset_[2], world_yaw_);
    return;
}

void VIVEnode::run()
{
    double tf_matrix[3][4];
    int run_hz_count = 0;

    // while (rclcpp::ok())
    // {
        vr_.update();
        int controller_count = 1;
        int tracker_count = 1;
        int lighthouse_count = 1;
        for (int i = 0; i < vr::k_unMaxTrackedDeviceCount; i++)
        {
            int dev_type = vr_.getDeviceMatrix(i, tf_matrix);
            if (dev_type == 0) continue;

            tf2::Transform tf;
            tf.setOrigin(tf2::Vector3(tf_matrix[0][3], tf_matrix[1][3], tf_matrix[2][3]));

            tf2::Quaternion quat;

            tf2::Matrix3x3 rot_matrix(tf_matrix[0][0], tf_matrix[0][1], tf_matrix[0][2],
                                    tf_matrix[1][0], tf_matrix[1][1], tf_matrix[1][2],
                                    tf_matrix[2][0], tf_matrix[2][1], tf_matrix[2][2]);
            
            rot_matrix.getRotation(quat);
            // tf.setRotation(quat);

            geometry_msgs::msg::TransformStamped t;

            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = "world_vive";
            
            t.transform.translation.x = tf_matrix[0][3];
            t.transform.translation.y = tf_matrix[1][3];
            t.transform.translation.z = tf_matrix[2][3];

            t.transform.rotation.w = quat.getW();
            t.transform.rotation.x = quat.getX();
            t.transform.rotation.y = quat.getY();
            t.transform.rotation.z = quat.getZ();

            std::string cur_sn = GetTrackedDeviceString(vr_.pHMD_, i, vr::Prop_SerialNumber_String);
            std::replace(cur_sn.begin(), cur_sn.end(), '-', '_');


            if (dev_type == 3)
            {
                t.child_frame_id = "tracker_" + cur_sn;
                tf_broadcaster_->sendTransform(t);
            }

            if (dev_type == 4)
            {
                t.child_frame_id = "lighthouse_" + cur_sn;
                tf_broadcaster_->sendTransform(t);
            }
        }

       

        tf2::Quaternion quat_world;
        quat_world.setRPY(M_PI_2, 0, world_yaw_);

        geometry_msgs::msg::TransformStamped t_world;
        t_world.header.stamp = this->get_clock()->now();
        t_world.header.frame_id = "world";
        t_world.child_frame_id = "world_vive";

        t_world.transform.translation.x = world_offset_[0];
        t_world.transform.translation.y = world_offset_[1];
        t_world.transform.translation.z = world_offset_[2];

        t_world.transform.rotation.w = quat_world.getW();
        t_world.transform.rotation.x = quat_world.getX();
        t_world.transform.rotation.y = quat_world.getY();
        t_world.transform.rotation.z = quat_world.getZ();

        tf_broadcaster_->sendTransform(t_world);

        auto& clk = *this->get_clock();
        RCLCPP_INFO_THROTTLE(logger_, clk, 1000, "Run() @ %d [fps]", [](int& cin){int ans = cin; cin = 0; return ans;}(run_hz_count));
        run_hz_count++;
        loop_rate_.sleep();
    }
//}



int main(int argc, char ** argv)
{   
    signal(SIGINT, mySigintHandler);
    rclcpp::init(argc, argv);
   auto node = std::make_shared<VIVEnode>(30);
    // rclcpp::spin(std::make_shared<VIVEnode>(30));
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    while (rclcpp::ok())
    {
        node->run();
        executor.spin_once();
    }

    rclcpp::shutdown();
    return 0;
    
}





