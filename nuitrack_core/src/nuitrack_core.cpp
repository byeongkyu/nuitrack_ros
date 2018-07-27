#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <nuitrack/Nuitrack.h>
#include <sensor_msgs/Image.h>
#include <nuitrack_msgs/UserData.h>
#include <nuitrack_msgs/UserDataArray.h>
#include <nuitrack_msgs/EventUserUpdate.h>

using namespace tdv::nuitrack;

namespace nodelet_nuitrack_ros
{

class NuitrackCore: public nodelet::Nodelet
{
public:
    NuitrackCore() {}

    virtual void onInit()
    {
        ros::NodeHandle& private_nh = getPrivateNodeHandle();
        pub_rgb_data_ = private_nh.advertise<sensor_msgs::Image>("/nuitrack/rgb/image_raw", 1);
        pub_user_data_ = private_nh.advertise<nuitrack_msgs::UserDataArray>("/nuitrack/detected_users", 10);
        pub_event_person_appeared_ = private_nh.advertise<nuitrack_msgs::EventUserUpdate>("/nuitrack/event_person_appeared", 10);
        pub_event_person_disappeared_ = private_nh.advertise<nuitrack_msgs::EventUserUpdate>("/nuitrack/event_person_disappeared", 10);

        try
        {
            Nuitrack::init();
        }
        catch (const Exception& e) {} // Do nothing

        colorSensor_ = ColorSensor::create();
        colorSensor_->connectOnNewFrame(std::bind(&NuitrackCore::onNewRGBFrame, this, std::placeholders::_1));

        OutputMode colorOutputMode = colorSensor_->getOutputMode();

        width_ = colorOutputMode.xres;
        height_ = colorOutputMode.yres;

        userTracker_ = UserTracker::create();
        userTracker_->connectOnNewUser(std::bind(&NuitrackCore::onNewUser, this, std::placeholders::_1));
        userTracker_->connectOnLostUser(std::bind(&NuitrackCore::onLostUser, this, std::placeholders::_1));
        userTracker_->connectOnUpdate(std::bind(&NuitrackCore::onUserUpdate, this, std::placeholders::_1));

        try
        {
            Nuitrack::run();
        }
        catch (const Exception& e)
        {
            std::cerr << "Can not start Nuitrack (ExceptionType: " << e.type() << ")" << std::endl;
            assert(false);
        }

        timer_ = private_nh.createTimer(ros::Duration(1/30), &NuitrackCore::timerCallback, this);
        NODELET_INFO("Initialized nuitrack_core...");
    }

    ~NuitrackCore()
    {
        try
        {
            timer_.stop();
            Nuitrack::release();
        }
        catch (const Exception& e) {} // Do nothing
    }

private:
    void timerCallback(const ros::TimerEvent& event)
    {
        try
        {
            Nuitrack::update(colorSensor_);
            Nuitrack::update(userTracker_);
        }
        catch (LicenseNotAcquiredException& e)
        {
            std::cerr << "LicenseNotAcquired exception (ExceptionType: " << e.type() << ")" << std::endl;
            assert(false);
        }
        catch (const Exception& e)
        {
            std::cerr << "Nuitrack update failed (ExceptionType: " << e.type() << ")" << std::endl;
            assert(false);
        }
    }

    void onNewUser(int id)
    {
        current_user_list_.push_back(id);

        nuitrack_msgs::EventUserUpdate msg;
        msg.key_id = id;
        msg.user_ids = current_user_list_;

        pub_event_person_appeared_.publish(msg);
    }

    void onLostUser(int id)
    {
        bool found_exist = false;
        for(size_t i = 0; i < current_user_list_.size(); i++)
        {
            if(current_user_list_[i] == id)
            {
                current_user_list_.erase(current_user_list_.begin() + i);
                found_exist = true;
                break;
            }
        }
        if(!found_exist)
            return;

        nuitrack_msgs::EventUserUpdate msg;
        msg.key_id = id;
        msg.user_ids = current_user_list_;

        pub_event_person_disappeared_.publish(msg);
    }

    void onUserUpdate(UserFrame::Ptr frame)
    {
        nuitrack_msgs::UserDataArray msg;
        std::vector<User> users = frame->getUsers();

        int width = frame->getCols();
        int height = frame->getRows();

        for(size_t i = 0; i < users.size(); i++)
        {
            nuitrack_msgs::UserData user;

            user.id = users[i].id;
            user.real.x = users[i].real.x;  user.real.y = users[i].real.y; user.real.z = users[i].real.z;
            user.proj.x = users[i].proj.x;  user.proj.y = users[i].proj.y; user.proj.z = users[i].proj.z;
            user.box.x_offset = users[i].box.left * width;
            user.box.y_offset = users[i].box.top * height;
            user.box.height = (users[i].box.bottom - users[i].box.top) * height;
            user.box.width = (users[i].box.right - users[i].box.left) * width;
            user.occlusion = users[i].occlusion;

            msg.users.push_back(user);
        }

        pub_user_data_.publish(msg);
    }

    void onNewRGBFrame(RGBFrame::Ptr frame)
    {
        const Color3* colorPtr = frame->getData();

        sensor_msgs::Image img;
        img.header.stamp = ros::Time::now();

        img.width = frame->getCols();
        img.height = frame->getRows();
        img.encoding = "rgb8";
        img.is_bigendian = 0;
        img.step = 3 * frame->getCols();

        img.data.resize(img.width * img.height * 3);

        for(size_t i = 0; i < img.width * img.height; i++)
        {
            img.data[i * 3 + 0] = colorPtr[i].blue;
            img.data[i * 3 + 1] = colorPtr[i].green;
            img.data[i * 3 + 2] = colorPtr[i].red;
        }

        pub_rgb_data_.publish(img);
    }

private:
    ros::Timer timer_;
    ros::Publisher pub_rgb_data_;
    ros::Publisher pub_user_data_;
    ros::Publisher pub_event_person_appeared_;
    ros::Publisher pub_event_person_disappeared_;

    int width_;
    int height_;
    std::vector<int> current_user_list_;

    ColorSensor::Ptr colorSensor_;
    UserTracker::Ptr userTracker_;
    SkeletonTracker::Ptr skeletonTracker_;
    HandTracker::Ptr handTracker_;
    GestureRecognizer::Ptr gestureRecognizer_;
};

PLUGINLIB_EXPORT_CLASS(nodelet_nuitrack_ros::NuitrackCore, nodelet::Nodelet);
}