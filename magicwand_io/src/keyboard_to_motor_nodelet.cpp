// ros includes
#include <ros/ros.h>

//Nodelet includes
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

//keyboard message include
#include <keyboard/Key.h>
#include <magicwand_msgs/Motor.h>

#include <boost/thread.hpp>
#include <cmath>

namespace mw
{

enum Key
{
  UP        = 273,
  DOWN      = 274,
  RIGHT     = 275,
  LEFT      = 276,
  SPACE     = 32,
  ENTER     = 13,
  BACKSPACE = 8,
  KEY_0     = 48,
  KEY_1     = 49,
  KEY_2     = 50,
  KEY_3     = 51,
  KEY_4     = 52,
  KEY_5     = 53,
  KEY_6     = 54,
  KEY_l     = 108,
  KEY_W     = 119,
  KEY_S     = 115,
};

class KeyboardToMotor : public nodelet::Nodelet
{
public:
  virtual void onInit();
  void downCallback(const keyboard::KeyConstPtr &msg);
  void upCallback(const keyboard::KeyConstPtr &msg);
  magicwand_msgs::Motor generateCommandVelocity(const double &left_vel, const double &right_vel);
  void loop();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber down_sub_, up_sub_;
  ros::Publisher command_pub_; //right_command_pub_;

  boost::shared_ptr<boost::thread> loop_thread_;

  magicwand_msgs::Motor command_; //right_command_;
  double max_vel_, vel_increment_;
};

void KeyboardToMotor::onInit()
{
  nh_ = getNodeHandle();
  pnh_ = getPrivateNodeHandle();
  //Get rosparam
  pnh_.getParam("max_vel", max_vel_);
  pnh_.getParam("vel_increment", vel_increment_);

  down_sub_ = nh_.subscribe("/keyboard/keydown", 1, &KeyboardToMotor::downCallback, this);
  up_sub_ = nh_.subscribe("/keyboard/keyup", 1, &KeyboardToMotor::upCallback, this);

  //Generate Publisher
  command_pub_ = nh_.advertise<magicwand_msgs::Motor>("/motor_command",10);
  // right_command_pub_ = nh_.advertise<magicwand_msgs::Motor>("/right_command",10);

  command_ = generateCommandVelocity(0.0, 0.0);
  // right_command_ = generateCommandVelocity(0.0, 0.0);

  //loop process
  loop_thread_ = boost::shared_ptr<boost::thread>
          (new boost::thread(boost::bind(&KeyboardToMotor::loop, this)));
}

magicwand_msgs::Motor KeyboardToMotor::generateCommandVelocity(const double &left_vel, const double &right_vel)
{
  magicwand_msgs::Motor command;
  command.slow_stop.data = false;
  command.fast_stop.data = false;

  if (std::fabs(left_vel) > max_vel_)
  {
    if (left_vel < 0)
    {
      command.left_vel = -max_vel_;
    }
    else
    {
      command.left_vel = max_vel_;
    }
  }
  else
  {
    command.left_vel = left_vel;
  }

  if (std::fabs(right_vel) > max_vel_)
  {
    if (right_vel < 0)
    {
      command.right_vel = -max_vel_;
    }
    else
    {
      command.right_vel = max_vel_;
    }
  }
  else
  {
    command.right_vel = right_vel;
  }

  return command;
}

void KeyboardToMotor::downCallback(const keyboard::KeyConstPtr &msg)
{
  int key_code = msg->code;
  switch (key_code)
  {
    case Key::ENTER:
      command_.left_vel = 0.0;
      command_.right_vel = 0.0;
      // right_command_.vel = 0.0;
      command_.fast_stop.data = true;
      // right_command_.fast_stop.data = true;
      command_.slow_stop.data = false;
      // right_command_.slow_stop.data = false;
      break;
    case Key::SPACE:
      command_.left_vel = 0.0;
      command_.right_vel = 0.0;
      // right_command_.vel = 0.0;
      command_.fast_stop.data = false;
      // right_command_.fast_stop.data = false;
      command_.slow_stop.data = true;
      // right_command_.slow_stop.data = true;
      break;
    case Key::UP:
      command_ = generateCommandVelocity(command_.left_vel, command_.right_vel + vel_increment_);
      break;
    case Key::DOWN:
      command_ = generateCommandVelocity(command_.left_vel, command_.right_vel - vel_increment_);
      break;
    case Key::KEY_W:
      command_ = generateCommandVelocity(command_.left_vel + vel_increment_, command_.right_vel);
      break;
    case Key::KEY_S:
      command_ = generateCommandVelocity(command_.left_vel - vel_increment_, command_.right_vel);
      break;
  }
}

void KeyboardToMotor::upCallback(const keyboard::KeyConstPtr &msg)
{
}

void KeyboardToMotor::loop()
{
  ros::Rate loop_rate(30);

  while(getMTPrivateNodeHandle().ok())
  {
    command_.header.stamp = ros::Time::now();
    // right_command_.header.stamp = ros::Time::now();
    command_pub_.publish(command_);
    // // right_command_pub_.publish(right_command_);

    loop_rate.sleep();
  }
}

}//namespace mw

PLUGINLIB_EXPORT_CLASS(mw::KeyboardToMotor, nodelet::Nodelet)
