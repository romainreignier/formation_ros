#include <cmath>
#include <iostream>
#include <sstream>

#include <gazebo/msgs/msgs.hh>
#include "TimerOverlayWidget.h"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(TimerOverlayWidget)

TimerOverlayWidget::TimerOverlayWidget()
  : GUIPlugin()
{
  // Set the frame background and foreground colors
  this->setStyleSheet(
      "QFrame { background-color : rgba(100, 100, 100, 255); color : white; }");

  // Create the main layout
  QHBoxLayout *mainLayout = new QHBoxLayout;

  // Create the frame to hold all the widgets
  QFrame *mainFrame = new QFrame();

  // Create the layout that sits inside the frame
  QHBoxLayout *frameLayout = new QHBoxLayout();

  QLabel *label = new QLabel(tr("Lap Time:"));

  // Create a time label
  QLabel *timeLabel = new QLabel(tr("00:00.00"));

  // Add the label to the frame's layout
  frameLayout->addWidget(label);
  frameLayout->addWidget(timeLabel);
  connect(this, SIGNAL(SetTimerTime(QString)),
      timeLabel, SLOT(setText(QString)), Qt::QueuedConnection);

  // Add frameLayout to the frame
  mainFrame->setLayout(frameLayout);

  // Add the frame to the main layout
  mainLayout->addWidget(mainFrame);

  // Remove margins to reduce space
  frameLayout->setContentsMargins(4, 4, 4, 4);
  mainLayout->setContentsMargins(0, 0, 0, 0);

  this->setLayout(mainLayout);

  // Position and resize this widget
  this->move(20, 10);
  this->resize(120, 30);

  // Create a node for transportation
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
}

TimerOverlayWidget::~TimerOverlayWidget()
{
    this->queue.clear();
    this->callbackQueueThread.join();
}

void TimerOverlayWidget::Load(sdf::ElementPtr _elem)
{
    ROS_INFO("Timer Widget loaded");

    // initialize ros
    if(!ros::isInitialized())
    {
      int argc = 0;
      char** argv;
      ros::init(argc, argv, "gazebo_gui", ros::init_options::NoSigintHandler);
    }

    this->nh = std::make_unique<ros::NodeHandle>();

    this->nh->setCallbackQueue(&this->queue);

    lapTimeSub = this->nh->subscribe("/lap_time", 1, &TimerOverlayWidget::TimerCb, this);

    this->callbackQueueThread = std::thread{&TimerOverlayWidget::QueueThread, this};
}

void TimerOverlayWidget::TimerCb(const std_msgs::Float64& _msg)
{
    this->SetTimerTime(QString::fromStdString(
        this->FormatTime(_msg.data)));
}

void TimerOverlayWidget::QueueThread()
{
    static const double timeout = 0.01;
    while(this->nh->ok())
    {
        this->queue.callAvailable(ros::WallDuration(timeout));
    }
}

std::string TimerOverlayWidget::FormatTime(const double& _sec) const
{
  std::ostringstream stream;
  unsigned int min, sec, msec;

  stream.str("");

  double secd, msecd;
  msecd = std::modf(_sec, &secd);
  
  sec = static_cast<unsigned int>(secd);
  msec = static_cast<unsigned int>(msecd * 1000);

  min = sec / 60;
  sec -= min * 60;

  stream << std::setw(2) << std::setfill('0') << min << ":";
  stream << std::setw(2) << std::setfill('0') << sec << ".";
  stream << std::setw(3) << std::setfill('0') << msec;

  return stream.str();
}