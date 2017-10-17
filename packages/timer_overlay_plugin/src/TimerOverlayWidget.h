#pragma once

#include <memory>
#include <string>
#include <thread>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gui/GuiPlugin.hh>
#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <gazebo/transport/transport.hh>
# include <gazebo/gui/gui.hh>
#endif

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Float64.h>

namespace gazebo
{
  class GAZEBO_VISIBLE TimerOverlayWidget : public GUIPlugin
  {
    Q_OBJECT

    public:
        TimerOverlayWidget();

        virtual ~TimerOverlayWidget();
        
        void Load(sdf::ElementPtr _elem) override;
	
    signals:
        // A signal used to set the timer line edit.
        void SetTimerTime(QString _string);

    private:
        // Helper function to format time string.
        std::string FormatTime(const double& _sec) const;
        
        // ros "gateway"
        void QueueThread();
        
        // subscriber callback
        void TimerCb(const std_msgs::Float64& _msg);
        
  private:
	physics::WorldPtr world;
        // Node used to establish communication with gzserver.
        transport::NodePtr node;
        
        // ros stuffs
        std::unique_ptr<ros::NodeHandle> nh;
        ros::CallbackQueue queue;
        std::thread callbackQueueThread;
        
        // subscribers
        ros::Subscriber lapTimeSub;
  };
}