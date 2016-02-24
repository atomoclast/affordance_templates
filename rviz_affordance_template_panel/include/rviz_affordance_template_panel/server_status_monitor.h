#ifndef AFFORDANCE_TEMPLATE_SERVER_STATUS_MONITOR_HPP
#define AFFORDANCE_TEMPLATE_SERVER_STATUS_MONITOR_HPP

#include <ros/ros.h>

#include <boost/thread.hpp> 
#include <boost/thread/mutex.hpp>

#include <rviz_affordance_template_panel/msg_headers.h>

namespace Ui {
class RVizAffordanceTemplatePanel;
}

namespace rviz_affordance_template_panel {

  class AffordanceTemplateServerStatusMonitor  {

    public:

      AffordanceTemplateServerStatusMonitor(ros::NodeHandle &nh, std::string srv_name, int update_rate=1);
      ~AffordanceTemplateServerStatusMonitor();

      void start();
      void stop();

      inline bool isReady() { return ready_; }
      inline bool isAvailable() { return available_; }

    protected:

      void run_function();
      void wait(int seconds);

      // boost thread
      boost::scoped_ptr<boost::thread> monitor_thread_;
      boost::mutex mutex;

      // ros stuff
      ros::ServiceClient srv_;
      std::string srv_name_;
      ros::NodeHandle nh_;

      // member functions
      int update_rate_;

      // status vars
      bool available_;
      bool ready_;
      bool running_;
  };
}

#endif // AFFORDANCE_TEMPLATE_SERVER_STATUS_MONITOR_HPP