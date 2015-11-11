#include <rviz_affordance_template_panel/server_status_monitor.h>

using namespace rviz_affordance_template_panel;
using namespace std;

AffordanceTemplateServerStatusMonitor::AffordanceTemplateServerStatusMonitor(ros::NodeHandle &nh, std::string srv_name, int update_rate) :
	nh_(nh),
	srv_name_(srv_name),
	update_rate_(update_rate),
	available_(false),
	ready_(false),
	running_(false)
{
	srv_ = nh_.serviceClient<affordance_template_msgs::GetAffordanceTemplateServerStatus>(srv_name_);
}

AffordanceTemplateServerStatusMonitor::~AffordanceTemplateServerStatusMonitor() {
	stop();
}

void AffordanceTemplateServerStatusMonitor::start() {
	ROS_INFO("AffordanceTemplateServerStatusMonitor::start()");
	monitor_thread_ = new boost::thread(boost::bind(&AffordanceTemplateServerStatusMonitor::run_function, this));
}

void AffordanceTemplateServerStatusMonitor::stop() {
	ROS_INFO("AffordanceTemplateServerStatusMonitor::stop()");
	running_ = false;
	monitor_thread_->join();
}

void AffordanceTemplateServerStatusMonitor::wait(int seconds) { 
	boost::this_thread::sleep(boost::posix_time::seconds(seconds)); 
} 

void AffordanceTemplateServerStatusMonitor::run_function() {
	ROS_DEBUG("AffordanceTemplateServerStatusMonitor::run_function() -- updating");
	running_ = true;
	available_ = false;
	ready_ = false;
	while(running_) {
		{
			boost::unique_lock<boost::mutex> scoped_lock(mutex);
			available_ = false;
			ready_ = false;
			if(srv_.exists()) {
				available_ = true;
				ready_ = false;
				ROS_DEBUG("AffordanceTemplateServerStatusMonitor::run_function() -- service exists");
				affordance_template_msgs::GetAffordanceTemplateServerStatus server_status;
				if (srv_.call(server_status))
				{
					ready_ = server_status.response.ready;
    				ROS_DEBUG("AffordanceTemplateServerStatusMonitor::run_function() -- got response: %d", (int)(server_status.response.ready)); 
				}
				else
				{
			        ROS_DEBUG("AffordanceTemplateServerStatusMonitor::run_function() -- service call failed");
				}
			} else {
				ROS_DEBUG("AffordanceTemplateServerStatusMonitor::run_function() -- service does not exist");
			}
		}
		wait(update_rate_);
	}
	ROS_DEBUG("AffordanceTemplateServerStatusMonitor::run_function() -- not running anymore");
}

