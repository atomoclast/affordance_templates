#ifndef AFFORDANCE_TEMPLATE_SERVER_STATUS_MONITOR_HPP
#define AFFORDANCE_TEMPLATE_SERVER_STATUS_MONITOR_HPP

#include <ros/ros.h>
#include <iostream>

#include <QColorDialog>

#include <boost/thread.hpp> 
#include <boost/thread/mutex.hpp>

#include <affordance_template_msgs/GetAffordanceTemplateServerStatus.h>

namespace Ui {
class RVizAffordanceTemplatePanel;
}

namespace rviz_affordance_template_panel
{
    class AffordanceTemplateServerStatusMonitor
    {

	public:

		AffordanceTemplateServerStatusMonitor(ros::NodeHandle &nh, std::string srv_name, int update_rate=1) :
			nh_(nh),
			srv_name_(srv_name),
			available_(false),
			ready_(false),
			running_(false),
			update_rate_(update_rate) {

			srv_ = nh_.serviceClient<affordance_template_msgs::GetAffordanceTemplateServerStatus>(srv_name);

			label_palette = new QPalette();// ui_->server_status_label->palette();

		}

		~AffordanceTemplateServerStatusMonitor() {
			running_ = false;
			wait(2);
			stop();
		}

		void start() {
			monitor_thread_ = new boost::thread(boost::bind(&AffordanceTemplateServerStatusMonitor::thread, this));
		}

		void stop() {
			running_ = false;
			ROS_INFO("AffordanceTemplateServerStatusMonitor::stop()");
			monitor_thread_->join();
		}

		bool isReady() { return ready_; }
		bool isAvailable() { return available_; }

		void setUI(Ui::RVizAffordanceTemplatePanel *ui){
			ui_ = ui;
			updateUI();
		}

		void setLabelText(QColor color, std::string text) {
			label_palette->setColor(QPalette::WindowText,color);
			ui_->server_status_label->setPalette(*label_palette);
			ui_->server_status_label->setText(QString(text.c_str()));		
		}

    protected:

    	void wait(int seconds) { 
  			boost::this_thread::sleep(boost::posix_time::seconds(seconds)); 
		} 

		void thread() {
			running_ = true;
			available_ = false;
			ready_ = false;
			ROS_DEBUG("AffordanceTemplateServerStatusMonitor::thread() -- updating");
			while(running_) {
				{
					boost::unique_lock<boost::mutex> scoped_lock(mutex);
					available_ = false;
					ready_ = false;
					if(srv_.exists()) {
	 					available_ = true;
						ready_ = false;
						ROS_DEBUG("AffordanceTemplateServerStatusMonitor::thread() -- service exists");
						affordance_template_msgs::GetAffordanceTemplateServerStatus server_status;
	    				if (srv_.call(server_status))
	    				{
	 						ready_ = server_status.response.ready;
	        				ROS_DEBUG("AffordanceTemplateServerStatusMonitor::thread() -- got response: %d", (int)(server_status.response.ready)); 
	    				}
	    				else
	    				{
					        ROS_ERROR("AffordanceTemplateServerStatusMonitor::thread() -- service call failed");
						}
					} else {
						ROS_DEBUG("AffordanceTemplateServerStatusMonitor::thread() -- service does not exist");
					}
					updateUI();
				}
				wait(update_rate_);
			}

			ROS_INFO("AffordanceTemplateServerStatusMonitor::thread() -- not running anymore");
		}

		void updateUI() {
			if(isAvailable()) {
				if(isReady()) {
					setLabelText(Qt::green, std::string("READY"));
				} else {
					setLabelText(Qt::blue, std::string("NOT READY"));
				}	
			} else {
				setLabelText(Qt::red, std::string("UNAVAILABLE"));
			}
		}

		// boost thread
		boost::thread *monitor_thread_;
    	boost::mutex mutex;

    	// ros stuff
		ros::ServiceClient srv_;
    	std::string srv_name_;
    	ros::NodeHandle nh_;

    	// member functions
		int update_rate_;
		bool available_;
		bool ready_;
		bool running_;

		// panel stuff
		Ui::RVizAffordanceTemplatePanel* ui_;
		QPalette *label_palette;
		QColor red;
		QColor blue;
		QColor green;
    };
}

#endif // AFFORDANCE_TEMPLATE_SERVER_STATUS_MONITOR_HPP