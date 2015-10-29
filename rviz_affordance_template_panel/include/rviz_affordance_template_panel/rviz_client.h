#ifndef AFFORDANCE_TEMPLATE_RVIZ_CLIENT_HPP
#define AFFORDANCE_TEMPLATE_RVIZ_CLIENT_HPP

/* ROS Includes */
#include <ros/ros.h>
#include <rviz/panel.h>
#include <ros/package.h>

/* stuff for getting urdf */
#include <urdf/model.h>
#include <kdl/frames.hpp>

/* qt */
#include <QGraphicsScene>
#include <QTableWidgetItem>
#include <QSlider>

/* Project Include */
#include <rviz_affordance_template_panel/affordance.h>
#include <rviz_affordance_template_panel/RecognitionObject.hpp>
#include <rviz_affordance_template_panel/robot_config.h>
#include <rviz_affordance_template_panel/controls.h>
#include <rviz_affordance_template_panel/util.h>
#include "ui_rviz_affordance_template_panel.h"

#include <geometry_msgs/Pose.h>

#include <rviz_affordance_template_panel/msg_headers.h>
#include <rviz_affordance_template_panel/template_status_info.h>
#include <rviz_affordance_template_panel/server_status_monitor.h>

namespace Ui {
class RVizAffordanceTemplatePanel;
}

namespace rviz_affordance_template_panel
{
    class AffordanceTemplateRVizClient 
    {
    public:

        // typedefs
        typedef boost::shared_ptr<Affordance> AffordanceSharedPtr;
        typedef boost::shared_ptr<RecognitionObject> RecognitionObjectSharedPtr;
        typedef boost::shared_ptr<RobotConfig> RobotConfigSharedPtr;
        typedef boost::shared_ptr<EndEffectorConfig> EndEffectorConfigSharedPtr;
        typedef boost::shared_ptr<EndEffectorPoseConfig> EndEffectorPoseIDConfigSharedPtr;
        typedef boost::shared_ptr<Controls> ControlsSharedPtr;
        typedef std::pair<std::string, int> TemplateInstanceID;

        // Constructors
        AffordanceTemplateRVizClient(ros::NodeHandle &nh, Ui::RVizAffordanceTemplatePanel* ui, QGraphicsScene *at_scene, QGraphicsScene *ro_scene);
        ~AffordanceTemplateRVizClient();

        // init function
        void init();

        // thread functions
        void start();
        void stop();

        // print stored template status info
        void printTemplateStatus();
        
        // helper functions for front-end widgets
        void getAvailableInfo();
        void getAvailableTemplates();
        void getAvailableRecognitionObjects();
        void getAvailableRobots();
        void getRunningItems();

        void addAffordanceDisplayItem();
        void addObjectDisplayItem();
        void addTrajectory();

        void selectAffordanceTemplate(QListWidgetItem* item);
        void deleteAffordanceTemplate();
        void killAffordanceTemplate(QListWidgetItem* item);
        void killRecognitionObject(QListWidgetItem* item);

        void saveAffordanceTemplate();
        void refreshCallback();
        void loadConfig();
        void safeLoadConfig();

        void changeRobot(int id);
        void changeSaveInfo(int id);
        void changeEndEffector(int id);

        void goToStart();
        void goToEnd();
        void stepBackward();
        void stepForward();
        void goToCurrentWaypoint();
        void executePlan();

        void updateRobotConfig(const QString& text);
        void updateEndEffectorGroupMap(const QString&);
        void updateObjectScale(int value);
        void updateEndEffectorScaleAdjustment(int value);
        void selectScaleObject(const QString& object_name);
        void scaleSliderReleased();
        void controlStatusUpdate();
        void resetScale();      
        void enableConfigPanel(int state);
        void selectTemplateTrajectory(const QString& text);
        
    protected:

        void run_function();

        void setupRobotPanel(const string& key);
        void setupEndEffectorConfigPanel(const string& key);

        bool tryToLoadRobotFromYAML();

        void removeAffordanceTemplates();
        int  sendAffordanceTemplateAdd(const string& class_name);
        void sendAffordanceTemplateKill(const string& class_name, int id);
        void sendSaveAffordanceTemplate();
        void sendAddTrajectory();

        void sendObjectScale(affordance_template_msgs::ScaleDisplayObjectInfo scale_info);
        void streamObjectScale(affordance_template_msgs::ScaleDisplayObjectInfo scale_info);

        void removeRecognitionObjects();
        void sendRecognitionObjectAdd(const string& object_name);
        void sendRecognitionObjectKill(const string& object_name, int id);

        bool addAffordance(const AffordanceSharedPtr& obj);
        bool removeAffordance(const AffordanceSharedPtr& obj);
        bool checkAffordance(const AffordanceSharedPtr& obj);

        bool addRecognitionObject(const RecognitionObjectSharedPtr& obj);
        bool removeRecognitionObject(const RecognitionObjectSharedPtr& obj);
        bool checkRecognitionObject(const RecognitionObjectSharedPtr& obj);

        bool addRobot(const RobotConfigSharedPtr& obj);
        bool removeRobot(const RobotConfigSharedPtr& obj);
        bool checkRobot(const RobotConfigSharedPtr& obj);

        void updateServerStatus();
        void setLabelText(QColor color, std::string text);

        void updateStatusFromControls();   
        void updateTable(std::string name, std::string trajectory);
        void doCommand(Controls::CommandType command_type);
        void sendScaleInfo();
        void setupDisplayObjectSliders(TemplateInstanceID template_instance);
        bool endEffectorInTrajectory(AffordanceTemplateStatusInfo::EndEffectorInfo ee_info);
                
        std::string getRobotFromDescription();
        std::vector<std::string> getSelectedEndEffectors();

        // boost thread
        boost::thread *thread_;
        boost::mutex mutex;

        // status flag
        bool running_;
        int server_status_;
        
        // ros node handle
        ros::NodeHandle nh_;

        // UI reference
        Ui::RVizAffordanceTemplatePanel* ui_;

        // GUI Widgets
        QGraphicsScene* affordanceTemplateGraphicsScene_;
        QGraphicsScene* recognitionObjectGraphicsScene_;

        // server status monitor thread
        AffordanceTemplateServerStatusMonitor *server_monitor_;

        // map to track instantiated object templates
        std::map<std::string, AffordanceSharedPtr> affordanceMap_;
        std::map<std::string, RecognitionObjectSharedPtr> recognitionObjectMap_;
        std::map<std::string, RobotConfigSharedPtr> robotMap_;
        std::string descriptionRobot_;
        std::string robot_name_;
        bool robot_configured_;
        
        // affordance template services
        ros::ServiceClient add_template_client_;
        ros::ServiceClient delete_template_client_;
        ros::ServiceClient add_trajectory_client_;
        ros::ServiceClient add_object_client_;
        ros::ServiceClient delete_object_client_;
        ros::ServiceClient plan_command_client_;
        ros::ServiceClient execute_command_client_;
        ros::ServiceClient get_robots_client_;
        ros::ServiceClient get_running_client_;
        ros::ServiceClient get_templates_client_;
        ros::ServiceClient get_objects_client_;
        ros::ServiceClient load_robot_client_;
        ros::ServiceClient save_template_client_;
        ros::ServiceClient scale_object_client_;
        ros::ServiceClient get_template_status_client_;
        ros::ServiceClient set_template_trajectory_client_;

        // affordance template publishers
        ros::Publisher scale_object_streamer_;

        // control helper class
        ControlsSharedPtr controls_;

        // template bookkeeping
        TemplateInstanceID selected_template;
        std::map<std::pair<TemplateInstanceID, std::string>, int> display_object_scale_map;
        std::map<std::pair<TemplateInstanceID, std::string>, int> end_effector_adjustment_map;
        std::map<std::string, AffordanceTemplateStatusInfo*> template_status_info; 

        // extra graphics stuff
        QPalette *label_palette_;
        QColor red, blue, green;

    };
}
#endif // AFFORDANCE_TEMPLATE_RVIZ_CLIENT_HPP