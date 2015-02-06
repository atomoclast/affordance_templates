#ifndef RVIZ_AFFORDANCE_TEMPLATE_PANEL_HPP
#define RVIZ_AFFORDANCE_TEMPLATE_PANEL_HPP

/* ROS Includes */
#include <ros/ros.h>
#include <rviz/panel.h>
#include <ros/package.h>
#include <urdf/model.h>
#include <kdl/frames.hpp>

/* qt */
#include <QGraphicsScene>
#include <QTableWidgetItem>
#include <QSlider>

/* Project Include */
#include "Affordance.hpp"
#include "RecognitionObject.hpp"
#include "RobotConfig.hpp"
#include "Controls.hpp"
#include "util.hpp"
#include "ui_rviz_affordance_template_panel.h"

#include "AffordanceTemplateStatusInfo.hpp"

#include <geometry_msgs/Pose.h>

// affordance template messages and services
#include <affordance_template_msgs/AffordanceTemplateConfig.h>
#include <affordance_template_msgs/RecognitionObjectConfig.h>
#include <affordance_template_msgs/EndEffectorConfig.h>
#include <affordance_template_msgs/EndEffectorPoseData.h>
#include <affordance_template_msgs/RobotConfig.h>
#include <affordance_template_msgs/WaypointInfo.h>
#include <affordance_template_msgs/WaypointTrajectory.h>

#include <affordance_template_msgs/AddAffordanceTemplate.h>
#include <affordance_template_msgs/AddAffordanceTemplateTrajectory.h>
#include <affordance_template_msgs/SaveAffordanceTemplate.h>
#include <affordance_template_msgs/AddRecognitionObject.h>
#include <affordance_template_msgs/AffordanceTemplatePlanCommand.h>
#include <affordance_template_msgs/AffordanceTemplateExecuteCommand.h>
#include <affordance_template_msgs/DeleteAffordanceTemplate.h>
#include <affordance_template_msgs/DeleteRecognitionObject.h>
#include <affordance_template_msgs/GetAffordanceTemplateConfigInfo.h>
#include <affordance_template_msgs/GetRecognitionObjectConfigInfo.h>
#include <affordance_template_msgs/GetRobotConfigInfo.h>
#include <affordance_template_msgs/GetRunningAffordanceTemplates.h>
#include <affordance_template_msgs/GetAffordanceTemplateStatus.h>
#include <affordance_template_msgs/LoadRobotConfig.h>
#include <affordance_template_msgs/ScaleDisplayObjectInfo.h>
#include <affordance_template_msgs/ScaleDisplayObject.h>

namespace Ui {
class RVizAffordanceTemplatePanel;
}

namespace rviz_affordance_template_panel
{
    class RVizAffordanceTemplatePanel : public rviz::Panel
    {
        Q_OBJECT
    public:
        typedef boost::shared_ptr<Affordance> AffordanceSharedPtr;
        typedef boost::shared_ptr<RecognitionObject> RecognitionObjectSharedPtr;
        typedef boost::shared_ptr<RobotConfig> RobotConfigSharedPtr;
        typedef boost::shared_ptr<EndEffectorConfig> EndEffectorConfigSharedPtr;
        typedef boost::shared_ptr<EndEffectorPoseConfig> EndEffectorPoseIDConfigSharedPtr;
        typedef boost::shared_ptr<Controls> ControlsSharedPtr;

        typedef std::pair<std::string, int> TemplateInstanceID;

        // Constructors
        RVizAffordanceTemplatePanel(QWidget* parent = 0);
        ~RVizAffordanceTemplatePanel();

        // Emit configuration change event.
        void configChanged();

        // print stored template status info
        void print_template_status();

    public Q_SLOTS:

        /** \brief add an affordance template icon to panel and store.
         */
        void addAffordanceDisplayItem();

        /** \brief add a recognition object icon to panel and store.
         */
        void addObjectDisplayItem();

        /** \brief save an affordance template with info from gui panel.
         */
        void saveAffordanceTemplate();

        /** \brief adds a new trajectory to the selected template info from gui panel.
         */
        void addTrajectory();

        /** \brief Send a request to get available template and robot classes and populate the panel.
         */
        void getAvailableInfo();

        /** \brief Send a request to get available template classes.
         */
        void getAvailableTemplates();

        /** \brief Send a request to get available recog object classes.
         */
        void getAvailableRecognitionObjects();

        /** \brief Send a request to get available robot config info.
         */
        void getAvailableRobots();

        /** \brief Send a request to get running templates and recognition objects on the server.
         */
        void getRunningItems();

        /** \brief refresh button callback
         */
        void refreshCallback();

        /** \brief Load Robot Config.
         */
        void loadConfig();

        /** \brief Load Robot Config.
         */
        void safeLoadConfig();

        /** \brief Robot Selection callback.
         */
        void changeRobot(int id);

        /** \brief Save As Template Selection callback.
         */
        void changeSaveInfo(int id);

        /** \brief Robot Config End Effector Selection callback.
         */
        void changeEndEffector(int id);

        /** \brief Delete Template button.
         */
        void deleteButton();

        /** \brief Save Template button.
         */
        void saveButton();

        /** \brief Reset Scale button.
         */
        void resetScaleButton();

        /** \brief Add Trajectory button.
         */
        void addTrajectoryButton();

        /** \brief Send a service request to kill a running template.
         */
        void killAffordanceTemplate(QListWidgetItem* item);

        /** \brief Send a service request to kill a running object recog.
         */
        void killRecognitionObject(QListWidgetItem* item);

        /** \brief user selected an item in the output window.
         */
        void selectAffordanceTemplate(QListWidgetItem* item);

        /** \brief Go To Start Command.
         */
        void go_to_start();

        // /** \brief Go To End Command.
        //  */
        void go_to_end();

        // /** \brief Stop Command.
        //  */
        //void stop();

        // /** \brief Step Backward Command.
        //  */
        void step_backward();

        // /** \brief Step Forward Command.
        //  */
        void step_forward();

        // /** \brief Go to Current.
        //  */
        void go_to_current_waypoint();

        // /** \brief Execute Command.
        //  */
        void execute_plan();

        // /** \brief Get Status Update.
        //  */
        void control_status_update();


        void updateObjectScale(int value);
        void updateEndEffectorScaleAdjustment(int value);
        void scaleSliderReleased();

        void enableConfigPanel(int state);
        void updateRobotConfig(const QString& text);
        void updateEndEffectorGroupMap(const QString&);

        void sendScaleInfo();
        void setupDisplayObjectSliders(TemplateInstanceID template_instance);
        void selectScaleObject(const QString& object_name);


    private:
        Ui::RVizAffordanceTemplatePanel* ui_;

        void setupWidgets();
        void setupRobotPanel(const string& key);
        void setupEndEffectorConfigPanel(const string& key);

        void removeAffordanceTemplates();
        int sendAffordanceTemplateAdd(const string& class_name);
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

        std::string getRobotFromDescription();
        std::vector<std::string> getSelectedEndEffectors();

        // GUI Widgets
        QGraphicsScene* affordanceTemplateGraphicsScene_;
        QGraphicsScene* recognitionObjectGraphicsScene_;

        // map to track instantiated object templates
        std::map<std::string, AffordanceSharedPtr> affordanceMap_;
        std::map<std::string, RecognitionObjectSharedPtr> recognitionObjectMap_;
        std::map<std::string, RobotConfigSharedPtr> robotMap_;
        std::string descriptionRobot_;
        std::string robot_name_;

        ros::NodeHandle nh_;

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

        // affordance template publishers
        ros::Publisher scale_object_streamer_;

        ControlsSharedPtr controls_;

    protected:
        
        std::map<std::pair<TemplateInstanceID, std::string>, int> display_object_scale_map;
        std::map<std::pair<TemplateInstanceID, std::string>, int> end_effector_adjustment_map;

        TemplateInstanceID selected_template;

        std::map<std::string, AffordanceTemplateStatusInfo*> template_status_info; 

    };
}
#endif // RVIZ_AFFORDANCE_TEMPLATE_PANEL_HPP
