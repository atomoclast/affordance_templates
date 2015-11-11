#ifndef RVIZ_AFFORDANCE_TEMPLATE_PANEL_HPP
#define RVIZ_AFFORDANCE_TEMPLATE_PANEL_HPP

/* ROS Includes */
#include <ros/ros.h>
#include <rviz/panel.h>
#include <ros/package.h>

#include "ui_rviz_affordance_template_panel.h"
#include <rviz_affordance_template_panel/rviz_client.h>

namespace Ui {
class RVizAffordanceTemplatePanel;
}

namespace rviz_affordance_template_panel
{
    class RVizAffordanceTemplatePanel : public rviz::Panel
    {
        Q_OBJECT
    public:

        // Constructors
        RVizAffordanceTemplatePanel(QWidget* parent = 0);
        ~RVizAffordanceTemplatePanel();

    public Q_SLOTS:

        // widget callback functions, wrapping client functions
        void addAffordanceDisplayItem() {
            client_->addAffordanceDisplayItem();
        }
        void addObjectDisplayItem() {
            client_->addObjectDisplayItem();
        }
        void selectAffordanceTemplate(QListWidgetItem* it) {
            client_->selectAffordanceTemplate(it);
        }
        void deleteAffordanceTemplate() {
            client_->deleteAffordanceTemplate();
        }
        void saveAffordanceTemplate() {
            client_->saveAffordanceTemplate();
        }
        void addTrajectory() {
            client_->addTrajectory();
        }
        void safeLoadConfig() {
            client_->safeLoadConfig();
        }
        void changeRobot(int d) {
            client_->changeRobot(d);
        }
        void changeSaveInfo(int d) {
            client_->changeSaveInfo(d);
        }
        void goToStart() {
            client_->goToStart();
        }
        void goToEnd() {
            client_->goToEnd();
        }
        void stepBackward() {
            client_->stepBackward();
        }
        void stepForward() {
            client_->stepForward();
        }
        void executePlan() {
            client_->executePlan();
        }
        void controlStatusUpdate() {
            client_->controlStatusUpdate();
        }
        void goToCurrentWaypoint() {
            client_->goToCurrentWaypoint();
        }
        void refreshCallback() {
            client_->refreshCallback();
        }
        void enableConfigPanel(int d) {
            client_->enableConfigPanel(d);
        }
        void updateRobotConfig(const QString& s) {
            client_->updateRobotConfig(s);
        }
        void updateEndEffectorGroupMap(const QString& s) {
            client_->updateEndEffectorGroupMap(s);
        }
        void updateObjectScale(int d) {
            client_->updateObjectScale(d);
        }
        void scaleSliderReleased() {
            client_->scaleSliderReleased();
        }
        void updateEndEffectorScaleAdjustment(int d) {
            client_->updateEndEffectorScaleAdjustment(d);
        }
        void resetScale() {
            client_->resetScale();
        }
        void selectScaleObject(const QString& s) {
            client_->selectScaleObject(s);
        }
        void selectTemplateTrajectory(const QString& s) {
            client_->selectTemplateTrajectory(s);
        }
      

    private:

        // UI pointer
        Ui::RVizAffordanceTemplatePanel* ui_;

        // setup widget function
        void setupWidgets();

        // GUI Widgets
        QGraphicsScene* affordanceTemplateGraphicsScene_;
        QGraphicsScene* recognitionObjectGraphicsScene_;

        // ros node handle
        ros::NodeHandle nh_;


    protected:
        
        // main client class
        AffordanceTemplateRVizClient *client_;

    };
}

#endif // RVIZ_AFFORDANCE_TEMPLATE_PANEL_HPP
