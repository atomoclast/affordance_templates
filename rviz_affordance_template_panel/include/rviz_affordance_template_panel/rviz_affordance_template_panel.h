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
        RVizAffordanceTemplatePanel(QWidget* parent = 0);
        ~RVizAffordanceTemplatePanel();

    public Q_SLOTS:
        // widget callback functions, wrapping client functions
        inline void addAffordanceDisplayItem() {
            client_->addAffordanceDisplayItem();
        }
        inline void selectAffordanceTemplate(QListWidgetItem* it) {
            client_->selectAffordanceTemplate(it);
        }
        inline void deleteAffordanceTemplate() {
            client_->deleteAffordanceTemplate();
        }
        inline void saveAffordanceTemplate() {
            client_->saveAffordanceTemplate();
        }
        inline void addTrajectory() {
            client_->addTrajectory();
        }
        inline void safeLoadConfig() {
            client_->safeLoadConfig();
        }
        inline void changeRobot(int d) {
            client_->changeRobot(d);
        }
        inline void changeSaveInfo(int d) {
            client_->changeSaveInfo(d);
        }
        inline void goToStart() {
            client_->goToStart();
        }
        inline void goToEnd() {
            client_->goToEnd();
        }
        inline void stepBackward() {
            client_->stepBackward();
        }
        inline void stepForward() {
            client_->stepForward();
        }
        inline void executePlan() {
            client_->executePlan();
        }
        inline void controlStatusUpdate() {
            client_->controlStatusUpdate();
        }
        inline void goToCurrentWaypoint() {
            client_->goToCurrentWaypoint();
        }
        inline void refreshCallback() {
            client_->refreshCallback();
        }
        inline void enableConfigPanel(int d) {
            client_->enableConfigPanel(d);
        }
        inline void updateRobotConfig(const QString& s) {
            client_->updateRobotConfig(s);
        }
        inline void updateEndEffectorGroupMap(const QString& s) {
            client_->updateEndEffectorGroupMap(s);
        }
        inline void updateObjectScale(int d) {
            client_->updateObjectScale(d);
        }
        inline void scaleSliderReleased() {
            client_->scaleSliderReleased();
        }
        inline void updateEndEffectorScaleAdjustment(int d) {
            client_->updateEndEffectorScaleAdjustment(d);
        }
        inline void resetScale() {
            client_->resetScale();
        }
        inline void selectScaleObject(const QString& s) {
            client_->selectScaleObject(s);
        }
        inline void selectTemplateTrajectory(const QString& s) {
            client_->selectTemplateTrajectory(s);
        }             

    private:
        // UI pointer
        Ui::RVizAffordanceTemplatePanel* ui_;

        // setup widget function
        void setupWidgets();

        // GUI Widgets
        QGraphicsScene* affordanceTemplateGraphicsScene_;

        // ros node handle
        ros::NodeHandle nh_;

    protected:
        AffordanceTemplateRVizClient *client_; // main client class
    };
}

#endif // RVIZ_AFFORDANCE_TEMPLATE_PANEL_HPP
