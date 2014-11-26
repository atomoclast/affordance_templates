#include "rviz_affordance_template_panel.hpp"

// TODO: don't like these
#define PIXMAP_SIZE 100
#define XOFFSET 20
#define YOFFSET 20

#define CLASS_INDEX 0
#define TRAJECTORY_DATA 1
#define IMAGE 2
#define FILENAME 3
#define DISPLAY_OBJECTS 4


#define OBJECT_INDEX 0
#define PACKAGE 1
#define LAUNCH_FILE 2

using namespace rviz_affordance_template_panel;
using namespace std;

RVizAffordanceTemplatePanel::RVizAffordanceTemplatePanel(QWidget *parent) :
    rviz::Panel(parent),
    ui_(new Ui::RVizAffordanceTemplatePanel),
    descriptionRobot_(""),
    controls_(new Controls(ui_))
{
    // Setup the panel.
    ui_->setupUi(this);

    // setup service clients
    add_template_client_ = nh_.serviceClient<affordance_template_msgs::AddAffordanceTemplate>("/affordance_template_server/add_template");
    add_object_client_ = nh_.serviceClient<affordance_template_msgs::AddRecognitionObject>("/affordance_template_server/add_recognition_object");
    add_trajectory_client_ = nh_.serviceClient<affordance_template_msgs::AddAffordanceTemplateTrajectory>("/affordance_template_server/add_trajectory");
    command_client_ = nh_.serviceClient<affordance_template_msgs::AffordanceTemplateCommand>("/affordance_template_server/command");
    delete_template_client_ = nh_.serviceClient<affordance_template_msgs::DeleteAffordanceTemplate>("/affordance_template_server/delete_template");
    delete_object_client_ = nh_.serviceClient<affordance_template_msgs::DeleteRecognitionObject>("/affordance_template_server/delete_recognition_object");
    get_robots_client_ = nh_.serviceClient<affordance_template_msgs::GetRobotConfigInfo>("/affordance_template_server/get_robots");
    get_objects_client_ = nh_.serviceClient<affordance_template_msgs::GetRecognitionObjectConfigInfo>("/affordance_template_server/get_recognition_objects");
    get_running_client_ = nh_.serviceClient<affordance_template_msgs::GetRunningAffordanceTemplates>("/affordance_template_server/get_running");
    get_templates_client_ = nh_.serviceClient<affordance_template_msgs::GetAffordanceTemplateConfigInfo>("/affordance_template_server/get_templates");
    load_robot_client_ = nh_.serviceClient<affordance_template_msgs::LoadRobotConfig>("/affordance_template_server/load_robot");
    save_template_client_ = nh_.serviceClient<affordance_template_msgs::SaveAffordanceTemplate>("/affordance_template_server/save_template");
    scale_object_client_ = nh_.serviceClient<affordance_template_msgs::ScaleDisplayObject>("/affordance_template_server/scale_object");

    // setup publishers
    scale_object_streamer_ = nh_.advertise<affordance_template_msgs::ScaleDisplayObjectInfo>("/affordance_template_server/scale_object_streamer", 10);
    
    controls_->setService(command_client_);

    setupWidgets();
    getAvailableInfo();

    descriptionRobot_ = getRobotFromDescription();
    if (descriptionRobot_ != "") {
        std::string yamlRobotCandidate = descriptionRobot_ + ".yaml";
        ROS_INFO("RVizAffordanceTemplatePanel::RVizAffordanceTemplatePanel() -- searching for Robot: %s", yamlRobotCandidate.c_str());
        map<string,RobotConfigSharedPtr>::const_iterator it = robotMap_.find(yamlRobotCandidate);
        if (it != robotMap_.end() ) {
            int idx= ui_->robot_select->findText(QString(yamlRobotCandidate.c_str()));
            ui_->robot_select->setCurrentIndex(idx);
            setupRobotPanel(yamlRobotCandidate);
            loadConfig();
        }
    }

    getRunningItems();

    selected_template = make_pair("",-1);
}

RVizAffordanceTemplatePanel::~RVizAffordanceTemplatePanel()
{
    delete ui_;
}

void RVizAffordanceTemplatePanel::setupWidgets() {

    affordanceTemplateGraphicsScene_ = new QGraphicsScene(this);
    ui_->affordanceTemplateGraphicsView->setScene(affordanceTemplateGraphicsScene_);

    recognitionObjectGraphicsScene_ = new QGraphicsScene(this);
    ui_->recognitionObjectGraphicsView->setScene(recognitionObjectGraphicsScene_);

    QObject::connect(affordanceTemplateGraphicsScene_, SIGNAL(selectionChanged()), this, SLOT(addAffordanceDisplayItem()));
    QObject::connect(recognitionObjectGraphicsScene_, SIGNAL(selectionChanged()), this, SLOT(addObjectDisplayItem()));

    QObject::connect(ui_->server_output_status, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(selectAffordanceTemplate(QListWidgetItem*)));
//    QObject::connect(ui_->server_output_status, SIGNAL(itemDoubleClicked(QListWidgetItem*)), this, SLOT(killAffordanceTemplate(QListWidgetItem*)));
    QObject::connect(ui_->delete_template_button, SIGNAL(clicked()), this, SLOT(deleteButton()));
    QObject::connect(ui_->save_as_button, SIGNAL(clicked()), this, SLOT(saveButton()));
    QObject::connect(ui_->add_traj_button, SIGNAL(clicked()), this, SLOT(addTrajectoryButton()));

    QObject::connect(ui_->load_config_button, SIGNAL(clicked()), this, SLOT(safeLoadConfig()));
    QObject::connect(ui_->robot_select, SIGNAL(currentIndexChanged(int)), this, SLOT(changeRobot(int)));
    QObject::connect(ui_->save_template_combo_box, SIGNAL(activated(int)), this, SLOT(changeSaveInfo(int)));

    QObject::connect(ui_->go_to_start_button, SIGNAL(clicked()), this, SLOT(go_to_start()));
    QObject::connect(ui_->go_to_end_button, SIGNAL(clicked()), this, SLOT(go_to_end()));
    QObject::connect(ui_->step_backwards_button, SIGNAL(clicked()), this, SLOT(step_backward()));
    QObject::connect(ui_->step_forward_button, SIGNAL(clicked()), this, SLOT(step_forward()));
    //QObject::connect(ui_->pause_button, SIGNAL(clicked()), this, SLOT(pause()));
    //QObject::connect(ui_->play_backwards_button, SIGNAL(clicked()), this, SLOT(play_backward()));
    //QObject::connect(ui_->play_button, SIGNAL(clicked()), this, SLOT(play_forward()));

    QObject::connect(ui_->refresh_button, SIGNAL(clicked()), this, SLOT(refreshCallback()));
    QObject::connect(ui_->robot_lock, SIGNAL(stateChanged(int)), this, SLOT(enableConfigPanel(int)));

    QObject::connect(ui_->robot_name, SIGNAL(textEdited(const QString&)), this, SLOT(updateRobotConfig(const QString&)));
    QObject::connect(ui_->moveit_package, SIGNAL(textEdited(const QString&)), this, SLOT(updateRobotConfig(const QString&)));
    QObject::connect(ui_->gripper_service, SIGNAL(textEdited(const QString&)), this, SLOT(updateRobotConfig(const QString&)));
    
    QObject::connect(ui_->frame_id, SIGNAL(textEdited(const QString&)), this, SLOT(updateRobotConfig(const QString&)));
    QObject::connect(ui_->robot_tx, SIGNAL(textEdited(const QString&)), this, SLOT(updateRobotConfig(const QString&)));
    QObject::connect(ui_->robot_ty, SIGNAL(textEdited(const QString&)), this, SLOT(updateRobotConfig(const QString&)));
    QObject::connect(ui_->robot_tz, SIGNAL(textEdited(const QString&)), this, SLOT(updateRobotConfig(const QString&)));
    QObject::connect(ui_->robot_rr, SIGNAL(textEdited(const QString&)), this, SLOT(updateRobotConfig(const QString&)));
    QObject::connect(ui_->robot_rp, SIGNAL(textEdited(const QString&)), this, SLOT(updateRobotConfig(const QString&)));
    QObject::connect(ui_->robot_ry, SIGNAL(textEdited(const QString&)), this, SLOT(updateRobotConfig(const QString&)));
    QObject::connect(ui_->ee_name, SIGNAL(textEdited(const QString&)), this, SLOT(updateEndEffectorGroupMap(const QString&)));
    QObject::connect(ui_->ee_id, SIGNAL(textEdited(const QString&)), this, SLOT(updateEndEffectorGroupMap(const QString&)));
    QObject::connect(ui_->ee_tx, SIGNAL(textEdited(const QString&)), this, SLOT(updateEndEffectorGroupMap(const QString&)));
    QObject::connect(ui_->ee_ty, SIGNAL(textEdited(const QString&)), this, SLOT(updateEndEffectorGroupMap(const QString&)));
    QObject::connect(ui_->ee_tz, SIGNAL(textEdited(const QString&)), this, SLOT(updateEndEffectorGroupMap(const QString&)));
    QObject::connect(ui_->ee_rr, SIGNAL(textEdited(const QString&)), this, SLOT(updateEndEffectorGroupMap(const QString&)));
    QObject::connect(ui_->ee_rp, SIGNAL(textEdited(const QString&)), this, SLOT(updateEndEffectorGroupMap(const QString&)));
    QObject::connect(ui_->ee_ry, SIGNAL(textEdited(const QString&)), this, SLOT(updateEndEffectorGroupMap(const QString&)));

    // object scaling stuff
    QObject::connect(ui_->object_scale_slider, SIGNAL(valueChanged(int)), this, SLOT(updateObjectScale(int)));
    QObject::connect(ui_->object_scale_slider, SIGNAL(sliderReleased()), this, SLOT(scaleSliderReleased()));
    QObject::connect(ui_->end_effector_adjustment_slider, SIGNAL(sliderReleased()), this, SLOT(scaleSliderReleased()));
    QObject::connect(ui_->end_effector_adjustment_slider, SIGNAL(valueChanged(int)), this, SLOT(updateEndEffectorScaleAdjustment(int)));
    QObject::connect(ui_->reset_scale_button, SIGNAL(clicked()), this, SLOT(resetScaleButton()));


}

void RVizAffordanceTemplatePanel::updateRobotConfig(const QString& text) {

    // get currently selected robot key
    string key = ui_->robot_select->currentText().toUtf8().constData();
    // now update robotMap with current values
    (*robotMap_[key]).name(ui_->robot_select->currentText().toUtf8().constData());
    (*robotMap_[key]).moveit_config_package(ui_->moveit_package->text().toUtf8().constData());
    (*robotMap_[key]).frame_id(ui_->frame_id->text().toUtf8().constData());
    (*robotMap_[key]).gripper_service(ui_->gripper_service->text().toUtf8().constData());

    vector<float> root_offset(7);
    vector<float> q = util::RPYToQuaternion(ui_->robot_rr->text().toFloat(), ui_->robot_rp->text().toFloat(), ui_->robot_ry->text().toFloat());
    root_offset[0] = ui_->robot_tx->text().toFloat();
    root_offset[1] = ui_->robot_ty->text().toFloat();
    root_offset[2] = ui_->robot_tz->text().toFloat();
    root_offset[3] = q[0];
    root_offset[4] = q[1];
    root_offset[5] = q[2];
    root_offset[6] = q[3];
    (*robotMap_[key]).root_offset(root_offset);
}

void RVizAffordanceTemplatePanel::updateEndEffectorGroupMap(const QString& text) {
    string robot_key = ui_->robot_select->currentText().toUtf8().constData();
    string key = ui_->end_effector_select->currentText().toUtf8().constData();
    for (auto& e: (*robotMap_[robot_key]).endeffectorMap) {
        if (e.second->name() == key) {
            e.second->name(ui_->ee_name->text().toUtf8().constData());
            e.second->id(ui_->ee_id->text().toInt());
            
            vector<float> pose_offset(7);
            vector<float> q = util::RPYToQuaternion(ui_->ee_rr->text().toFloat(), ui_->ee_rp->text().toFloat(), ui_->ee_ry->text().toFloat());
            pose_offset[0] = ui_->ee_tx->text().toFloat();
            pose_offset[1] = ui_->ee_ty->text().toFloat();
            pose_offset[2] = ui_->ee_tz->text().toFloat();
            pose_offset[3] = q[0];
            pose_offset[4] = q[1];
            pose_offset[5] = q[2];
            pose_offset[6] = q[3];
            e.second->pose_offset(pose_offset);

           /* totx = ui_->ee_totx->text().toFloat();
            toty = ui_->ee_toty->text().toFloat();
            totz = ui_->ee_totz->text().toFloat();
            torr = ui_->ee_torr->text().toFloat();
            torp = ui_->ee_torp->text().toFloat();
            tory = ui_->ee_tory->text().toFloat();*/

            float totx, toty, totz, torr, torp, tory;
            totx = toty = totz = torr = torp = tory = 0;

            vector<float> toq = util::RPYToQuaternion(torr, torp, tory);

            vector<float> tool_offset(7);
            tool_offset[0] = totx;
            tool_offset[1] = toty;
            tool_offset[2] = totz;
            tool_offset[3] = toq[0];
            tool_offset[4] = toq[1];
            tool_offset[5] = toq[2];
            tool_offset[6] = toq[3];
            e.second->tool_offset(tool_offset);
            break;
        }
    }
}

void RVizAffordanceTemplatePanel::enableConfigPanel(int state) {
    if (state == Qt::Checked) {
        ui_->groupBox->setEnabled(false);
        ui_->load_config_button->setEnabled(false);
    } else {
        ui_->groupBox->setEnabled(true);
        ui_->load_config_button->setEnabled(true);
    }
}

void RVizAffordanceTemplatePanel::refreshCallback() {
    removeAffordanceTemplates();
    getAvailableInfo();
    getRunningItems();
}

void RVizAffordanceTemplatePanel::getAvailableInfo() {
    getAvailableTemplates();
    getAvailableRecognitionObjects();
    getAvailableRobots();
}

void RVizAffordanceTemplatePanel::getAvailableTemplates() {
    ROS_INFO("querying available templates");    
    affordance_template_msgs::GetAffordanceTemplateConfigInfo srv;
    if (get_templates_client_.call(srv))
    {
        int yoffset = YOFFSET;
        affordanceTemplateGraphicsScene_->clear();
        for (auto& t: srv.response.templates) {
            string image_path = util::resolvePackagePath(t.image_path);
            string filename = t.filename;
            QMap<QString, QVariant> trajectory_map;
            QStringList display_objects;
            ROS_INFO("Found Affordance Template: %s", t.type.c_str());
            for (auto& traj: t.trajectory_info) {
                QMap<QString, QVariant> waypoint_map;
                for (auto& wp: traj.waypoint_info) {
                    waypoint_map[QString::number(wp.id)] = QVariant(wp.num_waypoints);
                }
                trajectory_map[QString(traj.name.c_str())] = waypoint_map;
            }       
            for (auto& objs: t.display_objects) {
                display_objects.append(QString(objs.c_str()));
            }       
            AffordanceSharedPtr pitem(new Affordance(t.type.c_str(), image_path, trajectory_map, display_objects, filename));
            pitem->setPos(XOFFSET, yoffset);
            yoffset += PIXMAP_SIZE + YOFFSET;
            if(!checkAffordance(pitem)) {
                addAffordance(pitem);
            }
            affordanceTemplateGraphicsScene_->addItem(pitem.get());

        }
        affordanceTemplateGraphicsScene_->update();
    }
    else
    {
        ROS_ERROR("Failed to call service get templates");
    }       
}

void RVizAffordanceTemplatePanel::getAvailableRecognitionObjects() {
    ROS_INFO("querying available recognition objects");    
    affordance_template_msgs::GetRecognitionObjectConfigInfo srv;
    if (get_objects_client_.call(srv))
    {
        int yoffset = YOFFSET;
        recognitionObjectGraphicsScene_->clear();
        for (auto& o: srv.response.recognition_objects) {
            string image_path = util::resolvePackagePath(o.image_path);
            RecognitionObjectSharedPtr pitem(new RecognitionObject(o.type, o.launch_file, o.package, image_path));
            pitem->setPos(XOFFSET, yoffset);
            yoffset += PIXMAP_SIZE + YOFFSET;
            if(!checkRecognitionObject(pitem)) {
                addRecognitionObject(pitem);
            }
            recognitionObjectGraphicsScene_->addItem(pitem.get());
                
        }
        recognitionObjectGraphicsScene_->update();
    }
    else
    {
        ROS_ERROR("Failed to call service get recog objects");
    }       
}

void RVizAffordanceTemplatePanel::getAvailableRobots() {

    ROS_INFO("querying available robots");    

    affordance_template_msgs::GetRobotConfigInfo srv;
    if (get_robots_client_.call(srv))
    {

        // load stuff for robot config sub panel
        ui_->robot_select->disconnect(SIGNAL(currentIndexChanged(int)));
        ui_->end_effector_select->disconnect(SIGNAL(currentIndexChanged(int)));
        ui_->robot_select->clear();
        ui_->end_effector_select->clear();

        for (auto& r: srv.response.robots) {

            RobotConfigSharedPtr pitem(new RobotConfig(r.filename));
            pitem->uid(r.filename);
            pitem->name(r.name);
            pitem->moveit_config_package(r.moveit_config_package);
            pitem->frame_id(r.frame_id);
            pitem->gripper_service(r.gripper_service);

            vector<float> root_offset = util::poseMsgToVector(r.root_offset);
            pitem->root_offset(root_offset);

            for (auto& e: r.end_effectors) {

                EndEffectorConfigSharedPtr eitem(new EndEffectorConfig(e.name));
                eitem->id(int(e.id));

                vector<float> pose_offset = util::poseMsgToVector(e.pose_offset);
                eitem->pose_offset(pose_offset);
                pitem->endeffectorMap[e.name] = eitem;

                vector<float> tool_offset = util::poseMsgToVector(e.tool_offset);
                eitem->tool_offset(tool_offset);
                pitem->endeffectorMap[e.name] = eitem;

            }
            for (auto& p: r.end_effector_pose_data) {
                EndEffectorPoseIDConfigSharedPtr piditem(new EndEffectorPoseConfig(p.name));
                piditem->id(int(p.id))  ;
                piditem->group(p.group);
                pitem->endeffectorPoseMap[p.name] = piditem;
            }

            addRobot(pitem);

            ui_->robot_select->addItem(QString(pitem->uid().c_str()));
        }

        setupRobotPanel(robotMap_.begin()->first);
        QObject::connect(ui_->robot_select, SIGNAL(currentIndexChanged(int)), this, SLOT(changeRobot(int)));
        QObject::connect(ui_->end_effector_select, SIGNAL(currentIndexChanged(int)), this, SLOT(changeEndEffector(int)));

        // set Controls
        controls_->setRobotMap(robotMap_);

    }
    else
    {
        ROS_ERROR("Failed to call service get robots");
    }

}

void RVizAffordanceTemplatePanel::setupRobotPanel(const string& key) {

    string name = (*robotMap_[key]).name();
    string pkg = (*robotMap_[key]).moveit_config_package();
    string frame_id = (*robotMap_[key]).frame_id();
    string gripper_service = (*robotMap_[key]).gripper_service();

    vector<float> root_offset = (*robotMap_[key]).root_offset();

    ui_->robot_name->setText(QString(name.c_str()));
    ui_->moveit_package->setText(QString(pkg.c_str()));
    ui_->frame_id->setText(QString(frame_id.c_str()));
    ui_->gripper_service->setText(QString(gripper_service.c_str()));

    ui_->robot_tx->setText(QString::number(root_offset[0]));
    ui_->robot_ty->setText(QString::number(root_offset[1]));
    ui_->robot_tz->setText(QString::number(root_offset[2]));

    vector<float> rpy = util::quaternionToRPY(root_offset[3],root_offset[4],root_offset[5],root_offset[6]);

    ui_->robot_rr->setText(QString::number(rpy[0]));
    ui_->robot_rp->setText(QString::number(rpy[1]));
    ui_->robot_ry->setText(QString::number(rpy[2]));

    ui_->end_effector_select->clear();

    for (auto& e: (*robotMap_[key]).endeffectorMap) {
        ui_->end_effector_select->addItem(e.second->name().c_str());
    }

    setupEndEffectorConfigPanel((*robotMap_[key]).endeffectorMap.begin()->first);

}

void RVizAffordanceTemplatePanel::setupEndEffectorConfigPanel(const string& key) {

    string robot_key = ui_->robot_select->currentText().toUtf8().constData();

    for (auto& e: (*robotMap_[robot_key]).endeffectorMap) {
        if (e.second->name() == key) {
            ui_->ee_name->setText(e.second->name().c_str());
            ui_->ee_id->setText(QString::number(e.second->id()));
            
            vector<float> pose_offset = e.second->pose_offset();
            ui_->ee_tx->setText(QString::number(pose_offset[0]));
            ui_->ee_ty->setText(QString::number(pose_offset[1]));
            ui_->ee_tz->setText(QString::number(pose_offset[2]));

            vector<float> rpy = util::quaternionToRPY(pose_offset[3],pose_offset[4],pose_offset[5],pose_offset[6]);
            ui_->ee_rr->setText(QString::number(rpy[0]));
            ui_->ee_rp->setText(QString::number(rpy[1]));
            ui_->ee_ry->setText(QString::number(rpy[2]));

            // FIX ME, THIS NEEDS TO BE DONE FOR TOOL OFFSET
            /*vector<float> tool_offset = e.second->tool_offset();
            ui_->ee_totx->setText(QString::number(tool_offset[0]));
            ui_->ee_toty->setText(QString::number(tool_offset[1]));
            ui_->ee_totz->setText(QString::number(tool_offset[2]));

            vector<float> torpy = util::quaternionToRPY(tool_offset[3],tool_offset[4],tool_offset[5],tool_offset[6]);
            ui_->ee_torr->setText(QString::number(torpy[0]));
            ui_->ee_torp->setText(QString::number(torpy[1]));
            ui_->ee_tory->setText(QString::number(torpy[2]));*/

            break;
        }
    }

}

void RVizAffordanceTemplatePanel::changeRobot(int id) {
    QString r = ui_->robot_select->itemText(id);
    setupRobotPanel(r.toUtf8().constData());
}

void RVizAffordanceTemplatePanel::changeEndEffector(int id) {
    QString ee = ui_->end_effector_select->itemText(id);
    setupEndEffectorConfigPanel(ee.toUtf8().constData());
}

void RVizAffordanceTemplatePanel::changeSaveInfo(int id) {

    if (id >= ui_->save_template_combo_box->count() || ui_->save_template_combo_box->count()==0 ) {
        ROS_WARN("RVizAffordanceTemplatePanel::changeSaveInfo() -- something funny!! clearing drop down");
        ui_->save_template_combo_box->clear();
        ui_->new_save_type->clear();
        ui_->new_filename->clear();
        ui_->new_image->clear();
        return;
    }  
    QString key = ui_->save_template_combo_box->itemText(id);
    vector<string> stuff = util::split(key.toStdString(), ':');
    
    string class_type = stuff[0];
    int at_id = int(atoi(stuff[1].c_str()));

    QList<QGraphicsItem*> list = affordanceTemplateGraphicsScene_->items();
    for (int i=0; i < list.size(); ++i) {
        // Get the object template class name from the first element in the QGraphicsItem's custom data
        // field. This field is set in the derived Affordance class when setting up the widgets.
        string class_name = list.at(i)->data(CLASS_INDEX).toString().toStdString();

        if (class_name != class_type) {
            continue;
        }
        string image_name = list.at(i)->data(IMAGE).toString().toStdString();
        string filename = list.at(i)->data(FILENAME).toString().toStdString();
        
        ROS_DEBUG("RVizAffordanceTemplatePanel::changeSaveInfo() -- %s", class_name.c_str());
        ROS_DEBUG("RVizAffordanceTemplatePanel::changeSaveInfo() -- %s", image_name.c_str());
        ROS_DEBUG("RVizAffordanceTemplatePanel::changeSaveInfo() -- %s", filename.c_str());
        
        vector<string> image_tokens = util::split(image_name, '/');
        vector<string> fname_tokens = util::split(filename, '/');

        ui_->new_save_type->setText(QString(class_name.c_str()));

        if(image_tokens.size()>0) 
        {
            string stripped_image = image_tokens[image_tokens.size()-1];
            ROS_INFO("RVizAffordanceTemplatePanel::changeSaveInfo() -- %s", stripped_image.c_str());
            ui_->new_image->setText(QString(stripped_image.c_str()));
        } else {
            ui_->new_image->setText(QString(image_name.c_str()));            
        }

        if(fname_tokens.size()>0) 
        {
            string stripped_fname = fname_tokens[fname_tokens.size()-1];
            ROS_INFO("RVizAffordanceTemplatePanel::changeSaveInfo() -- %s", stripped_fname.c_str());
            ui_->new_filename->setText(QString(stripped_fname.c_str()));
        } else {
            ui_->new_filename->setText(QString(filename.c_str()));
        }
        break;
    }   
}

void RVizAffordanceTemplatePanel::deleteButton() {
    if(ui_->server_output_status->currentItem()) {
        killAffordanceTemplate(ui_->server_output_status->currentItem());
    }
}

void RVizAffordanceTemplatePanel::saveButton() {
    saveAffordanceTemplate();
}

void RVizAffordanceTemplatePanel::addTrajectoryButton() {
    addTrajectory();
}

void RVizAffordanceTemplatePanel::removeAffordanceTemplates() {
    for (auto& pitem: affordanceTemplateGraphicsScene_->items()) {
        affordanceTemplateGraphicsScene_->removeItem(pitem);
    }
    affordanceMap_.clear();
    affordanceTemplateGraphicsScene_->update();
}

void RVizAffordanceTemplatePanel::removeRecognitionObjects() {
    for (auto& pitem: recognitionObjectGraphicsScene_->items()) {
        recognitionObjectGraphicsScene_->removeItem(pitem);
    }
    recognitionObjectMap_.clear();
    recognitionObjectGraphicsScene_->update();
}

int RVizAffordanceTemplatePanel::sendAffordanceTemplateAdd(const string& class_name) {
    ROS_INFO("Sending Add Template request for a %s", class_name.c_str());      
    affordance_template_msgs::AddAffordanceTemplate srv;
    srv.request.class_type = class_name;
    if (add_template_client_.call(srv))
    {
        ROS_INFO("Add successful, new %s with id: %d", class_name.c_str(), (int)(srv.response.id));
        return (int)(srv.response.id);
    }
    else
    {
        ROS_ERROR("Failed to call service add_template");
    }
    return -1;
}

void RVizAffordanceTemplatePanel::sendRecognitionObjectAdd(const string& object_name) {
    ROS_INFO("Sending Add Recognition Object request for a %s", object_name.c_str());      
    affordance_template_msgs::AddRecognitionObject srv;
    srv.request.object_type = object_name;
    if (add_object_client_.call(srv))
    {
        ROS_INFO("Add successful, new %s with id: %d", object_name.c_str(), (int)(srv.response.id));
    }
    else
    {
        ROS_ERROR("Failed to call service add_object");
    }
}

void RVizAffordanceTemplatePanel::sendAffordanceTemplateKill(const string& class_name, int id) {
    ROS_INFO("Sending kill to %s:%d", class_name.c_str(), id);      
    affordance_template_msgs::DeleteAffordanceTemplate srv;
    srv.request.class_type = class_name;
    srv.request.id = id;
    if (delete_template_client_.call(srv))
    {
        ROS_INFO("Delete successful");

        string full_name = class_name + ":" + to_string(id);
        int idx = ui_->save_template_combo_box->findText(QString(full_name.c_str()));
        if ( idx != -1) {
            ui_->save_template_combo_box->removeItem(idx);  
            if(ui_->save_template_combo_box->count()==0) {
                ui_->save_template_combo_box->clear();  
                ui_->new_save_type->clear();
                ui_->new_filename->clear();
                ui_->new_image->clear();
            }
        }

    }
    else
    {
        ROS_ERROR("Failed to call service delete_template");
    }
}

void RVizAffordanceTemplatePanel::sendRecognitionObjectKill(const string& object_name, int id) {
    ROS_INFO("Sending kill to %s:%d", object_name.c_str(), id);      
    affordance_template_msgs::DeleteRecognitionObject srv;
    srv.request.object_type = object_name;
    srv.request.id = id;
    if (delete_object_client_.call(srv))
    {
        ROS_INFO("Delete successful");
    }
    else
    {
        ROS_ERROR("Failed to call service delete_object");
    }
}

void RVizAffordanceTemplatePanel::killAffordanceTemplate(QListWidgetItem* item) {
    vector<string> template_info = util::split(item->text().toUtf8().constData(), ':');
    int id;
    istringstream(template_info[1]) >> id;
    sendAffordanceTemplateKill(template_info[0], id);
    getRunningItems();
}

void RVizAffordanceTemplatePanel::killRecognitionObject(QListWidgetItem* item) {
    vector<string> object_info = util::split(item->text().toUtf8().constData(), ':');
    int id;
    istringstream(object_info[1]) >> id;
    sendRecognitionObjectKill(object_info[0], id);
    getRunningItems();
}

void RVizAffordanceTemplatePanel::saveAffordanceTemplate() {
    sendSaveAffordanceTemplate();
    getRunningItems();
}

void RVizAffordanceTemplatePanel::addTrajectory() {
    sendAddTrajectory();
}

void RVizAffordanceTemplatePanel::selectAffordanceTemplate(QListWidgetItem* item) {
    vector<string> template_info = util::split(item->text().toUtf8().constData(), ':');
    int id = ui_->save_template_combo_box->findText(item->text());
    if(id != -1) {
        changeSaveInfo(id);
    }
    ui_->save_template_combo_box->setCurrentIndex(id);  

    string class_type = template_info[0];
    int template_id = atoi(template_info[1].c_str());
    selected_template = make_pair(class_type, template_id);

    setupDisplayObjectSliders(class_type, template_id);
}

void RVizAffordanceTemplatePanel::setupDisplayObjectSliders(std::string class_type, int id) {
    QList<QGraphicsItem*> list = affordanceTemplateGraphicsScene_->items();
    int v;
    ui_->object_scale_combo_box->clear();
    for (int i=0; i < list.size(); ++i) {
        string class_name = list.at(i)->data(CLASS_INDEX).toString().toStdString();
        if(class_name==class_type) {
            for (auto& c: list.at(i)->data(DISPLAY_OBJECTS).toStringList()) {
                
                ui_->object_scale_combo_box->addItem(QString(c.toStdString().c_str()));

                v = ui_->object_scale_slider->minimum() + (ui_->object_scale_slider->maximum() - ui_->object_scale_slider->minimum()) / 2;
                display_object_scale_map[c.toStdString()] = v;  

                v = ui_->end_effector_adjustment_slider->minimum() + (ui_->end_effector_adjustment_slider->maximum() - ui_->end_effector_adjustment_slider->minimum()) / 2;
                end_effector_adjustment_map[c.toStdString()] = v;   
            }
        }
    }
}

void RVizAffordanceTemplatePanel::updateObjectScale(int value) {
    string current_scale_object = ui_->object_scale_combo_box->currentText().toStdString();
    display_object_scale_map[current_scale_object] = value;
    if(ui_->stream_scale_check_box->isChecked()) {
        sendScaleInfo();
    }    
}

void RVizAffordanceTemplatePanel::updateEndEffectorScaleAdjustment(int value) {
    string current_scale_object = ui_->object_scale_combo_box->currentText().toStdString();
    end_effector_adjustment_map[current_scale_object] = value;
    if(ui_->stream_scale_check_box->isChecked()) {
        sendScaleInfo();
    }    
}

void RVizAffordanceTemplatePanel::scaleSliderReleased() {
    string current_scale_object = ui_->object_scale_combo_box->currentText().toStdString();
    display_object_scale_map[current_scale_object] = ui_->object_scale_slider->value();  
    end_effector_adjustment_map[current_scale_object] = ui_->end_effector_adjustment_slider->value();
    sendScaleInfo();
}

void RVizAffordanceTemplatePanel::sendScaleInfo() {

    string current_template_class = selected_template.first;
    int current_template_id = selected_template.second;

    if( (current_template_class == "") || (current_template_id == -1)) {
        ROS_WARN("RVizAffordanceTemplatePanel::sendScaleInfo() -- trying to scale object when nothing is selected");
        return;
    }    

    string current_scale_object = ui_->object_scale_combo_box->currentText().toStdString();
    
    int obj_scale = display_object_scale_map[current_scale_object];
    int ee_scale = end_effector_adjustment_map[current_scale_object];

    double min_value = atof(ui_->object_scale_min->text().toStdString().c_str());
    double max_value = atof(ui_->object_scale_max->text().toStdString().c_str());
    double range = max_value - min_value;

    double obj_scale_value = range*double(obj_scale)/100.0 + min_value;
    double ee_adj_value = (range/2.0)*double(ee_scale)/100.0 + (min_value+range/4.0);

    affordance_template_msgs::ScaleDisplayObjectInfo msg;
    msg.class_type = current_template_class;
    msg.id = current_template_id;
    msg.object_name = current_scale_object;
    msg.scale_factor = obj_scale_value;
    msg.end_effector_scale_factor = ee_adj_value;

    ROS_DEBUG("sending scale to template[%s:%d].%s  with scales(%2.2f,%2.2f) " , current_template_class.c_str(), current_template_id, current_scale_object.c_str(), obj_scale_value, ee_adj_value);
    streamObjectScale(msg);
}

void RVizAffordanceTemplatePanel::resetScaleButton() {

    int v;

    string current_scale_object = ui_->object_scale_combo_box->currentText().toStdString();

    v = ui_->object_scale_slider->minimum() + (ui_->object_scale_slider->maximum() - ui_->object_scale_slider->minimum()) / 2;
    display_object_scale_map[current_scale_object] = v;  
    ui_->object_scale_slider->setSliderPosition(v);  
    
    v = ui_->end_effector_adjustment_slider->minimum() + (ui_->end_effector_adjustment_slider->maximum() - ui_->end_effector_adjustment_slider->minimum()) / 2;
    end_effector_adjustment_map[current_scale_object] = v;   
    ui_->end_effector_adjustment_slider->setSliderPosition(v);  
    
    sendScaleInfo();

}

void RVizAffordanceTemplatePanel::sendObjectScale(affordance_template_msgs::ScaleDisplayObjectInfo scale_info) {
    affordance_template_msgs::ScaleDisplayObject srv;
    srv.request.scale_info = scale_info;
    if (scale_object_client_.call(srv))
    {
        ROS_INFO("Scale successful");
    }
    else
    {
        ROS_ERROR("Failed to scale objectd");
    }  
}

void RVizAffordanceTemplatePanel::streamObjectScale(affordance_template_msgs::ScaleDisplayObjectInfo scale_info) {  
     scale_object_streamer_.publish(scale_info);
}


void RVizAffordanceTemplatePanel::sendSaveAffordanceTemplate() {
    
    string key = ui_->save_template_combo_box->currentText().toUtf8().constData();
    vector<string> stuff = util::split(key, ':');

    string class_type = stuff[0];
    int id = int(atoi(stuff[1].c_str()));

    affordance_template_msgs::SaveAffordanceTemplate srv;
    srv.request.original_class_type = class_type;
    srv.request.id = id;

    //srv.request.new_class_type = ui_->new_save_type->text().toUtf8().constData());
    srv.request.new_class_type = ui_->new_save_type->text().toStdString();
    srv.request.image = ui_->new_image->text().toStdString();
    srv.request.filename = ui_->new_filename->text().toStdString();

    bool abort_flag = false;

    if(srv.request.new_class_type=="") {
        ROS_WARN("No Class Name entered, not sending anything...");
        abort_flag = true;  
    }
    if(srv.request.image=="") {
        ROS_WARN("No Image entered, not sending anything...");
        abort_flag = true;  
    }
    if(srv.request.filename=="") {
        ROS_WARN("No Filename entered, not sending anything...");
        abort_flag = true;  
    }
    if(abort_flag) {
        return;
    }

    ROS_INFO("Sending save to %s:%d, with image %s to file: %s", srv.request.new_class_type.c_str(), id, srv.request.image.c_str(), srv.request.filename.c_str());      

    if (save_template_client_.call(srv))
    {
        ROS_INFO("Save successful");
        int idx = ui_->save_template_combo_box->findText(QString(key.c_str()));
        if ( idx != -1) {
            ui_->save_template_combo_box->removeItem(idx);  
            if(ui_->save_template_combo_box->count()==0) {
                ui_->save_template_combo_box->clear();  
                ui_->new_save_type->clear();
                ui_->new_filename->clear();
                ui_->new_image->clear();
            }
        }
    }
    else
    {
        ROS_ERROR("Failed to save template");
    }
}

void RVizAffordanceTemplatePanel::sendAddTrajectory() {
    
    string key = ui_->save_template_combo_box->currentText().toUtf8().constData();
    vector<string> stuff = util::split(key, ':');

    string class_type = stuff[0];
    int id = int(atoi(stuff[1].c_str()));

    affordance_template_msgs::AddAffordanceTemplateTrajectory srv;
    srv.request.class_type = class_type;
    srv.request.id = id;
    srv.request.trajectory_name = ui_->new_traj_name->text().toStdString();

    if(srv.request.trajectory_name=="") {
        ROS_WARN("No Trajectory Name entered, not sending anything...");
        return;      
    }
     
    ROS_INFO("Sending add traj [%s] to %s:%d", srv.request.trajectory_name.c_str(), srv.request.class_type.c_str(), id);      

    if (add_trajectory_client_.call(srv))
    {
        ROS_INFO("Add Trajectory successful");
    }
    else
    {
        ROS_ERROR("Failed to add trajectory");
    }
}

void RVizAffordanceTemplatePanel::getRunningItems() {
    ROS_INFO("Requesting running affordance templates");      
    affordance_template_msgs::GetRunningAffordanceTemplates srv;
    if (get_running_client_.call(srv))
    {
        ui_->server_output_status->clear();
        ui_->control_template_box->clear();
        for (int i=0; i < srv.response.templates.size(); i++) {
            string t = srv.response.templates[i];
            ROS_INFO("Found running template: %s", t.c_str());
            ui_->server_output_status->addItem(QString::fromStdString(t.c_str()));
            ui_->server_output_status->item(i)->setForeground(Qt::blue);
            ui_->control_template_box->addItem(QString(t.c_str()));
            int idx = ui_->save_template_combo_box->findText(t.c_str());
            if (idx == -1) {
                ui_->save_template_combo_box->addItem(QString(t.c_str()));  
            }
        }
    }
    else
    {
      ROS_ERROR("Failed to call service get_running");
    }
    ui_->server_output_status->sortItems();
}

void RVizAffordanceTemplatePanel::safeLoadConfig() {
    if(ui_->robot_lock->isChecked()) {
        cout << "Can't load while RobotConfig is locked" << endl;
        return;
    }
    loadConfig();
}

void RVizAffordanceTemplatePanel::loadConfig() {

    ROS_WARN("RVizAffordanceTemplatePanel::loadConfig() -- WARNING::taking parameters loaded from original config, not the GUI yet!!! ");

    affordance_template_msgs::LoadRobotConfig srv;

    string key = ui_->robot_select->currentText().toUtf8().constData();

    string name = (*robotMap_[key]).name();
    string pkg = (*robotMap_[key]).moveit_config_package();
    string gripper_service = (*robotMap_[key]).gripper_service();
    string frame_id = (*robotMap_[key]).frame_id();
    vector<float> root_offset = (*robotMap_[key]).root_offset();

    srv.request.robot_config.filename = key;
    srv.request.robot_config.name = name;
    srv.request.robot_config.moveit_config_package = pkg;
    srv.request.robot_config.gripper_service = gripper_service;
    srv.request.robot_config.frame_id = frame_id;
    srv.request.robot_config.root_offset = util::vectorToPoseMsg(root_offset);
    
    // remove all rows from before
    while(ui_->end_effector_table->rowCount()>0) {
        ui_->end_effector_table->removeCellWidget(0,0);
        ui_->end_effector_table->removeCellWidget(0,1);
        ui_->end_effector_table->removeCellWidget(0,2);
        ui_->end_effector_table->removeCellWidget(0,3);
        ui_->end_effector_table->removeRow(0);
    }

    int r = 0;
    for (auto& e: (*robotMap_[key]).endeffectorMap) {

        affordance_template_msgs::EndEffectorConfig ee_config;
        ee_config.name =  e.second->name();
        ee_config.id =  e.second->id();
        
        vector<float> pose_offset = e.second->pose_offset();
        ee_config.pose_offset = util::vectorToPoseMsg(pose_offset);

        vector<float> tool_offset = e.second->tool_offset();
        ee_config.tool_offset = util::vectorToPoseMsg(tool_offset);

        // add rows to end effector controls table
        QTableWidgetItem *i= new QTableWidgetItem(QString(e.second->name().c_str()));
        ui_->end_effector_table->insertRow(r);

        ui_->end_effector_table->setItem(r,0,new QTableWidgetItem(QString(e.second->name().c_str())));
        ui_->end_effector_table->setItem(r,1,new QTableWidgetItem(QString("-")));
        ui_->end_effector_table->setItem(r,2,new QTableWidgetItem(QString("-")));

        QTableWidgetItem *pItem = new QTableWidgetItem();
        pItem->setCheckState(Qt::Checked);
        ui_->end_effector_table->setItem(r,3,pItem);

        r++;

        srv.request.robot_config.end_effectors.push_back(ee_config);

    }

    for (auto& e: (*robotMap_[key]).endeffectorPoseMap) {
        affordance_template_msgs::EndEffectorPoseData ee_pose;
        ee_pose.name = e.second->name();
        ee_pose.group = e.second->group();
        ee_pose.id = e.second->id();
        srv.request.robot_config.end_effector_pose_data.push_back(ee_pose);
    }

    ui_->end_effector_table->resizeColumnsToContents();
    ui_->end_effector_table->resizeRowsToContents();

    if (load_robot_client_.call(srv))
    {
        ROS_INFO("Load Robot Config call succesful");
    }
    else
    {
        ROS_ERROR("Failed to call service load_robot_config");
    }
        
    robot_name_ = key;
    controls_->setRobotName(robot_name_);

    if(ui_->robot_lock->isChecked()) {
        enableConfigPanel(Qt::Checked);
    } else {
        enableConfigPanel(Qt::Unchecked);        
    }
}


void RVizAffordanceTemplatePanel::addAffordanceDisplayItem() {
    // Add an object template to the InteractiveMarkerServer for each selected item.
    cout << "RVizAffordanceTemplatePanel::addAffordanceDisplayItem()" << endl;
    QList<QGraphicsItem*> list = affordanceTemplateGraphicsScene_->selectedItems();
    for (int i=0; i < list.size(); ++i) {
        // Get the object template class name from the first element in the QGraphicsItem's custom data
        // field. This field is set in the derived Affordance class when setting up the widgets.
        string class_name = list.at(i)->data(CLASS_INDEX).toString().toStdString();
        string image_name = list.at(i)->data(IMAGE).toString().toStdString();
        string filename = list.at(i)->data(FILENAME).toString().toStdString();
        
        ROS_INFO("RVizAffordanceTemplatePanel::addAffordanceDisplayItem() -- %s", class_name.c_str());
        int idx = sendAffordanceTemplateAdd(class_name);
        if (idx < 0) {
            ROS_ERROR("RVizAffordanceTemplatePanel::addAffordanceDisplayItem() something wrong!!");
            return;
        }

        vector<string> image_tokens = util::split(image_name, '/');
        vector<string> fname_tokens = util::split(filename, '/');
        string at_full_name = class_name + ":" + to_string(idx);
        if (ui_->save_template_combo_box->findText(at_full_name.c_str()) == -1) {
            ui_->save_template_combo_box->addItem(QString(at_full_name.c_str()));
            ui_->save_template_combo_box->setItemData(idx,QString(at_full_name.c_str()));
        }

        //cout << "RVizAffordanceTemplatePanel::addAffordanceDisplayItem() -- retrieving waypoint info" << endl;
        for (auto& c: list.at(i)->data(TRAJECTORY_DATA).toMap().toStdMap()) {
            string robot_key = ui_->robot_select->currentText().toUtf8().constData();
            for (auto& e: (*robotMap_[robot_name_]).endeffectorMap) {
                for (int r=0; r<ui_->end_effector_table->rowCount(); r++ ) {
                    if (e.second->name() == ui_->end_effector_table->item(r,0)->text().toStdString() ) {
                        ui_->end_effector_table->setItem(r,2,new QTableWidgetItem(QString::number(c.second.toInt())));
                    }
                }
            }
        }
    }
    getRunningItems();
}

void RVizAffordanceTemplatePanel::addObjectDisplayItem() {
    // Add an object template to the InteractiveMarkerServer for each selected item.
    cout << "RVizAffordanceTemplatePanel::addObjectDisplayItem()" << endl;
    QList<QGraphicsItem*> list = recognitionObjectGraphicsScene_->selectedItems();
    for (int i=0; i < list.size(); ++i) {
        // Get the object template class name from the first element in the QGraphicsItem's custom data
        // field. This field is set in the derived Affordance class when setting up the widgets.
        string object_name = list.at(i)->data(OBJECT_INDEX).toString().toStdString();
        ROS_INFO("RVizAffordanceTemplatePanel::addObjectDisplayItem() -- %s", object_name.c_str());
        sendRecognitionObjectAdd(object_name);
    }
    // update running templates
    getRunningItems();
}

bool RVizAffordanceTemplatePanel::addAffordance(const AffordanceSharedPtr& obj) {
    // check if template is in our map
    if (!checkAffordance(obj)) {
        affordanceMap_[(*obj).key()] = obj;
        return true;
    }
    return false;
}

bool RVizAffordanceTemplatePanel::addRecognitionObject(const RecognitionObjectSharedPtr& obj) {
    // check if template is in our map
    if (!checkRecognitionObject(obj)) {
        recognitionObjectMap_[(*obj).key()] = obj;
        return true;
    }
    return false;
}

bool RVizAffordanceTemplatePanel::removeAffordance(const AffordanceSharedPtr& obj) {
    // check if template is in our map
    if (checkAffordance(obj)) {
        affordanceMap_.erase((*obj).key());
        return true;
    }
    return false;
}

bool RVizAffordanceTemplatePanel::removeRecognitionObject(const RecognitionObjectSharedPtr& obj) {
    // check if template is in our map
    if (checkRecognitionObject(obj)) {
        recognitionObjectMap_.erase((*obj).key());
        return true;
    }
    return false;
}

bool RVizAffordanceTemplatePanel::checkAffordance(const AffordanceSharedPtr& obj) {
    if (affordanceMap_.find((*obj).key()) == affordanceMap_.end()) {
        return false;
    }
    return true;
}

bool RVizAffordanceTemplatePanel::checkRecognitionObject(const RecognitionObjectSharedPtr& obj) {
    if (recognitionObjectMap_.find((*obj).key()) == recognitionObjectMap_.end()) {
        return false;
    }
    return true;
}

bool RVizAffordanceTemplatePanel::addRobot(const RobotConfigSharedPtr& obj) {
    // check if robot is in our map
    if (!checkRobot(obj)) {
        robotMap_[(*obj).uid()] = obj;
        return true;
    }
    return false;
}

bool RVizAffordanceTemplatePanel::removeRobot(const RobotConfigSharedPtr& obj) {
    // check if robot is in our map
    if (checkRobot(obj)) {
        robotMap_.erase((*obj).uid());
        return true;
    }
    return false;
}

bool RVizAffordanceTemplatePanel::checkRobot(const RobotConfigSharedPtr& obj) {
    if (robotMap_.find((*obj).uid()) == robotMap_.end()) {
        return false;
    }
    return true;
}


void RVizAffordanceTemplatePanel::configChanged()
{
    rviz::Panel::configChanged();
}

std::string RVizAffordanceTemplatePanel::getRobotFromDescription() {
    std::string robot = "";
    urdf::Model model;
    if (!model.initParam("robot_description")) {
        ROS_ERROR("Failed to parse robot_description rosparam");
    } else {
        cout << "RVizAffordanceTemplatePanel::getRobotFromDescription() -- found robot: " << model.name_ << endl;
        robot = model.name_;
    }
    return robot;
}


#include <pluginlib/class_list_macros.h>
#if ROS_VERSION_MINIMUM(1,9,41)
    PLUGINLIB_EXPORT_CLASS(rviz_affordance_template_panel::RVizAffordanceTemplatePanel, rviz::Panel)
#else
    PLUGINLIB_DECLARE_CLASS(rviz_affordance_template_panel, RVizAffordanceTemplatePanel, rviz_affordance_template_panel::RVizAffordanceTemplatePanel, rviz::Panel)
#endif


