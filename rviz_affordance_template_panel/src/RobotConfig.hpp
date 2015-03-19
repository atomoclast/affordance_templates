#ifndef ROBOT_CONFIG_HPP
#define ROBOT_CONFIG_HPP

using namespace std;

namespace rviz_affordance_template_panel
{

	class EndEffectorConfig
    {
    public:
        EndEffectorConfig(const string& name) {name_=name;};
        ~EndEffectorConfig() {}

        string name() const { return name_; }
        void name(const string& name ) { name_=name; }

        int id() const { return id_; }
        void id(int id) { id_=id; }

        vector<float> pose_offset() const { return pose_offset_; }
        void pose_offset(const vector<float> pose_offset ) { pose_offset_=pose_offset; }

        vector<float> tool_offset() const { return tool_offset_; }
        void tool_offset(const vector<float> tool_offset ) { tool_offset_=tool_offset; }

    private:
        string name_;
        int id_;
        vector<float> pose_offset_;
        vector<float> tool_offset_;  
    };


    class EndEffectorPoseConfig
    {
    public:
        EndEffectorPoseConfig(const string& name) {name_=name;};
        ~EndEffectorPoseConfig() {}

        string name() const { return name_; }
        void name(const string& name ) { name_=name; }

        string group() const { return group_; }
        void group(const string& group ) { group_=group; }

        int id() const { return id_; }
        void id(int id) { id_=id; }

    private:
        string name_;
        string group_;
        int id_;
    };


    class RobotConfig
    {
    public:
    	typedef boost::shared_ptr<EndEffectorConfig> EndEffectorConfigSharedPtr;
        typedef boost::shared_ptr<EndEffectorPoseConfig> EndEffectorPoseIDConfigSharedPtr;

        RobotConfig(const string& uid) {uid_=uid;};
        ~RobotConfig() {}

        string uid() const { return uid_; }
        void uid(const string& uid ) { uid_=uid; }

        string name() const { return name_; }
        void name(const string& name ) { name_=name; }

        string frame_id() const { return frame_id_; }
        void frame_id(const string& frame_id ) { frame_id_=frame_id; }

        string config_package() const { return config_package_; }
        void config_package(const string& config_package ) { config_package_=config_package; }

        string config_file() const { return config_file_; }
        void config_file(const string& config_file ) { config_file_=config_file; }

        string planner_type() const { return planner_type_; }
        void planner_type(const string& planner_type ) { planner_type_=planner_type; }

        string gripper_service() const { return gripper_service_; }
        void gripper_service(const string& gripper_service ) { gripper_service_=gripper_service; }

		vector<float> root_offset() const { return root_offset_; }
        void root_offset(const vector<float> root_offset ) { root_offset_=root_offset; }

        std::map<std::string, EndEffectorConfigSharedPtr> endeffectorMap;
        std::map<std::string, EndEffectorPoseIDConfigSharedPtr> endeffectorPoseMap;


    private:
        string uid_;
        string name_;
        string frame_id_;
        string config_package_;
        string config_file_;
        string planner_type_;
        string gripper_service_;
        vector<float> root_offset_;


    };
}

#endif