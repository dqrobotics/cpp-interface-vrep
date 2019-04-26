#ifndef DQ_ROBOTICS_INTERFACE_VREP_HEADER_GUARD
#define DQ_ROBOTICS_INTERFACE_VREP_HEADER_GUARD

#include<atomic>
#include<vector>
#include<functional>
#include<map>
#include<string>

#include<dqrobotics/DQ.h>
#include<extApi.h>
#include<v_repConst.h>

const std::string VREP_OBJECTNAME_ABSOLUTE("VREP_OBJECTNAME_ABSOLUTE");

using namespace DQ_robotics;
using namespace Eigen;

class VrepInterface
{
public:
    enum OP_MODES
    {
        OP_BLOCKING,
        OP_STREAMING,
        OP_ONESHOT,
        OP_BUFFER
    };

    /**
     * @brief vrepInterface
     * Default constructor
     */
    VrepInterface(std::atomic_bool *no_blocking_loops=nullptr);

    /**
     * @brief connect
     * Connects to the VREP remote api server.
     * Calling this function is required before anything else can happen.
     * @param port
     * @param TIMEOUT_IN_MILISECONDS
     * @param MAX_TRY_COUNT
     * @return
     */
    bool connect(const int &port, const int& TIMEOUT_IN_MILISECONDS, const int& MAX_TRY_COUNT);
    bool connect(const std::string& ip, const int& port, const int& TIMEOUT_IN_MILISECONDS, const int& MAX_TRY_COUNT);
    /**
     * @brief disconnect
     * Call this after the last use of this object, or the clientid_ used
     * in this session might become unusable in the future.
     */
    void disconnect();
    /**
     * @brief disconnectAll
     * Tries disconnecting all remote API clients. Be careful with this.
     */
    void disconnect_all();

    /**
     * @brief startSimulation
     * Starts VREP simulation.
     */
    void start_simulation() const;
    /**
     * @brief stopSimulation
     * Stops VREP simulation
     */
    void stop_simulation()  const;

    bool is_simulation_running() const;

    /**
     * @brief getObjectHandle
     * Gets an object handle, using VREP_OP_BLOCKING
     * @param objectname
     * @return
     */
    int              get_object_handle(const std::string& objectname);
    /**
     * @brief getObjectHandles
     * A simpler way to call getObjectHandle with several objectnames.
     * @param objectnames
     * @return
     */
    std::vector<int> get_object_handles(const std::vector<std::string>& objectnames);

    /**
     * @brief getObjectTranslation
     * Gets the translation of an object as a quaternion.
     * @param handle
     * @param relative_to_handle
     * @param opmode
     * @return
     */
    DQ   get_object_translation(const int& handle, const int& relative_to_handle, const OP_MODES& opmode);
    DQ   get_object_translation(const int& handle, const std::string& relative_to_objectname, const OP_MODES& opmode)
    {
        return get_object_translation(handle,__get_handle_from_map(relative_to_objectname),opmode);
    }
    DQ   get_object_translation(const std::string& objectname, const int& relative_to_handle, const OP_MODES& opmode)
    {
        return get_object_translation(__get_handle_from_map(objectname),relative_to_handle,opmode);
    }
    DQ   get_object_translation(const std::string& objectname, const std::string& relative_to_objectname, const OP_MODES& opmode)
    {
        return get_object_translation(__get_handle_from_map(objectname),__get_handle_from_map(relative_to_objectname),opmode);
    }

    /**
     * @brief setObjectTranslation
     * Sets the translation of an object as a given quaternion.
     * @param handle
     * @param relative_to_handle
     * @param t
     * @param opmode
     */
    void set_object_translation(const int& handle, const int& relative_to_handle, const DQ& t, const OP_MODES& opmode) const;
    void set_object_translation(const int& handle, const std::string& relative_to_objectname, const DQ& t, const OP_MODES& opmode)
    {
        return set_object_translation(handle,__get_handle_from_map(relative_to_objectname),t,opmode);
    }
    void set_object_translation(const std::string& objectname, const int& relative_to_handle, const DQ& t, const OP_MODES& opmode)
    {
        return set_object_translation(__get_handle_from_map(objectname),relative_to_handle,t,opmode);
    }
    void set_object_translation(const std::string& objectname, const std::string& relative_to_objectname, const DQ& t, const OP_MODES& opmode)
    {
        return set_object_translation(__get_handle_from_map(objectname),__get_handle_from_map(relative_to_objectname),t,opmode);
    }

    /**
     * @brief getObjectRotation
     * Gets the object rotation as a quaternion.
     * @param handle
     * @param relative_to_handle
     * @param opmode
     * @return
     */
    DQ   get_object_rotation(const int& handle, const int& relative_to_handle, const OP_MODES& opmode);
    DQ   get_object_rotation(const int& handle, const std::string& relative_to_objectname, const OP_MODES& opmode)
    {
        return get_object_rotation(handle,__get_handle_from_map(relative_to_objectname),opmode);
    }
    DQ   get_object_rotation(const std::string& objectname, const int& relative_to_handle, const OP_MODES& opmode)
    {
        return get_object_rotation(__get_handle_from_map(objectname),relative_to_handle,opmode);
    }
    DQ   get_object_rotation(const std::string& objectname, const std::string& relative_to_objectname, const OP_MODES& opmode)
    {
        return get_object_rotation(__get_handle_from_map(objectname),__get_handle_from_map(relative_to_objectname),opmode);
    }


    /**
     * @brief setObjectRotation
     * Sets the object rotation as a given quaternion.
     * @param handle
     * @param relative_to_handle
     * @param r
     * @param opmode
     */
    void set_object_rotation(const int& handle, const int& relative_to_handle, const DQ& r, const OP_MODES& opmode) const;
    void set_object_rotation(const int& handle, const std::string& relative_to_objectname, const DQ& r, const OP_MODES& opmode)
    {
        return set_object_rotation(handle,__get_handle_from_map(relative_to_objectname),r,opmode);
    }
    void set_object_rotation(const std::string& objectname, const int& relative_to_handle, const DQ& r, const OP_MODES& opmode)
    {
        return set_object_rotation(__get_handle_from_map(objectname),relative_to_handle,r,opmode);
    }
    void set_object_rotation(const std::string& objectname, const std::string& relative_to_objectname, const DQ& r, const OP_MODES& opmode)
    {
        return set_object_rotation(__get_handle_from_map(objectname),__get_handle_from_map(relative_to_objectname),r,opmode);
    }

    /**
     * @brief getObjectPose
     * Gets the object pose as a dual quaternion.
     * @param handle
     * @param relative_to_handle
     * @param opmode
     * @return
     */
    DQ get_object_pose(const int& handle, const int& relative_to_handle, const OP_MODES& opmode);
    DQ get_object_pose(const int& handle, const std::string& relative_to_objectname, const OP_MODES& opmode)
    {
        return get_object_pose(handle,__get_handle_from_map(relative_to_objectname),opmode);
    }
    DQ get_object_pose(const std::string& objectname, const int& relative_to_handle, const OP_MODES& opmode)
    {
        return get_object_pose(__get_handle_from_map(objectname),relative_to_handle,opmode);
    }
    DQ get_object_pose(const std::string& objectname, const std::string& relative_to_objectname, const OP_MODES& opmode)
    {
        return get_object_pose(__get_handle_from_map(objectname),__get_handle_from_map(relative_to_objectname),opmode);
    }
    /**
     * @brief setObjectPose
     * Sets the object pose as a given dual quaternion.
     * @param handle
     * @param relative_to_handle
     * @param h
     * @param opmode
     */
    void set_object_pose(const int& handle, const int& relative_to_handle, const DQ& h, const OP_MODES& opmode) const;
    void set_object_pose(const int& handle, const std::string& relative_to_objectname, const DQ& h, const OP_MODES& opmode)
    {
        return set_object_pose(handle,__get_handle_from_map(relative_to_objectname),h,opmode);
    }
    void set_object_pose(const std::string& objectname, const int& relative_to_handle, const DQ& h, const OP_MODES& opmode)
    {
        return set_object_pose(__get_handle_from_map(objectname),relative_to_handle,h,opmode);
    }
    void set_object_pose(const std::string& objectname, const std::string& relative_to_objectname, const DQ& h, const OP_MODES& opmode)
    {
        return set_object_pose(__get_handle_from_map(objectname),__get_handle_from_map(relative_to_objectname),h,opmode);
    }

    /**
     * @brief getObjectPoses
     * Calls getObjectPose for a collection of handles.
     * @param handles
     * @param relative_to_handle
     * @param opmode
     * @return
     */
    std::vector<DQ> get_object_poses(const std::vector<int>& handles, const int& relative_to_handle, const OP_MODES& opmode);
    /**
     * @brief setObjectPoses
     * Calls setObjectPose for a collection of handles.
     * @param handles
     * @param relative_to_handle
     * @param hs
     * @param opmode
     */
    void            set_object_poses(const std::vector<int>& handles, const int& relative_to_handle, const std::vector<DQ>& hs, const OP_MODES& opmode) const;

    /**
     * @brief getJointPosition
     * Gets the position of a joint.
     * @param handle
     * @param opmode
     * @return
     */
    double   get_joint_position(const int& handle, const OP_MODES& opmode);
    /**
     * @brief setJointPosition
     * Sets the position of a joint.
     * @param handle
     * @param angle_rad
     * @param opmode
     */
    void     set_joint_position(const int& handle, const double& angle_rad, const OP_MODES& opmode) const;
    /**
     * @brief getJointPositions
     * Gets the position of a collection of joints.
     * @param handles
     * @param opmode
     * @return
     */
    VectorXd get_joint_positions(const std::vector<int>& handles, const OP_MODES& opmode);
    /**
     * @brief setJointPositions
     * Sets the positions of a collection of joints.
     * @param handles
     * @param angles_rad
     * @param opmode
     */
    void     set_joint_positions(const std::vector<int>& handles, const VectorXd& angles_rad, const OP_MODES& opmode) const;

    ///Deprecated
    int getHandle(const std::string& objectname, const OP_MODES& opmode);

private:
    std::map<std::string,int> name_to_handle_map_;

    int MAX_TRY_COUNT_;
    int TIMEOUT_IN_MILISECONDS_;
    int clientid_;
    long int global_retry_count_;
    std::atomic_bool* no_blocking_loops_;

    void __insert_or_update_map(const std::string& objectname, const int& handle);

    int __get_handle_from_map(const std::string& objectname);
    /**
     * @brief __remap_op_mode
     * Maps the VREP_INTERFACE_COMMAND_TYPES into operation modes that
     * VREP's remote API can understand.
     * @param opmode
     * @return
     */
    simxInt __remap_op_mode(const OP_MODES& opmode) const;

    /**
     * @brief __retry_function
     * Used to retry a function for MAX_TRY_COUNT_ times, each of which makes
     * the running thread sleep for TIMEOUT_IN_MILISECONDS_.
     * @param opmode
     * @return
     */
    void    __retry_function(const std::function<simxInt(void)> &f);
};

#endif

