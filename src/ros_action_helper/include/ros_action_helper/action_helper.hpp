
#ifndef RQT_ACQUISITION_CONTROLLER_ACTION_HELPER_HPP
#define RQT_ACQUISITION_CONTROLLER_ACTION_HELPER_HPP

#include <actionlib/client/simple_action_client.h>

/**
 * ROS actionlib helper macros. In case of using many actions, the implementation will quickly
 * become cumbersome and the code becomes cluttered.
 *
 * The following macros implement most of the requirements of an action in a client node.
 *
 * Usage:
 *
 * Assume we have a defined action Record.action which is supposed to trigger a camera capturing.
 * Integrating this action requires the following steps:
 *
 * Declaration my_client.hpp:
 *
 *   class my_client
 *   {
 *     ...
 *
 *     ACTION_IMPLEMENT(Record); // thats it for the declaration part!
 *
 *     ...
 *   };
 *
 * Implementation my_client.cpp
 *
 *   my_client::my_client():  ACTION(Record)("action_server_name"){ }
 *
 *   ACTION_ON_ACTIVE(my_client, record_action_namespace, Record) { ... }
 *   ACTION_ON_FEEDBACK(my_client, record_action_namespace, Record) { ... }
 *   ACTION_ON_DONE(my_client, record_action_namespace, Record) { ... }
 *
 *
 * An action is started by first setting the goal followed by the sending the goal
 *
 *  ACTION_GOAL(Record).some_attribute = ...;
 *  ACTION_SEND_GOAL(my_client, record_action_namespace, Record);
 *
 * Done.
 *
 *
 *
 * What the macros do:
 * The following is equivalent to the code presendted above
 *
 * Declaration my_client.hpp:
 *   class my_client
 *   {
 *     ...
 *   public:
 *      typedef  actionlib::SimpleActionClient<record_action_namespace::RecordAction>  RecordClient;
 *   private:
 *       private:
 *       void RecordActiveCB();
 *       void RecordDoneCB(const actionlib::SimpleClientGoalState state,
 *                         const record_action_namespace::RecordResultConstPtr result);
 *       void RecordFeedbackCB(record_action_namespace::RecordFeedbackConstPtr feedback);
 *       RecordClient Record_ac_;
 *       record_action_namespace::RecordGoal Record_goal_;
 *   }
 *
 * Implementation my_client.cpp
 *
 *   my_client::my_client(): Record_ac_("action_server_name") { ... }
 *
 *   void my_client::RecordActiveCB() { ... }
 *   void my_client::RecordDoneCB(const actionlib::SimpleClientGoalState state,
 *                              const record_action_namespace::RecordResultConstPtr result) { ... }
 *   void my_client::RecordFeedbackCB(record_action_namespace::RecordFeedbackConstPtr feedback) { ... }
 *
 * Starting an action:
 *
 *  Record_goal_.some_attribute = ...;
 *  Record_ac_.sendGoal(Record_goal_,
 *       boost::bind(&my_client::RecordDoneCB, this, _1, _2),
 *       boost::bind(&my_client::RecordActiveCB, this),
 *       boost::bind(&my_client::RecordFeedbackCB, this, _1))
 */

#define ACTION(action_name) action_name##_ac_
#define ACTION_GOAL(action_name) action_name##_goal_

#define ACTION_IMPLEMENT(action_namespace, action_name) \
    public: \
        typedef  actionlib::SimpleActionClient<action_namespace::action_name##Action> \
            action_name##Client;\
    private:\
        void action_name ## ActiveCB();\
        void action_name ## DoneCB(const actionlib::SimpleClientGoalState state,\
                                   const action_namespace::action_name##ResultConstPtr result);\
        void action_name ## FeedbackCB(action_namespace::action_name##FeedbackConstPtr feedback);\
        action_name##Client action_name##_ac_;\
        action_namespace::action_name##Goal action_name##_goal_;

#define ACTION_ON_ACTIVE(current_class_name, action_namespace, action_name)\
    void current_class_name::action_name ## ActiveCB()

#define ACTION_ON_FEEDBACK(current_class_name, action_namespace, action_name)\
    void current_class_name::action_name ## FeedbackCB(\
        action_namespace::action_name##FeedbackConstPtr feedback)

#define ACTION_ON_DONE(current_class_name, action_namespace, action_name)\
    void current_class_name::action_name ## DoneCB(\
        const actionlib::SimpleClientGoalState state,\
        const action_namespace::action_name##ResultConstPtr result)

#define ACTION_SEND_GOAL(current_class_name, action_namespace, action_name) \
    action_name##_ac_.sendGoal(action_name##_goal_, \
        boost::bind(&current_class_name::action_name##DoneCB, this, _1, _2),\
        boost::bind(&current_class_name::action_name##ActiveCB, this),\
        boost::bind(&current_class_name::action_name##FeedbackCB, this, _1))


#define ACTION_SHUTDOWN(action_name, is_active)\
    if (action_name##_ac_.isServerConnected()) \
    { \
        action_name##_ac_.cancelAllGoals(); \
        if (is_active) \
        { \
            action_name##_ac_.waitForResult(ros::Duration(0.5)); \
        } \
    } \
    else if (is_active) \
    { \
        action_name##_ac_.stopTrackingGoal(); \
    }

#endif
