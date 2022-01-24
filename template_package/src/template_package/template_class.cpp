#include "template_package/template_class.h"
#include "algorithm"

TemplateClass::TemplateClass() {

    /**
     * Initializing node handlers
     *
     * We are initializing public and private node handlers. Argument of the
     * constructor defines the node handler's namespace. "~" defines node's own
     * space, hence private node handler.
     */
    m_public_nh.reset(new ros::NodeHandle());

    m_private_nh.reset(new ros::NodeHandle("~"));

    /**
     * Initializing subscriber.
     *
     * First parameter to ros::NodeHandle::subscribe method is the topic name.
     * Second is queue_size. Third one is function point to the callback. Forth
     * argument is this class's pointer. We are supplying "this" as an argument
     * to make callback to call this object's private method.
     *
     * @see http://docs.ros.org/en/kinetic/api/roscpp/html/classros_1_1NodeHandle.html
     */
    m_subscriber = m_public_nh->subscribe(TOPIC_CHATTER, 10, &TemplateClass::f_example_callback, this);

    /**
     * Initializing publisher
     *
     * First parameter to ros::NodeHandle::advertise method is the topic name.
     * Second argument is the queue_size.
     *
     * @see http://docs.ros.org/en/kinetic/api/roscpp/html/classros_1_1NodeHandle.html
     */
    m_publisher = m_public_nh->advertise<std_msgs::String>(TOPIC_SPEAKER, 10);

    /**
     * Reading a parameter from ROS parameter server.
     *
     * There are several ways to read a parameter from ros server. Use which ever the one
     * you needed.
     */
    if(!m_private_nh->getParam(CONF_MY_STRING, m_message)) {
        ROS_WARN_STREAM("Inform me when I don't supply this variable if its important. Say something like:");
        ROS_WARN_STREAM("Parameter \"" << CONF_MY_STRING << "\" is not supplied but needed!");
    }

    /**
     * Another way of reading a value from the parameter server.
     */
    m_private_nh->param<int>(CONF_MY_INTEGER, m_integer, CONF_MY_INTEGER_DEFAULT);

}

void TemplateClass::f_example_callback(const std_msgs::String::ConstPtr &msg) {
    m_message = msg->data;


    std::string reversed;
    bool result = f_reverser(m_message, &reversed);

    if(result) {
        std_msgs::String out;
        out.data = reversed;
        m_publisher.publish(out);
    }

}

bool TemplateClass::f_reverser(const std::string& input, std::string *output) {

    if(input.empty()) {
        return false;
    } else  {
        *output = input;
        std::reverse(output->begin(), output->end());
        return true;
    }

}

void TemplateClass::spin_once() {

    /**
     * Calling ros::spinOnce will fire the queued ros topic events.
     * Messages will wait in the queue until ros::spinOnce is called.
     *
     * Using ros::spinOnce may cause processing older messages in the queue.
     * Choosing between ros::spinOnce and ros::spin is important.
     */

    if(m_message.empty()) {
        ROS_INFO_STREAM("I don't do anything yet. Jake is going to make me send a message");
    } else {
        ROS_INFO_STREAM("Jake told me " << m_message);
    }

    // after we spin once we need to tell ros to do the same
    ros::spinOnce();
}

void TemplateClass::spin(){

    /**
     * ros::Rate tries to ensure that loop will repeat itself in the specified frequency.
     */
    ros::Rate loop_rate(0.1); // in hz

    while (ros::ok()) {
        spin_once();
        loop_rate.sleep();
    }
}