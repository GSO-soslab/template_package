#ifndef TEMPLATE_CLASS_H
#define TEMPLATE_CLASS_H

#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * @brief A template class
 *
 * Style Guide:
 * - Use "PascalCase" for class names.
 * - Use "snake_case" for member and method names.
 * - Use doxygen style comments for documenting your code.
 * - Document your code in the header file.
 * - Use "m_" prefix for private members.
 * - Use "f_" prefix for private members.
 * - Use pointers rather than references for modifying variables within objects.
 * - Use "static constexpr" for defining constants.
 * - Use ALL_CAPITAL for constants.
 * - Define constants within the class definition.
 * - A library must not call exit().
 * - Avoid using MACRO descriptions.
 */
class TemplateClass {
private:

    /**
     * @brief An integer value
     *
     * @note Use "m_" prefix for private members
     * Begin private member names with prefix "m_". When you are using
     * an IDE, code completion becomes easier.
     */
    int m_integer;

    /**
     * @brief A message value
     *
     * @note Use "m_" prefix for private members
     * A private member.
     */
    std::string m_message;

    /**
     * @brief An example subscriber object
     *
     * @note Use "m_" prefix for private members
     * Subscriber objects has to be stored in order them to work properly.
     * After being created with ros::NodeHandle::Subscribe method, if not stored
     * in a memory location, object destroys itself when it gets out of the scope.
     * ros::NodeHandle::Subscribe doesn't register a subscriber to a node handle,
     * it creates a subscriber object instead.
     */
    ros::Subscriber m_subscriber;

    /**
     * @brief An example publisher object
     *
     * Create a publisher object if you need it.
     */
    ros::Publisher m_publisher;

    /**
     * @brief An example subscriber callback
     *
     * @note Use prefix "f_" for private methods.
     *
     * This callback is fired whenever there is a message in it's topic.
     *
     * @param msg A basic string message
     */
    void f_example_callback(const std_msgs::String::ConstPtr& msg);


    /**
     * @brief A fancy function
     *
     * Those type of fancy functions can be used in some cases. In scenario below
     * return value of the function is used for determining success status of the
     * function. However, actual result is outputted using a point back to the user.
     * In such cases, using reference would be sufficient but it would decrease the
     * code readability.
     *
     * It is preferable to use const reference to prevent unnecessary copies.
     *
     * @see https://google.github.io/styleguide/cppguide.html#Reference_Arguments
     *
     * @param input Input string to be reversed
     * @param output Resulting string
     */
    static bool f_reverser(const std::string& input, std::string *output);


    /**
     * @brief A constant
     *
     * A parameter name to describe "my_string".
     *
     * Avoid defining static strings within the code. Use predefined
     * constants instead. There might be more that one place that
     * you need to use them. Changing every one of them tedious job.
     *
     * See how they are used in the implementation.
     */
    static constexpr const char * CONF_MY_STRING = "my_string";

    /**
     * @brief Another constant
     *
     * A parameter name to describe "my_integer".
     *
     */
    static constexpr const char * CONF_MY_INTEGER = "my_integer";

    /**
     * @brief Yet another constnant.
     *
     * Define default values for your parameters as a constant in
     * the header file. It will make it easier to analyze the code
     */
    static constexpr const int CONF_MY_INTEGER_DEFAULT = 42;

    //! @brief A constant that defines the topic name
    static constexpr const char * TOPIC_CHATTER = "chatter";

    //! @brief A constant that defines the topic name
    static constexpr const char * TOPIC_SPEAKER = "speaker";


protected:

    /**
     * @brief Public ROS node handler
     *
     * Anything that is created using public node handler will not have any namespace.
     * For example a publisher is created using public node handler named "chatter", it
     * will publish to topic "/chatter". Not, "/template_node/chatter".
     *
     * Use public node handler to create subscriber and publishers. This will allow easy
     * remapping configuration in the launch file.
     */
    ros::NodeHandlePtr m_public_nh;

    /**
     * @brief Private ROS node handler
     *
     * Anything created using private node handler will be in the private namespace.
     * For example if a subscriber is created using private node handler name "say", it
     * will subscribe to the topic "/template_node/say". That example assumes that node
     * name is "template_node".
     *
     * Use private node handler to consume ros parameters.
     */
    ros::NodeHandlePtr m_private_nh;

public:

    /**
     * @brief Default constructor
     */
    TemplateClass();

    /**
     * @brief spin is designed to run the node continuously.
     *
     * Depending on the need, writing a spin method can be preferred over
     * using ros::spin(). If that is the case, method should be called
     * right after the object is created.
     */
    void spin();

    /**
     * @brief spin_once is intended to run the loop just once
     */
    void spin_once();

};

#endif // TEMPLATE_CLASS_H
