#include "template_package/template_class.h"

/**
 * @param argc Number of arguments supplied
 * @param argv Supplied arguments
 * @return Not needed.
 */
int main(int argc, char *argv[]) {

    /**
     * Initialize ROS node.
     */
    ros::init(argc, argv, "class_node_example");

    /**
     * instantiate the object
     */
    TemplateClass my_class;

    /**
     * Make the code spin using ros::spin().
     *
     * Using ros::spin will not call TemplateClass::spin!.
     * If you want TemplateClass::spin method to spin, you should
     * create a thread instead if you want to use ros::spin.
     */
     ros::spin();

    return 0;
}