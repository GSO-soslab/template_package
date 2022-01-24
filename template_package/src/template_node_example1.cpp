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
      * Make the code spin
      */
     my_class.spin();

     return 0;
}