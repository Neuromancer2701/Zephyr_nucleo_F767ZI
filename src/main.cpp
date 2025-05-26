#include <zephyr.h>
#include <stdio.h> // For printk

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

// rmw_microros.h might be needed if explicit transport configuration is used.
// However, with Zephyr modules, Kconfig often handles this.
// #include <rmw_microros/rmw_microros.h>

// Basic error checking macros
#ifndef RCCHECK
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printk("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); k_oops();}}
#endif
#ifndef RCSOFTCHECK
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printk("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
#endif

// Global variables for publisher and message
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
static int counter = 0; // Counter for the message data

// Timer callback to publish message
void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time; // Unused parameter

    if (timer != NULL) {
        msg.data = counter++;
        RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
        printk("Sent message: %d\n", msg.data);
    }
}

// Main function
void main(void)
{
    // NOTE: Transport initialization (e.g., UDP or Serial) is typically handled
    // by the micro-ROS Zephyr module via Kconfig settings (CONFIG_MICRO_ROS_TRANSPORT_UDP=y or
    // CONFIG_MICRO_ROS_TRANSPORT_SERIAL=y).
    // If manual setup were needed, it would look something like:
    // rmw_uros_set_custom_transport(true, (void *)"192.168.1.100", 8888); // Example for UDP
    // But this is generally not required when using the Zephyr module.

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_node_t node;
    rcl_timer_t timer;
    rclc_executor_t executor;

    // Initialize rclc support
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // Create a ROS2 node
    RCCHECK(rclc_node_init_default(&node, "zephyr_int32_publisher", "", &support));

    // Create a publisher
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "zephyr_int_topic")); // Topic name

    // Initialize message data (counter will update this in callback)
    msg.data = 0;

    // Create a timer to trigger the publisher regularly
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(1000), // 1000 ms = 1 second period
        timer_callback));

    // Create an executor to handle the timer
    // The '1' indicates one handle (our timer)
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    printk("Micro-ROS Zephyr publisher node started. Publishing integers on 'zephyr_int_topic'.\n");

    // Main loop: spin the executor and sleep
    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)); // Spin executor for 100ms
        k_msleep(100); // Sleep for 100ms
    }

    // Clean up (normally not reached in an embedded application's main loop)
    // RCCHECK(rcl_publisher_fini(&publisher, &node));
    // RCCHECK(rcl_node_fini(&node));
    // RCCHECK(rclc_support_fini(&support));
}
