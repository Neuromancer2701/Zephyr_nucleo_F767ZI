#include <zephyr.h>
#include <rcl/rcl.h>
#include <micro_ros/utilities.h>

void main(void) {
    rcl_node_t node;
    rcl_init_options_t options = rcl_get_zero_initialized_init_options();
    
    rcl_init_options_init(&options, rcl_get_default_allocator());
    
    rcl_node_options_t node_ops = rcl_node_get_default_options();
    rcl_node_init(&node, "zephyr_node", "", &options, &node_ops);

    while (1) {
        rcl_spin_some(&node, RCL_MS_TO_NS(10));
        k_sleep(K_SECONDS(1));
    }
}