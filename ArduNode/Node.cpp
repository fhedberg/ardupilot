#include "Node.h"

#define SCHED_TASK(func, rate_hz, max_time_micros) SCHED_TASK_CLASS(Node, &node, func, rate_hz, max_time_micros)

const AP_Scheduler::Task Node::scheduler_tasks[] = {
   SCHED_TASK(ahrs_update, 400, 400),
};

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

Node::Node(void)
{

}

void Node::setup()
{
    // initialise console serial port
    serial_manager.init_console();

    // load the default values of variables listed in var_info[]
    AP_Param::setup_sketch_defaults();

    // initialise the main loop scheduler
    scheduler.init(&scheduler_tasks[0], ARRAY_SIZE(scheduler_tasks), MASK_LOG_PM);
}

void Node::loop()
{
    scheduler.loop();
}

void Node::ahrs_update()
{
   hal.console->printf("ahrs\n");
}

Node node;

AP_HAL_MAIN_CALLBACKS(&node);
