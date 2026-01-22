px4_add_module(
    MODULE modules__pressure_force_sensor
    MAIN pressure_force_sensor
    STACK_MAIN 2000
    SRCS
        pressure_force_sensor.cpp
    DEPENDS
	px4_work_queue
)
