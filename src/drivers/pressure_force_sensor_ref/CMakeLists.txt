px4_add_module(
    MODULE modules__pressure_force_sensor_ref
    MAIN pressure_force_sensor_ref
    STACK_MAIN 2000
    SRCS
        pressure_force_sensor_ref.cpp
    DEPENDS
	px4_work_queue
)
