idf_component_register(
    SRCS "cansat_project.cpp"
         "rtos/tasks.cpp"
         "tmp117/tmp117.c"
         "lsm9ds1/lsm9ds1_hal.c"
         "lsm9ds1/lsm9ds1_reg.c"
         "dps310/dps310.c"
         "neo6m/neo6m.c"
         "gps_speed/2d_velocity.c"
         "i2c/i2c_bus.c"
         "fs/fs_handler.c"
         "rtos/rfm95_task.cpp"
    INCLUDE_DIRS "." "rtos" "tmp117" "lsm9ds1" "dps310" "neo6m""gps_speed" "EspHal" "i2c" "fs"
)
# This automatically builds the data/ folder and flashes it to the 'storage' partition
spiffs_create_partition_image(storage ../data FLASH_IN_PROJECT)