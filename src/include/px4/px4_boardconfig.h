#define CONFIG_PLATFORM_POSIX 1
#define CONFIG_BOARD_PLATFORM "posix"
#define CONFIG_BOARD_NOLOCKSTEP 1
#define CONFIG_BOARD_TOOLCHAIN ""
#define CONFIG_BOARD_ARCHITECTURE ""
#define CONFIG_BOARD_ROMFSROOT "px4fmu_common"
#define CONFIG_BOARD_ROOTFSDIR "."
#define CONFIG_BOARD_LINKER_PREFIX ""
#define CONFIG_BOARD_COMPILE_DEFINITIONS ""
#define CONFIG_BOARD_TESTING 1
#define CONFIG_BOARD_ETHERNET 1
#define CONFIG_BOARD_SERIAL_URT6 ""
#define CONFIG_BOARD_SERIAL_GPS1 ""
#define CONFIG_BOARD_SERIAL_GPS2 ""
#define CONFIG_BOARD_SERIAL_GPS3 ""
#define CONFIG_BOARD_SERIAL_GPS4 ""
#define CONFIG_BOARD_SERIAL_GPS5 ""
#define CONFIG_BOARD_SERIAL_TEL1 ""
#define CONFIG_BOARD_SERIAL_TEL2 ""
#define CONFIG_BOARD_SERIAL_TEL3 ""
#define CONFIG_BOARD_SERIAL_TEL4 ""
#define CONFIG_BOARD_SERIAL_TEL5 ""
#define CONFIG_BOARD_SERIAL_RC ""
#define CONFIG_BOARD_SERIAL_WIFI ""
#define CONFIG_BOARD_SERIAL_EXT2 ""
#define CONFIG_DRIVERS_CAMERA_TRIGGER 1
#define CONFIG_DRIVERS_DISTANCE_SENSOR_LIGHTWARE_LASER_SERIAL 1
#define CONFIG_DRIVERS_GPS 1
#define CONFIG_DRIVERS_OSD_MSP_OSD 1
#define CONFIG_DRIVERS_TONE_ALARM 1
#define CONFIG_MODULES_AIRSHIP_ATT_CONTROL 1
#define CONFIG_MODULES_AIRSPEED_SELECTOR 1
#define CONFIG_MODULES_ATTITUDE_ESTIMATOR_Q 1
#define CONFIG_MODULES_CAMERA_FEEDBACK 1
#define CONFIG_MODULES_COMMANDER 1
#define CONFIG_MODULES_CONTROL_ALLOCATOR 1
#define CONFIG_MODULES_DATAMAN 1
#define CONFIG_MODULES_DIFFERENTIAL_DRIVE 1
#define CONFIG_MODULES_EKF2 1
#define CONFIG_EKF2_VERBOSE_STATUS 1
#define CONFIG_EKF2_MULTI_INSTANCE 1
#define CONFIG_EKF2_AIRSPEED 1
#define CONFIG_EKF2_AUX_GLOBAL_POSITION 1
#define CONFIG_EKF2_AUXVEL 1
#define CONFIG_EKF2_BAROMETER 1
#define CONFIG_EKF2_BARO_COMPENSATION 1
#define CONFIG_EKF2_DRAG_FUSION 1
#define CONFIG_EKF2_EXTERNAL_VISION 1
#define CONFIG_EKF2_GNSS 1
#define CONFIG_EKF2_GNSS_YAW 1
#define CONFIG_EKF2_GRAVITY_FUSION 1
#define CONFIG_EKF2_MAGNETOMETER 1
#define CONFIG_EKF2_OPTICAL_FLOW 1
#define CONFIG_EKF2_RANGE_FINDER 1
#define CONFIG_EKF2_SIDESLIP 1
#define CONFIG_EKF2_TERRAIN 1
#define CONFIG_EKF2_WIND 1
#define CONFIG_MODULES_EVENTS 1
#define CONFIG_MODULES_FLIGHT_MODE_MANAGER 1
#define CONFIG_MODULES_FW_ATT_CONTROL 1
#define CONFIG_MODULES_FW_AUTOTUNE_ATTITUDE_CONTROL 1
#define CONFIG_MODULES_FW_POS_CONTROL 1
#define CONFIG_FIGURE_OF_EIGHT 1
#define CONFIG_MODULES_FW_RATE_CONTROL 1
#define CONFIG_MODULES_GIMBAL 1
#define CONFIG_MODULES_GYRO_CALIBRATION 1
#define CONFIG_MODULES_GYRO_FFT 1
#define CONFIG_MODULES_LAND_DETECTOR 1
#define CONFIG_MODULES_LANDING_TARGET_ESTIMATOR 1
#define CONFIG_MODULES_LOAD_MON 1
#define CONFIG_MODULES_LOCAL_POSITION_ESTIMATOR 1
#define CONFIG_MODULES_LOGGER 1
#define CONFIG_MODULES_MAG_BIAS_ESTIMATOR 1
#define CONFIG_MODULES_MANUAL_CONTROL 1
#define CONFIG_MODULES_MAVLINK 1
#define CONFIG_MAVLINK_DIALECT "development"
#define CONFIG_MODULES_MC_ATT_CONTROL 1
#define CONFIG_MODULES_MC_AUTOTUNE_ATTITUDE_CONTROL 1
#define CONFIG_MODULES_MC_HOVER_THRUST_ESTIMATOR 1
#define CONFIG_MODULES_MC_POS_CONTROL 1
#define CONFIG_MODULES_MC_RATE_CONTROL 1
#define CONFIG_MODULES_NAVIGATOR 1
#define CONFIG_MODE_NAVIGATOR_VTOL_TAKEOFF 1
#define CONFIG_MODULES_PAYLOAD_DELIVERER 1
#define CONFIG_MODULES_RC_UPDATE 1
#define CONFIG_MODULES_REPLAY 1
#define CONFIG_MODULES_ROVER_POS_CONTROL 1
#define CONFIG_MODULES_SENSORS 1
#define CONFIG_SENSORS_VEHICLE_AIRSPEED 1
#define CONFIG_SENSORS_VEHICLE_AIR_DATA 1
#define CONFIG_SENSORS_VEHICLE_ANGULAR_VELOCITY 1
#define CONFIG_SENSORS_VEHICLE_ACCELERATION 1
#define CONFIG_SENSORS_VEHICLE_GPS_POSITION 1
#define CONFIG_SENSORS_VEHICLE_MAGNETOMETER 1
#define CONFIG_SENSORS_VEHICLE_OPTICAL_FLOW 1
#define CONFIG_COMMON_SIMULATION 1
#define CONFIG_MODULES_SIMULATION_BATTERY_SIMULATOR 1
#define CONFIG_MODULES_SIMULATION_GZ_BRIDGE 1
#define CONFIG_MODULES_SIMULATION_PWM_OUT_SIM 1
#define CONFIG_MODULES_SIMULATION_SENSOR_AIRSPEED_SIM 1
#define CONFIG_MODULES_SIMULATION_SENSOR_BARO_SIM 1
#define CONFIG_MODULES_SIMULATION_SENSOR_GPS_SIM 1
#define CONFIG_MODULES_SIMULATION_SENSOR_MAG_SIM 1
#define CONFIG_MODULES_SIMULATION_SIMULATOR_MAVLINK 1
#define CONFIG_MODULES_SIMULATION_SIMULATOR_SIH 1
#define CONFIG_MODULES_TEMPERATURE_COMPENSATION 1
#define CONFIG_MODULES_UUV_ATT_CONTROL 1
#define CONFIG_MODULES_UUV_POS_CONTROL 1
#define CONFIG_MODULES_UXRCE_DDS_CLIENT 1
#define CONFIG_MODULES_VTOL_ATT_CONTROL 1
#define CONFIG_SYSTEMCMDS_ACTUATOR_TEST 1
#define CONFIG_SYSTEMCMDS_BSONDUMP 1
#define CONFIG_SYSTEMCMDS_DYN 1
#define CONFIG_SYSTEMCMDS_FAILURE 1
#define CONFIG_SYSTEMCMDS_LED_CONTROL 1
#define CONFIG_SYSTEMCMDS_PARAM 1
#define CONFIG_SYSTEMCMDS_PERF 1
#define CONFIG_SYSTEMCMDS_SD_BENCH 1
#define CONFIG_SYSTEMCMDS_SHUTDOWN 1
#define CONFIG_SYSTEMCMDS_SYSTEM_TIME 1
#define CONFIG_SYSTEMCMDS_TESTS 1
#define CONFIG_SYSTEMCMDS_TOPIC_LISTENER 1
#define CONFIG_SYSTEMCMDS_TUNE_CONTROL 1
#define CONFIG_SYSTEMCMDS_UORB 1
#define CONFIG_SYSTEMCMDS_VER 1
#define CONFIG_SYSTEMCMDS_WORK_QUEUE 1
#define CONFIG_EXAMPLES_DYN_HELLO 1
#define CONFIG_EXAMPLES_FAKE_GPS 1
#define CONFIG_EXAMPLES_FAKE_IMU 1
#define CONFIG_EXAMPLES_FAKE_MAGNETOMETER 1
#define CONFIG_EXAMPLES_HELLO 1
#define CONFIG_EXAMPLES_PX4_MAVLINK_DEBUG 1
#define CONFIG_EXAMPLES_PX4_SIMPLE_APP 1
#define CONFIG_EXAMPLES_WORK_ITEM 1
