#include "ev3lib_sensor.h"

ev3_sensor_ptr sensors_list = NULL;
ev3_sensor_ptr sensorUS = NULL, sensorGyro = NULL, sensorColor = NULL, sensorCompass = NULL;

void init_sensor(){
	//Loading all sensors
	sensors_list = ev3_load_sensors();

	//Get ultrasonic sensor
	sensorUS = ev3_search_sensor_by_port(sensors_list, SENSOR_US_PORT);
	if(sensorUS == NULL){
		error_message("[INIT_SENSOR] Ultra sonic sensor not found");
	}
	printf("[INIT_SENSOR] SensorUS: %s [%i]\n", sensorUS->driver_name, sensorUS->port);
	ev3_mode_sensor_by_name(sensorUS, "US-DIST-CM");
	ev3_open_sensor(sensorUS);

	//Get gyro sensor
    sensorGyro = ev3_search_sensor_by_port(sensors_list, SENSOR_GYRO_PORT);
    if(sensorGyro == NULL){
        error_message("[INIT_SENSOR] Gyro sensor not found");
    }
	printf("[INIT_SENSOR] SensorGyro: %s [%i]\n", sensorGyro->driver_name, sensorGyro->port);
    ev3_open_sensor(sensorGyro);
    
    //Get compass sensor
    sensorCompass = ev3_search_sensor_by_port(sensors_list, SENSOR_COMPASS_PORT);
    if(sensorCompass == NULL){
        error_message("[INIT_SENSOR] Compass sensor not found");
    }
	printf("[INIT_SENSOR] SensorCompass: %s [%i]\n", sensorCompass->driver_name, sensorCompass->port);
    ev3_open_sensor(sensorCompass);
    
	//Get Color sensor
    sensorColor = ev3_search_sensor_by_port(sensors_list, SENSOR_COLOR_PORT);
    if(sensorColor == NULL){
		//printf("[INIT_SENSOR] Color sensor not found\n");
        error_message("[INIT_SENSOR] Color sensor not found");
    }
	printf("[INIT_SENSOR] SensorColor: %s [%i]\n", sensorColor->driver_name, sensorColor->port);
	//ev3_mode_sensor_by_name(sensorColor, "COL-REFLECT");
	ev3_mode_sensor_by_name(sensorColor, "COL-COLOR");
	ev3_open_sensor(sensorColor);
}

void start_compass_calibration(){
    if(sensorCompass == NULL){
        printf("Compass sensor not found\n");
    }
    ev3_mode_sensor_by_name(sensorCompass, "CALIBRATION");
}

void stop_compass_calibration(){
    if(sensorCompass == NULL){
        printf("Compass sensor not found\n");
    }
    ev3_mode_sensor_by_name(sensorCompass, "COMPASS");
}

void update_sensor(uint8_t id){
	switch(id){
		case SENSOR_US:
			ev3_update_sensor_bin(sensorUS);
			break;
		case SENSOR_GYRO:
			ev3_update_sensor_bin(sensorGyro);
			break;
		case SENSOR_COLOR:
			ev3_update_sensor_bin(sensorColor);
			break;
        case SENSOR_COMPASS:
            ev3_update_sensor_bin(sensorCompass);
            break;
	}
}

int32_t get_sensor_value(uint8_t id){
	switch(id){
        case SENSOR_US:
            return sensorUS->bin_data[0].s32;
        case SENSOR_GYRO:
            return sensorGyro->bin_data[0].s32;
        case SENSOR_COLOR:
      		return sensorColor->bin_data[0].s32;
        case SENSOR_COMPASS:
            return sensorCompass->bin_data[0].s32;
        }
	return 0;
}

void destroy_sensors(){
	if(sensors_list != NULL)
		ev3_delete_sensors(sensors_list);
}
