/** ****************************************************************
 * @file model.c
 * @brief Simple model for decision making in a TRP with Khepera IV robot.
 * @author Louis L'Haridon
 * @version 0.1
 * @date 20 février 2022
 *
 * Decision making model based on a TRP homeostatis model with a Khepera IV robot.
***************************************************************** */
#include <khepera/khepera.h>
#include <pthread.h>
#include <stdio.h> 
#include <stdlib.h>
#include <math.h>

#define _USE_MATH_DEFINES ///< for using math constants
#define SPEED 200  ///< speed basic input
#define TIME 100000///< time for model update (in ms)
#define MAXBUFFERSIZE 128 ///< Buffer size for robot communication
#define MAX_DIST 500 ///< Maximum distance for ir sensor
#define MIN_DIST 80 ///< or 70 | minimum distance for ir sensor

knet_dev_t * dsPic; ///< robot pic microcontroller access

float left_speed; ///< speed of left motor
float right_speed; ///< speed of right motor

float var_energy  = 1.0; ///< physological variable for ernergy
float var_tegument = 1.0; ///< physological variable for tegument
float var_integrity = 1.0; ///< physological variable for integrity

float def_energy  = 1.0; ///< deficit for ernergy
float def_tegument = 1.0; ///< deficit for tegument
float def_integrity = 1.0; ///< deficit for integrity

float cue_energy  = 1.0; ///< cue for ernergy
float cue_tegument = 1.0; ///< cue for tegument
float cue_integrity = 1.0; ///< cue for integrity

float mot_energy  = 1.0; ///< motivation for ernergy
float mot_tegument = 1.0; ///< motivation for tegument
float mot_integrity = 1.0; ///< motivation for integrity

int _secure_led_animation = 0; ///< to secure threading led animation

int prev_sensors[8]; ///< to store previous sensors values for
int sensors[8]; ///< actual sensors values

float speed[8]; ///< table for speeed based on IR sensor values
float circ_speed[7]; ///< table for circular speeed based on IR sensor values (size is n-1 because of circular speeed)

/** ****************************************************************
 * Display robot battery informations
 * 
 * @brief display battery info
 * @return : 0
***************************************************************** */
int display_battery(){
	char buf[32]; // Uses 12 bytes, extra space for future compat
	kh4_battery_status(buf, dsPic);
	printf("Battery charge: %d%%\n", buf[3]);
	printf("Current: %4.0f mA\n",*(short*)(buf+4)*0.07813);
	printf("Temperature: %3.1f C\n",*(short*)(buf+8)*0.003906);
	printf("Voltage: %4.0f mV\n",*(short*)(buf+10)*9.76);
	printf("Charger: %s\n",kh4_battery_charge(dsPic)?"plugged":"unplugged");
	return 0;
}

/** ****************************************************************
 * Set robot leds
 * 
 * @param left, color for left led
 * @param right, color for right led 
 * @param back, color for back led 
 * @note For color selection : (0 off, 1 white, 2 green, 3 blue, 4 red)
 * @brief function that set color for 4 robot leds
 * @return : 0 when ok
***************************************************************** */
int set_leds(int left, int right, int back){
	int32_t lr, lg, lb, rr, rg, rb, br, bg, bb;
	switch(left){
		case 0: // off
			lr=0; lg=0; lb=0;
			break;
		case 1: //white
			lr=255; lg=255; lb=255;
			break;
		case 2: //green
			lr=0; lg=255; lb=0;
			break;
		case 3: //blue
			lr=0; lg=0; lb=255;
			break;
		case 4: //red
			lr=255; lg=0; lb=0;
			break;
	}
	switch(right){
		case 0: // off
			rr=0; rg=0; rb=0;
			break;
		case 1: //white
			rr=255; rg=255; rb=255;
			break;
		case 2: //green
			rr=0; rg=255; rb=0;
			break;
		case 3: //blue
			rr=0; rg=0; rb=255;
			break;
		case 4: //red
			rr=255; rg=0; rb=0;
			break;
	}
	switch(back){
		case 0: // off
			br=0; bg=0; bb=0;
			break;
		case 1: //white
			br=255; bg=255; bb=255;
			break;
		case 2: //green
			br=0; bg=255; bb=0;
			break;
		case 3: //blue
			br=0; bg=0; bb=255;
			break;
		case 4: //red
			br=255; bg=0; bb=0;
			break;
	}

	kh4_SetRGBLeds(lr, lg, lb, rr, rg, rb, br, bg, bb, dsPic);
	return 0;
}

/** ****************************************************************
 * Turn off leds 
 * 
 * @brief function that turn of all leds
 * @return : 0 when ok
***************************************************************** */
int turn_off_leds(){
	set_leds(0,0,0);
	return 0;
}

/** ****************************************************************
 * Robot death animation 
 * 
 * @brief function that make animation for death
 * @return : 0 when ok
***************************************************************** */
int death_animation(){
	int animation_time = 50000;
	int i;
	set_leds(0,0,0);
	usleep(animation_time);	
	for(i=0; i<4; i++){
		set_leds(2,0,0);
		usleep(animation_time);
		set_leds(0,2,0);
		usleep(animation_time);
		set_leds(0,0,2);
		usleep(animation_time);
	}	
	for(i=0; i<4; i++){
		set_leds(4,0,0);
		usleep(animation_time);
		set_leds(0,4,0);
		usleep(animation_time);
		set_leds(0,0,4);
		usleep(animation_time);
	}	
	for(i=0; i<6; i++){
		set_leds(4,4,4);
		usleep(animation_time);
		set_leds(0,0,0);
		usleep(animation_time);
	}	

	return 0;
}
 
/** ****************************************************************
 * Robot damage animation 
 * 
 * @brief function that make animation when robot has damage
 * @note _secure_led_animation is used to secure threading
***************************************************************** */
void* damage_animation(void *args){
	_secure_led_animation = 1;
	int animation_time = 50000;
	int i;
	set_leds(0,0,0);
	usleep(animation_time);	
	for(i=0; i<5; i++){
		set_leds(4,0,0);
		usleep(animation_time);
		set_leds(0,4,0);
		usleep(animation_time);
		set_leds(0,0,4);
		usleep(animation_time);
	}	
	_secure_led_animation = 0;
}

/** ****************************************************************
 * Function that takes motivations as input and return behaviral group
 * 
 * @brief Winner take all for decision making
 * @param m1 motivation for ernergy
 * @param m2 motivation for tegument
 * @param m3 motivation for integrity 
 * @return 1 if energy, 2 if tegument, 3 if integrity, -1 if error
 * @note WTA is used for selection
***************************************************************** */
int winner_takes_all(float m1, float m2, float m3){
	if((m1 > m2) && (m1 > m3))
		return 1;
	else if((m2 > m1) && (m2 > m3))
		return 2;
	else if((m3 > m1) && (m3 > m2))
		return 3;
	else
		return -1;
}

/** ****************************************************************
 * Make robot stop
 * 
 * @brief stop robot's motor
 * @return : none
 * @note Robot's wheels are stopeed and robot is set in stop mode.
***************************************************************** */
int stop_moving(){
	// Stop wheel motors
	printf("Stopping motors\n");
	kh4_SetMode(kh4RegSpeed, dsPic);
	kh4_set_speed(0, 0, dsPic);
	// LEDs off
	kh4_SetRGBLeds(0, 0, 0, 0, 0, 0, 0, 0, 0, dsPic);
	kh4_SetMode(kh4RegIdle, dsPic);
	return 0;
}

/** ****************************************************************
 * Make robot move
 * @brief function to make robot move
 * @param motor_left speed of robot left wheel in [-1.0,1.0] range
 * @param motor_right speed of robot right wheel in [-1.0,1.0] range 
 * @return : none
 * @note speed is computed with SPEED int, value is set to 200
***************************************************************** */
int move(float motor_left, float motor_right){
	kh4_SetMode(kh4RegSpeed, dsPic);
	if (kh4_set_speed(motor_left*SPEED, motor_right*SPEED, dsPic) < 0){
		printf("ERROR: Fail on set_speed\n");
		return -1;
	}
}

/** ****************************************************************
 * Control robot with keyboard
 * 
 * @return : none
 * @brief Robot is controlled with zqsd input, a for quit, e for stop
***************************************************************** */
int run(void){
	char ctrl;  // char used for keyboard input
	ctrl = 'r';
	while(ctrl!='a'){
		printf("z,q,s,d for robot control, e for stop, a for exit\n");
		scanf("%c", &ctrl);
		printf("%c \n", ctrl);
		switch(ctrl){
			case 'z':
				printf("Move forward\n");
				move(1.0,1.0);
			break;
			case 'q':
				printf("Move left\n");
				move(-1.0,1.0);
			break;
			case 's':
				printf("Move backwardt\n");
				move(-1.0,-1.0);
			break;
			case 'd':
				printf("Move right\n");
				move(1.0,-1.0);
			break;
			case 'e':
				stop_moving();
			break;
			case 'a':
				printf("Exit program\n");
				stop_moving();
			break;
			default:
				printf("Error : Unknown command\n");
		}
	}
	
}

/** ****************************************************************
 * Induce damage
 * 
 * @return 0 when ok
 * @param level, the level of damage from 0 to 1
 * @brief function that decrease physiological variable for integrity 
***************************************************************** */
int induce_damage(float level){
	var_integrity -= (level*0.01);
	update_vars(0);
	if(_secure_led_animation == 0){
		// Thread id
		pthread_t threadId;
		// Create a thread that will function damage_animation()
		int err = pthread_create(&threadId, NULL, &damage_animation, NULL);
		// Check if thread is created sucessfuly
	}
	return 0;
}

/** ****************************************************************
 * Read and print our sensors
 * 
 * @return 0 if ok, -2 if error
 * @brief function to read and print robot's ir and us sensors
***************************************************************** */
int read_and_print_sensors(void){
	char Buffer[MAXBUFFERSIZE], buf[MAXBUFFERSIZE];
	int i, sensor, ret;
	if(kh4_proximity_ir((char *)Buffer, dsPic)>=0){	
		printf("Reading sensor proximity \n");
		for (i=0;i<12;i++){
			sensor=(Buffer[i*2] | Buffer[i*2+1]<<8);
			printf(" %d ",sensor);
		}
		printf("\n");																		
		ret = 0;	
	}
	else
		ret = -2;

	if(kh4_measure_us((char *)Buffer, dsPic)>=0)
	{
		sprintf(buf,"g");
		printf("Reading sensor us \n");
		for (i=0;i<5;i++) {
			printf(" %d ",Buffer[i*2] | Buffer[i*2+1]<<8);
		}
		printf("\n");
		ret = 0;
	}
	else
		ret = -2;
	return ret;
}

/** ****************************************************************
 * Store previous sensors values
 * 
 * @return 0 if ok
 * @brief function that store actual for sensor values for later use
***************************************************************** */
int get_sensors_history(void){
	int i;
	for(i=0; i<8;i++){ 
		prev_sensors[i] = sensors[i];
	}
	return 0;
}

/** ****************************************************************
 * Read and store  sensor values
 * 
 * @return 0 if ok
 * @brief function to read and store sensors values
***************************************************************** */
int get_sensors(void){
		char Buffer[256];
		int sensval, i;
		// get ir sensor
		kh4_proximity_ir(Buffer, dsPic);				 
		//limit the sensor values, don't use ground sensors
		for (i = 0; i < 8; i++)	
		{
			sensval = (Buffer[i*2] | Buffer[i*2+1]<<8);
			if(sensval > MAX_DIST)
				sensors[i] = MAX_DIST;
			else if (sensval < MIN_DIST)
				sensors[i] = 0;
			else
				sensors[i] = (sensval-MIN_DIST)>>1;
		}
	return 0;
}

/** ****************************************************************
 * Compute internal deficits
 * 
 * @return 0 when ok
 * @brief function that compute deficits for physiological internal values 
***************************************************************** */
int compute_deficit(void){
	// def_energy = (0.85 - var_energy)>0 ?(0.85 - var_energy) : 0.0 ;
	// def_tegument = (0.85 - var_tegument)>0 ?(0.85 - var_tegument) : 0.0 ;
	// def_integrity = (0.85 - var_integrity)>0 ?(0.85 - var_integrity) : 0.0 ;
	def_energy = 1.0 - var_energy;
	def_tegument = 1.0 - var_tegument;
	def_integrity = 1.0 - var_integrity;
	return 0;
}

/** ****************************************************************
 * Compute motivations
 * 
 * @return 0 when ok
 * @brief function that compute motivations for physiological internal values 
***************************************************************** */
int compute_motivations(void){
	mot_energy = def_energy + (def_energy * cue_energy);
	mot_tegument = def_tegument + (def_tegument * cue_tegument);
	mot_integrity = def_integrity + (def_integrity * cue_integrity);
	return 0;
}

/** ****************************************************************
 * Get mean and normalize a table
 * 
 * @return mean noramlized between 0 and 1
 * @brief function that get mean of table and normalize it
***************************************************************** */
float get_mean_normalized(int table[], int size, int min, int max){
	float mean;
	int i;
	for(i=0; i<size; i++){
		mean += table[i];
	}
	mean /= size;
	return ((mean-min)/(max-min));
}

/** ****************************************************************
 * Get mean and normalize a  float table
 * 
 * @return mean noramlized between 0 and 1
 * @brief function that get mean of table and normalize it but for a float table
***************************************************************** */
float get_mean_normalized_f(float table[], int size, float min, float max){
	float mean;
	int i;
	for(i=0; i<size; i++){
		mean += table[i];
	}
	mean /= size;
	return ((mean-min)/(max-min));
}

/** ****************************************************************
 * Compute cues
 * 
 * @return 0 when ok
 * @brief function that compute cues
***************************************************************** */
int compute_cues(void){
	cue_energy = 0.06;
	cue_tegument = 0.055;
	cue_integrity = get_mean_normalized(sensors,8,MIN_DIST, MAX_DIST);
	return 0;
}

/** ****************************************************************
 * Decrease physoligical variables
 * 
 * @return 0 when ok
 * @brief function that decrease physiological variables 
***************************************************************** */
int decrease_physoligical_variables(void){
	var_energy -= 0.004;
	var_tegument -= 0.0015;
	return 0;
}

/** ****************************************************************
 * TODO Circular damage function
 * 
 * @return 0 when not, 1 when yes
 * @brief function that compute circulare based damage 
***************************************************************** */
int circ_damage(void){
	// TODO debug this function
	int i;
	int diff[7]; // generate an array of n-1 value to compute difference between ith sensor and ith-1 history value 
	float ray = 6; // robot's ray in cm
	for(i=1; i<7; i++){
		diff[i] = (sensors[i]-prev_sensors[i-1]); // compute distance between neighboor sensor history and actual value
		if(abs(diff[i]) < 0.5*sensors[i]){ // if this difference is less than 50% of actual sensor value
			circ_speed[i] = (M_PI*ray)/TIME;
		}
		// printf("s[%d]=%d-ps[%d]=%d => d[%d]=%d ",i, sensors[i], i-1, prev_sensors[i-1], i, diff[i]); 
		// printf(" s[%d]:%.2f | ",i, circ_speed[i]);
	}
	//  Now we're computing if scratching is spreading aroung robot and increasing damage if so
	for(i=1; i<7; i++){
		if( abs(circ_speed[i]-circ_speed[i-1]) < 0.5*circ_speed[i]){
			circ_speed[i-1] *= 2;
			circ_speed[i] *= 2;
		}
	}
	for(i=0;i<7; i++){
		induce_damage(circ_speed[i]); // inducing damage based on speed
	}
	return 0;
}

/** ****************************************************************
 * TODO Speed damage function
 * 
 * @return 0 when not, 1 when yes
 * @brief function that compute speed based damage 
***************************************************************** */
int speed_damage(void){
	int i;
	int diff[8];
	for(i=0; i<8; i++){
		// TODO : FIX ERROR HERE
		diff[i] = (sensors[i]-prev_sensors[i]); // compute distance between previous and current sensor data
		if(abs(diff[i]) > 0.05*(MAX_DIST-MIN_DIST)){ // if distance is greater than 5% of the actual MAX_DIST-MIN_DIST
			speed[i] = (speed[i] + (diff[i]/TIME))/2.0; // speed get mean of it previous value and actual speed
		}
		else
			speed[i] = 0.0;

		// printf("s[%d]=%d-ps[%d]=%d => d[%d]%d ",i, sensors[i], i, prev_sensors[i], i, diff[i]); 
		// printf(" s[%d]:%.2f | ",i, speed[i]);
	}
	printf("\n");
	float mean = get_mean_normalized_f(speed, 8, 0.0, (MAX_DIST/TIME)); // get mean of speed for all sensors
	printf("mean: %f\n", mean);
	// TODO : FIX ERROR HERE
	if(mean > 0.05*(1.0/8.0)){ // if mean speed is superior as 5% of max speed
		for(i=0; i<8; i++){
			if(speed[i]>0.05) // if for ith sensor speed is greater than 5% of max speed
				induce_damage(speed[i]); // induce damage
		}
		return 1; // there is damage
	}
	return 0;
}

/** ****************************************************************
 * check if damage
 * 
 * @return 0 when not, 1 when yes
 * @brief function that check if there is damage based on two types of damage 
***************************************************************** */
int check_if_damage(void){
	if(circ_damage() || speed_damage())
		return 1;
	return 0;
}

/** ****************************************************************
 * Update internal variables
 * 
 * @return 0 when ok
 * @param loopstart, an int set to 1 when it's loop start
 * @brief function that update the internal variables, compute deficits, cues and motivation 
***************************************************************** */
int update_vars(int loopstart){
	if(loopstart){
		decrease_physoligical_variables();
		get_sensors();
		check_if_damage();
	}
	compute_deficit();
	compute_cues();
	compute_motivations();
	return 0;
}

/** ****************************************************************
 * Print internal variables
 * 
 * @return 0 when ok
 * @brief function that print internal variables 
***************************************************************** */
int print_vars(){
    system("clear"); /*clear output screen*/
	printf("************************MODEL UPDATE**************************\n");
	printf("**************************************************************\n");
	printf("energy= %.2f | tegument = %.2f | integrity = %.2f\n", var_energy*100.0, var_tegument*100.0, var_integrity*100.0);
	printf("**************************************************************\n");
	printf("def= %.2f | def = %.2f | def = %.2f\n", def_energy*100.0, def_tegument*100.0, def_integrity*100.0);
	printf("**************************************************************\n");
	printf("cue= %.2f | cue = %.2f | cue = %.2f\n", cue_energy*100.0, cue_tegument*100, cue_integrity*100.0);
	printf("**************************************************************\n");
	printf("mot= %.2f | mot = %.2f | mot = %.2f\n", mot_energy*100.0, mot_tegument*100.0, mot_integrity*100.0);
	printf("**************************************************************\n");

	return 0;
}

/** ****************************************************************
 * Print sensors
 * 
 * @return 0 when ok
 * @brief function that print sensor values 
***************************************************************** */
int print_sensors(void){
	int i;
	printf("************************SENSOR VALUES*************************\n");
	printf("**************************************************************\n");
	printf("\t\t");
	for (i=0; i<8; i++){
		printf(" %d ", sensors[i]);
	}
	printf("\n**************************************************************\n");
	return 0;
}

/** ****************************************************************
 * Print sensor history
 * 
 * @return 0 when ok
 * @brief function that print previous sensor values 
***************************************************************** */
int print_sensors_history(void){
	int i;
	printf("************************HIST VALUES*************************\n");
	printf("**************************************************************\n");
	printf("\t\t");
	for (i=0; i<8; i++){
		printf(" %d ", prev_sensors[i]);
	}
	printf("\n**************************************************************\n");
	return 0;
}

/** ****************************************************************
 * Print sensor history and acutal diff
 * 
 * @return 0 when ok
 * @brief function that print diff between actual and previous sensor data 
***************************************************************** */
int print_sensors_diff(void){
	int i;
	printf("************************DIFF VALUES*************************\n");
	printf("**************************************************************\n");
	printf("\t\t");
	for (i=0; i<8; i++){
		printf(" %d ", sensors[i]-prev_sensors[i]);
	}
	printf("\n**************************************************************\n");
	return 0;
}


/** ****************************************************************
 * Print sensor speed
 * 
 * @return 0 when ok
 * @brief function that print speed table 
***************************************************************** */
int print_sensors_speed(void){
	int i;
	printf("*************************SPEED VALUES*************************\n");
	printf("**************************************************************\n");
	printf("\t");
	for (i=0; i<8; i++){
		printf(" %.2f ", speed[i]);
	}
	printf("\n**************************************************************\n");
	return 0;
}

/** ****************************************************************
 * Print sensor history and acutal diff
 * 
 * @return 0 when ok
 * @brief function that print diff between actual and previous sensor data 
***************************************************************** */
int print_sensors_circ_speed(void){
	int i;
	printf("*********************CIRC SPEED VALUES************************\n");
	printf("**************************************************************\n");
	printf("\t\t");
	for (i=0; i<7; i++){
		printf(" %.2f ", circ_speed[i]);
	}
	printf("\n**************************************************************\n");
	return 0;
}


/** ****************************************************************
 * Clean sensors infos print
 * 
 * @return 0 when ok
 * @brief function that print previous  and actual sensor values 
***************************************************************** */
int print_clean_sensor(void){
	print_sensors_history();
	print_sensors();
	print_sensors_diff();
	print_sensors_speed();
	print_sensors_circ_speed();
	return 0;
}

/** ****************************************************************
 * Eat function
 * 
 * @return 0 when ok
 * @brief function that increase physiological energy variable 
***************************************************************** */
int eat(void){
	var_energy += 0.05;
	return 0;
}

/** ****************************************************************
 * Food seeking behavior
 * 
 * @return 0 when ok
 * @brief function that select send mootor control for food seeking 
***************************************************************** */
int seek_food(void){
	move(0.8,0.8);
	return 0;
}

/** ****************************************************************
 * Energy behavioral group
 * 
 * @return 0 when ok
 * @brief function that select sub-behavioral group for energy 
***************************************************************** */
int energy_behavioral_group(void){
	int can_eat = 0;
	if(can_eat)
		eat();
	seek_food();
	return 0;
}

/** ****************************************************************
 * TODO ADD LEDS
 * Grooming animation
 * 
 * @return 0 when ok
 * @brief function that make a grooming animation 
***************************************************************** */
int groom_animation(void){
	move(-1.0,1.0);
	usleep(2*TIME);
	move(1.0,-1.0);
	usleep(2*TIME);
	return 0;
}

/** ****************************************************************
 * Grooming spot seeking behavior
 * 
 * @return 0 when ok
 * @brief function that select send mootor control for groooming spot seeking 
***************************************************************** */
int seek_grooming_spot(void){
	move(0.8,0.8);
	return 0;
}

/** ****************************************************************
 * Groom function
 * 
 * @return 0 when ok
 * @brief function that increase physiological tegument variable 
***************************************************************** */
int groom(void){
	var_energy += 0.05;
	groom_animation();
	return 0;
}

/** ****************************************************************
 * Tegument behavioral group
 * 
 * @return 0 when ok
 * @brief function that select sub-behavioral group for tegument 
***************************************************************** */
int tegument_behavioral_group(void){
	int can_groom = 0;
	if(can_groom)
		groom();
	seek_grooming_spot();
	return 0;
}

/** ****************************************************************
 * Danger avoidance behavior
 * 
 * @return 0 when ok
 * @brief function that give motor speed avoidance control 
***************************************************************** */
int avoid(void){
	int min = 0, max = 1023;
	// Normalization of sensors
	int i;
	float normalized_sensors[8];
	for(i=0;i<8;i++){
		normalized_sensors[i] = (sensors[i]-min)/(max-min);
	}

	//TODO 
	//Braitenberg avoidance
	float weight_l[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	float weight_r[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

	for(i=0;i<8; i++){
		left_speed += weight_l[i]*normalized_sensors[i];
		right_speed += weight_r[i]*normalized_sensors[i];
	}
	left_speed /= 8;
	right_speed /= 8;

	move(left_speed, right_speed);

	return 0;
}

/** ****************************************************************
 * Integrity behavioral group
 * 
 * @return 0 when ok
 * @brief function that select sub-behavioral group for integrity 
***************************************************************** */
int integrity_behavioral_group(void){
	avoid();
	return 0;
}

/** ****************************************************************
 * Select behavioral group to compute speed
 * 
 * @return 0 when ok
 * @param bhv the selected behavioral group
 * @brief function that select behavioral grroup to compute robot's speed based on input 
 * @note 1 if energy, 2 if tegument, 3 if integrity, -1 if error
***************************************************************** */
int compute_speed(int bhv){
	switch(bhv){
		case 1 : // energy behaviorral group
			energy_behavioral_group();
			break;
		case 2 : // tegument behaviorral group
			tegument_behavioral_group();
			break;
		case 3 : // integrity behaviorral group
			integrity_behavioral_group();
			break;
		case -1 : // Error -> stop robot
		default :
			left_speed = 0.0;
			right_speed = 0.0;
			break;
	}
	return 0;
}

/** ****************************************************************
 * Simple robot model
 * 
 * @return 1 is ok, 0 if error
 * @brief Robot model based on our work
***************************************************************** */
int model(void){
	int iterator = 0;
	get_sensors();
	while((var_energy>0) && (var_tegument>0) && (var_integrity>0)){
		iterator++;
		update_vars(1);
		int behaviral = winner_takes_all(mot_energy,mot_tegument,mot_integrity);
		compute_speed(behaviral);
		move(left_speed,right_speed);

		if(iterator==3){
			print_vars();
			print_clean_sensor();
			iterator=0;
		}
		get_sensors_history();
		usleep(TIME); // wait TIME
	}
	stop_moving();
	death_animation();
	return 0;
}

/** ****************************************************************
 * Main function
 * @brief Main program function
 * 
 * @param argc an int input non used on this function
 * @param argv a string input used to say if you want to run model or keyboard control
 * @return : none
 * @note type run for keyboard control
***************************************************************** */
int main(int argc, char *argv[]){
	// Set the libkhepera debug level - recommended for development.
	kb_set_debug_level(2);

	printf("Running...\n\n");

	// Init the Khepera library
	if(kb_init(argc, argv) < 0)
	{
		printf("ERROR: kb_init error (no privs? try sudo)\n");
		return -1;
	}

	// open K-Net device and store the handle
	dsPic  = knet_open("Khepera4:dsPic", KNET_BUS_I2C, 0, NULL);
	if (dsPic==NULL)
	{
		printf("ERROR: could not initiate comms with Kh4 dsPic\n");
		return -1;
	}

	// mute Ultrasounds
	kh4_activate_us(0,dsPic);

	int r = 0;

	if(strcmp(argv[1],"-r")==0){
		r = display_battery();
		r = run();
	}
	else if(strcmp(argv[1],"-m")==0){
		r = model();
	}
	else
		r = stop_moving();


	knet_close(dsPic);

	return r;
}
