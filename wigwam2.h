//
// wigmam2.h  based on wigmam1.h and mugged for RIOT III (non-APM) 
// * 2020-02-25
// *  included to get rudder_driver()
// *  intended for Arduino use
// ** ported to Arduino
// *  change RC_CENTER to RC_STRAIGHT for easy inegration with RIOT III
//

#undef PRINTF
// #define PRINTF printf

int rudder_driver(long time, int heading, int command) {
	/*
		basic idea is that the command sets (LEFT, STRAIGHT (CENTERED), RIGHT) but does not set the
		intensity of the rudder motion.  four factors come into play
		cycle_time:  time in timer units, 1000 is assumed to be 1 sec
		cycle_part1: time as a fraction < 1.0
		right_rudder_units: units the servo moves right at turn
		left_rudder_units:  units the servo moves left at turn

		auto calibration is not (yet) used, so heading is not used
	*/
	static struct { // must be static to maintain state....
		int cycle_time;  // in milliseconds
		float cycle_part1; // fractional part 1
		int right_rudder_units;
		int left_rudder_units;
		int goal_rate;   // degrees / sec
		int calibrate_max_tries;
		int prev_input_command;
		int prev_rc_command;
		long wait_until_time;
		long wait_until_time_done;
		bool calibrated; // true when both calibrated
		bool right_calibrated;
		bool left_calibrated;
		int right_calibrate_tries;
		int left_calibrate_tries;
	}
	//          cycle  fract  right left  goal   max  prev       prev              
	//          time   part1  units units rate tries  in cmd     rc cmd     .....dynamic fields....
	rd_parms = { 1500,  0.6,     5,    5,   6,    5,  RC_STRAIGHT, RC_STRAIGHT, 0, 0, 0, 0, 0, 0, 0 };
  // pre 2020-06-22b:   { 1000,  0.5,  
	// version zero, no calibration
	rd_parms.calibrated = true;
	if (!rd_parms.calibrated) {
		// calibrate here
	}
	int ret = RC_STRAIGHT;
	// note that this code is time dependent and at times just waits/skips...
	if (command != RC_STRAIGHT && time >= rd_parms.wait_until_time_done) {
		// time for a new action
		// commanded to turn right or left
		// hold the needed command until the required degrees in that direction have been completed
		// note that the rudder has *rotated* part and a settling *centered* part (to avoid "death dives")
		//
		// simple for now...
		//
#if 0  
    // Arduino compiler issue ...  
		switch (command) {
		case (RC_LEFT):  ret = RC_LEFT; break;
		case (RC_RIGHT): ret = RC_RIGHT; break;
		}
#else
    // for now:
    if (command == RC_LEFT) { ret = RC_LEFT; }
    else if (command == RC_RIGHT) { ret = RC_RIGHT; }
    // ret = command;
#endif   
#ifdef PRINTF   
		PRINTF("--DEBUG Set %d\n", ret);
#endif    
		rd_parms.wait_until_time = time + (long) (rd_parms.cycle_time * rd_parms.cycle_part1);
		rd_parms.wait_until_time_done = time + rd_parms.cycle_time;
		rd_parms.prev_rc_command = command;
#ifdef PRINTF     
		PRINTF("NOTE timers @ timer: %d (%d %d)\n", time, rd_parms.wait_until_time, rd_parms.wait_until_time_done);
#endif   
	}
	else {
		// no change
		if (time < rd_parms.wait_until_time) {
			if (rd_parms.prev_input_command == command) {
				// no timer pop and no command change
				ret = rd_parms.prev_rc_command;  // keep same action, ret was reset above ....
#ifdef PRINTF          
				PRINTF("--DEBUG No Change!\n");
#endif       
			}
			else {
				// command change (stop waiting!)
				rd_parms.prev_rc_command = command;
				rd_parms.wait_until_time = 0;
				rd_parms.wait_until_time_done = 0;
#ifdef PRINTF          
				PRINTF("--DEBUG Exit Early\n");
#endif       
			}
		}
		else {
			// now just centered while plane settles back level
			// so even tho have RC_LEFT or RC_RIGHT as a command, we request RC_STRAIGHT as the action
			// we are pulsing the turns....
			ret = RC_STRAIGHT;
#ifdef PRINTF        
			PRINTF("-- DEBUG Now Center\n");
#endif      
		}
	}
	rd_parms.prev_input_command = command;  // note these two are similar
	rd_parms.prev_rc_command = ret;         // but this one could change as code is imporved.
	// now scale the return!
	if (ret == RC_LEFT) {
		ret *= rd_parms.left_rudder_units;
  } else if (ret == RC_RIGHT) { 
		ret *= rd_parms.left_rudder_units;
	} // RC_STRAIGHT falls thru...
  // scale factor DEBUG WIGMAN
  ret *= 4;  // so 5 to 20...., 0 to 0, -5 to -20.....  
	return ret;
}

// end of wigwam1.h
