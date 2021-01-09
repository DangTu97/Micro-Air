/**
* Name: emission
* Based on the internal empty template. 
* Author: dongh
* Tags: 
*/


model emission
import "../global_variables.gaml"
import "vehicle.gaml"
/* Insert your model definition here */

species Emis skills: [moving]{
	string type <- 'CO'; // can add NOx, SO2 and PTM
	map<string, rgb> col <- ['CO'::#red, 'NOx'::#blue, 'SO2'::#orange, 'PTM'::#gray];
	vehicle root;
	float power <- 1.0;
	float diff_speed <- 5#m/#s;
	float starting_heading;
	float starting_speed <- 3#m/#s;
	float back_coeff <- 1.0;
	
	reflex move{
		// moving by affect of engine
		do move heading: starting_heading speed: starting_speed * back_coeff * step;
		// moving by diffusion
		do wander amplitude: 360.0 speed: diff_speed * step;
		// moving by affect of wind
		do move heading: wind_dir speed: wind_speed * step;
	}
	
	reflex dec_power{
		power <- max(0.0, power - step * de_power_coeff);
		back_coeff <- max(0.0, back_coeff - step * 0.25);
	}
	
	reflex disappear when: power <= 0{
		do die;
	}
	
	aspect default{
		draw circle(0.2#m) color: col[self.type];
	}	
}

species Sensor{
	rgb col <- #white;
	int CO_dens;
	int NOx_dens;
	int SO2_dens;
	int PTM_dens;
	
	reflex sensing{
		CO_dens <- length(Emis where (each.type = 'CO' and each.location distance_to self.location using topology(world) < 5#m));
		NOx_dens <- length(Emis where (each.type = 'NOx' and each.location distance_to self.location using topology(world) < 5#m));
		SO2_dens <- length(Emis where (each.type = 'SO2' and each.location distance_to self.location using topology(world) < 5#m));
		PTM_dens <- length(Emis where (each.type = 'PTM' and each.location distance_to self.location using topology(world) < 5#m));
	}
	
	aspect default{
		draw circle(1#m) color: col;
	}
}