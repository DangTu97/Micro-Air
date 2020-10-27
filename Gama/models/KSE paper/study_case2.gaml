/***
* Name: studycase2
* Author: dang tu
* Description: traffic simulation for Giai Phong road at 10:30 am, the road is 200 meters long and 10.5 meters wide.
* Tags: Tag1, Tag2, TagN
***/

model studycase2
import "vehicle.gaml"
import "../global_variables.gaml"

/* Insert your model definition here */

global {
	list<point> nodes <- [{10, 150}, {210, 150}];
	graph road_network <- graph([]);
	geometry shape <- square(EVIRONMENT_SIZE);
	float step <- STEP;
	float road_width <- 5.25 #m;
	int unit <- 8;
	
	int nb_vehicle;
	float vehicle_average_speed <- 0.0;
	float vehicle_timer <- 0.0;
	int vehicle_counter <- 0;
	string model_name <- 'study_case';
	
	init {
		create road {
			shape <- polyline(nodes);
			road_width <- 5.25 #m;
			geom_display <- shape + road_width;
			is_twoway <- false;
		}
		
		road_network <- as_edge_graph(road);
		
		road_width <- 5.25 #m;
		//free space for cars
		create init_space {
			geom <- polygon([nodes[0], nodes[0] + {road_width, 0}, nodes[0] + {road_width,-road_width}, nodes[0] + {0,-road_width}]);
		}
		//free space for motorbikes
		create init_space {
			geom <- polygon([nodes[0], nodes[0] + {road_width, 0}, nodes[0] + {road_width,road_width}, nodes[0] + {0,road_width}]);
		}
	}
	
	reflex init_traffic when: mod(cycle, 8) = 0 {
		create vehicle number: 1 {
			int k <- rnd(100);
			if k <= 1 { type <- 'BUS';} 
			else if (k > 2) and (k <= 28) { type <- 'CAR';}
			else { type <- 'MOTORBIKE';}
			
			timer <- 0;
			gender <- flip(0.5) ? 'MALE' : 'FEMALE';
			string my_string <- type + '_SAFESPEED_' + gender;
			switch my_string {
				match 'CAR_SAFESPEED_MALE' {max_speed <- flip(PROB_MAXSPEED_SAFESPEED) ? CAR_SAFESPEED_MALE : CAR_MAXSPEED;}
				match 'CAR_SAFESPEED_FEMALE' {max_speed <- flip(PROB_MAXSPEED_SAFESPEED) ? CAR_SAFESPEED_FEMALE : CAR_MAXSPEED;}
				match 'MOTORBIKE_SAFESPEED_MALE' {max_speed <- flip(PROB_MAXSPEED_SAFESPEED) ? MOTORBIKE_SAFESPEED_MALE : MOTORBIKE_MAXSPEED;}
				match 'MOTORBIKE_SAFESPEED_FEMALE' {max_speed <- flip(PROB_MAXSPEED_SAFESPEED) ? MOTORBIKE_SAFESPEED_FEMALE : MOTORBIKE_MAXSPEED;}
			}
			
			if type = 'BUS' {
				max_speed <- BUS_MAXSPEED;
			}
			source_node <- road_network.vertices[0];
			final_node <- road_network.vertices[1];
			
			if type = 'MOTORBIKE' {
				location <- any_location_in(init_space(1).geom);
			} else {
				location <- any_location_in(init_space(0).geom);
			}
			do set_type;
		}	
	}
	
	reflex update_info when:(mod(cycle,5*unit)=0) and (cycle != 0) {
		nb_vehicle <- length(vehicle);
	}
}

experiment my_experiment {
	float minimum_cycle_duration <- TIME_STEP;
	output {
		display my_display background: #grey{
			species road aspect: base;
			species vehicle aspect: base;
		}
		
		display my_chart refresh:every(200#cycle) {
			chart "Number of vehicles" position: {0, 0} size: {1.0,0.5} {
				data "vehicle" value:nb_vehicle color:#red;
			}
			chart "Average speed" position: {0, 0.5} size: {1.0,0.5}  {
				data "Speed" value:vehicle_average_speed color:#green;
			}
		}
	}
}