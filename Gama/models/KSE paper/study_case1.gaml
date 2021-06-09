/***
* Name: studycase
* Author: dang tu
* Description: traffic simulation for Giai Phong road at 7:30 am, the road is 200 meters long and 10.5 meters wide.
* Tags: Tag1, Tag2, TagN
***/

model studycase1
import "vehicle.gaml"
import "global_variables.gaml"

/* Insert your model definition here */
global {
	list<point> nodes <- [{10, 150}, {210, 150}];
	graph road_network <- graph([]);
	geometry shape <- square(EVIRONMENT_SIZE);
	float step <- STEP;
	float road_width <- 5.25 #m;
	int unit <- 20;
	
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
		
		//for bus, car
		create init_space {
			location <- {10, 146.5};
			geom <- rectangle(5, 2);
		}
		
		//for car or motorbike
		create init_space {
			location <- {10, 148.5};
			geom <- rectangle(5, 2);
		}
		
		create init_space {
			location <- {10, 150};
			geom <- rectangle(1, 1);
		}
		create init_space {
			location <- {10, 151};
			geom <- rectangle(1, 1);
		}
		create init_space {
			location <- {10, 152};
			geom <- rectangle(1, 1);
		}
		create init_space {
			location <- {10, 153};
			geom <- rectangle(1, 1);
		}
		create init_space {
			location <- {10, 154};
			geom <- rectangle(1, 1);
		}
	}
	
	reflex init_traffic when: mod(cycle, 20) = 0{	
		loop i from:0 to:6 {
			list<agent> var <- agents_overlapping(init_space(i).geom);
			write '----';
			write i;
			write var;
			if (length(var) <= 1 and i != 2) or (i = 2 and length(var) <= 2) {
				create vehicle number: 1 {
					if i = 0 { type <- flip(0.02) ? 'BUS' : 'CAR';}  
					else if i = 1 { type <- flip(0.5) ? 'CAR' : 'MOTORBIKE';}
					else {type <- 'MOTORBIKE';}
					
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
					location <- init_space(i).location + {0.15 - rnd(0.3), 0.15 - rnd(0.3)};
					do set_type;
				}
			}
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