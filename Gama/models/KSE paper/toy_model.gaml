/***
* Name: toymodel
* Author: dang tu
* Description: simulation on two-way road
* Tags: Tag1, Tag2, TagN
***/

model toymodel

/* Insert your model definition here */
import "vehicle.gaml"
import "global_variables.gaml"

global {
	list<point> nodes <- [{10, 50}, {210, 50}];
	graph road_network <- graph([]);
	geometry shape <- square(EVIRONMENT_SIZE);
	float step <- STEP;
	int unit <- 20;
	int traffic_volume_bottom <- 3; 
	int traffic_volume_top <- 5; 
	float PROB_GO_OPPOSITE <- 1.0;
	
	int nb_vehicle;
	float vehicle_average_speed <- 0.0;
	float vehicle_timer <- 0.0;
	int vehicle_counter <- 0;
	
	int nb_top; // number of vehicle at the top lane but move on bottom lane
	int nb_bottom; // number of vehicle at the bottom lane but move on top lane
	
	init {
		create road {
			shape <- polyline(nodes);
			road_width <- ROAD_WIDTH;
			geom_display <- shape + road_width;
			is_twoway <- true;
		}
		road_network <- as_edge_graph(road);
	}
	
	reflex init_traffic when: mod(cycle, unit) = 0 {
		create vehicle number: traffic_volume_bottom {
			int i <- rnd(1, 100);
			type <- (i < 2) ? 'BUS' : ( i < 20 ? 'CAR' : 'MOTORBIKE');
			
			if type = 'CAR' {	
				max_speed <- rnd(CAR_MAXSPEED/2, CAR_MAXSPEED);
			} else if type = 'MOTORBIKE' {
				max_speed <- rnd(MOTORBIKE_MAXSPEED/2, MOTORBIKE_MAXSPEED);
			} else {
				max_speed <- BUS_MAXSPEED;
			}

			
			source_node <- nodes[0];
			final_node <-  nodes[1];
			on_right_side <- true;
			do set_type;
			
			// location
			point p1 <- start_node;
			float a <- (target_node - start_node).x;
			float b <- (target_node - start_node).y;
			float d <- distance_to(target_node, start_node);
			point p2 <- p1 + {ROAD_WIDTH*a/d, ROAD_WIDTH*b/d};
			point center <- (p1 + p2)/2;
			float D <- 0.5*ROAD_WIDTH;
			point free_space_center <- center + {b*D/sqrt(a*a + b*b), - a*D/sqrt(a*a + b*b)};
			if angle_between(start_node, target_node, free_space_center) > 180 {
				free_space_center <- center + { -b*D/sqrt(a*a + b*b), a*D/sqrt(a*a + b*b)};	
			}	
			point p3 <- free_space_center*2 - p1;
			point p4 <- free_space_center*2 - p2;
			free_space <- polygon([p1,p2,p3,p4, p1]);
			location <- any_location_in(free_space);
		}
			
		create vehicle number: traffic_volume_top {
			int i <- rnd(1, 100);
			type <- (i < 2) ? 'BUS' : ( i < 20 ? 'CAR' : 'MOTORBIKE');
			
			if type = 'CAR' {	
				max_speed <- rnd(CAR_MAXSPEED/2, CAR_MAXSPEED);
			} else if type = 'MOTORBIKE' {
				max_speed <- rnd(MOTORBIKE_MAXSPEED/2, MOTORBIKE_MAXSPEED);
			} else {
				max_speed <- BUS_MAXSPEED;
			}
	
			source_node <- nodes[1];
			final_node <-  nodes[0];
			on_right_side <- true;
			do set_type;
			
			// location
			point p1 <- start_node;
			float a <- (target_node - start_node).x;
			float b <- (target_node - start_node).y;
			float d <- distance_to(target_node, start_node);
			point p2 <- p1 + {ROAD_WIDTH*a/d, ROAD_WIDTH*b/d};
			
			point center <- (p1 + p2)/2;
			float D <- 0.5*ROAD_WIDTH;
			point free_space_center <- center + {b*D/sqrt(a*a + b*b), - a*D/sqrt(a*a + b*b)};
			if angle_between(start_node, target_node, free_space_center) > 180 {
				free_space_center <- center + { -b*D/sqrt(a*a + b*b), a*D/sqrt(a*a + b*b)};	
			}	
			point p3 <- free_space_center*2 - p1;
			point p4 <- free_space_center*2 - p2;
			free_space <- polygon([p1,p2,p3,p4, p1]);
			location <- any_location_in(free_space);
			do update_polygon;
		}
	}
	
	reflex count_vehicles {
		nb_top <- length(vehicle where (each.source_node = nodes[1] and each.on_right_side = false));
		nb_bottom <- length(vehicle where (each.source_node = nodes[0] and each.on_right_side = false));
		if (nb_bottom + nb_top >= 4) and (abs(nb_bottom - nb_top) >= 3) and cycle > 500  and min(nb_bottom, nb_top) > 0{
			do pause;
		}
	}
}

species my_species {
	aspect default {
		draw shape color:#red;
	}
}

experiment my_experiment {
	float minimum_cycle_duration <- TIME_STEP;
	output {
		display my_display background: #grey{
			species road aspect: base;
			species vehicle aspect: base;
		}
		monitor "Upper lane flow" value: traffic_volume_top;
		monitor "Lower lane flow" value: traffic_volume_bottom;
		monitor "Number of wrong vehicles on upper lane" value: nb_top;
		monitor "Number of wrong vehicles on lower lane" value: nb_bottom;
	}
}