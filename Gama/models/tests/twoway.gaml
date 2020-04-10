/***
* Name: twoway2
* Author: dang tu
* Description: traffic simulation of two-way road
* Tags: Tag1, Tag2, TagN
***/

model twoway

import "../vehicle.gaml"

/* Insert your model definition here */

global {   
	graph road_network <- graph([]);
	int nb_vehicle <- 3;
	float road_width <- 3.0 #m;
	geometry shape <- square(environment_size);
	
	init {
		list<point> nodes <- [{10,10}, {160,10}, {160,4}, {10,4}];
		loop node over:nodes {
			road_network <- road_network add_node node.location;
		}
		road_network <- road_network add_edge (nodes at 0::nodes at 1);
		road_network <- road_network add_edge (nodes at 2::nodes at 3);
		
		loop vertex over:nodes {
			create roadNode {
				location <- vertex;
				shape <- circle(road_width);
			}
		}
		
		create road {
			shape <- polyline(reverse(nodes[0], nodes[1]));
			geom_display <- (shape + 3);
			angle <- angle_between(nodes[0], nodes[0] + {10,0}, nodes[1]);
			start <- nodes[0];
			end <- nodes[1];
		}
		
		create road {
			shape <- polyline(reverse(nodes[2], nodes[3]));
			geom_display <- (shape + 3);
			angle <- angle_between(nodes[2], nodes[2] + {10,0}, nodes[3]);
			start <- nodes[2];
			end <- nodes[3];
		}
		
		road(0).linked_road <- road(1);
		road(1).linked_road <- road(0);
//		write road(1).angle;
//		write road(0).angle;
		
	}
	
	reflex init_traffic {
		geometry space_left <- polygon([{10,7}, {16,7}, {16,13}, {10,13}]);
		geometry space_right <- polygon([{160,1}, {160,7}, {154,7}, {154,4}]);
		
		list<vehicle> vehicle_ovelap_spaceleft <- vehicle where (each overlaps space_left);
		if (length(vehicle_ovelap_spaceleft) = 0) {
			create vehicle number: 8 {
				name <- flip(0.2) ? 'car' : 'motorbike';
				if name = 'car' {
					length <- 3.8 #m;
					width <- 1.5 #m;
					df <- 0.25 #m;
					db <- 0.15 #m;
					dx <- width/2 + db;
					dy <- length/2 + df;
					speed <- 0.1;
					max_speed <- rnd(0.4, 1.0) #m/#s;
				} else {
					length <- 1.8 #m;
					width <- 0.7 #m;
					df <- 0.15 #m;
					db <- 0.1 #m;
					dx <- width/2 + db;
					dy <- length/2 + df;
					speed <- 0.1;
					max_speed <- rnd(0.2, 0.7) #m/#s;
				}
				
				source_node <- road_network.vertices[0];
				final_node <- road_network.vertices[1];
				do compute_shortest_path;
				next_node <- shortest_path[1];
				current_node <- source_node;
				location <- any_location_in(space_left);
				display_polygon <- false;
				
				road_belong <- road(0);
				prob <- 1.0;
			}
		}
		
		list<vehicle> vehicle_ovelap_spaceright <- vehicle where (each overlaps space_right);
//		if (length(vehicle_ovelap_spaceright) = 0) {
		if ((length(vehicle_ovelap_spaceright) = 0) and (mod(cycle, 30) = 0)) {
			create vehicle number: 1 {
				name <- flip(0.2) ? 'car' : 'motorbike';
				if name = 'car' {
					length <- 3.8 #m;
					width <- 1.5 #m;
					df <- 0.25 #m;
					db <- 0.15 #m;
					dx <- width/2 + db;
					dy <- length/2 + df;
					speed <- 0.1;
					max_speed <- rnd(0.4, 1.0) #m/#s;
				} else {
					length <- 1.8 #m;
					width <- 0.7 #m;
					df <- 0.15 #m;
					db <- 0.1 #m;
					dx <- width/2 + db;
					dy <- length/2 + df;
					speed <- 0.1;
					max_speed <- rnd(0.2, 0.7) #m/#s;
				}
				
				source_node <- road_network.vertices[2];
				final_node <- road_network.vertices[3];
				do compute_shortest_path;
				next_node <- shortest_path[1];
				current_node <- source_node;
				location <- any_location_in(space_right);
				display_polygon <- false;
				
				road_belong <- road(1);
				prob <- 0.0;
			}
		}
		
	}
	
}

experiment my_experiment {
	float minimum_cycle_duration <- 0.05;
	output {
		display my_display {
			species road aspect: base;
			species vehicle aspect: base;
			species roadNode aspect: base;
		}
	}
}