/***
* Name: traffic light
* Author: dang tu
* Description: simple traffic light
* Tags: Tag1, Tag2, TagN
***/

model trafficlight
import "../vehicle.gaml"

/* Insert your model definition here */

global {   
	graph road_network <- graph([]);
	int nb_vehicle <- 10;
	float road_width <- 3.0 #m;
	geometry shape <- square(environment_size);
	
	init {
		list<point> nodes <- [{10,10}, {100,10}, {160,50}];
		
		loop node over:nodes {
			road_network <- road_network add_node node.location;
		}
		road_network <- road_network add_edge (nodes at 0::nodes at 1);
		road_network <- road_network add_edge (nodes at 1::nodes at 2);
		
		loop vertex over:nodes {
			create roadNode {
				location <- vertex;
				shape <- circle(road_width+0.1);
				if (location = {100,10}) {
					is_traffic_signal <- true;
				}
			}
		}
		
		roadNode(1).roads_in <- [road(0)];
		roadNode(1).stop  <- [road(0)];
		
		loop i from: 0 to: length(nodes) - 2 {
			create road {
				shape <- polyline(reverse(nodes[i], nodes[i+1]));
				geom_display <- (shape + road_width);
				angle <- angle_between(nodes[i], nodes[i] + {10,0}, nodes[i+1]);
				start <- nodes[i];
				end <- nodes[i+1];
			}
		}
	}
	
	reflex init_traffic {
		geometry space <- roadNode(0).shape;
		list<vehicle> vehicle_ovelap <- vehicle where (each overlaps space);
		if (length(vehicle_ovelap) = 0) {
			create vehicle number: nb_vehicle {
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
				final_node <- road_network.vertices[2];
				do compute_shortest_path;
				location <- any_location_in(space);
				display_polygon <- false;
				
				road_belong <- road(0);
				prob <- 1.0;
			}
		}
	}
	
}

experiment my_experiment {
	float minimum_cycle_duration <- 0.05;
	output {
		display my_display {
			species road aspect: base;
			species roadNode aspect: base;
			species vehicle aspect: base;
		}
	}
}