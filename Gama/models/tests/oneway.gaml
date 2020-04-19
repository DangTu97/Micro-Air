/***
* Name: oneway
* Author: dang tu
* Description: traffic simulation on one-way road
* Tags: Tag1, Tag2, TagN
***/

model oneway
import "../vehicle.gaml"

/* Insert your model definition here */

global {   
	graph road_network <- graph([]);
	int nb_vehicle <- 10;
	float road_width <- 3.0 #m;
	geometry shape <- square(environment_size);
	
	init {
		list<point> nodes <- [{10,10}, {170,10}];
//		list<point> nodes <- [{10,10}, {160,160}];
		create road {
			shape <- polyline(reverse(nodes[0], nodes[1]));
			geom_display <- (shape + road_width);
			angle <- angle_between(nodes[0], nodes[0] + {10,0}, nodes[1]);
			start <- nodes[0];
			end <- nodes[1];
		}
		
		road_network <- as_edge_graph(road);
		
		loop vertex over:road_network.vertices {
			create roadNode {
				shape <- circle(road_width);
				location <- vertex;
			}
		}
//		write road_network;
//		write road_network.vertices;
	}
	
	reflex init_traffic {
		geometry space <- roadNode(1).shape;
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
				final_node <- road_network.vertices[1];
				do compute_shortest_path;
				location <- any_location_in(space);
				display_polygon <- false;
//				write shortest_path;
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
			species vehicle aspect: base;
			species roadNode aspect: base;
		}
	}
}