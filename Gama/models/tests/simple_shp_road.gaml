/***
* Name: simpleshproad
* Author: dang tu
* Description: 
* Tags: Tag1, Tag2, TagN
***/

model simpleshproad
import "../vehicle.gaml"
import "../global_variables.gaml"

/* Insert your model definition here */

global {
	file shape_file_roads <-  file("../includes/segments.shp");
	geometry shape <- envelope(shape_file_roads) + 2*road_width;
	int nb_vehicle <- 10;
	
	init {
		create road from: shape_file_roads {
//			write shape;
			geom_display <- shape + road_width;
			start <- shape.points[0];
			end <-  shape.points[1];
			angle <- angle_between(start, start + {10,0}, end);	
		}
		
		road_network <- as_edge_graph(road);
//		write road_network;
		loop vertex over:road_network.vertices {
			create roadNode {
				shape <- circle(road_width);
				location <- vertex;
			}
		}
		
	}
	
	reflex init_traffic {
		geometry space <- roadNode(0).shape;
		list<vehicle> vehicle_ovelap <- vehicle where (each overlaps space);
		if (length(vehicle_ovelap) = 0) {
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
				
				source_node <- road_network.vertices[0];
				final_node <- road_network.vertices[5];
				do compute_shortest_path;
				location <- any_location_in(space);
				display_polygon <- false;
//				write shortest_path;
				road_belong <- shortest_path[0];
				next_road <- shortest_path[1];
//				write shortest_path;
//				write road_belong;
//				write next_road;
				prob <- 1.0;
			}
		}
	}
}

experiment my_experiment {
	float minimum_cycle_duration <- 0.01;
	output {
		display my_display background: #grey{
			species road aspect: base;
//			species roadNode aspect: base;
			species vehicle aspect: base;
		}
	}
}