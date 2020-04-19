/***
* Name: simpleroad
* Author: dang tu
* Description: 
* Tags: Tag1, Tag2, TagN
***/

model simpleroad
import "../vehicle.gaml"

/* Insert your model definition here */

global {
	geometry shape <- square(500);
	
	init {
		
		list<list<point>> pair_of_nodes <- [[{10,50} , {100,50}], [{100,50} , {200,50}], [{200,50} , {300,100}],
							[{300,44} , {200,44}], [{200,44} , {100,44}], [{100,44}, {10,10}]];
							
		loop i from:0 to:length(pair_of_nodes) - 1 {
			create road {
				shape <- polyline(pair_of_nodes[i]);
				geom_display <- shape + road_width;
				start <- shape.points[0];
				end <-  shape.points[1];
				angle <- angle_between(start, start + {10,0}, end);	
			}
		}
		
		road_network <- as_edge_graph(road);
		write road_network;
		
		loop v over: road_network.vertices {
			create space {
				location <- v;
				shape <- circle(road_width);
			}
		}
//		road(1).linked_road <- road(4);
//		road(4).linked_road <- road(1);
	}
	
	reflex init_traffic {
		list<vehicle> vehicle_overlap_space0 <- vehicle where (each overlaps space(0));
		if (length(vehicle_overlap_space0) = 0) {
			create vehicle number: 5 {
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
				final_node <- road_network.vertices[3];
			
				display_polygon <- false;
				do compute_shortest_path;
				road_belong <- shortest_path[0];
				next_road <- shortest_path[1];
				location <- any_location_in(space(0).shape);
				prob <- 1.0;		
			}
		}
		
		list<vehicle> vehicle_overlap_space4 <- vehicle where (each overlaps space(4));
		if (length(vehicle_overlap_space4) = 0) {
			create vehicle number: 5 {
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
				
				source_node <- road_network.vertices[4];
				final_node <- road_network.vertices[7];
			
				display_polygon <- false;
				do compute_shortest_path;
				road_belong <- shortest_path[0];
				next_road <- shortest_path[1];
				location <- any_location_in(space(4).shape);
				prob <- 1.0;		
			}
		}
		
	}
}

species space {
	aspect default {
		draw shape color:#white;
	}
}

experiment my_experiment {
	float minimum_cycle_duration <- 0.01;
	output {
		display my_display background: #grey{
			species road aspect: base;
			species space;
			species vehicle aspect: base;
//			species roadNode aspect: base;
		}
	}
}