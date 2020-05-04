/***
* Name: test2way
* Author: dang tu
* Description: 
* Tags: Tag1, Tag2, TagN
***/

model test2way
import "../vehicle.gaml"
import "../global_variables.gaml"

/* Insert your model definition here */

global {
	geometry shape <- square(500);
	init {
		create road {
			shape <- polyline([{10,10}, {100,10}, {200, 50}]);
			is_twoway <- true;
			geom_display <- shape + road_width;
		}
		
		create road {
			shape <- polyline([{200, 50}, {300, 50}, {400,10}]);
			is_twoway <- true;
			geom_display <- shape + road_width;
		}
		
		list<point> nodes <- [{10,10}, {100,10}, {200, 50}, {300, 50}, {400,10}];
		loop node over:nodes {
			create space {
				location <- node;
				shape <- circle(road_width);
			}
		}
		
		road_network <- as_edge_graph(road);
		write road_network;
	}
	
	reflex init_traffic when: mod(cycle, 150) = 0 {
		create vehicle number: 10 {
			name <- flip(0.3) ? 'car' : 'motorbike';
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
			final_node <- road_network.vertices[0];
			do compute_shortest_path;
//			write shortest_path;
			road_belong <-  shortest_path[0];
			do compute_road_belong_nodes;
			display_polygon <- false;
			prob <- 0.0;
			location <- any_location_in(source_node + {rnd(3.0), - rnd(5.0)});
			start_node <- reverse(road_belong.shape.points)[0];
			target_node <- reverse(road_belong.shape.points)[1];
			angle <- angle_between(start_node, start_node + {10,0}, target_node);
			do compute_road_belong_nodes;
			do update_polygon;
			target_space <- polyline([target_node - {1.5*road_width, 0}, target_node + {1.5*road_width, 0}]) rotated_by (angle + 90);
		}
		
		create vehicle number: 0 {
			name <- flip(0.3) ? 'car' : 'motorbike';
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
			road_belong <-  shortest_path[0];	
			display_polygon <- false;
			prob <- 1.0;
			location <- any_location_in(source_node + {rnd(3.0), rnd(5.0)});
			start_node <- road_belong.shape.points[0];
			target_node <- road_belong.shape.points[1];
			angle <- angle_between(start_node, start_node + {10,0}, target_node);
			do compute_road_belong_nodes;
			do update_polygon;
			target_space <- polyline([target_node - {1.5*road_width, 0}, target_node + {1.5*road_width, 0}]) rotated_by (angle + 90);
		}
	}
}

experiment my_experiment {
	float minimum_cycle_duration <- 0.01;
	output {
		display my_display background: #grey{
			species road aspect: base;
			species space;
			species vehicle aspect: base;
		}
	}
}