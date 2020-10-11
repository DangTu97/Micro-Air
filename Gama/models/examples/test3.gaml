/***
* Name: test3
* Author: dang tu
* Description: 
* Tags: Tag1, Tag2, TagN
***/

model test3
import "../vehicle4.gaml"
import "../global_variables.gaml"
/* Insert your model definition here */

global {
	file shape_file_roads <-  file("../includes/road_circle.shp");
	geometry shape <- envelope(shape_file_roads) + 20;
	float step <- STEP;
	init {
		create road from: shape_file_roads with:[is_twoway:: read("type")]{
			width <- ROAD_WIDTH;
			geom_display <- shape + width;
		}
		road_network <- as_edge_graph(road);
	}
	
	reflex init_traffic when:mod(cycle,5)=0 {
		create vehicle number: 1 {
			type <- flip(0.3) ? 'CAR' : 'MOTORBIKE';
				if type = 'CAR' {
				length <- CAR_LENGTH;
				width <- CAR_WIDTH;
				df <- CAR_DF;
				db <- CAR_DB;
				dx <- width/2 + db;
				dy <- length/2 + df;
			} else if type = 'MOTORBIKE' {
				length <- MOTORBIKE_LENGTH;
				width <- MOTORBIKE_WIDTH;
				df <- MOTORBIKE_DF;
				db <- MOTORBIKE_DB;
				dx <- width/2 + db;
				dy <- length/2 + df;	
			} 
						
			speed <- INIT_SPEED;
			max_speed <- 14 #m/#s;
			polygon_width_size <- WIDTH_SIZE;
			minimun_length_size <- MINIMUM_LENGTH_SIZE;
			distance_check <- DISTANCE_CHECHK;
			acceleration_factor <- ACCELERATION_FACTOR;
			deceleration_factor <- DECELERATION_FACTOR;
			speed <- INIT_SPEED;
			prob_go_opposite <- PROB_GO_OPPOSITE;
			prob_turn_right <- PROB_TURN_RIGHT;
			display_polygon <- false;
			source_node <- {20,60};
			final_node <- {140,20};
			
			do compute_shortest_path;
			shortest_path <- [0, 2, 3, 1];
			if length(shortest_path) = 0 { do die; }
			road_belong <-  shortest_path[0];
			start_node <- source_node;
			do compute_road_belong_nodes;
			target_node <- road_belong_nodes[1];
			angle <- angle_between(start_node, start_node + {10,0}, target_node);
			
			location <- start_node + {0, 1.0 + rnd(6.0)};
			do update_polygon;
			do get_future_node;
			do get_transfer_geom;
			is_transferred <- false;
		}
		
		create vehicle number: 1 {
			type <- flip(0.3) ? 'CAR' : 'MOTORBIKE';
				if type = 'CAR' {
				length <- CAR_LENGTH;
				width <- CAR_WIDTH;
				df <- CAR_DF;
				db <- CAR_DB;
				dx <- width/2 + db;
				dy <- length/2 + df;
			} else if type = 'MOTORBIKE' {
				length <- MOTORBIKE_LENGTH;
				width <- MOTORBIKE_WIDTH;
				df <- MOTORBIKE_DF;
				db <- MOTORBIKE_DB;
				dx <- width/2 + db;
				dy <- length/2 + df;	
			} 
						
			speed <- INIT_SPEED;
			max_speed <- 14 #m/#s;
			polygon_width_size <- WIDTH_SIZE;
			minimun_length_size <- MINIMUM_LENGTH_SIZE;
			distance_check <- DISTANCE_CHECHK;
			acceleration_factor <- ACCELERATION_FACTOR;
			deceleration_factor <- DECELERATION_FACTOR;
			speed <- INIT_SPEED;
			prob_go_opposite <- PROB_GO_OPPOSITE;
			prob_turn_right <- PROB_TURN_RIGHT;
			display_polygon <- false;
			source_node <- {140,20};
			final_node <- {20,60};
			
			do compute_shortest_path;
			shortest_path <- [1, 5, 4, 0];
			if length(shortest_path) = 0 { do die; }
			road_belong <-  shortest_path[0];
			start_node <- source_node;
			do compute_road_belong_nodes;
			target_node <- road_belong_nodes[1];
			angle <- angle_between(start_node, start_node + {10,0}, target_node);
			
			location <- start_node + {0, - 1.0 - rnd(6.0)};
			do update_polygon;
			do get_future_node;
			do get_transfer_geom;
			is_transferred <- false;
		}
	}
}

experiment my_experiment {
	float minimum_cycle_duration <- TIME_STEP;
	output {
		display my_display background:#grey{
			species road aspect:base;
			species vehicle aspect:base;
		}
	}
}