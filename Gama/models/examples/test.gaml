/***
* Name: test
* Author: dang tu
* Description: 
* Tags: Tag1, Tag2, TagN
***/

model test
import "../vehicle3.gaml"
import "../global_variables.gaml"

/* Insert your model definition here */
global {
//	list<point> road_nodes <- [{10,10}, {100, 10}, {50, 50}, {120, 50}];
//	geometry shape <- square(EVIRONMENT_SIZE);
	file shape_file_roads <-  file("../includes/multilines.shp");
	geometry shape <- envelope(shape_file_roads) + 2*ROAD_WIDTH;

	float step <- STEP;
	init {
		// from: shape_file_roads
		create road from: shape_file_roads{
//			shape <- polyline(road_nodes);
			width <- ROAD_WIDTH;
			geom_display <- shape + width;
			is_twoway <- true;
		}
		
		road_network <- as_edge_graph(road);
		
		create vehicle number: 0 {
			type <- 'CAR';
			length <- CAR_LENGTH;
			width <- CAR_WIDTH;
			df <- CAR_DF;
			db <- CAR_DB;
			dx <- width/2 + db;
			dy <- length/2 + df;
						
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
			
			display_polygon <- true;
			source_node <- road_network.vertices[0];
			final_node <- road_network.vertices[1];
			do compute_shortest_path;
			if length(shortest_path) = 0 { do die; }
			road_belong <-  shortest_path[0];
			start_node <- source_node;
			do compute_road_belong_nodes;
			target_node <- road_belong_nodes[1];
			angle <- angle_between(start_node, start_node + {10,0}, target_node);
			
			location <- start_node;
			do update_polygon;
			
			do get_future_node;
			do get_transfer_geom;
			is_transferred <- false;
		}
	}
	
	reflex init_traffic when:mod(cycle,10)=0 {
		create vehicle number:1 {
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
			max_speed <- rnd(7,14) #m/#s;
			polygon_width_size <- WIDTH_SIZE;
			minimun_length_size <- MINIMUM_LENGTH_SIZE;
			distance_check <- DISTANCE_CHECHK;
			acceleration_factor <- ACCELERATION_FACTOR;
			deceleration_factor <- DECELERATION_FACTOR;
			speed <- INIT_SPEED;
			prob_go_opposite <- PROB_GO_OPPOSITE;
			prob_turn_right <- PROB_TURN_RIGHT;
			
			display_polygon <- false;
			source_node <- road_network.vertices[1];
			final_node <- road_network.vertices[0];
			do compute_shortest_path;
			if length(shortest_path) = 0 { do die; }
			road_belong <-  shortest_path[0];
			start_node <- source_node;
			do compute_road_belong_nodes;
			target_node <- road_belong_nodes[1];
			angle <- angle_between(start_node, start_node + {10,0}, target_node);
			
			location <- start_node + {0,rnd(7)};
			do update_polygon;
			
			do get_future_node;
			do get_transfer_geom;
			is_transferred <- false;
		}
		
		create vehicle number:1 {
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
			max_speed <- rnd(7,14) #m/#s;
			polygon_width_size <- WIDTH_SIZE;
			minimun_length_size <- MINIMUM_LENGTH_SIZE;
			distance_check <- DISTANCE_CHECHK;
			acceleration_factor <- ACCELERATION_FACTOR;
			deceleration_factor <- DECELERATION_FACTOR;
			speed <- INIT_SPEED;
			prob_go_opposite <- PROB_GO_OPPOSITE;
			prob_turn_right <- PROB_TURN_RIGHT;
			
			display_polygon <- false;
			source_node <- road_network.vertices[0];
			final_node <- road_network.vertices[1];
			do compute_shortest_path;
			if length(shortest_path) = 0 { do die; }
			road_belong <-  shortest_path[0];
			start_node <- source_node;
			do compute_road_belong_nodes;
			target_node <- road_belong_nodes[1];
			angle <- angle_between(start_node, start_node + {10,0}, target_node);
			
			location <- start_node + {0,rnd(1,7)};
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