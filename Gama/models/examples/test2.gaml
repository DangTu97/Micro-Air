/***
* Name: test
* Author: dang tu
* Description: 
* Tags: Tag1, Tag2, TagN
***/

model test2
import "../vehicle4.gaml"
import "../global_variables.gaml"

/* Insert your model definition here */
global {
	file shape_file_roads <-  file("../includes/test_road.shp");
	geometry shape <- envelope(shape_file_roads) + 2*ROAD_WIDTH;

	float step <- STEP;
	init {
		create road  from: shape_file_roads with:[is_twoway:read('twoway')]{
			width <- ROAD_WIDTH;
			geom_display <- shape + width;
		}
		
		road_network <- as_edge_graph(road);
		
		create vehicle number:50 {
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
			
			source_node <- one_of(road_network.vertices);
			final_node <- one_of(road_network.vertices);

//			source_node <- {398.9000000000233,810.9811546797864};
//			final_node <- {94.09999999997672,920.0811546798795};

			do compute_shortest_path;
			write shortest_path;
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
//			write angle_between({136.90000000002328,899.5811546798795,0.0}, {135.09999999997672,900.1811546799727,0.0},{126.69999999995343,902.9811546797864,0.0});
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