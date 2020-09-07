/***
* Name: test
* Author: dang tu
* Description: 
* Tags: Tag1, Tag2, TagN
***/

model test2
import "../vehicle3.gaml"
import "../global_variables.gaml"

/* Insert your model definition here */
global {
//	list<point> road_nodes <- [{10,10}, {100, 10}, {50, 50}, {120, 50}];
//	geometry shape <- square(EVIRONMENT_SIZE);
	file shape_file_roads <-  file("../includes/road.shp");
	geometry shape <- envelope(shape_file_roads) + 2*ROAD_WIDTH;

	float step <- STEP;
	init {
		create road  from: shape_file_roads{
//			shape <- polyline(road_nodes);
			width <- ROAD_WIDTH;
			geom_display <- shape + width;
			is_twoway <- true;
		}
		
		road_network <- as_edge_graph(road);
		
		create vehicle number: 1 {
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
			
//			source_node <- road_network.vertices[0];
//			final_node <- road_network.vertices[10];
//			source_node <- one_of(road_network.vertices);
//			final_node <- one_of(road_network.vertices);
			source_node <- {322.0999999999767, 665.781154679833};
			final_node <- {452.0999999999767, 55.981154679786414};
			
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