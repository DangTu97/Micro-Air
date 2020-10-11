/***
* Name: test4
* Author: dang tu
* Description: 
* Tags: Tag1, Tag2, TagN
***/

model test4
import "../vehicle4.gaml"
import "../global_variables.gaml"
/* Insert your model definition here */
global {
	file shape_file_roads <-  file("../includes/junction.shp");
	file shape_file_roadNodes <-  file("../includes/roadNode.shp");
	geometry shape <- envelope(shape_file_roads) + 20;
	float step <- STEP;
	init {
		create road from: shape_file_roads with:[is_twoway:: read("type")]{
			width <- ROAD_WIDTH;
			geom_display <- shape + width;
			switch is_twoway {
				match true {
					create road {
						shape <- polyline(reverse(myself.shape.points));
						width <- ROAD_WIDTH;
						geom_display <- shape + width;
						is_twoway <- true;
						linked_road <- myself;
						myself.linked_road <- self;
					}
				}
			}
		}
		
		create roadNode from: shape_file_roadNodes;
		road_network <- (as_driving_graph(road, roadNode));
	}
	
	reflex init_traffic when:mod(cycle,4)=0 {
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
			source_node <- one_of([roadNode(0),roadNode(3),roadNode(6),roadNode(8)]);
			final_node <- one_of([roadNode(0),roadNode(3),roadNode(6),roadNode(8)]);
			
			do compute_shortest_path;
			if length(shortest_path) = 0 { do die; }
			road_belong <-  shortest_path[0];
			start_node <- source_node;
			do compute_road_belong_nodes;
			target_node <- road_belong_nodes[1];
			angle <- angle_between(start_node, start_node + {10,0}, target_node);
			
			float D <- ROAD_WIDTH;
			float a <- (target_node - start_node).location.x;
			float b <- (target_node - start_node).location.y;   
			point right_point <- start_node + {-b*D/sqrt(a*a + b*b), a*D/sqrt(a*a + b*b)};
			float lambda <- rnd(0.0,1.0);
			location <- (start_node*lambda + right_point*(1-lambda));
			
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
//			species roadNode aspect:base;
			species vehicle aspect:base;
		}
	}
}
