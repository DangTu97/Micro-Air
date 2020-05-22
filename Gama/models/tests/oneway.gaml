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
			is_twoway <- false;
		}
		
		road_network <- as_edge_graph(road);
		
		loop vertex over:road_network.vertices {
			create traffic_light {
				shape <- circle(road_width);
				location <- vertex;
			}
		}
//		write road_network;
//		write road_network.vertices;
	}
	
	reflex init_traffic {
		geometry my_space <- traffic_light(0).shape;
		list<vehicle> vehicle_ovelap <- vehicle where (each overlaps my_space);
		if (length(vehicle_ovelap) = 0) {
			create vehicle number: 10 {
			name <- flip(0.3) ? 'car' : 'motorbike';
			if name = 'car' {
				length <- CAR_LENGTH;
				width <- CAR_WIDTH;
				df <- CAR_DF;
				db <- CAR_DB;
				dx <- width/2 + db;
				dy <- length/2 + df;
				speed <- INIT_SPEED;
				max_speed <- 0.2 + rnd(CAR_MAXSPEED - 0.2);
			} else {
				length <- MOTORBIKE_LENGTH;
				width <- MOTORBIKE_WIDTH;
				df <- MOTORBIKE_DF;
				db <- MOTORBIKE_DB;
				dx <- width/2 + db;
				dy <- length/2 + df;
				speed <- INIT_SPEED;
				max_speed <- 0.2 + rnd(MOTORBIKE_MAXSPEED - 0.2);
			}
			
			source_node <- road_network.vertices[1];
			final_node <- road_network.vertices[0];
			do compute_shortest_path;
//			write shortest_path;

			road_belong <-  shortest_path[0];
			display_polygon <- false;
			prob_go_opposite <- PROB_GO_OPPOSITE;
			prob_turn_right <- PROB_TURN_RIGHT;
//			location <- source_node + {rnd(3.0), rnd(6.0) - 3};
			location <- any_location_in(my_space);
			start_node <- road_belong.shape.points[0];
			target_node <- road_belong.shape.points[1];
			angle <- angle_between(start_node, start_node + {10,0}, target_node);
			do compute_road_belong_nodes;
			do update_polygon;
			target_space <- polyline([target_node - {1.5*road_width, 0}, target_node + {1.5*road_width, 0}]) rotated_by (angle + 90); }
		}
	}
}

experiment my_experiment {
	float minimum_cycle_duration <- TIME_STEP;
	output {
		display my_display background: #grey{
			species road aspect: base;
//			species space;
			species vehicle aspect: base;
//			species roadNode;
		}
	}
}