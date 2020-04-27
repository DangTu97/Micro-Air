/***
* Name: singleroad
* Author: dang tu
* Description: 
* Tags: Tag1, Tag2, TagN
***/

model singleroad
import "../vehicle.gaml"
import "../global_variables.gaml"


/* Insert your model definition here */

global {
	file shape_file_roads <-  file("../includes/multilines.shp");
	geometry shape <- envelope(shape_file_roads) + 2*road_width;
	bool  display3D<- false;
	init {
		create road from: shape_file_roads {
			geom_display <- shape + road_width;
			is_twoway <- true;
//			write length(shape.points);
		}
		
		list<point> nodes <- road(0).shape.points;
	
		road_network <- as_edge_graph(road);
		write road_network;
	}
	
	reflex init_traffic when:mod(cycle,200) = 0{
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
			
			source_node <- road_network.vertices[1];
			final_node <- road_network.vertices[0];
			do compute_shortest_path;
//			write shortest_path;

			road_belong <-  shortest_path[0];
			display_polygon <- false;
			prob <- 0.0;
			location <- source_node + {rnd(3.0), rnd(3.0)};
			start_node <- road_belong.shape.points[5];
			target_node <- road_belong.shape.points[4];
			angle <- angle_between(start_node, start_node + {10,0}, target_node);
			do compute_road_belong_nodes;
			do update_polygon;
			target_space <- polyline([target_node - {1.5*road_width, 0}, target_node + {1.5*road_width, 0}]) rotated_by (angle + 90);
		}
		
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
			
			source_node <- road_network.vertices[0];
			final_node <- road_network.vertices[1];
			do compute_shortest_path;
//			write shortest_path;
	
			road_belong <-  shortest_path[0];
			display_polygon <- false;
			prob <- 0.0;
			location <- source_node + {rnd(1.0), rnd(3.0)};
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
//			species space;
			species vehicle aspect: base;
		}
	}
}