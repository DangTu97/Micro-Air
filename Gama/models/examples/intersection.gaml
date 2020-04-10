/***
* Name: intersection
* Author: dang tu
* Description: traffic simulation with different directions
* Tags: Tag1, Tag2, TagN
***/

model intersection2

/* Insert your model definition here */

global {   
	graph my_graph <- graph([]);
	list<image_file> images <- [image_file("../includes/car.png"), image_file("../includes/motorbike.png")];
	int nb_vehicle <- 10;
	
	init {

		list<point> nodes <- [{0,0}, {50,0}, {100,0}, {50,50}, {50,100}];
//		map<point, point> edges <- [{50,100}::{50,50}, {50,50}::{0,0}, {50,50}::{50,0}, {50,50}::{100,0}];
		loop node over:nodes {
			my_graph <- my_graph add_node node.location;
		}
		
		my_graph <- my_graph add_edge (nodes at 4::nodes at 3);
		my_graph <- my_graph add_edge (nodes at 3::nodes at 0);
		my_graph <- my_graph add_edge (nodes at 3::nodes at 1);
		my_graph <- my_graph add_edge (nodes at 3::nodes at 2);
		
		loop vertex over:nodes {
			create roadNode {
				location <- vertex;
				if (location = {50,50}) {
//					shape <- polygon([{47,43}, {53,43}, {53, 51}, {47,51}]); 
					shape <- rectangle(6,1);
					is_traffic_signal <- true;
				}
			}
		}
		
		create road {
			shape <- polyline(reverse([{50,100}, {50,50}]));
			geom_display <- (shape + 3);
		}
		create road {
			shape <- polyline(reverse([{50,50}, {0,0}]));
			geom_display <- (shape + 3);
		}
		create road {
			shape <- polyline(reverse([{50,50}, {50,0}]));
			geom_display <- (shape + 3);
		}
		create road {
			shape <- polyline(reverse([{50,50}, {100,0}]));
			geom_display <- (shape + 3);
		}
	}
	
	reflex produce_traffic {
		geometry space <- polygon([{47,94}, {53,94}, {53,100}, {47,100}]);
		list<vehicle> vehicle_ovelap <- vehicle where (each overlaps space);
		if (length(vehicle_ovelap) = 0) {
			create vehicle number: nb_vehicle {
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
				
				source_node <- my_graph.vertices[4];
				final_node <- one_of([my_graph.vertices[0], my_graph.vertices[1], my_graph.vertices[2]]);
				do compute_shortest_path;
				next_node <- shortest_path[1];
				current_node <- source_node;
				location <- any_location_in(space);
			}
		}
	}
	
}

species roadNode skills: [skill_road_node] {
	bool is_traffic_signal;
	
	aspect geom3D {
		if (is_traffic_signal) {
			draw shape at:location color: #green;
		}
	}
}

species road skills: [skill_road] { 
	string oneway;
	geometry geom_display;
	int id;
	aspect geom {    
		draw geom_display border: #grey  color: #white ;
	}  
}

species vehicle skills:[moving] {
	string name;
	float length;
	float width;
	float max_speed;
	float df;
	float db;
	float dx;
	float dy;
	
	point final_node;
	point next_node;
	point current_node;
	point source_node;
	point target;
	list<point> shortest_path;
	
	geometry current;
	geometry front;
	geometry left;
	geometry right;
	
	list<vehicle> get_vehicle_conflict_front {
		list<vehicle> vehicle_conflict_front <- (vehicle at_distance(10 + dy)) where (each.current overlaps self.front);
		return vehicle_conflict_front;
	}
	
	list<vehicle> get_vehicle_conflict_left {
		list<vehicle> vehicle_conflict_left <- (vehicle at_distance(10 + dy)) where (each.current overlaps self.left);
		return vehicle_conflict_left;
	}
	
	list<vehicle> get_vehicle_conflict_right {
		list<vehicle> vehicle_conflict_right <- (vehicle at_distance(10 + dy)) where (each.current overlaps self.right);
		return vehicle_conflict_right;
	}
	
	bool check_go_straight {
		list<vehicle> vehicle_conflict_front <- get_vehicle_conflict_front();
//		bool is_on_road <- false;
//		ask road {
//			if (myself.front.location overlaps self.geom_display){
//				is_on_road <- true;
//			}
//		}
//		return (length(vehicle_conflict_front) = 0  and is_on_road) ? true : false;
		return (length(vehicle_conflict_front) = 0) ? true : false;
	}
	
	bool check_turn_left {
		list<vehicle> vehicle_conflict_left <- get_vehicle_conflict_left();
		bool is_on_road <- false;
		ask road {
			if (myself.left.location overlaps self.geom_display){
				is_on_road <- true;
			}
		}
		return ((length(vehicle_conflict_left) = 0) and is_on_road) ? true : false;
	}
	
	bool check_turn_right {
		list<vehicle> vehicle_conflict_right <- get_vehicle_conflict_right();
		bool is_on_road <- false;
		ask road {
			if (myself.right.location overlaps self.geom_display){
				is_on_road <- true;
			}
		}
		return ((length(vehicle_conflict_right) = 0) and is_on_road) ? true : false;
	}
	
	action update_polygon {
		current <- polygon([location + {dy, -dx}, location + {dy, dx},
						  location + {- dy, dx}, location + {-dy, -dx}]) rotated_by heading;
	    						  
		float d <- distance_to(current_node, next_node);
		float a <- (next_node - current_node).location.x;
		float b <- (next_node - current_node).location.y;
		float k1 <- 2*dy/d;
		float k2 <- - 2*dy/d;
		point p1 <- location + {k1*a, k1*b};
		point p2 <- location + {k2*a, k2*b};
		point front_point;
		float angle <- angle_between(current_node,current_node + {10,0}, next_node);
		if ( angle_between(location, location + {10,0}, p1) = angle ) {
			front_point <- p1;
		} else {
			front_point <- p2;
		}
		front <- polygon([front_point + {dy, -dx}, front_point + {dy, dx},
						  front_point + {- dy, dx}, front_point + {-dy, -dx}]) rotated_by angle;
						  
		
		float D <- 2*dx;
		point left_point <- front_point + {b*D/sqrt(a*a + b*b), - a*D/sqrt(a*a + b*b)};	  
		left <- polygon([left_point + {dy, -dx}, left_point + {dy, dx},
						  left_point + {- dy, dx}, left_point + {-dy, -dx}]) rotated_by angle;
		
		point right_point <- front_point + { -b*D/sqrt(a*a + b*b), a*D/sqrt(a*a + b*b)};				  
		right <- polygon([right_point + {dy, -dx}, right_point + {dy, dx},
						  right_point + {- dy, dx}, right_point + {-dy, -dx}]) rotated_by angle;
						  
		target <- front.location;
	}
	
	action compute_shortest_path {
   		path my_path <- path_between(my_graph, source_node, final_node);
        list my_path <- string(my_path) split_with('::[]()as path');
       
    	loop p over:my_path {
    		list a <- string(p) split_with('{}');
    		shortest_path <+ point('{' + a[0] + '}');
    	}
	}

	action speed_up {
		speed <- min(1.0, speed + 0.05);
	}
	
	action slow_down(vehicle vehicle_ahead) {
		speed <- max(0, speed - 0.4);
	}
	
	reflex move when:target != nil {
		if (distance_to(location, next_node) < 6) {
			// when vehicle reaches final destination, do die
			if (next_node = final_node) {
				do die;
			}
			
			// assign new next_node, current_node when vehicle reaches a node which is not the final destination
			roadNode node;
			ask roadNode {
				if (distance_to(myself.next_node, self.location) < 1) {
					node <- self;
				}   
			}
			if ((location overlaps node.shape) and (node.is_traffic_signal = true)) {
				int index <- (shortest_path index_of next_node);
				if (index < length(shortest_path) - 1) {
					current_node <- next_node;
					next_node <- shortest_path[index + 1];
				} else {
					next_node <- nil;
					do die;
				}
			}
		}
		
		list<vehicle> vehicle_conflict_front <- get_vehicle_conflict_front();
		list<vehicle> vehicle_conflict_left <- get_vehicle_conflict_left();
		list<vehicle> vehicle_conflict_right <- get_vehicle_conflict_right();
		
		if (speed > max_speed) {
			speed <- max_speed;
		}

		if (check_go_straight() = true) {
			do speed_up;
			target <- front.location;
		} else if (check_turn_left() = true) {
			do speed_up;
			target <- left.location;
		} else if (check_turn_right() = true) {
			do speed_up;
			target <- right.location;
		} else {
			target <- front.location;
			do slow_down(first(get_vehicle_conflict_front()));
		}
		do goto target: target speed:speed;
	}
	
	reflex update_line {
		do update_polygon;
	}
	
	aspect base {
		draw current color: #yellow;
		draw front color: #red;
		draw left color: #blue;
		draw right color: #blue;
		if (name = 'car') {
			draw images[0] size: {length, width} rotate:heading;
		}
		
		if (name = 'motorbike') {
			draw images[1] size: {length, width} rotate:heading;
		}
	}
}

experiment traffic_simulation {
	float minimum_cycle_duration <- 0.05;
	output {
		display city_display {
			species road aspect: geom;
			species roadNode aspect: geom3D;
			species vehicle aspect: base;
		}
	}
}