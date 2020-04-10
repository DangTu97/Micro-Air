/***
* Name: movingongrid2
* Author: dang tu
* Description: 
* Tags: Tag1, Tag2, TagN
***/

model movingongrid

/* Insert your model definition here */

global {
//	int gridx_min <- 23;
//	int gridx_max <- 28;
//	int gridy_min <- 23;
//	int gridy_max <- 28;
// 	int grid_size <- 50;
 	
	int gridx_min <- 46;
	int gridx_max <- 55;
	int gridy_min <- 46;
	int gridy_max <- 55;
	int grid_size <- 100;
	
	image_file image <- image_file("../includes/car.png");
	
	int d <- (gridx_max - gridx_min)/2 as int;
	
	init {
		ask my_cell {
			if (gridx_min <= grid_x and grid_x <= gridx_max) 
			or (gridy_min <= grid_y and grid_y <= gridy_max) {
				is_road <- true;
			}
		}
		
		create vehicle number:1 {
			on_cell <- my_cell[52,58];
			target_cell <- my_cell[8,48];
			location <- on_cell.location;
			
			optimal_route <- compute_optimal_path (on_cell, target_cell);
			write has_colision;
			do detect_colision;
			write has_colision;
			if has_colision {
				write "-----------";
				write optimal_route;
				do find_alternative_path;
				has_colision <- false;
				write optimal_route;
			}
			ask optimal_route {
				color <- #red;
			}
		}
		
		create vehicle number:1 {
			on_cell <- my_cell[55,58];
			target_cell <- my_cell[8,48];
			location <- on_cell.location;
			
			optimal_route <- compute_optimal_path (on_cell, target_cell);
			do detect_colision;
			if has_colision {
				do find_alternative_path;
				has_colision <- false;
			}
			ask optimal_route {
				color <- #red;
			}
		}
		
		create vehicle number:1 {
			on_cell <- my_cell[52,60];
			target_cell <- my_cell[8,48];
			location <- on_cell.location;
			
			optimal_route <- compute_optimal_path (on_cell, target_cell);
			write has_colision;
			do detect_colision;
			write has_colision;
			if has_colision {
				do find_alternative_path;
				has_colision <- false;
			}
			ask optimal_route {
				color <- #red;
			}
		}
		
		create vehicle number:1 {
			on_cell <- my_cell[55,60];
			target_cell <- my_cell[8,48];
			location <- on_cell.location;
			
			optimal_route <- compute_optimal_path (on_cell, target_cell);
			do detect_colision;
			if has_colision {
				do find_alternative_path;
				has_colision <- false;
			}
			ask optimal_route {
				color <- #red;
			}
		}	
		
		create vehicle number:1 {
			on_cell <- my_cell[48,6];
			target_cell <- my_cell[8,48];
			location <- on_cell.location;
			
			optimal_route <- compute_optimal_path (on_cell, target_cell);
			do detect_colision;
			if has_colision {
				do find_alternative_path;
				has_colision <- false;
			}
			ask optimal_route {
				color <- #red;
			}
		}

	}
}

grid my_cell width:grid_size height:grid_size neighbors:8 {
	bool is_road;
//	bool is_blocked;
	string side;

	reflex update_color when:cycle = 1 {
//		color <- side != nil ? ((side = "left") ? #red : #white) : #green;
		color <- is_road ? #white : #grey;
	}
}

species vehicle skills:[moving]{
	my_cell target_cell;
	my_cell on_cell;
	list<my_cell> optimal_route;
	bool has_colision;
	
	list<my_cell> compute_optimal_path(my_cell cell, my_cell destination_cell)  {
		list<my_cell> optimal_path <- [];
		my_cell temp <- cell;
		loop while: length(optimal_path) <= 3 {
//			write optimal_path;
			temp <- get_closest_cell(temp, destination_cell);
			optimal_path <+ temp;
		}
		return optimal_path;
	}
	
	my_cell get_closest_cell(my_cell cell, my_cell destination_cell) {
		list<my_cell> neighbors_of_cell <- get_neighbor(cell);
		 // 1 means to choose the closest cell;
		list<my_cell> cells <- neighbors_of_cell closest_to(destination_cell, 1);
		my_cell closest_cell <- cells[0];
		return closest_cell;
	}
	
	list<my_cell> get_neighbor(my_cell cell) {
		list<string> directions;
		list<my_cell> neighbors;
		int x <- cell.grid_x;
		int y <- cell.grid_y;
		// check near to gridx_max or gridx_min
		if (gridx_min <= x  and x <= gridx_max) {
			if abs(gridx_max - x) < abs(x - gridx_min) {
				directions <+ "up";
			} else {
				directions <+ "down";
			}
		}
		
		if (gridy_min <= y  and y <= gridy_max) {
			if abs(gridy_max - y) < abs(y - gridy_min) {
				directions <+ "right";
			} else {
				directions <+ "left";
			}
		}
		
		loop value over:directions {
			switch value {
				match "up" {
					neighbors <<+ (cell.neighbors where (each.grid_y = y - 1 and 
						(each.grid_x <= gridx_max and gridx_max - d <= each.grid_x)));
				}
				match "down" {
					neighbors <<+ (cell.neighbors where (each.grid_y = y + 1 and 
						(each.grid_x <= gridx_min + d and gridx_min <= each.grid_x)));				
				}
				match "right" {
					neighbors <<+ (cell.neighbors where (each.grid_x = x + 1 and 
						(each.grid_y <= gridy_max and gridy_max - d <= each.grid_y)));
				}
				match "left" {
					neighbors <<+ (cell.neighbors where (each.grid_x = x - 1 and 
						(each.grid_y <= gridx_min + d and gridy_min <= each.grid_y)));		
				}
			}
		}
		return neighbors;
	}
	
	action detect_colision {
		ask vehicle {
			if self != myself {
				if last(myself.optimal_route) = last(optimal_route) {
					myself.has_colision <- true;
					//write has_colision;
				}
			}
		}
	}

//	action detect_colision {
//		ask vehicle {
//			if self != myself {
//				int n <- min(length(self.optimal_route), length(myself.optimal_route));
//				write n;
//				if n > 0 {
//					loop i from:0 to: n-1 {
//						if myself.optimal_route[i] = self.optimal_route[i] {
//							has_colision <- true;
//						}
//					}
//				}
//			}
//		}
//	}
	
	action find_alternative_path {
		my_cell cell <- first(optimal_route);
		
		//change the road
		//get closest cell which differs from optimal_route[0]
		list<my_cell> possible_travel_cells <- (get_neighbor(on_cell) where (each != cell));
		list<my_cell> closest_alternative <- possible_travel_cells closest_to(target_cell, 1);
		optimal_route[0] <- closest_alternative[0];
	}
	
	reflex move when: target_cell != nil {
		ask target_cell {
			color <- #yellow;
		}
		
		if optimal_route != nil {
			ask optimal_route {
				color <- #white;
			}
		}
		
		//compute optimal path for vehicle
//		if cycle < 3 {
//			optimal_route <- compute_optimal_path (on_cell, target_cell);
//		} else if !(last(optimal_route) in target_cell.neighbors) {
//			optimal_route <- compute_optimal_path (on_cell, target_cell);
//		}
//		else {
//			optimal_route >>- first(optimal_route);
//		}
		
		
		//moving
		on_cell <- first(optimal_route);
		location <- on_cell.location;
		
		optimal_route <- compute_optimal_path (on_cell, target_cell);
		
		//detect colision if it happens then find alternative for optimal route
		do detect_colision;

		if has_colision {
			do find_alternative_path;
			has_colision <- false;
		}
		
		//mark path
//		ask optimal_route {
//			color <- #red;
//		}
//		ask on_cell {
//			color <- #blue;
//		}
		
		//check whether getting target or not
		if (target_cell in on_cell.neighbors) {
			on_cell <- target_cell;
			location <- on_cell.location;
			target_cell <- nil;
		}
	}
	
//	reflex check {
//		bool signal <- detect_colision(my_cell[52,58], my_cell[55,59]);
//		write signal;
//	}
	
	aspect car {
//		draw box(1,1,1) color: #blue;
		draw image size: {0.75, 0.45} rotate:heading;
	}
}

experiment my_exper {
	float minimum_cycle_duration <- 0.25;
	output {
		display my_display {
			grid my_cell lines:#black;
			species vehicle aspect:car;
		}
	}
}