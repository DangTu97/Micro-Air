/***
* Name: intersection
* Author: dang tu
* Description: 
* Tags: Tag1, Tag2, TagN
***/

model intersection
import "../KSE paper/vehicle.gaml"
import "emission.gaml"
import "../global_variables.gaml"
/* Insert your model definition here */

global {
	geometry shape <- square(500);
	float step <- 0.05#s;
	int nb_vehicles;
	int traffic_volume <- 5;
	float road_width <- ROAD_WIDTH;
	init {
		write step;
		list<list<point>> my_nodes <- [[{100,50}, {100,100}], [{100,100}, {100,150}], [{50,100}, {100,100}], [{100,100}, {150,100}]];
		loop nodes over:my_nodes {
			create road {
				shape <- polyline(nodes);
				road_width <- ROAD_WIDTH;
				geom_display <- (shape + road_width);
				is_twoway <- true;
			}
		}
		
		create traffic_light {
			location <- {100, 99 - road_width};
			shape <- circle(1);
			counter <- 0;
			is_green <- false;
			t_g <- GREEN_TIME;
			t_y <- YELLOW_TIME;
			t_r <- RED_TIME;
			direction_control <- 90.0;
			my_geom <- polyline([location, location + {-road_width, 0}]);
		}
		
		create traffic_light {
			location <- {100, 101 + road_width};
			shape <- circle(1);
			counter <- 0;
			is_green <- false;
			t_g <- GREEN_TIME;
			t_y <- YELLOW_TIME;
			t_r <- RED_TIME;
			direction_control <- 270.0;
			my_geom <- polyline([location, location + {road_width, 0}]);
		}
		
		create traffic_light {
			location <- {99 - road_width, 100};
			shape <- circle(1);
			counter <- RED_TIME;
			t_g <- GREEN_TIME;
			t_y <- YELLOW_TIME;
			t_r <- RED_TIME;
			is_green <- true;
			direction_control <- 0.0;
			my_geom <- polyline([location, location + {0, road_width}]);
		}
		
		create traffic_light {
			location <- {101 + road_width, 100};
			shape <- circle(1);
			counter <- RED_TIME;
			t_g <- GREEN_TIME;
			t_y <- YELLOW_TIME;
			t_r <- RED_TIME;
			is_green <- true;
			direction_control <- 180.0;
			my_geom <- polyline([location, location + {0, -road_width}]);
		}
		
		loop i from: 0 to:3 {
			road(i).light_belong <- traffic_light(i);
		}
		
		road_network <- as_edge_graph(road);
		write road_network;
		
		create block_space {
			location <- {100, 100};
			shape <- circle(2);
		}
		
		// ----------------------
		create init_space {
			location <- {100,50} + {-road_width/2, 2};
			my_point <- {100,50};
			geom <- rectangle(road_width, 2);
		}
		
		create init_space {
			location <- {100,150} + {road_width/2, -2};
			my_point <- {100,150};
			geom <- rectangle(road_width, 2);
		}
		
		create init_space {
			location <- {50,100} + {2,road_width/2};
			my_point <- {50,100};
			geom <- rectangle(road_width, 2) rotated_by 90;
		}
		
		create init_space {
			location <- {150,100} + {-2,-road_width/2};
			my_point <- {150,100};
			geom <- rectangle(road_width, 2) rotated_by 90;
		}
	}
	
	reflex init_traffic when:mod(cycle,20)=0 {
		list<point> targets <- [];
		loop i from:0 to:3 {
			if length(agents_overlapping(init_space(i).geom)) = 2 {
				targets <+ init_space(i).my_point;
			}
		}
		
		if length(targets) > 1 {
			create vehicle number: traffic_volume {
				int i <- rnd(1, 100);
				type <- (i < 2) ? 'BUS' : ( i < 20 ? 'CAR' : 'MOTORBIKE');
					
				if type = 'CAR' {
					max_speed <- CAR_MAXSPEED;
				} else if type = 'MOTORBIKE' {
					max_speed <- MOTORBIKE_MAXSPEED;
				} else {
					max_speed <- BUS_MAXSPEED;
				}

				source_node <- one_of(targets);
				final_node <-  one_of(targets where (each != source_node));
				do set_type;
				
				// location
				point p1 <- start_node;
				float a <- (target_node - start_node).x;
				float b <- (target_node - start_node).y;
				float d <- distance_to(target_node, start_node);
				point p2 <- p1 + {road_width*a/d, road_width*b/d};
				
				point center <- (p1 + p2)/2;
				float D <- 0.5*road_width;
				point free_space_center <- center + {b*D/sqrt(a*a + b*b), - a*D/sqrt(a*a + b*b)};
				if angle_between(start_node, target_node, free_space_center) > 180 {
					free_space_center <- center + { -b*D/sqrt(a*a + b*b), a*D/sqrt(a*a + b*b)};	
				}	
				point p3 <- free_space_center*2 - p1;
				point p4 <- free_space_center*2 - p2;
				free_space <- polygon([p1,p2,p3,p4, p1]);
				location <- any_location_in(free_space);
			}
		}
		
		nb_vehicles <- length(vehicle);
	}
}



experiment my_experiment {
	float minimum_cycle_duration <- TIME_STEP;
	output {
		display my_display background: #grey{
			species road aspect: base;
			species traffic_light aspect: base;
//			species free_myspace aspect: base;
//			species block_space aspect: base;
			species vehicle aspect: base;
			species Emis;
		}
		
		display my_chart refresh:every(200#cycle) {
			chart "Number of vehicles" position: {0, 0.5} size: {1.0,0.5}  {
				data "Vehicle" value:nb_vehicles color:#green;
			}
		}
		monitor "No. vehicles" value: nb_vehicles;
	}
}