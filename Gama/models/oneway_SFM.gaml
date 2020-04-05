/***
* Name: onewaySFM
* Author: dang tu
* Description: 
* Tags: Tag1, Tag2, TagN
***/

model onewaySFM

/* Insert your model definition here */

global {
	bool snapsave <- true;
	float environment_size <- 100.0 parameter: true;
	float margin <- 2.0;
	string scenario <- "frontal crossing" among: ["big crowd", "frontal crossing", "perpandicular crossing"] ;
	float step <- 0.3 min: 0.1 max: 1.0 step: 0.1 parameter: true;
				
	int nb_people <-  500 ;
	int nb_obstacles <- 0 parameter: true;
	float P_shoulder_length <- 0.45 parameter: true;
	float P_body_depth <- 0.28 parameter: true;
	bool P_use_body_geometry <- true parameter: true ;
	float P_proba_detour <- 0.5 parameter: true ;
	bool P_avoid_other <- true parameter: true ;
	float P_obstacle_consideration_distance <- 1.0 parameter: true ;
	
	
	string P_pedestrian_model among: ["simple", "SFM"] <- "SFM" parameter: true ;
	float P_obstacle_distance_repulsion_coeff <- 5.0 category: "simple model" parameter: true ;
	float P_overlapping_coefficient <- 2.0 category: "simple model" parameter: true ;
	float P_perception_sensibility <- 1.0 category: "simple model" parameter: true ;
	
	float P_A_SFM parameter: true <- 4.5 category: "SFM" ;
	float P_relaxion_SFM parameter: true <- 0.54 category: "SFM" ;
	float P_gama_SFM parameter: true <- 0.35 category: "SFM" ;
	float P_n_SFM <- 2.0 parameter: true category: "SFM" ;
	float P_n_prime_SFM <- 3.0 parameter: true category: "SFM";
	float P_lambda_SFM <- 2.0 parameter: true category: "SFM" ;
	
	geometry shape <- square(environment_size);
	geometry free_space <- copy(shape);
	
	list<image_file> images <- [image_file("../includes/car.png"), image_file("../includes/motorbike.png")];
	
	init {
		create road number: 1 {
			width <- 6 #m;
			length <- 100 #m;
			float l <- road(0).length;
			float w <- road(0).width;
			point p <- road(0).location;
			shape <- polygon([p - {w/2,l/2}, p + {w/2,-l/2}, p + {w/2,l/2}, p + {-w/2,l/2}]);
			float d <- 5 #m;
			bottom_space <- polygon([p + {-w/2,l/2 - d}, p + {w/2,l/2 - d}, p + {w/2,l/2}, p + {-w/2,l/2}]);
			top_space <- polygon([p - {w/2,l/2}, p + {w/2,-l/2}, p + {w/2,-l/2 + d}, p + {-w/2,-l/2 + d}]);
		}
	}
	
	reflex control_traffic {
		list<people> people_overlap <- (people where (each overlaps road(0).bottom_space));
		if (length(people_overlap) = 0) {
			create people number: 5 {
				pedestrian_model <- P_pedestrian_model;
				obstacle_distance_repulsion_coeff <- P_obstacle_distance_repulsion_coeff;
				obstacle_consideration_distance <-P_obstacle_consideration_distance;
				overlapping_coefficient <- P_overlapping_coefficient;
				perception_sensibility <- P_perception_sensibility ;
				shoulder_length <- P_shoulder_length;
				avoid_other <- P_avoid_other;
				proba_detour <- P_proba_detour;
				A_SFM <- P_A_SFM;
				relaxion_SFM <- P_relaxion_SFM;
				gama_SFM <- P_gama_SFM;
				n_SFM <- P_n_SFM;
				n_prime_SFM <- P_n_prime_SFM;
				lambda_SFM <- P_lambda_SFM;
				
				obstacle_species <- [people];
				//if (P_use_body_geometry) {shape <- compute_body();}
				if (nb_obstacles > 0) {obstacle_species<<obstacle;}
				
				location <- any_location_in(road(0).bottom_space);
				current_target <- closest_points_with(location, road(0).top_space)[1];
				
				name <- flip(0.2) ? 'car' : 'motorbike';
				if name = 'car' {
					length <- 3.8 #m;
					width <- 1.5 #m;
				} else {
					length <- 1.8 #m;
					width <- 0.7 #m;
				}
				shape <- rectangle(length, width) rotated_by heading;
				speed <- rnd(0.2, 1.0) #m/#s;
			}
		}
	}
}

species people skills: [pedestrian] schedules: shuffle(people) {
	rgb color <- rnd_color(255);
	point current_target;
	float speed <- 3 #km/#h;
	bool avoid_other <- true;
//	geometry shape update: P_use_body_geometry ? compute_body(): shape;
	
	string name;
	float length;
	float width;
	
	reflex move when: current_target != nil{
		if (nb_obstacles > 0) {
			do walk target: current_target bounds: road(0).shape;
		} else {
			do walk target: current_target bounds: road(0).shape speed: speed;
		}
		if (self distance_to current_target < 0.5) {
			do die;
		}
	}
	
	reflex update_shape {
		shape <- rectangle(length, width) rotated_by heading;
	}
	
	aspect default {
		if (name = 'car') {
			draw images[0] size: {length, width} rotate:heading;
		} else {
			draw images[1] size: {length, width} rotate:heading;
		}
//		draw shape color: #red border: #black;
	}
}

species obstacle {
	aspect default {
		draw shape color: #gray border: #black;
	}
}

species road {
	init {
		location <- {50,50};
	}
	geometry top_space;
	geometry bottom_space;
	float length;
	float width;
	aspect base {
		draw shape color: #white border:#grey;
		draw top_space color:#yellow;
		draw bottom_space color:#red;
	}
}

experiment frontal_crossing type: gui {
	float minimum_cycle_duration <- 0.05;
	action _init_ {
		create simulation with: [scenario :: "frontal crossing", nb_people::200];
	}
	output {
		display map synchronized:true autosave:snapsave {
//			graphics "areas" transparency: 0.5{
//				draw right_space color: #green border: #black;
//				draw left_space color: #red border: #black;
//			}
//			species obstacle;
			species road aspect:base;
			species people;
		}
	}
}

