/***
* Name: microair1
* Author: dang tu
* Description: 
* Tags: Tag1, Tag2, TagN
***/

model microair1

/* Insert your model definition here */
global {
	graph<geometry, geometry> my_graph;
	file my_csv_file <- csv_file ( "../includes/test.csv", "," );
	int s <- 68;
	matrix road_matrix;
	init {
		my_graph <- graph<geometry, geometry>([]);
		road_matrix <- matrix(my_csv_file);
		ask my_cell {
			grid_value <- float(road_matrix[grid_x ,grid_y]);
		}
		//create node where its grid_value equals to 1
		loop i over:my_cell {
			if (i.grid_value = 1) {
				create my_node number:1 {
					cell <- i;
					location <- cell.location;
				}
			}
		}
		//create vehicle agent
		create vehicle number: 150 {
			location <- one_of(my_cell).location;
      	}
      	//add nodes and edges to graph
		loop j over: my_node {
			my_graph <- my_graph add_node j.location;
			loop k over:my_node{
			if (j.cell in k.cell.neighbors) {
					my_graph <- my_graph add_edge (k.location::j.location);
				}
			}
		}
	}
	
	reflex pollution_evolution{
		//ask all cells to decrease their level of pollution
		ask my_cell {pollution <- pollution * 0.9;}
		//diffuse the pollutions to neighbor cells
		diffuse var: pollution on: my_cell proportion: 0.9 ;
	}
}

species my_node {
	my_cell cell;
	reflex find_location {
		location <- cell.location;
	}
	aspect default {
		draw circle(0.2) color:#red;
	}
}

grid my_cell width:s height:s {
	float pollution <- 0.0 min: 0.0 max: 100.0;
	rgb color <- #green update: rgb(255 *(pollution/30.0) , 255 * (1 - (pollution/30.0)), 0.0);
}

species vehicle skills: [moving]{
	//Target point of the agent
	point target;
	//Probability of leaving the cell
	float leaving_proba <- 0.1; 
	//Speed of the agent
	float speed <- 5 #km/#h;
	rgb color <- rnd_color(255);
	
	//Reflex to leave the cell to another cell
	reflex leave when: (target = nil) and (flip(leaving_proba)) {
		list<my_cell> reachable_cells <- my_cell where (grid_value < 1);
		target <- one_of(reachable_cells).location;
	}
	
	//Reflex to move to the target on the road network
	reflex move when: target != nil {
		path path_followed <- goto (target: target, on: as_edge_graph(my_graph.edges), recompute_path: false, return_path: true);
		if (path_followed != nil ) {
			ask (my_cell overlapping path_followed.shape) {
				pollution <- pollution + 10.0;
			}
		}
		if (location = target) {
			target <- nil;
		}	
	}
	
	aspect default {
		draw circle(0.25) color: color;
	}
}

experiment my_experiment type: gui {
	output {
		display carte type: opengl{
			species vehicle;
			//display the pollution grid in 3D using triangulation.
			grid my_cell elevation: (1-road_matrix)*3 triangulation: true transparency: 0.7;
		
		}
		display graph2 type: opengl{
			graphics "the graph" {
				loop e over: my_graph.edges {
					draw e color: #blue; 
				}
				loop n over: my_graph.vertices {
					draw circle(0.25) at: point(n) color: #red; 
				}
			}
		}
	}
}