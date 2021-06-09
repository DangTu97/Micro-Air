/**
* Name: testload
* Based on the internal empty template. 
* Author: tu
* Tags: 
*/


model testload

/* Insert your model definition here */

global {
	file road_node_file <- file('../../includes/map/nodes.shp');
	file road_file <- file('../../includes/map/edges.shp');
	file bound_file <- file('../../includes/map/bound.shp');
	geometry shape <- envelope(bound_file);
	init {
		create roadnode from:road_node_file with:[type::read('highway')]{
			geom_display <- shape + 1.5#m;
			if type != nil { write type; }
		}
		create road from:road_file with: [is_twoway::!bool(get('oneway')), junction::string(get('junction'))]{
			width <- 3#m;
			geom_display <- shape + width;
		}
	}
}

species road {
	bool is_twoway;
	float width;
	geometry geom_display;
	rgb color <- #white;
	string junction;
	
	aspect base {
		draw geom_display color:color border:#white;
		if is_twoway { draw shape color: #grey; }
		if (junction = 'roundabout') { draw geom_display color:#red; }
	}
}

species roadnode {
	geometry geom_display;
	string type;
	aspect base {
		draw geom_display color: (type = 'traffic_signals') ? #red : #blue;
	}
}

experiment my_experiment {
	output {
		display my_display background:#grey {
			species road aspect:base;
			species roadnode aspect:base;
		}
	}
}