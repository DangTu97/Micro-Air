/**
* Name: createSHPfile
* Based on the internal empty template. 
* Author: tu
* Tags: 
*/


model createSHPfile

/* Insert your model definition here */

global {
	float road_width <- 8#m;
	geometry shape <- square(200);
	init {
//		one intersection
//		list<list<point>> roads <- [[{100,50}, {100,100}], [{100,100}, {100,150}], [{50,100}, {100,100}], [{100,100}, {150,100}]];
//		list<point> nodes <- [{100,50}, {100,100}, {100,150}, {50,100}, {150,100}];

//		four intersections
		list<list<point>> roads <- [[{100,50}, {100,100}], [{100,100}, {100,200}], [{50,100}, {100,100}], [{100,100}, {200,100}], 
									[{200,100}, {200,50}], [{200,100}, {250, 100}], [{200,100}, {200,200}], [{50,200}, {100,200}],
									[{100,200}, {100,250}], [{100,200}, {200,200}], [{200,200}, {250,200}], [{200,200}, {200,250}]];
		list<point> nodes <- [{100,50}, {200,50}, {50,100}, {50,200}, {100,250}, {200,250}, {250,200}, {250,100}, {100,100}, {200,100}, {100,200}, {200,200}];
		list<point> intersections <- [{100,100}, {200,100}, {100,200}, {200,200}];
		loop enpoints over:roads {
			create road {
				shape <- polyline(enpoints);
				width <- road_width;
				geom_display <- (shape + width);
				is_twoway <- true;
			}
		}
		
		loop r over:nodes {
			create roadNode {
				location <- r;
				shape <- location;
				geom_display <- shape + circle(0.3#m);
//				if location = {100,100} {
				if intersections contains location {
					type <- 'intersection';
				}
			}
		}
		
		//save building geometry into the shapefile: add the attribute TYPE which value is set by the type variable of the building agent and the attribute ID 
		save roadNode to:"../includes/4_intersection_nodes.shp" type:"shp" attributes: ["id":: int(self), "type"::type];
		save road to:"../includes/4_intersection_roads.shp" type:"shp" attributes: ["id":: int(self)];  
	}
}

species road {
	bool is_twoway;
	geometry geom_display;
	rgb color <- #white;
	float width;
	
	aspect base {
		draw geom_display color:color border:#white;
		if is_twoway { draw shape color: #grey; }
	}
}


species roadNode {
	string type;
	geometry geom_display;
	aspect default {
		draw geom_display color:color border:#white;
	}
}

experiment main type: gui {
	output {
		display map background:#grey {
			species road;
			species roadNode;
		}
	}
}
