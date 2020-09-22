/***
* Name: test3
* Author: dang tu
* Description: 
* Tags: Tag1, Tag2, TagN
***/

model createroad
import "../vehicle4.gaml"
import "../global_variables.gaml"
/* Insert your model definition here */

global {
//	file shape_file_roads <-  file("../includes/RoadCircleLanes.shp");
	geometry shape <- envelope(200);
	float step <- STEP;
	graph road_network;
	init {
//		create road {
//			shape <- polyline([{-30, 30}, {0, 30}]);
//			width <- ROAD_WIDTH;
//			geom_display <- shape + width;
//			is_twoway <- true;
//		}
//		
//		create road {
//			shape <- polyline([{60, 10}, {90, -10}]);
//			width <- ROAD_WIDTH;
//			geom_display <- shape + width;
//			is_twoway <- true;
//		}
//		
//		create road {
//			shape <- polyline([{0, 30}, {20, 50}, {40, 50}, {60, 30}]);
//			width <- ROAD_WIDTH;
//			geom_display <- shape + width;
//			is_twoway <- false;
//		}
//		
//		create road {
//			shape <- polyline([{60, 30}, {60, 10}]);
//			width <- ROAD_WIDTH;
//			geom_display <- shape + width;
//			is_twoway <- false;
//		}
//		
//		create road {
//			shape <- polyline([{20,0}, {0, 20}, {0, 30}]);
//			width <- ROAD_WIDTH;
//			geom_display <- shape + width;
//			is_twoway <- false;
//		}
//		
//		create road {
//			shape <- polyline([{60,10}, {40, 0}, {20, 0}]);
//			width <- ROAD_WIDTH;
//			geom_display <- shape + width;
//			is_twoway <- false;
//		}
		
		//circle 2
		list<point> roadNodes <- [{10,70}, {30,60}, {50,90}, {40,120}, {80,70}, {80,50}, {110,30}, {50,30}, {40,10}];
		loop i from: 0 to:length(roadNodes) - 1 {
			create roadNode {
				location <- roadNodes[i];
			}
		}
		
		create road {
			shape <- polyline([{10,70}, {30, 60}]);
			width <- ROAD_WIDTH;
			geom_display <- shape + width;
			is_twoway <- true;
		}

		create road {
			shape <- polyline([{50,90}, {40, 120}]);
			width <- ROAD_WIDTH;
			geom_display <- shape + width;
			is_twoway <- true;
		}
		
		create road {
			shape <- polyline([{80,50}, {110, 30}]);
			width <- ROAD_WIDTH;
			geom_display <- shape + width;
			is_twoway <- true;
		}
		
		create road {
			shape <- polyline([{50,30}, {40, 10}]);
			width <- ROAD_WIDTH;
			geom_display <- shape + width;
			is_twoway <- true;
		}
		
		// ---------------- oneway -------------
		create road {
			shape <- polyline([{50,30}, {40,40}, {30, 60}]);
			width <- ROAD_WIDTH;
			geom_display <- shape + width;
			is_twoway <- false;
		}
		
		create road {
			shape <- polyline([{30,60}, {30,80}, {50, 90}]);
			width <- ROAD_WIDTH;
			geom_display <- shape + width;
			is_twoway <- false;
		}
		
		create road {
			shape <- polyline([{50,90}, {70,90}, {80, 70}]);
			width <- ROAD_WIDTH;
			geom_display <- shape + width;
			is_twoway <- false;
		}
		
		create road {
			shape <- polyline([{80,70}, {80, 50}]);
			width <- ROAD_WIDTH;
			geom_display <- shape + width;
			is_twoway <- false;
		}
		
		create road {
			shape <- polyline([{80,50}, {70,40}, {50, 30}]);
			width <- ROAD_WIDTH;
			geom_display <- shape + width;
			is_twoway <- false;
		}
		
		road_network <-  (as_driving_graph(road, roadNode));
		write road_network;

		save road type:'shp' to:"junction.shp" with:[is_twoway::"type"];
		save roadNode type:'shp' to:"roadNode.shp";
	}
}

experiment my_experiment {
	float minimum_cycle_duration <- TIME_STEP;
	output {
		display my_display background:#grey{
			species road aspect:base;
			species roadNode aspect:base;
//			species vehicle aspect:base;
		}
	}
}