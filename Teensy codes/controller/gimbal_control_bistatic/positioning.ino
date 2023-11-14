// Code related to positioning stuff

void load_survey_points() {

  Serial.print("Loading the survey points: Num=");
  Serial.println(sizeof(gnd_points) / sizeof(gnd_points[0]));

  // CHANGE THESE COORDINATES BEFORE FLIGHT

  // Coordinates of the first point (first Point-of-interest)
  // Set as Origin
  // 0, 1
  // Golf-course
 // gnd_points[0].latitude = 33.2961279;
 // gnd_points[0].longitude = -87.6396212;
  // Arboretum
//  gnd_points[0].latitude = 33.1938106;
//  gnd_points[0].longitude = -87.4815271;
  // SERC QUAD
  gnd_points[0].latitude = 33.215120;
  gnd_points[0].longitude = -87.543326;
  gnd_points[0].altitude = 0;

  // Second Point (Any point perpendicular to the CMR flight path)
  // Golf-course
//  gnd_points[1].latitude = 33.2962405;
//  gnd_points[1].longitude = -87.6396963;
  // Arboretum
//  gnd_points[1].latitude = 33.193790;
//  gnd_points[1].longitude = -87.4815466;
  // SERC QUAD
  gnd_points[1].latitude = 33.215101;
  gnd_points[1].longitude = -87.543098;
  gnd_points[1].altitude = 0;  

  // Third Survey Point
//    gnd_points[2].latitude = 0;
//    gnd_points[2].longitude = 0;
//    gnd_points[2].altitude = 0;
  }

void setup_survey_points() {
  // get the distance of points from the origin at the beginning

  // getting distance from the origin for each survey point
  int n = sizeof(gnd_points) / sizeof(gnd_points[0]);

  for (int ii = 0; ii < n; ii++) {
    gnd_points[ii].distance_from_origin = get_distance(gnd_points[0].latitude, gnd_points[0].longitude, gnd_points[ii].latitude, gnd_points[ii].longitude);
    gnd_points[ii].deviation_from_line = 0.0;
    
    Serial.print("Dist of point from origin:: ");
    Serial.print(gnd_points[ii].distance_from_origin);
    Serial.println("m");
    }
  
  // getting the last index
  line_bearing = get_bearing(gnd_points[0].latitude, gnd_points[0].longitude, gnd_points[n-1].latitude, gnd_points[n-1].longitude);
  line_length = get_distance(gnd_points[0].latitude, gnd_points[0].longitude, gnd_points[n-1].latitude, gnd_points[n-1].longitude);
  
  Serial.print("The length of the survey line is ");
  Serial.print(line_length);
  Serial.println("m");
  
  Serial.print("The bearing of the survey line is ");
  Serial.print(line_bearing);
  Serial.println(" deg");  
  
  }


void get_current_pos(){
  // gets current position from the GNSS

  Serial.println("Calling position in get_current_pos() .........");

  // using BFS namespace
  current_pos.latitude = gnss.lat_deg();
  current_pos.longitude = gnss.lon_deg();
  current_pos.altitude = gnss.alt_wgs84_m();

  // current distance from the origin
  current_pos.distance_from_origin = get_distance(gnd_points[0].latitude, gnd_points[0].longitude, current_pos.latitude, current_pos.longitude);

  // current bearing from the origin 
  float current_bearing_from_origin = get_bearing(gnd_points[0].latitude, gnd_points[0].longitude, current_pos.latitude, current_pos.longitude);

  // current deviation (in angles) from the line defined in setup_survey_points()
  current_pos.deviation_from_line = current_bearing_from_origin - line_bearing;

  if (_DEBUG) {  
    Serial.println("-----------------------");
    Serial.print("Current position:: \t");
    Serial.print(current_pos.latitude, 6);
    Serial.print("\t");
    Serial.print(current_pos.longitude, 6);
    Serial.print("\t");
    Serial.println(current_pos.altitude, 3);
  
    Serial.println("-----------------------");
    Serial.print("Distance from origin :: \t");
    Serial.print(current_pos.distance_from_origin);
    Serial.println("m");
    
    Serial.print("Current Bearing from origin :: \t");
    Serial.print(current_bearing_from_origin);
    Serial.println("deg");
    
    Serial.print("Angle deviation from line :: \t");
    Serial.print(current_pos.deviation_from_line);
    Serial.println("deg");
  }

}


void get_closest_relative_height() {
  // call this function at a certain predefined rate. 
  // Gets the relative height of the point-of-interest. 
  // TODO: Figure out a way to do this for a large area. 

  // Currently just hardcoding a fixed value of 10m. 
  closest_rel_height = 7; 
  }


static double get_distance(double lat1, double lon1, double lat2, double lon2) {
  // differences between lats and lons
  double dlat = (lat2 - lat1) * d2r;
  double dlon = (lon2 - lon1) * d2r;

  lat1 = lat1 * d2r;
  lat2 = lat2 * d2r;
  // Haversine Equations
  double a = pow(sin(dlat/2),2) + pow(sin(dlon/2), 2) * cos(lat1) * cos(lat2);
  double radius = 6378100;
  double c = 2 * asin(sqrt(a));
  return radius * c;
}


static double get_bearing(double lat1, double lon1, double lat2, double lon2) {
  lat1 = lat1 * d2r;
  lat2 = lat2 * d2r;
  lon1 = lon1 * d2r;
  lon2 = lon2 * d2r;

  // Haversine Equations
  double y = sin(lon2 - lon1) * cos(lat2);
  double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lon2 - lon1);
  double theta = atan2(y, x);
  double bearing = (int(theta*r2d) + 360) % 360;
  return bearing;
  }
