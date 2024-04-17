#include <BasicLinearAlgebra.h>
#include <Wire.h>
#include <MS5xxx.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SD.h>
#include <SPI.h>
#include <Adafruit_GPS.h>
#include <Adafruit_ADXL375.h>


//////////////////////////////////////////////////////////////////////////////////////////////////////
// ************************ Global Vars used in Setup and Loop go here **************************** //
//////////////////////////////////////////////////////////////////////////////////////////////////////

bool recov_trigger = false; // set true to activate the solenoid/explosives/whatever they use

///////////////////////////////////////////////////////////////////////////////////////////////////////
// ************************ Global Vars used in Setup and Loop go here **************************** //
//////////////////////////////////////////////////////////////////////////////////////////////////////


// ********************************** Barometric altimeter function *********************************//

MS5xxx barometric_altimeter(&Wire);
double pres;
double altitude;

void initialize_barometric_altimeter() {
  if(barometric_altimeter.connect()>0) {
    Serial.println("Error connecting barometric altimeter");
    delay(500);
    }
  barometric_altimeter.ReadProm(); // for calibration curves
}

void read_barometric_altimeter_data(){
  barometric_altimeter.Readout();
  pres = 0.01*(static_cast<double>(barometric_altimeter.GetPres())); //millibars
  altitude = (1-pow((pres/1013.25),0.190284))*145366.45*0.3048; //meters
}

// ******************************** End of Barometric altimeter functions ******************************* // 

// ************************** IMU and orientation calculations for BNO055 ********************************* //

// Initialize variables to save retrieved x,y,z values
double a11 = 0; double a12 = 0; double a13 = 0; 

// Initialize variables for rotation matrix
double rm11 = 0; double rm12 = 0; double rm13 = 0;
double rm21 = 0; double rm22 = 0; double rm23 = 0;
double rm31 = 0; double rm32 = 0; double rm33 = 0;

// Initialize variables to assign final x,y,z values
double bno_ax = 0; double bno_ay = 0; double bno_az = 0;  // acceleration
double q_x = 0; double q_y = 0; double q_z = 0; double q_w = 0;// quaternion save values

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

void initialize_bno055(void){
  Serial.println("Initializing Orientation Sensor");
  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("No BNO055 detected - wiring or I2C ADDR?");
  }
}

void getquat_rot_matrix(){          //Retrieve quaternion, convert to rotation matrix
  // Request quaternion data from BNO055
  imu::Quaternion quat = bno.getQuat();     
  q_x = quat.x();
  q_y = quat.y();
  q_z = quat.z();
  q_w = quat.w();
  // Build rotation matrix from quaternion
  rm11 = q_w*q_w + q_x*q_x - q_y*q_y - q_z*q_z;   
  rm12 = 2*q_x*q_y - 2*q_w*q_z;            
  rm13 = 2*q_x*q_z + 2*q_w*q_y;
  rm21 = 2*q_x*q_y + 2*q_w*q_z;       
  rm22 = q_w*q_w - q_x*q_x + q_y*q_y - q_z*q_z;          
  rm23 = 2*q_y*q_z - 2*q_w*q_x;     
  rm31 = 2*q_x*q_z - 2*q_w*q_y;       
  rm32 = 2*q_y*q_z + 2*q_w*q_x;            
  rm33 = q_w*q_w - q_x*q_x - q_y*q_y + q_z*q_z;
}

void getacc(){ 
  // Retrieve acceleration vector from sensor
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  // Store values in an acceleration matrix
  a11 = accel.x();
  a12 = accel.y();
  a13 = accel.z();
}

void getfinal(){ 
  // Perform matrix multiplication of rotation and acceleration matrix 
  bno_ax = rm11*a11 + rm12*a12 + rm13*a13;
  bno_ay = rm21*a11 + rm22*a12 + rm23*a13;
  bno_az = rm31*a11 + rm32*a12 + rm33*a13;
}

void get_bno055_data_rotated(){
  // Retrieve sensor data, store acceleration vector
  getacc();
  // After getting sensor data, jump to quaternion calculations
  getquat_rot_matrix();
  // Jump to calculations for matrix multiplication to recieve final x, y, z coordinates.
  getfinal();
}

// ************************** End of BNO055 section *************************** //


// ************************************** GPS section *************************************** //
float gps_latitude;
float gps_longitude;
float gps_altitude;
int gps_fix_quality;
float gps_speed;
float gps_angle;
int gps_ant_stat;

double enu_ae; // East
double enu_an; // North
double enu_az; // Up
char gps_lat_hem;
char gps_long_hem;

// Constants for Earth's radius in meters and the conversion from radians to degrees
const double earthRadiusM = 6371000; // meters
const double radToDeg = 57.2957795; // 1 radian = 57.2957795 degrees

Adafruit_GPS GPS(&Serial8);

void initialize_gps(){
  GPS.begin(115200);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
}

void parse_gps_string(){
  if (GPS.parse(GPS.lastNMEA())){
    gps_latitude = GPS.latitudeDegrees;
    gps_longitude = GPS.longitudeDegrees;
    gps_altitude = GPS.altitude;
    gps_fix_quality = GPS.fixquality;
    gps_speed = GPS.speed;
    gps_angle = GPS.angle;
    gps_ant_stat = GPS.antenna;
  }
}

// Function to convert rotated global acceleration from BNO055 in ENU coordinates and then to deg/s^2 GPS reference frame
void convertBNOaccelToENUdegrees() {
  // set the acceleration vector into ENU frame from BNO055 reference
  enu_ae = bno_ay;
  enu_an = bno_ax;
  enu_az = bno_az;

  // Converting m/s^2 to deg/s^2 for East and North components
  // The Up component remains in m/s^2 as it's not affected by Earth's curvature
  enu_ae = (enu_ae / earthRadiusM) * radToDeg;
  enu_an = (enu_an / earthRadiusM) * radToDeg;
  enu_az = enu_az; // Up remains the same
}
// ********************************* End of GPS section *************************************** //

// ********************************* ADXL375 (high g accel) *************************************** //

/* Assign a unique ID to this sensor at the same time */
Adafruit_ADXL375 accel = Adafruit_ADXL375(12345);

double adxl_x = 0; double adxl_y = 0; double adxl_z = 0;

void initialize_adxl(void)
{

  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL375 ... check your connections */
    Serial.println("no ADXL375 detected");
  }
}

void get_adxl_data(void)
{
  /* Get a new sensor event */
  sensors_event_t event;
  accel.getEvent(&event);

  adxl_x = event.acceleration.x;
  adxl_y = event.acceleration.y;
  adxl_z = event.acceleration.z;
}

// ***************************** End of ADXL375 (high g accel) *************************************** //


// ********************************** This section is defining global variables and functions for the 3D position kalman filter ********************************* //

// Timestep tracking
long last_3d_prediction;
long last_3d_update;

// Kalman filter settings go here:

// Standard deviations for the acceleration values in the update step
const double sigma_ax_squared = 1;
const double sigma_ay_squared = 1;
const double sigma_az_squared = 1;
bool k3_is_nonsingular = true;

// Kalman Filter output variables
double pos_3d_x = 0.0;
double pos_3d_y = 0.0;
double pos_3d_z = 0.0;
double vel_3d_x = 0.0;
double vel_3d_y = 0.0;
double vel_3d_z = 0.0;

BLA::Matrix<6,1, double> k3_X = {0, // State vector x
                                 0, // xdot
                                 0, // y
                                 0, // ydot
                                 0, // z
                                 0};// zdot 

BLA::Matrix<6,6, double> k3_P = {0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 0}; // State covariance
BLA::Matrix<6,6,double> k3_F; // Still need to declare the global variable
BLA::Matrix<6,3,double> k3_B; // control input matrix
BLA::Matrix<3,1,double> k3_U; // control vector ax, ay, az
BLA::Matrix<6,6,double> k3_Q; // process covariance matrix (calculated for each timestep from acceleration variances)
BLA::Matrix<3,3,double> k3_R; // Measurement covariance vector
const BLA::Matrix<3,6,double> k3_H = {1, 0, 0, 0, 0, 0,
                                      0, 0, 1, 0, 0, 0,
                                      0, 0, 0, 0, 1, 0}; // measurement mapping matrix (map coordinates to position)
BLA::Matrix<3,1,double> k3_Z; // Observation vector x, y, z
BLA::Matrix<3,1,double> k3_Y; // Residual vector
BLA::Matrix<6,3,double> k3_K; // Kalman gain
BLA::Matrix<3,3,double> k3_S; // matrix to be inverted
const BLA::Matrix<6,6,double> k3_I = {1, 0, 0, 0, 0, 0,
                                      0, 1, 0, 0, 0, 0,
                                      0, 0, 1, 0, 0, 0,
                                      0, 0, 0, 1, 0, 0,
                                      0, 0, 0, 0, 1, 0,
                                      0, 0, 0, 0, 0, 1}; // Identy matrix

// State transition matrix generation function, timestep dependent (timestep might vary for implementation of real K.F)
BLA::Matrix<6,6,double> k3_generateTransitionFunction(double timestep) {
  BLA::Matrix<6,6,double> mat = {1, timestep, 0, 0, 0, 0,
                                 0, 1, 0, 0, 0, 0,
                                 0, 0, 1, timestep, 0, 0,
                                 0, 0, 0, 1, 0, 0,
                                 0, 0, 0, 0, 1, timestep,
                                 0, 0, 0, 0, 0, 1};
  return mat;
}

// This is used to incorporate the acceleration vector into the prediction step
BLA::Matrix<6,3,double> k3_generateControlInputMatrix(double timestep){
  double half_t_squared = 0.5 * timestep * timestep; // precalculate for tiny speedup
  BLA::Matrix<6,3,double> mat = {half_t_squared, 0, 0,
                                timestep, 0, 0,
                                0, half_t_squared, 0,
                                0, timestep, 0,
                                0, 0, half_t_squared,
                                0, 0, timestep};
  return mat;
}

// Process noise covariance matrix generation function, timestep dependent, acceleration data is used during the prediction, not update step
// NOTE: THIS IMPLEMENTATION ASSUMES THE ACCELERATION VECTOR COMPONENETS ARE INDEPENDENT
// This also assumes the external disturbances are set to zero for the time being, since we don't have a way to model them yet.
// In reality, due to the rotation of the acceleration vector, errors in the 
// acceleration components are not quite independent.
BLA::Matrix<6,6,double> k3_generateProcessCovariance(double timestep) {
  double dt_squared = timestep * timestep;
  double half_dt_cubed = 0.5 * dt_squared * timestep;
  double quarter_dt_fourth = 0.5 * half_dt_cubed * timestep;

  // For the calculation of each standard deviation, see the first paragraph on page 3 of "AI-IMU Dead-Reckoning by Martin Brossard"
  // At least, I hope that formula is applicable here...
  // Standard deviations are multipled together to get variances and covariances.  Entries assumed to be independent left as zero

  BLA::Matrix<6,6,double> mat = {quarter_dt_fourth * sigma_ax_squared, half_dt_cubed * sigma_ax_squared, 0, 0, 0, 0,
                                 half_dt_cubed * sigma_ax_squared, dt_squared * sigma_ax_squared, 0, 0, 0, 0,
                                 0, 0, quarter_dt_fourth * sigma_ay_squared, half_dt_cubed * sigma_ay_squared, 0, 0,
                                 0, 0, half_dt_cubed * sigma_ay_squared, dt_squared * sigma_ay_squared, 0, 0,
                                 0, 0, 0, 0, quarter_dt_fourth * sigma_az_squared, half_dt_cubed * sigma_az_squared,
                                 0, 0, 0, 0, half_dt_cubed * sigma_az_squared, dt_squared * sigma_az_squared,};
  return mat;
}

// Measurement noise covariance matrix generation function, timestep dependent
BLA::Matrix<3,3,double> k3_generateMeasurementCovariance(double timestep) {
  BLA::Matrix<3,3,double> mat = {1, 0, 0,
                                 0, 1, 0,
                                 0, 0, 1};
  return mat*timestep;
}

void k3_kalmanFilterPredict(){
  // Calculate timestep, reset for next
  double prediction_timestep = static_cast<double>(millis() - last_3d_prediction);
  last_3d_prediction = millis();

  // Set the control vector;
  k3_U(0,0) = enu_ae;
  k3_U(1,0) = enu_an;
  k3_U(2,0) = enu_az;

  // Regenerate the matricies that are timestep dependent
  k3_B = k3_generateControlInputMatrix(prediction_timestep);
  k3_F = k3_generateTransitionFunction(prediction_timestep);
  k3_Q = k3_generateProcessCovariance(prediction_timestep);

  // Kalman filter math taken from "Kalman and Baysian Filters in Python" by Roger R. Labbe

  k3_X = k3_F * k3_X + k3_B * k3_U;  // Predict the state vector and covarianace
  k3_P = k3_F * k3_P * ~k3_F + k3_Q;
}


void k3_kalmanFilterUpdate(){
  // Calculate timestep, reset for next
  double update_timestep = static_cast<double>(millis() - last_3d_update);
  last_3d_update = millis();

  // Set the measurement value to barometric altitude
  k3_Z(0,0) = altitude;

  // Regenerate the matricies that are timestep dependent
  k3_R = k3_generateMeasurementCovariance(update_timestep);

  // Kalman filter math taken from "Kalman and Baysian Filters in Python" by Roger R. Labbe
  k3_Y = k3_Z - k3_H*k3_X;  // Calculate the residual
  k3_S = k3_H * k3_P * ~k3_H + k3_R; // Calculate measurment covariance

  k3_is_nonsingular = BLA::Invert(k3_S);
  k3_K = k3_P * ~k3_H * k3_S;  // Calculate the kalman gain (matrix S has already been inverted in previous step)

  k3_X = k3_X + k3_K * k3_Y;  // Update the state vector and covariance
  k3_P = (k3_I - k3_K * k3_H) * k3_P;
}

void k3_outputStateVector() {
  // Saves the state vector to global variables, remember order is x, xdot, y, ydot, z, zdot for the state vector
  pos_3d_x = k3_X(0,0);
  pos_3d_y = k3_X(2,0);
  pos_3d_z = k3_X(4,0);
  vel_3d_x = k3_X(1,0);
  vel_3d_y = k3_X(3,0);
  vel_3d_z = k3_X(5,0);
}

// ***************************************** End of section for 3D position kalman filter prediction and update functions ************************************** //

// ***************************************** This section is defining global variables and funtions for the 1D altitude kalman filter *************************************** //
// Timestep tracking:
long last_1d_prediction;
long last_1d_update;

// Kalman Filter output variables
double altitude_est = 0.0;
double vertical_speed_est = 0.0;
bool k1_is_nonsingular = true;

// Kalman filter matrix math variables
BLA::Matrix<2,1, double> k1_X = {0,
                                 0}; // State vector: z, z_dot
BLA::Matrix<2,2, double> k1_P = {500, 0,
                                 0, 49}; // State covariance
BLA::Matrix<2,2,double> k1_F; // Still need to declare the global variable
BLA::Matrix<2,1,double> k1_B; // control input matrix
BLA::Matrix<1,1,double> k1_U; // control vector
BLA::Matrix<2,2,double> k1_Q; // Process noise covariance matrix
BLA::Matrix<1,1,double> k1_R; // Measurement covariance
BLA::Matrix<1,2,double> k1_H = {1, 0}; // Measurement function
BLA::Matrix<1,1,double> k1_Z = {0}; // Measurement vector
BLA::Matrix<1,1,double> k1_Y = {0}; // Residual vector
BLA::Matrix<2,1,double> k1_K = {0, 0}; // Kalman gain
BLA::Matrix<1,1,double> k1_S = {0}; // Matrix to be inverted for kalman gain calculation
const BLA::Matrix<2,2,double> k1_I = {1., 0.,
                                      0., 1.}; // Identy matrix

// State transition matrix generation function, timestep dependent (timestep might vary for implementation of real K.F)
BLA::Matrix<2,2,double> k1_generateTransitionFunction(double timestep) {
  BLA::Matrix<2,2,double> mat = {1, timestep,
                                 0, 1};
  return mat;
}

// Control input matrix (timestep dependent), links acceleration into the prediction step
BLA::Matrix<2,1,double> k1_generateControlInputMatrix(double timestep){
  BLA::Matrix<2,1,double> mat = {0.5 * timestep * timestep,
                                timestep};
  return mat;
}

// Process noise covariance matrix generation function, timestep dependent
BLA::Matrix<2,2,double> k1_generateProcessCovariance(double timestep) {
  BLA::Matrix<2,2,double> mat = {0.1, 0.15,
                                 0.15, 0.2}; //{0.588, 1.175,1.175, 2.35};
  return mat*timestep;
}

// Measurement noise covariance matrix generation function, timestep dependent
BLA::Matrix<1,1,double> k1_generateMeasurementCovariance(double timestep) {
  BLA::Matrix<1,1,double> mat = {10000};
  return mat*timestep;
}

void k1_kalmanFilterPredict(){
  // Calculate timestep, reset for next
  double prediction_timestep = static_cast<double>(millis() - last_1d_prediction);
  last_1d_prediction = millis();

  k1_U(0,0) = bno_az; // set the control vector to the z axis acceleration from rotated bno vector

  // Regenerate the matricies that are timestep dependent
  k1_B = k1_generateControlInputMatrix(prediction_timestep);
  k1_F = k1_generateTransitionFunction(prediction_timestep);
  k1_Q = k1_generateProcessCovariance(prediction_timestep);

  // Kalman filter math taken from "Kalman and Baysian Filters in Python" by Roger R. Labbe
  k1_X = k1_F * k1_X + k1_B * k1_U;  // Predict the state vector and covarianace
  k1_P = k1_F * k1_P * ~k1_F + k1_Q;
}


void k1_kalmanFilterUpdate(){
    // Calculate timestep, reset for next
  double update_timestep = static_cast<double>(millis() - last_1d_update);
  last_1d_update = millis();

  // Set the measurement value to the barometric altimeter value
  k1_Z(0,0) = altitude;

  // Regenerate the matricies that are timestep dependent
  k1_R = k1_generateMeasurementCovariance(update_timestep);

  // Kalman filter math taken from "Kalman and Baysian Filters in Python" by Roger R. Labbe
  k1_Y = k1_Z - k1_H*k1_X;  // Calculate the residual
  k1_S = k1_H * k1_P * ~k1_H + k1_R; // Calculate measurment covariance

  bool is_nonsingular = BLA::Invert(k1_S);
  k1_K = k1_P * ~k1_H * k1_S;  // Calculate the kalman gain (matrix S has already been inverted in previous step)

  k1_X = k1_X + k1_K * k1_Y;  // Update the state vector and covariance
  k1_P = (k1_I - k1_K * k1_H) * k1_P;
}

void k1_outputStateVector() {
  // Saves the state vector to global variables
  altitude_est = k1_X(0,0);
  vertical_speed_est = k1_X(1,0);
}

// *********************** End of section for altitude kalman filter prediction and update functions ******************* //


// ************************** Data logging to SD card section ************************** //

const int chipSelect = BUILTIN_SDCARD;
String dataString;


// A function to append matrix headers to the string.
template <typename blaMatrix>
void append_matrix_header(blaMatrix mat, const char* matrix_name) {
  for (int i=0; i < mat.Rows; i++) {
    for (int j=0; j < mat.Cols; j++) {
      dataString += String(matrix_name);
      dataString += String(i);
      dataString += String(j);
      dataString += String(", ");
    }
  }
}

// A function to appends matrix data to the string data.
template <typename blaMatrix>
void append_matrix(blaMatrix mat) {
  for (int i=0; i < mat.Rows; i++) {
    for (int j=0; j < mat.Cols; j++) {
      dataString += String(mat(i,j));
      dataString += String(", ");
    }
  }
}


void write_sd_header(){
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
  }
  Serial.println("card initialized.");

  // add data labels to the string
  dataString += String("a11");
  dataString += ", ";
  dataString += String("a12");
  dataString += ", ";
  dataString += String("a13");
  dataString += ", ";
  dataString += String("bno_ax");
  dataString += ", ";
  dataString += String("bno_ay");
  dataString += ", ";
  dataString += String("bno_az");
  dataString += ", ";
  dataString += String("pres");
  dataString += ", ";
  dataString += String("altitude");
  dataString += ", ";
  dataString += String("k1_is_nonsingular");
  dataString += ", ";
  dataString += String("altitude_est");
  dataString += ", ";
  dataString += String("vertical_speed_est");
  dataString += ", ";
  dataString += String("recov_trigger");
  dataString += ", ";
  dataString += String("k3_is_nonsingular");
  dataString += ", ";
  dataString += String("pos_3d_x");
  dataString += ", ";
  dataString += String("pos_3d_y");
  dataString += ", ";
  dataString += String("pos_3d_z");
  dataString += ", ";
  dataString += String("vel_3d_x");
  dataString += ", ";
  dataString += String("vel_3d_y");
  dataString += ", ";
  dataString += String("vel_3d_z");
  dataString += ", ";
  dataString += String("gps_fix_quality");
  dataString += ", ";
  dataString += String("gps_latitude");
  dataString += ", ";
  dataString += String("gps_lat_hem");
  dataString += ", ";
  dataString += String("gps_longitude");
  dataString += ", ";
  dataString += String("gps_long_hem");
  dataString += ", ";
  dataString += String("gps_speed");
  dataString += ", ";
  dataString += String("gps_angle");
  dataString += ", ";
  dataString += String("gps_altitude");
  dataString += ", ";
  dataString += String("gps_ant_stat");
  dataString += ", ";
  dataString += String("q_x");
  dataString += ", ";
  dataString += String("q_y");
  dataString += ", ";
  dataString += String("q_z");
  dataString += ", ";
  dataString += String("q_w");
  dataString += ", ";
  dataString += String("adxl_x");
  dataString += ", ";
  dataString += String("adxl_y");
  dataString += ", ";
  dataString += String("adxl_z");
  dataString += ", ";
  append_matrix_header(k1_X, "k1_X");
  append_matrix_header(k1_P, "k1_P");
  append_matrix_header(k1_K, "k1_K");
  append_matrix_header(k3_X, "k3_X");
  append_matrix_header(k3_P, "k3_P");
  append_matrix_header(k3_K, "k3_K");

  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  dataFile.println(dataString);
  dataFile.close();
  dataString = "";
}


void write_sd_data(){  
  // make a string for assembling the data to log:
  String dataString = "";

  // add data to the string
  dataString += String(a11);
  dataString += ", ";
  dataString += String(a12);
  dataString += ", ";
  dataString += String(a13);
  dataString += ", ";
  dataString += String(bno_ax);
  dataString += ", ";
  dataString += String(bno_ay);
  dataString += ", ";
  dataString += String(bno_az);
  dataString += ", ";
  dataString += String(pres);
  dataString += ", ";
  dataString += String(altitude);
  dataString += ", ";
  dataString += String(k1_is_nonsingular);
  dataString += ", ";
  dataString += String(altitude_est);
  dataString += ", ";
  dataString += String(vertical_speed_est);
  dataString += ", ";
  dataString += String(recov_trigger);
  dataString += ", ";
  dataString += String(k3_is_nonsingular);
  dataString += ", ";
  dataString += String(pos_3d_x);
  dataString += ", ";
  dataString += String(pos_3d_y);
  dataString += ", ";
  dataString += String(pos_3d_z);
  dataString += ", ";
  dataString += String(vel_3d_x);
  dataString += ", ";
  dataString += String(vel_3d_y);
  dataString += ", ";
  dataString += String(vel_3d_z);
  dataString += ", ";
  dataString += String(gps_fix_quality);
  dataString += ", ";
  dataString += String(gps_latitude);
  dataString += ", ";
  //dataString += String(gps_lat_hem);
  //dataString += ", ";
  dataString += String(gps_longitude);
  dataString += ", ";
  //dataString += String(gps_long_hem);
  //dataString += ", ";
  dataString += String(gps_speed);
  dataString += ", ";
  dataString += String(gps_angle);
  dataString += ", ";
  dataString += String(gps_altitude);
  dataString += ", ";
  dataString += String(gps_ant_stat);
  dataString += ", ";
  dataString += String(q_x);
  dataString += ", ";
  dataString += String(q_y);
  dataString += ", ";
  dataString += String(q_z);
  dataString += ", ";
  dataString += String(q_w);
  dataString += ", ";
  dataString += String(adxl_x);
  dataString += ", ";
  dataString += String(adxl_y);
  dataString += ", ";
  dataString += String(adxl_z);
  dataString += ", ";
  append_matrix(k1_X);
  append_matrix(k1_P);
  append_matrix(k1_K);
  append_matrix(k3_X);
  append_matrix(k3_P);
  append_matrix(k3_K);

  // open the file.
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    
    // print to the serial port too:
    Serial.println(dataString);
    dataString = "";
  } else {
    // if the file isn't open, pop up an error:
    Serial.println("error opening datalog.txt");
  }
}

// ************************** End of Data logging to SD card section ************************** //



//////////////////////////////////////////////////////////////////////////////////////////////////////
// ******************************** Setup and Loop functions go here ****************************** //
//////////////////////////////////////////////////////////////////////////////////////////////////////

#define SOLENOID_DRIVER_PIN 32

bool recovery_conditions_met(){
  if (vertical_speed_est < 0.0){  // If rocket is estimated to be no longer going up, it is considered to be at apogee.
    recov_trigger = true;
  } 
  return recov_trigger;
}

void setup() {
  Serial.begin(115200);
  pinMode(SOLENOID_DRIVER_PIN, OUTPUT);
  initialize_barometric_altimeter();
  initialize_bno055();
  initialize_gps();
  initialize_adxl();
  write_sd_header();
}

void loop() {
  // put your main code here, to run repeatedly:
  read_barometric_altimeter_data();
  get_bno055_data_rotated();
  convertBNOaccelToENUdegrees();
  get_adxl_data();

  k1_kalmanFilterPredict();
  k3_kalmanFilterPredict();

  if (altitude < 30000){
    k1_kalmanFilterUpdate();
  }

  if (GPS.newNMEAreceived()) {
    parse_gps_string();
    k3_kalmanFilterUpdate();
  }

  if (recovery_conditions_met()){
    digitalWrite(SOLENOID_DRIVER_PIN, HIGH);
  }

  write_sd_data();
}  
