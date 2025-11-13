/*
 * Copyright 1996-2024 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:   Autonoumous vehicle controller example
 */

#include <webots/camera.h>
#include <webots/device.h>
#include <webots/display.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/keyboard.h>
#include <webots/lidar.h>
#include <webots/radar.h>
#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/touch_sensor.h>
#include <webots/vehicle/driver.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

// to be used as array indices
enum { X, Y, Z };

#define TIME_STEP 50
#define UNKNOWN 99999.99

// Line following PID
#define KP 0.25
#define KI 0.006
#define KD 2

bool PID_need_reset = false;

// Size of the yellow line angle filter
#define FILTER_SIZE 3

// enable various 'features'
bool enable_collision_avoidance = false;
bool enable_display = false;
bool has_gps = false;
bool has_camera = false;
bool data_collection_enabled = true;  // Enable data collection

// Data collection file
FILE *csv_file = NULL;
int step_count = 0;

// camera
WbDeviceTag camera;
int camera_width = -1;
int camera_height = -1;
double camera_fov = -1.0;

// SICK laser / Lidar
WbDeviceTag sick;
WbDeviceTag roof_lidar;
int sick_width = -1;
double sick_range = -1.0;
double sick_fov = -1.0;
int roof_lidar_width = -1;
int roof_lidar_layers = -1;
double roof_lidar_range = -1.0;
double roof_lidar_fov = -1.0;

// speedometer
WbDeviceTag display;
int display_width = 0;
int display_height = 0;
WbImageRef speedometer_image = NULL;

// GPS
WbDeviceTag gps;
double gps_coords[3] = {0.0, 0.0, 0.0};
double gps_speed = 0.0;

// Radar
WbDeviceTag front_radar;
int radar_targets_count = 0;

// Touch Sensors
WbDeviceTag touch_front_center;
WbDeviceTag touch_front_left;
WbDeviceTag touch_front_right;
WbDeviceTag touch_rear_center;
WbDeviceTag touch_rear_left;
WbDeviceTag touch_rear_right;

// Gyro
WbDeviceTag gyro;
double gyro_values[3] = {0.0, 0.0, 0.0};

// Inertial Unit
WbDeviceTag imu;
double imu_roll_pitch_yaw[3] = {0.0, 0.0, 0.0};
double imu_quaternion[4] = {0.0, 0.0, 0.0, 1.0};

// misc variables
double speed = 0.0;
double steering_angle = 0.0;
int manual_steering = 0;
bool autodrive = true;

// Starting position for reset
#define START_POS_X -45.0
#define START_POS_Y 45.88
#define START_POS_Z 0.4
#define START_ROT_X 0.0
#define START_ROT_Y 0.0
#define START_ROT_Z 1.0
#define START_ROT_ANGLE 3.14159

// Collision reset delay (in seconds)
#define COLLISION_RESET_DELAY 5.0

void print_help() {
  printf("You can drive this car!\n");
  printf("Select the 3D window and then use the cursor keys to:\n");
  printf("[LEFT]/[RIGHT] - steer\n");
  printf("[UP]/[DOWN] - accelerate/slow down\n");
}

void set_autodrive(bool onoff) {
  if (autodrive == onoff)
    return;
  autodrive = onoff;
  switch (autodrive) {
    case false:
      printf("switching to manual drive...\n");
      printf("hit [A] to return to auto-drive.\n");
      break;
    case true:
      if (has_camera)
        printf("switching to auto-drive...\n");
      else
        printf("impossible to switch auto-drive on without camera...\n");
      break;
  }
}

// set target speed
void set_speed(double kmh) {
  // max speed
  if (kmh > 250.0)
    kmh = 250.0;

  speed = kmh;

  printf("setting speed to %g km/h\n", kmh);
  wbu_driver_set_cruising_speed(kmh);
}

// positive: turn right, negative: turn left
void set_steering_angle(double wheel_angle) {
  // limit the difference with previous steering_angle
  if (wheel_angle - steering_angle > 0.1)
    wheel_angle = steering_angle + 0.1;
  if (wheel_angle - steering_angle < -0.1)
    wheel_angle = steering_angle - 0.1;
  steering_angle = wheel_angle;
  // limit range of the steering angle
  if (wheel_angle > 0.5)
    wheel_angle = 0.5;
  else if (wheel_angle < -0.5)
    wheel_angle = -0.5;
  wbu_driver_set_steering_angle(wheel_angle);
}

void change_manual_steer_angle(int inc) {
  set_autodrive(false);

  double new_manual_steering = manual_steering + inc;
  if (new_manual_steering <= 25.0 && new_manual_steering >= -25.0) {
    manual_steering = new_manual_steering;
    set_steering_angle(manual_steering * 0.02);
  }

  if (manual_steering == 0)
    printf("going straight\n");
  else
    printf("turning %.2f rad (%s)\n", steering_angle, steering_angle < 0 ? "left" : "right");
}

void check_keyboard() {
  int key = wb_keyboard_get_key();
  switch (key) {
    case WB_KEYBOARD_UP:
      set_speed(speed + 5.0);
      break;
    case WB_KEYBOARD_DOWN:
      set_speed(speed - 5.0);
      break;
    case WB_KEYBOARD_RIGHT:
      change_manual_steer_angle(+1);
      break;
    case WB_KEYBOARD_LEFT:
      change_manual_steer_angle(-1);
      break;
    case 'A':
      set_autodrive(true);
      break;
  }
}

// compute rgb difference
int color_diff(const unsigned char a[3], const unsigned char b[3]) {
  int i, diff = 0;
  for (i = 0; i < 3; i++) {
    int d = a[i] - b[i];
    diff += d > 0 ? d : -d;
  }
  return diff;
}

// returns approximate angle of yellow road line
// or UNKNOWN if no pixel of yellow line visible
double process_camera_image(const unsigned char *image) {
  int num_pixels = camera_height * camera_width;  // number of pixels in the image
  const unsigned char REF[3] = {95, 187, 203};    // road yellow (BGR format)
  int sumx = 0;                                   // summed x position of pixels
  int pixel_count = 0;                            // yellow pixels count

  const unsigned char *pixel = image;
  int x;
  for (x = 0; x < num_pixels; x++, pixel += 4) {
    if (color_diff(pixel, REF) < 30) {
      sumx += x % camera_width;
      pixel_count++;  // count yellow pixels
    }
  }

  // if no pixels was detected...
  if (pixel_count == 0)
    return UNKNOWN;

  return ((double)sumx / pixel_count / camera_width - 0.5) * camera_fov;
}

// filter angle of the yellow line (simple average)
double filter_angle(double new_value) {
  static bool first_call = true;
  static double old_value[FILTER_SIZE];
  int i;

  if (first_call || new_value == UNKNOWN) {  // reset all the old values to 0.0
    first_call = false;
    for (i = 0; i < FILTER_SIZE; ++i)
      old_value[i] = 0.0;
  } else {  // shift old values
    for (i = 0; i < FILTER_SIZE - 1; ++i)
      old_value[i] = old_value[i + 1];
  }

  if (new_value == UNKNOWN)
    return UNKNOWN;
  else {
    old_value[FILTER_SIZE - 1] = new_value;
    double sum = 0.0;
    for (i = 0; i < FILTER_SIZE; ++i)
      sum += old_value[i];
    return (double)sum / FILTER_SIZE;
  }
}

// returns approximate angle of obstacle
// or UNKNOWN if no obstacle was detected
double process_sick_data(const float *sick_data, double *obstacle_dist) {
  const int HALF_AREA = 20;  // check 20 degrees wide middle area
  int sumx = 0;
  int collision_count = 0;
  int x;
  *obstacle_dist = 0.0;
  for (x = sick_width / 2 - HALF_AREA; x < sick_width / 2 + HALF_AREA; x++) {
    float range = sick_data[x];
    if (range < 20.0) {
      sumx += x;
      collision_count++;
      *obstacle_dist += range;
    }
  }

  // if no obstacle was detected...
  if (collision_count == 0)
    return UNKNOWN;

  *obstacle_dist = *obstacle_dist / collision_count;
  return ((double)sumx / collision_count / sick_width - 0.5) * sick_fov;
}

void update_display() {
  const double NEEDLE_LENGTH = 50.0;

  // display background
  wb_display_image_paste(display, speedometer_image, 0, 0, false);

  // draw speedometer needle
  double current_speed = wbu_driver_get_current_speed();
  if (isnan(current_speed))
    current_speed = 0.0;
  double alpha = current_speed / 260.0 * 3.72 - 0.27;
  int x = -NEEDLE_LENGTH * cos(alpha);
  int y = -NEEDLE_LENGTH * sin(alpha);
  wb_display_draw_line(display, 100, 95, 100 + x, 95 + y);

  // draw text
  char txt[64];
  sprintf(txt, "GPS coords: %.1f %.1f", gps_coords[X], gps_coords[Z]);
  wb_display_draw_text(display, txt, 10, 130);
  sprintf(txt, "GPS speed:  %.1f", gps_speed);
  wb_display_draw_text(display, txt, 10, 140);
}

void compute_gps_speed() {
  const double *coords = wb_gps_get_values(gps);
  const double speed_ms = wb_gps_get_speed(gps);
  // store into global variables
  gps_speed = speed_ms * 3.6;  // convert from m/s to km/h
  memcpy(gps_coords, coords, sizeof(gps_coords));
}

double applyPID(double yellow_line_angle) {
  static double oldValue = 0.0;
  static double integral = 0.0;

  if (PID_need_reset) {
    oldValue = yellow_line_angle;
    integral = 0.0;
    PID_need_reset = false;
  }

  // anti-windup mechanism
  if (signbit(yellow_line_angle) != signbit(oldValue))
    integral = 0.0;

  double diff = yellow_line_angle - oldValue;

  // limit integral
  if (integral < 30 && integral > -30)
    integral += yellow_line_angle;

  oldValue = yellow_line_angle;
  return KP * yellow_line_angle + KI * integral + KD * diff;
}

// Reset vehicle to starting position
void reset_vehicle_position() {
  if (!wb_robot_get_supervisor()) {
    printf("Warning: Cannot reset position - controller is not a Supervisor\n");
    return;
  }
  
  WbNodeRef self_node = wb_supervisor_node_get_self();
  if (!self_node) {
    printf("Warning: Could not get self node for reset\n");
    return;
  }
  
  WbFieldRef translation_field = wb_supervisor_node_get_field(self_node, "translation");
  WbFieldRef rotation_field = wb_supervisor_node_get_field(self_node, "rotation");
  
  if (translation_field) {
    const double translation[3] = {START_POS_X, START_POS_Y, START_POS_Z};
    wb_supervisor_field_set_sf_vec3f(translation_field, translation);
  }
  
  if (rotation_field) {
    const double rotation[4] = {START_ROT_X, START_ROT_Y, START_ROT_Z, START_ROT_ANGLE};
    wb_supervisor_field_set_sf_rotation(rotation_field, rotation);
  }
  
  // Reset velocity to zero
  const double zero_velocity[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  wb_supervisor_node_set_velocity(self_node, zero_velocity);
  
  // Reset PID state
  PID_need_reset = true;
  
  printf("Vehicle reset to starting position\n");
}

// Initialize CSV file and write header
void init_csv_file() {
  if (!data_collection_enabled)
    return;

  char filename[256];
  time_t rawtime;
  struct tm *timeinfo;
  time(&rawtime);
  timeinfo = localtime(&rawtime);
  strftime(filename, sizeof(filename), "sensor_data_%Y%m%d_%H%M%S.csv", timeinfo);

  csv_file = fopen(filename, "w");
  if (!csv_file) {
    printf("Error: Could not create CSV file %s\n", filename);
    data_collection_enabled = false;
    return;
  }

  printf("Data collection enabled. Writing to: %s\n", filename);

  // Write CSV header
  fprintf(csv_file, "step,time,");
  fprintf(csv_file, "gps_x,gps_y,gps_z,gps_speed,");
  fprintf(csv_file, "gyro_x,gyro_y,gyro_z,");
  fprintf(csv_file, "imu_roll,imu_pitch,imu_yaw,imu_quat_x,imu_quat_y,imu_quat_z,imu_quat_w,");
  fprintf(csv_file, "radar_targets_count,");
  fprintf(csv_file, "roof_lidar_min_dist,roof_lidar_max_dist,roof_lidar_mean_dist,");
  fprintf(csv_file, "camera_red,camera_green,camera_blue,camera_gray,camera_recognition_objects,");
  fprintf(csv_file, "driver_speed,driver_steering_angle,driver_brake_intensity,");
  fprintf(csv_file, "collision_detected\n");
}

// Collect and write sensor data to CSV
void collect_and_write_sensor_data(double sim_time) {
  if (!data_collection_enabled || !csv_file)
    return;

  // GPS data
  double gps_x = 0.0, gps_y = 0.0, gps_z = 0.0, gps_speed_ms = 0.0;
  if (has_gps && gps) {
    const double *coords = wb_gps_get_values(gps);
    gps_x = coords[X];
    gps_y = coords[Y];
    gps_z = coords[Z];
    gps_speed_ms = wb_gps_get_speed(gps);
  }

  // Gyro data
  double gyro_x = 0.0, gyro_y = 0.0, gyro_z = 0.0;
  if (gyro) {
    const double *gyro_vals = wb_gyro_get_values(gyro);
    gyro_x = gyro_vals[X];
    gyro_y = gyro_vals[Y];
    gyro_z = gyro_vals[Z];
  }

  // IMU data
  double imu_roll = 0.0, imu_pitch = 0.0, imu_yaw = 0.0;
  double imu_qx = 0.0, imu_qy = 0.0, imu_qz = 0.0, imu_qw = 1.0;
  if (imu) {
    const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(imu);
    imu_roll = rpy[0];
    imu_pitch = rpy[1];
    imu_yaw = rpy[2];
    const double *quat = wb_inertial_unit_get_quaternion(imu);
    imu_qx = quat[0];
    imu_qy = quat[1];
    imu_qz = quat[2];
    imu_qw = quat[3];
  }

  // Radar data
  int radar_count = 0;
  if (front_radar) {
    radar_count = wb_radar_get_number_of_targets(front_radar);
  }

  // Touch sensor data - check if any sensor detected collision
  // Bumper TouchSensors return 1.0 when collision detected, 0.0 otherwise
  int collision_detected = 0;
  const char *touch_sensor_names[] = {
    "touch_front_center", "touch_front_left", "touch_front_right",
    "touch_rear_center", "touch_rear_left", "touch_rear_right"
  };
  WbDeviceTag touch_sensors[] = {
    touch_front_center, touch_front_left, touch_front_right,
    touch_rear_center, touch_rear_left, touch_rear_right
  };

  static bool collision_reset_pending = false;
  static double collision_start_time = -1.0;
  
  for (int i = 0; i < 6; i++) {
    if (touch_sensors[i]) {
      double value = wb_touch_sensor_get_value(touch_sensors[i]);
      if (value > 0.5) {  // safer than >=1.0
        printf("COLLISION DETECTED: %s = %.6f\n", touch_sensor_names[i], value);
        collision_detected = 1;
        if (!collision_reset_pending) {
          // Start the collision timer
          collision_reset_pending = true;
          collision_start_time = sim_time;
          printf("Collision detected, will reset in %.1f seconds...\n", COLLISION_RESET_DELAY);
        }
      }
    }
  }
  
  // Check if delay has passed and reset if needed
  if (collision_reset_pending && collision_start_time >= 0.0) {
    double elapsed = sim_time - collision_start_time;
    if (elapsed >= COLLISION_RESET_DELAY) {
      reset_vehicle_position();
      collision_reset_pending = false;
      collision_start_time = -1.0;
    }
  }
  
  // Reset the flag when no collision is detected
  if (!collision_detected && !collision_reset_pending) {
    collision_start_time = -1.0;
  }

  // Roof Lidar data (compute statistics)
  double lidar_min = 0.0, lidar_max = 0.0, lidar_mean = 0.0;
  if (roof_lidar) {
    const float *lidar_data = wb_lidar_get_range_image(roof_lidar);
    if (lidar_data) {
      int total_points = roof_lidar_width * roof_lidar_layers;
      lidar_min = INFINITY;
      lidar_max = 0.0;
      double sum = 0.0;
      int valid_points = 0;
      for (int i = 0; i < total_points; i++) {
        if (!isnan(lidar_data[i]) && !isinf(lidar_data[i]) && lidar_data[i] > 0 && lidar_data[i] < roof_lidar_range) {
          if (valid_points == 0) {
            // Initialize min/max with first valid point
            lidar_min = lidar_max = lidar_data[i];
          } else {
            if (lidar_data[i] < lidar_min)
              lidar_min = lidar_data[i];
            if (lidar_data[i] > lidar_max)
              lidar_max = lidar_data[i];
          }
          sum += lidar_data[i];
          valid_points++;
        }
      }
      if (valid_points > 0) {
        lidar_mean = sum / valid_points;
      } else {
        // No valid points found - set to 0.0 to indicate no detection
        lidar_min = 0.0;
        lidar_max = 0.0;
      }
    }
  }

  // Camera data - sample center pixel
  unsigned char camera_red = 0, camera_green = 0, camera_blue = 0, camera_gray = 0;
  int camera_recognition_count = 0;
  
  if (has_camera && camera) {
    const unsigned char *camera_image = wb_camera_get_image(camera);
    if (camera_image && camera_width > 0 && camera_height > 0) {
      // Sample center pixel
      int center_x = camera_width / 2;
      int center_y = camera_height / 2;
      camera_red = wb_camera_image_get_red(camera_image, camera_width, center_x, center_y);
      camera_green = wb_camera_image_get_green(camera_image, camera_width, center_x, center_y);
      camera_blue = wb_camera_image_get_blue(camera_image, camera_width, center_x, center_y);
      camera_gray = wb_camera_image_get_gray(camera_image, camera_width, center_x, center_y);
    }
    
    // Get recognition object count
    camera_recognition_count = wb_camera_recognition_get_number_of_objects(camera);
  }

  // Driver data
  double driver_speed = wbu_driver_get_current_speed();
  double driver_steering = wbu_driver_get_steering_angle();
  double driver_brake = wbu_driver_get_brake_intensity();

  // Write CSV row
  fprintf(csv_file, "%d,%.6f,", step_count, sim_time);
  fprintf(csv_file, "%.6f,%.6f,%.6f,%.6f,", gps_x, gps_y, gps_z, gps_speed_ms);
  fprintf(csv_file, "%.6f,%.6f,%.6f,", gyro_x, gyro_y, gyro_z);
  fprintf(csv_file, "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,", imu_roll, imu_pitch, imu_yaw, imu_qx, imu_qy, imu_qz, imu_qw);
  fprintf(csv_file, "%d,", radar_count);
  fprintf(csv_file, "%.6f,%.6f,%.6f,", lidar_min, lidar_max, lidar_mean);
  fprintf(csv_file, "%d,%d,%d,%d,%d,", camera_red, camera_green, camera_blue, camera_gray, camera_recognition_count);
  fprintf(csv_file, "%.6f,%.6f,%.6f,", driver_speed, driver_steering, driver_brake);
  fprintf(csv_file, "%d\n", collision_detected);

  step_count++;
}

int main(int argc, char **argv) {
  wbu_driver_init();

  // Initialize data collection
  init_csv_file();

  // check if there is a SICK and a display
  int j = 0;
  for (j = 0; j < wb_robot_get_number_of_devices(); ++j) {
    WbDeviceTag device = wb_robot_get_device_by_index(j);
    const char *name = wb_device_get_name(device);
    if (strcmp(name, "Sick LMS 291") == 0)
      enable_collision_avoidance = true;
    else if (strcmp(name, "display") == 0)
      enable_display = true;
    else if (strcmp(name, "gps") == 0)
      has_gps = true;
    else if (strcmp(name, "camera") == 0)
      has_camera = true;
  }

  // camera device
  if (has_camera) {
    camera = wb_robot_get_device("camera");
    if (camera) {
      wb_camera_enable(camera, TIME_STEP);
      camera_width = wb_camera_get_width(camera);
      camera_height = wb_camera_get_height(camera);
      camera_fov = wb_camera_get_fov(camera);
      // Enable camera recognition
      wb_camera_recognition_enable(camera, TIME_STEP);
      printf("Camera initialized: %dx%d, FOV=%.2f (recognition enabled)\n", camera_width, camera_height, camera_fov);
    } else {
      printf("Warning: Camera device not found\n");
    }
  } else {
    printf("Camera not available\n");
  }

  // SICK sensor
  if (enable_collision_avoidance) {
    sick = wb_robot_get_device("Sick LMS 291");
    if (sick) {
      wb_lidar_enable(sick, TIME_STEP);
      sick_width = wb_lidar_get_horizontal_resolution(sick);
      sick_range = wb_lidar_get_max_range(sick);
      sick_fov = wb_lidar_get_fov(sick);
      printf("SICK Lidar initialized: width=%d, range=%.2f, FOV=%.2f\n", sick_width, sick_range, sick_fov);
    } else {
      printf("Warning: SICK Lidar device not found\n");
    }
  } else {
    printf("SICK Lidar not available\n");
  }

  // initialize gps
  if (has_gps) {
    gps = wb_robot_get_device("gps");
    if (gps) {
      wb_gps_enable(gps, TIME_STEP);
      printf("GPS initialized\n");
    } else {
      printf("Warning: GPS device not found\n");
    }
  } else {
    printf("GPS not available\n");
  }

  // initialize display (speedometer)
  if (enable_display) {
    display = wb_robot_get_device("display");
    if (display) {
      speedometer_image = wb_display_image_load(display, "speedometer.png");
      printf("Display initialized\n");
    } else {
      printf("Warning: Display device not found\n");
    }
  } else {
    printf("Display not available\n");
  }

  // Initialize Radar
  front_radar = wb_robot_get_device("front_radar");
  if (front_radar) {
    wb_radar_enable(front_radar, TIME_STEP);
    printf("Radar initialized\n");
  } else {
    printf("Radar not available\n");
  }

  // Initialize Touch Sensors
  touch_front_center = wb_robot_get_device("touch_front_center");
  touch_front_left = wb_robot_get_device("touch_front_left");
  touch_front_right = wb_robot_get_device("touch_front_right");
  touch_rear_center = wb_robot_get_device("touch_rear_center");
  touch_rear_left = wb_robot_get_device("touch_rear_left");
  touch_rear_right = wb_robot_get_device("touch_rear_right");

  // Enable sensors
  if (touch_front_center)
    wb_touch_sensor_enable(touch_front_center, TIME_STEP);
  if (touch_front_left)
    wb_touch_sensor_enable(touch_front_left, TIME_STEP);
  if (touch_front_right)
    wb_touch_sensor_enable(touch_front_right, TIME_STEP);
  if (touch_rear_center)
    wb_touch_sensor_enable(touch_rear_center, TIME_STEP);
  if (touch_rear_left)
    wb_touch_sensor_enable(touch_rear_left, TIME_STEP);
  if (touch_rear_right)
    wb_touch_sensor_enable(touch_rear_right, TIME_STEP);

  // Log initialized sensors
  const char *touch_sensor_names[] = {
    "touch_front_center", "touch_front_left", "touch_front_right",
    "touch_rear_center", "touch_rear_left", "touch_rear_right"
  };
  WbDeviceTag touch_sensors[] = {
    touch_front_center, touch_front_left, touch_front_right,
    touch_rear_center, touch_rear_left, touch_rear_right
  };
  int touch_sensor_count = 0;
  for (int i = 0; i < 6; i++) {
    if (touch_sensors[i]) {
      printf("Touch sensor initialized: %s\n", touch_sensor_names[i]);
      touch_sensor_count++;
    }
  }
  if (touch_sensor_count == 0)
    printf("Warning: No touch sensors found!\n");

  // Initialize Gyro
  gyro = wb_robot_get_device("gyro");
  if (gyro) {
    wb_gyro_enable(gyro, TIME_STEP);
    printf("Gyro initialized\n");
  } else {
    printf("Gyro not available\n");
  }

  // Initialize Inertial Unit (IMU)
  imu = wb_robot_get_device("imu");
  if (imu) {
    wb_inertial_unit_enable(imu, TIME_STEP);
    printf("IMU initialized\n");
  } else {
    printf("IMU not available\n");
  }

  // Initialize Roof Lidar
  roof_lidar = wb_robot_get_device("roof_lidar");
  if (roof_lidar) {
    wb_lidar_enable(roof_lidar, TIME_STEP);
    roof_lidar_width = wb_lidar_get_horizontal_resolution(roof_lidar);
    roof_lidar_layers = wb_lidar_get_number_of_layers(roof_lidar);
    roof_lidar_range = wb_lidar_get_max_range(roof_lidar);
    roof_lidar_fov = wb_lidar_get_fov(roof_lidar);
    printf("Roof Lidar initialized: %dx%d layers, range=%.2f, fov=%.2f\n",
           roof_lidar_width, roof_lidar_layers, roof_lidar_range, roof_lidar_fov);
  } else {
    printf("Roof Lidar not available\n");
  }

  // start engine
  if (has_camera)
    set_speed(50.0);  // km/h
  wbu_driver_set_hazard_flashers(true);
  wbu_driver_set_dipped_beams(true);
  wbu_driver_set_antifog_lights(true);
  wbu_driver_set_wiper_mode(SLOW);

  // main loop
  while (wbu_driver_step() != -1) {
    // get user input
    // check_keyboard();
    static int i = 0;

    // updates sensors only every TIME_STEP milliseconds
    if (i % (int)(TIME_STEP / wb_robot_get_basic_time_step()) == 0) {
      // read sensors
      const unsigned char *camera_image = NULL;
      const float *sick_data = NULL;
      if (has_camera)
        camera_image = wb_camera_get_image(camera);
      if (enable_collision_avoidance)
        sick_data = wb_lidar_get_range_image(sick);

      if (autodrive && has_camera) {
        double yellow_line_angle = filter_angle(process_camera_image(camera_image));
        double obstacle_dist;
        double obstacle_angle;
        if (enable_collision_avoidance)
          obstacle_angle = process_sick_data(sick_data, &obstacle_dist);
        else {
          obstacle_angle = UNKNOWN;
          obstacle_dist = 0;
        }

        // avoid obstacles and follow yellow line
        if (enable_collision_avoidance && obstacle_angle != UNKNOWN) {
          // an obstacle has been detected
          wbu_driver_set_brake_intensity(0.0);
          // compute the steering angle required to avoid the obstacle
          double obstacle_steering = steering_angle;
          if (obstacle_angle > 0.0 && obstacle_angle < 0.4)
            obstacle_steering = steering_angle + (obstacle_angle - 0.25) / obstacle_dist;
          else if (obstacle_angle > -0.4)
            obstacle_steering = steering_angle + (obstacle_angle + 0.25) / obstacle_dist;
          double steer = steering_angle;
          // if we see the line we determine the best steering angle to both avoid obstacle and follow the line
          if (yellow_line_angle != UNKNOWN) {
            const double line_following_steering = applyPID(yellow_line_angle);
            if (obstacle_steering > 0 && line_following_steering > 0)
              steer = obstacle_steering > line_following_steering ? obstacle_steering : line_following_steering;
            else if (obstacle_steering < 0 && line_following_steering < 0)
              steer = obstacle_steering < line_following_steering ? obstacle_steering : line_following_steering;
          } else
            PID_need_reset = true;
          // apply the computed required angle
          set_steering_angle(steer);
        } else if (yellow_line_angle != UNKNOWN) {
          // no obstacle has been detected, simply follow the line
          wbu_driver_set_brake_intensity(0.0);
          set_steering_angle(applyPID(yellow_line_angle));
        } else {
          // no obstacle has been detected but we lost the line => we brake and hope to find the line again
          wbu_driver_set_brake_intensity(0.4);
          PID_need_reset = true;
        }
      }

      // update stuff
      if (has_gps)
        compute_gps_speed();
      if (enable_display)
        update_display();

      // Collect and write sensor data to CSV
      double sim_time = wb_robot_get_time();
      collect_and_write_sensor_data(sim_time);
    }

    ++i;
  }

  // Close CSV file
  if (csv_file) {
    fclose(csv_file);
    printf("Data collection complete. Total steps: %d\n", step_count);
  }

  wbu_driver_cleanup();

  return 0;  // ignored
}
