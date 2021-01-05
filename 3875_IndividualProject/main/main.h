#ifndef MAIN_H
#define MAIN_H

#include "m3pi.h"
#include "mbed.h"
#include <stdlib.h>

// Constants
#define A 0.5         // 20
#define B 1/10000     // 10000
#define C 1.5         // 2/3

#define SENS_THRESH 500    // >500 = black
#define TURN_SPEED 0.2

// API 
extern m3pi robot;

// LEDs 
extern BusOut leds;

// Buttons 
extern DigitalIn button_A;
extern DigitalIn button_B;
extern DigitalIn button_X;
extern DigitalIn button_Y;
extern DigitalIn button_enter;
extern DigitalIn button_back;

// Potentiometers
extern AnalogIn pot_P;
extern AnalogIn pot_I;
extern AnalogIn pot_D;
extern AnalogIn pot_S;

// Sensors
extern DigitalInOut QTRA; //connected to digital P26
extern DigitalInOut QTRB; //connected to digital P25

// Timers
Timer t_L;     // left encoder
Timer t_R;     // right encoder
Timer t_coord; // coordinate timer

// Prototypes

void read_encoders();
void init();
void calibrate();
void follow_line();
bool junction_detect();
char junction_logic();
void turn_select( char turn );
void left();
void right();
void back();
void goal();
void looped_goal();
void simplify();
void invert_path();
void non_looped();
void looped();
void node_logic();
bool coord_check();
void update_index();
void choose_turn();
void back_track();
int path_to_point_index( int path_point );
void print_data( char *word );
bool dead_end_removal(int index1, int index2);
void looped_goal_simplification();
void goal_path_explored( bool inverse );

// Global Variables
char path[100];
char inv_path[100];
int path_length = 0;
unsigned int *sensor;
float speed;
float proportional = 0.0;
float prev_proportional = 0.0;
float integral = 0.0;
float derivative = 0.0;
int encoder[2];
int dist_est_1 = 0;
int dist_est_2 = 0;

bool loop_check;
//bool completed = false;
bool _switch = false;
int dist = 0;
bool first = true;
bool first_g = true;
bool goal_node = false;
bool t_restart = true;
char dir;
int curr_index;
int curr_coords[2];
int total_points;
unsigned int looped_path[100];
unsigned int point[100];   // use a struct
unsigned int type[100];
unsigned int explored[100];
int coords_x[100];
int coords_y[100];
int shortest[100]; 
int short_length;
int goal_path1[100]; 
int goal_length1;
int goal_path2[100]; 
int goal_length2;
int goal_path3[100]; 
int goal_path4[100]; 
int temp_path[100];
int temp_length;
char* start = "L16 cdegreg4";
bool goal_sound = false;
char* victory = "e e e eee d e g g ggg";
char* oops =
    "v12 L16 o4 frc32<b32c32c#8cr8.erf";
char* doh =     
    "v12 L16 o4 c8.e8f#8ag8.e8c8<a<f#<f#<f#<g";
char* win = 
    "v12 L16 o5 eererce8g8r8<g8r8"
    "c8.<gr8<e8.<a8<b8<b-<a8<g.e.g.a8fgre8cd<b8."
    "c8.<gr8<e8.<a8<b8<b-<a8<g.e.g.a8fgre8cd<b8."
    "r8gf#fd#8er<g#<acr<acd"
    "r8gf#fd#8er>c8>c>c4"
    "r8gf#fd#8er<g#<acr<acd"
    "r8e-8rd8.c4";
//  "O6 T40 L16 d#<b<f#<d#<f#<bd#f#"
//  "T80 c#<b-<f#<c#<f#<b-c#8"
//  "T180 d#b<f#d#f#>bd#f#c#b-<f#c#f#>b-c#8 c>c#<c#>c#<b>c#<c#>c#c>c#<c#>c#<b>c#<c#>c#"
//  "c>c#<c#>c#<b->c#<c#>c#c>c#<c#>c#<b->c#<c#>c#"
//  "c>c#<c#>c#f>c#<c#>c#c>c#<c#>c#f>c#<c#>c#"
//  "c>c#<c#>c#f#>c#<c#>c#c>c#<c#>c#f#>c#<c#>c#d#bb-bd#bf#d#c#b-ab-c#b-f#d#";
char* bumblebee = 
    "! T144 L16 O6"
  "ag#gf#gf#fe fee-dc#cO5bb-"
  "ag#gf#gf#fe fee-dc#co4bb-"
  "ag#gf#gf#fe ag#gf#gf#fe"
  "ag#gf#fb-ag# ag#gf#ff#g#"
  "ag#gf#fb-ag# ag#gf#ff#g#"
  "ag#gf#gf#fe ff#gg#ab-ag#"
  "ag#gf#gf#fe ff#gg#abO5cc#"
  "dc#cO4bb-O5e-dc# dc#cO4bb-bO5cc#"
  "dc#cO4bb-O5e-dc# dc#cO4bb-bO5cc#"
  "dc#cO4b>cbb-a b-bO5cc#de-dc"
  "dc#cO4b>cbb-a b-bO5cc#de-dc"
  "dO4dMSdddddd e-de-O5e-e-de->e-"
  "dddddddd e-de->e-e-de-O6e-"
  "MLdO5e-dc#de-dc# de-dc#de-dc#"
  "de-eff#fee- de-eff#fee-"
  "dO4gMSgggggg a-ga->a-a-ga->a-"
  "gO5ggggggg a-ga->a-a-ga-O6a-"
  "MLfO5a-gf#ga-af# ga-gf#ga-gf#"
  "ga-ab-bb-aa- ga-ab-bb-aa-"
  "gf#fee-a-gf# gf#fee-eff#"
  "gf#fefee-d e-eff#ff#gg#"
  "ag#gf#gf#fe fee-dc#cO4bb-"
  "ab-ag#ab-ag# ab-ag#ab-ag#"
  "ab-ag#ab-ag# ab-ag#ab-ag#"
  "ag#gf#gf#fe fee-dc#cO3bb-"
  "ab-ag#ab-ag# ab-ag#ab-ag#"
  "ab-ag#ab-ag# ab-ag#ab-ag#"
  "ab-bO4cc#dd#e ff#gg#ab-bO5c"
  "c#dd#eff#gg# ab-ag#ab-ag#"
  "ag#gf#fb-ag# ag#gf#ff#g#"
  "ag#gf#fb-ag# ag#gf#ff#g#"
  "ag#gf#gf#fe ff#gg#ab-ag#"
  "ag#gf#gf#fe ff#gg#abO5cc#"
  "dc#cO4bb-O5e-dc# dc#cO4bb-bO5cc#"
  "dc#cO4bb-O5e-dc# dc#cO4bb-bO5cc#"
  "dc#cO4b>cbb-a b-bO5cc#de-dc"
  "dc#cO4b>cbb-a b-bO5cc#de-dc"
  "dO4dMSdddddd e-de-O5e-e-de->e-"
  "dddddddd e-de->e-e-de-O6e-"
  "MLdO5e-dc#de-dc# de-dc#de-dc#"
  "de-eff#fee- de-eff#fee-"
  "dO4gMSgggggg a-ga->a-a-ga->a-"
  "gO5ggggggg a-ga->a-a-ga-O6a-"
  "MLfO5a-gf#ga-af# ga-gf#ga-gf#"
  "ga-ab-bb-aa- ga-ab-bb-aa-"
  "gf#fee-a-gf# gf#fee-eff#"
  "gf#fefee-d e-eff#ff#gg#"
  "ag#gf#gf#fe fee-dc#cO4bb-"
  "ab-ag#ab-ag# ab-ag#ab-ag#"
  "ab-ag#ab-ag# ab-ag#ab-ag#"
  "ag#gf#gf#fe fee-dc#cO3bb-"
  "ab-ag#ab-ag# ab-ag#ab-ag#"
  "ab-ag#ab-ag# ab-ag#ab-ag#"
  "ab-bO4cc#dd#e ff#gg#ab-bO5c"
  "c#dd#eff#gg# ab-ag#ab-ag#"
  "ag#gf#fb-ag# ag#gf#ff#gg#"
  "ag#gf#fb-ag# ag#gf#ff#gg#"
  "ag#gf#gf#fe ff#gg#ab-ag#"
  "ag#gf#gf#fe ff#gg#abO6cc#"
  "dc#c<b<b-e-dc# dc#cO5bb-bO6cc#"
  "dc#c<b<b-e-dc# dc#cO5bb-bO6cc#"
  "dc#c<bcO5bb-a b-bO6cc#de-dc#"
  "dc#cO5bb-bO6cc# defgab-ag#"
  "ag#gf#fb-ag# ag#gf#ff#gg#"
  "ag#gf#fb-ag# ag#gf#ff#gg#"
  "a8c#de-eff# gf#fefee-d"
  "c#dd#eff#gg# ab-ag#ab-ag#"
  "a8O5c#de-eff# gf#fefee-d"
  "c#dd#eff#gg# ab-ag#abO6cc#"
  "dc#cO5bcbb-a b-ag#gf#fee-"
  "dc#cO4b>cbb-a b-ag#gf#fee-"
  "de-dc#e-de->e- de-dedfdg"
  "ab-ag#b>abO5b aR<ab<bO6cO4bO6c#"
  "L8dRL16O3ab-bO4c c#de-eff#gg#"
  "ab-bO5cc#de-e ff#gg#abO6cc#"
  "L2d O7d L4O5dR";
#endif