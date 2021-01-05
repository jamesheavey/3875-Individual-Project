#include "main.h"
#include <math.h> 

// API
m3pi robot;

// LEDs
BusOut leds(LED4,LED3,LED2,LED1);

// Buttons 
DigitalIn button_A(p18);
DigitalIn button_B(p17);
DigitalIn button_X(p21);
DigitalIn button_Y(p22);
DigitalIn button_enter(p24);
DigitalIn button_back(p23);

// Potentiometers
AnalogIn pot_P(p15);
AnalogIn pot_I(p16);
AnalogIn pot_D(p19);
AnalogIn pot_S(p20);

// Sensors
DigitalInOut enc_L(p26); //connected to digital P26
DigitalInOut enc_R(p25); //connected to digital P25

// Main

int main()
{
    init();
    
    robot.lcd_goto_xy(0,0);
    robot.lcd_print("A=SIMPLE", 10);
    robot.lcd_goto_xy(0,1);
    robot.lcd_print("B=LOOPED", 10);
    
    while(button_A.read() == 1 && button_B.read() == 1) {}
    
    if (button_B.read()) { loop_check = true; }   // non-looped
    if (button_A.read()) { loop_check = false; }  // looped

    robot.lcd_clear();
    robot.lcd_goto_xy(0,0);
    robot.lcd_print("  ENTER   ", 10);
    robot.lcd_goto_xy(0,1);
    robot.lcd_print("= start   ", 10);
    
    robot.play_music(start, 13);
    
    calibrate();
    
    robot.lcd_clear();
    
    speed = 0.3;//(pot_S*0.3)+0.2;   // have it so max is 0.5 and min is 0.2 (this lowest doesnt work)
    
    float dt = 1/50;           // updating 50 times a second

    while (1) {
        
        if (loop_check == true) {
            non_looped();
        } else {
            looped();
        }
        
        wait(dt);
    }
}

void read_encoders() 
{
        enc_R.output();    // Set the I/O line to an output
        enc_L.output();
        enc_R.mode(PullUp);
        enc_L.mode(PullUp);
        
        wait_us(10);         // Must be atleast 10us for the 10 nF capacitor to charge
        enc_R.mode(PullNone);
        enc_L.mode(PullNone);   
        enc_R = 1;            // Drive the line high
        enc_L = 1;
        
        t_R.start();
        enc_R.input();        // Make the I/O line an input (high impedance)
        while (enc_R == 1 || t_R.read_us() < 1000);  // replace 1000 with a hard variable (1000 = 1ms = 1kHz sampling) (might be able to drop this further
        // sampling time is required to be this high for times when there is no reflectance but we only care about high reflectance
        
        // maybe i could wait a few microseconds, see if the encoder is still high, if high then no reflectance, if low, the high reflectance
        // this would increase sampling time
        
        // also, the fact that the waits are in the same loop means that the loop will run at different speeds depending on whether a sensor is triggered or not
        // if both are triggered it will run fast, otherwise it will have to wait 1000+ us for each sensor
        
        // this therefore needs to be done in parallel and also must not affect the time of other operations in the main loop
        encoder[0] = t_R.read_us();   // Measure the time for the capacitor to discharge by waiting for the I/O line to go low
        t_R.stop();
        t_R.reset();
        
        t_L.start();
        enc_L.input();
        while (enc_L == 1 || t_L.read_us() < 1000);
        encoder[1] = t_L.read_us();        
        t_L.stop();
        t_L.reset();
}

void init()
{
    robot.init();

    button_A.mode(PullUp);
    button_B.mode(PullUp);
    button_X.mode(PullUp);
    button_Y.mode(PullUp);
    button_enter.mode(PullUp);
    button_back.mode(PullUp);

    leds = 0b0000;
}

void calibrate()
{
    leds = 0b1111;
    robot.reset_calibration();

    while (button_enter.read() == 1) {}  // wait for enter to be pressed
    
    wait(2.0);  
    
    robot.auto_calibrate();  
    
    robot.stop();
    wait(0.05);
    robot.scan();
    
    leds = 0b0000;
}

void non_looped() 
{
    follow_line();
    
    if ( junction_detect() ) {
        char turn = junction_logic(); 
        turn_select(turn);
        
        path[path_length] = turn;
        path_length ++;
    }
    
    simplify();
    
    robot.lcd_clear();
    robot.lcd_print(path,100);

    //robot.display_data();
}

void looped()  
{
    if( first ) {   // init the start node on first loop run only
        first = false;
        curr_coords[0] = 0;
        curr_coords[1] = 0;
        dir = 'N';
        total_points = 0;
        point[total_points] = total_points;       // first point is 0
        coords_x[total_points] = curr_coords[0];
        coords_y[total_points] = curr_coords[1];
        type[total_points] = 0b0100;              // start is always 1 exit type in the north direction
        explored[total_points] = 0b0000;          
        looped_path[total_points] = 0;            // start node is '0'
    }
    
      // follow line until reaching a junction, determine its type and coordinates
    if ( t_restart ){                // only start the timer if it isnt already started
        t_coord.start();
        t_restart = false;
        
    }
    
    follow_line();
    
    if ( junction_detect() ) {
        path_length++;                                  // increment the path position index
        
        float time = t_coord.read();
        int dist_est = ceil(time*2); // scaled so that a longer straight will have a different time to a shorter straight
        t_coord.stop();
        t_coord.reset();
        t_restart = true; //restart the timer next loop
        
        if (dir == 'N'){ curr_coords[1] += dist_est; }  // y coord
        if (dir == 'E'){ curr_coords[0] += dist_est; }  // x coord
        if (dir == 'S'){ curr_coords[1] -= dist_est; }
        if (dir == 'W'){ curr_coords[0] -= dist_est; }
        
        // check that the coordinates are not already in the list, if not add the point, if it is already return the point number and increment the explored
        if (coord_check()) {  // coord_check returns true if curr_coords coordinates are not present in coords_x and y
            
            total_points++;
            node_logic(); // determines what junction type we are at updates the explored (path entered on) and type arrays accordingly
            
            if(goal_node) { point[total_points] = 100; goal_node = false; goal_sound = true; }  // 100 will be the indicator for the goal node that we can visit once mapping is complete
            else { point[total_points] = total_points; } // numbered 0 -> total_points
            
            coords_x[total_points] = curr_coords[0];
            coords_y[total_points] = curr_coords[1];
        }
        
        update_index();

        // use current coords to find which point to place in path
    
        looped_path[path_length] = point[curr_index]; //returns an int of which point we are at what its called
        choose_turn(); //looks at the point we are at, examines the type vs explored and makes the appropriate turn also updates the explored
        
        // check_explored(); // iterates through all existing points, if all explored match type, then mapping is complete
        // if not, make a func that traverses back through the bath until reaching that node, then explore the unexplored path
        // i.e. the function will take the node ID integer as an argument and go backwards through the path until reaching that node (appending each node along the way to the end of the path)
        
        
        
        // needs a function that checks if current node has any paths left to explore, if not, then it must return via the path to a node that isnt fully explored and continue from there
        
    }


//    robot.lcd_clear();
//    char *b = &dir;
//    robot.lcd_print(b,1);

      
    //robot.display_data();
}

bool coord_check()
{
    bool result = true;
    //returns true if the current coords dont match a previous point
    for(int i = 0; i <= total_points; i++) {
        if(curr_coords[0] == coords_x[i] && curr_coords[1] == coords_y[i]) { 
            result = false; 
        }
    }
    
    return result;
}

void update_index() // update index (pointer to current point/type/coords/explored)
{
    // checks the curr_coords againts the coords array, returns the index to relate to the point array
    for(int i = 0; i <= total_points; i++) {
        if(curr_coords[0] == coords_x[i] && curr_coords[1] == coords_y[i]) { 
            curr_index = i; 
        }
    }
}

int path_to_point_index( int path_point )
{
    for(int i = 0; i <= total_points; i++) {
        if(path_point == point[i]) { 
            return i;
        } 
    }
    
    return curr_index;  // default 
}

void node_logic()
{
    // is done when a new node is discovered, needs to update the nodes type and the path explored upon entry
    
    // first determine what turns are available relative to the robots current direction (left, straight etc.)
    // convert these relative available turns into available absolute diections (N,E etc.)
    // set _type  to the appropriate value based on available directions (including entry direction = opposite of current)
    // set _explored entry path as 1
    // set type[total_points] = _type; & explored[total_points] = _explored;
    
    bool north = false;
    bool south = false;
    bool east = false;
    bool west = false;
    
    bool left = false;
    bool straight = false;
    bool right = false;
    bool goal = false;
    
    int _type = 0b0000;
    int _explored = 0b0000;
    
    if (sensor[0] > SENS_THRESH || sensor[4] > SENS_THRESH) {
        while ( (sensor[0] > SENS_THRESH || sensor[4] > SENS_THRESH) && (sensor[1] > SENS_THRESH || sensor[2] > SENS_THRESH || sensor[3] > SENS_THRESH) ) {
            robot.forward(speed);
            robot.scan();
            if ( sensor[0] > SENS_THRESH ) { left = true; }
            if ( sensor[4] > SENS_THRESH ) { right = true; }
        }
        
        if ( sensor[0] > SENS_THRESH && sensor[4] > SENS_THRESH && sensor[2] < SENS_THRESH ) {
            wait(0.02);        // maybe change or replace w something better
            robot.scan();
            if ( sensor[0] > SENS_THRESH && sensor[4] > SENS_THRESH && sensor[2] < SENS_THRESH ) {
                goal = true;
            }
        }
        robot.scan();
        
        if ( sensor[1] > SENS_THRESH || sensor[2] > SENS_THRESH || sensor[3] > SENS_THRESH ) {
            straight = true;
        }
    }
    
    if(goal) {
        if( dir == 'N' ) { south = true;       _explored |= 0b0001; }  // sets the direction opposite to entry direction as an explored path 
        else if ( dir == 'E' ) { west = true;  _explored |= 0b1000; }
        else if ( dir == 'S' ) { north = true; _explored |= 0b0100; }
        else if ( dir == 'W' ) { east = true;  _explored |= 0b0010; }
        
        if ( west  ) { _type |= 0b1000; }
        if ( north ) { _type |= 0b0100; }
        if ( east  ) { _type |= 0b0010; }
        if ( south ) { _type |= 0b0001; }
        
        goal_node = true;
    }
    
    else {
        int angle = 0;
        int reset_ang = 0;
        
        if (dir == 'E') { angle = 90; }
        else if (dir == 'S') { angle = 180; }
        else if (dir == 'W') { angle = 270; }
        
        reset_ang = angle;
        
        if (left) { 
            angle += 270;
            angle = angle % 360;
            if (angle == 0) { north = true; }
            if (angle == 180) { south = true; }
            if (angle == 90) { east = true; }
            if (angle == 270) { west = true; }
            angle = reset_ang;
        }
        
        if (right) { 
            angle += 90;
            angle = angle % 360;
            if (angle == 0) { north = true; }
            if (angle == 180) { south = true; }
            if (angle == 90) { east = true; }
            if (angle == 270) { west = true; }
            angle = reset_ang;
        }
          
        if (straight) { 
            if (angle == 0) { north = true; }
            if (angle == 180) { south = true; }
            if (angle == 90) { east = true; }
            if (angle == 270) { west = true; }
        }
        
        if( dir == 'N' ) { south = true;       _explored |= 0b0001; }  // sets the direction opposite to entry direction as an explored path 
        else if ( dir == 'E' ) { west = true;  _explored |= 0b1000; }  // this is acc done in choose turn so might not be needed here
        else if ( dir == 'S' ) { north = true; _explored |= 0b0100; }
        else if ( dir == 'W' ) { east = true;  _explored |= 0b0010; }
        
        if ( west  ) { _type |= 0b1000; }
        if ( north ) { _type |= 0b0100; }
        if ( east  ) { _type |= 0b0010; }
        if ( south ) { _type |= 0b0001; }
    }   
     
    type[total_points] = _type;      // maybe update_index and use curr_index instead of total_points
    explored[total_points] = _explored;
    
}

void choose_turn() 
{
    // look at cuurent coords, find what node we are at
    // looks at the type vs the explored and does the turn that is equivalent to the first 1 in type that is a 0 in explored (WNES priority)
    // sets the explored of the current node to 1 in whatever path is chosen
    // also update dir

    if( dir == 'N' ) {       explored[curr_index] |= 0b0001; }  // sets the direction opposite to entry direction as an explored path 
    else if ( dir == 'E' ) { explored[curr_index] |= 0b1000; }
    else if ( dir == 'S' ) { explored[curr_index] |= 0b0100; }
    else if ( dir == 'W' ) { explored[curr_index] |= 0b0010; }
//    print_data("enter junc");
    int unexp_paths = type[curr_index] & ~( explored[curr_index] );  // produces a binary of 1's in the available unexplored paths
    
    if (unexp_paths == 0b0000) {
        back_track();
        unexp_paths = type[curr_index] & ~( explored[curr_index] );
    }

    int curr_angle = 0;
    if ( dir == 'E' ) { curr_angle = 90;}
    else if ( dir == 'S' ) { curr_angle = 180;}
    else if ( dir == 'W' ) { curr_angle = 270;}
    
    int desired_angle = 0;
    if ( (unexp_paths & 0b1000) == 0b1000) { 
        desired_angle = 270; 
        dir = 'W'; 
        explored[curr_index] |= 0b1000;
    }
    else if ( (unexp_paths & 0b0100) == 0b0100) { 
        desired_angle = 0;
        dir = 'N';
        explored[curr_index] |= 0b0100;
    }
    else if ( (unexp_paths & 0b0010) == 0b0010) { 
        desired_angle = 90; 
        dir = 'E';
        explored[curr_index] |= 0b0010;
    }
    else if ( (unexp_paths & 0b0001) == 0b0001) { 
        desired_angle = 180;
        dir = 'S';
        explored[curr_index] |= 0b0001;
    }
     
    int turn_angle = (desired_angle - curr_angle + 360) % 360;
    
//    robot.lcd_clear();
//    robot.lcd_print("turn" , 4);
//    wait(2);
//    robot.stop();
//    robot.lcd_clear();
//    if( turn_angle == 0)  { robot.lcd_print("S",1); } 
//    else if( turn_angle == 90)  { robot.lcd_print("R",1); } 
//    else if( turn_angle == 180)  { robot.lcd_print("B",1); } 
//    else if( turn_angle == 270)  { robot.lcd_print("L",1); } 
//    wait(2);
//    
//    print_data("After Turn");
    
    if( turn_angle == 0)  { robot.forward(speed); wait(0.1); turn_select('S'); } 
    else if( turn_angle == 90)  { robot.forward(speed); wait(0.03); turn_select('R'); } 
    else if( turn_angle == 180)  { turn_select('B'); } 
    else if( turn_angle == 270)  { robot.forward(speed); wait(0.03); turn_select('L'); } 
}

void back_track()
{
    // find the closest previous node with unexplored paths and go back through the path until reaching that node, updating the path and directions appropriately, then choose turn
    // also if no nodes have unexplored paths set complete to true
    
    bool fully_explored = true;     
            
    int d_node;   // an index to the most recent (desired) node with unexplored paths
    
    for(int i = total_points; i >= 0; i--) {    // start from the most recently discovered node
        if( explored[i] != type[i] ) {          
            fully_explored = false;
            d_node = i;
            break;
        }
    }
    
    if( fully_explored == true && first_g == true ) { first_g = false; looped_goal(); }
    
    else {
            // compare node coordinates to previous node coordinates
            // determine which direction the previous node is compared to current
            // set the current nodes direction path to 0 and the opposite direction path to 0
            // decrement the count (setting the previous as current node and the next previous as previous)
            // when previous node == point[pointer1] break
            // choose turn should then do all those turns and arrive at correct node
        
        // check if the current node exists before the discovery of the desired node
        // if it does, check the number of nodes between before and after
        // which ever is shorter, set the 0 of those nodes
        
//        robot.stop();
//        robot.lcd_print("bt",2);
//        wait(1);
//        robot.lcd_clear();
//        
//        char buf1[2], buf2[2];
//        sprintf(buf1,"%d",point[d_node]);
//        sprintf(buf2,"%d",point[curr_index]);
//        robot.lcd_print(buf1,2);
//        robot.lcd_goto_xy(0,1);
//        robot.lcd_print(buf2,2); 
//
//        wait(1);
//        robot.lcd_clear();
          
        bool check = dead_end_removal(point[d_node], point[curr_index]);
        char dir_diff;
        
        if(check) {
            for(int j = 0; j <= short_length; j++) {
                int curr_node = path_to_point_index(shortest[j]);
                int next_node = path_to_point_index(shortest[j+1]);
                if(coords_x[next_node] != coords_x[curr_node]) {
                    if(coords_x[next_node] - coords_x[curr_node] > 0){
                        dir_diff = 'E';
                    } else {
                        dir_diff = 'W';
                    }
                } else if( coords_y[next_node] != coords_y[curr_node] ) {
                    if(coords_y[next_node] - coords_y[curr_node] > 0){
                        dir_diff = 'N';
                    } else {
                        dir_diff = 'S';
                    }
                }
                
                if( dir_diff == 'N' ) { 
                    explored[curr_node] &= 0b1011;
                }
                else if( dir_diff == 'E' ) { 
                    explored[curr_node] &= 0b1101; 
                }
                else if( dir_diff == 'S' ) { 
                    explored[curr_node] &= 0b1110;
                }
                else if( dir_diff == 'W' ) { 
                    explored[curr_node] &= 0b0111;
                }
                if(point[next_node] == point[d_node]) { break; }
            }  
        }
        
        else { 
        
            for(int j = short_length; j >= 0; j--) {
                int curr_node = path_to_point_index(shortest[j]);
                int next_node = path_to_point_index(shortest[j-1]);
                if(coords_x[next_node] != coords_x[curr_node]) {
                    if(coords_x[next_node] - coords_x[curr_node] > 0){
                        dir_diff = 'E';
                    } else {
                        dir_diff = 'W';
                    }
                } else if( coords_y[next_node] != coords_y[curr_node] ) {
                    if(coords_y[next_node] - coords_y[curr_node] > 0){
                        dir_diff = 'N';
                    } else {
                        dir_diff = 'S';
                    }
                }
                
                if( dir_diff == 'N' ) { 
                    explored[curr_node] &= 0b1011;
                }
                else if( dir_diff == 'E' ) { 
                    explored[curr_node] &= 0b1101; 
                }
                else if( dir_diff == 'S' ) { 
                    explored[curr_node] &= 0b1110;
                }
                else if( dir_diff == 'W' ) { 
                    explored[curr_node] &= 0b0111;
                }
                if(point[next_node] == point[d_node]) { break; }
            }
        }
        
 //       print_data("aft bt");

    }
}

bool dead_end_removal( int point1, int point2 )   // change into dead_end_removal and have it take two indexes and a path and return an array
{   
    // dead end removal between start and final node
    // add the inverse of the result to end of the looped_path
    // then separate into before and after, simplify both and compare
    robot.stop();
    int index1, index2;
    int d_index, before_index, after_index;
    bool before = false;
    
    for(int k = 0; k <= path_length; k++) {
        if(looped_path[k] == point1) { d_index = k; }
    }
    
    for(int k = 0; k <= path_length; k++) {
        if (looped_path[k] == point2 && k < d_index) { before = true; before_index = k; }
        if (looped_path[k] == point2 && k > d_index) { after_index = k; }
    }
        
    if(before){
        if( (d_index - before_index) <= (after_index - d_index) ){
            before = true;
        }
        else{
            before = false;
        }
    }
    
    if(before) {
        index1 = before_index;
        index2 = d_index;
        
    }else{
        index1 = d_index;
        index2 = after_index;
    }
    
    int temp_array[100];
    
    for( int x = 0; x <= path_length; x++ ) { shortest[x] = NULL; }
    
    for( int i = index1; i <= index2; i++ ) { shortest[i-index1] = looped_path[i]; }
    
    short_length = index2 - index1;
    
    // for every node in path, check if repeated
    // if rpeated, move everyting before the first occurence to the final occurence
    // save new path and repeat
    // when no node is repeated, end
    // or end when iterated through the entire array
    
    int i = 0;
    while( i <= short_length ) {
        int count = i;
        for( int j = 0; j <= short_length; j++ ) {
            if( shortest[i] == shortest[j] ){
                count = j;
            }
        }
        if( count != i ) { 
            
            for( int k = 0; k <= i; k++ ) {
                temp_array[k] = shortest[k];
            }
            int ind = 1;
            for( int z = count+1; z <= path_length; z++ ) {
                temp_array[i+ind] = shortest[z];
                ind++;
            }
            // clear the array
            for( int x = 0; x <= short_length; x++ ) { shortest[x] = NULL; }
            
            short_length -= (count-i);
            i = -1;
            
            for( int x = 0; x <= short_length; x++ ) { shortest[x] = temp_array[x]; }
            
            for( int x = 0; x <= short_length; x++ ) { temp_array[x] = NULL; }
        }
        i++;
    }
    
    //robot.lcd_print("len",10);
//    wait(1);
//    robot.lcd_clear();
//    
//    char buffe[3];
//    sprintf(buffe,"%d", short_length);
//    robot.lcd_print(buffe,3);
//    wait(1);
//    robot.lcd_clear();
//    
//    robot.lcd_print("path",10);
//    wait(1);
//    robot.lcd_clear();
//    
//    leds = 0b1111;
//    for(int m = 0; m <= short_length; m++) {
//        leds = ~leds;
//        char buffer[3];
//        sprintf(buffer,"%d", shortest[m]);
//        robot.lcd_print(buffer,3);
//        wait(1);
//        robot.lcd_clear();
//    }
    
    return before;
    // print these to check
    // add inverted sub path to the real path and increment the path length by real_len -1
}

void shorten( int path[], int length ) 
{
    int temp_array[100];
    int i = 0;
    
    while( i <= length ) {
        int count = i;
        for( int j = 0; j <= length; j++ ) {
            if( path[i] == path[j] ){
                count = j;
            }
        }
        if( count != i ) { 
            
            for( int k = 0; k <= i; k++ ) {
                temp_array[k] = path[k];
            }
            int ind = 1;
            for( int z = count+1; z <= length; z++ ) {
                temp_array[i+ind] = path[z];
                ind++;
            }
            // clear the array
            for( int x = 0; x <= length; x++ ) { path[x] = NULL; }
            
            length -= (count-i);
            i = -1;
            
            for( int x = 0; x <= length; x++ ) { path[x] = temp_array[x]; }
            
            for( int x = 0; x <= length; x++ ) { temp_array[x] = NULL; }
        }
        i++;
    }
    
    temp_length = length;
    
    for( int x = 0; x <= 100; x++ ) { temp_path[x] = NULL; }
    
    for( int x = 0; x <= length; x++ ) { temp_path[x] = path[x]; }
}

void looped_goal_simplification()   // change into dead_end_removal and have it take two indexes and a path and return an array
{   
    for( int i = 0; i <= path_length; i++ ) {
        goal_path1[i] = looped_path[i];
        if( looped_path[i] == 100 ) { goal_length1 = i; break; }
    }
    
    for( int i = goal_length1; i >= 0; i-- ) {
        goal_path3[goal_length1-i] = goal_path1[i];
    }
    int goal_length3 = goal_length1;
    
    for( int i = path_length; i >= 0; i-- ) {
        goal_path2[path_length-i] = looped_path[i];
        if( looped_path[i] == 100 ) { goal_length2 = path_length-i; break; }
    }
    
    for( int i = goal_length2; i >= 0; i-- ) {
        goal_path4[goal_length2-i] = goal_path2[i];
    }
    int goal_length4 = goal_length2;
    
    int temp_array[100];
    // path 1
    int i = 0;
    while( i <= goal_length1 ) {
        int count = i;
        for( int j = 0; j <= goal_length1; j++ ) {
            if( goal_path1[i] == goal_path1[j] ){
                count = j;
            }
        }
        if( count != i ) { 
            
            for( int k = 0; k <= i; k++ ) {
                temp_array[k] = goal_path1[k];
            }
            int ind = 1;
            for( int z = count+1; z <= goal_length1; z++ ) {
                temp_array[i+ind] = goal_path1[z];
                ind++;
            }
            // clear the array
            for( int x = 0; x <= goal_length1; x++ ) { goal_path1[x] = NULL; }
            
            goal_length1 -= (count-i);
            i = -1;
            
            for( int x = 0; x <= goal_length1; x++ ) { goal_path1[x] = temp_array[x]; }
            
            for( int x = 0; x <= goal_length1; x++ ) { temp_array[x] = NULL; }
        }
        i++;
    }
    // path 2
    i = 0;
    while( i <= goal_length2 ) {
        int count = i;
        for( int j = 0; j <= goal_length2; j++ ) {
            if( goal_path2[i] == goal_path2[j] ){
                count = j;
            }
        }
        if( count != i ) { 
            
            for( int k = 0; k <= i; k++ ) {
                temp_array[k] = goal_path2[k];
            }
            int ind = 1;
            for( int z = count+1; z <= goal_length2; z++ ) {
                temp_array[i+ind] = goal_path2[z];
                ind++;
            }
            // clear the array
            for( int x = 0; x <= goal_length2; x++ ) { goal_path2[x] = NULL; }
            
            goal_length2 -= (count-i);
            i = -1;
            
            for( int x = 0; x <= goal_length2; x++ ) { goal_path2[x] = temp_array[x]; }
            
            for( int x = 0; x <= goal_length2; x++ ) { temp_array[x] = NULL; }
        }
        i++;
    }
    // path 3
    i = 0;
    while( i <= goal_length3 ) {
        int count = i;
        for( int j = 0; j <= goal_length3; j++ ) {
            if( goal_path3[i] == goal_path3[j] ){
                count = j;
            }
        }
        if( count != i ) { 
            
            for( int k = 0; k <= i; k++ ) {
                temp_array[k] = goal_path3[k];
            }
            int ind = 1;
            for( int z = count+1; z <= goal_length3; z++ ) {
                temp_array[i+ind] = goal_path3[z];
                ind++;
            }
            // clear the array
            for( int x = 0; x <= goal_length3; x++ ) { goal_path3[x] = NULL; }
            
            goal_length3 -= (count-i);
            i = -1;
            
            for( int x = 0; x <= goal_length3; x++ ) { goal_path3[x] = temp_array[x]; }
            
            for( int x = 0; x <= goal_length3; x++ ) { temp_array[x] = NULL; }
        }
        i++;
    }
    i = 0;
    while( i <= goal_length4 ) {
        int count = i;
        for( int j = 0; j <= goal_length4; j++ ) {
            if( goal_path4[i] == goal_path4[j] ){
                count = j;
            }
        }
        if( count != i ) { 
            
            for( int k = 0; k <= i; k++ ) {
                temp_array[k] = goal_path4[k];
            }
            int ind = 1;
            for( int z = count+1; z <= goal_length4; z++ ) {
                temp_array[i+ind] = goal_path4[z];
                ind++;
            }
            // clear the array
            for( int x = 0; x <= goal_length4; x++ ) { goal_path4[x] = NULL; }
            
            goal_length4 -= (count-i);
            i = -1;
            
            for( int x = 0; x <= goal_length4; x++ ) { goal_path4[x] = temp_array[x]; }
            
            for( int x = 0; x <= goal_length4; x++ ) { temp_array[x] = NULL; }
        }
        i++;
    }
    
    if( goal_length2 <= goal_length1 && goal_length2 <= goal_length3 && goal_length2 <= goal_length4 ) {
        for( int x = 0; x <= goal_length1; x++ ) { goal_path1[x] = NULL;  }
        for( int x = 0; x <= goal_length2; x++ ) { 
            goal_path1[x] = goal_path2[x];
        }
        goal_length1 = goal_length2;
    } 
    else if( goal_length3 <= goal_length1 && goal_length3 <= goal_length2 && goal_length3 <= goal_length4 ) {
        for( int x = 0; x <= goal_length1; x++ ) { goal_path1[x] = NULL;  }
        for( int x = goal_length3; x >= 0; x-- ) { 
            goal_path1[goal_length3 - x] = goal_path3[x];
        }
        goal_length1 = goal_length3;
    } 
    else if( goal_length4 <= goal_length1 && goal_length4 <= goal_length2 && goal_length4 <= goal_length3 ) {
        for( int x = 0; x <= goal_length1; x++ ) { goal_path1[x] = NULL;  }
        for( int x = goal_length4; x >= 0; x-- ) { 
            goal_path1[goal_length4 - x] = goal_path4[x];
        }
        goal_length1 = goal_length4;
    } 
    
//    robot.lcd_print("path",10);
//    wait(1);
//    robot.lcd_clear();
//    
//    leds = 0b1111;
//    for(int m = 0; m <= path_length; m++) {
//        leds = ~leds;
//        char buffer[3];
//        sprintf(buffer,"%d", looped_path[m]);
//        robot.lcd_print(buffer,3);
//        wait(1);
//        robot.lcd_clear();
//    }
    
//    robot.lcd_print("g path",10);
//    wait(1);
//    robot.lcd_clear();
//    
//    leds = 0b1111;
//    for(int m = 0; m <= goal_length1; m++) {
//        leds = ~leds;
//        char buffer[3];
//        sprintf(buffer,"%d", goal_path1[m]);
//        robot.lcd_print(buffer,3);
//        wait(1);
//        robot.lcd_clear();
//    }
    
    // print these to check
    // add inverted sub path to the real path and increment the path length by real_len -1
}

void follow_line() 
{
    robot.scan();
    sensor = robot.get_sensors(); // returns the current values of all the sensors from 0-1000
    
    leds = 0b0110;
    
    proportional = robot.read_line();  // returns a value between -1,1     (-1 = PC0 or further , -1 to -0.5 = PC1 (-0.5 is directly below PC1) , -0.5 to 0 = PC2 , 0 to 0.5 = PC3 , 0.5 to 1 and further = PC4)
    derivative = proportional - prev_proportional;
    integral += proportional;
    prev_proportional = proportional;
    
    // calculate motor correction
    float motor_correction = proportional*A + integral*B + derivative*C;
    
    // make sure the correction is never greater than the max speed as the motor will reverse
    if( motor_correction > speed ) {
        motor_correction = speed;
    }
    if( motor_correction < -speed ) {
        motor_correction = -speed;
    }

    if( proportional < 0 ) {
        robot.motors(speed+motor_correction,speed);
    } else {
        robot.motors(speed,speed-motor_correction);
    }
    
//    read_encoders();     
//    if (encoder[0] > 3100) { dist_est_1 += 1; }  // going to have to reset these dist estimates every junction (in the if (junc_detect()) statement)
//    if (encoder[1] > 3100) { dist_est_2 += 1; }  // might not need to actually use 2pir/3 could just add arbitrary numbers
}

bool junction_detect() 
{
    if ( sensor[0] > SENS_THRESH || sensor[4] > SENS_THRESH ) {
        return true;
    } else if ( sensor[1] < SENS_THRESH && sensor[2] < SENS_THRESH && sensor[3] < SENS_THRESH ) {
        return true;
    } else {
        return false;
    }
}

char junction_logic() 
{
    bool straight = false;
    bool left = false;
    bool right = false;
    bool goal = false;
    
    if (sensor[0] > SENS_THRESH || sensor[4] > SENS_THRESH) {
        while ( (sensor[0] > SENS_THRESH || sensor[4] > SENS_THRESH) && (sensor[1] > SENS_THRESH || sensor[2] > SENS_THRESH || sensor[3] > SENS_THRESH) ) {
            robot.forward(speed);
            robot.scan();
            if ( sensor[0] > SENS_THRESH ) { left = true; }
            if ( sensor[4] > SENS_THRESH ) { right = true; }
        }
        
        if ( sensor[0] > SENS_THRESH && sensor[4] > SENS_THRESH && sensor[2] < SENS_THRESH ) {
            wait(0.05);        // maybe change or replace w something better
            robot.scan();
            if ( sensor[0] > SENS_THRESH && sensor[4] > SENS_THRESH && sensor[2] < SENS_THRESH ) {
                goal = true;
            }
        }
        
        robot.scan();
        
        if ( sensor[1] > SENS_THRESH || sensor[2] > SENS_THRESH || sensor[3] > SENS_THRESH ) {
            straight = true;
        }
        
    } else if (sensor[1] < SENS_THRESH && sensor[2] < SENS_THRESH && sensor[3] < SENS_THRESH) {
        return 'B';
    }
    
    if (goal) {
        return 'G';
    } else if (left) {
        return 'L';
    } else if (straight) {
        return 'S';
    } else if (right) {
        return 'R';
    } else {
        return 'S';
    }
}


void turn_select( char turn )
{
    switch(turn) {
        case 'G':
            goal();
        case 'L':
            left();
            break;
        case 'S':
            break;
        case 'R':
            right();
            break;
        case 'B':
            back();
            break;
    }
}
        
void left()
{
    robot.play_music("gg",3);
    
    leds = 0b1100;

    while (sensor[0] > SENS_THRESH) { robot.scan(); }
    
    robot.spin_left(TURN_SPEED);
    
    while (sensor[0] < SENS_THRESH) { robot.scan(); }
    
    while (sensor[0] > SENS_THRESH) { robot.scan(); }
    
    while (sensor[1] < SENS_THRESH) { robot.scan(); }
    
    while (sensor[1] > SENS_THRESH) { robot.scan(); }
}

void right()
{
    robot.play_music("ee",3);
    
    leds = 0b0011;

    while (sensor[4] > SENS_THRESH) { robot.scan(); }
    
    robot.spin_right(TURN_SPEED);
    
    while (sensor[4] < SENS_THRESH) { robot.scan(); }
    
    while (sensor[4] > SENS_THRESH) { robot.scan(); }
    
    while (sensor[3] < SENS_THRESH) { robot.scan(); }
    
    while (sensor[3] > SENS_THRESH) { robot.scan(); }
}

void back() 
{
    if(goal_sound){
        robot.play_music("ccgg",5);
        goal_sound = false;
    }
    
    else{
        robot.play_music("ggcc",5);
        //robot.play_music(oops,50);
    }
    
    leds = 0b1111;

    robot.spin_right(TURN_SPEED);
    
    wait(0.65);
}

void simplify()
{
    // check if the last one was a 'B'
    // if it was, iterate over the last three turns and check the total angle change
    // replace the three turns with the new single turn
    
    if( path[path_length-2] == 'B' && path_length >= 3) {
        int angle_change = 0;
        
        for (int i = 1; i <= 3; i++) {
            if (path[path_length - i] == 'L') { angle_change += 270; }
            else if (path[path_length - i] == 'R') { angle_change += 90; }
            else if (path[path_length - i] == 'B') { angle_change += 180; }
        }
        
        angle_change = angle_change % 360;
        
        if (angle_change == 0) { path[path_length - 3] = 'S'; }
        else if (angle_change == 90) { path[path_length - 3] = 'R'; }
        else if (angle_change == 180) { path[path_length - 3] = 'B'; }
        else if (angle_change == 270) { path[path_length - 3] = 'L'; }
        
        for (int i = 1; i <= 2; i++) { path[path_length - i] = NULL; }   // clear the other turns
        
        path_length -= 2;        
    }
}  

void goal()
{ 
    invert_path();
    
    leds = 0b0000;
    
    robot.lcd_clear();
    robot.lcd_print(inv_path,100);
    
    while(1) {
        int pointer = 0;
        
        robot.stop();
        
        leds = 0b1001;
        wait(0.2);
        leds = 0b0110;
        wait(0.2);
        
        robot.reverse(speed);
        while(sensor[0] > SENS_THRESH || sensor[4] > SENS_THRESH) { robot.scan(); }
        wait(0.05);
        
        robot.spin_right(TURN_SPEED);
        while(sensor[2] > SENS_THRESH) { robot.scan(); }
        while(sensor[3] < SENS_THRESH) { robot.scan(); } 
        while(sensor[3] > SENS_THRESH) { robot.scan(); } 

        robot.stop();
        
        while(pointer <= path_length) {
            follow_line();
            
            if ( junction_detect() ) {  // if junction found
                char na = junction_logic();   // aids turing fluidity (char not necessary therefore could clean up a bit) 
                turn_select(inv_path[pointer]);
                if(inv_path[pointer] == 'S') {      // make this better
                    robot.forward(speed);
                    leds = 0b1010;
                    while(sensor[0] > SENS_THRESH || sensor[4] > SENS_THRESH) { robot.scan(); }
                }
                pointer++;
            }
        }
        
        back();
        
        robot.stop();
        robot.lcd_goto_xy(0,0);
        robot.lcd_print("  ENTER   ", 10);
        robot.lcd_goto_xy(0,1);
        robot.lcd_print("=restart", 10);
        
        while ( button_enter.read() == 1 ) { speed = (pot_S*0.3)+0.2; }  // keep looping waiting for Enter to be pressed (can change speed)
        
        robot.lcd_clear();
        robot.lcd_print(path,100);
        
        pointer = 0;
        
        leds = 0b1001;
        wait(0.2);
        leds = 0b0110;
        wait(0.2);
        leds = 0b1001;
        wait(0.2);
        leds = 0b0110;
        wait(0.2);
        
        while(pointer <= path_length) {
            follow_line();
            
            if ( junction_detect() ) {  // if junction found
                char na = junction_logic();   // aids turing fluidity (char not necessary therefore could clean up a bit) 
                turn_select(path[pointer]);
                if(path[pointer] == 'S') {      // make this better
                    robot.forward(speed);
                    leds = 0b1010;
                    while(sensor[0] > SENS_THRESH || sensor[4] > SENS_THRESH) { robot.scan(); }
                }
                pointer++;
            }
        }
    }
}   

void looped_goal() 
{
    // return to start (adding nodes to path)
    // simplify path, check path before reaching goal and path after reaching goal, use the shorter one
    // loop between start and goal
    turn_select('B');
    robot.stop();
    
    if( dir == 'N' ) { dir = 'S'; }
    else if( dir == 'S' ) { dir = 'N'; }
    else if( dir == 'E' ) { dir = 'W'; }
    else if( dir == 'W' ) { dir = 'E'; }
    
    looped_goal_simplification();
    
    while(1) {
        
        if( _switch == false ) {
            //robot.play_music(victory,22);
            robot.play_music(win,100);
            robot.lcd_clear();
            robot.lcd_goto_xy(0,0);
            robot.lcd_print("  ENTER   ", 10);
            robot.lcd_goto_xy(0,1);
            robot.lcd_print("=restart", 10);
            t_L.start();
            while( button_enter.read() == 1 ) {
                if(t_L > 9.5) {
                    t_L.stop();
                    robot.play_music(win,100);
                    t_L.reset();
                    t_L.start();
                }
                leds = 0b1001;
                wait(0.2);
                leds = 0b0110;
                wait(0.2);
            }
            robot.play_music("  ccgg",7);
        } else {
            robot.play_music(victory,22);
            leds = 0b1001;
            wait(0.2);
            leds = 0b0110;
            wait(0.2);
        }
                
        wait(2);
        
        t_restart = true;
        
        goal_path_explored(_switch);
        
        while(1) {
        
            if ( t_restart ){                // only start the timer if it isnt already started
                t_coord.start();
                t_restart = false;
            }
            
            follow_line();
            
            if( junction_detect() ) {
                float time = t_coord.read();
                int dist_est = ceil(time*2); // scaled so that a longer straight will have a different time to a shorter straight
                t_coord.stop();
                t_coord.reset();
                t_restart = true; //restart the timer next loop
                
                if (dir == 'N'){ curr_coords[1] += dist_est; }  // y coord
                if (dir == 'E'){ curr_coords[0] += dist_est; }  // x coord
                if (dir == 'S'){ curr_coords[1] -= dist_est; }
                if (dir == 'W'){ curr_coords[0] -= dist_est; }
                
                update_index();
                
                if(point[curr_index] == 100 && _switch == false) { _switch = true; break;}
                if(point[curr_index] == 0 && _switch == true) { _switch = false; break;}
                
                choose_turn(); //looks at the point we are at, examines the type vs explored and makes the appropriate turn also updates the explored
            
            }
        }
        
        turn_select('B');
        robot.stop();
    
        if( dir == 'N' ) { dir = 'S'; }
        else if( dir == 'S' ) { dir = 'N'; }
        else if( dir == 'E' ) { dir = 'W'; }
        else if( dir == 'W' ) { dir = 'E'; }
        
    }  
}

void goal_path_explored( bool inverse )
{
    char dir_diff;
    
    if ( inverse ) {
        for(int j = goal_length1; j >= 0; j--) {
            int curr_node = path_to_point_index(goal_path1[j]);
            int next_node = path_to_point_index(goal_path1[j-1]);
            if(coords_x[next_node] != coords_x[curr_node]) {
                if(coords_x[next_node] - coords_x[curr_node] > 0){
                    dir_diff = 'E';
                } else {
                    dir_diff = 'W';
                }
            } else if( coords_y[next_node] != coords_y[curr_node] ) {
                if(coords_y[next_node] - coords_y[curr_node] > 0){
                    dir_diff = 'N';
                } else {
                    dir_diff = 'S';
                }
            }
            
            if( dir_diff == 'N' ) { 
                explored[curr_node] &= 0b1011;
            }
            else if( dir_diff == 'E' ) { 
                explored[curr_node] &= 0b1101; 
            }
            else if( dir_diff == 'S' ) { 
                explored[curr_node] &= 0b1110;
            }
            else if( dir_diff == 'W' ) { 
                explored[curr_node] &= 0b0111;
            }
        }
    }
    
    else{
        
        for(int j = 0; j <= goal_length1; j++) {
            int curr_node = path_to_point_index(goal_path1[j]);
            int next_node = path_to_point_index(goal_path1[j+1]);
            if(coords_x[next_node] != coords_x[curr_node]) {
                if(coords_x[next_node] - coords_x[curr_node] > 0){
                    dir_diff = 'E';
                } else {
                    dir_diff = 'W';
                }
            } else if( coords_y[next_node] != coords_y[curr_node] ) {
                if(coords_y[next_node] - coords_y[curr_node] > 0){
                    dir_diff = 'N';
                } else {
                    dir_diff = 'S';
                }
            }
            
            if( dir_diff == 'N' ) { 
                explored[curr_node] &= 0b1011;
            }
            else if( dir_diff == 'E' ) { 
                explored[curr_node] &= 0b1101; 
            }
            else if( dir_diff == 'S' ) { 
                explored[curr_node] &= 0b1110;
            }
            else if( dir_diff == 'W' ) { 
                explored[curr_node] &= 0b0111;
            }
        }
    }
}

void invert_path()
{
    // only call once then can use infinitely
    for( int i = 0; i < path_length; i++ ){
        if ( path[path_length-1-i] == 'L' ) { inv_path[i] = 'R'; }
        else if ( path[path_length-1-i] == 'R' ) { inv_path[i] = 'L'; }
        else { inv_path[i] = path[path_length-1-i]; }
    }
}

void print_data(char *word)
{
    robot.lcd_clear();
    robot.lcd_print(word,10);
    robot.stop();
    wait(2);
    
    char buffer1[2];
    char buffer2[2];
    char buffer3[2];
    robot.lcd_clear();
    sprintf(buffer1,"%x",type[curr_index]);
    sprintf(buffer2,"%x",explored[curr_index]);
    
//    sprintf(buffer1,"%d",curr_coords[0]);
//    sprintf(buffer2,"%d",curr_coords[1]);
    sprintf(buffer3,"%d",curr_index);
    robot.lcd_print(buffer1,2); 
    robot.lcd_goto_xy(0,1);
    robot.lcd_print(buffer2,2); 
    robot.lcd_goto_xy(5,0);
    char *b = &dir;
    robot.lcd_print(b,2); 
    robot.lcd_goto_xy(5,1);
    robot.lcd_print(buffer3,2);
    
    robot.stop();
    wait(2);
}
