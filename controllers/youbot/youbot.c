/*
 * Copyright 1996-2019 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:   Starts with a predefined behaviors and then
 *                read the user keyboard inputs to actuate the
 *                robot
 */
 
#include <webots/keyboard.h>            
#include <webots/robot.h>
#include <webots/supervisor.h>

#include <arm.h>
#include <base.h>
#include <gripper.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h> 

#include <webots/camera.h>
#include <webots/accelerometer.h>
#include <webots/lidar.h>
#include <webots/compass.h>
#include <webots/gps.h>
#include <webots/range_finder.h>
#include <webots/gyro.h>
#include <webots/light_sensor.h>
#include <webots/receiver.h>
#include <webots/distance_sensor.h>

#include <youbot_zombie_1.h>


int robot_angle = 0;
#define TIME_STEP 32

//////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////// ONLY USE THE FOLLOWING FUNCTIONS TO MOVE THE ROBOT /////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////


void stop()
{
	base_reset();
}

void go_forward()
{
	base_forwards();
}

void go_backward()
{
	base_backwards();
}

void turn_left()
{
	base_turn_left();
	robot_angle = robot_angle + 90;
	if (robot_angle == 360)
		robot_angle = 0;

}

void turn_right()
{
	base_turn_right();	
	robot_angle = robot_angle - 90;
	if (robot_angle == -90)
		robot_angle = 270;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////// CHANGE CODE BELOW HERE ONLY ////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * typedefs go here
*/
typedef struct Colors {
  unsigned int total;
  unsigned int green;
  unsigned int blue;
  unsigned int aqua;
  unsigned int purple;
  
  unsigned int red;
  unsigned int yellow;
  unsigned int orange;
  unsigned int pink;
  unsigned int mid_red;
  unsigned int mid_yellow;
  unsigned int mid_orange;
  unsigned int mid_pink;
  
  unsigned int wall;
  unsigned int black;
  
} Colors;

typedef enum
{
  AVOID_ZOMBIE,
  AVOID_OBSTACLE,
  GET_BERRY,
} robot_states_t;

typedef enum {
  RED,
  YELLOW, 
  ORANGE,
  PINK
} berry_colors_t;


int above_stump(int b_x, int b_y, const unsigned char *image)
{
  for (int temp_y = b_y; temp_y < 64; temp_y+=1)
  {
    int r = wb_camera_image_get_red(image, 128, b_x, temp_y);
    int g = wb_camera_image_get_green(image, 128, b_x, temp_y);
    int b = wb_camera_image_get_blue(image, 128, b_x, temp_y);
    if ( ((5 < r && r < 15) && (5 < g && g < 15) && (8 < b && b < 19)) ||
         ((20 < r && r < 35) && (20 < g && g < 35) && (20 < b && b < 35)) )
         { return 1; }
  }
  return 0;
}

// function that counts colored pixels
struct Colors color_seen(const unsigned char *image)
{
  int total = 0;
  int green = 0;
  int blue = 0;
  int aqua = 0;
  int purple = 0;
  
  int red = 0;
  int yellow = 0;
  int orange = 0;
  int pink = 0;
  int mid_red = 0;
  int mid_yellow = 0;
  int mid_orange = 0;
  int mid_pink = 0;
  
  int wall = 0;
  int black = 0;
  
  for (int x = 0; x < 128; x+=1)
  {
    for (int y = 0; y < 64; y+=1) 
    {

        total++;
        int r = wb_camera_image_get_red(image, 128, x, y);
        int g = wb_camera_image_get_green(image, 128, x, y);
        int b = wb_camera_image_get_blue(image, 128, x, y);
        
          // zombies    
          if ( ((15 < r && r < 25) && (115 < g && g < 145) && (15 < b && b < 25)) ||
               ((15 < r && r < 25) && (85 < g && g < 100) && (15 < b && b < 25)) ||
               ((28 < r && r < 45) && (190 < g && g < 210) && (30 < b && b < 45)) ||
               ((7 < r && r < 14) && (45 < g && g < 57) && (8 < b && b < 19)) )
               { green++; }
          if ( ((5 < r && r < 15) && (30 < g && g < 50) && (85 < b && b < 108)) ||
               ((18 < r && r < 40) && (114 < g && g < 150) && (205 < b && b < 244)) )
               { blue++; }
          if ( ((7 < r && r < 16) && (59 < g && g < 74) && (59 < b && b < 74)) ||
               ((30 < r && r < 49) && (170 < g && g < 180) && (140 < b && b < 155)) ||
               ((30 < r && r < 49) && (180 < g && g < 195) && (157 < b && b < 173)) ||
               ((30 < r && r < 49) && (200 < g && g < 240) && (190 < b && b < 220)) )
               { aqua++; }
          if ( ((145 < r && r < 25) && (115 < g && g < 145) && (15 < b && b < 25)) ||
               ((43 < r && r < 65) && (17 < g && g < 30) && (90 < b && b < 130)) ||
               ((110 < r && r < 130) && (40 < g && g < 56) && (180 < b && b < 200)) )
               { purple++; } 
                
          // berries      
          if (!above_stump(x, y, image))
          {
            if ( ((60 < r && r < 78) && (13 < g && g < 28) && (13 < b && b < 28)) ||
                 ((190 < r && r < 225) && (53 < g && g < 66) && (37 < b && b < 49)) )
                  { red++; 
                    if (54 < x && x < 74)
                      {mid_red++;}
                  }
            if ( ((202 < r && r < 220) && (190 < g && g < 205) && (25 < b && b < 36)) ||
                  ((65 < r && r < 75) && (63 < g && g < 74) && (8 < b && b < 18)) )
                  { yellow++; 
                    if (54 < x && x < 74)
                      {mid_yellow++;}
                  }
            if ( ((188 < r && r < 200) && (117 < g && g < 129) && (77 < b && b < 89)) ||
                  ((55 < r && r < 68) && (33 < g && g < 43) && (28 < b && b < 37)) )
                  { orange++; 
                    if (54 < x && x < 74)
                      {mid_orange++;}
                  }
            if ( ((188 < r && r < 200) && (117 < g && g < 129) && (162 < b && b < 175)) ||
                  ((56 < r && r < 68) && (33 < g && g < 44) && (62 < b && b < 73)) )
                  { pink++; 
                     if (54 < x && x < 74)
                       {mid_pink++;}
                  }
          }
              
          // obstacles  
          if ( 
          // ((r > 50 && g-5 < r && r < g+5) && (g > b-15)) ||
               // ((60 < r && r < 70) && (60 < g && g < 70) && (60 < b && b < 70)) ||
               ((65 < r && r < 75) && (70 < g && g < 80) && (90 < b && b < 100)))
               // ((203 < r && r < 220) && (203 < g && g < 220) && (203 < b && b < 220)) )
               { wall++; }
          if ( ((5 < r && r < 15) && (5 < g && g < 15) && (8 < b && b < 19)) ||
               ((20 < r && r < 35) && (20 < g && g < 35) && (20 < b && b < 35)) )
               { black++; }
    }
  }
  // printf("total=%d, \nzombies: green=%d, blue=%d, aqua=%d, purple=%d, \nberries: red=%d, yellow=%d, orange=%d, pink=%d, \nmid_berries: red=%d, yellow=%d, orange=%d, pink=%d, \nobstacles: wall=%d, black=%d\n", 
          // total, green, blue, aqua, purple, red, yellow, orange, pink, mid_red, mid_yellow, mid_orange, mid_pink, wall, black);
  
  struct Colors res = {
    .total=total,
    .green=green,
    .blue=blue,
    .aqua=aqua,
    .purple=purple,
    
    .red=red,
    .yellow=yellow,
    .orange=orange,
    .pink=pink,
    .mid_red=mid_red,
    .mid_yellow=mid_yellow,
    .mid_orange=mid_orange,
    .mid_pink=mid_pink,
    
    .wall=wall,
    .black=black};
  return res;
}

int near_obstacle(struct Colors c, const unsigned char *image)
{
  if (c.black > 1000 || c.wall > 5400)
  {
    return 1;
  }
  
  int solid_strip = 0;
  for (int x = 0; x < 128; x+=1)  // if a pixel is wall, check if whole vertical strip is wall
  {
    int r = wb_camera_image_get_red(image, 128, x, 3);
    int g = wb_camera_image_get_green(image, 128, x, 3);
    int b = wb_camera_image_get_blue(image, 128, x, 3);    
    
    if (((r > 50 && g-5 < r && r < g+5) && (g > b-15)) ||
        ((60 < r && r < 70) && (60 < g && g < 70) && (60 < b && b < 70)) ||
        ((65 < r && r < 75) && (70 < g && g < 80) && (90 < b && b < 100)) ||
        ((203 < r && r < 220) && (203 < g && g < 220) && (203 < b && b < 220)) )
      {
        solid_strip = 1;
        for (int y = 0; y < 50; y+=1) 
        {
          int r = wb_camera_image_get_red(image, 128, x, y);
          int g = wb_camera_image_get_green(image, 128, x, y);
          int b = wb_camera_image_get_blue(image, 128, x, y);
          
          if (!(((r > 50 && g-5 < r && r < g+5) && (g > b-15)) ||
                ((60 < r && r < 70) && (60 < g && g < 70) && (60 < b && b < 70)) ||
                ((65 < r && r < 75) && (70 < g && g < 80) && (90 < b && b < 100)) ||
                ((203 < r && r < 220) && (203 < g && g < 220) && (203 < b && b < 220))))
          {
            solid_strip = 0;
            break;
          }
        }
        if (solid_strip == 1)
          { return 1; }
      }
  }
  return 0;
}

void robot_control()
{
  const unsigned char *front_image = wb_camera_get_image(4);
  const unsigned char *back_image = wb_camera_get_image(8);
  const unsigned char *right_image = wb_camera_get_image(9);
  const unsigned char *left_image = wb_camera_get_image(10);
  
  struct Colors front_colors = color_seen(front_image);
  struct Colors right_colors = color_seen(right_image);

  // print_Colors(right_colors);
       if (near_obstacle(front_colors, front_image))
         {printf("OBSTACLE\nwall=%d, black=%d\n", front_colors.wall, front_colors.black);}
}




void print_Colors(struct Colors c)
{
    printf("total=%d, \nzombies: green=%d, blue=%d, aqua=%d, purple=%d, \nberries: red=%d, yellow=%d, orange=%d, pink=%d, \nmid_berries: red=%d, yellow=%d, orange=%d, pink=%d, \nobstacles: wall=%d, black=%d\n", 
          c.total, c.green, c.blue, c.aqua, c.purple, c.red, c.yellow, c.orange, c.pink, c.mid_red, c.mid_yellow, c.mid_orange, c.mid_pink, c.wall, c.black);
  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////// CHANGE CODE ABOVE HERE ONLY ////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////


int main(int argc, char **argv) 
{

  struct Robot robot_info = {100,100};
  wb_robot_init();
  base_init();
  arm_init();
  gripper_init();
  passive_wait(0.1);

  //display_helper_message();

  int pc = 0;
  wb_keyboard_enable(TIME_STEP);
  int timer = 0;
  
  WbNodeRef robot_node = wb_supervisor_node_get_from_def("Youbot");
  WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");
  
  get_all_berry_pos();
  
  int robot_not_dead = 1;
   
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////// CHANGE CODE BELOW HERE ONLY ////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
    

  wb_camera_enable(4,TIME_STEP); // front of robot
  wb_camera_enable(8,TIME_STEP); // back
  wb_camera_enable(9,TIME_STEP); // right
  wb_camera_enable(10,TIME_STEP); // left


  int i = 0;



  robot_states_t State = GET_BERRY;

  /* initialize the scores for berries priorities */
  int berryScores[4] = {0,0,0,0}; // order is [red,yellow,orange,pink]

  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////// CHANGE CODE ABOVE HERE ONLY ////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  while (robot_not_dead == 1) 
  {

	if (robot_info.health < 0)
    {
		robot_not_dead = 0;
		printf("ROBOT IS OUT OF HEALTH\n");
	}
	
	if (timer % 2 == 0)
	{  
		const double *trans = wb_supervisor_field_get_sf_vec3f(trans_field);
		check_berry_collision(&robot_info, trans[0], trans[2]);
		check_zombie_collision(&robot_info, trans[0], trans[2]);
		//printf("%f\n", trans[0]);
	}
    if (timer == 16)
    {
        update_robot(&robot_info);
        timer = 0;
    }

    step();


    
    int c = keyboard(pc);
    pc = c;
    timer=timer+1;
    
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////// CHANGE CODE BELOW HERE ONLY ////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    // this is called everytime step.

    if (i < 100)
    {
    	base_forwards();
    }
    if (i == 100)
    {
      base_reset();
    	base_turn_left();
    }
    if (i == 300)
    {
    	i = 0;
    }
    i++;

    // call robot control every 10 

    char imageFileName[32];

    if (i % 10 == 0) {
      robot_control();

      // DEBUG: dump image in file
      // sprintf(imageFileName, "image%06d.png", i);
      // wb_camera_save_image(4, imageFileName,100);
    }

    /* Finite State Machine */
    switch (State)
    {
    case AVOID_OBSTACLE:
      /* code */
      break;

    case AVOID_ZOMBIE:
      /* code */
      break;

    case GET_BERRY:
      /**
       * if state is get berry, do berry-getting behavior
       * 
       * global berryPriorityList = []
       * 
       * best_berry = RED;
       * get camera image for front, left, right, back
       * 
       * berries1 = getBerriesinImage(image1) // process if berries present in each image
       * ...
       * berries4 = getBerriesinImage(image4) 
       * 
       * for next highest priority berry in berryPriorityList:
       *  if berry in front or back:
       *    move robot front or backwards:
       *    break;
       *  else if berry left or right:
       *    move robot left or right
       *    break;
       *  
       * 
       * if armor, health, or energy change by enough
       *  update berry priority list
       * 
       * check surroundings
       * update global state
       * */

      
      break;
    default:
      break;
    }


    

    //  if (wb_receiver_get_queue_length(rec) > 0)
  	// {
  	// 	const char *buffer = wb_receiver_get_data(rec);
   //      printf("Communicating: received \"%s\"\n", buffer);
   //  	wb_receiver_next_packet(rec);
   //  }

    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////// CHANGE CODE ABOVE HERE ONLY ////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
  }

  wb_robot_cleanup();

  return 0;
}
