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

// typedef struct BerryScore {
//   int red_count;
//   int yellow_count;
//   int orange_count;
//   int pink_count;
// } BerryScore;

typedef struct Berry {
  int score;
  berry_colors_t color;
} Berry;

/**
 * helper functions
*/
// returns 1 if above berry at (b_x, b_y) is above a stump
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
          if ( ((r > 50 && g-5 < r && r < g+5) && (g > b-15)) ||
               ((60 < r && r < 70) && (60 < g && g < 70) && (60 < b && b < 70)) ||
               ((65 < r && r < 75) && (70 < g && g < 80) && (90 < b && b < 100)) ||
               ((203 < r && r < 220) && (203 < g && g < 220) && (203 < b && b < 220)) )
               { wall++; }
          if ( ((5 < r && r < 15) && (5 < g && g < 15) && (8 < b && b < 19)) ||
               ((20 < r && r < 35) && (20 < g && g < 35) && (20 < b && b < 35)) )
               { black++; }
    }
  }
  // printf("total=%d, \nzombies: green=%d, blue=%d, aqua=%d, purple=%d, \nberries: red=%d, yellow=%d, orange=%d, pink=%d, \nmid_berries: red=%d, yellow=%d, orange=%d, pink=%d, \nobstacles: wall=%d, black=%d\n", total, green, blue, aqua, purple, red, yellow, orange, pink, mid_red, mid_yellow, mid_orange, mid_pink, wall, black);
  
  struct Colors res = {
    .total=total,
    .green=green,
    .blue=blue,
    .aqua=aqua*0.7,
    .purple=purple*1.6,
    
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

// returns 1 if too close to obstacle (-> DEFINITELY TURN OR GO BACKWARDS)
int near_obstacle(struct Colors c, const unsigned char *image)
{
  if (c.black > 1000 || c.wall > 5400)
  {
    return 1;
  }
  
  int vert_strip = 0;
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
        vert_strip = 1;
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
            vert_strip = 0;
            break;
          }
        }
        if (vert_strip == 1)
          { return 1; }
      }
  }
  int horiz_strip = 0;
  for (int y = 20; y < 64; y+=1)  // if a pixel is cliff wall, check if horizontal strip near bottom is cliff wall
  {
    int r = wb_camera_image_get_red(image, 128, 25, y);
    int g = wb_camera_image_get_green(image, 128, 25, y);
    int b = wb_camera_image_get_blue(image, 128, 25, y);
    
    if ((60 < r && r < 80) && (60 < g && g < 80) && (60 < b && b < 80))
      {
        horiz_strip = 1;
        for (int x = 0; x < 120; x+=1) 
        {
          int r = wb_camera_image_get_red(image, 128, x, y);
          int g = wb_camera_image_get_green(image, 128, x, y);
          int b = wb_camera_image_get_blue(image, 128, x, y);
          
          if (!((60 < r && r < 80) && (60 < g && g < 80) && (60 < b && b < 75)))
          {
            horiz_strip = 0;
            break;
          }
        }
        if (horiz_strip == 1)
          { return 1; }
      }
  }
  return 0;
}


// used for debugging color_seen function
void print_Colors(struct Colors c)
{
    printf("total=%d, \nzombies: green=%d, blue=%d, aqua=%d, purple=%d, \nberries: red=%d, yellow=%d, orange=%d, pink=%d, \nmid_berries: red=%d, yellow=%d, orange=%d, pink=%d, \nobstacles: wall=%d, black=%d\n", 
          c.total, c.green, c.blue, c.aqua, c.purple, c.red, c.yellow, c.orange, c.pink, c.mid_red, c.mid_yellow, c.mid_orange, c.mid_pink, c.wall, c.black);
}

int threshold = 150;
int side_threshold = 75;

  // print_Colors(right_colors);
  // if (near_obstacle(front_colors, front_image))
  //    {printf("OBSTACLE\nwall=%d, black=%d\n", front_colors.wall, front_colors.black);}

// Could have different threshold for aqua vs others
int clear(Colors colors, int threshold) {
  if ((colors.green < threshold) && (colors.aqua < threshold) && (colors.purple < threshold) && (colors.blue < threshold)) {
     return 1;
  }
  return 0;
}

void wrap_turn_left (int *turning, int *start_turning_i, int i) {
  *turning = 1;
  *start_turning_i = i;
  stop();
  turn_left();
}

void wrap_turn_right (int *turning, int *start_turning_i, int i) {
  *turning = 1;
  *start_turning_i = i;
  stop();
  turn_right();
}

  
/**
 * returns an array of booleans in the order [red,yellow,orange,pink]
 * that indicates if the given berry color exists in the image
 * ex:[0,0,0,0] means that there are no berries in the image
*/
int *getBerriesInImage(Colors color_map) {
  int *berries_in_image;
  berries_in_image = malloc(sizeof(int) * 4);

  berries_in_image[0] = color_map.red > 10;
  berries_in_image[1] = color_map.yellow > 10;
  berries_in_image[2] = color_map.orange > 10;
  berries_in_image[3] = color_map.pink > 10; 

  return berries_in_image;
}

int *getBerriesInMidImage(Colors color_map) {
  int *berries_in_image;
  berries_in_image = malloc(sizeof(int) * 4);

  berries_in_image[0] = color_map.mid_red > 10;
  berries_in_image[1] = color_map.mid_yellow > 10;
  berries_in_image[2] = color_map.mid_orange > 10;
  berries_in_image[3] = color_map.mid_pink > 10;

  return berries_in_image;
}


/**
 * Returns 1 if the list has berries, 0 otherwise
*/
int hasAnyBerries(int *berryList) {
  for (int i = 0; i < 4; i++) {
    if (berryList[i] > 0) {
      return 1;
    }
  }
  return 0;
}

int hasBerryColor(int *berryList, Berry berry) {
  berry_colors_t color = berry.color;
  return berryList[color];
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


  // int i = 0;


  robot_states_t State = GET_BERRY;

  /* initialize the scores for berries priorities */
  Berry berryScores[4] = {
    {.color = RED, .score = 0},
    {.color = YELLOW, .score = 0},
    {.color = ORANGE, .score = 0},
    {.color = PINK, .score = 0}
  }; 
  // order is [red,yellow,orange,pink]


  int losing_health = 0;
  int prev_health = 100;
  int i = 0;
  int turning = 0;
  int start_turning_i = 0;
  int lateral_berryflag = 0;
  int starting_i = 0; // for going forward after we see a berry on lateral sides
  int wasBerryInView = 0;
  int berryInViewTimer = 0;
  int wasBerryInBackView = 0;
  int berryInViewBackTimer = 0;
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
    if (timer == 16)
    {
      if (robot_info.health < prev_health) {
        prev_health = robot_info.health;
        losing_health = 1;
      }
      else {
        losing_health = 0;
      }
    }

    i++;
    if (turning == 1)
    {
      if (i - start_turning_i < 180)
         {continue;}
      else
        {turning = 0;
         continue;}
    }

    if (lateral_berryflag == 1 && i - starting_i > 400)
    {
      go_forward();
      lateral_berryflag = 0;
    }

    if (wasBerryInView == 1 && i - berryInViewTimer > 300)
    {
        wasBerryInView = 0;
    }

    if (wasBerryInBackView == 1 && i - berryInViewBackTimer > 300)
    {
      wasBerryInBackView = 0;
    }

    /* Finite State Machine */
    switch (State)
    {
    case AVOID_OBSTACLE:
      /* code */
      break;

    case AVOID_ZOMBIE:;
      /* code */
      const unsigned char *front = wb_camera_get_image(4);
      const unsigned char *back = wb_camera_get_image(8);
      const unsigned char *right = wb_camera_get_image(9);
      const unsigned char *left = wb_camera_get_image(10);

      Colors front_cols = color_seen(front);
      Colors back_cols = color_seen(back);
      Colors right_cols = color_seen(right);
      Colors left_cols = color_seen(left);

      // print_Colors(back_cols);

      // If losing health, drop everything and move forward or backward
      if (losing_health && (robot_info.energy != 0))
      {
        if (near_obstacle(front_cols, front) || (!clear(front_cols, threshold)))
        {
          go_backward();
        }
        else
        {
          go_forward();
        }
      }

      // corners
      else if ((near_obstacle(front_cols, front) && near_obstacle(right_cols, right)) ||
                (near_obstacle(front_cols, front) && near_obstacle(left_cols, left)) ||
                (near_obstacle(back_cols, back) && near_obstacle(left_cols, left)) ||
                (near_obstacle(back_cols, back) && near_obstacle(right_cols, right)))
      {
        if (near_obstacle(front_cols, front) && near_obstacle(right_cols, right))
        {
          wrap_turn_right(&turning, &start_turning_i, i);
        }
        else if (near_obstacle(front_cols, front) && near_obstacle(left_cols, left))
        {
          wrap_turn_left(&turning, &start_turning_i, i);
        }
        else if (near_obstacle(back_cols, back) && near_obstacle(left_cols, left))
        {
          wrap_turn_right(&turning, &start_turning_i, i);
        }
        else
        {
          wrap_turn_left(&turning, &start_turning_i, i);
        }
      }

      // Both no zombies and not obstacles so stop
      else if ((clear(front_cols, threshold) && clear(back_cols, threshold) && clear(right_cols, side_threshold) && clear(left_cols, side_threshold)) &&
                (!near_obstacle(front_cols, front) && !near_obstacle(back_cols, back) && !near_obstacle(right_cols, right) && !near_obstacle(left_cols, left)))
      {
        // pass in robot energy and put threshold here?
        stop();

        /**
       *  go to GET_BERRY state is below threshold
       * */
        if (robot_info.energy < 30)
        {
          State = GET_BERRY;
          printf("Changing State to: GET_BERRY\n");
        }
      }

      // clear of all zombies but there's there an obstacle on one side.
      else if ((clear(front_cols, threshold) && clear(back_cols, threshold) && clear(right_cols, side_threshold) && clear(left_cols, side_threshold)))
      {
        if (near_obstacle(front_cols, front) && near_obstacle(back_cols,back)) 
        {
          wrap_turn_right(&turning, &start_turning_i, i);
          printf("avoiding front and back obstacles\n");
        }
        else if (near_obstacle(front_cols, front))
        {
          go_backward();
          printf("avoiding front obstacle\n");
        }
        else if (near_obstacle(back_cols, back) || near_obstacle(left_cols, left) || near_obstacle(right_cols, right))
        {
          go_forward();
          printf("avoiding back,left,right obstacle\n");
        }
      }

      // Front is clear but another direction isn't
      else if (clear(front_cols, threshold))
      {
        if (!near_obstacle(front_cols, front))
        {
          go_forward();
        }
        else
        {
          if (clear(back_cols, threshold))
          {
            go_backward();
          }
          else if (clear(right_cols, side_threshold) && clear(left_cols, side_threshold))
          {
            // choose direction for now to see if it works
            // berry_finder() to determine turn direction
            wrap_turn_right(&turning, &start_turning_i, i);
          }
          else if (clear(right_cols, side_threshold))
          {
            wrap_turn_right(&turning, &start_turning_i, i);
          }
          // left clear
          else
          {
            wrap_turn_left(&turning, &start_turning_i, i);
          }
        }
      }

      // Front is not clear
      else
      {
        if (clear(back_cols, threshold) && (!near_obstacle(back_cols, back)))
        {
          // no need to see if we should turn instead of going back?
          go_backward();
        }
        else if (clear(right_cols, side_threshold) && clear(left_cols, side_threshold))
        {
          // choose direction for now to see if it works
          // berry_finder() to determine turn direction

          int *berriesRightMiddleList = getBerriesInMidImage(right_cols);
          if (hasAnyBerries(berriesRightMiddleList))
          {
            wrap_turn_right(&turning, &start_turning_i, i);
          }
          else
          {
            wrap_turn_left(&turning, &start_turning_i, i);
          }

          free(berriesRightMiddleList);
        }
        else if (clear(right_cols, side_threshold))
        {
          wrap_turn_right(&turning, &start_turning_i, i);
        }
        else if (clear(left_cols, side_threshold))
        {
          wrap_turn_left(&turning, &start_turning_i, i);
        }
        // nothing clear, find least bad path
        // is it ever worth to turn left or right instead of going forward/backward?
        else
        {
          if (near_obstacle(front_cols, front))
          {
            go_backward();
          }
          else if (near_obstacle(back_cols, back))
          {
            go_forward();
          }
          else if ((back_cols.aqua < threshold) && (front_cols.aqua < threshold))
          {
            if ((back_cols.purple < threshold) && (front_cols.purple < threshold))
            {
              if ((back_cols.green < threshold) && (front_cols.green < threshold))
              {
                // can't both be clear because surrounded on all sides
                if (back_cols.blue < threshold)
                {
                  go_backward();
                }
                else
                {
                  go_forward();
                }
              }
              else if (back_cols.green < threshold)
              {
                go_backward();
              }
              else
              {
                go_forward();
              }
            }
            else if (back_cols.purple < threshold)
            {
              go_backward();
            }
            else
            {
              go_forward();
            }
          }
          else if (back_cols.aqua < threshold)
          {
            go_backward();
          }
          else
          {
            go_forward();
          }
        }
      }
      break;

    case GET_BERRY:;
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
     */

      const unsigned char *frontImage = wb_camera_get_image(4);
      const unsigned char *backImage = wb_camera_get_image(8);
      const unsigned char *rightImage = wb_camera_get_image(9);
      const unsigned char *leftImage = wb_camera_get_image(10);

      Colors frontColors = color_seen(frontImage);
      Colors backColors = color_seen(backImage);
      Colors rightColors = color_seen(rightImage);
      Colors leftColors = color_seen(leftImage);

      /* check that no zombies */
      /* checks that if we're near an obstacle on any side*/
      if (!(clear(rightColors, side_threshold) && clear(leftColors, side_threshold) && clear(frontColors, threshold) && clear(backColors, threshold)) ||
          (near_obstacle(frontColors, frontImage) || near_obstacle(backColors, backImage) || near_obstacle(rightColors, rightImage) || near_obstacle(leftColors, leftImage)) ||
          (losing_health && robot_info.energy != 0))
      {
        // go to avoid
        State = AVOID_ZOMBIE;
        printf("CHANGING STATE TO: AVOID_ZOMBIE\n");
      }

      /* get berry */
      int *berriesFrontList = getBerriesInImage(frontColors);
      int *berriesBackList = getBerriesInImage(backColors);
      int *berriesRightList = getBerriesInImage(rightColors);
      int *berriesLeftList = getBerriesInImage(leftColors);

      int *berriesRightMiddleList = getBerriesInMidImage(rightColors);
      int *berriesLeftMiddleList = getBerriesInMidImage(leftColors);
      int *berriesFrontMiddleList = getBerriesInMidImage(frontColors);
      int *berriesBackMiddleList = getBerriesInMidImage(backColors);

      int move_flag = 0;
      for (int j = 0; j < 4; j++)
      {
        // get this ranked berry
        Berry berry = berryScores[j];

        if (hasBerryColor(berriesFrontMiddleList, berry) || wasBerryInView)
        {
          if (wasBerryInView == 0)
          {
            berryInViewTimer = i;
          }
          wasBerryInView = 1;
          move_flag = 1;
          go_forward();
          printf("berry in front, %d, berry=%d\n", i, berry.color);
          break;
        }
        else if (hasBerryColor(berriesBackMiddleList, berry) || wasBerryInBackView)
        {
          if (wasBerryInBackView == 0)
          {
            berryInViewBackTimer = i;
          }
          wasBerryInBackView = 1;
          move_flag = 1;
          go_backward();
          printf("berry in back, %d, berry=%d\n", i, berry.color);
          break;
        }
        else if (!lateral_berryflag)
        {
          // for these, check if they're in the middle of the frame
          if (hasBerryColor(berriesRightMiddleList, berry))
          {
            move_flag = 1;
            wrap_turn_right(&turning, &start_turning_i, i);
            printf("berry in right, %d, berry=%d\n", i, berry.color);

            // don't turn for the next 200 time steps
            lateral_berryflag = 1;
            starting_i = i;

            break;
          }
          else if (hasBerryColor(berriesLeftMiddleList, berry))
          {
            move_flag = 1;
            wrap_turn_right(&turning, &start_turning_i, i);
            printf("berry in left, %d, berry=%d\n", i, berry.color);

            // don't turn for the next 200 time steps
            lateral_berryflag = 1;
            starting_i = i;

            break;
          }
        }
      }

      //default behavior - don't move
      if (move_flag == 0)
      {
        printf("berry default: go backward, i=%d\n", i);
        go_backward();
      }

      // if we hit a berry, update our score table

      /**
      * TODO: if we see a zombie or lose health, go to avoid zombie state
      */

      free(berriesFrontList);
      free(berriesBackList);
      free(berriesRightList);
      free(berriesLeftList);
      free(berriesLeftMiddleList);
      free(berriesRightMiddleList);

      break;

    default:
      break;
    }



    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////// CHANGE CODE ABOVE HERE ONLY ////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
  }

  wb_robot_cleanup();

  return 0;
}
