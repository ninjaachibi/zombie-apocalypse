#include <stdio.h>
#include <stdlib.h>


#define TIME_STEP 32

double berry_pos[20][3];

/* = {
    {1.59, 0.0249509, 3.11978e-18}, //0
    {1.21, 0.0246076, -8.69651e-18}, //1
    {7.72, 0.0249509	, 2.60366e-16}, //2
    {-7.29, 0.0249509, -3.44}, //3
    {1.21, 0.0249509, 5.93}, //4
    {9.04, 0.0249509, -8.69}, //5
    {-6.68, 0.0249509, -2.38}, //6
    {-8.87, 0.0249509, 3.34}, //7
    {-5.05, 0.0249509, 6.77}, //8
    {4.8, 0.0249509, 3.82} //9
};*/

/////////////////////////////// HEALTH AND ENERGY ///////////////////////////////
struct Robot
{
  int health;
  int energy;
};

char* concat(const char *s1, const char *s2)
{
    const size_t len1 = strlen(s1);
    const size_t len2 = strlen(s2);
    char *result = malloc(len1 + len2 + 1); // +1 for the null-terminator
    // in real code you would check for errors in malloc here
    memcpy(result, s1, len1);
    memcpy(result + len1, s2, len2 + 1); // +1 to copy the null-terminator
    return result;
}

void get_all_berry_pos()
{
	for (int i=0; i<20; i++)
	{
		char* str1;
	    char* str2;
	    char snum[5];
	    str1 = "Berry";
	    //str2 = itoa(i, snum, 10);
	    sprintf(str2, "%d", i);
	    char* str3 = concat(str1, str2);
	 
	    WbNodeRef berry_node = wb_supervisor_node_get_from_def(str3);
	    WbFieldRef berry_trans_field = wb_supervisor_node_get_field(berry_node, "translation");
	    const double *berry_trans = wb_supervisor_field_get_sf_vec3f(berry_trans_field);
	    
	    //printf("%f", berry_trans[0]);
	    berry_pos[i][0] = berry_trans[0];
	    berry_pos[i][2] = berry_trans[2];
	}
}

void print_health_energy(struct Robot robot_info)
{
  printf("HEALTH: %d, ENERGY: %d \n", robot_info.health, robot_info.energy);
  
}

void berry_collision(int berry_id, struct Robot *robot_info )
{
  char* str1;
  char* str2;
  char snum[5];
  str1 = "Berry";
  str2 = itoa(berry_id, snum, 10);
  char* str3 = concat(str1, str2);
 
  WbNodeRef berry_node = wb_supervisor_node_get_from_def(str3);
  WbFieldRef berry_trans_field = wb_supervisor_node_get_field(berry_node, "translation");
  const double DESTROY[3] = { 20, 20, 20};
  wb_supervisor_field_set_sf_vec3f(berry_trans_field, DESTROY);
  
  robot_info->energy = robot_info->energy + 40;
  if (robot_info->energy > 100)
  {
	  robot_info->energy = 100;
  } 
}

float absolute(float value)
{
	float sq =  value * value;
	return sqrt(sq);
}

void check_berry_collision(struct Robot *robot_info, const double robot_x, const double robot_z)
{
	for (int i =0; i< 20; i++)
	{
		double berry_x = berry_pos[i][0];
		double berry_z = berry_pos[i][2];
		
		double distance = absolute(berry_x - robot_x) + absolute(berry_z - robot_z);
		if (distance < 0.38)
		{
			berry_collision(i, robot_info);
			berry_pos[i][0] = 40;
			berry_pos[i][2] = 40;
		}
	}
}

void check_zombie_collision(struct Robot *robot_info, const double robot_x, const double robot_z)
{
	
  WbNodeRef zombie1 = wb_supervisor_node_get_from_def("zombie1");
  WbFieldRef zombie1_trans_field = wb_supervisor_node_get_field(zombie1, "translation");
  const double *zombie1_trans = wb_supervisor_field_get_sf_vec3f(zombie1_trans_field);
  
  double zombie1_x = zombie1_trans[0];
  double zombie1_z = zombie1_trans[2];
  
  double distance1 = absolute(zombie1_x - robot_x) + absolute(zombie1_z - robot_z);
  
  if (distance1<0.5)
  {
	  robot_info->health = robot_info->health-1;
  }
  
  WbNodeRef zombie2 = wb_supervisor_node_get_from_def("zombie2");
  WbFieldRef zombie2_trans_field = wb_supervisor_node_get_field(zombie2, "translation");
  const double *zombie2_trans = wb_supervisor_field_get_sf_vec3f(zombie2_trans_field);
  
  double zombie2_x = zombie2_trans[0];
  double zombie2_z = zombie2_trans[2];
  
  double distance2 = absolute(zombie2_x - robot_x) + absolute(zombie2_z - robot_z);
  
  if (distance2<0.5)
  {
	  robot_info->health = robot_info->health-1;
  }
  
  WbNodeRef zombie3 = wb_supervisor_node_get_from_def("zombie3");
  WbFieldRef zombie3_trans_field = wb_supervisor_node_get_field(zombie3, "translation");
  const double *zombie3_trans = wb_supervisor_field_get_sf_vec3f(zombie3_trans_field);
  
  double zombie3_x = zombie3_trans[0];
  double zombie3_z = zombie3_trans[2];
  
  double distance3 = absolute(zombie3_x - robot_x) + absolute(zombie3_z - robot_z);
  
  if (distance3<0.5)
  {
	  robot_info->health = robot_info->health-1;
  }
}

void update_robot(struct Robot *robot_info)
{
	if (robot_info->energy == 0)
	{
		robot_info->health = robot_info->health -1;
    }
	if (robot_info->energy > 0)
    {
		robot_info->energy = robot_info->energy -1;
    }

    print_health_energy(*robot_info);
    

}



///////////////////////    KEYBOARD ////////////////////////////////////////// 

static void display_helper_message() {
  printf("Control commands:\n");
  printf(" Arrows:       Move the robot\n");
  printf(" Page Up/Down: Rotate the robot\n");
  printf(" +/-:          (Un)grip\n");
  printf(" Shift + arrows:   Handle the arm\n");
  printf(" Space: Reset\n");
}

int keyboard(int pc)
{
  int c = wb_keyboard_get_key();
    if ((c >= 0) && c != pc) {
      switch (c) {
        case WB_KEYBOARD_UP:
          //printf("Go forwards\n");
          base_forwards();
          break;
        case WB_KEYBOARD_DOWN:
          //printf("Go backwards\n");
          base_backwards();
          break;
        case WB_KEYBOARD_LEFT:
          //printf("Strafe left\n");
          base_strafe_left();
          break;
        case WB_KEYBOARD_RIGHT:
          //printf("Strafe right\n");
          base_strafe_right();
          break;
        case WB_KEYBOARD_PAGEUP:
          //printf("Turn left\n");
          base_turn_left();
          break;
        case WB_KEYBOARD_PAGEDOWN:
          //printf("Turn right\n");
          base_turn_right();
          break;
        case WB_KEYBOARD_END:
        case ' ':
          //printf("Reset\n");
          base_reset();
          arm_reset();
          break;
        case '+':
        case 388:
        case 65585:
          printf("Grip\n");
          gripper_grip();
          break;
        case '-':
        case 390:
          printf("Ungrip\n");
          gripper_release();
          break;
        case 332:
        case WB_KEYBOARD_UP | WB_KEYBOARD_SHIFT:
          printf("Increase arm height\n");
          arm_increase_height();
          break;
        case 326:
        case WB_KEYBOARD_DOWN | WB_KEYBOARD_SHIFT:
          printf("Decrease arm height\n");
          arm_decrease_height();
          break;
        case 330:
        case WB_KEYBOARD_RIGHT | WB_KEYBOARD_SHIFT:
          printf("Increase arm orientation\n");
          arm_increase_orientation();
          break;
        case 328:
        case WB_KEYBOARD_LEFT | WB_KEYBOARD_SHIFT:
          printf("Decrease arm orientation\n");
          arm_decrease_orientation();
          break;
        default:
          fprintf(stderr, "Wrong keyboard input\n");
          break;
      }
    }
    return c;
}

static void step() {
  if (wb_robot_step(TIME_STEP) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

static void passive_wait(double sec) {
  double start_time = wb_robot_get_time();
  do {
    step();
  } while (start_time + sec > wb_robot_get_time());
}
