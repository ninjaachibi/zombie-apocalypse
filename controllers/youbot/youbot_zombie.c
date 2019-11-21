#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <youbot_zombie.h>

void add(int i)
{
	int j = i;
}

void print_health_energy(struct Robot robot_info)
{
  printf("HEALTH: %d, ENERGY: %d \n", robot_info.health, robot_info.energy);
}
