/*
 * main.cpp
 *
 *  Created on: Dec 21, 2017
 *      Author: tapgar
 */

#include "Planner.h"

int main() {

	Planner planner;

	if (!planner.Init())
	{
		printf("planner init failed... quitting\n");
		return -1;
	}

	while (true)
		planner.Run();

	return 0;

}



