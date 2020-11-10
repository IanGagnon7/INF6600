#ifndef __OUTPUT_CSV_H__
#define __OUTPUT_CSV_H__

#include <iostream>
#include <fstream>
#include "sys_continu.h"
#include "drone.h"

void csvInit(std::ofstream &file);
void outputLine(std::ofstream &csvFile, unsigned int msTime, SysContinu* sys, struct TaskData* tsk);

#endif
