#ifndef _DEBUGFILE_HPP_
#define _DEBUGFILE_HPP_

#include <fstream>
#include <sstream>
#include <iostream>
#include <time.h>
#include <string.h>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <control/pathfollowing.hpp>

//debug文件路径
#define FILE_PATH  "./data/"

void GetFile();
void Debug(std::shared_ptr<class controller> node);
void CloseFile();
void OutputRingBuf();

#endif