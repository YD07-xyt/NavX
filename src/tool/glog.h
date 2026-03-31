#pragma once 

#ifndef GLOG_HPP_
#define GLOG_HPP_

#include <glog/logging.h>
#define RESET "\033[0m"
#define RED "\033[31m"
#define GREEN "\033[32m"
#define YELLOW "\033[33m"
#define BLUE "\033[34m"
#define MAGENTA "\033[35m"
#define CYAN "\033[36m"
#define WHITE "\033[37m"

#define gINFO(msg) LOG(INFO) << GREEN << " ---> "<< msg << RESET
#define gWARNING(msg) LOG(WARNING) << YELLOW << " ---> "<< msg << RESET
#define gERROR(msg) LOG(ERROR) << RED << " ---> "<< msg << RESET


#endif