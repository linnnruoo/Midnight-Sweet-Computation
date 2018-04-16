#ifndef __EXTRA__
#define __EXTRA__

#include <iostream>
#include <stack>
#include <ctype.h>

int startBacktrack = 0;

stack<string> backtrackstack; //store the packet
stack<char> movementstack; //store the commands

//to check for valid input
bool is_number(const string& s){
    
    return !s.empty() && find_if(s.begin(), s.end(), [](char c) { return !isdigit(c); }) == s.end();
}


#endif