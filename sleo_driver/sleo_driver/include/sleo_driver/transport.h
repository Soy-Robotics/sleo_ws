#include <stdio.h>
#include <iostream>  
#include <sstream> 
using namespace std;  

namespace roboteq{

template<typename T> std::string toString(const T& t){
    ostringstream oss;  
    oss<<t;             
    return oss.str();   
}

}
