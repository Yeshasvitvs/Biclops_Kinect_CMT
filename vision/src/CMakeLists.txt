cmake_minimum_required(VERSION 2.8.3)
project(cmt_code)


## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)
find_package(Qt4 REQUIRED COMPONENTS
  QtCore
  QtGui
)



include_directories(${PROJECT_SOURCE_DIR}
    
)



## Declare a cpp executable
add_executable(cmt	main.cpp
                        CMT.cpp                              
)




## Specify libraries to link a library or executable target against
target_link_libraries(cmt
  
  ${OpenCV_LIBS}
  
)





