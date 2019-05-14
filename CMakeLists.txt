CMAKE_MINIMUM_REQUIRED(VERSION 3.1)
PROJECT(dqrobotics-interface-vrep)
set (CMAKE_CXX_STANDARD 11)

#ADD_DEFINITIONS(-g -O2 -Wall)
FIND_PACKAGE(Eigen3 REQUIRED)

add_definitions(-DNON_MATLAB_PARSING)
add_definitions(-DMAX_EXT_API_CONNECTIONS=255)
add_definitions(-DDO_NOT_USE_SHARED_MEMORY)

INCLUDE_DIRECTORIES(EIGEN3_INCLUDE_DIR)
INCLUDE_DIRECTORIES(${PROJECT_NAME} include)
INCLUDE_DIRECTORIES(${PROJECT_NAME} coppeliarobotics/remoteApi)
INCLUDE_DIRECTORIES(${PROJECT_NAME} coppeliarobotics/include)
INCLUDE_DIRECTORIES(${PROJECT_NAME} cpp/include)

################################################################
# DEFINE AND INSTALL LIBRARY AND INCLUDE FOLDER
################################################################

ADD_LIBRARY(${PROJECT_NAME} SHARED 
    src/vrep_interface.cpp
    coppeliarobotics/remoteApi/extApi.c
    coppeliarobotics/remoteApi/extApiPlatform.c
    )

SET_TARGET_PROPERTIES(${PROJECT_NAME} 
    PROPERTIES PUBLIC_HEADER
    include/dqrobotics/interfaces/vrep_interface.h
    )

INSTALL(TARGETS ${PROJECT_NAME} 
    LIBRARY DESTINATION "lib"
    PUBLIC_HEADER DESTINATION "include/dqrobotics/interfaces"
    PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)

################################################################
# INSTALL SOURCE FILES (So that the debugger can find them)
################################################################

# base folder
INSTALL(FILES 
    src/vrep_interface.cpp
    DESTINATION "src/dqrobotics/interfaces")
