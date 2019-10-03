CMAKE_MINIMUM_REQUIRED(VERSION 3.1)
PROJECT(dqrobotics-interface-vrep)
set (CMAKE_CXX_STANDARD 11)

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
    src/dqrobotics/interfaces/vrep/DQ_VrepInterface.cpp
    src/dqrobotics/interfaces/vrep/DQ_VrepInterfaceMapElement.cpp
    src/dqrobotics/interfaces/vrep/DQ_VrepRobot.cpp
    src/dqrobotics/interfaces/vrep/robots/LBR4pVrepRobot.cpp
    src/dqrobotics/interfaces/vrep/robots/YouBotVrepRobot.cpp
    coppeliarobotics/remoteApi/extApi.c
    coppeliarobotics/remoteApi/extApiPlatform.c
    )

SET_TARGET_PROPERTIES(${PROJECT_NAME} 
    PROPERTIES PUBLIC_HEADER
    "include/dqrobotics/interfaces/vrep/DQ_VrepInterface.h"
    )

INSTALL(TARGETS ${PROJECT_NAME} 
    LIBRARY DESTINATION "lib"
    PUBLIC_HEADER DESTINATION "include/dqrobotics/interfaces/vrep"
    PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)

################################################################
# INSTALL HEADERS IN SUBFOLDERS
################################################################

# Other Headers
INSTALL(FILES
    include/dqrobotics/interfaces/vrep/DQ_VrepRobot.h
    include/dqrobotics/interfaces/vrep/DQ_VrepInterfaceMapElement.h
    DESTINATION "include/dqrobotics/interfaces/vrep")

# Robots
INSTALL(FILES
    include/dqrobotics/interfaces/vrep/robots/LBR4pVrepRobot.h
    include/dqrobotics/interfaces/vrep/robots/YouBotVrepRobot.h
    DESTINATION "include/dqrobotics/interfaces/vrep/robots")

################################################################
# INSTALL SOURCE FILES (So that the debugger can find them)
################################################################

# base folder
INSTALL(FILES 
    src/dqrobotics/interfaces/vrep/DQ_VrepInterface.cpp
    src/dqrobotics/interfaces/vrep/DQ_VrepInterfaceMapElement.cpp
    src/dqrobotics/interfaces/vrep/DQ_VrepRobot.cpp
    DESTINATION "src/dqrobotics/interfaces/vrep")

# robots folder
INSTALL(FILES
    src/dqrobotics/interfaces/vrep/robots/LBR4pVrepRobot.cpp
    src/dqrobotics/interfaces/vrep/robots/YouBotVrepRobot.cpp
    DESTINATION "src/dqrobotics/interfaces/vrep/robots")
