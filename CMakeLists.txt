cmake_minimum_required(VERSION 3.8)
project(OrthoDisplaySystem)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON) # For meta object compiler
set(CMAKE_AUTORCC ON) # Resource files
set(CMAKE_AUTOUIC ON) # UI files
#set(CMAKE_FIND_LIBRARY_SUFFIXES ${CMAKE_STATIC_LIBRARY_SUFFIX})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
find_package(Qt5 REQUIRED Widgets)
find_package(VTK REQUIRED)

include_directories(
	${CMAKE_CURRENT_BINARY_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}
 	Data
	GroupFiles
	vtkDerviedClass
)
set( Project_Sources  
	MainWindow.cpp
	main.cpp
 	Data/OrthoData.cpp 
	Data/Gum.cpp  
	Data/Arch.cpp
	Data/Tooth.cpp
	GroupFiles/CameraCoordinate.cpp
	GroupFiles/GroupModelObservation.cpp
	GroupFiles/GroupBase.cpp
	GroupFiles/GroupFdiMark.cpp
	RendererContainer.cpp
	GroupFiles/GroupContrast.cpp
                GroupFiles/GroupOcclusion.cpp
                GroupFiles/AnimationCue.cpp
	VTKDerviedClass/CVtkInteractorStyle.cpp
	GroupManager.cpp
                GroupFiles/GroupMoveCalculation.cpp
GroupFiles/GroupGridLine.cpp
GroupFiles/GroupPathPlanningPSO.cpp
GroupFiles/MyGLWidget.cpp
)
set( Project_Headers
	MainWindow.h
 	Data/OrthoData.h 
  	Data/Gum.h
 	Data/Arch.h
	Data/Tooth.h
	Common.h
	GroupFiles/CameraCoordinate.h
	GroupFiles/GroupModelObservation.h
	GroupFiles/GroupBase.h
	GroupFiles/GroupFdiMark.h
	RendererContainer.h
	GroupFiles/GroupContrast.h
                GroupFiles/GroupOcclusion.h
                GroupFiles/AnimationCue.h
	VTKDerviedClass/CVtkInteractorStyle.h
	GroupManager.h
                GroupFiles/GroupMoveCalculation.h
GroupFiles/GroupGridLine.h
GroupFiles/GroupPathPlanningPSO.h
GroupFiles/MyGLWidget.h
)

Set(Project_UIs 
	MainWindow.ui

)
set(Project_Resources
	MainWindow.qrc
)

add_executable(OrthoDisplaySystem ${Project_Sources} ${Project_Headers} ${Project_UIs} ${Project_UIs})
target_link_libraries(OrthoDisplaySystem ${VTK_LIBRARIES} Qt5::Widgets)
target_include_directories(OrthoDisplaySystem PUBLIC ${EXTRA_INCLUDES})
