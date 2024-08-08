all: recording_generated.h pose_generated.h trajectory_generated.h

recording_generated.h: recording.fbs
	flatc --cpp --cpp-std=c++17 --gen-object-api recording.fbs

trajectory_generated.h: trajectory.fbs
	flatc --cpp --cpp-std=c++17 --gen-object-api trajectory.fbs

pose_generated.h: pose.fbs
	flatc --cpp --cpp-std=c++17 --gen-object-api pose.fbs
	
