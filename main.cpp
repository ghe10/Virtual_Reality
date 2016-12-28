
//Allow PhaseSpace locations to be accessed directly in Vizard.
//Modified version of the sensor sample provided by Vizard that uses the owl interface to PhaseSpace.
//
//Supports multiple sensors
//
//Must be the master. TODO: why can't this run as a slave?
//
//Changed 6/10
//  Now only waits for 1000 attempts before giving up on rigid bodies
//  Default offset is 0,0,0
//  Orientations are correct for Vizard
//
//Changed 1/13
//  Updated for viz 4 and recompiled with new interface
//  Also correctly updates the orientation for point markers
//      TODO: make version 2 with a threaded, real time reader and new Vizard interface
//
//Changed 3/13 
//  threaded version so the timing is much improved
//   This is intermediate toward using the new Vizard interface
//
//Changed 5/13 
//   Improved the memory management to avoid sampling issues


#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <set>
#include <string>
#include <fstream>

#include <boost/thread.hpp>
#include <windows.h>

#include "sensor.h"
#include "owl.h"
#include "CPrecisionClock.h"

using namespace std;

const int MAX_MARKER_COUNT = 128;
const int MAX_RIGID_COUNT = 128;

//constants
string OWL_SERVER;// "192.168.1.220"
float LOCAL_OWL_FREQUENCY = OWL_MAX_FREQUENCY;

size_t LOCAL_FLAGS = 0;//OWL_SLAVE;    //0 means this tries to grab the server

const int RIGID_AVERAGE_SAMPLES = 10;
const int MAX_RIGID_AVERAGE_TRIALS = 1000;     //stop trying to get rigid markers after this long

//set up the coordinate system
//OFFSET units are from PhaseSpace (millimeters)
//SCALE sets the units seen by Vizard (1 give mm, .1 gives cm, ...)
float SCALE_X = 1.0f;
float SCALE_Y = 1.0f;
float SCALE_Z = 1.0f;
/* 1) to get the offset: in vizard: type in the command line: print sensor2.getPosition()
2) change the signs of the number*/
float OFFSET_X = 0.0f;
float OFFSET_Y = 0.0f;
float OFFSET_Z = 0.0f;

//used to request that the origin be reset
bool REQUEST_RESET_ORIGIN = false;
int ORIGIN_ID = 0;

//the number of the largest marker number added
int MARKER_COUNT = 0;
set<int> USED_MARKER;   //the used markers

//flag if the server is going or not
bool SERVER_STARTED = false;

//flag streaming (more reliable than owl)
bool STREAMING = false;

//this is used to read the markers and rigids, only called by the first sensor, so no thread issues
OWLRigid* GLOBAL_RIGIDS;
OWLMarker* GLOBAL_MARKERS;

//keep track of the total number of rigids
int RIGID_COUNT = 0;

//align times with vizard
cPrecisionClock simClock;  //a clock
double timeOffset = 0.;   //add this to the clock's getCPUTimeSeconds to get the current vizard tick

//threading stuff
boost::shared_ptr<boost::thread> READ_THREAD;
boost::mutex block_mutex;
void threadMe();
bool REQUEST_SHUTDOWN = false;

//struct to hold the individual sensor objects
struct dataRecordMember {
	float time, x, y, z;
	int ttl;
	float qw, qx, qy, qz;
};
struct SPhaseSpaceSensor{
	VRUTSensorObj* instance;      //the Vizard object
	int trackerID;                //unique for each sensor
	vector<int> markers;          //markers used
	bool isRigid;                 //true if this is a rigid, false for just a marker
	int rigidNumber;              //the nth rigid in the return from GetRigids
	vector<float*> rigidBodyDefinition;  //rigid body setup (if used)
	bool isStarted;               //test if this is going
	bool needsInitialization;     //test if we need to initialize
	int samples;                  //the number of samples taken (used for averaging)
	bool requestRecording;        //record this marker (or rigid)
	bool dumpRecording;           //dump this marker's data (or rigid)
	float lastGood[7];           //store the last good measurement
	vector<dataRecordMember> record;  //data record
};

//struct for the commdata (new to the x2)
struct packet{
	unsigned char sysid[8];
	unsigned char count[1];
	bool ttl_0:1;
	bool ttl_1:1;
	bool ttl_2:1;
	bool ttl_3:1;
};

//Stream the data in a SPhaseSpaceSensor for debugging/checking.
ostream& operator<<(ostream& out, const SPhaseSpaceSensor& s) {
	out << "   Server ip: " << OWL_SERVER << "\n";
	out << "   Sample frequency " << LOCAL_OWL_FREQUENCY << "\n";
	out << "   Markers:";
	for(int i = 0; i < s.markers.size(); ++i) {
		out << " " << s.markers[i];
	}
	out << "\n";
	if(s.isStarted) {
		out << "   Started = true.\n";
		out << "   Samples on last trial = " << s.samples << "\n";
	} else {
		out << "   Started = false.\n";
	}
	if(s.needsInitialization) {
		out << "   Needs initialization = true ... do not trust output.\n";
	} else {
		//don't try to access an uninitialized rigid body
		if(s.isRigid) {
			out << "   This is a rigid body\n" << flush;
			out << "   Tracker ID is " << s.trackerID << "\n" << flush;
			for(int i = 0; i < s.markers.size(); ++i) {
				out << "      Marker " << s.markers[i] << " -> (" << flush;
				out << s.rigidBodyDefinition[i][0] << ", " << flush;
				out << s.rigidBodyDefinition[i][1] << ", " << flush;
				out << s.rigidBodyDefinition[i][2] << ")\n" << flush;
			}
		}
	}
	return out;
}

//global list of all sensors,
//each sensor in the list is updated at each call to UpdateSensor
const int PREALLOCATION_SIZE = 100000; //pre allocate this many time samples
vector<SPhaseSpaceSensor*> ALL_SENSORS;

//vector<double> counterHack;

void DoDumpFile(const int& id, char* name) {
	boost::mutex::scoped_lock l(block_mutex);

	ofstream dumpFile;
	dumpFile.open(name);
	if(!dumpFile.good()) {
		cout << "Warning: could not open " << name << " for writing ... will use test_ps_dump.txt\n" << flush;
		dumpFile.close();
		dumpFile.open("test_ps_dump.txt");
	}
	dumpFile.precision(12);
	for(int i = 0; i < ALL_SENSORS[id]->record.size(); ++i) {
		dumpFile << ALL_SENSORS[id]->record[i].time << " "
			<< ALL_SENSORS[id]->record[i].ttl << " "
			<< ALL_SENSORS[id]->record[i].x << " "
			<< ALL_SENSORS[id]->record[i].y << " "
			<< ALL_SENSORS[id]->record[i].z;
		if(ALL_SENSORS[id]->isRigid) {
			dumpFile << " " << ALL_SENSORS[id]->record[i].qw << " "
				<< ALL_SENSORS[id]->record[i].qx << " "
				<< ALL_SENSORS[id]->record[i].qy << " "
				<< ALL_SENSORS[id]->record[i].qz;
		}
		dumpFile << "\n";
	}
	dumpFile.close();

	//ofstream ch("counter_hack.txt");
	//ch.precision(10);
	//for(int i = 0; i < counterHack.size(); ++i) {
	//	ch << counterHack[i] << "\n";
	//}
	//ch.close();
}

// DO NOT MODIFY THESE DECLARATIONS----------------
extern "C" __declspec(dllexport) void QuerySensor(void *);
extern "C" __declspec(dllexport) void InitializeSensor(void *);
extern "C" __declspec(dllexport) void UpdateSensor(void *);
extern "C" __declspec(dllexport) void CommandSensor(void *);
extern "C" __declspec(dllexport) void ResetSensor(void *);
extern "C" __declspec(dllexport) void CloseSensor(void *);
//  end DO NOT MODIFY---------------------------------

//Safely enable/disable streaming.
//Only acts if this would change the streaming state.
//Turning this off then on should clear the owl data stream.
//Does nothing if the server is not started.
void SetOwlStreaming(const bool& b) {
	if(!SERVER_STARTED) {
		return;
	}
	if(b && !STREAMING) {
		owlSetInteger(OWL_STREAMING, OWL_ENABLE);
		owlSetInteger(OWL_COMMDATA, OWL_ENABLE);
		STREAMING = true;
	} else if (!b && STREAMING){
		STREAMING = false;
		owlSetInteger(OWL_COMMDATA, OWL_DISABLE);
		owlSetInteger(OWL_STREAMING, OWL_DISABLE);
	} else {
		return;
	}
}

//Update the number of rigids created.
void UpdateRigidCount() {
	RIGID_COUNT = 0;
	for(int i = 0; i < ALL_SENSORS.size(); ++i) {
		if(ALL_SENSORS[i]->isRigid) {
			ALL_SENSORS[i]->rigidNumber = RIGID_COUNT;
			++RIGID_COUNT;
		}
	}
}

//Define relative locations of the rigid body.
//Can test for error if the returned vector has size 0
vector<float*> CreateRigidLocations(const int& id) {
	vector<float*> rigidTemp;

	//check that we're streaming (should be), already checked size
	if(!SERVER_STARTED) {
		cout << "Error in rigid body creation: PhaseSpace server not started ... fail\n";
		return rigidTemp;
	}

	//create the object
	int i;
	int markerCount = ALL_SENSORS[id]->markers.size();

	//stream some data from OWL
	OWLMarker * markers = new OWLMarker[MARKER_COUNT];
	int n = -1; 

	//clear/initialize the stream
	int tracker = ALL_SENSORS[id]->trackerID;
	owlTrackeri(tracker, OWL_CREATE, OWL_POINT_TRACKER);
	for(int i = 0; i < ALL_SENSORS[id]->markers.size(); ++i) {
		owlMarkeri(MARKER(tracker,i), OWL_SET_LED, ALL_SENSORS[id]->markers.at(i));
	}
	owlTracker(tracker, OWL_ENABLE);
	SetOwlStreaming(true);
	while(owlGetMarkers(markers, MARKER_COUNT) > 0) {}

	//dynamically grab the rigid body positions
	for(i = 0; i < markerCount; ++i) {
		int markerID = ALL_SENSORS[id]->markers[i];
		float* newPoint = new float[3];
		newPoint[0] = 0.0f;
		newPoint[1] = 0.0f;
		newPoint[2] = 0.0f;
		int count = 0;
		int trial = 0;
		do {
			n = owlGetMarkers(markers, MARKER_COUNT);
			if(n > 0) {
				if(markers[markerID].cond > 0.1f) {
					newPoint[0] += markers[markerID].x;
					newPoint[1] += markers[markerID].y;
					newPoint[2] += markers[markerID].z;
					++count;
				} else {
					///////This dumps too many times normally////
					//  cout << "   Error in rigid body creation: unable to see marker " << " " << markerID 
					//     << " strength = " << markers[markerID].cond 
					//     << "\n   If this persists close and check the markers.\n" << flush;
				}
				++trial;
			}
		} while(count < RIGID_AVERAGE_SAMPLES && trial < MAX_RIGID_AVERAGE_TRIALS);
		if(trial < MAX_RIGID_AVERAGE_TRIALS) {
			newPoint[0] /= float(RIGID_AVERAGE_SAMPLES);
			newPoint[1] /= float(RIGID_AVERAGE_SAMPLES);
			newPoint[2] /= float(RIGID_AVERAGE_SAMPLES);
			rigidTemp.push_back(newPoint);
		} else {
			cout << "Error: unable to read markers ... fail\n" << flush;
			return rigidTemp;
		}
	}

	//define the first marker as 0,0,0
	for(i = 1; i < rigidTemp.size(); ++i) {
		rigidTemp[i][0] -= rigidTemp[0][0];
		rigidTemp[i][1] -= rigidTemp[0][1];
		rigidTemp[i][2] -= rigidTemp[0][2];
	}
	rigidTemp[0][0] = 0.0f;
	rigidTemp[0][1] = 0.0f;
	rigidTemp[0][2] = 0.0f;

	//clean up the stream
	SetOwlStreaming(false);
	owlTracker(tracker, OWL_DISABLE);
	owlTracker(tracker, OWL_DESTROY);
	delete[] markers;

	return rigidTemp;
}

//set up tracker
//The id is the location of the sensor in ALL_SENSORS
//Do nothing if the server is not started
void SetUpSensor(const int& id) {
	if(!SERVER_STARTED) {
		return;
	}

	//Get the sensor pointed to by the local id
	VRUTSensorObj* sensor = ALL_SENSORS[id]->instance;

	if(ALL_SENSORS[id]->markers.size() == 0) {
		cout << "Warning in phasespace: no markers set for sensor " << id 
			<< " (in order of creation) ... ignoring\n" << flush;
		ALL_SENSORS[id]->isStarted = false;
		ALL_SENSORS[id]->needsInitialization = false;
		return;
	}

	if( (LOCAL_FLAGS & OWL_SLAVE) == OWL_SLAVE ) {
		//server is already going, so no need to initialize anything
		SetOwlStreaming(true);
		ALL_SENSORS[id]->isStarted = true;
		ALL_SENSORS[id]->needsInitialization = false;
		return;
	}

	//turn off any streaming
	SetOwlStreaming(false);

	//Check if the sensor is already running (and stop it)
	int tracker = ALL_SENSORS[id]->trackerID;
	if(ALL_SENSORS[id]->isStarted) {
		ALL_SENSORS[id]->isStarted = false; //tell other threads not to access this
		owlTracker(tracker, OWL_DISABLE);
		owlTracker(tracker, OWL_DESTROY);
	}

	if(ALL_SENSORS[id]->isRigid) {
		//handle rigids here

		//create the rigid body definition (must be called before attempting to create the 
		//   rigid body because it used the same markers).
		//Create rigid locations will clean up after itself
		//cout << "gh0\n" << flush;
		ALL_SENSORS[id]->rigidBodyDefinition = CreateRigidLocations(id);
		//cout << "gh1\n" << flush;
		if(ALL_SENSORS[id]->rigidBodyDefinition.size() != ALL_SENSORS[id]->markers.size()) {
			cout << "Error in rigid body creation: unable to capture markers ... skipping\n" << flush;
			ALL_SENSORS[id]->isStarted = false;
			ALL_SENSORS[id]->needsInitialization = false;
			sensor->status = false;
			return;
		}

		//create the tracker for this sensor
		owlTrackeri(tracker, OWL_CREATE, OWL_RIGID_TRACKER);
		if(!owlGetStatus()) { // 0-- errors, 1- correct
			cout << "Error in tracker setup: unable to create tracker " << tracker << " .. skipping.\n" << flush;
			ALL_SENSORS[id]->isStarted = false;
			ALL_SENSORS[id]->needsInitialization = false;
			sensor->status = false;
			return;
		}

		//Initialize the tracker
		for(int i = 0; i < ALL_SENSORS[id]->markers.size(); i++) {
			int currentMarker = ALL_SENSORS[id]->markers.at(i);
			owlMarkeri(MARKER(tracker, i), OWL_SET_LED, currentMarker);
			if(!owlGetStatus()) {
				ALL_SENSORS[id]->isStarted = false;
				ALL_SENSORS[id]->needsInitialization = false;
				cout << "Error in default tracker setup: unable to add marker " 
					<< currentMarker << " to tracker " << tracker << "\n" << flush;
				sensor->status = false;
				return;
			}
			owlMarkerfv(MARKER(tracker, i), OWL_SET_POSITION, ALL_SENSORS[id]->rigidBodyDefinition.at(i));
			if(!owlGetStatus()) {
				ALL_SENSORS[id]->isStarted = false;
				ALL_SENSORS[id]->needsInitialization = false;
				cout << "Error in tracker setup: unable to add rigid body ... ignoring\n" << flush;
				sensor->status = false;
				return;
			}
		}
		owlTracker(tracker, OWL_ENABLE);

		if(!owlGetStatus()) {
			cout << "Error in default tracker setup: unable to start tracker.\n" << flush;
			sensor->status = false;
			return;
		}

	} else {
		//handle point trackers here
		owlTrackeri(tracker, OWL_CREATE, OWL_POINT_TRACKER);
		for(int i = 0; i < ALL_SENSORS[id]->markers.size(); ++i) {
			owlMarkeri(MARKER(tracker,i), OWL_SET_LED, ALL_SENSORS[id]->markers.at(i));
		}
		owlTracker(tracker, OWL_ENABLE);
		if(!owlGetStatus()) {
			cout << "Error in tracker setup: unable to start tracker " << id 
				<< " ... will be unlinkable.\n" << flush;
			sensor->status = false;
			return;
		}
	}

	//turn on streaming
	SetOwlStreaming(true);

	//mark as started
	ALL_SENSORS[id]->isStarted = true;
	ALL_SENSORS[id]->needsInitialization = false;

}

void StartServer() {
	//Check if the stream has already been initialized
	if(!SERVER_STARTED) {
		//have to start the server, must be first pass
		cout << "Starting OWL server ... " << flush;
		owlInit(OWL_SERVER.c_str(), LOCAL_FLAGS);
		if(owlGetStatus() == 0) {
			cout << "Warning: OWL initialization error ... unknown result (may have no effect).\n" << flush;
			cout << "         Is it possible another program is running PhaseSpace (master)?\n" << flush;
			cout << "         Be aware that this occasionally just happens (retry)\n" << flush;
			return;
		}
		cout << "done\n" << flush;
		//create the marker holder space
		GLOBAL_MARKERS = new OWLMarker[MARKER_COUNT];
		GLOBAL_RIGIDS = new OWLRigid[RIGID_COUNT];
		if( (LOCAL_FLAGS & OWL_SLAVE) == OWL_SLAVE ) {
			owlSetFloat(OWL_FREQUENCY, LOCAL_OWL_FREQUENCY);
		}
		SERVER_STARTED = true;

		//check initialization
		for(int i = 0; i < ALL_SENSORS.size(); ++i) {
			if(ALL_SENSORS[i]->needsInitialization) {
				cout << "Starting phasespace initialization for sensor " << i << " ... " << flush;
				SetUpSensor(i);
				cout << "done\n" << flush;
			}
		}

		//start up the read thread
		SetOwlStreaming(true);
		READ_THREAD.reset(new boost::thread(threadMe));
		SetThreadPriority(READ_THREAD->native_handle(), THREAD_PRIORITY_HIGHEST);
	}
}

void QuerySensor(void *sensor)
{
	// This function gets called during the Vizard initialization process.
	// Its only purpose is to set the sensor type (see choices in sensor.h),
	// so that it can be automatically made available to the user according
	// to its type.
	// No initialization or communication should be attempted at this point
	// because the device may never be requested by user (and it might not
	// even be connected!).

	//A short description of your plugin
	strcpy(((VRUTSensorObj *)sensor)->version, "PhaseSpace Interface v2.0b");
	//Your plugin type e.g.( SENSOR_HEADPOS | SENSOR_HEADORI | SENSOR_QUATERNION )
	((VRUTSensorObj *)sensor)->type = SENSOR_HEADPOS | SENSOR_HEADORI | SENSOR_QUATERNION;
}


void InitializeSensor(void *sensor)
{
	//Called each time an instance is created.

	//This function will attempt to connect to the device and initialize any variables.

	//Check if the server is already started
	if(SERVER_STARTED) {
		cout << "Error: cannot add PhaseSpace markers after starting the server ... skipping\n" << flush;
		((VRUTSensorObj *)sensor)->status = false;
		return;
	}

	//You can set the size of the data field to any value.
	//After this function is called, Vizard will allocate the
	//data field of the plugin to the size given here. Then you
	//can put any values you want in the data field and the user
	//can get those values by calling the "get" command on the
	//sensor object in the script.
	//It is suggested that the size of the data field be 7 or higher
	((VRUTSensorObj*)sensor)->dataSize = 7;

	//If you have multiple instances you can store your own unique
	//identifier in the user data fields.
	((VRUTSensorObj*)sensor)->user[0] = ALL_SENSORS.size();

	//Create the sensor
	SPhaseSpaceSensor* newSensor = new SPhaseSpaceSensor();
	newSensor->instance = ((VRUTSensorObj*)sensor);
	newSensor->isStarted = false;
	newSensor->needsInitialization = false;
	newSensor->isRigid = false;
	newSensor->trackerID = ALL_SENSORS.size();
	newSensor->samples = 0;
	ALL_SENSORS.push_back(newSensor);

	cout << "Added sensor id " << newSensor->trackerID << "\n" << flush;

	((VRUTSensorObj *)sensor)->status = true;
}


void UpdateSensor(void *sensor)
{
	// Update the sensor data fields (see sensor.h)
	// Fields 0-2: reserved for x, y, z position data
	// Fields 3-5: reserved for yaw, pitch, roll if SENSOR_EULER1 is specified in type
	// or
	// Fields 3-6: reserved for quaternion data if SENSOR_QUATERNION is specified in type
	// 
	// eg:
	// ((VRUTSensorObj *)sensor)->data[0] = newX;
	// If this plugin is not a SENSOR_HEADPOS or SENSOR_HEADORI then you can fill
	// the data fields with whatever you want and retrieve it with the "<sensor>.get()" command
	// in your script.

	//wait for the server to be started
	if(!SERVER_STARTED) {
		return;
	}

	//check streaming status
	if(!STREAMING) {
		return;
	}

	if(READ_THREAD) {
		//lock then update the position
		boost::mutex::scoped_lock l(block_mutex);

		for(int i = 0; i < ALL_SENSORS.size(); ++i) {
			if(ALL_SENSORS[i]->isStarted) {
				if(ALL_SENSORS[i]->isRigid) {
					//handle rigids here
					ALL_SENSORS[i]->instance->data[0] = -1.0f * SCALE_X * ( ALL_SENSORS[i]->lastGood[0] + OFFSET_X );
					ALL_SENSORS[i]->instance->data[1] = SCALE_Y * ( ALL_SENSORS[i]->lastGood[1] + OFFSET_Y );
					ALL_SENSORS[i]->instance->data[2] = SCALE_Z * ( ALL_SENSORS[i]->lastGood[2] + OFFSET_Z );
					ALL_SENSORS[i]->instance->data[3] = -1.0f * ALL_SENSORS[i]->lastGood[4];
					ALL_SENSORS[i]->instance->data[4] = ALL_SENSORS[i]->lastGood[5];
					ALL_SENSORS[i]->instance->data[5] = ALL_SENSORS[i]->lastGood[6];
					ALL_SENSORS[i]->instance->data[6] = -1.0f * ALL_SENSORS[i]->lastGood[3];
				} else {
					ALL_SENSORS[i]->instance->data[0] = -1.0f * SCALE_X * ( ALL_SENSORS[i]->lastGood[0] + OFFSET_X );
					ALL_SENSORS[i]->instance->data[1] = SCALE_Y * ( ALL_SENSORS[i]->lastGood[1] + OFFSET_Y );
					ALL_SENSORS[i]->instance->data[2] = SCALE_Z * ( ALL_SENSORS[i]->lastGood[2] + OFFSET_Z );
					ALL_SENSORS[i]->instance->data[3] = 0.0f;
					ALL_SENSORS[i]->instance->data[4] = 0.0f;
					ALL_SENSORS[i]->instance->data[5] = 0.0f;
					ALL_SENSORS[i]->instance->data[6] = 1.0f;
				}
			}
		}
	}
}

//this will be put into a thread (just pulled out of the old UpdateSensor command)
void threadMe() {
	dataRecordMember insert;
	while(true) {

		//if(ALL_SENSORS[0]->requestRecording) {
		//	counterHack.push_back(simClock.getCPUTimeSeconds());
		//}

		//update if the sensor has data
		unsigned char buffer[1024];
		int n = owlGetMarkers(GLOBAL_MARKERS, MARKER_COUNT);
		int m = owlGetRigids(GLOBAL_RIGIDS, RIGID_COUNT);
		if(n>0 || m>0) {
			//lock then read the position (shouldn't take very long)
			boost::mutex::scoped_lock l(block_mutex);

			owlGetString(OWL_COMMDATA, (char*)buffer);

			for(int i = 0; i < ALL_SENSORS.size(); ++i) {
				if(ALL_SENSORS[i]->isStarted) {
					if(ALL_SENSORS[i]->isRigid) {
						//handle rigids here
						int rigidNumber = ALL_SENSORS[i]->rigidNumber;
						if(m > 0) {
							//handle requests (simple callback functionality)
							if(REQUEST_RESET_ORIGIN && ORIGIN_ID == i) {
								OFFSET_X = -1.0f*GLOBAL_RIGIDS[rigidNumber].pose[0];
								OFFSET_Y = -1.0f*GLOBAL_RIGIDS[rigidNumber].pose[1];
								OFFSET_Z = -1.0f*GLOBAL_RIGIDS[rigidNumber].pose[2];
								REQUEST_RESET_ORIGIN = false;
							}
							if(ALL_SENSORS[i]->requestRecording) {
								insert.time = simClock.getCPUTimeSeconds()+timeOffset;
								insert.ttl = ((packet*)buffer)->ttl_0;
								insert.x = -1.0 * SCALE_X * ( GLOBAL_RIGIDS[rigidNumber].pose[0] + OFFSET_X );
								insert.y = SCALE_Y * ( GLOBAL_RIGIDS[rigidNumber].pose[1] + OFFSET_Y );
								insert.z = SCALE_Z * ( GLOBAL_RIGIDS[rigidNumber].pose[2] + OFFSET_Z );
								insert.qw = -1.0 * GLOBAL_RIGIDS[rigidNumber].pose[4];
								insert.qx = GLOBAL_RIGIDS[rigidNumber].pose[5];
								insert.qy = GLOBAL_RIGIDS[rigidNumber].pose[6];
								insert.qz = -1.0 * GLOBAL_RIGIDS[rigidNumber].pose[3];
								ALL_SENSORS[i]->record.push_back(insert);
							}
							//store this as the last good estimate
							if(GLOBAL_RIGIDS[rigidNumber].cond > 0.1f) {
								ALL_SENSORS[i]->samples += 1;
								for(int k = 0; k < 7; ++k) {
									ALL_SENSORS[i]->lastGood[k] = GLOBAL_RIGIDS[rigidNumber].pose[k];
								}
							}
						}
					} else {
						//handle point markers here
						if(n > 0) {
							int id = ALL_SENSORS[i]->markers[0];
							if(REQUEST_RESET_ORIGIN && ORIGIN_ID == i) {
								OFFSET_X = -1.0f*GLOBAL_MARKERS[id].x;
								OFFSET_Y = -1.0f*GLOBAL_MARKERS[id].y;
								OFFSET_Z = -1.0f*GLOBAL_MARKERS[id].z;
								REQUEST_RESET_ORIGIN = false;
							}
							if(ALL_SENSORS[i]->requestRecording) {
								insert.time = simClock.getCPUTimeSeconds()+timeOffset;
								insert.ttl =  ((packet*)buffer)->ttl_0;
								insert.x = -1.0f * SCALE_X * ( GLOBAL_MARKERS[id].x + OFFSET_X );
								insert.y = SCALE_Y * ( GLOBAL_MARKERS[id].y + OFFSET_Y );
								insert.z = SCALE_Z * ( GLOBAL_MARKERS[id].z + OFFSET_Z );
								ALL_SENSORS[i]->record.push_back(insert);
							}
							if( GLOBAL_MARKERS[id].cond > 0.1 ) {
								ALL_SENSORS[i]->samples += 1;
								ALL_SENSORS[i]->lastGood[0] = GLOBAL_MARKERS[id].x;
								ALL_SENSORS[i]->lastGood[1] = GLOBAL_MARKERS[id].y;
								ALL_SENSORS[i]->lastGood[2] = GLOBAL_MARKERS[id].z;
							}
						}
					}
				}
			}
		}

		if(REQUEST_SHUTDOWN) {
			return;
		}

	}
}


void ResetSensor(void *sensor)
{
	// If the user were to send a reset command, do whatever makes sense to do.
}

void CommandSensor(void *sensor)
{
	// The user has sent a command to the sensor.
	// The command is saved in sensor->command.
	// 3 floating point numbers are saved under
	// sensor->data Fields 0-2.
	// A string is saved in sensor->custom. Don't forget
	// to cast it to a (char*)
	char msg[32];
	float x,y,z;

	int id = ((VRUTSensorObj *)sensor)->user[0];

	cout << "Calling command " << (int) ((VRUTSensorObj *)sensor)->command
		<< " for sensor " << id << "\n" << flush;

	strcpy(msg,(char *)((VRUTSensorObj *)sensor)->custom);
	x = ((VRUTSensorObj *)sensor)->data[0];
	y = ((VRUTSensorObj *)sensor)->data[1];
	z = ((VRUTSensorObj *)sensor)->data[2];

	int marker;
	switch( (int) ((VRUTSensorObj *)sensor)->command) {
case 1:
	cout << "Sensor data is:\n" << *(ALL_SENSORS[id]) << "\n" << flush;
	break;
case 2:
	SCALE_X = x; SCALE_Y = y; SCALE_Z = z;
	break;
case 3:
	OFFSET_X = x; OFFSET_Y = y; OFFSET_Z = z;
	break;
case 4:
	REQUEST_RESET_ORIGIN = true;
	ORIGIN_ID = id;
	break;
case 5:
	//add a marker (will not start streaming)
	if(ALL_SENSORS[id]->isStarted) {
		cout << "Error: cannot add markers after enabling the sensor ... skipping\n" << flush;
		return;
	}
	marker = int(x+0.5f);
	if(marker < 0 || marker >= MAX_MARKER_COUNT) {
		cout << "Error: marker " << marker << " out of range ... skipping\n" << flush;
	} else if (USED_MARKER.find(marker) != USED_MARKER.end()) {
		cout << "Error: marker already used and they cannot be shared ... skipping\n" <<flush;
	} else {
		ALL_SENSORS[id]->markers.push_back(marker);
		MARKER_COUNT = max(MARKER_COUNT, marker+1);
		USED_MARKER.insert(marker);
	}
	break;
case 6:
	//set as a rigid body
	//waits for streaming (command 8) to start
	//TODO: must be called after marker setup, change to be dynamic
	if(ALL_SENSORS[id]->isStarted) {
		cout << "Error: cannot restart sensor ... skipping\n" << flush;
		return;
	}
	if(ALL_SENSORS[id]->markers.size() >= 3) {
		ALL_SENSORS[id]->isRigid = true;
		ALL_SENSORS[id]->needsInitialization = true;
		UpdateRigidCount();
	} else {
		cout << "Warning: must add at least three markers to the object to create a rigid body ... skipping fairly gracefullly\n" << flush;
	}
	break;
case 7:
	//set as a point marker
	//waits for streaming (command 8) to start
	//TODO: must be called after marker setup, change to be dynamic
	if(ALL_SENSORS[id]->isStarted) {
		cout << "Error: cannot restart sensor ... skipping\n" << flush;
		return;
	}
	ALL_SENSORS[id]->isRigid = false;
	ALL_SENSORS[id]->needsInitialization = true;
	break;
case 8:
	//start the server (it handles checking)
	if(OWL_SERVER.length() < 1) {
		cout << "Error: server not set ... ignoring start.\nSet the server and try again.\n" << flush;
	} else {
		StartServer();
	}
	break;
case 9:
	//set the OWL server. cannot be called after starting the server
	//                     (threads interfere, not a priority)
	if(!SERVER_STARTED) {
		cout << "Setting OWL server ip = " << msg << "\n" << flush;
		OWL_SERVER.clear();
		OWL_SERVER.append(msg);
	}
	break;
case 10:
	//set the sample frequency. cannot be called after starting the server
	//                     (threads interfere, not a priority)
	if(!SERVER_STARTED) {
		if(x > 0.0f && x <= 960.){//OWL_MAX_FREQUENCY) {
			cout << "Setting frequency = " << x << "\n" << flush;
			LOCAL_OWL_FREQUENCY = x;
		} else {
			cout << "Error: bad frequency " << x << " ... ignoring\n" << flush;
		}
	}
	break;
case 11:
	//update the flag for startup, cannot be called after server starts
	if(!SERVER_STARTED) {
		LOCAL_FLAGS = int(x+0.5);
		if( (LOCAL_FLAGS & OWL_SLAVE) == OWL_SLAVE) {
			cout << "PhaseSpace: Setting as a slave" << "\n" << flush;
		}
		if( (LOCAL_FLAGS & OWL_POSTPROCESS) == OWL_POSTPROCESS) {
			cout << "PhaseSpace: Enabling post processing" << "\n" << flush;
		}
		if( (LOCAL_FLAGS & OWL_MODE1) == OWL_MODE1) {
			cout << "PhaseSpace: Setting as mode 1" << "\n" << flush;
		}
		if( (LOCAL_FLAGS & OWL_MODE2) == OWL_MODE2) {
			cout << "PhaseSpace: Setting as mode 2" << "\n" << flush;
		}
		if( (LOCAL_FLAGS & OWL_MODE3) == OWL_MODE3) {
			cout << "PhaseSpace: Setting as mode 3" << "\n" << flush;
		}
		if( (LOCAL_FLAGS & OWL_MODE4) == OWL_MODE4) {
			cout << "PhaseSpace: Setting as mode 4" << "\n" << flush;
		}
		//not sure what these flags do
		if( (LOCAL_FLAGS & OWL_FILE) == OWL_FILE) {
			cout << "PhaseSpace: Unknown flag OWL_FILE" << "\n" << flush;
		}
		/*if( (LOCAL_FLAGS & OWL_ASYNC) == OWL_ASYNC) {
		cout << "PhaseSpace: Unknown flag OWL_ASYNC" << "\n" << flush;
		}*/
		if( (LOCAL_FLAGS & OWL_LASER) == OWL_LASER) {
			cout << "PhaseSpace: Unknown flag OWL_LASER" << "\n" << flush;
		}
		if( (LOCAL_FLAGS & OWL_CALIB) == OWL_CALIB) {
			cout << "PhaseSpace: Unknown flag OWL_CALIB" << "\n" << flush;
		}
		/*if( (LOCAL_FLAGS & OWL_DIAGNOSTIC) == OWL_DIAGNOSTIC) {
		cout << "PhaseSpace: Unknown flag OWL_DIAGNOSTIC" << "\n" << flush;
		}*/
		if( (LOCAL_FLAGS & OWL_CALIBPLANAR) == OWL_CALIBPLANAR) {
			cout << "PhaseSpace: Unknown flag OWL_CALIBPLANAR" << "\n" << flush;
		}
	}
	break;

	//high speed recording stuff
case 100:
	//request recording of this phasespace marker and clear anything that was there
	{
		boost::mutex::scoped_lock l(block_mutex);
		ALL_SENSORS[id]->requestRecording = false; //just in case threading changes
		ALL_SENSORS[id]->record.clear();
		ALL_SENSORS[id]->record.reserve(PREALLOCATION_SIZE);
		ALL_SENSORS[id]->requestRecording = true;
	}
	break;
case 101:
	//stop recording of the phasespace markers, does not dump or clear the data
	{
		boost::mutex::scoped_lock l(block_mutex);
		ALL_SENSORS[id]->requestRecording = false;
	}
	break;
case 102:
	//dump the phasespace data, uses the file specified by the message (or test_ps_dump.txt)
	DoDumpFile(id, msg);
	break;
case 103:
	//clear the current recording
	{
		boost::mutex::scoped_lock l(block_mutex);
		ALL_SENSORS[id]->record.clear();
		ALL_SENSORS[id]->record.reserve(PREALLOCATION_SIZE);
	}
	break;
case 104:
	//synchronize the current time with the Vizard tick
	{
		boost::mutex::scoped_lock l(block_mutex);
		timeOffset = x-simClock.getCPUTimeSeconds();
		cout << "Setting PhaseSpace time offset to " << x << " - " << simClock.getCPUTimeSeconds() << " = " << timeOffset << "\n" << flush;
	}
	break;

default:
	break;
	}
}

void CloseSensor(void *sensor)
{
	// Go ahead, clean up, and close files and COM ports.
	//Called only once no matter how many instances were created

	if(SERVER_STARTED) {
		//clean up the marker/rigid holders
		delete [] GLOBAL_MARKERS;
		delete [] GLOBAL_RIGIDS;

		//stop the server
		owlDone();
	}

	//wait for the read thread to unblock
	if(READ_THREAD) {
		REQUEST_SHUTDOWN = true;
		READ_THREAD->join();
	}
	//nothing but local clean up if this is a slave connection
	if( (LOCAL_FLAGS & OWL_SLAVE) == OWL_SLAVE ) {
		//clean up the marker/rigid holders
		delete [] GLOBAL_MARKERS;
		delete [] GLOBAL_RIGIDS;
		return;
	}

	//stop the trackers
	for(int i = 0; i < ALL_SENSORS.size(); ++i) {
		if(ALL_SENSORS[i]->isRigid && ALL_SENSORS[i]->isStarted) {
			owlTracker(ALL_SENSORS[i]->trackerID, OWL_DISABLE);
			owlTracker(ALL_SENSORS[i]->trackerID, OWL_DESTROY);
			//delete the rigids coords if they've been created.
			if(! (ALL_SENSORS[i]->needsInitialization) ) {
				for(int j = 0; j < ALL_SENSORS[i]->markers.size(); ++j) {
					delete[] ALL_SENSORS[i]->rigidBodyDefinition[j];
				}
			}
		}
		delete ALL_SENSORS[i];
	}

}
