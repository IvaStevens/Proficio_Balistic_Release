#include <stdio.h>
#include "json.hpp"
#include "Dragonfly.h"
#include "Dragonfly_config.h"
#include "params.h"
#include <vector>
#include <math.h>
#include <string>
#include <fstream>
#include <streambuf>

using namespace std;
using json = nlohmann::json;

class TargetNode
{

  public:
    double  distance; //preferred is probably public getters and setters...
    int direction;
    int     width;
    double  force;

    TargetNode(double dist, int dir, int wid, double frc)
    {
      distance = dist;
      direction = dir;
      width = wid;
      force = frc;
    };
  
    bool operator==(const TargetNode& other) const
    {
      return
        distance == other.distance &&
        direction == other.direction &&
        width == other.width &&
        force == other.force;
    }
};

vector<TargetNode> initTargetList( 
  vector<int> dirs,
  vector<double> forces,
  vector<int> widths,
  vector<double> distances)
{
  // assert widths/distances are the same length
  if (distances.size() != widths.size())
  {
    throw "Yaml error, widths and distances must have same length";
  }
  vector<TargetNode> targetList;

  for (uint k = 0; k < forces.size(); k++)
  {
    for (uint j = 0; j < dirs.size(); j++)
    {
      for (uint i = 0; i < distances.size(); i++)
      {
        double newDist = distances.at(i);
        int newDir = dirs.at(j);
        int newWid = widths.at(i);
        double newForce = forces.at(k);
        TargetNode node = TargetNode(newDist, newDir, newWid, newForce);
        targetList.push_back(node);        
      }
    }
  }
  return targetList;
};


bool modifyTask(
  MDF_MODIFY_TASK edits,
  vector<TargetNode>::iterator &targetIter,
  TargetNode &nextTarget, 
  bool &rewardable)
{
  bool targetChanged = false;
  switch (edits.type)
  {
    case CONTROL: // Manually control the robot position. TODO
      break;
    case FREEZE: // Freeze the robot in place. TODO
      break;
    case HOME: // Move the robot back home. TODO
      break;
    case REPEAT: // Repeat current trial
    {
      // Automatically used nextTrial which already
      // is the same as the current
      targetChanged = true;
      break;
    }
    case REWARD: // Give a reward now
      break;
    case REWARD_SET: // Set reward level
      break;
    case REWIND: // Do last trial again
    {
      nextTarget = *(--targetIter);
      targetChanged = true;
      break;
    }
    case SKIP: // Skip next trial
    {
      ++targetIter;
      nextTarget = *(++targetIter);
      targetChanged = true;
      break;
    }
    case TARGET: // Set the target
    {
      int dir = edits.direction;
      double dist = edits.distance;
      double force = edits.force;
      int wid = edits.target_width;
      TargetNode node TargetNode(dist, dir, wid, force);
      nextTarget = node;
      targetChanged = true;
      break;
    }
    default: // Bad messgae sent. Error (?)
      break;
  }
  return targetChanged;
};


/**
 * main
 *
 * Run Experiment on the Burt robot
 */
int main( int argc, char *argv[])
{
  if (argc < 1) 
  {
      cout << "Missing configuration file" << endl;
      return -1;
  }
  char *configPath = argv[1];
   
  //Read in configuration file (json)
  json configJson; 
  ifstream textStream(configPath);
  textStream >> configJson;
  
  vector<int> trials;
  vector<int> directions;   // = {2, 1, 0};
  vector<double> forces;    // = {0.01, 0.02, 0.03, 0.04};
  vector<int> widths;       // = {1, 2, 3};
  vector<double> distances; // = {0.10, 0.15, 0.20};
  double tError;            // = 0.03;
  //double version;
  
  // Set task configuration
  if (configJson["Version"] == 1.0)
  {
      //version = (double) configJson["Version"];
      tError = (double) configJson["tError"];
      
      forces = configJson["Combos"]["force"].get<vector<double>>();
      distances = configJson["Combos"]["distance"].get<vector<double>>();
      
      directions = configJson["Combos"]["direction"].get<vector<int>>();
      trials = configJson["Combos"]["#"].get<vector<int>>();
      //widths = configJson["combos"]["widths"].get<vector<int>>();
  }

	try 
	{
		Dragonfly_Module mod( 0, 0);
		mod.ConnectToMMM();
		mod.Subscribe( MT_FORCE_FEEDBACK);
		mod.Subscribe( MT_POSITION_FEEDBACK);
		mod.Subscribe( MT_TRIAL_STATUS_FEEDBACK);
		mod.Subscribe( MT_RT_POSITION_FEEDBACK);
		mod.Subscribe( MT_TASK_STATE_CONFIG);
		mod.Subscribe( MT_BURT_STATUS);

    // ===================================================================================
    int currentState = START;
    bool userDefState = false;
    bool userDefTarget = false;
    int nStates = 6;
    bool rewardable = false;
    bool shouldReset = false;
    double tError = 0.03;
    int nextState = RESET;

    //TODO Read in yaml file
    vector<int> directions = {0, 0, 0, 0}; //2, 4, 6};
    vector<double> forces = {0.01, 0.02, 0.03, 0.04};
    vector<int> widths = {3, 5, 10};
    vector<double> distances = {0.4, 0.6, 0.8};

    // Initialize targetList
    vector<TargetNode> targetList = initTargetList(directions, forces, widths, distances);
    vector<TargetNode>::iterator targetIter = targetList.begin();
    TargetNode currentTarget = *targetIter;
    TargetNode nextTarget = *targetIter;

    // Send first message
    MDF_TASK_STATE_CONFIG trial_input_data;
    
    // Set next Target
    trial_input_data.state = currentState;
    trial_input_data.direction = currentTarget.direction;
    trial_input_data.force = currentTarget.force;
    trial_input_data.distance = currentTarget.distance;
    trial_input_data.target_width = currentTarget.width;
    //trial_input_data.targetError = tError;
    
    // Send start message
    CMessage task_state_config_M( MT_TASK_STATE_CONFIG);
    task_state_config_M.SetData( &trial_input_data, sizeof(trial_input_data));
    mod.SendMessageDF( &task_state_config_M);
    std::cout << "Sending first message..." << std::endl;
    
    
    // Run the experiment
    while(1)
    {
      CMessage M;
      mod.ReadMessage( &M);

      // Check for new messages
      switch( M.msg_type)
      {
        // This should probably be a different message
        case MT_TASK_STATE_CONFIG:
          break;

        /* interrupt flag was sent
        case MT_FLAG:
          break;
        */

        // BURT sent an msg about state termination.
        case MT_BURT_STATUS:
        {
          //std::cout << "burt message sent \n";
          MDF_BURT_STATUS burt_status_data;
          M.GetData( &burt_status_data);
          // if there is a success for given state
          if (burt_status_data.task_complete)
          {// move to the next state
            if (burt_status_data.task_success)
            {
              shouldReset = true;
              if (!userDefState)
              { // progress next state as usual if not set else where
                nextState = RESET;//(currentState % (nStates)) + 1;
              }
            // Subject error occurred, start next trial, no reward
            } 
            else
            {
              rewardable = false;
              shouldReset = true;
            }
            // pull up next TARGET parameters
            if ( nextTarget == currentTarget)
            {
              currentTarget = *(++targetIter);
            } else {
              currentTarget = nextTarget;
            }

            // determine next state      
            if ( true )//nextState ==  RESET || shouldReset)
            { // Determine next target (send out with state)

              // Send out RESET messages and next target parameters

              // Set next Target
              trial_input_data.direction = currentTarget.direction;
              trial_input_data.force = currentTarget.force;
              trial_input_data.distance = currentTarget.distance;
              trial_input_data.target_width = currentTarget.width;
              
              //trial_input_data.targetError = tError;
              //trial_input_data.state = nextState;
              
              // Print target definition
              cout << "Sending out next trial data..." << endl << "required force: " 
                << trial_input_data.force  << endl << "Target distance: " 
                << trial_input_data.distance << endl << "Direction of movement: " 
                << trial_input_data.direction << endl;

              // If this is a rewardable transition, do so.
              if (rewardable) {
                cout << "REWARD!!!" << endl;
              }

              // Reset variables
              shouldReset = false;
              userDefState = false;
              userDefTarget = false;
              nextTarget = currentTarget;
              
              // TODO: THIS SHOULD BE REMOVED LATER
              trial_input_data.state =  START;
              //Send message
              task_state_config_M.SetData( &trial_input_data, sizeof(trial_input_data));
              mod.SendMessageDF( &task_state_config_M); 
              
            }
            else
            {
              // Send message
              trial_input_data.state = REST;
              task_state_config_M.SetData( &trial_input_data, sizeof(trial_input_data));
              mod.SendMessageDF( &task_state_config_M);              
            }
          }
          break;
        }
        
        // Default because we are running into issues
        default:
        {
          std::cout << "weird message sent: " << M.msg_type << std::endl;
        }
      }
    }
    // ===================================================================================
	}
	catch( UPipeClosedException &e)
	{
		MyCString s;
		e.AppendTraceToString( s);
		std::cout << "UPipeClosedException: " << s.GetContent() << std::endl;
	}
	catch( UPipeException &e)
	{
		MyCString s;
		e.AppendTraceToString( s);
		std::cout << "UPipeException: " << s.GetContent() << std::endl;
	}
	catch( MyCException &e)
	{
		MyCString s;
		e.AppendTraceToString( s);
		std::cout << "MyCException: " << s.GetContent() << std::endl;
	}
	catch( exception &e)
	{
		std::cout << "Unknown Exception!" << e.what() << std::endl;
	}
	catch(...)
	{
		MyCString s;
		std::cout << "Unknown Exception!" << std::endl;
	}

	std::cout << "Exiting cleanly." << std::endl;
}
