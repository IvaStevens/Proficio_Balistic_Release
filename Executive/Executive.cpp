#include <stdio.h>
#include "Dragonfly.h"
#include "Dragonfly_config.h"
#include "params.h"

class TargetNode
{
  int  distance;
  int direction;
  int     width;
  double  force;

  public:
    TargetNode(int dist, int dir, int wid, double frc)
    {
      distance = dist;
      direction = dir;
      width = wid;
      force = frc;
    };
};

vector<TargetNode> initTargetList( 
  vector<int> dirs,
  vector<int> forces,
  vector<int> widths,
  vector<double> distances)
{
  // assert widths/distances are the same length
  if (distances.size() != widths.size())
  {
    throw Exception("Yaml error, widths and distances must have same length");
  }
  vector<TargetNode> targetList;

  for (Iter forIt = forces.begin(); forIt!=forces.end(); ++forIt)
  {
    for (Iter dirIt = dirs.begin(); dirIt!=dirs.end(); ++dirIt)
    {
      for (int i = 0; i < distances.size(); i++)
      {
        TargetNode node = TargetNode(distances.begin()+i, *dirIt, widths.begin()+i, *forIt);
        targetList.push_back(node);        
      }
    }
  }
  return targetList;
};


int main( int argc, char *argv[])
{
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
    int nextState = (currentState % (nStates)) + 1;
    bool userDefState = false;
    bool userDefTarget = false;
    int nStates = 6;
    bool rewardable = false;
    bool shouldReset = false;
    double tError = 0.03;

    // Read in yaml file

    // Initialize targetList
    vector<TargetNode> targetList = initTargetList(directions, forces, widths, distances);
    Iter targetIter = targetList.begin();
    TargetNode currentTarget = *targetIter;
    TargetNode nextTarget = NULL;

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
          MDF_BURT_STATUS burt_status_data;
					M.GetData( &burt_status_data);
          // if there is a success for given state
          if (burt_status_data.task_complete)
          {// move to the next state
            if (burt_status_data.task_success)
            {
              shouldReset = false;
              if (!userDefState)
              { // progress next state as usual if not set else where
                nextState = (currentState % (nStates)) + 1;
              }
            // Subject error occurred, start next trial, no reward
            } else {
              shouldReset = true;
              rewardable = false;
              break;
            }
            // pull up next TARGET parameters
            if ( nextTarget == NULL)
            {
              currentTarget = targetIter++;
            } else {
              currentTarget = nextTarget;
            }

            // determine next state      
            if ( nextState ==  RESET || shouldReset)
            { // Determine next target (send out with state)

              // Send out RESET messages and next target parameters
              CMessage trial_input_M( MT_TRIAL_INPUT);
              trial_input_M.SetData( &trial_input_data, sizeof(trial_input_data));
              mod.SendMessageDF( &trial_input_M);

              CMessage task_state_config_M( MT_TASK_STATE_CONFIG);
              task_state_config_M.SetData( &task_state_data, sizeof(task_state_data));
              mod.SendMessageDF( &task_state_config_M);      

              // Set next Target
              trial_input_data.direction = currentTarget.direction;
              trial_input_data.forceThreshold = currentTarget.force;
              trial_input_data.targetDistance = currentTarget.distance;
              trial_input_data.width = currentTarget.width;
              trial_input_data.targetError = tError;

              // Print target definition
              cout << "XorYorZ " << trial_input_data.XorYorZ << endl << "force threshold " 
                << trial_input_data.forceThreshold  << endl << "target distance " 
                << trial_input_data.targetDistance << endl << "UpOrDown " 
                << trial_input_data.UpOrDown << endl;
              cout << "Sent out data" << endl;

              // If this is a rewardable transition, do so.
              if (!shouldReset) {
                cout << "REWARD!!!" << endl
              }

              // Reset variables
              shouldReset = false;
              userDefState = false;
              userDefTarget = false;
              nextTarget = NULL;
            }            
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
	catch(...)
	{
		MyCString s;
		std::cout << "Unknown Exception!" << std::endl;
	}

	std::cout << "Exiting cleanly." << std::endl;
}
