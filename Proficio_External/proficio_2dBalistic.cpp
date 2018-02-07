/*  
 * Code to run Balistic Force Trials on the Proficio Robot
 * 
 * Author(s): Henry Friedlander 2017, Ivana Stevens 2018
 * 
 */
 
#include "proficio_2dBalistic.h"  
#include "/home/robot/src/Proficio_Systems/magnitude.h"
#include "/home/robot/src/Proficio_Systems/normalize.h"
#include "/home/robot/Proficio_Balistic_Release/Dragonfly_Message_defs/Dragonfly_config.h"
#include "params.h"
#include "target_polygon.h"

#include <dragonfly/Dragonfly.h>

#include <string>
#include <signal.h>
#include <typeinfo>
#include <iostream>
#include <fstream>
#include <time.h>
//#include <chrono>
#include <stdio.h>
//#include <mutex>
//#include <thread>
#include <ctime>

#include <cmath>
#include <Eigen/Core>
#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/haptic_object.h>

#include <boost/bind.hpp>
#include <boost/tuple/tuple.hpp>

#include <barrett/config.h>                     
#include <barrett/exception.h>                  
#include <barrett/math.h>  
#include <barrett/products/product_manager.h>  
#include <barrett/systems.h>                  
#include <proficio/systems/utilities.h>
#include <barrett/units.h>

// Networking
#include <netinet/in.h>
#include <sys/types.h>

#include <proficio/systems/utilities.h>
#define BARRETT_SMF_VALIDATE_ARGS
//#define NO_CONTROL_PENDANT

#include <proficio/standard_proficio_main.h>    // NOLINT(build/include_order)

BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;
BARRETT_UNITS_TYPEDEFS(6);  // defines v_type to have length 6

const char* remoteHost = NULL;
// EDIT FOR VIBRATIONS
double kpLine = 3e3;
double kdLine = 4e1;
bool game_exit = false;
bool thresholdMet = false;
int XorYorZ;
double forceThreshold;
double targetDistance;
int UpOrDown;
cf_type cforce;
cp_type pos;


/** 
 * validate_args
 */
bool validate_args(int argc, char** argv) {
  switch (argc) {
    case 4:
      kdLine = atof(argv[3]);
    case 3:
      kpLine = atof(argv[2]);
    case 2:
      remoteHost = argv[1];
      break;
    default:
      remoteHost = "127.0.0.1";
      printf("Defaulting to 127.0.0.1\n");
  }
  printf("Gains: kp = %f; kd = %f\n", kpLine, kdLine);
  return true;
}

namespace barrett {
  namespace systems {
    v_type msg_tmp;
    barrett::systems::ExposedOutput<v_type> message;

    class BalisticForce : public HapticObject {
      BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

    public:
      /** 
       * BalisticForce
       */
      BalisticForce(const cp_type& center, 
      
          const std::string& sysName = "BalisticForce") :
        HapticObject(sysName),
        c(center),
        depth(0.0), dir(0.0) 
      {}
      virtual ~BalisticForce() { mandatoryCleanUp(); }

      const cp_type& getCenter() const { return c; }

    protected:
      /** 
       * operate
       */
      virtual void operate() {
        pos = input.getValue();
        for (int i=0;i<3;i++){
          dir[i] = 0.0;
        }
        if (!thresholdMet){
          cforce = c - pos;
          if ((barrett::math::sign(forceThreshold)==1 && cforce[XorYorZ] >= forceThreshold) ||
            (barrett::math::sign(forceThreshold)==-1 && cforce[XorYorZ] <= forceThreshold))
          {
            thresholdMet = true;
            std::cout << "threshold met" << std::endl;
          }
          
          depth = cforce.norm();
          dir[XorYorZ] = cforce[XorYorZ];
        }else{
          depth = 0.0;
        }
        for (int i=0;i<3;i++){
          msg_tmp[i] = pos[i];
        }
        for (int i=3;i<6;i++){
          msg_tmp[i]=0;
        }
        msg_tmp[XorYorZ+3] = UpOrDown * targetDistance;
        
        message.setValue(msg_tmp);
        
        depthOutputValue->setData(&depth);
        directionOutputValue->setData(&dir);
      }

      cp_type c;

      // state & temporaries

      double depth;
      cf_type dir;
      

    private:
      DISALLOW_COPY_AND_ASSIGN(BalisticForce);

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

  }
}

namespace barrett {
  namespace systems {

    class HapticLine : public HapticObject {
      BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

    public:
      /** 
       * HapticLine
       */
      HapticLine(const cp_type& center, 
          const std::string& sysName = "HapticLine") :
        HapticObject(sysName),
        c(center),
        depth(0.0), dir(0.0)
      {}
      virtual ~HapticLine() { mandatoryCleanUp(); }

      const cp_type& getCenter() const { return c; }

    protected:
      /** 
       * operate
       */
      virtual void operate() {
        inputForce = c - input.getValue();
        inputForce[XorYorZ] = 0;
        
        depth = inputForce.norm();
        dir = inputForce;
        depthOutputValue->setData(&depth);
        directionOutputValue->setData(&dir);
      }

      cp_type c;

      // state & temporaries
      cf_type inputForce;

      double depth;
      cf_type dir;

    private:
      DISALLOW_COPY_AND_ASSIGN(HapticLine);

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

  }
}


namespace cube_sphere {
  /** 
   * exit_program_callback
   *
   * When killed from outside (by GUI), this allows a graceful exit.
   */
  void exit_program_callback(int signum) { game_exit = true; }
}  // namespace cube_sphere


/**
 * scale
 */
cf_type scale(boost::tuple<cf_type, double> t) {
  return t.get<0>() * t.get<1>();
}


/**
 * instantiate_proficio
 *
 * Instantiate all of the BURT parameters
 * These are not task specific so...
 */
template <size_t DOF>
void instantiate_proficio( barrett::ProductManager& product_manager,  // NOLINT
                  barrett::systems::Wam<DOF>& wam,           // NOLINT
                  const cp_type system_center,
                  const Config& side)
{
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
  wam.gravityCompensate();
  std::srand(time(NULL)); //initialize the random seed
  barrett::SafetyModule* safety_module = product_manager.getSafetyModule();
  barrett::SafetyModule::PendantState ps;
  safety_module->getPendantState(&ps);
  
  std::string filename = "calibration_data/wam3/";
  if (side == LEFT) {
    filename = filename + "LeftConfig.txt";
  } else if (side == RIGHT) {
    filename = filename + "RightConfig.txt";
  }  

  // Catch kill signals if possible for a graceful exit.
  signal(SIGINT, cube_sphere::exit_program_callback);
  signal(SIGTERM, cube_sphere::exit_program_callback);
  signal(SIGKILL, cube_sphere::exit_program_callback);

  proficio::systems::UserGravityCompensation<DOF> gravity_comp(
      barrett::EtcPathRelative(filename).c_str());
  gravity_comp.setGainZero();

  // Instantiate Systems
  NetworkHaptics<DOF> network_haptics(product_manager.getExecutionManager(),
                                      remoteHost, &gravity_comp);
  
  const cp_type ball_center(0.4, -0.15, 0.05);
  wam.moveTo(system_center);
  printf("Done Moving Arm! \n");
  const cp_type box_center(0.35, 0.2, 0.0);
  const barrett::math::Vector<3>::type box_size(0.2, 0.2, 0.2);

  barrett::systems::BalisticForce ball(ball_center);
  barrett::systems::HapticLine line(ball_center);
  
  barrett::systems::Summer<cf_type> direction_sum;
  barrett::systems::Summer<double> depth_sum;
  barrett::systems::PIDController<double, double> pid_controller;
  barrett::systems::Constant<double> zero(0.0);
  barrett::systems::TupleGrouper<cf_type, double> tuple_grouper;
  
  barrett::systems::Callback<boost::tuple<cf_type, double>, cf_type> mult(  // NOLINT
      scale);
      
  barrett::systems::ToolForceToJointTorques<DOF> tf2jt;
  barrett::systems::Summer<jt_type, 3> joint_torque_sum("+++");
  // EDIT FOR VIBRATIONS
  jt_type jtLimits(45.0);
  jtLimits[2] =25.0;
  jtLimits[0] = 55.0;
  proficio::systems::JointTorqueSaturation<DOF> joint_torque_saturation(
      jtLimits);
  // EDIT FOR VIBRATIONS
  v_type dampingConstants(20.0);
  dampingConstants[2] = 10.0;
  dampingConstants[0] = 50.0;
  jv_type velocity_limits(1.7);
  proficio::systems::JointVelocitySaturation<DOF> velsat(dampingConstants,
                                                         velocity_limits);

  barrett::systems::Normalize<cf_type> normalizeHelper;
  barrett::systems::Magnitude<cf_type, double> magnitudeHelper;
  
  jv_type joint_vel_filter_freq(20.0);
  barrett::systems::FirstOrderFilter<jv_type> joint_vel_filter;
  joint_vel_filter.setLowPass(joint_vel_filter_freq);

  // configure Systems
  pid_controller.setKp(kpLine);
  pid_controller.setKd(kdLine);

  // line up coordinate axis with python visualization
  barrett::systems::modXYZ<cp_type> mod_axes;
  mod_axes.negX();
  mod_axes.negY();
  
  mod_axes.xOffset(0.85);
  if (side == LEFT) {
    mod_axes.yOffset(0.27);
  } else if (side == RIGHT) {
    mod_axes.yOffset(-0.27);
  }
  mod_axes.zOffset(-0.2);
  
  

  // line up forces so that they correlate correctly with python visualization
  barrett::systems::modXYZ<cf_type> mod_force;
  mod_force.negX();
  mod_force.negY();
  barrett::systems::connect(wam.jpOutput, gravity_comp.input);
  barrett::systems::connect(wam.jvOutput, joint_vel_filter.input);
  barrett::systems::connect(joint_vel_filter.output, velsat.input);

  barrett::systems::connect(wam.toolPosition.output, mod_axes.input);
  barrett::systems::forceConnect(barrett::systems::message.output, network_haptics.input);
  barrett::systems::connect(mod_axes.output, ball.input);
  barrett::systems::connect(mod_axes.output, line.input);

  barrett::systems::connect(ball.directionOutput, direction_sum.getInput(0));
  barrett::systems::connect(line.directionOutput, direction_sum.getInput(1));

  barrett::systems::connect(wam.kinematicsBase.kinOutput, tf2jt.kinInput);
  barrett::systems::connect(direction_sum.output, magnitudeHelper.input);
  barrett::systems::connect(direction_sum.output, normalizeHelper.input);
  barrett::systems::connect(normalizeHelper.output, tuple_grouper.getInput<0>());
  
  barrett::systems::connect(magnitudeHelper.output, pid_controller.referenceInput);
  barrett::systems::connect(zero.output, pid_controller.feedbackInput);
  barrett::systems::connect(pid_controller.controlOutput, tuple_grouper.getInput<1>());

  barrett::systems::connect(tuple_grouper.output, mult.input);
  barrett::systems::connect(mult.output, mod_force.input);
  barrett::systems::connect(mod_force.output, tf2jt.input);
  barrett::systems::connect(tf2jt.output, joint_torque_sum.getInput(0));
  barrett::systems::connect(gravity_comp.output, joint_torque_sum.getInput(1));
  barrett::systems::connect(velsat.output, joint_torque_sum.getInput(2));
  barrett::systems::connect(joint_torque_sum.output, joint_torque_saturation.input);

  // adjust velocity fault limit
  product_manager.getSafetyModule()->setVelocityLimit(1.5);
  product_manager.getSafetyModule()->setTorqueLimit(3.0);
  wam.idle();
  barrett::systems::connect(joint_torque_saturation.output, wam.input);
}


/**
 * get_timestamp
 *
 * Get time since January 1, 1970 (probably)
 * @returns {unsigned long long} 
 */
unsigned long long getTimestamp()
{
  struct timeval tv;
  unsigned long long milliseconds_since_epoch =
    (unsigned long long)(tv.tv_sec) * 1000 +
    (unsigned long long)(tv.tv_usec) / 1000;
  
  return milliseconds_since_epoch;
}


/**
 * proficio_main
 *
 * Run experiment in BURT robot
 */
template <size_t DOF>
int proficio_main(int argc, char** argv,
                  barrett::ProductManager& product_manager,  // NOLINT
                  barrett::systems::Wam<DOF>& wam,           // NOLINT
                  const Config& side) {
  
  // Initializing Dragonfly
  Dragonfly_Module mod( MID_CUBE_SPHERE, 0);
  try
  {
    mod.ConnectToMMM();
  }
  catch( exception &e)
	{
		std::cout << "Unknown Exception!" << e.what() << std::endl;
	}
  
  // Subscribe to executive messages
  mod.Subscribe( MT_TRIAL_INPUT);
  mod.Subscribe( MT_TASK_STATE_CONFIG);
  
  // Instantiate Proficio
  const cp_type system_center(0.450, -0.120, 0.250);
  instantiate_proficio(product_manager, wam, system_center, side);

  // Run Trial
  int trialNumber = 1;
  cp_type cp;
  cp_type target_center;
  jt_type jt;
  target_center[0] = 0.439;
  target_center[1] = 0.417;
  target_center[2] = 0.366;
  double target_error = 0.03;
  
  bool taskComplete = false;
  bool taskSuccess = false;
  bool hasError = false;
  int state = RESET;
  
  std::deque<double> scores;
	CMessage Consumer_M;
  std::string labels [5] = {"XorYorZ (0 -> x, 1 -> y, 2 -> z):                                 ",
							"UpOrDown (-1 -> negative direction, 1 -> positive direction):     ",
							"forceThreshold:                                                   ",
							"targetDistance:                                                   ",
							"targetError:                                                      "};
  
  bool trialCompleted = true;
  
  // Todo: these shouldn't be modifiable easily.
  double targetWidth = 0.28125;
  double trackLength = 0.53125; //0.5234375; actual reachign distance
  // double zDepth = 0
  double targetReached = false;
  
  // declare other params
  double angle, distance, comboInd, force;
 
  TargetZone * trialManager = new TargetZone(system_center[0], system_center[1],
                                          system_center[2], targetWidth,
                                          trackLength*2); 
  EnumParser<STATES> parser;
  int taskState = 0; // Experiment state
  int trialState = 0; // Burt trail state
  
  // TODO: timers should be passed from executive
  int rampTimeF, holdTimeF, rampTimeT, holdTimeT;
  rampTimeF = holdTimeF = rampTimeT = holdTimeT = 50*CLOCKS_PER_SEC;
  
  // Timer
  clock_t timer;
  //boost::mutex mtx;
  
  while (true) {  // Allow the user to stop and resume with pendant buttons
		cp = barrett::math::saturate(wam.getToolPosition(), 9.999);
		// this code is to be sent to the python visualization
    /*
		MDF_RT_POSITION_FEEDBACK pos_data;
		pos_data.distanceFromCenter = std::abs(cp[XorYorZ] - system_center[XorYorZ]) * 1240 / 0.2;
		CMessage pos_M( MT_RT_POSITION_FEEDBACK);
		pos_M.SetData( &pos_data, sizeof(pos_data));
		//mod.SendMessageDF( &pos_M); 
		
		// send messages signifying how far it is along the track
		// receive messages from consumer where the target is
		// jt = barrett::math::saturate(wam.getJointTorques(), 99.99);
		MDF_FORCE_FEEDBACK force_data;
		force_data.x = cforce[0];
		force_data.y = cforce[1];
		force_data.z = cforce[2];
		CMessage force_M( MT_FORCE_FEEDBACK);
		force_M.SetData( &force_data, sizeof(force_data));
		//mod.SendMessageDF( &force_M); */
    
    MDF_BURT_STATUS burt_status_data;
    
    //------------- Ping for these stats-----------------------------    
    // Set Task state
    burt_status_data.task_complete = taskComplete;
    burt_status_data.task_success = taskSuccess;
    burt_status_data.timestamp = getTimestamp();
    burt_status_data.state = state;
    burt_status_data.error = hasError;
    
    // Set Force Data
    burt_status_data.force_x = cforce[0];
    burt_status_data.force_y = cforce[1];
    burt_status_data.force_z = cforce[2];
    
    // Set Position Data  TODO: MOVE CONVERSION ELSEWHERE
    burt_status_data.pos_x = cp[0]* 1280 / 0.2; // Assume this is accurate
    burt_status_data.pos_y = cp[1]* 1280 / 0.2; // TODO: check that cp has the right value
    burt_status_data.pos_z = cp[2]* 1280 / 0.2; // and is set properly
    
    // Send Message
    CMessage M( MT_BURT_STATUS );
    M.SetData( &burt_status_data, sizeof(burt_status_data) );
    mod.SendMessageDF( &M );
    
    //std::cout << "while..." << std::endl;
    //-----------------------------------------------------------------
		// ** POSITION JUDGE**
    switch(state)
    {
      // Run the experiment
      case START: //NOT SET HERE
      {
        wam.moveTo(system_center);
        //std::cout << "Starting..." << std::endl;
        state = FORCE_RAMP; // intentional fall-through
        timer = clock();
      }
      case FORCE_RAMP:
      { // get to correct force in X time
        //std::cout << "State: " << state << std::endl; 
        if (rampTimeF < (clock() - timer))
        {
          //mtx.lock();
          std::cout << "OUT OF TIME A" << state << std::endl;
          state = FAIL;
          taskSuccess = false;
          taskComplete = true;
          //mtx.unlock();
          break;
        }
        // Check force vector
        if (!trialManager->adequateForce(cforce[0], cforce[1], force, 0.1))
        {
          break;
        }
        //mtx.lock();
        state = FORCE_HOLD;
        //mtx.unlock();
        timer = clock(); // reset timer and intentionally fall through
      }
      case FORCE_HOLD:
      { // hold force for X time (?)
        //std::cout << "State: " << state << std::endl; 
        //mtx.lock();
        state = TARGET_MOVE;
        //mtx.unlock();
        timer = clock();
        //break; // TODO: intentional fall-through
      }
      case TARGET_MOVE:
      { 
        std::cout << "State: " << state << std::endl; 
        // Move to target    
        if (rampTimeT < (clock() - timer))
        {
          //mtx.lock();
          std::cout << "OUT OF TIME B" << state << std::endl;
          state = FAIL;
          taskSuccess = false;
          taskComplete = true;
          //mtx.unlock();
          break;
        }
        // If exist track, fail
        /*
        if (!trialManager->inZone(cp[0], cp[1])) // or time is up
        {
          //mtx.lock();
          state = FAIL;
          taskSuccess = false;
          taskComplete = true;
          //mtx.unlock();
          break;
        } */
        else if (trialManager->inTarget(cp[0], cp[1]))
        { // in target zone
          timer = clock();
          //mtx.lock();
          state = TARGET_HOLD; // intentional fall-through 
          //mtx.unlock();   
        }
        else
        {
          break;
        }
      }
      case TARGET_HOLD:
      {
        //std::cout << "State: " << state << std::endl; 
        if (holdTimeT > (clock() - timer))
        {
          // if held long enough then success
          //mtx.lock();
          state = SUCCESS;
          taskSuccess = true;
          taskComplete = true;
          //mtx.unlock();
        }
        // if exits zone, then fail
        /*
        if (!trialManager->inTarget(cp[0], cp[1]));
        {
          //mtx.lock();
          state = FAIL;
          taskSuccess = false;
          taskComplete = true;
          //mtx.unlock();
        } */
        break;
      }
      // Reach target, send success message
      case SUCCESS:
        std::cout << "State success: " << state << std::endl; 
      // Did not reach target send fail
      case FAIL:
        std::cout << "State fail: " << state << std::endl; 
      // reset parameters and prep for next round
      case RESET: //NOT SET HERE
      {
        taskComplete = false;
        std::cout << "State reseting: " << state << std::endl; 
        wam.moveTo(system_center);
        mod.ReadMessage( &Consumer_M);
        if (Consumer_M.msg_type == MT_TASK_STATE_CONFIG)
        {
          std::cout << "New message" << std::endl; 
          MDF_TASK_STATE_CONFIG task_state_data;
          Consumer_M.GetData( &task_state_data);
          distance = task_state_data.distance * trackLength;
          state = task_state_data.state;
          angle = task_state_data.direction;
          force = task_state_data.force;
          
          // Set new trial parameters
          trialManager->setTarget(distance, targetWidth);
          trialManager->rotate(angle);
         
          comboInd = task_state_data.target_combo_index;
         
          // Other for later....
          //rep_num
          //idle_timeout
          //timeout
          //ts_time
        }
        break;
      }
      // Error occurred stop everything until safe to restart
      case ERROR:
      {
        std::cout << "State: " << state << std::endl; 
        // TODO
        wam.moveTo(system_center);
        break;
      }
      case REST: //NOT SET HERE
      {
        std::cout << "State: " << state << std::endl; 
        wam.moveTo(system_center);
        // lock in place
        break;
      }
      default:
      {
       std::cout << "State: " << state << std::endl; 
      }
    } // ** END POSITION JUDGE ** 
    
	if (product_manager.getSafetyModule()->getMode() == barrett::SafetyModule::IDLE)
  {
		wam.moveHome();
		return 0;
  }
  barrett::btsleep(0.02);
#ifndef NO_CONTROL_PENDANT
#endif
  }
  return 0;
}
