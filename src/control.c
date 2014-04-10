#include "control.h"

#pragma GCC diagnostic ignored "-Wchar-subscripts"
#pragma GCC diagnostic ignored "-Wunused-parameter"

#ifndef GOALS_POOL
   #define GOALS_POOL 500
#endif
#ifndef EPSILON
   #define EPSILON 50.0 // e where we say we're at the goal.
#endif

#define CONTROL_TRICK_ENABLE

#ifdef PID_CONTROLLER_ENABLE
   #define KI 21.27
   #define KP 9.09
#endif
#ifdef CONTROL_TRICK_ENABLE
   #define K 70 //Parameter to tweak! I might even put it into a variable, maybe. Perhaps. Mayhaps.
// K << : Trajectoire smooth et lisse (grande courbure) ; K >> : Trajectoire dure
#endif
// UARTprintf("Servo Protocol error: %d\n", rval); // %d = int, %u = uint.
//             UARTprintf("Servo Protocol error: %d\n", (int) (rval * 1000.0)); // Where rval is float.

extern volatile bool ROBOT_start;
extern volatile bool controlStop;
extern volatile bool force_angle;
extern volatile float forced_angle_value;

static float u1;
static float u2;

#ifdef PID_CONTROLLER_ENABLE
   static float pid_integral = 0.0;
   static portTickType cpu_last_tick;
   //static float pid_last_error = 0.0; // For derivative term.
#endif

void removeCurrentGoalState();
void updateState();
void planner();
void tracker(portTickType* xLastWakeTime);
void turnToAngle(portTickType* xLastWakeTime,float forced_angle);

void controlTask (void* pvParameters)
{
   portTickType xLastWakeTime;
   xLastWakeTime = xTaskGetTickCount();
/*
   while (!ROBOT_start)
   {
      vTaskDelayUntil (&xLastWakeTime, (10 / portTICK_RATE_MS));
   }
*/
   pln2("Control launched");
  
   world_set_stop_state(true);

   while (!controlStop)
   {
      //UARTprintf("Control alive");
      if (ROBOT_start) 
        ctrl_refresh(&xLastWakeTime);
      
      vTaskDelayUntil (&xLastWakeTime, (10 / portTICK_RATE_MS));
   }

   while(true){}
}

void ctrl_refresh(portTickType* xLastWakeTime) 
{
   // if the goal buffer still has some goals in it (or if the robot has to stop)
   if (!goals_full_or_empty() && !world_get_stop_state()) 
   {
      planner();
      tracker(xLastWakeTime);
   }
   else
      ctrl_stop(xLastWakeTime);
}

bool ctrl_restart(portTickType* xLastWakeTime) 
{
   // No goals or not stopped
   if(goals_full_or_empty() || !world_get_stop_state())
   {
      ctrl_stop(xLastWakeTime);
      return false;
   }
   // There is a goal and we are stopped
   else
   {
      #ifdef PID_CONTROLLER_ENABLE
         cpu_last_tick = xTaskGetTickCount();
         pid_integral = 0.0;
      #endif

      planner();

      if (custom_sqrt(u1*u1 + u2*u2) <= EPSILON) // If we had reached the goal.
         removeCurrentGoalState();

      world_set_stop_state(false);

      return true;
   }
}


void ctrl_stop(portTickType* xLastWakeTime) 
{
   world_set_stop_state(true);

   for (int i = 0; i < 3; i++)
   {
      servoLeft(xLastWakeTime, 0x00, 0x00);
      servoRight(xLastWakeTime, 0x00, 0x00);

      servoSync();
   }
}

void removeCurrentGoalState() 
{
   #ifdef PID_CONTROLLER_ENABLE
      pid_integral = 0.0;
   #endif

   world_goal_remove_peek();
}

void planner() 
{
   PositionGoal goal = world_pick_next_goal();
   Coord current = world_get_coord();

   u1 = goal.x - current.x;
   u2 = goal.y - current.y;
}

void tracker(portTickType* xLastWakeTime) 
{
   PositionGoal goal = world_pick_next_goal();
   State currentstate = world_get_state();

   if(custom_sqrt(u1*u1 + u2*u2) <= EPSILON) 
   {
      if(goal.stop)
      {
         ctrl_stop(xLastWakeTime);
         return;
      }
      else
      {
         removeCurrentGoalState();
         planner();
      }
   }


   float v;
   float w;
   #ifdef CONTROL_TRICK_ENABLE
      //#define K 10 //Parameter to tweak! I might even put it into a variable, maybe. Perhaps. Mayhaps.
      float k = goal.k;
      v = custom_cos(currentstate.phi)*u1 + custom_sin(currentstate.phi)*u2;
      w = custom_cos(currentstate.phi)*u2/k - custom_sin(currentstate.phi)*u1/k;
   #endif

//   UARTprintf("Forward controller velocity = %X\n", (int) (v * 1000.0));
//   UARTprintf("Angular controller velocity = %X\n", (int) (w * 1000.0));

   float right_velocity = (2.0*v + w*INTER_WHEEL)/WHEEL_DIAM;
   float left_velocity = (2.0*v - w*INTER_WHEEL)/WHEEL_DIAM;
   //Now let's scale the velocities between [-1, 1]
   if (custom_abs(right_velocity) > custom_abs(left_velocity)) {
      left_velocity = left_velocity/custom_abs(right_velocity) ;
      right_velocity = right_velocity/custom_abs(right_velocity) ;
   }
   else {
      right_velocity = right_velocity/custom_abs(left_velocity) ;
      left_velocity = left_velocity/custom_abs(left_velocity) ;
   }

   left_velocity = left_velocity * (float) 0x01FF;
   right_velocity = right_velocity * (float) 0x01FF;

   float rv = (int) right_velocity;
   float lv = (int) left_velocity;

   if (left_velocity != custom_abs(left_velocity))
      lv = custom_absinthe(lv) + 0x0400;
   
   if (right_velocity != custom_abs(right_velocity))
      rv = custom_absinthe(rv) + 0x0400;

   for (int i = 0; i < 3; i++) {
      servoLeft(xLastWakeTime, *((char*) &lv + 1), *((char*) &lv));
      servoRight(xLastWakeTime, *((char*) &rv + 1), *((char*) &rv));
      servoSync();
   }
}
