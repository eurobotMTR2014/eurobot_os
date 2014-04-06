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


typedef struct Objective_t {
   float x;
   float y;
   float phi;
   float k;
   bool stop;
} Objective;

static state currentstate;
static Objective goals[GOALS_POOL];
static char firstgoal = 0; // When firstgoal == nextgoals,
static char nextgoals = 0; // there is no desired goal.
//static char stopping = 0;
static int rv = 0;
static int lv = 0;
/*
static portTickType cpu_tick;
static Encoder er;
static Encoder last_er;
static Encoder el;
static Encoder last_el;
*/

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
   /*
   IntMasterDisable();
   currentstate.stop = true;
   IntMasterEnable();
   */
   world_set_stop_state(true);

   while (!controlStop)
   {
      //UARTprintf("Control alive");
      if (ROBOT_start) {
        ctrl_refresh(&xLastWakeTime);
      }
      vTaskDelayUntil (&xLastWakeTime, (10 / portTICK_RATE_MS));
   }

   while(true);
}
/*
void ctrl_initControl(float x, float y, float phi) {
   UARTprintf("Start control init.\n"); // %d = int, %u = uint.
   ctrl_resetState(x, y, phi, true);
   

   // !!! QEI0 -> QEI1  // Removed changement
   er.ulBase = QEI0_BASE;
   updateEncoder(&er);
   last_er.ulBase = QEI0_BASE;
   updateEncoder(&last_er);
   el.ulBase = QEI1_BASE;
   updateEncoder(&el);
   last_el.ulBase = QEI1_BASE;
   updateEncoder(&last_el);

   UARTprintf("Done control init.\n"); // %d = int, %u = uint. 
   
}
*/

void ctrl_refresh(portTickType* xLastWakeTime) {
      if (!goals_full_or_empty() && !world_get_stop_state()) {
         //world_update_state(); // Calcule positon courante et angle courant
         planner();
         tracker(xLastWakeTime);
      }
      else {
         /* DEBUG
         if(firstgoal == nextgoals)
            UARTprintf("firstgoal != nextgoals\n");
         else if(currentstate.stop)
            UARTprintf("!currentstate.stop\n");
         */
         //world_update_state();
         ctrl_stop(xLastWakeTime);
      }
}

bool ctrl_restart(portTickType* xLastWakeTime) {

   // No goals or not stopped
   if(goals_full_or_empty() || !world_get_stop_state()){
      ctrl_stop(xLastWakeTime);
      return false;
   }
   // There is a goal and we are stopped
   else{
      #ifdef PID_CONTROLLER_ENABLE
         cpu_last_tick = xTaskGetTickCount();
         pid_integral = 0.0;
      #endif

      planner();

      if (custom_sqrt(u1*u1 + u2*u2) <= EPSILON) { // If we had reached the goal.
         removeCurrentGoalState();
      }

      world_set_stop_state(false);

      return true;
   }



   /*
   if (firstgoal == nextgoals || !currentstate.stop) {
      for (int i = 0; i < 3; i++) {
         servoLeft(xLastWakeTime, 0x00, 0x00);
         servoRight(xLastWakeTime, 0x00, 0x00);
         servoSync();
      }
      IntMasterDisable();
      currentstate.stop = true;
      IntMasterEnable();
//      stopping = 0;
      return false;
   }
   else { // If there is a goal, and we are stopped.
//      stopping = 0;
      #ifdef PID_CONTROLLER_ENABLE
         cpu_last_tick = xTaskGetTickCount();
         pid_integral = 0.0;
      #endif
      planner(); // In case goals have changed.
      if (custom_sqrt(u1*u1 + u2*u2) <= EPSILON) { // If we had reached the goal.
         removeCurrentGoalState();
      }
      IntMasterDisable();
      currentstate.stop = false; //Let's move.
      IntMasterEnable();
//      ctrl_refresh(xLastWakeTime);
//      if (!currentstate.stop) { //If we have restarted.
//         return true;
//      }
//      else {
//         return false; //Something went wrong: we restarted and stopped. Maybe you just don't have any more goal?
//      }
      return true;
   }
   */
}


/*
void ctrl_flush() {
   nextgoals = firstgoal;
}
*/


void ctrl_stop(portTickType* xLastWakeTime) {
/*
//   stopping++;
//   int backwardsright = rv & 0x0400;
//   int backwardsleft = lv & 0x0400;
//
//   int temprv = (rv & 0x03FF) >> stopping;
//   int templv = (lv & 0x03FF) >> stopping;
//
//   if (stopping > 2) {
//      temprv = temprv >> 1;
//      templv = templv >> 1;
//   }
//   if (stopping > 4){
//      temprv = temprv >> 1;
//      templv = templv >> 1;
//   }
//
//   temprv = temprv ^ backwardsright;
//   templv = templv ^ backwardsleft;
//
//   servoLeft(xLastWakeTime, *((char*) &templv + 1), *((char*) &templv));
//   servoRight(xLastWakeTime, *((char*) &temprv + 1), *((char*) &temprv));
//   servoSync();

//    if (stopping > 5) {
      rv = 0;
      lv = 0;
//      stopping = 0;
      IntMasterDisable();
      currentstate.stop = true;
      IntMasterEnable();
      for (int i = 0; i < 3; i++){
         //UARTprintf("SERVO 1\n");
         servoLeft(xLastWakeTime, 0x00, 0x00);
         //UARTprintf("SERVO 2\n");
         servoRight(xLastWakeTime, 0x00, 0x00);
         //UARTprintf("SERVO 3\n");
         servoSync();
         //UARTprintf("SERVO 4\n");
      }
//   }
*/
      world_set_stop_state(true);

      for (int i = 0; i < 3; i++){
         servoLeft(xLastWakeTime, 0x00, 0x00);
         servoRight(xLastWakeTime, 0x00, 0x00);

         servoSync();
      }
}

/*
const state* ctrl_getCurrentState() {
   return &currentstate;
}
*/



void removeCurrentGoalState() {
   /*
   firstgoal = (firstgoal + 1) % GOALS_POOL;
   */
   #ifdef PID_CONTROLLER_ENABLE
      pid_integral = 0.0;
   #endif

   world_goal_remove_peek();
}


void updateState() 
{
   // to remove from here
}

void planner() {
   PositionGoal goal = world_pick_next_goal();
   Coord current = world_get_coord();

   u1 = goal.x - current.x;
   u2 = goal.y - current.y;
/*
   Objective* d = &goals[firstgoal];
   u1 = d->x - currentstate.x;
   u2 = d->y - currentstate.y;
*/
//   UARTprintf("planned trajectory (u1 = %d, ", (int) (u1 * 1000.0)); // %d = int, %u = uint.
//   UARTprintf("u2 = %d) from goal %d\n", (int) (u2 * 1000.0), firstgoal); // %d = int, %u = uint.

}

void tracker(portTickType* xLastWakeTime) {
   PositionGoal goal = world_pick_next_goal();
   State currentstate = world_get_state();

   if(custom_sqrt(u1*u1 + u2*u2) <= EPSILON) {
      if(goal.stop){
         ctrl_stop(xLastWakeTime);
         return;
      }
      else{
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
   rv = (int) right_velocity;
   lv = (int) left_velocity;
   if (left_velocity != custom_abs(left_velocity)) {
      lv = custom_absinthe(lv) + 0x0400;
   }
   if (right_velocity != custom_abs(right_velocity)) {
      rv = custom_absinthe(rv) + 0x0400;
   }

   for (int i = 0; i < 3; i++) {
      servoLeft(xLastWakeTime, *((char*) &lv + 1), *((char*) &lv));
      servoRight(xLastWakeTime, *((char*) &rv + 1), *((char*) &rv));
      servoSync();
   }


   /*
   Objective* d = &goals[firstgoal];
   if (custom_sqrt(u1*u1 + u2*u2) <= EPSILON) { // WE ARE DONE, check the stop condition. If there is none, throw the old state, goto next.
//      UARTprintf("Dude, awesome, we are done.\n"); // %d = int, %u = uint.
      if (d->stop) {
//         UARTprintf("goals firstgoal had a stop in it.\n"); // %d = int, %u = uint.
           ctrl_stop(xLastWakeTime);
//         servoLeft(xLastWakeTime, 0x00, 0x00);
//         servoRight(xLastWakeTime, 0x00, 0x00);
//         servoSync();
         return;
      }
      else {
         removeCurrentGoalState();
         planner();
         //ctrl_refresh(xLastWakeTime);
      }
   }
   float v;
   float w;
   #ifdef CONTROL_TRICK_ENABLE
      //#define K 10 //Parameter to tweak! I might even put it into a variable, maybe. Perhaps. Mayhaps.
      float k = d->k;
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
   rv = (int) right_velocity;
   lv = (int) left_velocity;
   if (left_velocity != custom_abs(left_velocity)) {
      lv = custom_absinthe(lv) + 0x0400;
   }
   if (right_velocity != custom_abs(right_velocity)) {
      rv = custom_absinthe(rv) + 0x0400;
   }

   for (int i = 0; i < 3; i++) {
      servoLeft(xLastWakeTime, *((char*) &lv + 1), *((char*) &lv));
      servoRight(xLastWakeTime, *((char*) &rv + 1), *((char*) &rv));
      servoSync();
   }
   */
}

/*
void ctrl_resetState(float x, float y, float phi, bool stop) {
   IntMasterDisable();
   currentstate.x = x;
   currentstate.y = y;
   currentstate.phi = phi;
   currentstate.stop = stop;
   IntMasterEnable();
}

void ctrl_calibx(float x) {
   IntMasterDisable();
   currentstate.x = x;
   IntMasterEnable();
}

void ctrl_caliby(float y) {
   IntMasterDisable();
   currentstate.y = y;
   IntMasterEnable();
}

void ctrl_calibphi(float phi) {
   IntMasterDisable();
   currentstate.phi = phi;
   IntMasterEnable();
}


char ctrl_getForward() {
   if (currentstate.stop) {
      return 2;
   }
   Objective* d = &goals[firstgoal];
   if (d->k > 0) {
      return 1;
   }
   return 0;
}

*/
