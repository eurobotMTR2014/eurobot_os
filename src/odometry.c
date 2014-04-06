#include "odometry.h"

#pragma GCC diagnostic ignored "-Wchar-subscripts"
#pragma GCC diagnostic ignored "-Wunused-parameter"

static portTickType cpu_tick;

extern bool ROBOT_start;

void odometryTask (void* odometryTask){
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	while(!ROBOT_start)
		vTaskDelayUntil (&xLastWakeTime, (10 / portTICK_RATE_MS));

	while(ROBOT_start){
		world_update_state();
      vTaskDelayUntil (&xLastWakeTime, (10 / portTICK_RATE_MS));
	}

   while(1){}
}

void updateEncoder(volatile Encoder* enc) 
{
   enc->time = xTaskGetTickCount();
   enc->tickvalue = (int) QEIPositionGet(enc->ulBase);
   enc->forward = (QEIDirectionGet(enc->ulBase) == (unsigned long) 1);
}

State getDisplacement(Encoder er, Encoder el, Encoder last_er, Encoder last_el, float cs_phi)
{
   // even when on the reversed side.
   int dtr;
   if ((er.forward && (er.tickvalue >= last_er.tickvalue)) ||
      (!er.forward && (er.tickvalue <= last_er.tickvalue))) 
   {
      dtr = ((float) (er.tickvalue - last_er.tickvalue));
   }
   else if (er.forward) 
   {
      dtr = ((float) ((er.tickvalue + 1024) - last_er.tickvalue));
   }
   else 
   {
      dtr = ((float) (er.tickvalue - (last_er.tickvalue + 1024)));
   }

   int dtl;
   if ((el.forward && (el.tickvalue >= last_el.tickvalue)) ||
      (!el.forward && (el.tickvalue <= last_el.tickvalue))) 
   {
      dtl = ((float) (el.tickvalue - last_el.tickvalue));
   }
   else if (el.forward) 
   {
      dtl = ((float) ((el.tickvalue + 1024) - last_el.tickvalue));
   }
   else 
   {
      dtl = ((float) (el.tickvalue - (last_el.tickvalue + 1024)));
   }
   
   /* dtl ou dtr > 0 -> forward */

   /* Trick: when going for instance Forward, then Backwards but last_er.tick was still < er.tick */
   if (custom_abs(dtr) > 800) {
      if (dtr > 0) {
         dtr = 1024 - dtr;
      }
      else {
         dtr = dtr + 1024;
      }
   }

   if (custom_abs(dtl) > 800) {
      if (dtl > 0) {
         dtl = 1024 - dtl;
      }
      else {
         dtl = dtl + 1024;
      }
   }

   float dr = (float) dtr * ENC_TRNSF;
   float dl = (float) dtl * ENC_TRNSF;

   float dphi = ((dr - dl)/INTER_WHEEL);
   float dx = ((dr + dl)/2) * custom_cos(cs_phi); // Approximations! Terms in [1 - cos(dphi)] neglected
   float dy = ((dr + dl)/2) * custom_sin(cs_phi); // Approximations! Terms in sin(dphi) taken as dphi.

   //UARTprintf("dx : %d | dy : %d | dphi : %d\n", (int) dx, (int) dy, (int) dphi);

   State ds;

   ds.x = dx;
   ds.y = dy;
   ds.phi = dphi;

   return ds;
}