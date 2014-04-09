xQueueHandle odometryQueue = NULL;

static Encoder el;
static Encoder er;
static Encoder last_el;
static Encoder last_er;

static portTickType cpu_tick;

state currentstate;

extern bool ROBOT_start;

void odometryTask (void* odometryTask){
	ortTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	er.ulBase = QEI0_BASE;
	last_er.ulBase = QEI0_BASE;
	el.ulBase = QEI1_BASE;
	last_el.ulBase = QEI1_BASE;

	/* Initialization */
	updateEncoder(&last_el);
	updateEncoder(&last_er);
	updateEncoder(&el);
	updateEncoder(&er);

	updateState();
	currentstate.stop = true;

	while(!ROBOT_start)
		vTaskDelayUntil (&xLastWakeTime, (10 / portTICK_RATE_MS));

	while(ROBOT_start){
		/* Update values of encoders */
		updateEncoder(&el);
		updateEncoder(&er);

		/* Compute the current state */
		updateState();

		/* Send values to the queue */
		xQueueSend(odometryQueue, (void*) &currentstate, 0);

		/* Wait */
		vTaskDelayUntil (&xLastWakeTime, (10 / portTICK_RATE_MS));
	}

  }
}


void updateEncoder(Encoder* enc) {
   enc->time = xTaskGetTickCount();
   enc->tickvalue = (int) QEIPositionGet(enc->ulBase);
   enc->forward = (QEIDirectionGet(enc->ulBase) == (unsigned long) 1);
   //enc->velocity = QEIVelocityGet(enc);

   //UARTprintf("tickvalue = %d\n, ", (int) (enc->tickvalue)); // %d = int, %u = uint.
}


void updateState() {
   last_er.tickvalue = er.tickvalue;
   last_er.time = er.time;
   last_er.forward = er.forward;
   //last_er.velocity = er.velocity;

   last_el.tickvalue = el.tickvalue;
   last_el.time = el.time;
   last_el.forward = el.forward;
   //last_el.velocity = el.velocity;


   //UARTprintf("LEFT : ");
   IntMasterDisable();
   updateEncoder(&el);
   updateEncoder(&er);
   cpu_tick = xTaskGetTickCount();
   IntMasterEnable();
   el.tickvalue = 1023 - el.tickvalue;
   el.forward = !el.forward;
   //ARTprintf("el = %d, ", (int) (el.tickvalue)); // %d = int, %u = uint. IT WORKS TILL HERE
   //UARTprintf("er = %d\n", (int) (er.tickvalue)); // %d = int, %u = uint.
   // Note : can trust forward flag & tickvalue increase when forward
   

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
   float dx = ((dr + dl)/2) * custom_cos(currentstate.phi); // Approximations! Terms in [1 - cos(dphi)] neglected
   float dy = ((dr + dl)/2) * custom_sin(currentstate.phi); // Approximations! Terms in sin(dphi) taken as dphi.
   //UARTprintf("dtr = %d\n", dtr);
   //UARTprintf("dtl = %d\n", dtl);
   //UARTprintf("dr = %d\n", (int) (dr * 1000.0));
   //UARTprintf("dl = %d\n", (int) (dl * 1000.0));
   //UARTprintf("ENC_TRNSF_LOL = %d\n", (int) (ENC_TRNSF * 1000.0));
   //UARTprintf("dphi = %d\n", (int) (dphi * 1000.0));
   //UARTprintf("dx = %d\n", (int) (dx * 1000.0));
   //UARTprintf("dy = %d\n", (int) (dy * 1000.0));

   IntMasterDisable();
   currentstate.phi += dphi;
   if (currentstate.phi < -PI) {
      currentstate.phi += 2*PI;
   }
   if (currentstate.phi > PI) {
      currentstate.phi += -2*PI;
   }
   currentstate.x += dx;
   currentstate.y += dy;
   IntMasterEnable();

   //UARTprintf("state (in mm) (x = %d, ", (int) (currentstate.x * 1.0)); // %d = int, %u = uint.
   //UARTprintf("y = %d, ", (int) (currentstate.y * 1.0)); // %d = int, %u = uint.
   //UARTprintf("phi = %d (in rad*1000))\n", (int) (currentstate.phi * 1000.0)); // %d = int, %u = uint.

   //State updated.

}
