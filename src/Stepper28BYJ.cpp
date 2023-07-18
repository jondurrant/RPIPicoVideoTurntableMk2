/*
 * Stepper28BYJ.cpp
 *
 *  Stepper Motor driven by ULN2003 driver using FreeRTOS
 *
 *
 *  Created on: 24 Dec 2022
 *      Author: jondurrant
 */

#include "Stepper28BYJ.h"
#include "uRosBridge.h"
#include <cstdio>
#include <inttypes.h>

extern"C"{
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include "rosidl_runtime_c/string_functions.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"
}

#include <cmath>


//Structure to use for queue
enum StepperAction {StepperStep, StepperTo, StepperReset, StepperCont};
struct StepperReq {
	StepperAction action;
	int16_t steps;
	float rpm;
	bool cw;
	uint32_t id;
};
typedef struct StepperReq StepperReqT;




/***
 * Constructor
 * @param gp1 - GPIO Pad
 * @param gp2 - GPIO Pad
 * @param gp3 - GPIO Pad
 * @param gp4 - GPIO Pad
 * @param slotGP - GPIO Pad for slot detector
 */
Stepper28BYJ::Stepper28BYJ(
		uint8_t gp1,
		uint8_t gp2,
		uint8_t gp3,
		uint8_t gp4,
		uint8_t hallGP ){
	pGPPAD[0] = gp1;
	pGPPAD[1] = gp2;
	pGPPAD[2] = gp3;
	pGPPAD[3] = gp4;

	xHallGP = hallGP;

	xCmdQ = xQueueCreate( STEPPER_QUEUE_LEN, sizeof(StepperReqT));
	if (xCmdQ == NULL){
		printf("ERROR: Unable to create Queue\n");
	}
	setDelay(14);
}


/***
 * Destructor
 */
Stepper28BYJ::~Stepper28BYJ() {
	if (xCmdQ != NULL){
		vQueueDelete(xCmdQ);
	}
}

/***
 * Get the static depth required in words
 * @return - words
 */
configSTACK_DEPTH_TYPE Stepper28BYJ::getMaxStackSize(){
	return 200;
}

/***
 * Initialise the GPIO
 */
void Stepper28BYJ::init(){
	for (uint8_t i=0; i < 4; i++){
		gpio_init(pGPPAD[i]);
		gpio_set_dir(pGPPAD[i], GPIO_OUT);
		gpio_put(pGPPAD[i], 0);
	}

	gpio_init(xHallGP);
	gpio_set_dir(xHallGP, GPIO_IN);

}

/***
 * Run loop for the agent.
 */
void Stepper28BYJ::run(){

	init();
	setupRosMsgs();


	uint8_t s;
	uint32_t id = 0;

	//printf("Stepper started\n");

	for(;;) {
		bool b = gpio_get(xHallGP);

		if (!b) {
			if (xState == StepperStartHunt){
				xState = StepperCalibrateOne;
				step(100, 14);

				//printf("Stepper moved to Calibrate %d\n", xPos);
			} else if (xState == StepperCalibrateTwo){
				//printf("Number of steps is %d\n", xPos);
				xTotalSteps = xPos;
				xState = StepperCmd;
			}

			xPos = 0;

		}

		//Stop Processing
		if (xState == StepperStop){
			xTargetPos = xPos;
			if (pObserver != NULL){
				pObserver->actionComplete(id);
			}
			xState = StepperCmd;
			id = processQueue();
		}


		if (xPos == xTargetPos){
			if (id != 0){
				//printf("Target reached %d\n", xTargetPos);
				if (pObserver != NULL){
					pObserver->actionComplete(id);
				}
				if (xState == StepperCalibrateOne){
					xState = StepperCalibrateTwo;
					stepTo(0, 14, true);
				}
			}
			id = processQueue();
		}

		if ((id != 0) && (xPos != xTargetPos)){
			doStep();
		}

		if (xState == StepperStartHunt){
			//printf("StepperStartHunt %d\n", xPos);
			doStep();
		}

		//Public Joint State
		if ((xState != StepperCalibrateOne) &&
			(xState != StepperCalibrateTwo) &&
			(xState != StepperStartHunt)){
			pubRosPos();
		}

		//printf("Delay(%d) would be %d\n", xSeq, xDelay[xSeq] );
		vTaskDelay(xDelay[xSeq]);
	}
}


/***
 * Do a single step on the Stepper motor in required direction.
 */
void Stepper28BYJ::doStep(){
	//uint8_t sequence[STEPPER_SEQ_LEN] = {3, 6, 12, 9};
	uint8_t sequence[STEPPER_SEQ_LEN] = {9, 12, 6, 3};

	if (xClockwise){
		xPos = modPos(xPos + 1);
		xSeq = (xSeq + 1) % STEPPER_SEQ_LEN;
	} else {
		xPos = modPos(xPos - 1);
		if (xSeq == 0){
			xSeq = 3;
		} else {
			xSeq = xSeq - 1;
		}
	}
	//printf("Pos %d Target = %d\n", xPos, xTargetPos);
	uint8_t s = sequence[xSeq];
	for (uint8_t b = 0; b < STEPPER_SEQ_LEN; b++){
		uint8_t m = 1 << b;
		if ((m & s) > 0){
			gpio_put(pGPPAD[b], 1);
		} else {
			gpio_put(pGPPAD[b], 0);
		}
	}
	//printf("%d %d %d\n", xPos, xPos % 4, s);
}

/***
 * Step motor through x full steps
 * Possitive is clockwise, negative is counter clockwise
 * @param step - Step count >0 CW, < 0 CCW
 * @param rpm - Revolutions per mininute (under 14 to work)
 * RPM 0 is maximum speed
 */
uint32_t Stepper28BYJ::step(int16_t step, float rpm){
	StepperReqT req;

	req.action = StepperStep;
	req.steps = step;
	req.rpm = rpm;
	req.id = xNextId++;

	if (xCmdQ != NULL){
		BaseType_t res = xQueueSendToBack(xCmdQ, (void *)&req, 0);
		if (res != pdTRUE){
			printf("WARNING: Queue is full\n");
		}
	}
	return req.id;
}


/***
 * Process request from the queue
 */
uint32_t Stepper28BYJ::processQueue(){
	StepperReqT req;
	float delay;

	if (xCmdQ == NULL){
		return 0;
	}

	BaseType_t res = xQueueReceive(xCmdQ, (void *)&req, 0);
	if (res == pdTRUE){
		switch(req.action){
		case StepperStep:
			xTargetPos = modPos(xPos + req.steps);
			if (req.steps < 0){
				xClockwise = false;
			} else {
				xClockwise = true;
			}

			setDelay(req.rpm);

			break;
		case StepperTo:
			xTargetPos = req.steps;
			xClockwise = req.cw;
			setDelay(req.rpm);
			break;
		case StepperCont:
			printf("Step cont\n");
			xTargetPos = -1;
			xClockwise = req.cw;
			setDelay(req.rpm);
			break;
		default:
			break;
		}

		return req.id;
	}
	return 0;
}

/***
 * Calculate the pos of stepper based on modulus maths
 * @param pos
 * @return
 */
int16_t Stepper28BYJ::modPos(int16_t pos){

	if (xState == StepperCmd){
		int16_t p = pos % xTotalSteps;
		if (p < 0){
			p = xTotalSteps + p;
		}
		return p;
	} else {
		return pos;
	}
}

/***
 * Split RPM speed as a set of four delays over the four phases
 * @param rpm - 0 will set to maximum speed
 */
void Stepper28BYJ::setDelay(float rpm){
	float delay;

	if (rpm <= 0.0){
		for (uint8_t i=0; i < STEPPER_SEQ_LEN; i++){
			xDelay[i] = DELAY;
		}
		return;
	}

	delay = (60000.0 / rpm) / (float)xTotalSteps;

	for (uint8_t i=0; i < STEPPER_SEQ_LEN; i++){
		xDelay[i] = delay;
	}

	uint16_t e = (delay * (float)STEPPER_SEQ_LEN) - xDelay[0]*STEPPER_SEQ_LEN;
	switch(e){
	case 1:
		xDelay[1]++;
		break;
	case 2:
		xDelay[1]++;
		xDelay[3]++;
		break;
	case 3:
		xDelay[1]++;
		xDelay[2]++;
		xDelay[3]++;
		break;
	}

}



/***
 * Calibrate the disk and leave at possition zero
 */
void Stepper28BYJ::calibrate(){
	xState = StepperStartHunt;
}

/***
 * Step to a possition
 * @param pos = Possition between 0 and 4047
 * @param rpm - speed, 0 to 14. 0 is max speed
 * @param cw - clockwise
 */
uint32_t Stepper28BYJ::stepTo(int16_t pos, float rpm, bool cw){
	StepperReqT req;

	req.action = StepperTo;
	req.steps = pos;
	req.rpm = rpm;
	req.cw = cw;
	req.id = xNextId++;

	if (xCmdQ != NULL){
		BaseType_t res = xQueueSendToBack(xCmdQ, (void *)&req, 0);
		if (res != pdTRUE){
			printf("WARNING: Queue is full\n");
		}
	}

	return req.id;
}

/***
 * Rotate to Degrees possition (0 to 360)
 * @param deg - Float 0 to 360
 * @param rpm - Rotation speed, under 14
 * @param cw - Clockwise or CCW.
 * @return id of the instruction
 */
uint32_t Stepper28BYJ::stepToDeg(float deg, float rpm, bool cw){
	int16_t pos = (float)xTotalSteps * (deg / 360.0);
	return stepTo(pos, rpm, cw);
}




/***
 * Set Observer
 * @param obs
 */
void Stepper28BYJ::setObserver(StepperObserver *obs){
	pObserver = obs;
}

/***
 * Get state of stepper motor
 * @return StepperCmd if in normal mode - otherwise probably calibrating
 */
StepperState Stepper28BYJ::getState(){
	return xState;
}

/***
 * Stop the current step action.
 * If another action in queue it will then start that
 */
void Stepper28BYJ::stepStop(){
	xState = StepperStop;
}

/***
 * Rotate continuously at RPM rate
 * @param rpm - Revolutions per minute
 * @param cw - True if clockwise
 * @return
 */
uint32_t Stepper28BYJ::stepContinuous(float rpm, bool cw){
	StepperReqT req;

	req.action = StepperCont;
	req.steps = 0;
	req.rpm = rpm;
	req.cw = cw;
	req.id = xNextId++;

	if (rpm > MAX_RPM){
		rpm = MAX_RPM;
	}

	if (xCmdQ != NULL){
		BaseType_t res = xQueueSendToBack(xCmdQ, (void *)&req, 0);
		if (res != pdTRUE){
			printf("WARNING: Queue is full\n");
		}
	}
	return req.id;
}


/***
 * Stop the stepper motor
 * Clear the queue of actions
 */
void Stepper28BYJ::stepHaltClear(){
	stepStop();
	xQueueReset(xCmdQ);
}

/***
 * Stop the stepper,
 * Clear the queue of actions
 * Return to zero possition
 * @return
 */
uint32_t Stepper28BYJ::stepReset(){
	stepHaltClear();
	bool cw = true;

	if (xPos < (xTotalSteps - xPos)){
		cw = false;
	}
	return stepToDeg(0.0, 3, cw);
}

/***
 * Get the current angle of the stepper motor
 * @return Degrees from Hall sensor
 */
float Stepper28BYJ::getDegrees(){
	return 360.0 * ((float)xPos / (float)xTotalSteps);
}

/***
 * Return radians
 * 0 is Magent
 * @return -PI to PI
 */
double Stepper28BYJ::getRadians(){
	double rad, ratio;
	int half = xTotalSteps /2;
	if (getPos() == 0){
		rad = 0.0;
	} else {
		if (getPos() <= half){
			ratio = ((double) getPos() / (double) half);
			rad = ratio * M_PI;
		} else {
			int inv = half - (getPos() - half);
			ratio = ((double) inv / (double) half);
			rad = ratio * M_PI * (-1.0);
		}
	}
	return rad * (-1.0);
}


/***
 * Get Step possition. Hall sensor is possition 0
 * Counts clockwise
 * @return step possition
 */
uint16_t Stepper28BYJ::getPos(){
	return xPos;
}


/***
 * Setup the Ros Msg Structure;
 */
void Stepper28BYJ::setupRosMsgs(){
	//Setup the joint state msg
	sensor_msgs__msg__JointState__init(&xJointStateMsg);
	rosidl_runtime_c__double__Sequence__init(&xJointStateMsg.position, 1);
	xJointStateMsg.position.data[0] = getRadians();
	xJointStateMsg.position.size = 1;
	xJointStateMsg.position.capacity = 1;
	rosidl_runtime_c__String__Sequence__init(&xJointStateMsg.name, 1);
	if (!rosidl_runtime_c__String__assign(&xJointStateMsg.name.data[0], pJointName)){
		printf("ERROR: Joined assignment failed\n");
	}
	xJointStateMsg.name.size=1;
	xJointStateMsg.name.capacity=1;

	//Setup sub msgs
	geometry_msgs__msg__Twist__init(&xTwistMsg);
	geometry_msgs__msg__PoseStamped__init(&xPoseMsg);
}

/***
 * Pub Position to ROS
 */
void Stepper28BYJ::pubRosPos(){
	//Populate the Joint possition message
	int64_t time = rmw_uros_epoch_nanos();

	xJointStateMsg.header.stamp.sec = time / 1000000000;
	xJointStateMsg.header.stamp.nanosec = time % 1000000000;
	double a = getRadians();
	xJointStateMsg.position.data[0] = a;
	if (!uRosBridge::getInstance()->publish(&xPubJoint,
			&xJointStateMsg,
			this,
			NULL)){
		//printf("Joint Pub failed\n");
	}
}

/***
 * Provide a count of the number of entities
 * @return number of entities >=0
 */
uint Stepper28BYJ::getCount(){
	return xCount;
}

/***
 * Create the publishing entities
 * @param node
 * @param support
 */
void Stepper28BYJ::createEntities(rcl_node_t *node, rclc_support_t *support){
	rclc_publisher_init_default(
			&xPubJoint,
			node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
			"joint_states");

	rclc_subscription_init_default(
		  &xSubTwist,
		  node,
		  ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
		  "cmd_vel");

	rclc_subscription_init_default(
		  &xSubPose,
		  node,
		  ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped),
		  "do_pose");

	xCount = 3;
}

/***
 * Destroy the publishing entities
 * @param node
 * @param support
 */
void Stepper28BYJ::destroyEntities(rcl_node_t *node, rclc_support_t *support){
	xCount = 0;
	rcl_publisher_fini(&xPubJoint, 		node);
	rcl_subscription_fini(&xSubTwist, 	node);
	rcl_subscription_fini(&xSubPose, 	node);
}

/***
 * Return the number of handles needed by the executor
 * @return
 */
uint Stepper28BYJ::getHandles(){
	return 2;
}


/***
 * Add subscribers, guards and timers to the executor
 * @param executor
 */
void Stepper28BYJ::addToExecutor(rclc_executor_t *executor){
	buildContext(&xSubTwistContext, NULL);
	buildContext(&xSubPoseContext, NULL);

	rclc_executor_add_subscription_with_context(
		executor,
		&xSubTwist,
		&xTwistMsg,
		uRosEntities::subscriptionCallback,
		&xSubTwistContext,
		ON_NEW_DATA);

	rclc_executor_add_subscription_with_context(
		executor,
		&xSubPose,
		&xPoseMsg,
		uRosEntities::subscriptionCallback,
		&xSubPoseContext,
		ON_NEW_DATA);
}

/***
 * Handle subscription msg
 * @param msg
 * @param localContext
 */
void Stepper28BYJ::handleSubscriptionMsg(const void* msg, uRosSubContext_t* context){

	if (context == &xSubTwistContext){
		geometry_msgs__msg__Twist * pTwistMsg = (geometry_msgs__msg__Twist *) msg;

		double radpersec = pTwistMsg->angular.z;

		double rpm = radpersec / (M_PI * 2) * 60.0;

		printf("Twist %f becomes %f rpm\n", radpersec, rpm);

		stepStop();
		if (radpersec != 0.0){
			if (radpersec > 0){
				stepContinuous(rpm, true);
			} else {
				stepContinuous(rpm * (-1.0), false);
			}
		}
	}

	if (context == &xSubPoseContext){
		int targetPos;
		geometry_msgs__msg__PoseStamped *pPoseMsg;
		pPoseMsg = (geometry_msgs__msg__PoseStamped *) msg;

		//Calculate target angle
		geometry_msgs__msg__Quaternion *q = &pPoseMsg->pose.orientation;
		double siny_cosp = 2 * (q->w * q->z + q->x * q->y);
		double cosy_cosp = 1 - 2 * (q->y * q->y + q->z * q->z);
		double a = std::atan2(siny_cosp, cosy_cosp);

		//Calculate target pos
		double half = xTotalSteps /2.0;
		targetPos = 0;
		double ratio = a/M_PI;
		if (ratio < 0.0){
			ratio = ratio * (-1);
		}
		if (a > 0.0) {
			targetPos = ratio * half;
		} else if (a < 0.0) {
			targetPos = xTotalSteps - (ratio * half);
		}
		if (targetPos > xTotalSteps){
			targetPos = xTotalSteps;
		}

		//Calc direction
		uint steps = 0;
		uint current = getPos();
		bool cw = true;
		if (targetPos > current){
			steps = targetPos - current;
		} else {
			steps = (targetPos + xTotalSteps) - current;
		}
		if (steps > half){
			steps = half - (steps - half);
			cw = false;
		}

		//Calc difference between timestamp and now
		int64_t now = rmw_uros_epoch_nanos();
		uint32_t sec = now / 1000000000;
		uint32_t nanosec = now % 1000000000;
		double dif = pPoseMsg->header.stamp.sec - sec;
		double ndif = pPoseMsg->header.stamp.nanosec % 1000000000;
		ndif = (ndif - nanosec) / 1000.0 / 1000.0 /1000.0;
		//If timestamp in past the 0 time
		if (dif < 0.0){
			dif = 0.0;
		}
		if ((dif == 0.0) && (ndif < 0.0)){
			dif = 0.0;
		} else {
			dif = dif + ndif;
		}

		//Calculate speed
		double rpm, stepspersec;
		if (dif > 0.0){
			stepspersec = (double)steps / dif;
			rpm = (stepspersec / xTotalSteps) * 60.0;
		} else {
			rpm = MAX_RPM;
		}

		//printf("Time %f steps per sec %f Target speed %f\n", dif, stepspersec, rpm);


		stepTo(targetPos, rpm, cw);

	}
}


/***
 * Minimum radians between two values.
 * In CW or CCW direction
 * @param a1 - radians -PI to +PI
 * @param a2 - radians -PI to +PI
 * @return Negative if CCW
 */
double Stepper28BYJ::angleDiff(double a1, double a2){
	double cenA1 = a1 + M_PI;
	double cenA2 = a2 + M_PI;

	double cw = cenA1 - cenA2;
	if (cw < 0.0){
		cw = cw * -1.0;
	}
	return cw;
}
