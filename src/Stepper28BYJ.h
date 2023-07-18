/*
 * Stepper28BYJ.h
 *
 * Stepper Motor driven by ULN2003 driver using FreeRTOS
 *
 *  Created on: 24 Dec 2022
 *      Author: jondurrant
 */

#ifndef FREERTOS_28BYJ_48_SRC_STEPPER28BYJ_H_
#define FREERTOS_28BYJ_48_SRC_STEPPER28BYJ_H_

#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "Agent.h"
#include "StepperObserver.h"
#include <math.h>
#include "uRosEntities.h"

extern "C"{
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <sensor_msgs/msg/joint_state.h>
}

#define DELAY 2
#define STEPPER_QUEUE_LEN 10
#define STEPS28BYJ 2048
#define STEPPER_SEQ_LEN 4


enum StepperState {StepperCmd, StepperCalibrateOne, StepperCalibrateTwo, StepperStartHunt, StepperGotoStart, StepperStop};

class Stepper28BYJ : public Agent, public uRosEntities {
public:
	/***
	 * Constructor
	 * @param gp1 - GPIO Pad
	 * @param gp2 - GPIO Pad
	 * @param gp3 - GPIO Pad
	 * @param gp4 - GPIO Pad
	 * @param Hall Sensor - GPIO Pad for slot detector
	 */
	Stepper28BYJ(uint8_t gp1, uint8_t gp2, uint8_t gp3, uint8_t gp4, uint8_t hallGP );
	virtual ~Stepper28BYJ();

	/***
	 * Step motor through x full steps
	 * Possitive is clockwise, negative is counter clockwise
	 * @param step - Step count >0 CW, < 0 CCW
	 * @param rpm - Revolutions per mininute (under 14 to work)
	 * RPM 0 is maximum speed
	 */
	uint32_t step(int16_t step, float rpm);


	/***
	 * Calibrate the disk and leave at possition zero
	 */
	void calibrate();

	/***
	 * Step to a possition
	 * @param pos = Possition between 0 and Maximum Steps
	 * @param rpm - speed, 0 to 14. 0 is max speed
	 * @param cw - clockwise
	 */
	uint32_t stepTo(int16_t pos, float rpm, bool cw);


	/***
	 * Rotate to Degrees possition (0 to 360)
	 * @param deg - Float 0 to 360
	 * @param rpm - Rotation speed, under 14
	 * @param cw - Clockwise or CCW.
	 * @return id of the instruction
	 */
	uint32_t stepToDeg(float deg, float rpm, bool cw);

	/***
	 * Get state of stepper motor
	 * @return StepperCmd if in normal mode - otherwise probably calibrating
	 */
	StepperState getState();

	/***
	 * Get the current angle of the stepper motor
	 * @return Degrees from Hall sensor
	 */
	float getDegrees();

	/***
	 * Return radians
	 * 0 is Magent
	 * @return -PI to PI
	 */
	double getRadians();


	/***
	 * Get Step possition. Hall sensor is possition 0
	 * Counts clockwise
	 * @return step possition
	 */
	uint16_t getPos();

	/***
	 * Stop the current step action.
	 * If another action in queue it will then start that
	 */
	void stepStop();

	/***
	 * Rotate continuously at RPM rate
	 * @param rpm - Revolutions per minute
	 * @param cw - True if clockwise
	 * @return
	 */
	uint32_t stepContinuous(float rpm, bool cw);

	/***
	 * Stop the stepper,
	 * Clear the queue of actions
	 * Return to zero possition
	 * @return
	 */
	uint32_t stepReset();

	/***
	 * Stop the stepper motor
	 * Clear the queue of actions
	 */
	void stepHaltClear();

	/***
	 * Set Observer
	 * @param obs
	 */
	void setObserver(StepperObserver *obs);

	/***
	 * Create the publishing entities
	 * @param node
	 * @param support
	 */
	virtual void createEntities(rcl_node_t *node, rclc_support_t *support);

	/***
	 * Destroy the publishing entities
	 * @param node
	 * @param support
	 */
	virtual void destroyEntities(rcl_node_t *node, rclc_support_t *support);

	/***
	 * Provide a count of the number of entities
	 * @return number of entities >=0
	 */
	virtual uint getCount();

	/***
	 * Return the number of handles needed by the executor
	 * @return
	 */
	virtual uint getHandles();


	/***
	 * Add subscribers, guards and timers to the executor
	 * @param executor
	 */
	virtual void addToExecutor(rclc_executor_t *executor);

protected:

	/***
	 * Run loop for the agent.
	 */
	virtual void run();


	/***
	 * Get the static depth required in words
	 * @return - words
	 */
	virtual configSTACK_DEPTH_TYPE getMaxStackSize();

private:
	/***
	 * Initialise the GPIO
	 */
	void init();

	/***
	 * Process request from the queue
	 */
	uint32_t processQueue();

	/***
	 * Calculate the pos of stepper based on modulus maths
	 * @param pos
	 * @return
	 */
	int16_t modPos(int16_t pos);

	/***
	 * Split RPM speed as a set of four delays over the four phases
	 * @param rpm - 0 will set to maximum speed
	 */
	void setDelay(float rpm);

	/***
	 * Do a single step on the Stepper motor in required direction.
	 */
	void doStep();


	/***
	 * Setup the Ros Msg Structure;
	 */
	void setupRosMsgs();

	/***
	 * Pub Position to ROS
	 */
	void pubRosPos();

	//GPIO Pads for stepper motor coils
	uint8_t pGPPAD[4];

	//Queue of commands
	QueueHandle_t xCmdQ;

	//Target possition in steps
	int16_t xTargetPos = 0;

	//Current possition in steps
	int16_t xPos = 0;

	//Total number of steps in 360, set by calibation
	int16_t xTotalSteps = STEPS28BYJ;

	//Sequence of the stepper coils (0 to 3)
	uint8_t xSeq = 0;

	//Delay to wait betweeen each step of the sequence
	uint16_t xDelay[STEPPER_SEQ_LEN];

	//Current direction
	bool xClockwise = true;

	//Hall sensor GP Pad
	uint8_t xHallGP;

	//Current observer if any
	StepperObserver *pObserver = NULL;

	//Action id for actions in the queue.
	uint32_t xNextId = 1;

	//Current state of the stepper motor. Start in calibration
	StepperState xState = StepperStartHunt;


	rcl_subscription_t xSubTwist;
	uRosSubContext_t   xSubTwistContext;
	rcl_subscription_t xSubPose;
	uRosSubContext_t   xSubPoseContext;

	geometry_msgs__msg__Twist 		xTwistMsg;
	geometry_msgs__msg__PoseStamped xPoseMsg;
	rcl_publisher_t xPubJoint;
	uint xCount = 0;

	sensor_msgs__msg__JointState xJointStateMsg;
	const char *pJointName = "post_to_table";
};

#endif /* FREERTOS_28BYJ_48_SRC_STEPPER28BYJ_H_ */
