#pragma once
#include "RobotArm.h"

#include <frc/TimedRobot.h>

#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include "wpi/span.h"

#include "ctre/Phoenix.h"
#include "frc/Joystick.h"

void AutoArm::MoveArmVertical(double hightSetpoint){
    hightSetpoint *= 100; // TODO: CONVERT INCHES TO ENCODERS
    ArmVerticalMotor.Set(ControlMode::Position, hightSetpoint);
};

void AutoArm::MoveArmHorizontal(double reachSetpoint){
    reachSetpoint *= 100; // TODO: CONVERT INCHES TO ENCODERS
    ArmHorizontalMotor.Set(ControlMode::Position, reachSetpoint);
};

void AutoArm::RotateSwerve(double swerveSetpoint){
    
};

void AutoArm::TiltWristForwardBackward(double tiltSetpoint){};

void AutoArm::RotateWristLeftRight(double rotationSetpoint){

};

void AutoArm::StartSuctionPump(){
    VacuumPumpMotor.Set(0.5); // set the vacuum pump motor to 50%
};

void AutoArm::StopSuctionPump(){
    VacuumPumpMotor.Set(0); // stop the vacuum pump motor
};



// reset
void AutoArm::ResetWrist(){
    this->TiltWristForwardBackward(0);
    this->RotateWristLeftRight(0);
};
// plan for the following function:
// use the limelight to line the robot up with the goal, wait for user input to drop the cone
void AutoArm::PlaceConeHigh(){
    // TODO: configure the limelight pipeline to the "high" vision target mode

    // variable name followed by default value
    limeX = table->GetNumber("tx",0.0); // target x
    limeY = table->GetNumber("ty",0.0); // target y
    targetArea = table->GetNumber("ta",0.0); // target area
    bTargetValid = table->GetBoolean("tv",false); // target valid
    if (bTargetValid == true){
        // the limelight is locked onto a target so:
        // limeX turns the swerve base twards target
        swerveSetpointDegrees -= (limeX) * 0.01; // 0.01 scales speed
        this->RotateSwerve(swerveSetpointDegrees);
        // limeY controls the arm height (PID)
        heightSetpointInches -= (limeY) * 0.01;
        this->MoveArmVertical(heightSetpointInches);
        // targetArea controls the arm extension (PID)
        reachSetPointInches -= (targetArea-OPTIMAL_TARGET_AREA) * 0.01;
        this->MoveArmHorizontal(reachSetPointInches);
        // wrist points cone straight up
        this->ResetWrist();
    }
};

void AutoArm::PlaceCubeHigh(){};

void AutoArm::PlaceConeMid(){};

void AutoArm::PlaceCubeMid(){};

void AutoArm::ManualControl(frc::Joystick Joystick){ // TODO: change this to the correct controller class
    // TODO: move arm based on joystick/controller input
};

