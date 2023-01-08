#pragma once

#include <string>

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include <frc/TimedRobot.h>

#include <string>
#include <frc/Joystick.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "ctre/Phoenix.h"

class AutoArm : public frc::TimedRobot{
    public:
        // this will automatically place a cone on the highest avalible post. 
        void PlaceConeHigh();
        // this will automatically place a cube on the highest shelf
        void PlaceCubeHigh();
        // this will automatically place a cone on the lowest avalible post. 
        void PlaceConeMid();
        // this will automatically place a cube on the lower shelf
        void PlaceCubeMid();
        // this takes driver input to manually move the arm
        void ManualControl(frc::Joystick Joystick);
        // this starts the suction pump
        void StartSuctionPump();
        // this stops the suction pump
        void StopSuctionPump();
        // get functions for the limelight
        double GetLimeX();  
        double GetLimeY();

        // these might never be implimented
        void PickupCone();
        void PickupCube();

        // limelight variables
        std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
        double limeX = 0;
        double limeY = 0;
        double targetArea = 0;
        bool bTargetValid = false;
        // constants
        const double ACCEPTED_ERROR_X = 10;
        const double ACCEPTED_ERROR_Y = 10;
        const double ACCEPTED_ERROR_AREA = 100;
        const double OPTIMAL_TARGET_AREA = 100;
        // arm variables
        double heightSetpointInches = 10;
        double reachSetPointInches = 10;
        double swerveSetpointDegrees = 0;
        double tiltSetpointDegrees = 0;
        double rotationSetpointDegrees = 0;
    private:
        // motor control functions

        // this makes the wrist paralel to the ground, 0 tilt 0 rotation
        void ResetWrist();
        // moves the arm up or down to a setpoint (inches)
        void MoveArmVertical(double hightSetpoint);
        // moves the arm forward or back to a setpoint (inches)
        void MoveArmHorizontal(double reachSetPoint);
        // turns the driveBase to a set angle (degrees)
        void RotateSwerve(double swerveSetpoint);
        // tilt the wrist to a set angle (degrees)
        void TiltWristForwardBackward(double tiltSetpoint);
        // rotate the wrist to a set angle (degrees)
        void RotateWristLeftRight(double rotationSetpoint);
        // turn suction on or off using a solenoid
        void EnableSuction(bool bSuctionOnOff);


        // motors
        //TODO: SET MOTORS TO THE CORRECT TYPE OF MOTOR
        WPI_TalonFX ArmVerticalMotor{41}; // motor for up and down
        WPI_TalonFX ArmHorizontalMotor{42}; // motor for in and out
        WPI_TalonFX VacuumPumpMotor{40}; // motor that creates suction
        WPI_TalonFX WristTiltMotor{43}; // motor for up and down
        WPI_TalonFX WristRotateMotor{44}; // motor for in and out

};