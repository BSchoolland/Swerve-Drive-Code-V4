#pragma once
#include "Utilities/Vector.h"
#include "AngleChooser.h"


// converts an 4 x 10 array giving coordinates, angles, and times into an autonomous setpoint array.
class AutoSetpointList
{
private:
    double aP = 5;
    double mP = 70;
    double aA = 30;
    double mA = 70;
    double p;
    double positionChangeTime;
    double angleChangeTime;
    double changeTime;
    double angleRate;
    double positionRate;
    double difference;
    double angleDifference;
    Vector *lastPosition;
    Vector *nextPosition;
    Vector *position;
    Vector *positionDifference;
    double angles[750];
    Vector *positions[750];
    AngleChooser angleChooser{};

public:
    
    AutoSetpointList(double setpoints[10][4]) {
        lastPosition = new Vector(0, 0, false);
        nextPosition = new Vector(0, 0, false);
        position = new Vector(0, 0, false);
        positionDifference = new Vector(0, 0, false);
        int t = 0;
        for (int i = 0; i < 750; i++) {
          positions[i] = new Vector(0, 0, 1);
        }
        for (int i = 1; i < 10; i++) {  // loops through the setpoint list
            lastPosition->set(setpoints[i-1][0],setpoints[i-1][1], false); 
            nextPosition->set(setpoints[i][0],setpoints[i][1], false);
            positionDifference->set(nextPosition);
            positionDifference->subtractVector(lastPosition);
            difference = positionDifference->getMagnitude();
            angleDifference = angleChooser.getShortestDirection(setpoints[i-1][2], setpoints[i][2]);
            positionChangeTime = mP/2/aP + difference/2/mP;
            angleChangeTime = mA/2/aA + angleDifference/2/mA;
            if (positionChangeTime >= angleChangeTime) {
                angleRate *= angleChangeTime/positionChangeTime;
                positionRate = 1;
                changeTime = positionChangeTime;
            } else {
                positionRate *= positionChangeTime/angleChangeTime;
                angleRate = 1;
                changeTime = angleChangeTime;
            }
          
            for (int j = 0; j < setpoints[i-1][3] * 50; j++) {
                if (t<750) {
                    positions[t] = lastPosition;
                    angles[t] = setpoints[i-1][2];
                    t++;
                }
            }
            for (int k = 0; k < changeTime * 50; k++) {  // 
                if (t<750) {
                    position->set(positionDifference->getAngle(), 1, true);
                    position->scale(getPathValue(k/50.0*positionRate, difference, aP, mP));
                    position->addVector(lastPosition);
                    positions[t]->set(position);
                  
                    if (angleDifference != 0) {
                        angles[t] = setpoints[i-1][2] + angleDifference/std::abs(angleDifference)*getPathValue(k/50.0*angleRate, angleDifference, aA, mA);
                    } else {angles[t] = setpoints[i-1][2];}
                    t++;
                }
            }
        }
    }

    double getPathValue(double t, double d, double a, double m) {
        if (d <= pow(m, 2)/a) {
            if (t <= sqrt(d/a)) {
                p = a*pow(t, 2)/2;
            } else {
                p = -a*pow(t - (2*sqrt(d/a)), 2)/2 + d;
            }
        } else {
            if (t <= m/a) {
                p = a*pow(t, 2)/2;
            } else if (m/a < t && t <= d/m) {
                p = m*t - pow(m, 2)/(2*a);
            } else {
                p = -a*pow(t - m/a - d/m, 2)/2 + d;
            }
        }
        return p;
    }

    Vector *getPosition(int index) {
        return positions[index];
    }

    double getAngle(int index) {
        return angles[index];
    }
};