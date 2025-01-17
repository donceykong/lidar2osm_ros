#ifndef ROBOT_HPP
#define ROBOT_HPP

struct Robot {
    double r = 0.0; 
    double g = 0.0; 
    double b = 0.0;

    void setRobot(int robotNumber) {
        // Init robot of interest
        if (robotNumber == 1) {         // Green
            r = 87.0/255.0; 
            g = 227.0/255.0; 
            b = 137.0/255.0;
        }
        else if (robotNumber == 2) {    // Purple
            r = 192.0/255.0; 
            g = 97.0/255.0; 
            b = 203.0/255.0;
        }
        else if (robotNumber == 3) {    // Orange
            r = 255.0/255.0; 
            g = 163.0/255.0; 
            b = 72.0/255.0;
        }
        else if (robotNumber == 4) {    // Blue
            r = 98.0/255.0; 
            g = 160.0/255.0; 
            b = 234.0/255.0;
        }
    }
};

#endif  // ROBOT_HPP