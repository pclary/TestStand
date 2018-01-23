#ifndef COMMON_STRUCTS_H_
#define COMMON_STRUCTS_H_

#include <Eigen/Dense>

typedef enum {
	SSA = 0,
	Double = 1,
	SSB = 2,
	MS = 3,
	SS_Left = 4,
	SS_Right = 5
} PHASE_STATE;

typedef struct {
	PHASE_STATE eType;
	double T;
} PHASE_Info;


typedef struct {
	double x_m;
	double y_m;
} Point;

typedef struct {
	double x_m;
	double y_m;
	double z_m;
	double a_rad;
} Point3D;

typedef struct {
	Point lb;
	Point ub;
} Patch;


typedef struct {
        Point3D xT;
        double heading;
        double step_time;
        double ds_perc;
        double step_height;
        int num_steps;
        bool bStartLeft;
        bool bNewPlan;
} USER_Params;


typedef Eigen::Matrix<double, 15, 1> ROMPosMatrix;


#endif
