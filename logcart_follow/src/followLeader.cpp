#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <logcart_follow/LeaderTrack.h>
#include <logcart_driver/Joystick.h>
#include <list>
#include <cmath>
#include <string>
#include <vector>

class LeaderFollower
{
public:
    struct PIControllerParams
    {
        double errorSum;
        double kp;
        double ki;
        double kdpositive;
        double kdnegative;
        double velocity;
        double piNegFactor;
    };

    struct PlanData
    {
        bool valid;
        double goalDistancePenalty;
        double obstacleProximityPenalty;
        double chordAngle;
        double radius;
        double arcDistance;
        double velocity;
        double yawrate;
        int numberOfSteps;
        double penalty;
    };

    class GridPoint
    {
    public:
        GridPoint(int x, int y) : x(x), y(y)
        {
        }
        int x, y;
    };

    class MapPoint
    {
    public:
        MapPoint(double x, double y) : x(x), y(y)
        {
        }
        double x, y;
    };

    struct Rect
    {
        Rect()
        {
        }
        Rect(double x1, double x2, double y1, double y2) : x1(x1), x2(x2), y1(y1), y2(y2)
        {
        }
        double x1, x2, y1, y2;
    };

    ros::Publisher cmdPub, gridPub, trajPub, obsPub;
    ros::NodeHandle n;
    nav_msgs::MapMetaData mapData;
    nav_msgs::OccupancyGrid og;
    std_msgs::String traj;
    std_msgs::String obsLoc;
    double costMapGradient;
    ros::Time prevRun, prevOdom;
    double stopGap;
    double regionGap0;
    double regionGap1;
    double regionGap2;
    double regionGap3;
    double regionGap4;
    double speedAtGap0;
    double speedAtGap1;
    double speedAtGap2;
    double speedAtGap3;
    double largerPathCompensation, smallerPathCompensation;
    double yawrateOscillationThreshold1, yawrateOscillationThreshold2, yawrateOscillationCoefficient1,
        yawrateOscillationCoefficient2;
    double previousYawrate;
    double previousAngle;
    double minSpeedDifference;
    double maxLinearVelocity;
    double maxAngularVelocity;
    double goalDistancePenaltyFactor;
    double obstacleProximityPenaltyFactor;
    double trajectoryPlannerAngularResolution;
    double minFollowerTurningRadius;
    double pathArcStepSize;
    double maxPathArcDistance;
    double prevDist;
    double goalPenaltyShape;
    int numberOfTrajectoryPlans;
    PIControllerParams piContParams;
    int estopCount, estopCountThreshold;
    logcart_follow::LeaderTrack lastTrack;
    double lidarToTurningAxis, centerAxisToSideBoundary;
    double mapWidth, mapHeight, YOffset, XOffset;
    Rect gridBoundaries;
    GridPoint originGrid = GridPoint(0, 0);
    bool trackingEnabled;

    bool rectPtCollision(Rect p, double x, double y)
    {
        if (x < p.x2 && x > p.x1 && y < p.y2 && y > p.y1)
            return true;
        else
            return false;
    }

    bool rectCollision(Rect r1, Rect r2)
    {
        if (r1.x1 < r2.x2 && r1.x2 > r2.x1 && r1.y1 < r2.y2 && r1.y2 > r2.y1)
            return true;
        else
            return false;
    }

    void clearMap()
    {
        for (int i = 0; i < mapData.width * mapData.height; i++)
        {
            og.data[i] = 0;
        }
    }
    void initMap()
    {
        og.data.clear();
        for (int i = 0; i < mapData.width * mapData.height; i++)
        {
            og.data.push_back(0);
        }
    }

    GridPoint mapToGrid(MapPoint mPt)
    {
        return GridPoint((mPt.x + XOffset) / mapData.resolution, (mPt.y + YOffset) / mapData.resolution);
    }

    MapPoint GridToMap(GridPoint gPt)
    {
        return MapPoint(gPt.x * mapData.resolution - XOffset, gPt.y * mapData.resolution - YOffset);
    }

    int getI(int x, int y)
    {
        return mapData.width * x + y;
    }

    int8_t getGridData(GridPoint gPt)
    {
        if (pointInGridBoundary(gPt))
            return og.data[getI(gPt.x, gPt.y)];
        else
            return 0;
        // return 127;
    }

    int8_t getGridData(int x, int y)
    {
        return getGridData(GridPoint(x, y));
    }

    int8_t getGridData(double x, double y)
    {
        GridPoint gPt = mapToGrid(MapPoint(x, y));
        return getGridData(gPt);
    }

    void setGridData(GridPoint gPt, int8_t val)
    {
        if (pointInGridBoundary(gPt))
            og.data[getI(gPt.x, gPt.y)] = val;
    }

    void setGridData(int x, int y, int8_t val)
    {
        setGridData(GridPoint(x, y), val);
    }

    void setGridData(double x, double y, int8_t val)
    {
        GridPoint gPt = mapToGrid(MapPoint(x, y));
        setGridData(gPt, val);
    }

    bool pointInGridBoundary(GridPoint gPt)
    {
        if (gPt.y < mapData.width && gPt.y >= 0 && gPt.x < mapData.height && gPt.x >= 0)
            return true;
        else
            return false;
    }

    bool pointInGridBoundary(double x, double y)
    {
        GridPoint gPt = mapToGrid(MapPoint(x, y));
        return pointInGridBoundary(gPt);
    }

    LeaderFollower(ros::NodeHandle &n)
        : n(n), mapWidth(4.0), mapHeight(4.0), YOffset(2.0), XOffset(1.5), prevDist(0.0), estopCount(0), trackingEnabled(false)
    {
        std::string cmd_velTopic;
        ros::param::get("~cmd_velTopic", cmd_velTopic);
        cmdPub = this->n.advertise<geometry_msgs::Twist>(cmd_velTopic, 1000);
        gridPub = this->n.advertise<nav_msgs::OccupancyGrid>("grid", 1000);
        trajPub = this->n.advertise<std_msgs::String>("trajectory", 10);
        obsPub = this->n.advertise<std_msgs::String>("obstacle_loc", 100);

        mapData.resolution = 0.05;
        mapData.width = mapWidth / mapData.resolution;
        mapData.height = mapHeight / mapData.resolution;
        og.info = mapData;

        gridBoundaries = Rect(-XOffset, mapData.resolution * mapData.height - XOffset, -YOffset,
                              mapData.resolution * mapData.width - YOffset);

        initMap();
        clearMap();

        prevRun = ros::Time::now();
        prevOdom = ros::Time::now();
        originGrid = mapToGrid(MapPoint(0, 0));
        lastTrack.centroid.x = 0;
        lastTrack.centroid.y = 0;
        lastTrack.leaderTrackStatus = lastTrack.LEADERTRACKSTATUS_LEADER_NONE;

        piContParams.errorSum = 0;
        piContParams.velocity = 0;
        ros::param::get("~kdpositive", piContParams.kdpositive);
        ros::param::get("~kdnegative", piContParams.kdnegative);
        ros::param::get("~costMapGradient", costMapGradient);
        ros::param::get("~minSpeedDifference", minSpeedDifference);
        ros::param::get("~largerPathCompensation", largerPathCompensation);
        ros::param::get("~smallerPathCompensation", smallerPathCompensation);
        ros::param::get("~max_linear_velocity", maxLinearVelocity);
        ros::param::get("~yawrateOscillationThreshold1", yawrateOscillationThreshold1);
        ros::param::get("~yawrateOscillationThreshold2", yawrateOscillationThreshold2);
        ros::param::get("~yawrateOscillationCoefficient1", yawrateOscillationCoefficient1);
        ros::param::get("~yawrateOscillationCoefficient2", yawrateOscillationCoefficient2);
        ros::param::get("~max_angular_velocity", maxAngularVelocity);
        ros::param::get("~goalDistancePenaltyFactor", goalDistancePenaltyFactor);
        ros::param::get("~obstacleProximityPenaltyFactor", obstacleProximityPenaltyFactor);
        ros::param::get("~stopGap", stopGap);
        ros::param::get("~regionGap0", regionGap0);
        ros::param::get("~regionGap1", regionGap1);
        ros::param::get("~regionGap2", regionGap2);
        ros::param::get("~regionGap3", regionGap3);
        ros::param::get("~regionGap4", regionGap4);
        ros::param::get("~speedAtGap0", speedAtGap0);
        ros::param::get("~speedAtGap1", speedAtGap1);
        ros::param::get("~speedAtGap2", speedAtGap2);
        ros::param::get("~speedAtGap3", speedAtGap3);
        ros::param::get("~trajectoryPlannerAngularResolution", trajectoryPlannerAngularResolution);
        ros::param::get("~numberOfTrajectoryPlans", numberOfTrajectoryPlans);
        ros::param::get("~pathArcStepSize", pathArcStepSize);
        ros::param::get("~maxPathArcDistance", maxPathArcDistance);
        ros::param::get("~goalPenaltyShape", goalPenaltyShape);
        ros::param::get("~estopCountThreshold", estopCountThreshold);
        ros::param::get("~lidarToTurningAxis", lidarToTurningAxis);             // 0.48
        ros::param::get("~centerAxisToSideBoundary", centerAxisToSideBoundary); // 0.35
    }

    void inflatObstacles(int val, int8_t toVal, int shape)
    {
        int x, y, ind;

        for (int i = 0; i < mapData.height; i++)
        {
            for (int j = 0; j < mapData.width; j++)
            {
                if (getGridData(i, j) == val)
                {
                    for (int dy = -1; dy <= 1; dy++)
                    {
                        for (int dx = -1; dx <= 1; dx++)
                        {
                            if (abs(dx - dy) == 1 || shape == 0)
                            { // makes expansion a 45deg square, then full
                                // square, alternately
                                if (getGridData(i + dx, j + dy) == 0)
                                    setGridData(i + dx, j + dy, toVal);
                            }
                        }
                    }
                }
            }
        }
    }

    void deflateObstacles()
    {
        int setValue;
        for (int i = 0; i < mapData.height; i++)
        {
            for (int j = 0; j < mapData.width; j++)
            {
                if (getGridData(i, j) != 100)
                {
                    setValue = getGridData(i, j) - abs(i - 29);
                    if (setValue < 0)
                    {
                        setValue = 0;
                    }
                }
                setGridData(i, j, setValue);
            }
        }
    }

    bool estop()
    {
        int xThresF = 0.2 / mapData.resolution;
        int xThresB = 0.8 / mapData.resolution;
        int yThres = 0.2 / mapData.resolution;

        for (int x = -xThresB; x <= xThresF; x++)
        {
            for (int y = -yThres; y <= yThres; y++)
            {
                if (x >= -1 && x <= 0 && y <= 1 && y >= -1)
                {
                    break;
                }
                if (getGridData(originGrid.x + x, originGrid.y + y) == 100)
                {
                    char buffer[20];
                    snprintf(buffer, 20, "X: %d, Y: %d", x, y);
                    obsLoc.data = buffer;
                    obsPub.publish(obsLoc);
                    return true;
                }
            }
        }
        return false;
    }

    void inflatObsAndPubGrid()
    {
        int shape = 1;
        for (int i = 0; i < 100 - costMapGradient; i += costMapGradient)
        {
            inflatObstacles(100 - i, 100 - i - costMapGradient, shape);
            if (shape == 1)
            {
                shape = 0;
            }
            else
            {
                shape = 1;
            }
        }
        // deflateObstacles();
        og.header.stamp = ros::Time::now();

        gridPub.publish(og);
    }

    double eucDist(double x1, double y1, double x2, double y2)
    {
        return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
    }

    void visualizeTrajectory(double velocity, double yawrate)
    {
        double dt = 0.05, v = velocity, x = -lidarToTurningAxis, y = 0, vth = yawrate, th = 0, dX, dY, dTh;

        int numberOfSteps = 0;
        while (pointInGridBoundary(x, y) && numberOfSteps < 100)
        {
            dX = v * cos(th) * dt;
            dY = v * sin(th) * dt;
            dTh = vth * dt;
            vth *= 0.995;
            x += dX;
            y += dY;
            th += dTh;
            numberOfSteps++;
            setGridData(x, y, 20);
        }
    }

    PlanData getScore(double chordAngle, double goalX, double goalY)
    {
        double dt = 0.05, v = 0.0, x = -lidarToTurningAxis, y = 0, vth = chordAngle, th = 0, dX, dY, dTh, distScore;
        double flx, frx, fly, fry, blx, brx, bly, bry, sinth, costh;
        int8_t penaltyFL, penaltyFR, penaltyBL, penaltyBR, penaltyF[10];
        double fX, fY;
        double fXConst, fYConst;
        double minFollowerTurningRadius;
        double pathTurningRadius;
        double pathFullArcLength;
        double pathSimulatedArcLength;
        double leaderDistanceToArcCenter;
        double minleaderDistanceToArc;
        double leaderArcProjectionChordLength;
        double leaderArcProjectionArcLength;
        double leaderArcProjectionX;
        double leaderArcProjectionY;
        double stepPerimeter = 0;
        double stepChord = 0;
        double stepAngle = 0;
        double stepX = 0;
        double stepY = 0;
        PlanData data;
        data.valid = true;
        data.goalDistancePenalty = 0.0;
        data.obstacleProximityPenalty = 0.0;
        data.chordAngle = chordAngle;
        data.radius = 0.0;
        data.arcDistance = 0.0;
        data.velocity = 0.0;
        data.yawrate = 0.0;
        data.numberOfSteps = 0;
        data.penalty = 0.0;

        minFollowerTurningRadius = maxLinearVelocity / maxAngularVelocity; // Path Simulated Chord Length

        if (fabs(chordAngle) < trajectoryPlannerAngularResolution)
        {
            pathTurningRadius = 0.0; // infinity, straight line path
            //pathFullArcLength = maxPathArcDistance;

            chordAngle = 0.0;
            minleaderDistanceToArc = fabs(goalY);
            leaderArcProjectionX = goalX;
            leaderArcProjectionY = 0.0;

            leaderArcProjectionChordLength = goalX;
            leaderArcProjectionArcLength = goalX;
        }
        else
        {
            pathTurningRadius = minFollowerTurningRadius / 2.0 / sin(chordAngle);
            //pathFullArcLength = fabs(3.14 * pathTurningRadius);

            leaderDistanceToArcCenter = sqrt(pow((goalY - pathTurningRadius), 2) + pow(goalX, 2));
            minleaderDistanceToArc = fabs(fabs(pathTurningRadius) - leaderDistanceToArcCenter);
            leaderArcProjectionX = goalX * fabs(pathTurningRadius) / leaderDistanceToArcCenter;
            leaderArcProjectionY =
                (goalY - pathTurningRadius) * fabs(pathTurningRadius) / leaderDistanceToArcCenter + pathTurningRadius;

            leaderArcProjectionChordLength = sqrt(pow(leaderArcProjectionX, 2) + pow(leaderArcProjectionY, 2));
            leaderArcProjectionArcLength =
                2 * fabs(pathTurningRadius) * fabs(asin(leaderArcProjectionChordLength / 2 / fabs(pathTurningRadius)));
        }

        pathFullArcLength = leaderArcProjectionArcLength;

        if (pathFullArcLength > maxPathArcDistance)
        {
            pathSimulatedArcLength = maxPathArcDistance;
        }
        else
        {
            pathSimulatedArcLength = pathFullArcLength;
        }

        data.goalDistancePenalty = pow(minleaderDistanceToArc, goalPenaltyShape);

        data.radius = pathTurningRadius;
        data.arcDistance = leaderArcProjectionArcLength;

        while (pointInGridBoundary(stepX, stepY) && stepPerimeter < pathSimulatedArcLength)
        { // Max number of steps = pi *
            // minFollowerTurningRadius, min = 2*
            // minFollowerTurningRadius
            stepPerimeter = data.numberOfSteps * pathArcStepSize;
            if (pathTurningRadius == 0.0)
            {
                stepAngle = 0.0;
                stepChord = stepPerimeter;
            }
            else
            {
                stepAngle = stepPerimeter / pathTurningRadius / 2;
                stepChord = 2 * pathTurningRadius * sin(stepAngle);
            }
            stepX = stepChord * cos(stepAngle) - lidarToTurningAxis;
            stepY = stepChord * sin(stepAngle);
            data.numberOfSteps += 1;

            costh = cos(stepAngle);
            sinth = sin(stepAngle);

            // 9 points normal to path coincident with lidar X coordinate, 80 cm width
            for (int c = -35, i = 0; c <= 35; c += 10)
            {
                fX = lidarToTurningAxis * costh - (c / 100.0 * sinth) + stepX;
                fY = lidarToTurningAxis * sinth + (c / 100.0 * costh) + stepY;
                penaltyF[i] = getGridData(fX, fY);
                // ROS_INFO("penaltyF%d: %d", c,  getGridData(fX, fY));
                i++;
            }

            // 30 cm to the left, 30 cm behind axis, make space for luggage holder
            blx = -lidarToTurningAxis * costh + centerAxisToSideBoundary * sinth + stepX;
            bly = -lidarToTurningAxis * sinth - centerAxisToSideBoundary * costh + stepY;
            penaltyF[8] = getGridData(blx, bly);
            // ROS_INFO("penaltyBL: %d", penaltyBL);

            // 30 cm to the RIGHT, 30 cm behind axis, make space for luggage holder
            brx = -lidarToTurningAxis * costh - centerAxisToSideBoundary * sinth + stepX;
            bry = -lidarToTurningAxis * sinth + centerAxisToSideBoundary * costh + stepY;
            penaltyF[9] = getGridData(brx, bry);
            // ROS_INFO("penaltyBR: %d", penaltyBR);

            for (int i = 0; i < 10; i++)
            {
                if (penaltyF[i] == 100 && stepPerimeter < 1.35)
                {
                    data.valid = false;
                    return data;
                }
            }

            for (int i = 0; i < 10; i++)
            {
                data.obstacleProximityPenalty +=
                    (int)round(penaltyF[i] * pow(((maxPathArcDistance - stepPerimeter) / maxPathArcDistance), 2));
            }
        }
        return data;
    }

    double distanceToVelocity(double dist, double goalX, double prevDist, ros::Duration runInterval)
    {
        double leaderFollowerSpeedDifference;
        double targetSpeed;
        double dFactorNeg;
        double dFactorPlus;

        leaderFollowerSpeedDifference = (dist - prevDist) / runInterval.toSec();

        if (leaderFollowerSpeedDifference < minSpeedDifference)
        {
            dFactorNeg = leaderFollowerSpeedDifference * piContParams.kdnegative;
            dFactorPlus = 0.0;
            // ROS_INFO("dNegative on, speed diff: %f ms-1",
            // leaderFollowerSpeedDifference);
        }
        else if (leaderFollowerSpeedDifference >= minSpeedDifference && leaderFollowerSpeedDifference < 0.0)
        {
            dFactorNeg = 0.0;
            dFactorPlus = 0.0;
        }
        else if (leaderFollowerSpeedDifference >= 0.0)
        {
            dFactorNeg = 0.0;
            dFactorPlus = leaderFollowerSpeedDifference * piContParams.kdpositive;
        }

        if (dist < stopGap)
        {
            targetSpeed = 0.0 + dFactorNeg * 0.0 + dFactorPlus * 0.0;
        }
        else if (dist >= stopGap && dist < regionGap0)
        {
            targetSpeed =
                0.0 + (speedAtGap0) * (dist - stopGap) / (regionGap0 - stopGap) + dFactorNeg * 1.0 + dFactorPlus * 0.0;
        }
        else if (dist >= regionGap0 && dist < regionGap1)
        {
            targetSpeed = speedAtGap0 + (speedAtGap1 - speedAtGap0) * (dist - regionGap0) / (regionGap1 - regionGap0) +
                          dFactorNeg * 1.0 + dFactorPlus * 0.0;
        }
        else if (dist >= regionGap1 && dist < regionGap2)
        {
            targetSpeed = speedAtGap1 + (speedAtGap2 - speedAtGap1) * (dist - regionGap1) / (regionGap2 - regionGap1) +
                          dFactorNeg * 1.0 + dFactorPlus * 0.0; // Constant speed region
        }
        else if (dist >= regionGap2 && dist < regionGap3)
        {
            targetSpeed = speedAtGap2 + (speedAtGap3 - speedAtGap2) * (dist - regionGap2) / (regionGap3 - regionGap2) +
                          dFactorNeg * 1.0 + dFactorPlus * 0.0;
        }
        else if (dist >= regionGap3)
        {
            targetSpeed = speedAtGap3 + (maxLinearVelocity - speedAtGap3) * (dist - regionGap3) / (regionGap4 - regionGap3) +
                          dFactorNeg * 1.0 + dFactorPlus * 0.0; // catchup
        }

        targetSpeed = (targetSpeed > maxLinearVelocity) ? maxLinearVelocity : targetSpeed;
        if (targetSpeed < 0.0)
        {
            targetSpeed = 0.0;
        }

        return targetSpeed;
    }

    geometry_msgs::Twist determineTrajectory(double goalX, double goalY)
    {
        double pathChordAngle; // Where chord length = 2*minFollowerTurningRadius
        double dZ, dX, penalty, goalImportance;
        double error;
        double dist;
        double targetSpd;
        double dFactor;
        double leaderArcDistance;
        double leaderRadius;
        double leaderAngle;
        double goalXFromAxis;
        double pathTurningRadius;
        double pathArcLengthToLeader;

        geometry_msgs::Twist cmd;
        std::list<PlanData> plans;
        PlanData maxPenalties;
        PlanData planPenalty;
        PlanData selectedPlan;
        plans.clear();
        maxPenalties.obstacleProximityPenalty = 1;
        maxPenalties.goalDistancePenalty = 1;
        maxPenalties.numberOfSteps = 0;
        selectedPlan.penalty = 99999;

        ros::Duration runInterval = ros::Time::now() - prevRun;
        prevRun = ros::Time::now();

        goalXFromAxis = goalX + lidarToTurningAxis; // Distance between LIDAR and axis

        leaderAngle = atan2(goalY, goalXFromAxis);

        if (fabs(leaderAngle) < trajectoryPlannerAngularResolution)
        {
            leaderRadius = 0.0; // infinity, straight line path
            leaderArcDistance = sqrt(pow(goalXFromAxis, 2) + pow(goalY, 2));
        }
        else
        {
            leaderRadius = sqrt(pow(goalXFromAxis, 2) + pow(goalY, 2)) / (2 * sin(leaderAngle));
            leaderArcDistance = leaderAngle * leaderRadius * 2.0;
        }

        if (leaderAngle > 1.50 - trajectoryPlannerAngularResolution * numberOfTrajectoryPlans / 2)
        {
            leaderAngle = 1.50 - trajectoryPlannerAngularResolution * numberOfTrajectoryPlans / 2;
        }
        else if (leaderAngle < -1.50 + trajectoryPlannerAngularResolution * numberOfTrajectoryPlans / 2)
        {
            leaderAngle = -1.50 + trajectoryPlannerAngularResolution * numberOfTrajectoryPlans / 2;
        }

        //        dX = (dX > 0.2) ? dX : 0.2;
        for (int i = -(numberOfTrajectoryPlans / 2); i <= (numberOfTrajectoryPlans / 2); i++)
        {
            pathChordAngle = trajectoryPlannerAngularResolution * i + leaderAngle;

            planPenalty = getScore(pathChordAngle, goalXFromAxis, goalY);
            // Plan penalty is a function of only radius

            if (planPenalty.valid)
            {
                if (planPenalty.goalDistancePenalty > maxPenalties.goalDistancePenalty)
                {
                    maxPenalties.goalDistancePenalty = planPenalty.goalDistancePenalty;
                }
                if (planPenalty.obstacleProximityPenalty > maxPenalties.obstacleProximityPenalty)
                {
                    maxPenalties.obstacleProximityPenalty = planPenalty.obstacleProximityPenalty;
                }
                if (planPenalty.numberOfSteps > maxPenalties.numberOfSteps)
                {
                    maxPenalties.numberOfSteps = planPenalty.numberOfSteps;
                }
                plans.push_back(planPenalty);
            }
        }

        for (auto const &plan : plans)
        {
            penalty = goalDistancePenaltyFactor * plan.goalDistancePenalty / maxPenalties.goalDistancePenalty;
            penalty +=
                (obstacleProximityPenaltyFactor * plan.obstacleProximityPenalty / maxPenalties.obstacleProximityPenalty);
            // penalty = penalty * plan.numberOfSteps / maxPenalties.numberOfSteps;
            // //Normalize to path length. MIGHT NOT BE NECESSARY
            if (penalty < selectedPlan.penalty)
            {
                selectedPlan = plan;
                selectedPlan.penalty = penalty;
            }
        }

        pathTurningRadius = selectedPlan.radius;
        pathArcLengthToLeader = (leaderArcDistance - lidarToTurningAxis);

        selectedPlan.velocity = distanceToVelocity(pathArcLengthToLeader, goalX, prevDist, runInterval);

        prevDist = pathArcLengthToLeader;

        if (fabs(pathTurningRadius) < 0.001)
        {
            selectedPlan.yawrate = 0.0;
        }
        else
        {
            selectedPlan.yawrate = selectedPlan.velocity / pathTurningRadius;
        }

        if (fabs(selectedPlan.yawrate) <= yawrateOscillationThreshold1)
        {
            selectedPlan.yawrate = yawrateOscillationCoefficient1 * selectedPlan.yawrate;
        }
        else if (fabs(selectedPlan.yawrate) > yawrateOscillationThreshold1 &&
                 fabs(selectedPlan.yawrate) <= yawrateOscillationThreshold2)
        {
            selectedPlan.yawrate = yawrateOscillationCoefficient2 * selectedPlan.yawrate;
        }

        previousYawrate = selectedPlan.yawrate;
        previousAngle = leaderAngle;

        if (fabs(selectedPlan.chordAngle) - fabs(leaderAngle) < 0)
        {
            selectedPlan.velocity =
                selectedPlan.velocity *
                (1 - (fabs(selectedPlan.chordAngle) - fabs(leaderAngle)) * (largerPathCompensation / 0.6));
        }
        else
        {
            selectedPlan.velocity =
                selectedPlan.velocity *
                (1 - (fabs(selectedPlan.chordAngle) - fabs(leaderAngle)) * (smallerPathCompensation / 0.6));
        }

        if (estop())
        {
            estopCount++;
            if (estopCount > estopCountThreshold)
            {
                estopCount = 0;
                traj.data = "estop obstacle";
                estopBehavior(cmd);
            }
        }

        else if (pathArcLengthToLeader < stopGap && pathArcLengthToLeader >= 0.25)
        {
            traj.data = "reached";
            leaderReachedBehavior(cmd);
        }
        else if (selectedPlan.penalty < 99998)
        {
            traj.data = "foundtrajectory " + std::to_string(goalX);
            foundTrajectoryBehavior(cmd, selectedPlan.velocity, selectedPlan.yawrate);
        }
        else
        {
            traj.data = "notrajectory";
            noTrajectoryFoundBehavior(cmd);
        }
        trajPub.publish(traj);
        return cmd;
    }

    void estopBehavior(geometry_msgs::Twist &cmd)
    {
        cmd.linear.x = 0;
        cmd.angular.z = 0;
    }

    void lostLeaderBehavior(geometry_msgs::Twist &cmd)
    {
        cmd.linear.x = 0;
        cmd.angular.z = 0;
    }

    void leaderReachedBehavior(geometry_msgs::Twist &cmd)
    {
        cmd.linear.x = 0;
        cmd.angular.z = 0;
    }

    void foundTrajectoryBehavior(geometry_msgs::Twist &cmd, double X, double Z)
    {
        cmd.linear.x = X;
        cmd.angular.z = Z;
    }

    void noTrajectoryFoundBehavior(geometry_msgs::Twist &cmd)
    {
        cmd.linear.x = 0;
        cmd.angular.z = 0;
    }

    void registerPointToGrid(float range, float angle)
    {
        if (!(std::isnan(range) || std::isinf(range) || range < 0.03))
        {
            float x = range * cos(angle);
            float y = range * sin(angle);
            setGridData(x, y, 100);
        }
    }

    void leaderTrackCallback(const logcart_follow::LeaderTrack::ConstPtr &msg)
    {
        float x, y;
        geometry_msgs::Twist cmd_vel;
        if (msg->leaderTrackStatus != msg->LEADERTRACKSTATUS_LEADER_NONE && msg->leaderTrackStatus != msg->LEADERTRACKSTATUS_LEADER_LOST)
        {
            clearMap();
            float currAngle = msg->obstacles.angle_min;
            for (const auto &point : msg->obstacles.ranges)
            {
                registerPointToGrid(point, currAngle);
                currAngle += msg->obstacles.angle_increment;
            }
            inflatObsAndPubGrid();
            cmd_vel = determineTrajectory(msg->centroid.x, msg->centroid.y);
        }
        else
        {
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
            traj.data = "noleader";
            trajPub.publish(traj);
        }
        if (trackingEnabled)
        {
            cmdPub.publish(cmd_vel);
        }
    }
    // void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    // {
    //     ros::Duration runInterval = ros::Time::now() - prevOdom;
    //     double deltaT = runInterval.toSec();
    //     prevOdom = ros::Time::now();

    //     lastTrack.centroid.x -= (msg->twist.twist.linear.x * deltaT) * (cos(msg->twist.twist.angular.z * deltaT));
    //     lastTrack.centroid.y -= (msg->twist.twist.linear.x * deltaT) * (sin(msg->twist.twist.angular.z * deltaT));
    // }

    void enableTrackCallback(const logcart_driver::Joystick::ConstPtr &msg)
    {
        if (msg->enableTracking == 1)
        {
            trackingEnabled = true;
        }
        else
        {
            trackingEnabled = false;
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "follow_leader");
    ros::NodeHandle n;

    LeaderFollower follower(n);
    std::string odomTopic;
    ros::param::get("~odomTopic", odomTopic);
    ros::Subscriber leaderTrackSub = n.subscribe("leadertrack", 1, &LeaderFollower::leaderTrackCallback, &follower);
    // ros::Subscriber odomSub = n.subscribe(odomTopic, 1, &LeaderFollower::odomCallback, &follower);
    ros::Subscriber enableTrackingSub = n.subscribe("joy_buttons", 1, &LeaderFollower::enableTrackCallback, &follower);

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    return 0;
}
