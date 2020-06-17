#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <logcart_driver/Joystick.h>
#include <logcart_follow/LeaderTrack.h>
#include <logcart_follow/Tagangle.h>
#include <algorithm>
#include <cmath>
#include <list>
#include <string>
#include <vector>

#define PI 3.141592654
#define SQRT2PI 2.506628275
#define DEG2RAD(x) x * 0.017453293
// #define DEFAULT_CAM_ANGLE_SD 0.5
// #define DEFAULT_CAM_RANGE_SD 0.5
// #define DEFAULT_CAM_ARCLEN_SD 0.5
// #define DEFAULT_LIDAR_ANGLE_SD 0.4
// #define DEFAULT_LIDAR_RANGE_SD 0.2
// #define DEFAULT_LIDAR_ARCLEN_SD 0.5
//#define THRESHOLD_FACTOR 0.5
#define PUBLISH_LEADER_TRACK

class LeaderTracker
{
	enum LeaderTrackingState
	{
		LEADER_NONE = 0,
		LEADER_IDENTIFIED,
		LEADER_TRACKING,
		LEADER_LOST
	};

	static float probabilityDensityFunction(float mean, float standardDeviation, float input)
	{
		return exp(-pow((input - mean), 2) / (2 * pow(standardDeviation, 2))) / (standardDeviation / SQRT2PI);
	}

	class Distribution
	{
		float mean;
		float standardDeviation;
		float threshold;

	public:
		Distribution()
		{
		}
		Distribution(float m, float sd, float th)
		{
			mean = m;
			standardDeviation = sd;
			threshold = LeaderTracker::probabilityDensityFunction(m, sd, m) * th;
		}

		float getMean()
		{
			return mean;
		}
		float getSD()
		{
			return standardDeviation;
		}
		float getThreshold()
		{
			return threshold;
		}
	};

	struct SingleScanPoint
	{
		float angle = 0.0;
		float range = 0.0;
		bool isLeader = false;
	};

	LeaderTrackingState leaderTrackState = LEADER_NONE;

	int leader_tag_id;
	Distribution defaultAngle; // (deg)
	Distribution defaultRange;
	Distribution defaultArclen;
	Distribution lidarAngle; // centroid (rad)
	Distribution lidarRange; // centroid
	Distribution lidarArclen;
	double objectPointGapThreshold, globPointGapThreshold;
	int gapSizeThreshold, minimumLeaderSizeForPostProcess;
	logcart_follow::Tagangle lastCameraMsg;
	ros::NodeHandle n;
	ros::Publisher pub, trackStatePub;
#ifdef PUBLISH_LEADER_TRACK
	ros::Publisher ldrTrkPub;
#endif
	bool hasExistingTrack, doEdgeDetection;
	double lidarArcLenSD, lidarRangeSD, defaultArcLenSD, defaultRangeSD, thresholdFactor;
	int maxLeaderSize;
	bool trackingEnabled;

public:
	LeaderTracker(ros::NodeHandle &n) : n(n), hasExistingTrack(false), trackingEnabled(false)
	{
		pub = this->n.advertise<logcart_follow::LeaderTrack>("leadertrack", 1);
		trackStatePub = this->n.advertise<std_msgs::String>("track_status", 1);
#ifdef PUBLISH_LEADER_TRACK
		ldrTrkPub = this->n.advertise<sensor_msgs::LaserScan>("leader_track", 1);
#endif

		ros::param::get("~lidarArcLen_StandardDeviation", lidarArcLenSD);
		ros::param::get("~lidarRange_StandardDeviation", lidarRangeSD);
		ros::param::get("~defaultArcLen_StandardDeviation", defaultArcLenSD);
		ros::param::get("~defaultRange_StandardDeviation", defaultRangeSD);
		ros::param::get("~thresholdFactor", thresholdFactor);
		ros::param::get("~objectPointGapThreshold", objectPointGapThreshold);
		ros::param::get("~globPointGapThreshold", globPointGapThreshold);
		ros::param::get("~gapSizeThreshold", gapSizeThreshold);
		ros::param::get("~minimumLeaderSizeForPostProcess", minimumLeaderSizeForPostProcess);
		ros::param::get("~maxLeaderSize", maxLeaderSize);
		ros::param::get("~doEdgeDetection", doEdgeDetection);
		defaultAngle = Distribution(0.0, 0.0, thresholdFactor);
		defaultRange = Distribution(0.6, defaultRangeSD, thresholdFactor);
		defaultArclen = Distribution(0.0, defaultArcLenSD, thresholdFactor);
	}

	// void camCallback(const logcart_follow::Tagangle::ConstPtr &msg)
	// {
	// 	// Check tag id against known leader
	// 	ros::param::get("leader_tag_id", leader_tag_id);
	// 	if (msg->id == leader_tag_id)
	// 	{
	// 		cameraAngle = Distribution(DEG2RAD(msg->angle), 0.0, thresholdFactor);
	// 		cameraRange = Distribution(msg->approx_distance, camRangeSD, thresholdFactor);
	// 		cameraArclen = Distribution(0, camArcLenSD, thresholdFactor);
	// 		lastCameraMsg = *msg;
	// 		if (leaderTrackState == LEADER_LOST)
	// 		{
	// 			hasExistingTrack = false;
	// 		}
	// 		leaderTrackState = LEADER_TRACKING;
	// 	}
	// }

	bool isFit(Distribution &d, float input)
	{
		return d.getThreshold() < probabilityDensityFunction(d.getMean(), d.getSD(), input);
	}

	std::vector<uint16_t> markOutLiers(const std::vector<float> &ranges)
	{
		std::vector<uint16_t> marks;
		marks.reserve(50);
		for (int i = 1; i < ranges.size() - 1; i++)
		{
			if (!(ranges[i] == INFINITY || ranges[i] == NAN))
			{
				bool badNeighbour1 = abs(ranges[i] - ranges[i - 1]) > 0.1 || ranges[i - 1] == INFINITY || ranges[i - i] == NAN;
				bool badNeighbour2 = abs(ranges[i] - ranges[i + 1]) > 0.1 || ranges[i + 1] == INFINITY || ranges[i + i] == NAN;
				if (badNeighbour1 && badNeighbour2)
				{
					marks.push_back(i);
				}
			}
		}
		if (abs(ranges[0] - ranges[1]) > 0.1)
			marks.push_back(0);
		if (abs(ranges[ranges.size() - 1] - ranges[ranges.size() - 2]) > objectPointGapThreshold)
			marks.push_back(ranges.size() - 1);
		return marks;
	}

	void removeOutliers(sensor_msgs::LaserScan &scan, const std::vector<uint16_t> &marks)
	{
		for (const auto &mark : marks)
			scan.ranges[mark] = INFINITY;
	}

	int edgeFoundCleanBackward(std::vector<SingleScanPoint> &laserScanPoints, int index, int &leaderPointsFound)
	{
		for (int i = 0; i <= gapSizeThreshold; i++)
		{
			laserScanPoints[index + i].isLeader = false;
			leaderPointsFound--;
		}
		for (; index >= 0; index--)
			laserScanPoints[index].isLeader = false;
	}

	int edgeFoundCleanForward(std::vector<SingleScanPoint> &laserScanPoints, int index, int &leaderPointsFound)
	{
		for (int i = 0; i <= gapSizeThreshold; i++)
		{
			laserScanPoints[index - i].isLeader = false;
			leaderPointsFound--;
		}
		for (; index < laserScanPoints.size(); index++)
			laserScanPoints[index].isLeader = false;
	}

	void cleanUpRestOfPoints(std::vector<SingleScanPoint> &laserScanPoints, int backwardIndex, int forwardIndex)
	{
		for (; backwardIndex >= 0; backwardIndex--)
			laserScanPoints[backwardIndex].isLeader = false;
		for (; forwardIndex < laserScanPoints.size(); forwardIndex++)
			laserScanPoints[forwardIndex].isLeader = false;
	}

	void leaderTrackPostProcess(std::vector<SingleScanPoint> &laserScanPoints, const std::vector<int> &leaderIndex)
	{
		if (leaderIndex.size() < minimumLeaderSizeForPostProcess)
			return;

		bool backwardEdgeFound = false, forwardEdgeFound = false;
		int backwardIndex = leaderIndex[(leaderIndex.size() / 2) - 1], backwardGapCount = 0;
		double lastKnowReliableRangeBackward = laserScanPoints[backwardIndex + 1].range;
		int forwardIndex = leaderIndex[(leaderIndex.size() / 2) + 2], forwardGapCount = 0;
		double lastKnowReliableRangeForward = laserScanPoints[forwardIndex - 1].range;
		int leaderPointsFound = 2;
		bool complete = false;

		while (!complete)
		{
			if (!backwardEdgeFound)
			{
				if (std::abs(laserScanPoints[backwardIndex].range - lastKnowReliableRangeBackward) < objectPointGapThreshold &&
					std::isfinite(laserScanPoints[backwardIndex].range))
				{
					lastKnowReliableRangeBackward = laserScanPoints[backwardIndex].range;
					backwardGapCount = 0;
					laserScanPoints[backwardIndex].isLeader = true;
					leaderPointsFound++;
				}
				else if (std::abs(laserScanPoints[backwardIndex].range - lastKnowReliableRangeBackward) < globPointGapThreshold &&
						 std::isfinite(laserScanPoints[backwardIndex].range))
				{
					laserScanPoints[backwardIndex].isLeader = true;
					backwardGapCount++;
					leaderPointsFound++;
				}
				else
				{
					backwardGapCount++;
					laserScanPoints[backwardIndex].isLeader = false;
				}
				if (backwardGapCount > gapSizeThreshold)
				{
					backwardEdgeFound = true;
					edgeFoundCleanBackward(laserScanPoints, backwardIndex, leaderPointsFound);
				}
				if (--backwardIndex < 0)
					backwardEdgeFound = true;
			}

			if (!forwardEdgeFound)
			{
				if (std::abs(laserScanPoints[forwardIndex].range - lastKnowReliableRangeForward) < objectPointGapThreshold &&
					std::isfinite(laserScanPoints[forwardIndex].range))
				{
					lastKnowReliableRangeForward = laserScanPoints[forwardIndex].range;
					forwardGapCount = 0;
					laserScanPoints[forwardIndex].isLeader = true;
					leaderPointsFound++;
				}
				else if (std::abs(laserScanPoints[forwardIndex].range - lastKnowReliableRangeForward) < globPointGapThreshold &&
						 std::isfinite(laserScanPoints[forwardIndex].range))
				{
					laserScanPoints[forwardIndex].isLeader = true;
					forwardGapCount++;
					leaderPointsFound++;
				}
				else
				{
					forwardGapCount++;
					laserScanPoints[forwardIndex].isLeader = false;
				}

				if (forwardGapCount > gapSizeThreshold)
				{
					forwardEdgeFound = true;
					edgeFoundCleanForward(laserScanPoints, forwardIndex, leaderPointsFound);
				}
				if (++forwardIndex >= laserScanPoints.size())
					forwardEdgeFound = true;
			}
			if (leaderPointsFound > maxLeaderSize)
			{
				complete = true;
				cleanUpRestOfPoints(laserScanPoints, backwardIndex, forwardIndex);
			}
			if (backwardEdgeFound && forwardEdgeFound)
			{
				complete = true;
			}
		}
	}

	void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
	{
		if (trackingEnabled)
		{
			std::vector<SingleScanPoint> laserScanPoints;
			std::vector<int> leaderIndex;
			ros::Duration interval(0.2);
			sensor_msgs::LaserScan laserMsg = *msg;
			laserMsg.header.frame_id = "laser";
			// removeOutliers(laserMsg, markOutLiers(laserMsg.ranges));
			// std::reverse(msgReversed.ranges.begin(), msgReversed.ranges.end());

			/* Message to be published */
			logcart_follow::LeaderTrack filteredScanMsg;
#ifdef PUBLISH_LEADER_TRACK
			sensor_msgs::LaserScan leaderTrk;
			leaderTrk = laserMsg;
			leaderTrk.ranges = std::vector<float>(laserMsg.ranges.size(), INFINITY);
#endif

			laserScanPoints.clear();
			laserScanPoints.reserve(laserMsg.ranges.size());
			if (leaderTrackState == LEADER_NONE && !hasExistingTrack)
			{
				filteredScanMsg.obstacles = laserMsg;
				SingleScanPoint currPoint;
				currPoint.angle = laserMsg.angle_min;
				bool isLeaderFound = false;
				for (const auto &point : laserMsg.ranges)
				{
					currPoint.range = point;

					if (isFit(defaultArclen, currPoint.range * (defaultAngle.getMean() - currPoint.angle)) && isFit(defaultRange, currPoint.range))
					{
						currPoint.isLeader = true;
						isLeaderFound = true;
						leaderIndex.push_back(laserScanPoints.size());
					}
					else
						currPoint.isLeader = false;
					/* Push SingleScanPoint structure into vector */
					laserScanPoints.push_back(currPoint);

					/* Angle */
					currPoint.angle += laserMsg.angle_increment;
				}
				if (doEdgeDetection)
					leaderTrackPostProcess(laserScanPoints, leaderIndex);

				float rangeAcc = 0, angleAcc = 0;
				uint8_t numLeaderPoints = 0;
				/* Remove leader points from published message */
				for (int i = 0; i < laserScanPoints.size(); i++)
				{
					if (laserScanPoints[i].isLeader == true)
					{
						rangeAcc += laserScanPoints[i].range;
						angleAcc += laserScanPoints[i].angle;
						numLeaderPoints++;
						filteredScanMsg.obstacles.ranges[i] = INFINITY;
#ifdef PUBLISH_LEADER_TRACK
						leaderTrk.ranges[i] = laserScanPoints[i].range;
#endif
					}
				}
				if (!isLeaderFound)
				{
					leaderTrackState = LEADER_NONE;
					filteredScanMsg.leaderTrackStatus = filteredScanMsg.LEADERTRACKSTATUS_LEADER_NONE;
					hasExistingTrack = false;
				}
				else
				{
					float avgRange = rangeAcc / numLeaderPoints;
					float avgAngle = angleAcc / numLeaderPoints;
					filteredScanMsg.centroid.x = avgRange * cos(avgAngle);
					filteredScanMsg.centroid.y = avgRange * sin(avgAngle);
					filteredScanMsg.leaderTrackStatus = filteredScanMsg.LEADERTRACKSTATUS_LEADER_TRACKING;

					lidarAngle = Distribution(avgAngle, 0.0, thresholdFactor);
					lidarRange = Distribution(avgRange, lidarRangeSD, thresholdFactor);
					lidarArclen = Distribution(0, lidarArcLenSD, thresholdFactor);
					hasExistingTrack = true;
					leaderTrackState = LEADER_TRACKING;
				}
			}
			else if (leaderTrackState == LEADER_TRACKING && hasExistingTrack)
			{
				filteredScanMsg.obstacles = laserMsg;
				SingleScanPoint currPoint;
				currPoint.angle = laserMsg.angle_min;
				bool isLeaderFound = false;
				for (const auto &point : laserMsg.ranges)
				{
					currPoint.range = point;
					if (isFit(lidarArclen, currPoint.range * (lidarAngle.getMean() - currPoint.angle)) && isFit(lidarRange, currPoint.range))
					{
						currPoint.isLeader = true;
						isLeaderFound = true;
						leaderIndex.push_back(laserScanPoints.size());
					}
					else
						currPoint.isLeader = false;
					/* Push SingleScanPoint structure into vector */
					laserScanPoints.push_back(currPoint);

					/* Angle */
					currPoint.angle += laserMsg.angle_increment;
				}
				if (doEdgeDetection)
					leaderTrackPostProcess(laserScanPoints, leaderIndex);

				float rangeAcc = 0, angleAcc = 0;
				uint8_t numLeaderPoints = 0;
				/* Remove leader points from published message */
				for (int i = 0; i < laserScanPoints.size(); i++)
				{
					if (laserScanPoints[i].isLeader == true)
					{
						rangeAcc += laserScanPoints[i].range;
						angleAcc += laserScanPoints[i].angle;
						numLeaderPoints++;
						filteredScanMsg.obstacles.ranges[i] = INFINITY;
#ifdef PUBLISH_LEADER_TRACK
						leaderTrk.ranges[i] = laserScanPoints[i].range;
#endif
					}
				}

				if (!isLeaderFound)
				{
					leaderTrackState = LEADER_LOST;
					filteredScanMsg.leaderTrackStatus = filteredScanMsg.LEADERTRACKSTATUS_LEADER_LOST;
					hasExistingTrack = false;
				}
				else
				{
					float avgRange = rangeAcc / numLeaderPoints;
					float avgAngle = angleAcc / numLeaderPoints;
					filteredScanMsg.centroid.x = avgRange * cos(avgAngle);
					filteredScanMsg.centroid.y = avgRange * sin(avgAngle);
					filteredScanMsg.leaderTrackStatus = filteredScanMsg.LEADERTRACKSTATUS_LEADER_TRACKING;

					lidarAngle = Distribution(avgAngle, 0.0, thresholdFactor);
					lidarRange = Distribution(avgRange, lidarRangeSD, thresholdFactor);
					lidarArclen = Distribution(0, lidarArcLenSD, thresholdFactor);
					hasExistingTrack = true;
					leaderTrackState = LEADER_TRACKING;
				}
			}
			else // leaderTrackState == LEADER_LOST
			{
				hasExistingTrack = false;
				filteredScanMsg.leaderTrackStatus = filteredScanMsg.LEADERTRACKSTATUS_LEADER_LOST;
				filteredScanMsg.obstacles = laserMsg;
				leaderTrackState = LEADER_LOST;
			}

			std_msgs::String trackStatus;
			if (filteredScanMsg.leaderTrackStatus == filteredScanMsg.LEADERTRACKSTATUS_LEADER_TRACKING)
			{
				trackStatus.data = "Tracking";
			}
			else if (filteredScanMsg.leaderTrackStatus == filteredScanMsg.LEADERTRACKSTATUS_LEADER_LOST)
			{
				trackStatus.data = "Lost";
			}
			else if (filteredScanMsg.leaderTrackStatus == filteredScanMsg.LEADERTRACKSTATUS_LEADER_NONE)
			{
				trackStatus.data = "None";
			}
			trackStatePub.publish(trackStatus);
			pub.publish(filteredScanMsg);
#ifdef PUBLISH_LEADER_TRACK
			ldrTrkPub.publish(leaderTrk);
#endif
		}
	}

	bool startTrackCallback(std_srvs::Empty::Request &request, std_srvs::Empty::Request &response)
	{
		leaderTrackState = LEADER_NONE;
		return true;
	}

	void enableTrackCallback(const logcart_driver::Joystick::ConstPtr &msg)
	{
		if (msg->enableTracking == 1)
		{
			leaderTrackState = LEADER_NONE;
			hasExistingTrack = false;
			trackingEnabled = true;
		}
		else
		{
			leaderTrackState = LEADER_LOST;
			hasExistingTrack = false;
			trackingEnabled = false;
		}
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "track_leader");

	ros::NodeHandle n;

	LeaderTracker tracker(n);

	// ros::Subscriber camSub = n.subscribe("openmv", 1, &LeaderTracker::camCallback, &tracker);
	ros::Subscriber leaderScanSub = n.subscribe("scan_filtered", 1, &LeaderTracker::laserScanCallback, &tracker);
	ros::Subscriber enableTrackSub = n.subscribe("joy_buttons", 1, &LeaderTracker::enableTrackCallback, &tracker);
	ros::ServiceServer startTrack = n.advertiseService("start_track", &LeaderTracker::startTrackCallback, &tracker);

	ros::spin();
	return 0;
}