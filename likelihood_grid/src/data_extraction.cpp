
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

#include <sensor_msgs/Joy.h>

bool start;
bool stop;
bool check;

geometry_msgs::Point point;

float dist(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

uint pdist(geometry_msgs::Point p)
{
    geometry_msgs::Point p1, p2, p3, p4, p5, p6, p7, p8;
    p1.x = p8.x = 3.0;
    p1.y =  6.4;
    p8.y = -6.4;

    p2.x = p7.x = 5.0;
    p2.y = 5.0;
    p7.y = -5.0;

    p3.x = p6.x = 6.4;
    p3.y = 3.0;
    p6.y = -3.0;

    p4.x = p5.x = 7.0;
    p4.y = 1.0;
    p5.y = -1.0;

    std::vector<float> dvec;
    uint res = 1;

    dvec.push_back(dist(p, p1));
    dvec.push_back(dist(p, p2));
    dvec.push_back(dist(p, p3));
    dvec.push_back(dist(p, p4));
    dvec.push_back(dist(p, p5));
    dvec.push_back(dist(p, p6));
    dvec.push_back(dist(p, p7));
    dvec.push_back(dist(p, p8));

    float min = dvec.at(0);


    for(size_t i = 0; i < dvec.size(); i++)
    {
        if(dvec.at(i) < min)
        {
            res = i+1;
            min = dvec.at(i);
        }
    }

//    ROS_WARN("res: %d",res);
    return res;
}

void maxCallBack(const geometry_msgs::PointStampedConstPtr & msg)
{
    point = msg->point;
//    ROS_INFO("HI: (%.4f, %.4f)", point.x, point.y);

}

void joyCallBack(const sensor_msgs::JoyConstPtr& msg)
{

    if(msg->buttons[1])
    {
        start = true;
        stop = false;
        ROS_ERROR("START");
    }
    if(msg->buttons[2])
    {
        stop = true;
        start = false;
        ROS_ERROR("STOP: (%.4f, %.4f)", point.x, point.y);
    }
    if(msg->buttons[3])
    {
        check = true;
    }
    if(check && !msg->buttons[3])
    {
        check = false;
        ROS_INFO("NOW!");
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "data_extraction");
    ros::NodeHandle n;
    ros::Rate looprate(10);
    start = false;
    check = false;
    point.x = point.y = 0.0;
    float counter = 0;

    std::vector<uint> pc(8,0);

    std::vector<uint> pvec;

    ros::Subscriber max_sub = n.subscribe("maximum_probability", 10, maxCallBack);
    ros::Subscriber joy_sub = n.subscribe("/teleop/joy", 10, joyCallBack);

    while(ros::ok())
    {

        if(start)
        {
            pvec.push_back(pdist(point));
            counter++;
        }
        else
        {
            if(!pvec.empty()){

                for(uint8_t i = 0; i < 8;i++)
                {
                    pc.at(i) = std::count(pvec.begin(), pvec.end(), i+1);
                    float percent = pc.at(i) / counter;
                    ROS_INFO("it was on p[%d] %d times. %.2f percent", i+1, pc.at(i), percent);
                }

                ROS_WARN("Ended on %d", pvec.back());

                pvec.clear();
                counter = 0;
            }
        }
        ros::spinOnce();
        looprate.sleep();

    }

    return 0;
}
