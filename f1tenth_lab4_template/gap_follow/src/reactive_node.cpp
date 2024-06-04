#include <sstream>
#include <string>
#include <cmath>
#include <array>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

#define ARRAY_LENGTH 1080
#define ARRAY_LENGTH_MIN 280 // 45 + 25 = 280
#define ARRAY_LENGTH_MAX 800 // 225 - 25 = 800
// 270/1080 = 0.25, 45/0.25 = 180, (270 - 45)/0.25 = 900

using namespace std;

const string &lidarscan_topic = "/scan";
const string &drive_topic = "/drive";

class ReactiveFollowGap : public rclcpp::Node {

public:

    ReactiveFollowGap() : Node("reactive_node")
    {

	scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(lidarscan_topic, 10, std::bind(&ReactiveFollowGap::scan_callback, this, std::placeholders::_1));

        drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);

        scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_pub", 10);

    }

    float getUserInput(const std::string& prompt)
    {
        float input;
        std::cout << prompt;
        std::cin >> input;

        return input;
    }

private:

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;

    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    sensor_msgs::msg::LaserScan scanpub_msg;

    const int radius = getUserInput("radius(100): ");
    const int ok_range = getUserInput("ok_range(50): ");
    const float max_value = getUserInput("max_value(1.4): ");
    const float bubble_radius = getUserInput("bubble_radius(0.0): ");
    //float last_processed_ranges[ARRAY_LENGTH] = {0.0};

    vector<float> preprocess_lidar(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
    {
	vector<float> filtered_ranges;

	for (int i = 0; i < ARRAY_LENGTH; ++i)
	{
            if (i < ARRAY_LENGTH_MIN || i > ARRAY_LENGTH_MAX)
            {
                filtered_ranges.push_back(0.0);
            }
	    else if (isnan(scan_msg->ranges[i]))
	    {
	        filtered_ranges.push_back(0.0);
		cout << "insert zero" << endl;
	    }
	    else if (scan_msg->ranges[i] > max_value || isinf(scan_msg->ranges[i]))
	    {
		filtered_ranges.push_back(max_value);
	    }
	    else
	    {
	        filtered_ranges.push_back(scan_msg->ranges[i]);
	    }

	    //float before_ranges = filtered_ranges[i];
	    //filtered_ranges[i] = (last_processed_ranges[i] == 0.0) ? filtered_ranges[i] : (filtered_ranges[i] + last_processed_ranges[i]) / 2.0;
	    // 이전 데이터와의 평균으로 어느정도 보정
	    //last_processed_ranges[i] = before_ranges;

	}

        return filtered_ranges;

    }

    int find_closest_point(vector<float>& ranges_vec)
    {
	// 인덱스 반환
        return min_element(ranges_vec.begin() + ARRAY_LENGTH_MIN, ranges_vec.begin() + ARRAY_LENGTH_MAX) - ranges_vec.begin();
    }

    void process_bubble(vector<float>* ranges_vec, int closest_index)
    {
	// ranges_vec 벡터를 주소로 받아와서 원본 수정
	// 가장 작은 거리 + 버블 반지름을 기준으로 그보다 작은경우 0으로 초기화
	const float center_point_distance = ranges_vec->at(closest_index);

        int left = max(ARRAY_LENGTH_MIN, closest_index - radius);
        // 만약 closest_index의 왼쪽 범위가 100보다 작다면 무조건 0이 더 큼
        int right = min(ARRAY_LENGTH_MAX - 1, closest_index + radius);
        // 만약 closest_index의 오른쪽 범위가 1080보다 크다면 무조건 1079가 더 작음

        for (int i = ARRAY_LENGTH_MIN; i < ARRAY_LENGTH_MAX; ++i)
	{
	    if (ranges_vec->at(i) < (center_point_distance + bubble_radius) || (i >= left && i <= right))
	    {
	        ranges_vec->at(i) = 0.0;
	    }
	}
    }

    pair<int, int> find_max_gap(vector<float>& ranges_vec)
    {
	int max_start = 0;
	int max_size = 0;
	int max_before = 0;

	int current_index = ARRAY_LENGTH_MIN;
	//uint_8로 변경하기

	// 0.1보다 크면 반복하여 그동안의 길이를 저장하고 current_index 업데이트
	while (current_index < ARRAY_LENGTH_MAX)
	{
            max_size = 0;

	    while (ranges_vec[current_index] > 0.1)
	    {
        	current_index++;
		max_size++;
	    }

	    if (max_size > max_before)
	    {
	        max_before = max_size;
		max_start = current_index - max_size;
	    }

	    current_index++;

	}

        if (max_size > max_before)
        // current_index가 ARRAY_LENGTH_MAX를 넘는 순간 종료되므로
        // 그동안의 max_size값이 max_before보다 크다면 값을 할당해줘야함
        {
            max_before = max_size;
            max_start = current_index - max_size;
        }

	// max_gap의 시작점과 끝점을 반환
        return {max_start, max_start + max_before - 1};
        // - 1은 max_start가 포함되기 때문?
    }

    int find_best_point(int start_index, int end_index, vector<float>& ranges_vec)
    {
        vector<int> max_indexs;

        float max = *max_element(ranges_vec.begin() + start_index, ranges_vec.begin() + end_index);
        auto it = find(ranges_vec.begin() + start_index, ranges_vec.begin() + end_index, max);

        while (it != ranges_vec.begin() + end_index)
        {
            max_indexs.push_back(it - ranges_vec.begin());
            it = find(it+1, ranges_vec.begin() + end_index, max);
        }

        int max_point = max_indexs[max_indexs.size() / 2];
        int i = 0;
        while (((ranges_vec[max_point] - ranges_vec[max_point - i]) > 2.0 || (ranges_vec[max_point] - ranges_vec[max_point + i]) > 2.0) && (max_indexs.size()) != 0)
        {
            max_indexs.erase(max_indexs.begin() + max_indexs.size() / 2);
            max_point = max_indexs[max_indexs.size() / 2];

            if (i != ok_range)
            {
                i++;
            }
            else
            {
                break;
            }
        }

        return max_point;

    }

    float Convert(float radian) { return (radian * (M_PI / 180.0)); }

    float get_velocity(float steering_angle)
    {

        if (abs(steering_angle) >= Convert(0.0) && abs(steering_angle) < Convert(10.0))
	{
	    return 2.0;
	}
	else if (abs(steering_angle) >= Convert(10.0) && abs(steering_angle) < Convert(20.0))
	{
	    return 1.0;
	}
	else
	{
	    return 0.5;
	}

    }


    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
    {


        float angle_min = scan_msg->angle_min;
        float angle_increment = scan_msg->angle_increment;

        auto ranges_vec = preprocess_lidar(scan_msg);
	// 벡터 전처리(nan, inf, max) & 이전 벡터값과의 평균
        const int closest_index = find_closest_point(ranges_vec);
        // 가장 작은 벡터 요소의 인덱스(가장 가까운 거리)
	//-> ranges_vec은 벡터 즉 연속된 저장공간을 가지기 때문에 ranges_vec은 첫번째 주소를 가르킴
	//-> 하여 함수는 주소를 받기 때문에 포인터 자료형으로 받아야지 주소로 접근하여 데이터를 취득가능
	process_bubble(&ranges_vec, closest_index);
        // 벡터의 가장 작은 요소 + 버블 반지름을 기준으로 벡터의 요소를 0으로 초기화(원본 변경)
        const auto [max_gap_start, max_gap_end] = find_max_gap(ranges_vec);
        // max_gap을 찾는 과정
	const int max_point = find_best_point(max_gap_start, max_gap_end, ranges_vec);
        //printf("closest_index = %d, max_gap_start = %d, max_gap_end = %d, max_point = %d\n", closest_index, max_gap_start, max_gap_end, max_point);
        // max_gap의 최장거리 인덱스 찾기
	//const float max_range = scan_msg->ranges[max_point];
        // max_range 저장

        const float steering_angle = (max_point * angle_increment) + angle_min;
        //printf("max_deg = %1.f도, max_range = %f, steering_angle = %f\n", max_point * 0.25 - 45, max_range, steering_angle);
        //printf("closest_index = %d, max_point = %d\n", closest_index, max_point);
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = get_velocity(steering_angle);
        drive_publisher_->publish(drive_msg);
        scanpub_msg = *scan_msg;
        scanpub_msg.ranges = ranges_vec;
        scan_publisher_->publish(scanpub_msg);

    }
};


int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}
