// PCL_Test.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include <chrono>
#include <thread>
#include <filesystem>
#include <fstream>
#include <string>
#include <fmt/format.h>

#ifdef _DEBUG
#pragma comment(lib,"fmtd.lib")
#else
#pragma comment(lib,"fmt.lib")
#endif


#pragma pack(push,1)
struct PcdPoint
{
	float x;
	float y;
	float z;
	float intensity;
};

// Public Header结构体
struct PublicHeader {
	char file_signature[16];
	uint8_t versionA;
	uint8_t versionB;
	uint8_t versionC;
	uint8_t versionD;
	char magic_code[4];
	// filesignature前10个字节为livox_tech,后6个字节填充为0
	//versionA versionB为1，versionC versionD为0，表示此格式的版本
	//magiccode为0xAC0EA767
};
// Private Header结构体
struct PrivateHeader {
	unsigned int frame_duration;//一帧的持续时间，单位ms
	unsigned char device_count;//DeviceInfo的数量
};

// Device Info结构体
struct DeviceInfo {
	char lidar_sn[16];
	char hub_sn[16];
	unsigned char device_index; //Index in device info list
	unsigned char device_type; //设备类型 0：lidar hub  1:Mid-40.100 2:Tele-15 4:Horizon
	unsigned char extrinsic_enable;//0：禁用外部参数，应在没有外部参数的情况下计算点云；   1：启用外部参数，应使用外部参数计算云点；
	float roll;
	float pitch;
	float yaw;
	float x;
	float y;
	float z;
};

// Frame Header结构体
struct FrameHeader {
	long long cur_offset;
	long long next_offset;
	long long frame_index;
};
// Package结构体
struct Package {
	unsigned char device_index;//refer to deviceInfo
	uint8_t version;
	uint8_t slot_id;
	uint8_t lidar_id;
	uint8_t reserved;
	uint32_t status_code;
	uint8_t timestamp_type;
	uint8_t data_type;//点云坐标格式
	uint64_t timestamp; //ns
};

struct Datatype_0 {
	int32_t x;
	int32_t y;
	int32_t z;
	int32_t reflectivity;
};

struct Datatype_1 {
	int32_t depth;
	int32_t theta;
	int32_t phi;
	int32_t reflectivity;
};

struct Datatype_2 {
	int32_t x;
	int32_t y;
	int32_t z;
	uint8_t reflectivity;
	uint8_t tag;
};

struct Datatype_3 {
	int32_t depth;
	int32_t theta;
	int32_t phi;
	int32_t reflectivity;
	uint8_t tag;
};

struct Datatype_4 {
	int32_t x1;
	int32_t y1;
	int32_t z1;
	uint8_t reflectivity1;
	uint8_t tag1;
	int32_t x2;
	int32_t y2;
	int32_t z2;
	uint8_t reflectivity2;
	uint8_t tag2;
};

struct Datatype_5 {
	int32_t theta;
	int32_t phi;
	int32_t depth1;
	int32_t reflectivity1;
	uint8_t tag1;
	int32_t depth2;
	int32_t reflectivity2;
	uint8_t tag2;
};

struct Datatype_6 {
	float gyro_x;
	float gyro_y;
	float gyro_z;
	float acc_x;
	float acc_y;
	float acc_z;
};

struct Datatype_7 {
	int32_t x1;            /**< X axis, Unit:mm */
	int32_t y1;            /**< Y axis, Unit:mm */
	int32_t z1;            /**< Z axis, Unit:mm */
	uint8_t reflectivity1; /**< Reflectivity */
	uint8_t tag1;          /**< Tag */
	int32_t x2;            /**< X axis, Unit:mm */
	int32_t y2;            /**< Y axis, Unit:mm */
	int32_t z2;            /**< Z axis, Unit:mm */
	uint8_t reflectivity2; /**< Reflectivity */
	uint8_t tag2;          /**< Tag */
	int32_t x3;            /**< X axis, Unit:mm */
	int32_t y3;            /**< Y axis, Unit:mm */
	int32_t z3;            /**< Z axis, Unit:mm */
	uint8_t reflectivity3; /**< Reflectivity */
	uint8_t tag3;          /**< Tag */
};

/** Triple extend spherical coordinate format. */
struct Datatype_8 {
	uint16_t theta;        /**< Zenith angle[0, 18000], Unit: 0.01 degree */
	uint16_t phi;          /**< Azimuth[0, 36000], Unit: 0.01 degree */
	uint32_t depth1;       /**< Depth, Unit: mm */
	uint8_t reflectivity1; /**< Reflectivity */
	uint8_t tag1;          /**< Tag */
	uint32_t depth2;       /**< Depth, Unit: mm */
	uint8_t reflectivity2; /**< Reflectivity */
	uint8_t tag2;          /**< Tag */
	uint32_t depth3;       /**< Depth, Unit: mm */
	uint8_t reflectivity3; /**< Reflectivity */
	uint8_t tag3;          /**< Tag */
};

/** IMU data format. */
typedef struct {
	float gyro_x;        /**< Gyroscope X axis, Unit:rad/s */
	float gyro_y;        /**< Gyroscope Y axis, Unit:rad/s */
	float gyro_z;        /**< Gyroscope Z axis, Unit:rad/s */
	float acc_x;         /**< Accelerometer X axis, Unit:g */
	float acc_y;         /**< Accelerometer Y axis, Unit:g */
	float acc_z;         /**< Accelerometer Z axis, Unit:g */
} LivoxImuPoint;
#pragma pack(pop)  // 恢复默认的字节对齐方式


void Convert(std::filesystem::path filepath)
{
	std::cout << filepath.string() << std::endl;

	auto root = filepath.parent_path();
	auto name = filepath.filename().string();

	auto dir = root / (name + "_output");
	std::filesystem::create_directories(dir);

	int frame_index = 0;
	int file_index = 0;
	std::vector<PcdPoint> cloud_points;

	std::ifstream file(filepath, std::ios::binary);  // 打开二进制文件
	if (!file) {
		std::cout << "无法打开文件." << std::endl;
		return;
	}

	// 读取Public Header
	PublicHeader publicHeader;
	file.read(reinterpret_cast<char*>(&publicHeader), sizeof(PublicHeader));

	// 读取Private Header
	PrivateHeader privateHeader;
	file.read(reinterpret_cast<char*>(&privateHeader), sizeof(PrivateHeader));
	// 读取DeviceInfo
	DeviceInfo deviceInfo;
	file.read(reinterpret_cast<char*>(&deviceInfo), sizeof(DeviceInfo));

	while (file.peek() != EOF) {
		long long bytes = 24;//frameheader字节24
		// 读取Frame Header
		FrameHeader frame_header;
		file.read(reinterpret_cast<char*>(&frame_header), sizeof(FrameHeader));
		// 计算每个Frame的字节数
		uint32_t frame_size = frame_header.next_offset - frame_header.cur_offset;

		frame_index++;
		while (bytes < frame_size) {
			Package package;
			file.read(reinterpret_cast<char*>(&package), sizeof(Package));
			bytes += 19;

			if (package.data_type == 7)
			{
				for (int i = 0; i < 30; i++)
				{
					Datatype_7 pointdata;
					file.read(reinterpret_cast<char*>(&pointdata), sizeof(Datatype_7));
					bytes += sizeof(Datatype_7);

					PcdPoint pt;
					pt.x = double((pointdata.x1) / 1000.f);
					pt.y = double((pointdata.y1) / 1000.f);
					pt.z = double((pointdata.z1) / 1000.f);
					pt.intensity = pointdata.reflectivity1;

					cloud_points.push_back(pt);
				}
			}
			else if (package.data_type == 6)
			{
				Datatype_6 pointdata;
				file.read(reinterpret_cast<char*>(&pointdata), sizeof(Datatype_6));
				bytes += sizeof(Datatype_6);
			}
			else if (package.data_type == 0)
			{
				for (int i = 0; i < 100; i++)
				{
					Datatype_0 pointdata;
					file.read(reinterpret_cast<char*>(&pointdata), sizeof(Datatype_0));
					bytes += sizeof(Datatype_0);

					PcdPoint pt;
					pt.x = double((pointdata.x) / 1000.f);
					pt.y = double((pointdata.y) / 1000.f);
					pt.z = double((pointdata.z) / 1000.f);
					pt.intensity = pointdata.reflectivity;

					cloud_points.push_back(pt);
				}
			}
			else if (package.data_type == 2)
			{
				for (int i = 0; i < 96; i++)
				{
					Datatype_2 pointdata;
					file.read(reinterpret_cast<char*>(&pointdata), sizeof(Datatype_2));
					bytes += sizeof(Datatype_2);

					PcdPoint pt;
					pt.x = double((pointdata.x) / 1000.f);
					pt.y = double((pointdata.y) / 1000.f);
					pt.z = double((pointdata.z) / 1000.f);
					pt.intensity = pointdata.reflectivity;

					cloud_points.push_back(pt);
				}
			}
		}

		if (frame_index == 40)
		{
			auto filename = fmt::format("{}.pcd", ++file_index);
			auto filepath = dir / filename;

			std::string pcd_header =
				"# .PCD v.7 - Point Cloud Data file format\n"
				"FIELDS "
				"x y z intensity\n"
				"SIZE 4 4 4 4\n"
				"TYPE F F F F\n"
				"COUNT 1 1 1 1\n"
				"WIDTH {}\n"
				"HEIGHT 1\n"
				"VIEWPOINT 0 0 0 1 0 0 0\n"
				"POINTS {}\n"
				"DATA binary\n";

			std::ofstream fs(filepath, std::ios::binary);
			auto text = fmt::format(pcd_header, (int)cloud_points.size(), (int)cloud_points.size());
			fs.write(text.c_str(), text.length());
			fs.write((char*)cloud_points.data(), cloud_points.size() * sizeof(PcdPoint));
			fs.close();

			std::cout << filename << std::endl;

			frame_index = 0;
			cloud_points.clear();
		}
	}

	file.close();
}

int main() {
	auto root = std::filesystem::current_path();
	for (auto& item : std::filesystem::directory_iterator(root))
	{
		auto filepath = std::filesystem::path(item);
		auto ext = filepath.extension();

		if (ext == ".lvx")
		{
			Convert(filepath);
		}
	}

	return 0;
}

// 运行程序: Ctrl + F5 或调试 >“开始执行(不调试)”菜单
// 调试程序: F5 或调试 >“开始调试”菜单

// 入门使用技巧: 
//   1. 使用解决方案资源管理器窗口添加/管理文件
//   2. 使用团队资源管理器窗口连接到源代码管理
//   3. 使用输出窗口查看生成输出和其他消息
//   4. 使用错误列表窗口查看错误
//   5. 转到“项目”>“添加新项”以创建新的代码文件，或转到“项目”>“添加现有项”以将现有代码文件添加到项目
//   6. 将来，若要再次打开此项目，请转到“文件”>“打开”>“项目”并选择 .sln 文件
