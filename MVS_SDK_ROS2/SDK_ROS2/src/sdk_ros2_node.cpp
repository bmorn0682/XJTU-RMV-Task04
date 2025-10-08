#include "rclcpp/rclcpp.hpp"
#include "MvCameraControl.h"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include <cstdio>
#include <cstring>
#include <iostream>
#include <ostream>
#include <string>
#include <thread>
#include <memory>

bool g_bExit = false;
class CameraNode : public rclcpp::Node
{
public:
	CameraNode() : Node("camera_node")
	{
		// 声明参数并设置默认值
        this->declare_parameter("exposure_time", 5000.0);  // μs
        this->declare_parameter("gain", 1.0);
        this->declare_parameter("frame_rate", 30.0);
        this->declare_parameter("pixel_format", std::string("mono8"));
		this->declare_parameter("real_frame_rate", 40.0);
        this->declare_parameter("real_exposure_time", 6000.0);
        this->declare_parameter("real_gain", 2.0);
        this->declare_parameter("real_pixel_format", std::string("mono8"));
		

		callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&CameraNode::parametersCallback, this, std::placeholders::_1));
    	
		RCLCPP_INFO(this->get_logger(), "Camera node started");
		publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/image_raw", 10);

		timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&CameraNode::update_params, this)
        );
		Camera_thread_ = std::thread(&CameraNode::cameraLoop, this);
	}

	~CameraNode()
	{
		// signal worker thread(s) to exit
		g_bExit = true;
		// give worker threads a moment to clean up
		sleep(1);
		if (Camera_thread_.joinable())
            Camera_thread_.join();
	}

	void cameraLoop()
	{
		int nRet = MV_OK;
		do 
    	{
			// ch:初始化SDK | en:Initialize SDK
			nRet = MV_CC_Initialize();
			if (MV_OK != nRet)
			{
				printf("Initialize SDK fail! nRet [0x%x]\n", nRet);
				break;
			}

			// 枚举设备 
			MV_CC_DEVICE_INFO_LIST stDeviceList;
			memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
			nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE | MV_GENTL_CAMERALINK_DEVICE | MV_GENTL_CXP_DEVICE | MV_GENTL_XOF_DEVICE, &stDeviceList);
			if (MV_OK != nRet)
			{
				printf("Enum Devices fail! nRet [0x%x]\n", nRet);
				break;
			}
			std::cout<<"DeviceNum= "<< stDeviceList.nDeviceNum << std::endl;
			// 需根据任务需求修改
			if (stDeviceList.nDeviceNum > 0)
			{
				for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++)
				{
					printf("[device %d]:\n", i);
					MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
					if (NULL == pDeviceInfo)
					{
						break;
					} 
					//PrintDeviceInfo(pDeviceInfo);            
				}  
			} 
			else
			{
				printf("Find No Devices!\n");
				break;
			}

			//printf("Please Input camera index(0-%d):", stDeviceList.nDeviceNum-1);
			//unsigned int nIndex = 0;
			//std::cin >> nIndex;

			//if (nIndex >= stDeviceList.nDeviceNum)
			//{
			//	printf("Input error!\n");
			//	break;
			//}

			// ch:选择设备并创建句柄 | en:Select device and create handle
			nRet = MV_CC_CreateHandle(&handle_, stDeviceList.pDeviceInfo[0]);
			DeviceInfo = stDeviceList.pDeviceInfo[0];
			if (MV_OK != nRet)
			{
				printf("Create Handle fail! nRet [0x%x]\n", nRet);
				break;
			}

			// ch:打开设备 | en:Open device
			nRet = MV_CC_OpenDevice(handle_);

			// ch:注册异常消息回调 | en:Register exception callback
			MV_CC_RegisterExceptionCallBack(handle_, &onExceptionCallback, this);
			MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", true);
			if (MV_OK != nRet)
			{
				printf("Open Device fail! nRet [0x%x]\n", nRet);
				break;
			}

			// ch:设置触发模式为off | en:Set trigger mode as off
			nRet = MV_CC_SetEnumValue(handle_, "TriggerMode", 0);
			if (MV_OK != nRet)
			{
				printf("Set Trigger Mode fail! nRet [0x%x]\n", nRet);
				break;
			}

			// ch:开始取流 | en:Start grab image
			nRet = MV_CC_SetImageNodeNum(handle_, 5);
			if (MV_OK != nRet)
        	{
            	printf("Set number of image node fail! nRet [0x%x]\n", nRet);
            	break;
        	}

			nRet = MV_CC_StartGrabbing(handle_);
			
			if (MV_OK != nRet)
			{
				printf("Start Grabbing fail! nRet [0x%x]\n", nRet);
				break;
			}

			pthread_t nThreadID = 0;
			nRet = pthread_create(&nThreadID, NULL ,WorkThread , this);
			if (nRet != 0)
			{
				printf("thread create failed.ret = %d\n",nRet);
				break;
			}

			PressEnterToExit();


			// ch:停止取流 | en:Stop grab image
			nRet = MV_CC_StopGrabbing(handle_);
			if (MV_OK != nRet)
			{
				printf("Stop Grabbing fail! nRet [0x%x]\n", nRet);
				break;
			}

			// ch:关闭设备 | Close device
			nRet = MV_CC_CloseDevice(handle_);
			if (MV_OK != nRet)
			{
				printf("ClosDevice fail! nRet [0x%x]\n", nRet);
				break;
			}

			// ch:销毁句柄 | Destroy handle
			nRet = MV_CC_DestroyHandle(handle_);
			if (MV_OK != nRet)
			{
				printf("Destroy Handle fail! nRet [0x%x]\n", nRet);
				break;
			}
			handle_ = NULL;
		} while (0);
		MV_CC_CloseDevice(handle_);
		if (handle_ != NULL)
		{
			MV_CC_DestroyHandle(handle_);
			handle_ = NULL;
		}

		MV_CC_Finalize();
	}
private:
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
	std::thread Camera_thread_;
	void* handle_ = nullptr;
    bool is_reconnecting_ = false;
	MV_CC_DEVICE_INFO* DeviceInfo = nullptr;
	rclcpp::TimerBase::SharedPtr timer_;
	    // 参数存储变量
    double exposure_time_;
    double gain_;
    float frame_rate_;
    std::string pixel_format_;
	MVCC_FLOATVALUE exposure_val;
	MVCC_FLOATVALUE frame_rate_val;
	MVCC_FLOATVALUE gain_val;

	OnSetParametersCallbackHandle::SharedPtr callback_handle_;

	static void* WorkThread(void* pUser)
	{
		int nRet = MV_OK;
		CameraNode* node = static_cast<CameraNode*>(pUser);
		MV_FRAME_OUT stImageInfo = {};
		memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT));

		while(1)
		{
			if(g_bExit)
			{
				break;
			}

			nRet = MV_CC_GetImageBuffer(node->handle_, &stImageInfo, 1000);
			if (nRet == MV_OK)
			{
				sensor_msgs::msg::Image img_raw = makeRosImageFromRaw(node,stImageInfo);
				node->publisher_->publish(img_raw);
				MV_CC_FreeImageBuffer(node->handle_, &stImageInfo);
			}
			else{
				printf("No data[%x]\n", nRet);
			}
		}

		return 0;
	}

	void PressEnterToExit(void)
	{
		int c;
		while ( (c = getchar()) != '\n' && c != EOF );
		fprintf( stderr, "\nPress enter to exit.\n");
		while( getchar() != '\n');
		g_bExit = true;
		sleep(1);
	}

	static sensor_msgs::msg::Image makeRosImageFromRaw(CameraNode* node, MV_FRAME_OUT &frame)				
	{
		sensor_msgs::msg::Image img;
		img.header.stamp = node->get_clock()->now();
		img.header.frame_id = "camera_optical_frame";
		img.width = frame.stFrameInfo.nWidth;
		img.height = frame.stFrameInfo.nHeight;
		img.encoding = PixelTypeToRosEncoding(frame.stFrameInfo.enPixelType);
		if (img.encoding == "unknown") {
			fprintf(stderr, "Unsupported pixel format: %ld\n", frame.stFrameInfo.enPixelType);
		}
		img.is_bigendian = 0;   // 小端系统一般是 0
		int ncannel = sensor_msgs::image_encodings::numChannels(img.encoding);
		int bytes_per_channel = sensor_msgs::image_encodings::bitDepth(img.encoding) / 8;
		img.step = frame.stFrameInfo.nWidth * ncannel * bytes_per_channel;   // 每行的字节数 = 宽度 * 通道数 * 每通道字节数
		img.data.resize(img.step * img.height);
    	memcpy(img.data.data(), static_cast<unsigned char*>(frame.pBufAddr), img.data.size());
		return img;
	}

	static std::string PixelTypeToRosEncoding(unsigned int enPixelType)
{
    using namespace std;
    switch (enPixelType)
    {
        case PixelType_Gvsp_Mono8:
            return "mono8";
        case PixelType_Gvsp_Mono10:
        case PixelType_Gvsp_Mono12:
        case PixelType_Gvsp_Mono10_Packed:
        case PixelType_Gvsp_Mono12_Packed:
		case PixelType_Gvsp_Mono14:
		case PixelType_Gvsp_Mono16:
            return "mono16"; // 转换时注意右移补齐到16位

        case PixelType_Gvsp_BayerRG8:
            return "bayer_rggb8";
        case PixelType_Gvsp_BayerBG8:
            return "bayer_bggr8";
        case PixelType_Gvsp_BayerGB8:
            return "bayer_gbrg8";
        case PixelType_Gvsp_BayerGR8:
            return "bayer_grbg8";
		case PixelType_Gvsp_BayerBG16:
			return "bayer_bggr16";
		case PixelType_Gvsp_BayerGB16:
			return "bayer_gbrg16";
		case PixelType_Gvsp_BayerRG16:
			return "bayer_rggb16";
		case PixelType_Gvsp_BayerGR16:
			return "bayer_grbg16";

        case PixelType_Gvsp_RGB8_Packed:
            return "rgb8";
        case PixelType_Gvsp_BGR8_Packed:
            return "bgr8";
		case PixelType_Gvsp_RGB16_Packed:
			return "rgb16";
		case PixelType_Gvsp_BGR16_Packed:
			return "bgr16";
		case PixelType_Gvsp_RGBA8_Packed:
			return "rgba8";
		case PixelType_Gvsp_BGRA8_Packed:
			return "bgra8";
		case PixelType_Gvsp_RGBA16_Packed:
			return "rgba16";
		case PixelType_Gvsp_BGRA16_Packed:
			return "bgra16";

        case PixelType_Gvsp_YUV422_Packed:
            return "yuv422";
        case PixelType_Gvsp_YUV422_YUYV_Packed:
            return "yuv422_yuyv";

        default:
            return "unknown";
    }
}

	 void update_params()
    {
        // 读取相机实际值更新 ROS 参数
        MV_CC_GetFloatValue(handle_, "ExposureTime", &exposure_val);
		MV_CC_GetFloatValue(handle_, "ResultingFrameRate", &frame_rate_val);//ResultingFrameRate
		MV_CC_GetFloatValue(handle_, "Gain", &gain_val);
        set_parameter(rclcpp::Parameter("real_exposure_time", exposure_val.fCurValue));
		set_parameter(rclcpp::Parameter("real_frame_rate", frame_rate_val.fCurValue));
		set_parameter(rclcpp::Parameter("real_gain", gain_val.fCurValue));
		RCLCPP_INFO(this->get_logger(),"Current FrameRate is: %f",frame_rate_val.fCurValue);
    }

	rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &param : params)
        {
            if (param.get_name() == "exposure_time")
            {
                exposure_time_ = param.as_double();
				MV_CC_SetFloatValue(handle_, "ExposureTime", exposure_time_);
                
            }
            else if (param.get_name() == "gain")
            {
                gain_ = param.as_double();
                MV_CC_SetFloatValue(handle_, "Gain", gain_);
            }
            else if (param.get_name() == "frame_rate")
            {
                frame_rate_ = param.as_double();
				MV_CC_StopGrabbing(handle_);
    			MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", static_cast<float>(frame_rate_));
				MV_CC_StartGrabbing(handle_);
            }
            else if (param.get_name() == "pixel_format")
            {
                pixel_format_ = param.as_string();
				MV_CC_SetEnumValueByString(handle_, "PixelFormat", pixel_format_.c_str());
			}
        }
		return result;
	}

	static void __stdcall onExceptionCallback(unsigned int nMsgType, void* pUser)
    {
        auto* self = static_cast<CameraNode*>(pUser);
        self->handleException(nMsgType);
    }

    // 实际处理逻辑
    void handleException(unsigned int nMsgType)
    {
        switch (nMsgType)
        {
        case MV_EXCEPTION_DEV_DISCONNECT:
            RCLCPP_ERROR(this->get_logger(), "Camera disconnected!");
			if (is_reconnecting_) return;  // 防止重复重连
            is_reconnecting_ = true;

            std::thread([this]() {
                const int max_retry = 5;
                const int retry_delay_ms = 2000;
                bool reconnected = false;
				for (int i = 1; i <= max_retry; ++i)
                {
                    RCLCPP_INFO(this->get_logger(), "Trying to reconnect camera... Attempt %d/%d", i, max_retry);

                    MV_CC_StopGrabbing(handle_);
					MV_CC_CloseDevice(handle_);
					MV_CC_DestroyHandle(handle_);
					handle_ = nullptr;
                    std::this_thread::sleep_for(std::chrono::milliseconds(retry_delay_ms));

					MV_CC_CreateHandle(&handle_,DeviceInfo);
					int nRet = MV_CC_OpenDevice(handle_);
					if (nRet != MV_OK)
					{
						RCLCPP_ERROR(this->get_logger(), "Failed to open device. nRet [0x%x]", nRet);
						continue;
					}
					nRet = MV_CC_SetEnumValue(handle_, "TriggerMode", 0);
					if (MV_OK != nRet)
					{
						printf("Set Trigger Mode fail! nRet [0x%x]\n", nRet);
						break;
					}
					nRet = MV_CC_StartGrabbing(handle_);
					if (nRet == MV_OK)
					{
						RCLCPP_INFO(this->get_logger(), "Reconnected to camera successfully.");
						reconnected = true;
						break;
					}
                }

                if (!reconnected)
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to reconnect after %d attempts.", max_retry);
                }

                is_reconnecting_ = false;
            }).detach();
            break;

        case MV_E_USB_DEVICE:
            RCLCPP_ERROR(this->get_logger(), "USB_DEVICE exception!");
            break;

		case MV_E_USB_DRIVER:
			RCLCPP_ERROR(this->get_logger(), "USB_DRIVER exception!");
			break;

        default:
            RCLCPP_WARN(this->get_logger(), "Unknown MVS exception: 0x%x", nMsgType);
            break;
        }
    }
};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	// instantiate our CameraNode (starts cameraLoop in background)
	auto node = std::make_shared<CameraNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
