/*
* 语音听写(iFly Auto Transform)技术能够实时地将语音转换成对应的文字。
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "robot_voice/qisr.h"
#include "robot_voice/msp_cmn.h"
#include "robot_voice/msp_errors.h"
#include "robot_voice/speech_recognizer.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

#define FRAME_LEN	640 
#define	BUFFER_SIZE	4096

int wakeupFlag = 0;
int resultFlag = 0;


static void show_result(char *string, char is_over)
{
	printf("识别成功!!!\n");
	resultFlag =1;
	printf("\r识别结果: [ %s ]", string);
	if(is_over)
		putchar('\n');
}

static char *g_result = NULL;
static unsigned int g_buffersize = BUFFER_SIZE;

void on_result(const char *result, char is_last)
{
	if (result) {
		size_t left = g_buffersize - 1 - strlen(g_result);
		size_t size = strlen(result);
		if (left < size) {
			g_result = (char*)realloc(g_result, g_buffersize + BUFFER_SIZE);
			if (g_result)
				g_buffersize += BUFFER_SIZE;
			else {
				printf("mem alloc failed\n");
				return;
			}
		}
		strncat(g_result, result, size);
		show_result(g_result, is_last);
	}
}
void on_speech_begin()
{
	if (g_result)
	{
		free(g_result);
	}
	g_result = (char*)malloc(BUFFER_SIZE);
	g_buffersize = BUFFER_SIZE;
	memset(g_result, 0, g_buffersize);

	printf("开始监听...\n");
}
void on_speech_end(int reason)
{
	if (reason == END_REASON_VAD_DETECT)
		printf("\n监听结束!!! \n");
	else
		printf("\nRecognizer error %d\n", reason);
}


/* demo recognize the audio from microphone */
static void demo_mic(const char* session_begin_params)
{
	int errcode;
	int i = 0;

	struct speech_rec iat;

	struct speech_rec_notifier recnotifier = {
		on_result,
		on_speech_begin,
		on_speech_end
	};

	errcode = sr_init(&iat, session_begin_params, SR_MIC, &recnotifier);
	if (errcode) {
		printf("speech recognizer init failed\n");
		return;
	}
	errcode = sr_start_listening(&iat);
	if (errcode) {
		printf("start listen failed %d\n", errcode);
	}
	/* demo 4 seconds recording */
	while(i++ < 4)
		sleep(1);
	errcode = sr_stop_listening(&iat);
	if (errcode) {
		printf("stop listening failed %d\n", errcode);
	}

	sr_uninit(&iat);
}

void WakeUp(const std_msgs::String::ConstPtr& msg)
{
    printf("waking up\r\n");
    //usleep(700*1000);
    wakeupFlag=1;
}


/* main thread: start/stop record ; query the result of recgonization.
 * record thread: record callback(data write)
 * helper thread: ui(keystroke detection)
 */
int main(int argc, char* argv[])
{	
	// ros初始化
	ros::init(argc,argv,"Voice_recognition");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);

	//订阅语音唤醒信号
	ros::Subscriber wakeupSub = n.subscribe("voiceWakeup",1000, WakeUp);

	//发布语音识别到的结果
	ros::Publisher voiceWorldPub = n.advertise<std_msgs::String>("voiceWords",1000);

	ROS_INFO("休眠中...");
	int count = 0;


	int ret = MSP_SUCCESS;
	int upload_on =	1; /* 是否导入用户字典 */
	/* 登陆参数，需要保持与libmsc一致 */
	const char* login_params = "appid = 6013f797, work_dir = .";
	int aud_src = 0; /* 从麦克风或者文件来识别 */

	/*
	* See "iFlytek MSC Reference Manual"
	*/
	const char* session_begin_params =
		"sub = iat, domain = iat, language = zh_cn, "
		"accent = mandarin, sample_rate = 16000, "
		"result_type = plain, result_encoding = utf8";

	/* 登陆. */
	ret = MSPLogin(NULL, NULL, login_params);
	if (MSP_SUCCESS != ret)	{
		printf("MSPLogin failed , Error code %d.\n",ret);
		goto exit; // login fail, exit the program
	}

	printf("\n###########################################################################\n");
	printf("## 欢迎使用语音控制小车运动demo ##\n");
	printf("## 每次识别前需要发送任意唤醒信息  ##\n");
	printf("## 可以识别的命令如下：  ##\n");
	printf("## 前进/后退  ##\n");
	printf("## 左转/右转  ##\n");
	printf("## 加速/减速  ##\n");
	printf("## 停止  ##\n");
	printf("## 其他命令将会被认为无效命令，小车将保持当前运动状态  ##\n");
	printf("###########################################################################\n\n");

	//循环识别语音
	while(ros::ok())
	{
		if(wakeupFlag){
			ROS_INFO("已经唤醒，可以开始识别...");
			printf("请从麦克风输入任意语音\n");
			demo_mic(session_begin_params);//语音识别主函数
			wakeupFlag = 0;
		}

		if(resultFlag){
			resultFlag = 0;
			std_msgs::String msg;
			msg.data = g_result;
			voiceWorldPub.publish(msg);
		}

		ros::spinOnce();
		loop_rate.sleep();
		count++;
	}


exit:
	MSPLogout(); // 注销

	return 0;
}
