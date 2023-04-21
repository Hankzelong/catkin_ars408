//样例只是提供一个简单的调用so库的方法供参考，程序接收，与发送函数设置在两个线程中，并且线程没有同步。
//现实中客户编程中，发送与接收函数不能同时调用（不支持多线程），如果在多线程中，一定需要互锁。需要客户自行完善代码。
  



#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include "ars408/controlcan.h"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include "ars408/ClusterList.h"

#include <math.h>
#include <signal.h>
#include <ctime>
#include <cstdlib>
#include "unistd.h"

typedef struct Cluster
{
	int ID;
    float Rad_distance;  //径向距离
    float Hor_angle; //水平角度
    float Rad_in_velocity;  //径向来向速度
    float Rad_go_velocity;  //径向去向速度
    float RCS;   //雷达反射截面积
    //float Obj_duration;  //目标持续时间
    float Obj_size;  //目标大小
    float Y_distance;  //Y轴坐标
	float X_distance;  //X轴坐标
	float r_l_Sub_velocity;  //从右向左分速度
	float in_Sub_velocity;  //来向分速度
	float l_r_Sub_velocity;  //从左向右分速度
	float go_Sub_velocity;  //去向分速度
} cluster;

typedef struct Data
{
    long long start_index; //起始标志
    uint num_cluster; //目标点数
    cluster point_cloud; //点云
    long long end_index; //结束标志
}data;

VCI_BOARD_INFO pInfo;//用来获取设备信息。
int count=0;//数据列表中，用来存储列表序号。
VCI_BOARD_INFO pInfo1 [50];
int num=0;
//void *receive_func(void* param)  //接收线程。
//{
	
	//printf("run thread exit\n");//退出接收线程	
	//pthread_exit(0);
//}
void mySigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}


int main(int argc, char **argv)
{
		ros::init(argc,argv,"ars408",ros::init_options::NoSigintHandler);
    	ros::NodeHandle node_obj;
    	ros::Publisher pub = node_obj.advertise<ars408::ClusterList>("ars408_radar",3000);
    	ros::Rate loop_rate(10);

		printf(">>this is hello !\r\n");//指示程序已运行

		num=VCI_FindUsbDevice2(pInfo1);

		printf(">>USBCAN DEVICE NUM:");printf("%d", num);printf(" PCS");printf("\n");

		for(int i=0;i<num;i++)
		{
		printf("Device:");printf("%d", i);printf("\n");
                printf(">>Get VCI_ReadBoardInfo success!\n");
		
		printf(">>Serial_Num:%c", pInfo1[i].str_Serial_Num[0]);
		printf("%c", pInfo1[i].str_Serial_Num[1]);
		printf("%c", pInfo1[i].str_Serial_Num[2]);
		printf("%c", pInfo1[i].str_Serial_Num[3]);
		printf("%c", pInfo1[i].str_Serial_Num[4]);
		printf("%c", pInfo1[i].str_Serial_Num[5]);
		printf("%c", pInfo1[i].str_Serial_Num[6]);
		printf("%c", pInfo1[i].str_Serial_Num[7]);
		printf("%c", pInfo1[i].str_Serial_Num[8]);
		printf("%c", pInfo1[i].str_Serial_Num[9]);
		printf("%c", pInfo1[i].str_Serial_Num[10]);
		printf("%c", pInfo1[i].str_Serial_Num[11]);
		printf("%c", pInfo1[i].str_Serial_Num[12]);
		printf("%c", pInfo1[i].str_Serial_Num[13]);
		printf("%c", pInfo1[i].str_Serial_Num[14]);
		printf("%c", pInfo1[i].str_Serial_Num[15]);
		printf("%c", pInfo1[i].str_Serial_Num[16]);
		printf("%c", pInfo1[i].str_Serial_Num[17]);
		printf("%c", pInfo1[i].str_Serial_Num[18]);
		printf("%c", pInfo1[i].str_Serial_Num[19]);printf("\n");

		printf(">>hw_Type:%c", pInfo1[i].str_hw_Type[0]);
		printf("%c", pInfo1[i].str_hw_Type[1]);
		printf("%c", pInfo1[i].str_hw_Type[2]);
		printf("%c", pInfo1[i].str_hw_Type[3]);
		printf("%c", pInfo1[i].str_hw_Type[4]);
		printf("%c", pInfo1[i].str_hw_Type[5]);
		printf("%c", pInfo1[i].str_hw_Type[6]);
		printf("%c", pInfo1[i].str_hw_Type[7]);
		printf("%c", pInfo1[i].str_hw_Type[8]);
		printf("%c", pInfo1[i].str_hw_Type[9]);printf("\n");	

		printf(">>Firmware Version:V");
		printf("%x", (pInfo1[i].fw_Version&0xF00)>>8);
		printf(".");
		printf("%x", (pInfo1[i].fw_Version&0xF0)>>4);
		printf("%x", pInfo1[i].fw_Version&0xF);
		printf("\n");
	}
		printf(">>\n");
		printf(">>\n");
		printf(">>\n");
	if(VCI_OpenDevice(VCI_USBCAN2,0,0)==1)//打开设备
	{
		printf(">>open deivce success!\n");//打开设备成功
	}else
	{
		printf(">>open deivce error!\n");
		exit(1);
	}
	if(VCI_ReadBoardInfo(VCI_USBCAN2,0,&pInfo)==1)//读取设备序列号、版本等信息。
	{
                printf(">>Get VCI_ReadBoardInfo success!\n");
		
		//printf(" %08X", pInfo.hw_Version);printf("\n");
		//printf(" %08X", pInfo.fw_Version);printf("\n");
		//printf(" %08X", pInfo.dr_Version);printf("\n");
		//printf(" %08X", pInfo.in_Version);printf("\n");
		//printf(" %08X", pInfo.irq_Num);printf("\n");
		//printf(" %08X", pInfo.can_Num);printf("\n");
		printf(">>Serial_Num:%c", pInfo.str_Serial_Num[0]);
		printf("%c", pInfo.str_Serial_Num[1]);
		printf("%c", pInfo.str_Serial_Num[2]);
		printf("%c", pInfo.str_Serial_Num[3]);
		printf("%c", pInfo.str_Serial_Num[4]);
		printf("%c", pInfo.str_Serial_Num[5]);
		printf("%c", pInfo.str_Serial_Num[6]);
		printf("%c", pInfo.str_Serial_Num[7]);
		printf("%c", pInfo.str_Serial_Num[8]);
		printf("%c", pInfo.str_Serial_Num[9]);
		printf("%c", pInfo.str_Serial_Num[10]);
		printf("%c", pInfo.str_Serial_Num[11]);
		printf("%c", pInfo.str_Serial_Num[12]);
		printf("%c", pInfo.str_Serial_Num[13]);
		printf("%c", pInfo.str_Serial_Num[14]);
		printf("%c", pInfo.str_Serial_Num[15]);
		printf("%c", pInfo.str_Serial_Num[16]);
		printf("%c", pInfo.str_Serial_Num[17]);
		printf("%c", pInfo.str_Serial_Num[18]);
		printf("%c", pInfo.str_Serial_Num[19]);printf("\n");

		printf(">>hw_Type:%c", pInfo.str_hw_Type[0]);
		printf("%c", pInfo.str_hw_Type[1]);
		printf("%c", pInfo.str_hw_Type[2]);
		printf("%c", pInfo.str_hw_Type[3]);
		printf("%c", pInfo.str_hw_Type[4]);
		printf("%c", pInfo.str_hw_Type[5]);
		printf("%c", pInfo.str_hw_Type[6]);
		printf("%c", pInfo.str_hw_Type[7]);
		printf("%c", pInfo.str_hw_Type[8]);
		printf("%c", pInfo.str_hw_Type[9]);printf("\n");

		printf(">>Firmware Version:V");
		printf("%x", (pInfo.fw_Version&0xF00)>>8);
		printf(".");
		printf("%x", (pInfo.fw_Version&0xF0)>>4);
		printf("%x", pInfo.fw_Version&0xF);
		printf("\n");	
	}else
	{
		printf(">>Get VCI_ReadBoardInfo error!\n");
		exit(1);
	}

	//初始化参数，严格参数二次开发函数库说明书。
	VCI_INIT_CONFIG config;
	config.AccCode=0;
	config.AccMask=0xFFFFFFFF;
	config.Filter=1;//接收所有帧
	config.Timing0=0x00;/*波特率500 Kbps  0x00  0x1C*/
	config.Timing1=0x1C;
	config.Mode=0;//正常模式		
	
	if(VCI_InitCAN(VCI_USBCAN2,0,0,&config)!=1)
	{
		printf(">>Init CAN1 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);
	}

	if(VCI_StartCAN(VCI_USBCAN2,0,0)!=1)
	{
		printf(">>Start CAN1 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);

	}

	if(VCI_InitCAN(VCI_USBCAN2,0,1,&config)!=1)
	{
		printf(">>Init can2 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);

	}
	if(VCI_StartCAN(VCI_USBCAN2,0,1)!=1)
	{
		printf(">>Start can2 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);

	}


	//需要发送的帧，结构体设置
//	VCI_CAN_OBJ send[1];
//	send[0].ID=0;
//	send[0].SendType=0;
//	send[0].RemoteFlag=0;
//	send[0].ExternFlag=1;
//	send[0].DataLen=8;
	
//	int i=0;
//	for(i = 0; i < send[0].DataLen; i++)
//	{
//		send[0].Data[i] = i;
//	}


//	int times = 5;
//	while(times--)
//	{
//		if(VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1)
//		{
//			printf("Index:%04d  ",count);count++;
//			printf("CAN1 TX ID:0x%08X",send[0].ID);
//			if(send[0].ExternFlag==0) printf(" Standard ");
//			if(send[0].ExternFlag==1) printf(" Extend   ");
//			if(send[0].RemoteFlag==0) printf(" Data   ");
//			if(send[0].RemoteFlag==1) printf(" Remote ");
//			printf("DLC:0x%02X",send[0].DataLen);
//			printf(" data:0x");

//			for(i=0;i<send[0].DataLen;i++)
//			{
//				printf(" %02X",send[0].Data[i]);
//			}

//			printf("\n");
//			send[0].ID+=1;
//		}
//		else
//		{
//			break;
//		}
		
//		if(VCI_Transmit(VCI_USBCAN2, 0, 1, send, 1) == 1)
//		{
//			printf("Index:%04d  ",count);count++;
//			printf("CAN2 TX ID:0x%08X", send[0].ID);
//			if(send[0].ExternFlag==0) printf(" Standard ");
//			if(send[0].ExternFlag==1) printf(" Extend   ");
//			if(send[0].RemoteFlag==0) printf(" Data   ");
//			if(send[0].RemoteFlag==1) printf(" Remote ");
//			printf("DLC:0x%02X",send[0].DataLen);
//			printf(" data:0x");			
//			for(i = 0; i < send[0].DataLen; i++)
//			{
//				printf(" %02X", send[0].Data[i]);
//			}
//			printf("\n");
//			send[0].ID+=1;
//		}
//		else	break;
//	}
	//延时单位us，这里设置 10 000 000=10s    10s后关闭接收线程，并退出主程序。
//	m_run0=0;//线程关闭指令。
//	pthread_join(threadid,NULL);//等待线程关闭。
//	usleep(100000);//延时100ms。
//	VCI_ResetCAN(VCI_USBCAN2, 0, 0);//复位CAN1通道。
//	usleep(100000);//延时100ms。
//	VCI_ResetCAN(VCI_USBCAN2, 0, 1);//复位CAN2通道。
//	usleep(100000);//延时100ms。
//	VCI_CloseDevice(VCI_USBCAN2,0);//关闭设备。
	//除收发函数外，其它的函数调用前后，最好加个毫秒级的延时，即不影响程序的运行，又可以让USBCAN设备有充分的时间处理指令。
	//goto ext;
	while(ros::ok())
    	{
        	ars408::ClusterList clusterlist_msg;
         	
			data cluster_data_buffer[3000];
			int m_run0=1;
			int reclen=0;
			int ind=0;
			VCI_CAN_OBJ rec[3000];//接收缓存，设为3000为佳。
			int i,j;
			static int c_id=0;
			static int k;
	while((m_run0)&0x0f)
	{
		if((reclen=VCI_Receive(VCI_USBCAN2,0,ind,rec,3000,100))>0)//调用接收函数，如果有数据，进行数据处理显示。
		{
			for(j=0;j<reclen;j++)
			{
				printf("Index:%04d  ",count);count++;//序号递增
				printf("CAN%d RX ID:0x%08X", ind+1, rec[j].ID);//ID
//			if(rec[j].ID == 0x600) printf("CAN%d RX ID:0x%08X", ind+1, rec[j].ID);
				if(rec[j].ExternFlag==0) printf(" Standard ");//帧格式：标准帧
				if(rec[j].ExternFlag==1) printf(" Extend   ");//帧格式：扩展帧
				if(rec[j].RemoteFlag==0) printf(" Data   ");//帧类型：数据帧
				if(rec[j].RemoteFlag==1) printf(" Remote ");//帧类型：远程帧
				printf("DLC:0x%02X",rec[j].DataLen);//帧长度
				printf(" data:0x");	//数据
				if(rec[j].ID == 0x600)
				{
					int cluster_nearnum = rec[j].Data[0];
					int cluster_farnum = rec[j].Data[1];
				int cluster_meascounter = rec[j].Data[2]*16+rec[j].Data[3];
					int version = rec[j].Data[4] >> 4;
					printf(" %d\n", cluster_nearnum);
					printf(" %d\n", cluster_farnum);
					printf(" %d\n", cluster_meascounter);
					printf(" %d\n", version);
					cluster_data_buffer[c_id].num_cluster=cluster_nearnum+cluster_farnum;
					k=c_id;
					c_id++;	
				}
				if(rec[j].ID == 0x701)
				{
				//for(i = 0; i < rec[j].DataLen; i++)
				//{
					int cluster_ID = rec[j].Data[0];
					int cluster_x_label = (rec[j].Data[1] << 5)|(rec[j].Data[2] & 0xf8);
					int cluster_y_label = ((rec[j].Data[2] &0x03) << 8)|rec[j].Data[3];
					int cluster_x_velocity = (rec[j].Data[4] << 2) | (rec[j].Data[5] & 0xc0);
					int cluster_y_velocity = ((rec[j].Data[5] &0x3f) << 3) | (rec[j].Data[6] & 0xe0);
					int cluster_prop = rec[j].Data[6] & 0x07;
					int cluster_RCS = rec[j].Data[7];
					float x_label = cluster_x_label*0.2-500;
					float y_label = cluster_y_label*0.2-102.3; 
					float x_velocity = cluster_x_velocity *0.25 - 128;
					float y_velocity = cluster_y_velocity *0.25 - 64;
					float  RCS = cluster_RCS*0.5-64;
					printf(" %d\n", cluster_ID);
					printf(" %f\n", x_label);
					printf(" %f\n", y_label);
					printf(" %f\n", x_velocity);
					printf(" %f\n", y_velocity);
					printf(" %d\n", cluster_prop);
					printf(" %f\n", RCS);
					cluster_data_buffer[k].point_cloud.ID=cluster_ID;
					cluster_data_buffer[k].point_cloud.X_distance=x_label;
					cluster_data_buffer[k].point_cloud.Y_distance=y_label;
					cluster_data_buffer[k].point_cloud.Rad_distance=sqrt(x_label*x_label+y_label*y_label);
					if(x_velocity>=0) cluster_data_buffer[k].point_cloud.go_Sub_velocity=x_velocity;
					else cluster_data_buffer[k].point_cloud.in_Sub_velocity=fabs(x_velocity);
					if(y_velocity>=0) cluster_data_buffer[k].point_cloud.r_l_Sub_velocity=y_velocity;
					else cluster_data_buffer[k].point_cloud.l_r_Sub_velocity=fabs(y_velocity);
					if(x_velocity>=0)
					cluster_data_buffer[k].point_cloud.Rad_go_velocity=sqrt(x_velocity*x_velocity+y_velocity*y_velocity);
					else
					cluster_data_buffer[k].point_cloud.Rad_in_velocity=sqrt(x_velocity*x_velocity+y_velocity*y_velocity);
					cluster_data_buffer[k].point_cloud.RCS=RCS;
					cluster_data_buffer[k].point_cloud.Obj_size=pow(10,(RCS/10));
					//int dis = rec[j].Data[i];
					//printf(" %d", dis);
					//printf(" %d", rec[j].Data[i]);
					//printf(" %02X", rec[j].Data[i]);
				//}
				}
				ars408::Cluster cluster_rec;
				cluster_rec.id =cluster_data_buffer[k].point_cloud.ID;
				cluster_rec.X_distance=cluster_data_buffer[k].point_cloud.X_distance;
				cluster_rec.Y_distance=cluster_data_buffer[k].point_cloud.Y_distance;	
				cluster_rec.Rad_distance=cluster_data_buffer[k].point_cloud.Rad_distance;
				cluster_rec.Hor_angle=atan2(cluster_data_buffer[k].point_cloud.Y_distance,cluster_data_buffer[k].point_cloud.X_distance)*180/3.1416;
				cluster_rec.go_Sub_velocity=cluster_data_buffer[k].point_cloud.go_Sub_velocity;
				cluster_rec.in_Sub_velocity=cluster_data_buffer[k].point_cloud.in_Sub_velocity;
				cluster_rec.r_l_Sub_velocity=cluster_data_buffer[k].point_cloud.r_l_Sub_velocity;
				cluster_rec.l_r_Sub_velocity= cluster_data_buffer[k].point_cloud.l_r_Sub_velocity;
				cluster_rec.Rad_go_velocity=cluster_data_buffer[k].point_cloud.Rad_go_velocity;
				cluster_rec.Rad_in_velocity=cluster_data_buffer[k].point_cloud.Rad_in_velocity;
				cluster_rec.RCS=cluster_data_buffer[k].point_cloud.RCS;
				cluster_rec.Obj_size=cluster_data_buffer[k].point_cloud.Obj_size;
				clusterlist_msg.clusters.push_back(cluster_rec);
				printf(" %d\n", cluster_rec.id);
				//printf(" %f\n", cluster_data_buffer[k].point_cloud.X_distance);
				//printf(" %f\n", cluster_data_buffer[k].point_cloud.Y_distance);
				//printf(" %f\n", cluster_data_buffer[k].point_cloud.Rad_distance);
				//printf(" %f\n", cluster_data_buffer[k].point_cloud.RCS);
				//printf(" %f\n", cluster_data_buffer[k].point_cloud.Obj_size);
				//printf(" TimeStamp:0x%08X",rec[j].TimeStamp);//时间标识
				//printf("\n");
			}
		   		usleep(1000000);
				m_run0=0;
		}
				clusterlist_msg.header.stamp = ros::Time::now();
				clusterlist_msg.header.frame_id  =  k;
				//ROS_INFO("%f",clusterlist_msg.clusters);
          		pub.publish(clusterlist_msg);
		//ind=!ind;//变换通道号，以便下次读取另一通道，交替读取。		
	}
          	
	  	signal(SIGINT, mySigintHandler);
        ros::spinOnce();
        loop_rate.sleep();
    	}
	return 0;
}

