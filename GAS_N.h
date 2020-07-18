#pragma once

#include <stddef.h>

#pragma pack(push)
#pragma pack(2)

#define GA_API  extern "C" __declspec(dllimport)

#define MAX_MACRO_CHAR_LENGTH (128)

//函数执行返回值
#define GA_COM_SUCCESS			        (0)	//执行成功
#define GA_COM_ERR_EXEC_FAIL			(1)	//执行失败
#define GA_COM_ERR_LICENSE_WRONG		(2)	//license不支持
#define GA_COM_ERR_DATA_WORRY			(7)	//参数错误
#define GA_COM_ERR_SEND					(-1)//发送失败
#define GA_COM_ERR_CARD_OPEN_FAIL		(-6)//打开失败
#define GA_COM_ERR_TIME_OUT				(-7)//无响应
#define GA_COM_ERR_COM_OPEN_FAIL        (-8)//打开串口失败

//轴状态位定义
#define AXIS_STATUS_RESERVE0            (0x00000001)	//保留
#define AXIS_STATUS_SV_ALARM            (0x00000002)	//驱动器报警标志（1-伺服有报警，0-伺服无报警）
#define AXIS_STATUS_POS_SOFT_LIMIT      (0x00000004)	//正软限位触发标志（规划位置大于正向软限位时置1）
#define AXIS_STATUS_NEG_SOFT_LIMIT      (0x00000008)	//负软位触发标志（规划位置小于负向软限位时置1）
#define	AXIS_STATUS_FOLLOW_ERR          (0x00000010)	//轴规划位置和实际位置的误差大于设定极限时置1┅
#define AXIS_STATUS_POS_HARD_LIMIT      (0x00000020)	//正硬限位触发标志（正限位开关电平状态为限位触发电平时置1）
#define AXIS_STATUS_NEG_HARD_LIMIT      (0x00000040)	//负硬限位触发标志（负限位开关电平状态为限位触发电平时置1）
#define AXIS_STATUS_IO_SMS_STOP         (0x00000080)	//IO平滑停止触发标志（正限位开关电平状态为限位触发电平时置1，规划位置大于正向软限位时置1）
#define AXIS_STATUS_IO_EMG_STOP         (0x00000100)	//IO紧急停止触发标志（负限位开关电平状态为限位触发电平时置1，规划位置小于负向软限位时置1）
#define AXIS_STATUS_ENABLE              (0x00000200)	//电机使能标志
#define	AXIS_STATUS_RUNNING             (0x00000400)	//规划运动标志，规划器运动时置1
#define AXIS_STATUS_ARRIVE              (0x00000800)	//电机到位（规划器静止，规划位置和实际位置的误差小于设定误差带，并在误差带内保持设定时间后，置起到位标志）
#define AXIS_STATUS_HOME_RUNNING        (0x00001000)	//正在回零
#define AXIS_STATUS_HOME_SUCESS	        (0x00002000)	//回零成功
#define AXIS_STATUS_HOME_SWITCH			(0x00004000)	//零位信号
#define AXIS_STATUS_INDEX				(0x00008000)    //z索引信号
#define AXIS_STATUS_GEAR_START  		(0x00010000)    //电子齿轮开始啮合
#define AXIS_STATUS_GEAR_FINISH         (0x00020000)    //电子齿轮完成啮合

//输入IO类型宏定义
#define MC_LIMIT_POSITIVE               0
#define MC_LIMIT_NEGATIVE               1
#define MC_ALARM                        2
#define MC_HOME                         3
#define MC_GPI                          4
#define MC_ARRIVE                       5
#define MC_IP_SWITCH                    6
#define MC_MPG                          7

//输出IO类型宏定义
#define MC_ENABLE                       10
#define MC_CLEAR                        11
#define MC_GPO                          12

#define MC_DAC                          20
#define MC_STEP                         21
#define MC_PULSE                        22
#define MC_ENCODER                      23
#define MC_ADC                          24

#define MC_AXIS                         30
#define MC_PROFILE                      31
#define MC_CONTROL                      32

//高速捕获输入类型宏定义
#define CAPTURE_HOME                    1
#define CAPTURE_INDEX                   2
#define CAPTURE_PROBE1                  3
#define CAPTURE_PROBE2                  4

//PT模式宏定义
#define PT_MODE_STATIC                  0
#define PT_MODE_DYNAMIC                 1

#define PT_SEGMENT_NORMAL               0
#define PT_SEGMENT_EVEN                 1
#define PT_SEGMENT_STOP                 2

#define GEAR_MASTER_ENCODER             1
#define GEAR_MASTER_PROFILE             2
#define GEAR_MASTER_AXIS                3

#define FOLLOW_MASTER_ENCODER           1
#define FOLLOW_MASTER_PROFILE           2
#define FOLLOW_MASTER_AXIS              3

#define FOLLOW_EVENT_START              1
#define FOLLOW_EVENT_PASS               2

//电子齿轮启动事件定义
#define GEAR_EVENT_IMMED                1//立即启动电子齿轮
#define GEAR_EVENT_BIG_EQU              2//主轴规划或者编码器位置大于等于指定数值时启动电子齿轮
#define GEAR_EVENT_SMALL_EQU            3//主轴规划或者编码器位置小于等于指定数值时启动电子齿轮
#define GEAR_EVENT_IO_ON                4//指定IO为ON时启动电子齿轮
#define GEAR_EVENT_IO_OFF               5//指定IO为OFF时启动电子齿轮

#define FOLLOW_SEGMENT_NORMAL           0
#define FOLLOW_SEGMENT_EVEN             1
#define FOLLOW_SEGMENT_STOP             2
#define FOLLOW_SEGMENT_CONTINUE         3

#define FROCAST_LEN (200)                     //前瞻缓冲区深度

#define INTERPOLATION_AXIS_MAX          6
#define CRD_FIFO_MAX                    4096
#define CRD_MAX                         2
#define CRD_OPERATION_DATA_EXT_MAX      2

#define CRD_OPERATION_TYPE_NONE         0
#define CRD_OPERATION_TYPE_BUF_IO_DELAY 1
#define CRD_OPERATION_TYPE_LASER_ON     2
#define CRD_OPERATION_TYPE_LASER_OFF    3
#define CRD_OPERATION_TYPE_BUF_DA       4
#define CRD_OPERATION_TYPE_LASER_CMD    5
#define CRD_OPERATION_TYPE_LASER_FOLLOW 6
#define CRD_OPERATION_TYPE_LMTS_ON      7
#define CRD_OPERATION_TYPE_LMTS_OFF     8
#define CRD_OPERATION_TYPE_SET_STOP_IO  9
#define CRD_OPERATION_TYPE_BUF_MOVE     10
#define CRD_OPERATION_TYPE_BUF_GEAR     11
#define CRD_OPERATION_TYPE_SET_SEG_NUM  12
#define CRD_OPERATION_TYPE_STOP_MOTION  13
#define CRD_OPERATION_TYPE_SET_VAR_VALUE 14
#define CRD_OPERATION_TYPE_JUMP_NEXT_SEG 15
#define CRD_OPERATION_TYPE_SYNCH_PRF_POS 16
#define CRD_OPERATION_TYPE_VIRTUAL_TO_ACTUAL 17
#define CRD_OPERATION_TYPE_SET_USER_VAR  18

#define INTERPOLATION_MOTION_TYPE_LINE  0
#define INTERPOLATION_MOTION_TYPE_CIRCLE 1
#define INTERPOLATION_MOTION_TYPE_HELIX 2

#define INTERPOLATION_CIRCLE_PLAT_XY    0
#define INTERPOLATION_CIRCLE_PLAT_YZ    1
#define INTERPOLATION_CIRCLE_PLAT_ZX    2

#define INTERPOLATION_HELIX_CIRCLE_XY_LINE_Z    0
#define INTERPOLATION_HELIX_CIRCLE_YZ_LINE_X    1
#define INTERPOLATION_HELIX_CIRCLE_ZX_LINE_Y    2

#define INTERPOLATION_CIRCLE_DIR_CW     0
#define INTERPOLATION_CIRCLE_DIR_CCW    1

#pragma once

#define RES_LIMIT                       8
#define RES_ALARM                       8
#define RES_HOME                        8
#define RES_GPI                         16
#define RES_ARRIVE                      8

#define RES_ENABLE                      8
#define RES_CLEAR                       8
#define RES_GPO                         16

#define RES_DAC                         12
#define RES_STEP                        8
#define RES_PULSE                       8
#define RES_ENCODER                     10

#define AXIS_MAX                        8
#define PROFILE_MAX                     8
#define CONTROL_MAX                     8

#define PRF_MAP_MAX                     2
#define ENC_MAP_MAX                     2

#define STEP_DIR                        0
#define STEP_PULSE                      1

//点位模式参数结构体
typedef struct TrapPrm
{
	double acc;
	double dec;
	double velStart;
	short  smoothTime;
}TTrapPrm;

//JOG模式参数结构体
typedef struct JogPrm
{
	double dAcc;
	double dDec;
	double dSmooth;
}TJogPrm;



//坐标系参数结构体
typedef struct _CrdPrm
{
    short dimension;                              // 坐标系维数
    short profile[8];                      // 关联profile和坐标轴(从1开始)
    double synVelMax;                             // 最大合成速度
    double synAccMax;                             // 最大合成加速度
    short evenTime;                               // 最小匀速时间
    short setOriginFlag;                          // 设置原点坐标值标志,0:默认当前规划位置为原点位置;1:用户指定原点位置
    long originPos[8];                     // 用户指定的原点位置
}TCrdPrm;

//命令类型
enum _CMD_TYPE
{
	CMD_G00=1,		//快速定位
	CMD_G01,		//直线插补
	CMD_G02,		//顺圆弧插补
	CMD_G03,		//逆圆弧插补
	CMD_G04,		//延时,G04 P1000是暂停1秒(单位为ms),G04 X2.0是暂停2秒
	CMD_G05,		//设置自定义插补段段号
	CMD_G54,

	CMD_M00 = 11,  //暂停
	CMD_M30,        //结束
	CMD_M31,        //切换到XY1Z坐标系
	CMD_M32,        //切换到XY2Z坐标系
	CMD_M99,        //循环

	CMD_SET_IO = 101,     //设置IO
	CMD_WAIT_IO,           //等待IO
	CMD_IF_GOTO,           //IF GOTO语句
};


//G00(快速定位)命令参数
struct _G00PARA{
	float synVel; //插补段合成速度
	float synAcc; //插补段合成加速度
    long lX;       //X轴到达位置绝对位置(单位：pluse)
    long lY;       //Y轴到达位置绝对位置(单位：pluse)
    long lZ;       //Z轴到达位置绝对位置(单位：pluse)
    long lA;       //A轴到达位置绝对位置(单位：pluse)
   
	unsigned char iDimension; //参与插补的轴数量
	long segNum;
};
//G01(直线插补)命令参数(任意2到3轴，上位机保证)
struct _G01PARA{
	float synVel;    //插补段合成速度
	float synAcc;    //插补段合成加速度
	float velEnd;   //插补段的终点速度
    long lX;       //X轴到达位置绝对位置(单位：pluse)
    long lY;       //Y轴到达位置绝对位置(单位：pluse)
    long lZ;       //Z轴到达位置绝对位置(单位：pluse)
    long lA;       //A轴到达位置绝对位置(单位：pluse)
    
	long segNum;

	unsigned char iDimension; //参与插补的轴数量
	unsigned char iPreciseStopFlag;   //精准定位标志位，如果为1，终点按照终点坐标来
    
};

//G02_G03(顺圆弧插补)命令参数(任意2轴，上位机保证)
struct _G02_3PARA{
	float synVel;    //插补段合成速度
	float synAcc;    //插补段合成加速度
	float velEnd;   //插补段的终点速度
    int iPlaneSelect;       //平面选择0：XY平面 1：XZ平面 2：YZ平面
    int iEnd1;              //第一轴终点坐标（单位um）
    int iEnd2;              //第二轴终点坐标（单位um）
    int iI;                 //圆心坐标（单位um）(相对于起点)
    int iJ;                 //圆心坐标（单位um）(相对于起点)
	long segNum;
    unsigned char iPreciseStopFlag;   //精准定位标志位，如果为1，终点按照终点坐标来
};

//G04延时
struct _G04PARA{
unsigned long ulDelayTime;       //延时时间,单位MS
long segNum;
};

//G05设置用户自定义段号
struct _G05PARA{
long lUserSegNum;       //用户自定义段号
};

//SetIO设置物理IO
struct _SetIOPara{
	unsigned short nCarkIndex;  //板卡索引，0代表主卡，1代表扩展卡1，2代表扩展卡2......依次类推
	unsigned short nDoMask;
	unsigned short nDoValue;
	long lUserSegNum;
};

//G代码参数
union _CMDPara{
    struct _G00PARA     G00PARA;
    struct _G01PARA     G01PARA;
    struct _G02_3PARA   G02_3PARA;
    struct _G04PARA     G04PARA;
    struct _G05PARA     G05PARA;
	struct _SetIOPara   SetIOPara;
};

//每一行程序结构体
typedef struct _CrdData{
    unsigned char CMDType;              //指令类型，支持最多255种指令0：GOO 1：G01 2：G02 FF:文件结束
    union _CMDPara CMDPara;         //指令参数，不同命令对应不同参数
}TCrdData;

//前瞻参数结构体
typedef struct _LookAheadPrm
{
	int lookAheadNum;                               //前瞻段数
	TCrdData *pLookAheadBuf;                        //前瞻缓冲区指针
	double dSpeedMax[INTERPOLATION_AXIS_MAX];	    //各轴的最大速度(p/ms)
	double dAccMax[INTERPOLATION_AXIS_MAX];			//各轴的最大加速度
	double dMaxStepSpeed[INTERPOLATION_AXIS_MAX];   //各轴的最大速度变化量（相当于启动速度）
	double dScale[INTERPOLATION_AXIS_MAX];			//各轴的脉冲当量
}TLookAheadPrm;

//插补段数据操作信息结构体
typedef struct CrdBufOperation
{
    short flag;                                   // 标志该插补段是否含有IO和延时
    unsigned short delay;                         // 延时时间
    short nDoType;                                 // 缓存区IO的类型,0:不输出IO
    unsigned short doMask;                        // 缓存区IO的输出控制掩码
    unsigned short doValue;                       // 缓存区IO的输出值
    unsigned short dataExt[CRD_OPERATION_DATA_EXT_MAX];     // 辅助操作扩展数据
}TCrdBufOperation;

//轴回零参数
typedef struct _AxisHomeParm{
	short		nHomeMode;					//回零方式：0--无 1--HOME回原点	2--HOME加Index回原点3----Z脉冲	
	short		nHomeDir;					//回零方向，1-正向回零，0-负向回零
	long        lOffset;                    //回零偏移，回到零位后再走一个Offset作为零位

	double		dHomeRapidVel;			    //回零快移速度，单位：Pluse/ms
	double		dHomeLocatVel;			    //回零定位速度，单位：Pluse/ms
	double		dHomeIndexVel;			    //回零寻找INDEX速度，单位：Pluse/ms
	double      dHomeAcc;                   //回零使用的加速度

}TAxisHomePrm;

//其他指令列表
GA_API int GA_Open(short iType,char* cName);
GA_API int GA_Close(void);
GA_API int GA_SetCardNo(short iCardNum);
GA_API int GA_GetCardNo(short *pCardNum);
GA_API int GA_Reset();
GA_API int GA_GetVersion(char *pVersion);
GA_API int GA_SetPrfPos(short profile,long prfPos);
GA_API int GA_SynchAxisPos(long mask);
GA_API int GA_ZeroPos(short nAxisNum,short nCount=1);
GA_API int GA_SetAxisBand(short nAxisNum,long lBand,long lTime);
GA_API int GA_GetAxisBand(short nAxisNum,long *pBand,long *pTime);

//系统配置信息
GA_API int GA_AlarmOn(short nAxisNum);
GA_API int GA_AlarmOff(short nAxisNum);
GA_API int GA_GetAlarmOnOff(short nAxisNum,short *pAlarmOnOff);
GA_API int GA_AlarmSns(unsigned short nSense);
GA_API int GA_GetAlarmSns(short *pSense);
GA_API int GA_LmtsOn(short nAxisNum,short limitType=-1);
GA_API int GA_LmtsOff(short nAxisNum,short limitType=-1);
GA_API int GA_GetLmtsOnOff(short nAxisNum,short *pPosLmtsOnOff, short *pNegLmtsOnOff);
GA_API int GA_LmtSns(unsigned short nSense);
GA_API int GA_GetLmtSns(unsigned short *pSense);
GA_API int GA_ProfileScale(short nAxisNum,short alpha,short beta);
GA_API int GA_EncScale(short nAxisNum,short alpha,short beta);
GA_API int GA_StepDir(short step);
GA_API int GA_StepPulse(short step);
GA_API int GA_GetStep(short nAxisNum,short *pStep);
GA_API int GA_StepSns(unsigned short sense);
GA_API int GA_GetStepSns(short *pSense);
GA_API int GA_EncSns(unsigned short sense);
GA_API int GA_GetEncSns(short *pSense);
GA_API int GA_EncOn(short nEncoderNum);
GA_API int GA_EncOff(short nEncoderNum);
GA_API int GA_GetEncOnOff(short nAxisNum,short *pEncOnOff);
GA_API int GA_SetPosErr(short nAxisNum,long lError);
GA_API int GA_GetPosErr(short nAxisNum,long *pError);
GA_API int GA_SetStopDec(short nAxisNum,double decSmoothStop,double decAbruptStop);
GA_API int GA_GetStopDec(short nAxisNum,double *pDecSmoothStop,double *pDecAbruptStop);
GA_API int GA_CtrlMode(short nAxisNum,short mode);
GA_API int GA_GetCtrlMode(short nAxisNum,short *pMode);
GA_API int GA_SetStopIo(short nAxisNum,short stopType,short inputType,short inputIndex);

//运动状态检测指令列表
GA_API int GA_GetSts(short nAxisNum,long *pSts,short nCount=1,unsigned long *pClock=NULL);
GA_API int GA_ClrSts(short nAxisNum,short nCount=1);
GA_API int GA_GetPrfMode(short profile,long *pValue,short nCount=1,unsigned long *pClock=NULL);
GA_API int GA_GetPrfPos(short nAxisNum,double *pValue,short nCount=1,unsigned long *pClock=NULL);
GA_API int GA_GetPrfVel(short nAxisNum,double *pValue,short nCount=1,unsigned long *pClock=NULL);
GA_API int GA_GetPrfAcc(short nAxisNum,double *pValue,short nCount=1,unsigned long *pClock=NULL);
GA_API int GA_GetAxisPrfPos(short nAxisNum,double *pValue,short nCount=1,unsigned long *pClock=NULL);
GA_API int GA_GetAxisPrfVel(short nAxisNum,double *pValue,short nCount=1,unsigned long *pClock=NULL);
GA_API int GA_GetAxisPrfAcc(short nAxisNum,double *pValue,short nCount=1,unsigned long *pClock=NULL);
GA_API int GA_GetAxisEncPos(short nAxisNum,double *pValue,short nCount=1,unsigned long *pClock=NULL);
GA_API int GA_GetAxisEncVel(short nAxisNum,double *pValue,short nCount=1,unsigned long *pClock=NULL);
GA_API int GA_GetAxisEncAcc(short nAxisNum,double *pValue,short nCount=1,unsigned long *pClock=NULL);
GA_API int GA_GetAxisError(short nAxisNum,double *pValue,short nCount=1,unsigned long *pClock=NULL);
GA_API int GA_Stop(long lMask,long lOption);
GA_API int GA_AxisOn(short nAxisNum);
GA_API int GA_AxisOff(short nAxisNum);


//点位运动指令列表（包括点位和速度模式）
GA_API int GA_PrfTrap(short nAxisNum);
GA_API int GA_SetTrapPrm(short nAxisNum,TTrapPrm *pPrm);
GA_API int GA_SetTrapPrmSingle(short nAxisNum,double dAcc,double dDec,double dVelStart,short  dSmoothTime);
GA_API int GA_GetTrapPrm(short nAxisNum,TTrapPrm *pPrm);
GA_API int GA_GetTrapPrmSingle(short nAxisNum,double* dAcc,double* dDec,double* dVelStart,short*  dSmoothTime);
GA_API int GA_PrfJog(short nAxisNum);
GA_API int GA_SetJogPrm(short nAxisNum,TJogPrm *pPrm);
GA_API int GA_SetJogPrmSingle(short nAxisNum,double dAcc,double dDec,double dSmooth);
GA_API int GA_GetJogPrm(short nAxisNum,TJogPrm *pPrm);
GA_API int GA_GetJogPrmSingle(short nAxisNum,double* dAcc,double* dDec,double* dSmooth);
GA_API int GA_SetPos(short nAxisNum,long pos);
GA_API int GA_GetPos(short nAxisNum,long *pPos);
GA_API int GA_SetVel(short nAxisNum,double vel);
GA_API int GA_GetVel(short nAxisNum,double *pVel);
GA_API int GA_SetMultiVel(short nAxisNum,double *pVel,short nCount=1);
GA_API int GA_SetMultiPos(short nAxisNum,long *pPos,short nCount=1);
GA_API int GA_Update(long mask);

//电子齿轮模式指令列表
GA_API int GA_PrfGear(short nAxisNum,short dir=0);
GA_API int GA_SetGearMaster(short nAxisNum,short nMasterAxisNum,short masterType=GEAR_MASTER_PROFILE);
GA_API int GA_GetGearMaster(short nAxisNum,short *nMasterAxisNum,short *pMasterType=NULL);
GA_API int GA_SetGearRatio(short nAxisNum,long masterEven,long slaveEven,long masterSlope=0,long lStopSmoothTime = 200);
GA_API int GA_GetGearRatio(short nAxisNum,long *pMasterEven,long *pSlaveEven,long *pMasterSlope=NULL,long *pStopSmoothTime=NULL);
GA_API int GA_GearStart(long mask);
GA_API int GA_GearStop(long mask);
GA_API int GA_SetGearEvent(short nAxisNum,short nEvent,double startPara0,double startPara1);
GA_API int GA_GetGearEvent(short nAxisNum,short *pEvent,double *pStartPara0,double *pStartPara1);

//PT模式指令列表
GA_API int GA_PrfPt(short nAxisNum,short mode=PT_MODE_STATIC);
GA_API int GA_PtSpace(short nAxisNum,long *pSpace,short nCount);
GA_API int GA_PtRemain(short nAxisNum,long *pRemainSpace,short nCount);
GA_API int GA_PtData(short nAxisNum,short* pData,long lLength,double dDataID);
GA_API int GA_PtClear(long lAxisMask);
GA_API int GA_PtStart(long lAxisMask);

//插补运动模式指令列表
GA_API int GA_SetCrdPrm(short nCrdNum,TCrdPrm *pCrdPrm);
GA_API int GA_GetCrdPrm(short nCrdNum,TCrdPrm *pCrdPrm);
GA_API int GA_InitLookAhead(short nCrdNum,short FifoIndex,TLookAheadPrm* plookAheadPara);
GA_API int GA_CrdClear(short nCrdNum,short FifoIndex);
GA_API int GA_LnXY(short nCrdNum,long x,long y,double synVel,double synAcc,double velEnd=0,short FifoIndex=0,long segNum = 0);
GA_API int GA_LnXYZ(short nCrdNum,long x,long y,long z,double synVel,double synAcc,double velEnd=0,short FifoIndex=0,long segNum = 0);
GA_API int GA_ArcXYC(short nCrdNum,long x,long y,double xCenter,double yCenter,short circleDir,double synVel,double synAcc,double velEnd=0,short FifoIndex=0,long segNum = 0);
GA_API int GA_ArcXZC(short nCrdNum,long x,long z,double xCenter,double zCenter,short circleDir,double synVel,double synAcc,double velEnd=0,short FifoIndex=0,long segNum = 0);
GA_API int GA_ArcYZC(short nCrdNum,long y,long z,double yCenter,double zCenter,short circleDir,double synVel,double synAcc,double velEnd=0,short FifoIndex=0,long segNum = 0);
GA_API int GA_BufIO(short nCrdNum,unsigned short nDoType,unsigned short nCardIndex,unsigned short doMask,unsigned short doValue,short FifoIndex=0,long segNum = 0);
GA_API int GA_BufDelay(short nCrdNum,unsigned long ulDelayTime,short FifoIndex=0,long segNum = 0);
GA_API int GA_CrdData(short nCrdNum,void *pCrdData,short FifoIndex=0);
GA_API int GA_CrdStart(short mask,short option);
GA_API int GA_SetOverride(short nCrdNum,double synVelRatio);
GA_API int GA_GetCrdPos(short nCrdNum,double *pPos);
GA_API int GA_CrdSpace(short nCrdNum,long *pSpace,short FifoIndex=0);
GA_API int GA_CrdStatus(short nCrdNum,short *pRun,long *pSegment,short FifoIndex=0);
GA_API int GA_SetUserSegNum(short nCrdNum,long segNum,short FifoIndex=0);
GA_API int GA_GetUserSegNum(short nCrdNum,long *pSegment,short FifoIndex=0);
GA_API int GA_GetRemainderSegNum(short nCrdNum,long *pSegment,short FifoIndex=0);
GA_API int GA_GetLookAheadSpace(short nCrdNum,long *pSpace,short nFifoIndex=0);

//访问硬件资源指令列表
GA_API int GA_GetDi(short nDiType,long *pValue);
GA_API int GA_GetDiRaw(short nDiType,long *pValue);
GA_API int GA_SetDo(short nDoType,long value);
GA_API int GA_SetDoBit(short nDoType,short nDoNum,short value);
GA_API int GA_SetDoBitReverse(short nDoType,short nDoNum,long value,short reverseTime);
GA_API int GA_GetDo(short nDoType,long *pValue);
GA_API int GA_GetEncPos(short nEncodeNum,double *pValue,short nCount=1,unsigned long *pClock=NULL);
GA_API int GA_GetEncVel(short nEncodeNum,double *pValue,short nCount=1,unsigned long *pClock=NULL);
GA_API int GA_SetEncPos(short nEncodeNum,long encPos);
GA_API int GA_SetExtDoValue(short nCardIndex,unsigned long *value,short nCount=1);
GA_API int GA_GetExtDiValue(short nCardIndex,unsigned long *pValue,short nCount=1);
GA_API int GA_GetExtDoValue(short nCardIndex,unsigned long *pValue,short nCount=1);
GA_API int GA_SetExtDoBit(short nCardIndex,short nBitIndex,unsigned short nValue);
GA_API int GA_GetExtDiBit(short nCardIndex,short nBitIndex,unsigned short *pValue);
GA_API int GA_GetExtDoBit(short nCardIndex,short nBitIndex,unsigned short *pValue);

//比较输出指令
GA_API int GA_CmpPluse(short nChannel, short nPluseType1, short nPluseType2, short nTime1,short nTime2);
GA_API int GA_CmpBufSetChannel(short nBuf1ChannelNum,short nBuf2ChannelNum);
GA_API int GA_CmpBufData(short nCmpEncodeNum, short nPluseType, short nStartLevel, short nTime, long *pBuf1, short nBufLen1, long *pBuf2, short nBufLen2,short nAbsPosFlag=0);
GA_API int GA_CmpBufSts(short *pStatus,unsigned short *pCount);
GA_API int GA_CmpBufStop(short nChannel);
GA_API int GA_CmpRpt(short nEncode, short nEncodeType,short nChannel,long lStartPos, long lRptTime, long lInterval, short nTime);


//高速硬件捕获指令列表
GA_API int GA_SetCaptureMode(short nEncodeNum,short mode);
GA_API int GA_GetCaptureMode(short nEncodeNum,short *pMode,short nCount=1);
GA_API int GA_GetCaptureStatus(short nEncodeNum,short *pStatus,long *pValue,short nCount=1,unsigned long *pClock=NULL);
GA_API int GA_SetCaptureSense(short nEncodeNum,short mode,short sense);
GA_API int GA_GetCaptureSense(short nEncodeNum,short mode,short *sense);
GA_API int GA_ClearCaptureStatus(short nEncodeNum);

//安全机制指令列表
GA_API int GA_SetSoftLimit(short nAxisNum,long lPositive,long lNegative);
GA_API int GA_GetSoftLimit(short nAxisNum,long *pPositive,long *pNegative);
GA_API int GA_SetHardLimP(short nAxisNum,short nType ,short nCardIndex,short nIOIndex);
GA_API int GA_SetHardLimN(short nAxisNum,short nType ,short nCardIndex,short nIOIndex);

//自动回零相关API
GA_API int GA_HomeStart(int iAxisNum);
GA_API int GA_HomeStop(int iAxisNum);
GA_API int GA_HomeSetPrm(int iAxisNum,TAxisHomePrm *pAxisHomePrm);
GA_API int GA_HomeSetPrmSingle(short iAxisNum,short nHomeMode,short nHomeDir,long lOffset,double dHomeRapidVel,double dHomeLocatVel,double dHomeIndexVel,double dHomeAcc);
GA_API int GA_HomeGetPrm(int iAxisNum,TAxisHomePrm *pAxisHomePrm);
GA_API int GA_HomeGetPrmSingle(short iAxisNum,short *nHomeMode,short *nHomeDir,long *lOffset,double* dHomeRapidVel,double* dHomeLocatVel,double* dHomeIndexVel,double* dHomeAcc);
GA_API int GA_HomeGetSts(int iAxisNum,unsigned short* pStatus);

#pragma pack(pop)