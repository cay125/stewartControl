#pragma once

#include <stddef.h>

#pragma pack(push)
#pragma pack(2)

#define GA_API  extern "C" __declspec(dllimport)

#define MAX_MACRO_CHAR_LENGTH (128)

//����ִ�з���ֵ
#define GA_COM_SUCCESS			        (0)	//ִ�гɹ�
#define GA_COM_ERR_EXEC_FAIL			(1)	//ִ��ʧ��
#define GA_COM_ERR_LICENSE_WRONG		(2)	//license��֧��
#define GA_COM_ERR_DATA_WORRY			(7)	//��������
#define GA_COM_ERR_SEND					(-1)//����ʧ��
#define GA_COM_ERR_CARD_OPEN_FAIL		(-6)//��ʧ��
#define GA_COM_ERR_TIME_OUT				(-7)//����Ӧ
#define GA_COM_ERR_COM_OPEN_FAIL        (-8)//�򿪴���ʧ��

//��״̬λ����
#define AXIS_STATUS_RESERVE0            (0x00000001)	//����
#define AXIS_STATUS_SV_ALARM            (0x00000002)	//������������־��1-�ŷ��б�����0-�ŷ��ޱ�����
#define AXIS_STATUS_POS_SOFT_LIMIT      (0x00000004)	//������λ������־���滮λ�ô�����������λʱ��1��
#define AXIS_STATUS_NEG_SOFT_LIMIT      (0x00000008)	//����λ������־���滮λ��С�ڸ�������λʱ��1��
#define	AXIS_STATUS_FOLLOW_ERR          (0x00000010)	//��滮λ�ú�ʵ��λ�õ��������趨����ʱ��1��
#define AXIS_STATUS_POS_HARD_LIMIT      (0x00000020)	//��Ӳ��λ������־������λ���ص�ƽ״̬Ϊ��λ������ƽʱ��1��
#define AXIS_STATUS_NEG_HARD_LIMIT      (0x00000040)	//��Ӳ��λ������־������λ���ص�ƽ״̬Ϊ��λ������ƽʱ��1��
#define AXIS_STATUS_IO_SMS_STOP         (0x00000080)	//IOƽ��ֹͣ������־������λ���ص�ƽ״̬Ϊ��λ������ƽʱ��1���滮λ�ô�����������λʱ��1��
#define AXIS_STATUS_IO_EMG_STOP         (0x00000100)	//IO����ֹͣ������־������λ���ص�ƽ״̬Ϊ��λ������ƽʱ��1���滮λ��С�ڸ�������λʱ��1��
#define AXIS_STATUS_ENABLE              (0x00000200)	//���ʹ�ܱ�־
#define	AXIS_STATUS_RUNNING             (0x00000400)	//�滮�˶���־���滮���˶�ʱ��1
#define AXIS_STATUS_ARRIVE              (0x00000800)	//�����λ���滮����ֹ���滮λ�ú�ʵ��λ�õ����С���趨���������������ڱ����趨ʱ�������λ��־��
#define AXIS_STATUS_HOME_RUNNING        (0x00001000)	//���ڻ���
#define AXIS_STATUS_HOME_SUCESS	        (0x00002000)	//����ɹ�
#define AXIS_STATUS_HOME_SWITCH			(0x00004000)	//��λ�ź�
#define AXIS_STATUS_INDEX				(0x00008000)    //z�����ź�

//����IO���ͺ궨��
#define MC_LIMIT_POSITIVE               0
#define MC_LIMIT_NEGATIVE               1
#define MC_ALARM                        2
#define MC_HOME                         3
#define MC_GPI                          4
#define MC_ARRIVE                       5
#define MC_IP_SWITCH                    6
#define MC_MPG                          7

//���IO���ͺ궨��
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

//���ٲ����������ͺ궨��
#define CAPTURE_HOME                    1
#define CAPTURE_INDEX                   2
#define CAPTURE_PROBE1                  3
#define CAPTURE_PROBE2                  4

//PTģʽ�궨��
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

#define GEAR_EVENT_START                1
#define GEAR_EVENT_PASS                 2
#define GEAR_EVENT_AREA                 5

#define FOLLOW_SEGMENT_NORMAL           0
#define FOLLOW_SEGMENT_EVEN             1
#define FOLLOW_SEGMENT_STOP             2
#define FOLLOW_SEGMENT_CONTINUE         3

#define FROCAST_LEN (200)                     //ǰհ���������

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

//��λģʽ�����ṹ��
typedef struct TrapPrm
{
	double acc;
	double dec;
	double velStart;
	short  smoothTime;
}TTrapPrm;

//JOGģʽ�����ṹ��
typedef struct JogPrm
{
	double dAcc;
	double dDec;
	double dSmooth;
}TJogPrm;



//����ϵ�����ṹ��
typedef struct _CrdPrm
{
    short dimension;                              // ����ϵά��
    short profile[AXIS_MAX];                      // ����profile��������(��1��ʼ)
    double synVelMax;                             // ���ϳ��ٶ�
    double synAccMax;                             // ���ϳɼ��ٶ�
    short evenTime;                               // ��С����ʱ��
    short setOriginFlag;                          // ����ԭ������ֵ��־,0:Ĭ�ϵ�ǰ�滮λ��Ϊԭ��λ��;1:�û�ָ��ԭ��λ��
    long originPos[AXIS_MAX];                     // �û�ָ����ԭ��λ��
}TCrdPrm;

//��������
enum _CMD_TYPE
{
	CMD_G00=1,		//���ٶ�λ
	CMD_G01,		//ֱ�߲岹
	CMD_G02,		//˳Բ���岹
	CMD_G03,		//��Բ���岹
	CMD_G04,		//��ʱ,G04 P1000����ͣ1��(��λΪms),G04 X2.0����ͣ2��
	CMD_G05,		//�����Զ���岹�ζκ�
	CMD_G54,

	CMD_M00 = 11,  //��ͣ
	CMD_M30,        //����
	CMD_M31,        //�л���XY1Z����ϵ
	CMD_M32,        //�л���XY2Z����ϵ
	CMD_M99,        //ѭ��

	CMD_SET_IO = 101,     //����IO
	CMD_WAIT_IO,           //�ȴ�IO
	CMD_IF_GOTO,           //IF GOTO���
};


//G00(���ٶ�λ)�������
struct _G00PARA{
	float synVel; //�岹�κϳ��ٶ�
	float synAcc; //�岹�κϳɼ��ٶ�
    long lX;       //X�ᵽ��λ�þ���λ��(��λ��pluse)
    long lY;       //Y�ᵽ��λ�þ���λ��(��λ��pluse)
    long lZ;       //Z�ᵽ��λ�þ���λ��(��λ��pluse)
    long lA;       //A�ᵽ��λ�þ���λ��(��λ��pluse)
   
	unsigned char iDimension; //����岹��������
	long segNum;
};
//G01(ֱ�߲岹)�������(����2��3�ᣬ��λ����֤)
struct _G01PARA{
	float synVel;    //�岹�κϳ��ٶ�
	float synAcc;    //�岹�κϳɼ��ٶ�
	float velEnd;   //�岹�ε��յ��ٶ�
    long lX;       //X�ᵽ��λ�þ���λ��(��λ��pluse)
    long lY;       //Y�ᵽ��λ�þ���λ��(��λ��pluse)
    long lZ;       //Z�ᵽ��λ�þ���λ��(��λ��pluse)
    long lA;       //A�ᵽ��λ�þ���λ��(��λ��pluse)
    
	long segNum;

	unsigned char iDimension; //����岹��������
	unsigned char iPreciseStopFlag;   //��׼��λ��־λ�����Ϊ1���յ㰴���յ�������
    
};

//G02_G03(˳Բ���岹)�������(����2�ᣬ��λ����֤)
struct _G02_3PARA{
	float synVel;    //�岹�κϳ��ٶ�
	float synAcc;    //�岹�κϳɼ��ٶ�
	float velEnd;   //�岹�ε��յ��ٶ�
    int iPlaneSelect;       //ƽ��ѡ��0��XYƽ�� 1��XZƽ�� 2��YZƽ��
    int iEnd1;              //��һ���յ����꣨��λum��
    int iEnd2;              //�ڶ����յ����꣨��λum��
    int iI;                 //Բ�����꣨��λum��(��������)
    int iJ;                 //Բ�����꣨��λum��(��������)
	long segNum;
    unsigned char iPreciseStopFlag;   //��׼��λ��־λ�����Ϊ1���յ㰴���յ�������
};

//G04��ʱ
struct _G04PARA{
unsigned long ulDelayTime;       //��ʱʱ��,��λMS
long segNum;
};

//G05�����û��Զ���κ�
struct _G05PARA{
long lUserSegNum;       //�û��Զ���κ�
};

//SetIO��������IO
struct _SetIOPara{
	unsigned short nCarkIndex;  //�忨������0����������1������չ��1��2������չ��2......��������
	unsigned short nDoMask;
	unsigned short nDoValue;
	long lUserSegNum;
};

//G�������
union _CMDPara{
    struct _G00PARA     G00PARA;
    struct _G01PARA     G01PARA;
    struct _G02_3PARA   G02_3PARA;
    struct _G04PARA     G04PARA;
    struct _G05PARA     G05PARA;
	struct _SetIOPara   SetIOPara;
};

//ÿһ�г���ṹ��
typedef struct _CrdData{
    unsigned char CMDType;              //ָ�����ͣ�֧�����255��ָ��0��GOO 1��G01 2��G02 FF:�ļ�����
    union _CMDPara CMDPara;         //ָ���������ͬ�����Ӧ��ͬ����
}TCrdData;

//ǰհ�����ṹ��
typedef struct _LookAheadPrm
{
	int lookAheadNum;                               //ǰհ����
	TCrdData *pLookAheadBuf;                        //ǰհ������ָ��
	double dSpeedMax[INTERPOLATION_AXIS_MAX];	    //���������ٶ�(p/ms)
	double dAccMax[INTERPOLATION_AXIS_MAX];			//����������ٶ�
	double dMaxStepSpeed[INTERPOLATION_AXIS_MAX];   //���������ٶȱ仯�����൱�������ٶȣ�
	double dScale[INTERPOLATION_AXIS_MAX];			//��������嵱��
}TLookAheadPrm;

//�岹�����ݲ�����Ϣ�ṹ��
typedef struct CrdBufOperation
{
    short flag;                                   // ��־�ò岹���Ƿ���IO����ʱ
    unsigned short delay;                         // ��ʱʱ��
    short doType;                                 // ������IO������,0:�����IO
    unsigned short doMask;                        // ������IO�������������
    unsigned short doValue;                       // ������IO�����ֵ
    unsigned short dataExt[CRD_OPERATION_DATA_EXT_MAX];     // ����������չ����
}TCrdBufOperation;

//��������
typedef struct _AxisHomeParm{
	short		nHomeMode;					//���㷽ʽ��0--�� 1--HOME��ԭ��	2--HOME��Index��ԭ��3----Z����	
	short		nHomeDir;					//���㷽��1-������㣬0-�������
	long        lOffset;                    //����ƫ�ƣ��ص���λ������һ��Offset��Ϊ��λ

	double		dHomeRapidVel;			    //��������ٶȣ���λ��Pluse/ms
	double		dHomeLocatVel;			    //���㶨λ�ٶȣ���λ��Pluse/ms
	double		dHomeIndexVel;			    //����Ѱ��INDEX�ٶȣ���λ��Pluse/ms
	double      dHomeAcc;                   //����ʹ�õļ��ٶ�

}TAxisHomePrm;

//����ָ���б�
GA_API int GA_Open(short iType,char* cName);
GA_API int GA_Close(void);
GA_API int GA_SetCardNo(short iCardNum);
GA_API int GA_GetCardNo(short *pCardNum);
GA_API int GA_Reset();
GA_API int GA_GetVersion(char *pVersion);
GA_API int GA_SetPrfPos(short profile,long prfPos);
GA_API int GA_SynchAxisPos(long mask);
GA_API int GA_ZeroPos(short nAxisNum,short nCount=1);

//ϵͳ������Ϣ
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
GA_API int GA_EncOn(short nEncoder);
GA_API int GA_EncOff(short nEncoder);
GA_API int GA_GetEncOnOff(short nAxisNum,short *pEncOnOff);
GA_API int GA_SetPosErr(short nAxisNum,long lError);
GA_API int GA_GetPosErr(short nAxisNum,long *pError);
GA_API int GA_SetStopDec(short nAxisNum,double decSmoothStop,double decAbruptStop);
GA_API int GA_GetStopDec(short nAxisNum,double *pDecSmoothStop,double *pDecAbruptStop);
GA_API int GA_CtrlMode(short nAxisNum,short mode);
GA_API int GA_GetCtrlMode(short nAxisNum,short *pMode);
GA_API int GA_SetStopIo(short nAxisNum,short stopType,short inputType,short inputIndex);

//�˶�״̬���ָ���б�
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


//��λ�˶�ָ���б�������λ���ٶ�ģʽ��
GA_API int GA_PrfTrap(short nAxisNum);
GA_API int GA_SetTrapPrm(short nAxisNum,TTrapPrm *pPrm);
GA_API int GA_GetTrapPrm(short nAxisNum,TTrapPrm *pPrm);
GA_API int GA_PrfJog(short nAxisNum);
GA_API int GA_SetJogPrm(short nAxisNum,TJogPrm *pPrm);
GA_API int GA_GetJogPrm(short nAxisNum,TJogPrm *pPrm);
GA_API int GA_SetPos(short nAxisNum,long pos);
GA_API int GA_GetPos(short nAxisNum,long *pPos);
GA_API int GA_SetVel(short nAxisNum,double vel);
GA_API int GA_GetVel(short nAxisNum,double *pVel);
GA_API int GA_Update(long mask);

//�岹�˶�ģʽָ���б�
GA_API int GA_SetCrdPrm(short nCrdNum,TCrdPrm *pCrdPrm);
GA_API int GA_GetCrdPrm(short nCrdNum,TCrdPrm *pCrdPrm);
GA_API int GA_InitLookAhead(short nCrdNum,short fifo,TLookAheadPrm* plookAheadPara);
GA_API int GA_CrdClear(short nCrdNum,short fifo);
GA_API int GA_LnXY(short nCrdNum,long x,long y,double synVel,double synAcc,double velEnd=0,short fifo=0,long segNum = 0);
GA_API int GA_LnXYZ(short nCrdNum,long x,long y,long z,double synVel,double synAcc,double velEnd=0,short fifo=0,long segNum = 0);
GA_API int GA_ArcXYC(short nCrdNum,long x,long y,double xCenter,double yCenter,short circleDir,double synVel,double synAcc,double velEnd=0,short fifo=0,long segNum = 0);
GA_API int GA_ArcXZC(short nCrdNum,long x,long z,double xCenter,double zCenter,short circleDir,double synVel,double synAcc,double velEnd=0,short fifo=0,long segNum = 0);
GA_API int GA_ArcYZC(short nCrdNum,long y,long z,double yCenter,double zCenter,short circleDir,double synVel,double synAcc,double velEnd=0,short fifo=0,long segNum = 0);
GA_API int GA_BufIO(short nCrdNum,unsigned short doType,unsigned short nCardIndex,unsigned short doMask,unsigned short doValue,short fifo=0,long segNum = 0);
GA_API int GA_BufDelay(short nCrdNum,unsigned long ulDelayTime,short fifo=0,long segNum = 0);
GA_API int GA_CrdData(short nCrdNum,void *pCrdData,short fifo=0);
GA_API int GA_CrdStart(short mask,short option);
GA_API int GA_SetOverride(short nCrdNum,double synVelRatio);
GA_API int GA_GetCrdPos(short nCrdNum,double *pPos);
GA_API int GA_CrdSpace(short nCrdNum,long *pSpace,short fifo=0);
GA_API int GA_CrdStatus(short nCrdNum,short *pRun,long *pSegment,short fifo=0);
GA_API int GA_SetUserSegNum(short nCrdNum,long segNum,short fifo=0);
GA_API int GA_GetUserSegNum(short nCrdNum,long *pSegment,short fifo=0);
GA_API int GA_GetRemainderSegNum(short nCrdNum,long *pSegment,short fifo=0);
GA_API int GA_GetLookAheadSpace(short nCrdNum,long *pSpace,short nFifoIndex=0);

//����Ӳ����Դָ���б�
GA_API int GA_GetDi(short diType,long *pValue);
GA_API int GA_GetDiRaw(short diType,long *pValue);
GA_API int GA_SetDo(short doType,long value);
GA_API int GA_SetDoBit(short doType,short nDoNum,short value);
GA_API int GA_SetDoBitReverse(short doType,short nDoNum,long value,short reverseTime);
GA_API int GA_GetDo(short doType,long *pValue);
GA_API int GA_GetEncPos(short encoder,double *pValue,short nCount=1,unsigned long *pClock=NULL);
GA_API int GA_GetEncVel(short encoder,double *pValue,short nCount=1,unsigned long *pClock=NULL);
GA_API int GA_SetEncPos(short encoder,long encPos);
GA_API int GA_SetExtDoValue(short nCardIndex,unsigned long *value,short nCount=1);
GA_API int GA_GetExtDiValue(short nCardIndex,unsigned long *pValue,short nCount=1);
GA_API int GA_GetExtDoValue(short nCardIndex,unsigned long *pValue,short nCount=1);
GA_API int GA_SetExtDoBit(short nCardIndex,short nBitIndex,unsigned short nValue);
GA_API int GA_GetExtDiBit(short nCardIndex,short nBitIndex,unsigned short *pValue);
GA_API int GA_GetExtDoBit(short nCardIndex,short nBitIndex,unsigned short *pValue);

//����Ӳ������ָ���б�
GA_API int GA_SetCaptureMode(short encoder,short mode);
GA_API int GA_GetCaptureMode(short encoder,short *pMode,short nCount=1);
GA_API int GA_GetCaptureStatus(short encoder,short *pStatus,long *pValue,short nCount=1,unsigned long *pClock=NULL);
GA_API int GA_SetCaptureSense(short encoder,short mode,short sense);
GA_API int GA_GetCaptureSense(short encoder,short mode,short *sense);
GA_API int GA_ClearCaptureStatus(short encoder);

//��ȫ����ָ���б�
GA_API int GA_SetSoftLimit(short nAxisNum,long positive,long negative);
GA_API int GA_GetSoftLimit(short nAxisNum,long *pPositive,long *pNegative);


#pragma pack(pop)