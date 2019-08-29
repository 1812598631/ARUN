/********************************************************************
*Copyright(C）2014-2016,沈阳艾克申机器人技术开发有限责任公司
*FileName：	   point2point.c
*Author：      Peng Xu
*Date：        2016/10/21
*Description：
*		       差速轮闭环直线函数
*			   使用前需要更改AngleClosedLoop()函数（角度闭环函数），对于电机pid的调整
*！！注意：初始使用时，请小速度调试，避免因P、D参数不适合或者错误导致运动对象异常运动造成危险
*Version：     Beta版
*
********************************************************************/


#include "math.h"
#include "point2point.h" 

/**
* @name 	CcltAngleAdd
* @brief	对-180,180交界处作处理
* @param	angle1:角度1;
			angle2:角度2;
* @retval
*/
static float CcltAngleAdd(float angle1, float angle2)
{
	float result = 0.0f;
	result = angle1 + angle2;
	if (result >  180.0f)  result -= 360.0f;
	if (result < -180.0f)  result += 360.0f;
	return result;
	
}


/**
  * @name 		VelTwoCtl
  * @brief  	正数朝前
  * @param  	vellBase:朝前速度；
			    vellErr:符号为正，朝左拐
  * @retval 	
  */
static void VelDecomposeCtl(int vellBase,int vellErr)
{
	//1号电机速度赋值函数
	VelCrl(MOTOR_ONE,vellBase - vellErr);				
	//2号电机速度赋值函数
	VelCrl(MOTOR_TWO, vellBase + vellErr);				
}


/**
* @name 	CcltAngleSub
* @brief	对-180,180交界处作处理
* @param	minuend: 被减数;
			subtrahend: 减数 A - B,A为被减数，B为减数;
* @retval
*/
float CcltAngleSub(float minuend, float subtrahend)
{
	float result = 0.0f;
	result = minuend - subtrahend;
	if (result >  180.0f)  result -= 360.0f;
	if (result < -180.0f)  result += 360.0f;
	return result;
}

/**
* @name 	CcltTwoLineIntersection2
* @brief	计算两条直线的交点
* @param	line1:直线1;
			line2:直线2;
* @retval
*/
static ActPoint CcltTwoLineIntersection2(ActLine2 line1, ActLine2 line2)
{
	ActPoint intersection;
	//斜率
	float k1 = 0.0f;
	float k2 = 0.0f;

	//因为浮点运算,未对与x轴垂直的直线处理。
	k1 = tan(line1.angle * CHANGE_TO_RADIAN);
	k2 = tan(line2.angle * CHANGE_TO_RADIAN);

	intersection.x = (line1.point.x*k1 - line1.point.y - line2.point.x * k2 + line2.point.y)
						 / (k1 - k2);
	intersection.y = k1 * (intersection.x - line1.point.x) + line1.point.y;

	return intersection;
}


/**
* @name 	CcltLineAngle
* @brief	计算两点直线方向角度
* @param	pointStart:起始点：
			pointEnd:终止点;
* @retval
*/
static float CcltLineAngle(ActPoint pointStart, ActPoint pointEnd)
{
	float a = 0.0f;
	float b = 0.0f;
	
	a = pointEnd.y - pointStart.y;
	b = pointEnd.x - pointStart.x;
	//atan2f范围可以包含-180到180  
	return (atan2f(a, b) * CHANGE_TO_ANGLE); 
}

static float s_angleErrOld = 0.0f; 
void Point2Ponit_Deinit()
{
	s_angleErrOld = 0.0f;
}

/**
  * @name 	    AngleClosedLoop
  * @brief  	角度闭环
  * @param  	presentAngle: 当前角度;
				targetAngle: 目标角度;
				vellP: 旋转速度比例;
				vellBase:前进速度;
  * @retval 	
  */


static float AngleClosedLoop(float presentAngle,float targetAngle,float vellP,float vellBase)
{
#define P1    10
#define P2    5
#define P3    0
#define VRATE 1.0f
		float angleErr = 0.0f;				
	//目标角度与实际角度的差值开二次根
	float angleErrSqrt = 0.0f;			
	//上一周期目标角度与实际角度的差值
	//angleErrErr = angleErr - s_angleErrOld;
	float angleErrErr = 0.0f;			
	//旋转速度
	float vellErr = 0.0f;			
	
	//实际速度转换
	vellBase = VRATE * vellBase;
	//目标角度与实际角度的差值
	
	
	//求目标角度与实际角度的差值
	angleErr = CcltAngleSub(targetAngle , presentAngle);

	//进行开根处理，目的压缩偏差量。
	if(angleErr > 0.0f)										
	{
		angleErrSqrt = sqrt(angleErr);
	}
	else
	{
		angleErrSqrt = -sqrt(fabsf(angleErr));
	}
	
	//求本周期角度差值与上周期角度差值的差值
	angleErrErr = angleErr - s_angleErrOld;				
	
	//当偏差过大时用P调解，当偏差不大是用PD调解
	if(fabsf(angleErr) > 10.0f)								
	{
		vellErr = angleErrSqrt * P1 * vellP;
	}
	else
	{
		vellErr = angleErrSqrt * P2 * vellP + angleErrErr * P3;
	}
	
	//进行速度赋值
	VelDecomposeCtl(vellBase,vellErr);					
	s_angleErrOld = angleErr;
 return angleErr;
}



/**
  * @name 	MvByLine
  * @brief  	直线闭环，速度为正
  * @param  	presentLine：当前姿态；
				targetLine：v目标点目标方向，等同于目标直线；
				speed:直线速度;
  * @retval 	返回值为当前点到目标点距离。单位mm
  */
float *MvByLine(ActLine2 presentLine, ActLine2 targetLine, float speed)
{
  static float a[2];
	//当前点到目标直线的垂线
	ActLine2 verticalLine;											
	//中点	
	ActPoint  midpoint;											
	//控制的目标角度
	float targetAngle = 0.0f;	
	//当前点到目标点距离	
	float psntToEndDis = 0.0f;																						  
	//目标直线加90度，求出垂线方向
	verticalLine.angle = CcltAngleAdd(targetLine.angle, 90.0f);
	//把当前点赋给垂线
	verticalLine.point = presentLine.point;		
	//计算垂线与目标直线的交点
	midpoint = CcltTwoLineIntersection2(targetLine, verticalLine);
	//计算中点横坐标	
	midpoint.x = (midpoint.x + targetLine.point.x) * 0.5f;													  
	//计算中点纵坐标
	midpoint.y = (midpoint.y + targetLine.point.y) * 0.5f;													  
	//计算当前点到中点方向
	targetAngle = CcltLineAngle(presentLine.point, midpoint);												  
	//当前点到目标点距离
	psntToEndDis = sqrtf((presentLine.point.x - targetLine.point.x) * (presentLine.point.x - targetLine.point.x)
					+ (presentLine.point.y - targetLine.point.y) * (presentLine.point.y - targetLine.point.y));
	//进行角度闭环控制
	a[0]=AngleClosedLoop(presentLine.angle,targetAngle,2.0f,speed);
	a[1]=psntToEndDis;
	return a;
}
