//
//  vehicle_detection_sensor.h
//  Awareness_1.0
//
//  Created by Atsushi Fujita on 2016/11/16.
//  Copyright © 2016年 Atsushi Fujita. All rights reserved.
//

#ifndef VEHICLE_DETECTION_SENSOR_H
#define VEHICLE_DETECTION_SENSOR_H

//#include "common_math.h"

//------------------------------------------------------------------------------
// 提案手法のパラメータ設定など；不要な定数もある
//------------------------------------------------------------------------------

//挙動を決定するパラメータ値
#define LEGACY_NODE_ID 234		//レガシー車両ID開始番号
#define START_FRAME 0		  //推定開始時刻（フレーム，秒）
#define INTERVAL_SEND 100	  //BSM送信間隔

//センサ誤差モデル
#define ESTIMATTION_GPS_POSITION_ERROR 5      //mean = 5m
#define ESTIMATTION_RADAR_POSITION_ERROR 5.25 //{GPS+RADAR} error
#define ESTIMATION_RADAR_ERROR 0.25
#define ESTIMATTION_SPEED_ERROR 0.158         //0.158=σ=0.5/√10

//マッチングで利用するパラメータ
#define LIKELIHOOD 1			      //マッチング成功時尤度の増加値
#define DISTANCE_FOR_MATCHING 1.0
#define DISTANCE_FOR_MARGE    3.0
#define DISTANCE_FOR_SENSE    3.0
#define MAX_DIRECT_SENSING 7	      //この距離内ならば直接センシングできるはず
#define MAX_DIRECT_DEGREES 90
#define DISTANCE_MATCHING_RECEIVE  20	//受信車両のうちマッチング対象車両とするための距離

//車両情報削除のための閾値
#define INFO_DELETE 		200
#define THRESHOLD_DELETE 	10.0

//General
#define	FLOAT_MAX	100000
#define BRAKING_SPEED 9.7		//急加速時の制動距離
#define AREA_RANGE 2000			//車両の存在範囲（m）ループシナリオに対応するため
#define INTERVAL_TIMES 1		//センサによる情報取得間隔（秒）

//情報識別のためのフラグ
#define IDENTIFIER_INIT       0
#define IDENTIFIER_SENSOR     1000
#define IDENTIFIER_V2V        2000
#define IDENTIFIER_SENSOR_V2V 3000
#define IDENTIFIER_V2V_SENDER 4000
#define IDENTIFIER_LEGACY     5000

#define SENSOR_DETECT_ERROR 5


//------------------------------------------------------------------------------
// センサ誤差モデル（MoNAで考慮するかは要検討）
//------------------------------------------------------------------------------
//#define GPS_EXPMODEL//実機から作成したモデル
/*
#ifdef GPS_EXPMODEL
	#define ALPHA				0.25
	#define BASE_GPS_DEVIATION	0.25
	#define BASE_GPS_AVG		10
	#define ZIG_GPS_DEVIATION	0.5
	#define INTERVAL_PLANET		10
#else
	#define GPS_NORMAL	//正規分布モデル
#endif
*/

#define GPS_NORMAL //正規分布モデル

//_/_/混入率_/_/
#define EQUIP_RATIO	1		//EQUIP_RATIOに1台が保持
//シンプルに車両IDで乱数で与えてよいかは要検討のこと

#define RATIO_HIGH
//#define RATIO_MIDDLE
//#define RATIO_LOW


using namespace ScenSim;

namespace Wave {

float normalDistRandom(float m, float sigma){
// box and muller法
// 一様分布に従う確率変数から標準ガウス分布に従う確率変数を生成させる手法
	float r1,r2;
	float Z1,Z2;
	r1 = (float)rand()/RAND_MAX;
	r2 = (float)rand()/RAND_MAX;
	Z1 = sigma * (float)sqrt(-2*log(r1)) * (float)cos(2*PI*r2)+ m; // x成分
	Z2 = sigma * (float)sqrt(-2*log(r1)) * (float)sin(2*PI*r2)+ m;  // y成分
	return Z1;
}

//For velocity, acceleration and so on.
class VectorType{
public:
    float x,y;

    VectorType(): x(0), y(0)
    {}

    VectorType(const float& initX, const float& initY): x(initX), y(initY)
    {}

    void clear() {
        x = 0;
        y = 0;
    }
}; //VectorType//


float distance2Coordinate(float a_x, float a_y, float b_x, float b_y){
	float tmp_x = a_x - b_x;
    float tmp_y = a_y - b_y;
    float result = sqrt(tmp_x * tmp_x + tmp_y * tmp_y);
	return result;
} //distance2Coordinate


VectorType rotateXY(
        const float x,
        const float y,
        const float theta)
{
    float cs = cos(theta);
    float sn = sin(theta);
    
    return VectorType(x*cs + y*sn, y*cs - x*sn);
}


int crossProduct(
        const VectorType p1,
        const VectorType p2,
        const VectorType p3)
{
    float ZCoord = p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y);
    if ( ZCoord > 0 ) {return  1;} // 左
    else if ( ZCoord < 0 ) {return -1;} // 右
	else {return  0;} // 線上
}


bool isDetectableBySensor(
        const VectorType relativeCoord,
        const VectorType selfVelocity)
{

    //最大検出距離50としている
    float expandDist = 50/distance2Coordinate(selfVelocity.x, selfVelocity.y, 0, 0);
    
    VectorType origin = VectorType(0.00, 0.00);
    VectorType areaCenter = VectorType(selfVelocity.x * expandDist, selfVelocity.y * expandDist);
    VectorType areaLeft = rotateXY(areaCenter.x, areaCenter.y, 3.14/3);
    VectorType areaRight = rotateXY(areaCenter.x, areaCenter.y, (-1)*3.14/3);
    VectorType areaLeftMiddle = rotateXY(areaCenter.x, areaCenter.y, 3.14/6);
    VectorType areaRightMiddle = rotateXY(areaCenter.x, areaCenter.y, (-1)*3.14/6);
    
    int pOR = crossProduct(relativeCoord, origin, areaRight);
    int pRM = crossProduct(relativeCoord, areaRight, areaRightMiddle);
    int pMC = crossProduct(relativeCoord, areaRightMiddle, areaCenter);
    int pCM = crossProduct(relativeCoord, areaCenter, areaLeftMiddle);
    int pML = crossProduct(relativeCoord, areaLeftMiddle, areaLeft);
    int pLO = crossProduct(relativeCoord, areaLeft, origin);
    
    if ((pOR > 0) && (pRM > 0) && (pMC > 0) && (pCM > 0) && (pML > 0) && (pLO > 0)) {
        return true;
    } else if ((pOR < 0) && (pRM < 0) && (pMC < 0) && (pCM < 0) && (pML < 0) && (pLO < 0)) {
        return true;
    }
    
    return false;
} //isDetectableBySensor




//------------------------------------------------------------------------------
// GPS, Velocity, Sensor Information
//------------------------------------------------------------------------------
class GPSInformation{
public:
    TimeType sensedTime;
    float xPositionMeters, yPositionMeters;

    GPSInformation(const TimeType& initSensedTime, 
                   const float& initXPositionMeters,
                   const float& initYPositionMeters)
        :
    sensedTime(initSensedTime),
	xPositionMeters(initXPositionMeters),
	yPositionMeters(initYPositionMeters)
    {}

    ~GPSInformation(){
  		#ifdef MEMORY_LEAK_DEBUG_TMP
			cout<<sensedTime<<":GpsInformation delete\t"<<xPositionMeters<<", "<<yPositionMeters<<endl;
		#endif
    }
}; //GPS Information


class VelocityInformation{
public:
    TimeType sensedTime;
    VectorType velocity;
	
    VelocityInformation(const TimeType& initSensedTime, 
    				    const  VectorType& initVelocity)
        :
    sensedTime(initSensedTime),
	velocity(initVelocity)
    {}

    ~VelocityInformation(){
  		#ifdef MEMORY_LEAK_DEBUG_TMP
			cout<<sensedTime<<":VelocityInformation delete\t"<<velocity.x<<", "<<velocity.y<<endl;
		#endif
    }
}; //Velocity Information


class SensorInformation{
public:
    TimeType sensedTime;
    float xPositionMeters, yPositionMeters;
	NodeIdType observerNodeId;
	
    SensorInformation(const TimeType& initSensedTime, 
    				  const float& initXPositionMeters,
    				  const float& initYPositionMeters,
					  const NodeIdType& initObserverNodeId)
        :
    sensedTime(initSensedTime),
	xPositionMeters(initXPositionMeters),
    yPositionMeters(initYPositionMeters),
	observerNodeId(initObserverNodeId)
    {}

    ~SensorInformation(){
  		#ifdef MEMORY_LEAK_DEBUG_TMP
			cout<<sensedTime<<":SensorInformation delete\t"<<xPositionMeters<<", "<<yPositionMeters<<endl;
		#endif
    }
}; // SensorInformation


class PositionCandidate{
public:
	TimeType sensedTime;
    float xPositionMeters, yPositionMeters, positionError;
	NodeIdType candNodeId;
    
    PositionCandidate(){}
    ~PositionCandidate(){}
    
    PositionCandidate(const TimeType& initSensedTime,
    				  const float& initXPositionMeters,
    				  const float& initYPositionMeters,
 					  const float& initPositionError,
 					  const NodeIdType& initCandNodeId)
        :
    sensedTime(initSensedTime),
	xPositionMeters(initXPositionMeters),
    yPositionMeters(initYPositionMeters),
	positionError(initPositionError),
	candNodeId(initCandNodeId)
    {}
    
    void PrintOut(){
        cout <<"DEBUG_CANDIDATE:"<<"<<" << sensedTime << ">> "
        << xPositionMeters << ", "
        << yPositionMeters<< ", " 
        << positionError<<","
        << candNodeId<<endl;
    }
}; //Position Candidate


//--------------------------------------------------------------------------------------
// Matching Table（マッチングに用いるテーブル）；更新，クエリ，出力関数
//--------------------------------------------------------------------------------------
class MatchingTable{
public: 
    NodeIdType neighborId;
    float likelihood;
    
    MatchingTable(){}
    ~MatchingTable(){}
};


//------------------------------------------------------------------------------
// Vehicle Information：車両情報についてのクラス，周辺認識・受信パケット・センシングの各情報の管理にすべて利用
//------------------------------------------------------------------------------
class VehicleInformation{
public:
    NodeIdType sourceNodeId, targetNodeId;
    TimeType sensedTime, updatedTime, lastSendTime;

    VectorType size;
    float xExactPosition, yExactPosition;
    bool sendFlag, oldSendFlag; //送信フラグ（target情報の送信の有無，true=送信）
	int sendCount;              //送信回数
    
    int identifierFlag;          //車両情報の種別 0:初期，1:センサ+自車，2:V2V，3:センサ+V2V

    float xPositionMeters, yPositionMeters;  //現在時刻での推定位置
    float estPositionError;	                 //現在時刻での推定位置の誤差
    VectorType velocity, acceleration;

	//GPS測定位置
    float xMeasuredPosition, yMeasuredPosition;
    
    //センサ測定時のポジション
    float xSensorPosition, ySensorPosition;
    
	//1タイムスロット前の情報：保持理由 = 隣接ノードから正しい速度情報が送られたときに，その情報から位置を更新する必要があるため
    float xPriPositionMeters, yPriPositionMeters; //1つ前のタイムスロットの推定位置
    float estPriPositionError;	                  //1つ前のタイムスロットの推定誤差

	//推定に利用する各情報（テーブル）
    map<TimeType, shared_ptr<GPSInformation> > gpsInfoTable;
    map<TimeType, shared_ptr<VelocityInformation> > velocityInfoTable;
    multimap<TimeType, shared_ptr<SensorInformation> > sensorInfoTable;

	//相対位置（マッチングに利用）
    float xRelativePosition, yRelativePosition;
    NodeIdType mgInfoId;   //情報へのポインタ
	float matchingYudo;
   
   
    //-----------------------------------
    //送信優先度の評価関数に関連する変数
    //-----------------------------------
    float lanePosition;
    float surroundDensity;
    int vehicleType;
    float priority;
    
    //-----------------------------------
    //評価に関連する変数
    //-----------------------------------
    int msgOverhead;

    VehicleInformation() {} //コンストラクタ
    
    ~VehicleInformation() {
		#ifdef MEMORY_LEAK_DEBUG_TMP
			cout<<updatedTime<<":"<<sourceNodeId<<"->"<<targetNodeId<<" delete "<<xPositionMeters<<", "<<yPositionMeters<<endl;
		#endif
    } //デコンストラクタ

    VehicleInformation(
        const NodeIdType& initSourceNodeId,
        const NodeIdType& initTargetNodeId,
        const TimeType& initSensedTime,
        const VectorType& initSize,
        const float& initXExactPosition,
        const float& initYExactPosition,
        const float& initXPositionMeters,
        const float& initYPositionMeters,
        const float& initEstPositionError,
        const VectorType& initVelocity,
        const VectorType& initAcceleration,
        const int initIdentifierFlag,
        const int initMsgOverhead
        )
        :
        sourceNodeId(initSourceNodeId),
        targetNodeId(initTargetNodeId),
        sensedTime(initSensedTime),
		updatedTime(initSensedTime),
        lastSendTime(initSensedTime),
        size(initSize),
        xExactPosition(initXExactPosition),
        yExactPosition(initYExactPosition),
		sendFlag(false),
		oldSendFlag(false),
		sendCount(0),
		xPositionMeters(initXPositionMeters),
        yPositionMeters(initYPositionMeters),
		estPositionError(initEstPositionError),
        velocity(initVelocity),
        acceleration(initAcceleration),
        xMeasuredPosition(initXPositionMeters),
        yMeasuredPosition(initYPositionMeters),
        xSensorPosition(initXPositionMeters),
        ySensorPosition(initYPositionMeters),
        xPriPositionMeters(initXPositionMeters),
        yPriPositionMeters(initYPositionMeters),
		estPriPositionError(initEstPositionError),
		xRelativePosition(0),
		yRelativePosition(0),
		mgInfoId(LEGACY_NODE_ID),
		matchingYudo(0),
        identifierFlag(initIdentifierFlag),
        lanePosition(-10),
        surroundDensity(-1),
        vehicleType(5),    //車両タイプが未決定（初期値）
        priority(-1),
        msgOverhead(0)
        {}


    void PrintOut() {
        cout << "<" << sensedTime << "> "
			 << "[" << updatedTime << "] "
			 << sourceNodeId << "->"
             << targetNodeId << ": "
             << "P(" << xPositionMeters << ", "
             << yPositionMeters << ") "
             << "^P(" << xExactPosition << ", "
             << yExactPosition << ") "
             << "OldP(" << xPriPositionMeters << ", "
             << yPriPositionMeters << ") "
             << "V(" << velocity.x << ", "
             << velocity.y << ") "
             << "E(" << estPositionError << ") "
             << "B(" << xSensorPosition <<", "
             << ySensorPosition <<") "
             << "M(" << xMeasuredPosition <<", "
             << yMeasuredPosition <<") "
             << "R(" << xRelativePosition <<", "
             << yRelativePosition <<") "
             << endl;
			PrintTable();
    }

	void ClearTable();
	void PrintTable();

	void CurrentComplement();
	void CurrentRelativeComplement(const float& sVelocityX, const float& sVelocityY);
    
	PositionCandidate Complement(
		const TimeType& sensedTime,
		const float& xPositionMeters, 
		const float& yPositionMeters, 
		const float& positionError,
		const NodeIdType& nodeId
	);

	VectorType AvgVelocity();

    //先進車両か，レガシー車両かの判定
    bool EquipFlag() {
		if(targetNodeId >= LEGACY_NODE_ID) return false;
		return true;
	}
	
	void AddGpsInformation(
        const float& newGPSXPosition,
        const float& newGPSYPosition,
        const TimeType& newSensedTime
	);

	void AddVelocityInformation(
        const VectorType& newVelocity,
        const TimeType& newSensedTime
	);

	void AddSensorInformation(
        const float& newSensorXPosition,
        const float& newSensorYPosition,
        const TimeType& newSensedTime,
        const NodeIdType& observerId
	);
}; //Vehicle Information


//GPSデータをテーブルに追加
inline 
void VehicleInformation::AddGpsInformation(
        const float& newGPSXPosition,
        const float& newGPSYPosition,
        const TimeType& newSensedTime)
{
	shared_ptr<GPSInformation> GPSInfoPtr(new 
	GPSInformation(newSensedTime,
 				newGPSXPosition,
				newGPSYPosition));
	gpsInfoTable.insert(make_pair(newSensedTime, GPSInfoPtr));
} //AddGpsInformation


//速度データをテーブルに追加
inline 
void VehicleInformation::AddVelocityInformation(
        const VectorType& newVelocity,
        const TimeType& newSensedTime)
{		
	if((newVelocity.x == FLOAT_MAX)||(newVelocity.y == FLOAT_MAX))return;
    
    shared_ptr<VelocityInformation> VelocityInfoPtr(new VelocityInformation(newSensedTime, newVelocity));
    velocityInfoTable.insert(make_pair(newSensedTime, VelocityInfoPtr));
} //AddVelocityInformation


//センサデータをテーブルに追加，positionは相対位置ではない
inline 
void VehicleInformation::AddSensorInformation(
        const float& newSensorXPosition,
        const float& newSensorYPosition,
        const TimeType& newSensedTime,
        const NodeIdType& observerId)
{		
		//挿入前に同じ観測ノードかつ同じフレームに観測された測定結果がある場合は
		//基準となるGPS位置が同じであるため，削除する
		
		long long int newMS = (newSensedTime / MILLI_SECOND) / 100; // newSensedTime = nano sec.
		int newFlameCount = (int)newMS / 10;
		typedef multimap<TimeType, shared_ptr<SensorInformation> >::iterator SensorIterType;
    
		SensorIterType siter = sensorInfoTable.begin();
    
		while(siter != sensorInfoTable.end()){
			shared_ptr<SensorInformation> SensorInfo = (*siter).second;
			if(SensorInfo->observerNodeId == observerId){
                //観測車両のIDとテーブル内の車両IDが一致する場合
				long long int tmpMS = (SensorInfo->sensedTime / MILLI_SECOND) / 100;
				int tmpFlameCount = (int)tmpMS / 10;
				SensorIterType tmpiter;
				if(newFlameCount == tmpFlameCount){
					//消去，リスト更新
					tmpiter = siter;
					++tmpiter;	
					sensorInfoTable.erase(siter);
					siter = tmpiter;
				}else{
                    //消去しない
					++siter;
				}
			} else {
                ++siter;
			}
		}
    
        //テーブルに情報追加
		shared_ptr<SensorInformation> SensorInfoPtr(new SensorInformation(newSensedTime, newSensorXPosition, newSensorYPosition, observerId));
        sensorInfoTable.insert(make_pair(newSensedTime, SensorInfoPtr));
} //AddSensorInfomation


//テーブル内のデータの時間（sensedTime）と現在時刻（sensedTime）の差分が大きい：INFO_DELETE * 100 * MILLI_SECOND 以上 の場合，そのデータを削除
inline
void VehicleInformation::ClearTable(){
    typedef map<TimeType, shared_ptr<GPSInformation> >::iterator GpsIterType;
    typedef map<TimeType, shared_ptr<VelocityInformation> >::iterator VelocityIterType;
    typedef multimap<TimeType, shared_ptr<SensorInformation> >::iterator SensorIterType;
    
	GpsIterType gpsiter = gpsInfoTable.begin();
	while(gpsiter != gpsInfoTable.end()){
		TimeType tmpTime = sensedTime - (*gpsiter).first;
		if(tmpTime > INFO_DELETE * 100 * MILLI_SECOND){
			gpsInfoTable.erase(gpsiter++);
			continue;
		}
		++gpsiter;
	}
    
	VelocityIterType viter = velocityInfoTable.begin();
	while(viter != velocityInfoTable.end()){
		TimeType tmpTime = sensedTime - (*viter).first;
		if(tmpTime > INFO_DELETE * 100 * MILLI_SECOND){
			velocityInfoTable.erase(viter++);
			continue;
		}
		++viter;
	}

	SensorIterType siter = sensorInfoTable.begin();
	while(siter != sensorInfoTable.end()){
		TimeType tmpTime = sensedTime - (*siter).first;
		if(tmpTime > INFO_DELETE * 100 * MILLI_SECOND){
			sensorInfoTable.erase(siter++);
			continue;
		}
		++siter;
	}
} //ClearTable
	

//テーブル内の情報をプリント
inline
void VehicleInformation::PrintTable(){
    typedef map<TimeType, shared_ptr<GPSInformation> >::iterator GpsIterType;
    typedef map<TimeType, shared_ptr<VelocityInformation> >::iterator VelocityIterType;
    typedef multimap<TimeType, shared_ptr<SensorInformation> >::iterator SensorIterType;
    
	GpsIterType gpsiter = gpsInfoTable.begin();
	while(gpsiter != gpsInfoTable.end()){
		shared_ptr<GPSInformation> GpsInfo = (*gpsiter).second;
		cout<<"\tGPS:"<<GpsInfo->sensedTime<<","
			<<GpsInfo->xPositionMeters<<","
			<<GpsInfo->yPositionMeters<<endl;
		++gpsiter;
	}

	VelocityIterType viter = velocityInfoTable.begin();
	while(viter != velocityInfoTable.end()){
		shared_ptr<VelocityInformation> VInfo = (*viter).second;
		cout<<"\tSpeed:"<<VInfo->sensedTime<<","
			<<VInfo->velocity.x<<","
			<<VInfo->velocity.y<<endl;
		++viter;
	}

	SensorIterType siter = sensorInfoTable.begin();
	while(siter != sensorInfoTable.end()){
		shared_ptr<SensorInformation> SensorInfo = (*siter).second;
		cout<<"\tSensor:"<<SensorInfo->sensedTime<<","
			<<SensorInfo->xPositionMeters<<","
			<<SensorInfo->yPositionMeters<<","
			<<SensorInfo->observerNodeId<<endl;
		++siter;
	}
} //PrintTable



inline
VectorType VehicleInformation::AvgVelocity(){
    typedef map<TimeType, shared_ptr<VelocityInformation> >::iterator VelocityIterType;
	VelocityIterType viter = velocityInfoTable.begin();
	VectorType tmpvel = VectorType(FLOAT_MAX, FLOAT_MAX);
    
	int count = 0; //速度情報数のカウンタ
	while(viter != velocityInfoTable.end()){
		shared_ptr<VelocityInformation> VInfo = (*viter).second;
        
        // 絶対値で 100m/s 以上となることはない想定
		if((VInfo->velocity.x < 100)&&(VInfo->velocity.y < 100)){
			if((VInfo->velocity.x > -100)&&(VInfo->velocity.y > -100)){
                //ループ時の速度は含まない
				++count;
				if(count == 1){
					tmpvel.x = VInfo->velocity.x;
					tmpvel.y = VInfo->velocity.y;
				}else{
					tmpvel.x = tmpvel.x + VInfo->velocity.x;
					tmpvel.y = tmpvel.y + VInfo->velocity.y;
				}
			}
		}
		++viter;
	}
	if(count > 0){
		return VectorType((tmpvel.x / count), (tmpvel.y / count));
	}else{
		return VectorType(FLOAT_MAX, FLOAT_MAX);
	}	
} //AvgVelocity
	


//速度情報を利用して，車両位置をアップデート（補完）
inline
PositionCandidate VehicleInformation::Complement(
		const TimeType& pcSensedTime,
		const float& pcXPositionMeters, 
		const float& pcYPositionMeters, 
		const float& pcPositionError,
		const NodeIdType& pcNodeId)
{
	TimeType resultTime = pcSensedTime;
	float resultXPosition = pcXPositionMeters;
	float resultYPosition = pcYPositionMeters;
	float resultPositionError = pcPositionError;
    typedef map<TimeType, shared_ptr<VelocityInformation> >::iterator 	VelocityIter;
    
    TimeType lastTime = sensedTime - 100*MILLI_SECOND;
    
	for(TimeType tmpTime = pcSensedTime + 100*MILLI_SECOND; tmpTime <= lastTime; tmpTime += 100*MILLI_SECOND){
        //100ms 毎のループ

		VelocityIter viter = velocityInfoTable.find(tmpTime);
		if(viter == velocityInfoTable.end()){
            //tmpTimeにおける速度情報が欠損している場合
			if((velocity.x != FLOAT_MAX)&&(velocity.y != FLOAT_MAX)){
                //速度情報を利用して位置情報をアップデート（補完）
				resultXPosition += velocity.x / 10;
				resultYPosition += velocity.y / 10;
			}else{
            
			}
			resultPositionError += (float)BRAKING_SPEED / (2 * (float)sqrt(10.0));
            
		}else{
            //tmpTimeにおける速度情報が存在している場合
			shared_ptr<VelocityInformation> 
				tmpVelocityInfo = (*viter).second;
			resultXPosition += tmpVelocityInfo->velocity.x  / 10;
			resultYPosition += tmpVelocityInfo->velocity.y  / 10;
			resultPositionError += (float)ESTIMATTION_SPEED_ERROR;
		}
	}

	PositionCandidate pcresult = PositionCandidate(
		resultTime,
		resultXPosition,
		resultYPosition,
		resultPositionError,
		pcNodeId
	);
	return pcresult;
} //Complement


inline
void VehicleInformation::CurrentComplement(){
	//センサによる隣接車両情報の更新前なので，位置誤差は等加速を考慮した値にする
	if((velocity.x != FLOAT_MAX)&&(velocity.y != FLOAT_MAX)){
		xPositionMeters = xPriPositionMeters + (float)0.1 * velocity.x;
		yPositionMeters = yPriPositionMeters + (float)0.1 * velocity.y;
	}
	estPositionError = estPriPositionError + (float)BRAKING_SPEED / (2 * (float)sqrt(10.0));
} //CurrentComplement


inline
void VehicleInformation::CurrentRelativeComplement(const float& sVelocityX, const float& sVelocityY){
	if((velocity.x != FLOAT_MAX)&&(velocity.y != FLOAT_MAX)){
		if((sVelocityX != FLOAT_MAX)&&(sVelocityY != FLOAT_MAX)){
			xRelativePosition += (float)0.1 * (velocity.x - sVelocityX);
			yRelativePosition += (float)0.1 * (velocity.y - sVelocityY);
		}
	}
} //CurrentRelativeComplement



//--------------------------------------------------------------------------------------
// MatchingGraphCandidate（マッチング候補用のデータ構造）
//--------------------------------------------------------------------------------------
class MatchingGraphCandidate{
public:
	float likelihood;
    shared_ptr<VehicleInformation> mgcInfoPtr;	//情報へのポインタ
	MatchingGraphCandidate() {}
	MatchingGraphCandidate(const float&, const shared_ptr<VehicleInformation>&);
	MatchingGraphCandidate(const float& );
	~MatchingGraphCandidate() {
   		#ifdef MEMORY_LEAK_DEBUG_TMP
			cout<<mgcInfoPtr->sourceNodeId<<"->"<<mgcInfoPtr->targetNodeId<<" MatchingGraphCandidate delete"<<endl;
		#endif
		mgcInfoPtr.reset();
	}
};

inline
MatchingGraphCandidate::MatchingGraphCandidate(const float& initLikelihood, const shared_ptr<VehicleInformation>& initMgcInfoPtr){
	likelihood = initLikelihood;
	mgcInfoPtr = initMgcInfoPtr;
}

inline
MatchingGraphCandidate::MatchingGraphCandidate(const float& initLikelihood){
	likelihood = initLikelihood;
}


//------------------------------------------------------------------------------
// MatchingGraphNode（マッチング用のデータ構造）
//------------------------------------------------------------------------------
class MatchingGraphNode{
public:
	TimeType sensedTime;
	NodeIdType sourceId, targetId, matchId;	//マッチした先進車両のID
    float matchYudo;
    
	float xRelativePosition, yRelativePosition; //自車両からの相対位置
	float xAbsolutePosition, yAbsolutePosition; //（変換）絶対位置
	
    VectorType currentVelocity; //現在の速度
	TimeType vectorTime;
    
	float estPositionError;
	bool sensedFlag, oldSensedFlag;
	shared_ptr<VehicleInformation> mgnInfoPtr;		//直接観測データへのポインタ
    map<NodeIdType, shared_ptr<MatchingGraphCandidate> > matchingCandidateList;  //候補車両のリスト
    map<TimeType, shared_ptr<VelocityInformation> > velocityInfoTable;	//速度のリスト

    MatchingGraphNode() {}
    MatchingGraphNode(const NodeIdType&, const NodeIdType&, const float&, const float&);
    ~MatchingGraphNode() {
   		mgnInfoPtr.reset();
		matchingCandidateList.clear();

   		#ifdef MEMORY_LEAK_DEBUG_TMP
			cout<<sensedTime<<":"<<sourceId<<"=>"<<targetId<<" MatchingGraphNode delete"<<endl;
		#endif
    }
    
	void PrintOut();
	void DisplayPrintOut();
	void UpdateMatchingCandidateList(const NodeIdType&, const float&, const shared_ptr<VehicleInformation>&);
	void UpdateMatchingCandidateList(const NodeIdType&, const float&);
	void ClearMatchingCandidateList(const NodeIdType& targetId);
	NodeIdType UpdateMatching();
	bool alreadyMatching();
	void printMatchingCandidateList();
	void CurrentComplement(const TimeType& currentTime);
	void CurrentRelativeComplement(const float&, const float&);

	bool RecvUpdateMatching(const NodeIdType& recvTargetId, const shared_ptr<VehicleInformation>& initMgcInfoPtr);
	void Estimate(const TimeType& currentTime, int& gps_num, int& sensor_num);
	void ClearMatchingCandidateList();
	void AddMatchingCandidateList();
	void UpdateVehicleInformation();
	void AddVelocityInformation(
        const VectorType& newVelocity,
        const TimeType& newSensedTime
	);
	void AddAllVelocityInfo();
	void CompleteVelocityInfo();

	void MakePositionCandidate(
		const TimeType& pcSensedTime,
		const float& pcXPositionMeters,
		const float& pcYPositionMeters,
		const float& pcPositionError,
		const NodeIdType& pcNodeId,
		PositionCandidate& resultCandiate, 
		const TimeType& nowTime);
	void DeleteProcess();
}; // class MatchingGraphNode


inline
MatchingGraphNode::MatchingGraphNode(const NodeIdType& initSourceId,
                                     const NodeIdType& initTargetId,
                                     const float& initXRelativePosition,
                                     const float& initYRelativePosition){
	sourceId = initSourceId;
	targetId = initTargetId;
	matchId = initSourceId;
	matchYudo = 0;
	xRelativePosition = initXRelativePosition;
	yRelativePosition = initYRelativePosition;
	xAbsolutePosition = 0;
	yAbsolutePosition = 0;
	currentVelocity.x = 0;
	currentVelocity.y = 0;
	estPositionError = 0;
	sensedTime = 0;
	vectorTime = 0;
	sensedFlag = false;
	oldSensedFlag = false;
}


inline 
void MatchingGraphNode::AddVelocityInformation(
        const VectorType& newVelocity,
        const TimeType& newSensedTime)
{		
	if((newVelocity.x == FLOAT_MAX)||(newVelocity.y == FLOAT_MAX))return;
		shared_ptr<VelocityInformation> VelocityInfoPtr(new VelocityInformation(newSensedTime, newVelocity));
		 velocityInfoTable.insert(make_pair
			(newSensedTime, VelocityInfoPtr));
} //AddVelocityInformation


inline void MatchingGraphNode::PrintOut(){
	    cout <<sourceId<< "=>"<<targetId<<"("<<matchId<<"):"
//		<< "T[" << sensedTime << "] "
		<< "R_P(" << xRelativePosition << ", "
        << yRelativePosition << ") "
        << "A_P(" << xAbsolutePosition << ", "
        << yAbsolutePosition<< ") "
        << "V(" << currentVelocity.x << ", "
        << currentVelocity.y << ") "
        << "E(" << estPositionError << ") "
		<< "VT[" << vectorTime << "]  YD("
		<< matchYudo<<") ";
    
		if(sensedFlag == true){
			cout<<"Sensed"<<endl;
		}else{
			cout<<endl;
		}
} //PrintOut


inline void MatchingGraphNode::DisplayPrintOut(){
		cout << xAbsolutePosition <<" "
        << yAbsolutePosition 
        << " 1.5 90 0"<<endl;
} //DisplayPrintOut


inline void MatchingGraphNode::AddMatchingCandidateList(){
    if(mgnInfoPtr != nullptr){
		typedef map<NodeIdType, shared_ptr<MatchingGraphCandidate> >::iterator IterType;
		IterType t_iter = matchingCandidateList.find(mgnInfoPtr->targetNodeId);
        
		if(t_iter != matchingCandidateList.end()){
			cout<< mgnInfoPtr->targetNodeId <<endl;
			assert(t_iter == matchingCandidateList.end());
		}
        
		shared_ptr<MatchingGraphCandidate> targetCandidateInfo = shared_ptr<MatchingGraphCandidate>(new MatchingGraphCandidate(FLOAT_MAX, mgnInfoPtr));
		matchingCandidateList.insert(make_pair(mgnInfoPtr->targetNodeId, targetCandidateInfo));
    }
} //AddMatchingCandidateList


inline void MatchingGraphNode::UpdateMatchingCandidateList(const NodeIdType& targetId, const float& initLikelihood, const shared_ptr<VehicleInformation>& targetPtr){
    typedef map<NodeIdType, shared_ptr<MatchingGraphCandidate> >::iterator IterType;
	IterType t_iter = matchingCandidateList.find(targetId);
    
	if(t_iter != matchingCandidateList.end()){
		shared_ptr<MatchingGraphCandidate> targetCandidateInfo = (*t_iter).second;
		if(targetCandidateInfo->likelihood != FLOAT_MAX){
			targetCandidateInfo->likelihood = targetCandidateInfo->likelihood + initLikelihood;
		}
			targetCandidateInfo->mgcInfoPtr = targetPtr;
	}else{
		shared_ptr<MatchingGraphCandidate> targetCandidateInfo = shared_ptr<MatchingGraphCandidate>(new MatchingGraphCandidate(initLikelihood, targetPtr));
		matchingCandidateList.insert(make_pair(targetId, targetCandidateInfo));
	}
} //UpdateMatchingCandidateList


inline void MatchingGraphNode::UpdateMatchingCandidateList(const NodeIdType& targetId, const float& initLikelihood){
    typedef map<NodeIdType, shared_ptr<MatchingGraphCandidate> >::iterator IterType;
	IterType t_iter = matchingCandidateList.find(targetId);
	if(t_iter != matchingCandidateList.end()){
		shared_ptr<MatchingGraphCandidate> targetCandidateInfo = (*t_iter).second;
		if(targetCandidateInfo->likelihood != FLOAT_MAX){
			targetCandidateInfo->likelihood = targetCandidateInfo->likelihood + initLikelihood;
			(*t_iter).second = targetCandidateInfo;
		}
	}else{
		shared_ptr<MatchingGraphCandidate> targetCandidateInfo = shared_ptr<MatchingGraphCandidate>(new MatchingGraphCandidate(initLikelihood));
		matchingCandidateList.insert(make_pair(targetId, targetCandidateInfo));
	}
} //UpdateMatchingCandidateList


inline void MatchingGraphNode::ClearMatchingCandidateList(const NodeIdType& targetId){
    typedef map<NodeIdType, shared_ptr<MatchingGraphCandidate> >::iterator IterType;
	//IterType t_iter = 
	matchingCandidateList.erase(targetId);
//	if(t_iter != matchingCandidateList.end()){
//		shared_ptr<MatchingGraphCandidate> targetCandidateInfo = (*t_iter).second;
//		targetCandidateInfo->likelihood = 0;
//		(*t_iter).second = targetCandidateInfo;
//	}
} //ClearMatchingCandidateList


inline void MatchingGraphNode::printMatchingCandidateList(){
    typedef map<NodeIdType, shared_ptr<MatchingGraphCandidate> >::iterator IterType;
	float tmpLikelihood = 0;
	shared_ptr<VehicleInformation> tmpInfoPtr;
    
	for(IterType t_iter = matchingCandidateList.begin(); (t_iter != matchingCandidateList.end()); t_iter++){
		NodeIdType targetId = (*t_iter).first;
		shared_ptr<MatchingGraphCandidate> targetCandidateInfo = (*t_iter).second;
		tmpLikelihood = targetCandidateInfo->likelihood;
		tmpInfoPtr = targetCandidateInfo->mgcInfoPtr;
		cout<<targetId<<"("<<tmpLikelihood<<"), "; //<<endl;
//		if(tmpInfoPtr != nullptr)tmpInfoPtr->PrintOut();
	}
} //PrintMatchingCandidateList


inline bool MatchingGraphNode::alreadyMatching(){
    typedef map<NodeIdType, shared_ptr<MatchingGraphCandidate> >::iterator IterType;
	bool rflag = false;
	for(IterType t_iter = matchingCandidateList.begin(); (t_iter != matchingCandidateList.end()); t_iter++){
		NodeIdType targetId = (*t_iter).first;
		shared_ptr<MatchingGraphCandidate> targetCandidateInfo = (*t_iter).second;
		if((targetId < LEGACY_NODE_ID)&&(targetCandidateInfo->likelihood == FLOAT_MAX)){
			rflag = true;
		}
	}
	return rflag;
} //alreadyMatching


inline void MatchingGraphNode::ClearMatchingCandidateList(){
	//消去する
	typedef map<NodeIdType, shared_ptr<MatchingGraphCandidate> >::iterator IterType;
	IterType t_iter = matchingCandidateList.begin();
	NodeIdType tmpTargetId = 0;
	float tmpLikelihood = 0;
	shared_ptr<MatchingGraphCandidate> tmpMatchingCandidate;
	shared_ptr<VehicleInformation> tmpMgcInfo;
	IterType tmp_iter;
    
	while(t_iter != matchingCandidateList.end()){
		tmpTargetId = (*t_iter).first;
		tmpMatchingCandidate = (*t_iter).second;
		tmpLikelihood = tmpMatchingCandidate->likelihood;
		tmpMgcInfo = tmpMatchingCandidate->mgcInfoPtr;
		if(tmpMgcInfo != nullptr){
			if((tmpTargetId >= LEGACY_NODE_ID)||(sourceId == tmpMgcInfo->sourceNodeId)){
				tmp_iter = t_iter;
				++tmp_iter;
				matchingCandidateList.erase(t_iter);
				t_iter = tmp_iter;
			}else{
				++t_iter;
			}
		}else{
			++t_iter;
		}
	}
} //ClearMatchingCandidateList


inline void MatchingGraphNode::DeleteProcess(){
    typedef map<NodeIdType, shared_ptr<MatchingGraphCandidate> >::iterator IterType;
	IterType t_iter = matchingCandidateList.begin();
	NodeIdType tmpTargetId = 0;
	float tmpLikelihood = 0;
	shared_ptr<MatchingGraphCandidate> tmpMatchingCandidate;
	shared_ptr<VehicleInformation> tmpMgcInfo;
	IterType tmp_iter;
    
	while(t_iter != matchingCandidateList.end()){
		tmpTargetId = (*t_iter).first;
		tmpMatchingCandidate = (*t_iter).second;
		tmpLikelihood = tmpMatchingCandidate->likelihood;
		tmpMgcInfo = tmpMatchingCandidate->mgcInfoPtr;
		if(tmpMgcInfo != nullptr){
			if(tmpMgcInfo->mgInfoId == targetId){
				tmpMgcInfo->mgInfoId = LEGACY_NODE_ID;
				tmpMgcInfo->matchingYudo = 0;
			}
		}
		++t_iter;
	}
} //DeleteProcess



inline NodeIdType MatchingGraphNode::UpdateMatching(){
    typedef map<NodeIdType, shared_ptr<MatchingGraphCandidate> >::iterator IterType;
	float maxLikelihood = 0;
	if(targetId == 1){
		//自車両のデータの場合はmatchId = sourceIdとする
		matchId = sourceId;
		matchYudo = FLOAT_MAX;
		return 0;
	}
	
	if(matchYudo == FLOAT_MAX) return 0;

	matchYudo = 0;
	matchId = sourceId;
	NodeIdType m_targetId;
	float tmpLikelihood = 0;
	shared_ptr<VehicleInformation> minInfoPtr;
	shared_ptr<VehicleInformation> tmpInfoPtr;

	for(IterType t_iter = matchingCandidateList.begin(); (t_iter != matchingCandidateList.end()); t_iter++){
		m_targetId = (*t_iter).first;
		shared_ptr<MatchingGraphCandidate> targetCandidateInfo = (*t_iter).second;
		tmpLikelihood = targetCandidateInfo->likelihood;
		tmpInfoPtr = targetCandidateInfo->mgcInfoPtr;
		bool flag = false;
		if(tmpInfoPtr != nullptr){
			flag = true;
		}

//		if(tmpInfoPtr == nullptr) continue;
		if((flag)&&(tmpInfoPtr->mgInfoId == FLOAT_MAX)){
			//対象のデータが既に固定マッチングされている場合
			continue;
		}
		if(tmpLikelihood == FLOAT_MAX){
			//固定マッチングされている場合
			if(flag){
				tmpInfoPtr->mgInfoId = targetId;
				tmpInfoPtr->matchingYudo = FLOAT_MAX;
			}
			matchId = m_targetId;
			matchYudo = FLOAT_MAX;
			return 0;
		}

		if(tmpLikelihood > maxLikelihood){
			if(flag){
				if(tmpInfoPtr->matchingYudo < tmpLikelihood){
					//既にマッチングされているものよりも大きければＯｋ
					maxLikelihood = tmpLikelihood;
					matchId = m_targetId;
					matchYudo = maxLikelihood;
					minInfoPtr = tmpInfoPtr;
				}else{
					//既にマッチングされているものより小さければ更新しない
					if(tmpLikelihood == FLOAT_MAX)targetCandidateInfo->likelihood = 0;
				}
			}else{
				maxLikelihood = tmpLikelihood;
				matchId = m_targetId;
				matchYudo = maxLikelihood;
			}
		}
	}
	NodeIdType resultId = 0;	//マッチングした情報が既に異なる車両情報とマッチングされていた場合
	if(matchId != sourceId){
		if(minInfoPtr != nullptr){
			resultId = minInfoPtr->mgInfoId;
			minInfoPtr->mgInfoId = targetId;
			minInfoPtr->matchingYudo = maxLikelihood;
			if((targetId != resultId)&&(resultId !=LEGACY_NODE_ID)){
				return resultId;
			}
		}else{
			return 0;
		}
	}
	return 0;
} //UpdateMatching


inline bool MatchingGraphNode::RecvUpdateMatching(const NodeIdType& recvTargetId, const shared_ptr<VehicleInformation>& initMgcInfoPtr){
    typedef map<NodeIdType, shared_ptr<MatchingGraphCandidate> >::iterator IterType;
	NodeIdType m_targetId;
	float tmpLikelihood = 0;
	shared_ptr<VehicleInformation> tmpInfoPtr;
	for(IterType t_iter = matchingCandidateList.begin(); (t_iter != matchingCandidateList.end()); t_iter++){
		m_targetId = (*t_iter).first;
		shared_ptr<MatchingGraphCandidate> targetCandidateInfo = (*t_iter).second;
		tmpLikelihood = targetCandidateInfo->likelihood;
		if((tmpLikelihood == FLOAT_MAX)&&(m_targetId == recvTargetId)){
			initMgcInfoPtr->mgInfoId = targetId;
			initMgcInfoPtr->matchingYudo = FLOAT_MAX;
			if(targetCandidateInfo->mgcInfoPtr == nullptr){
				targetCandidateInfo->mgcInfoPtr = initMgcInfoPtr;
			}
			return true;
		}
	}
	return false;
} //RecvUpdateMatching


inline void MatchingGraphNode::CurrentComplement(const TimeType& currentTime){
	//速度が測定された場合には補間を行わない
	if((currentVelocity.x != FLOAT_MAX)&&(currentVelocity.y != FLOAT_MAX)){
		xAbsolutePosition = xAbsolutePosition + (float)0.1 * currentVelocity.x;
		yAbsolutePosition = yAbsolutePosition + (float)0.1 * currentVelocity.y;

		if(currentTime == vectorTime){
			estPositionError = estPositionError + (float)ESTIMATTION_SPEED_ERROR;
		}else{
			estPositionError = estPositionError + (float)BRAKING_SPEED / (2 * (float)sqrt(10.0));
		}
	}else{
		estPositionError = estPositionError + (float)BRAKING_SPEED / (2 * (float)sqrt(10.0));
	}
}

inline void MatchingGraphNode::CurrentRelativeComplement(const float& myVelocityX, const float& myVelocityY){
	if((currentVelocity.x != FLOAT_MAX)&&(currentVelocity.y != FLOAT_MAX)){
		if((myVelocityX != FLOAT_MAX)&&(myVelocityY != FLOAT_MAX)){
			xRelativePosition = xRelativePosition + (float)0.1 * ( currentVelocity.x - myVelocityX);
			yRelativePosition = yRelativePosition + (float)0.1 * (currentVelocity.y - myVelocityY);
		}
	}
}

inline void MatchingGraphNode::UpdateVehicleInformation(){
	typedef map<NodeIdType, shared_ptr<MatchingGraphCandidate> > ::iterator mCandIterType;
	NodeIdType tmpNodeId;
	shared_ptr<MatchingGraphCandidate> tmpMatchingCand;
	shared_ptr<VehicleInformation> tmpData;
	for(mCandIterType m_iter = matchingCandidateList.begin(); m_iter != matchingCandidateList.end(); m_iter++){
		tmpNodeId = (*m_iter).first;
		tmpMatchingCand = (*m_iter).second;
		tmpData = tmpMatchingCand->mgcInfoPtr;
		if(tmpData == nullptr)continue;
		if(tmpData->sourceNodeId != sourceId)continue;
		tmpData->xPositionMeters = xAbsolutePosition;
		tmpData->yPositionMeters = yAbsolutePosition;
		tmpData->estPositionError = estPositionError;
	}
}

inline void  MatchingGraphNode::MakePositionCandidate(
		const TimeType& pcSensedTime,
		const float& pcXPositionMeters,		const float& pcYPositionMeters,
		const float& pcPositionError,
		const NodeIdType& pcNodeId,
		PositionCandidate& resultCandidate, 
		const TimeType& nowTime
		){
	TimeType resultTime = pcSensedTime;
	float resultXPosition = pcXPositionMeters;
	float resultYPosition = pcYPositionMeters;
	float resultPositionError = pcPositionError;
    typedef map<TimeType, shared_ptr<VelocityInformation> >::iterator 	VelocityIter;
    bool returnFlag = false;
//    TimeType lastTime = sensedTime;	//現在の時刻
    TimeType lastTime = nowTime;	//現在の時刻
	
	if(lastTime == pcSensedTime){
		resultCandidate = PositionCandidate(
			resultTime,
			resultXPosition,
			resultYPosition,
			resultPositionError,
			pcNodeId
		);
		return;
	}
	assert(lastTime > pcSensedTime);

	TimeType beginTime = pcSensedTime + 100*MILLI_SECOND;
	TimeType tmpTime_velo = 0;
	TimeType tmpTime_curr = pcSensedTime;
	VelocityIter viter = velocityInfoTable.find(beginTime);
	VectorType tmpVelocity;
	tmpVelocity.x = FLOAT_MAX;
	tmpVelocity.y = FLOAT_MAX;

	float tmpVectorError = 0;

	if(viter == velocityInfoTable.end()){
		//beginTimeの速度が存在しないときは解候補生成せずに終了
		returnFlag = false;
	}else{
		returnFlag = true;
		shared_ptr<VelocityInformation> tmpVelocityInfo = (*viter).second;
		tmpTime_velo = tmpVelocityInfo->sensedTime;
		tmpTime_curr = tmpTime_curr + 100*MILLI_SECOND;	//ここまで更新された
		resultXPosition = resultXPosition + tmpVelocityInfo->velocity.x  / 10;
		resultYPosition = resultYPosition + tmpVelocityInfo->velocity.y  / 10;
		resultPositionError = resultPositionError + (float)ESTIMATTION_SPEED_ERROR;
		tmpVelocity = tmpVelocityInfo->velocity;
		tmpVectorError = 0;

		while(tmpTime_velo < lastTime){
			++viter;
			if(viter == velocityInfoTable.end()){

				while((tmpTime_curr+100*MILLI_SECOND) <= lastTime){
				//速度が抜けている			
					if((tmpVelocity.x == FLOAT_MAX)&&(tmpVelocity.y == FLOAT_MAX)){
						//近い速度がない場合は解の候補を生成しない
						returnFlag = false;
						resultCandidate = PositionCandidate(
							resultTime,
							resultXPosition,
							resultYPosition,
							-1,
							pcNodeId
						);
						return;
					}
					resultXPosition = resultXPosition + tmpVelocity.x  / 10;
					resultYPosition = resultYPosition + tmpVelocity.y  / 10;
					tmpVectorError = tmpVectorError + (float)BRAKING_SPEED / (2 * (float)sqrt(10.0));
					resultPositionError = resultPositionError + tmpVectorError; //+(float)ESTIMATTION_SPEED_ERROR 
					tmpTime_curr = tmpTime_curr + 100 * MILLI_SECOND;
				}//while
				returnFlag = true;	
				break;
			}
			tmpVelocityInfo = (*viter).second;
			tmpTime_velo = tmpVelocityInfo->sensedTime;
			assert(tmpTime_velo <= lastTime);
			assert(tmpTime_curr < tmpTime_velo);

			while((tmpTime_curr+100*MILLI_SECOND) < tmpTime_velo){
				//速度が抜けている			
				if((tmpVelocity.x == FLOAT_MAX)&&(tmpVelocity.y == FLOAT_MAX)){
					//近い速度がない場合は解の候補を生成しない
					returnFlag = false;
					resultCandidate = PositionCandidate(
						resultTime,
						resultXPosition,
						resultYPosition,
						-1,
						pcNodeId
					);
					return;
				}
				resultXPosition = resultXPosition + tmpVelocity.x  / 10;
				resultYPosition = resultYPosition + tmpVelocity.y  / 10;
				tmpVectorError = tmpVectorError + (float)BRAKING_SPEED / (2 * (float)sqrt(10.0));
				resultPositionError = resultPositionError + tmpVectorError ;//(float)ESTIMATTION_SPEED_ERROR + 
				tmpTime_curr = tmpTime_curr + 100*MILLI_SECOND;
			}//while

			tmpTime_curr = tmpTime_curr + 100*MILLI_SECOND;
			assert(tmpTime_curr == tmpTime_velo);
			resultXPosition = resultXPosition + tmpVelocityInfo->velocity.x  / 10;
			resultYPosition = resultYPosition + tmpVelocityInfo->velocity.y  / 10;
			resultPositionError = resultPositionError + (float)ESTIMATTION_SPEED_ERROR;
			tmpVelocity = tmpVelocityInfo->velocity;
			tmpVectorError = 0;
		}
	}//if

	if(returnFlag){
		resultCandidate = PositionCandidate(
			resultTime,
			resultXPosition,
			resultYPosition,
			resultPositionError,
			pcNodeId
		);
	}else{
		resultCandidate = PositionCandidate(
			resultTime,
			resultXPosition,
			resultYPosition,
			-1,
			pcNodeId
		);
	}
} //CurrentComplement


inline void MatchingGraphNode::AddAllVelocityInfo(){
	typedef map<NodeIdType, shared_ptr<MatchingGraphCandidate> > ::iterator mCandIterType;
    typedef map<TimeType, shared_ptr<VelocityInformation> >::iterator VelocityIterType;
	VelocityIterType viter;
	shared_ptr<VelocityInformation> VeloInfo;
	NodeIdType tmpNodeId = 0;
	shared_ptr<MatchingGraphCandidate> tmpMatchingCand;
	shared_ptr<VehicleInformation> tmpData;

	//全消去
	velocityInfoTable.clear();

	for(mCandIterType m_iter = matchingCandidateList.begin(); m_iter != matchingCandidateList.end(); m_iter++){
		tmpNodeId = (*m_iter).first;
		if(tmpNodeId < LEGACY_NODE_ID){
			if(tmpNodeId != matchId)continue;
		}
		tmpMatchingCand = (*m_iter).second;
		tmpData = tmpMatchingCand->mgcInfoPtr;
		if(tmpData == nullptr) continue;
		viter = tmpData->velocityInfoTable.begin();
		while(viter != tmpData->velocityInfoTable.end()){
			VeloInfo = (*viter).second;
		    velocityInfoTable.insert(make_pair (VeloInfo->sensedTime, VeloInfo));
			++viter;
		}
	}
	CompleteVelocityInfo();
} //AddAllvelocityInfo


inline void MatchingGraphNode::CompleteVelocityInfo(){
    typedef map<TimeType, shared_ptr<VelocityInformation> >::iterator VelocityIterType;
	VelocityIterType viter_curr;
	VelocityIterType viter_next;
	shared_ptr<VelocityInformation> CurrInfo;
	shared_ptr<VelocityInformation> NextInfo;
	viter_curr = velocityInfoTable.begin();
	viter_next = viter_curr;
	float tmpvx = 0;
	float tmpvy = 0;
	TimeType tmptime = 0;
	long long int count = 0;
	float hokanx = 0;
	float hokany = 0;

	while(viter_curr != velocityInfoTable.end()){
		viter_next = viter_curr;
		++viter_next;
		if(viter_next == velocityInfoTable.end())break;
		CurrInfo = (*viter_curr).second;
		NextInfo = (*viter_next).second;
		assert((*viter_curr).first < (*viter_next).first);

		if((CurrInfo->sensedTime + 100*MILLI_SECOND) != NextInfo->sensedTime){
			//速度が足りない

			long long int counttmp2 = (NextInfo->sensedTime / 100 ) /MILLI_SECOND; 
			long long int counttmp3 = (CurrInfo->sensedTime / 100) /MILLI_SECOND;
			count = counttmp2 - counttmp3;
			assert(count > 0);
			hokanx = (NextInfo->velocity.x - CurrInfo->velocity.x) / count;
			hokany = (NextInfo->velocity.y - CurrInfo->velocity.y) / count;
			tmpvx = CurrInfo->velocity.x + hokanx;
			tmpvy = CurrInfo->velocity.y + hokany;
			tmptime = CurrInfo->sensedTime + 100*MILLI_SECOND;

			while(tmptime < NextInfo->sensedTime){
				AddVelocityInformation(
					VectorType(tmpvx, tmpvy),
					tmptime);
				tmptime = tmptime + 100*MILLI_SECOND;
				tmpvx = tmpvx + hokanx;
				tmpvy = tmpvy + hokany;
			}
		}
		viter_curr = viter_next;
	}
} //CompleteVelocityInfo


inline void MatchingGraphNode::Estimate(const TimeType& currentTime, int& r_gps_num, int& r_sensor_num){
	typedef map<NodeIdType, shared_ptr<MatchingGraphCandidate> > ::iterator mCandIterType;
	typedef map<TimeType, shared_ptr<GPSInformation> >::iterator GpsIterType;
    typedef map<TimeType, shared_ptr<VelocityInformation> >::iterator VelocityIterType;
    typedef multimap<TimeType, shared_ptr<SensorInformation> >::iterator SensorIterType;

	float sum_weight = 0;
	float sum_all_x= 0;
	float sum_all_y= 0;
	int sum_num = 0;
	int gps_num = 0;
	int sensor_num = 0;

	int try_gps_num = 0;
	int try_sensor_num = 0;
	r_gps_num = 0;
	r_sensor_num = 0;

	NodeIdType tmpNodeId = 0;
	shared_ptr<MatchingGraphCandidate> tmpMatchingCand;
	shared_ptr<VehicleInformation> tmpData;
	PositionCandidate tmp;
	GpsIterType gpsiter;
	shared_ptr<GPSInformation> GpsInfo;
	SensorIterType siter;
	shared_ptr<SensorInformation> SensorInfo;

	for(mCandIterType m_iter = matchingCandidateList.begin(); m_iter != matchingCandidateList.end(); m_iter++){
		tmpNodeId = (*m_iter).first;
		tmpMatchingCand = (*m_iter).second;
		tmpData = tmpMatchingCand->mgcInfoPtr;
		if(tmpData == nullptr)continue;
		if(tmpNodeId < LEGACY_NODE_ID){
			if((tmpNodeId != sourceId)&&(tmpNodeId != matchId))continue;
		}

		gpsiter = tmpData->gpsInfoTable.begin();
		while(gpsiter != tmpData->gpsInfoTable.end()){
			GpsInfo = (*gpsiter).second;
#ifdef ESTIMATE_DEBUG
			if((sourceId ==ESTIMATE_DEBUG_SOURCE)&&(targetId ==ESTIMATE_DEBUG_TARGET)){
				cout<<"DEBUG_G:"<<sourceId<<":"<<GpsInfo->xPositionMeters<<", "<<GpsInfo->yPositionMeters<<endl;
			}
#endif
			MakePositionCandidate(
				GpsInfo->sensedTime,		
				GpsInfo->xPositionMeters,
				GpsInfo->yPositionMeters,
				ESTIMATTION_GPS_POSITION_ERROR,
				tmpData->sourceNodeId,
				tmp,
				currentTime
			);
#ifdef ESTIMATE_DEBUG
			if((sourceId ==ESTIMATE_DEBUG_SOURCE)&&(targetId ==ESTIMATE_DEBUG_TARGET)){
				tmp.PrintOut();
			}
#endif
			++try_gps_num;
			if((tmp.xPositionMeters < AREA_RANGE)&&(tmp.yPositionMeters < AREA_RANGE)){
				if(tmp.positionError >= 0){
					++sum_num;
					++gps_num;
					sum_weight = sum_weight + 1/(tmp.positionError);
					sum_all_x = sum_all_x + tmp.xPositionMeters/(tmp.positionError);
					sum_all_y = sum_all_y + tmp.yPositionMeters/(tmp.positionError);
				}
			}else{

			}
			++gpsiter;
		}
		
		siter = tmpData->sensorInfoTable.begin();
		while(siter != tmpData->sensorInfoTable.end()){
			SensorInfo = (*siter).second;
#ifdef ESTIMATE_DEBUG
			if((sourceId ==ESTIMATE_DEBUG_SOURCE)&&(targetId ==ESTIMATE_DEBUG_TARGET)){
			cout<<"DEBUG_S:"<<sourceId<<":"<<SensorInfo->xPositionMeters<<", "<<SensorInfo->yPositionMeters<<endl;
			}
#endif
			MakePositionCandidate(
				SensorInfo->sensedTime,		
				SensorInfo->xPositionMeters,
				SensorInfo->yPositionMeters,
				ESTIMATTION_RADAR_POSITION_ERROR,
				SensorInfo->observerNodeId,
				tmp,
				currentTime);
#ifdef ESTIMATE_DEBUG
			if((sourceId ==ESTIMATE_DEBUG_SOURCE)&&(targetId ==ESTIMATE_DEBUG_TARGET)){
				tmp.PrintOut();
			}
#endif
			++try_sensor_num;
			if((tmp.xPositionMeters < AREA_RANGE)&&(tmp.yPositionMeters < AREA_RANGE)){
				if(tmp.positionError >= 0){
					++sum_num;
					++sensor_num;
					sum_weight = sum_weight + 1/(tmp.positionError);
					sum_all_x = sum_all_x + tmp.xPositionMeters/(tmp.positionError);
					sum_all_y = sum_all_y + tmp.yPositionMeters/(tmp.positionError);
				}
			}else{

			}
			++siter;
		}
	}
#ifdef ESTIMATE_DEBUG
			if((sourceId ==ESTIMATE_DEBUG_SOURCE)&&(targetId ==ESTIMATE_DEBUG_TARGET)){
			cout<<"Estimate:"<<sum_num<<", "<<gps_num<<", "<<sensor_num<<endl;
		}
#endif
	//全消去
	velocityInfoTable.clear();

	if(sum_num > 0){
		float tmp2 = estPositionError;
		CurrentComplement(currentTime);
		
		float tmpEstPositionError = sqrt(sum_num / (sum_weight * sum_weight));
		if(estPositionError >= tmpEstPositionError){
			xAbsolutePosition = sum_all_x  / sum_weight;
			yAbsolutePosition = sum_all_y  / sum_weight;
			estPositionError = tmpEstPositionError;
			#ifdef ESTIMATE_DEBUG
			if((sourceId ==ESTIMATE_DEBUG_SOURCE)&&(targetId ==ESTIMATE_DEBUG_TARGET)){
				cout<<"Estimate_Plus:"<<gps_num<<", "<<sensor_num<<endl;
				cout<<"Estimate_Plus:"<<xAbsolutePosition<<", "<<yAbsolutePosition<<", "<<tmpEstPositionError<<", "<<estPositionError<<endl;
			}
			#endif
			r_gps_num = gps_num;
			r_sensor_num = sensor_num;
		}else{
			r_gps_num = gps_num;
			r_sensor_num = sensor_num;
			#ifdef ESTIMATE_DEBUG
			if((sourceId ==ESTIMATE_DEBUG_SOURCE)&&(targetId ==ESTIMATE_DEBUG_TARGET)){
				cout<<"Estimate_Minus:"<<gps_num<<", "<<sensor_num<<endl;
				cout<<"Estimate_Minus:"<<xAbsolutePosition<<", "<<yAbsolutePosition<<", "<<tmpEstPositionError<<", "<<estPositionError<<endl;
			}
			#endif
		}
#ifdef ESTIMATE_DEBUG
			if((sourceId ==ESTIMATE_DEBUG_SOURCE)&&(targetId ==ESTIMATE_DEBUG_TARGET)){
			cout<<"Estimate:"<<gps_num<<", "<<sensor_num<<endl;
			cout<<"Estimate:"<<xAbsolutePosition<<", "<<yAbsolutePosition<<", "<<estPositionError<<endl;
		}
#endif
	}else{
		CurrentComplement(currentTime);
	}
} //Estimate



//------------------------------------------------------------------------------------------
// VehicleSensors : GPS, Velocity, LRS すべての情報をここに含んでいる
//------------------------------------------------------------------------------------------
class VehicleSensors{
public:
    VehicleSensors(
        const NodeIdType& initNodeId,
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const RandomNumberGeneratorSeedType& nodeSeed,
        const size_t& interfaceIndex);
    ~VehicleSensors() {}

    shared_ptr<VehicleInformation>& GetSelfPosition(const TimeType& targetTime);
    map<NodeIdType, shared_ptr<VehicleInformation> >& GetNeighborsPositions(const TimeType& targetTime);
	double GetMovingDirection();
    float radarSensingDistanceMeters;

private:
    NodeIdType nodeId;
    ifstream* inputFilePtr;
    
    TimeType lastSensingTime;
    shared_ptr<VehicleInformation> currentSelfInfoPtr;
    shared_ptr<VehicleInformation> oldSelfInfoPtr;      //現在時刻における情報更新のために前時刻の情報が必要
    map<NodeIdType, shared_ptr<VehicleInformation> >* currentNeighborsInfoPtr;
    map<NodeIdType, shared_ptr<VehicleInformation> >* oldNeighborsInfoPtr;   //現在時刻における情報更新のために前時刻の情報が必要
    
    float gpsErrorOffsetX, gpsErrorOffsetY;

	float gpsPositionErrorMeters;
    float radarPositionErrorMeters;
    float radarSensingOrientationDegrees;
    float radarSensingSquaredDistanceMeters;
    int   radarSensingArea;
	float radarVelocityError;
    float radarErrorOffsetX;
    float radarErrorOffsetY;

	//新しいGPSモデルのための変数：現在は全く利用していない
	float oldGpsErrorOffsetX, oldGpsErrorOffsetY;
	float baseGpsErrorOffsetX, baseGpsErrorOffsetY;
	float planetCounter; //信号受信衛星数
	float alphaAvg;
    
	double movingDirection;

    VectorType defaultVehicleSize;

    RandomNumberGenerator aRandomNumberGenerator;
    static const long int SEED_HASH = 453905;

    void UpdateData(const TimeType& targetTime);
    
    void UpdateVelocity(
        VectorType& newVelocity,
        const float& oldXPos,
        const float& oldYPos,
        const float& newXPos,
        const float& newYPos);
    
    void UpdateAcceleration(
        VectorType& newAcceleration,
        const VectorType& oldVelocity,
        const VectorType& newVelocity) const;
    
    void UpdateHeading(
        const float& oldXPos,
        const float& oldYPos,
        const float& newXPos,
        const float& newYPos);

}; //Class VehicleSensors


//------------------------------------------------------------------------
// センサログの読み込み，情報更新についての詳細
//------------------------------------------------------------------------
inline
VehicleSensors::VehicleSensors(
    const NodeIdType& initNodeId,
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const RandomNumberGeneratorSeedType& nodeSeed,
    const size_t& interfaceIndex)
    :
    nodeId(initNodeId),
    lastSensingTime(INFINITE_TIME),
    aRandomNumberGenerator(HashInputsToMakeSeed(nodeSeed, interfaceIndex, SEED_HASH)),
    defaultVehicleSize((float)5,(float)1.8)
{
    //センサログの処理
    std::ostringstream inputStream;
    
    assert(theParameterDatabaseReader.ParameterExists("SENSOR-LOG-FILE"));
    string sensorLogFile = theParameterDatabaseReader.ReadString("SENSOR-LOG-FILE");

    
    //絶対パス指定としているのでシミュレーション実行時注意
    inputStream << sensorLogFile << "SensorLog-" << nodeId << ".log";
    string inputFileName = inputStream.str();
    
    inputFilePtr = new ifstream(inputFileName.c_str());

    if (!inputFilePtr->good()) {
        cerr << "Could Not open preprocessed sensor log file: " << inputFileName << endl;
        exit(1);
    }
    
    //configファイルから設定事項をread
    assert(theParameterDatabaseReader.ParameterExists("GPS-POSITION-ERROR-METERS"));
    gpsPositionErrorMeters = (float)(theParameterDatabaseReader.ReadDouble("GPS-POSITION-ERROR-METERS"));

    assert(theParameterDatabaseReader.ParameterExists("RADAR-POSITION-ERROR-METERS"));
    radarPositionErrorMeters = (float)(theParameterDatabaseReader.ReadDouble("RADAR-POSITION-ERROR-METERS"));

    assert(theParameterDatabaseReader.ParameterExists("RADAR-SENSING-ORIENTATION-DEGREES"));
    radarSensingOrientationDegrees = (float)(theParameterDatabaseReader.ReadDouble("RADAR-SENSING-ORIENTATION-DEGREES"));
    //assert((radarSensingOrientationDegrees>0)&&(radarSensingOrientationDegrees<=90));
    
    assert(theParameterDatabaseReader.ParameterExists("RADAR-SENSING-AREA"));
    radarSensingArea = (int)(theParameterDatabaseReader.ReadInt("RADAR-SENSING-AREA"));

    assert(theParameterDatabaseReader.ParameterExists("RADAR-SENSING-DISTANCE-METERS"));
    radarSensingDistanceMeters = (float)(theParameterDatabaseReader.ReadDouble("RADAR-SENSING-DISTANCE-METERS"));
    radarSensingSquaredDistanceMeters = (float)(radarSensingDistanceMeters * radarSensingDistanceMeters);

    assert(theParameterDatabaseReader.ParameterExists("RADAR-VELOCITY-ERROR-METER-PER-SECOND"));
    radarVelocityError = (float)(theParameterDatabaseReader.ReadDouble("RADAR-VELOCITY-ERROR-METER-PER-SECOND"));
    

    //calculate location detection error by GPS; this value is fixed for each vehicle.
    //誤差分布の与え方
	#ifdef GPS_NORMAL
        //基本的にはこちらの誤差分布で
        float errorMeters = (float)(gpsPositionErrorMeters * aRandomNumberGenerator.GenerateRandomDouble());
        float errorRadians = (float)(2 * PI * aRandomNumberGenerator.GenerateRandomDouble());
        gpsErrorOffsetX = sin(errorRadians) * errorMeters;
        gpsErrorOffsetY = cos(errorRadians) * errorMeters;
	#else
        float errorMeters = (float)normalDistRandom(BASE_GPS_AVG, BASE_GPS_DEVIATION);
        float errorRadians = (float)(2*PI*rand()/RAND_MAX);
        
        baseGpsErrorOffsetX = cos(errorRadians) * errorMeters;
        baseGpsErrorOffsetY = sin(errorRadians) * errorMeters;
        oldGpsErrorOffsetX = -1;
        oldGpsErrorOffsetY = -1;
        errorMeters = (float)normalDistRandom(0, ZIG_GPS_DEVIATION);
        errorRadians = (float)(2*PI*rand()/RAND_MAX);
        float diffGpsErrorOffsetX = baseGpsErrorOffsetX + cos(errorRadians) * errorMeters;
        float diffGpsErrorOffsetY = baseGpsErrorOffsetY + sin(errorRadians) * errorMeters;
        gpsErrorOffsetX = diffGpsErrorOffsetX;
        gpsErrorOffsetY = diffGpsErrorOffsetY;
        planetCounter = INTERVAL_PLANET;
        alphaAvg = (float)ALPHA;
	#endif


	//速度誤差の与え方を変更，This value is fixed for each vehicle.
	errorMeters = (float)(radarVelocityError * aRandomNumberGenerator.GenerateRandomDouble());
    errorRadians = (float)(2 * PI * aRandomNumberGenerator.GenerateRandomDouble());
    radarErrorOffsetX = sin(errorRadians) * errorMeters;
    radarErrorOffsetY = cos(errorRadians) * errorMeters;
    
    //ポインタ初期化
	oldNeighborsInfoPtr = nullptr;
	currentNeighborsInfoPtr = nullptr;
	
	movingDirection = 0;
} //VehicleSensors


inline
double VehicleSensors::GetMovingDirection(){
	return movingDirection;
} //GetMovingDirection


inline
shared_ptr<VehicleInformation>&
VehicleSensors::GetSelfPosition(const TimeType& targetTime)
{
    (*this).UpdateData(targetTime);
    return currentSelfInfoPtr;

} //GetSelfPosition


inline
map<NodeIdType, shared_ptr<VehicleInformation> >& 
VehicleSensors::GetNeighborsPositions(const TimeType& targetTime)
{
    (*this).UpdateData(targetTime);
    return (*currentNeighborsInfoPtr);

} //GetNeighborsPositions


inline
void VehicleSensors::UpdateVelocity(
        VectorType& newVelocity,
        const float& oldXPos,
        const float& oldYPos,
        const float& newXPos,
        const float& newYPos)
{
    //radarError --> 速度誤差
    //速度誤差を与える
    float errorMeters = (float)(radarVelocityError * aRandomNumberGenerator.GenerateRandomDouble());
    float errorRadians = (float)(2 * PI * aRandomNumberGenerator.GenerateRandomDouble());
    radarErrorOffsetX = sin(errorRadians) * errorMeters;
    radarErrorOffsetY = cos(errorRadians) * errorMeters;
	//add error
    newVelocity.x = (newXPos - oldXPos) * 10 + radarErrorOffsetX; // means /0.1S(=100MS)
    newVelocity.y = (newYPos - oldYPos) * 10 + radarErrorOffsetY; // measn /0.1S(=100MS)

	#ifdef SENSOR_DEBUG
        cout<<"Speed Error: "<<radarVelocityError<<endl;//temp
        cout<<"Speed Error offset: "<<radarErrorOffsetX<<", "<<radarErrorOffsetY<<""<<endl;//temp
        cout<<"Speed Before: ("<<(newXPos - oldXPos) * 10<<", "<<(newXPos - oldXPos) * 10<<")"<<endl;//temp
        cout<<"Speed After: ("<<newVelocity.x<<", "<<newVelocity.y<<")"<<endl;//temp
	#endif
} //UpdateVelocity


inline
void VehicleSensors::UpdateAcceleration(
        VectorType& newAcceleration,
        const VectorType& oldVelocity,
        const VectorType& newVelocity) const
{
	if((oldVelocity.x != FLOAT_MAX)&&(oldVelocity.y != FLOAT_MAX)){
	    newAcceleration.x = (newVelocity.x - oldVelocity.x) * 10 / INTERVAL_TIMES; // means /0.1S(=100MS)
	    newAcceleration.y = (newVelocity.y - oldVelocity.y) * 10 / INTERVAL_TIMES; // means /0.1S(=100MS)
	}
} //UpdateAcceleration//

inline
void VehicleSensors::UpdateHeading(
        const float& oldXPos,
        const float& oldYPos,
        const float& newXPos,
        const float& newYPos)
{

    float diffx = newXPos - oldXPos;
    float diffy = newYPos - oldYPos;
    
    double thetaRadian = atan(diffx/diffy);
    double theta = thetaRadian * 180 / PI;
    
    movingDirection = theta;
} //UpdateHeading



//------------------------------------------------------------------------------------------
// VehicleSensors::UpdateData センサログ情報からデータを読み取り，センサ検出車両のデータを保持できるようにする
//------------------------------------------------------------------------------------------
inline
void VehicleSensors::UpdateData(const TimeType& targetTime) {
    // Unit: targetTime, sensingTime = nano second
    
    // 更新しない
    if (targetTime == lastSensingTime) { return; }
    
    //今回の手法でGPS情報を1秒の単位で測位することが必要かは検討課題（今回は位置推定メインではないので）
	//1秒に1回のみGPS情報を更新するためのコード
	long long int tmpMS =(targetTime / MILLI_SECOND) / 100;
	int slotCount = tmpMS % 10;

    //センサログファイルのメイン処理
    while(!inputFilePtr->eof()) {
        //parse a line
        string aLine;
        getline((*inputFilePtr), aLine);

        if (IsAConfigFileCommentLine(aLine)) { continue; }

        DeleteTrailingSpaces(aLine);
        istringstream lineStream(aLine);
        string firstNodeIdString;
        lineStream >> firstNodeIdString;
        int firstNodeId = atoi(firstNodeIdString.c_str());

        if (firstNodeId != nodeId) {
            cerr << "Error: Bad preprocessed sensor log file line: " << aLine << ": "<< firstNodeId << endl;
            exit(1);
        }
        
        string sensingTimeString;
        TimeType sensingTime;
        lineStream >> sensingTimeString; //sensed time: nano second.
        
        bool success;
        ConvertStringToTime(sensingTimeString, sensingTime, success);
        if (!success) {
            cerr << "Error: Bad preprocessed sensor log file line: " << aLine << ": "<< sensingTimeString << endl;
            exit(1);
        }//if//

        if (abs(sensingTime - targetTime) >= 100000000) { continue; } //時間同期していないので，関係のない時間のセンサログを読み飛ばす．BSMがシミュレーション開始直後から送信されていないので．
        
        //get self position
        string xPosString, yPosString;

        lineStream >> xPosString;
        float xPosOfSelf;
        double xPosOfSelfDouble;
        ConvertStringToDouble(xPosString, xPosOfSelfDouble, success);
        if (!success) {
            cerr << "Error: Bad preprocessed sensor log file line: " << aLine << ": "<< xPosString << endl;
            exit(1);
        }//if//
        xPosOfSelf = (float)(xPosOfSelfDouble);

        lineStream >> yPosString;
        float yPosOfSelf;
        double yPosOfSelfDouble;
        ConvertStringToDouble(yPosString, yPosOfSelfDouble, success);
        if (!success) {
            cerr << "Error: Bad preprocessed sensor log file line: " << aLine << ": "<< yPosString << endl;
            exit(1);
        }//if//
        yPosOfSelf = (float)(yPosOfSelfDouble);

        string orientationString;
        lineStream >> orientationString;
        double sourceOrientation;
        ConvertStringToDouble(orientationString, sourceOrientation, success);
        if (!success) {
            cerr << "Error: Bad preprocessed sensor log file line: " << aLine << ": "<< orientationString << endl;
            exit(1);
        }//if//
		
		movingDirection = sourceOrientation; //進行方向  0度（上），90度（右），180度（下），270度（左）
        int identifierFlag = IDENTIFIER_SENSOR;  //情報種別

        VectorType newVelocity = VectorType(FLOAT_MAX, FLOAT_MAX);
		VectorType newAcceleration = VectorType(FLOAT_MAX, FLOAT_MAX);

        //前時刻の位置情報から速度・加速度をアップデート
        if (oldSelfInfoPtr != nullptr) {    //all except first sensing
            (*this).UpdateVelocity(
                newVelocity, 
                oldSelfInfoPtr->xExactPosition, 
                oldSelfInfoPtr->yExactPosition, 
                xPosOfSelf,
                yPosOfSelf);
            (*this).UpdateAcceleration(newAcceleration, oldSelfInfoPtr->velocity, newVelocity);
            (*this).UpdateHeading(oldSelfInfoPtr->xExactPosition, oldSelfInfoPtr->yExactPosition,xPosOfSelf,yPosOfSelf);
        }//if
 
        float newXPos = 0;
        float newYPos = 0;

        if((slotCount == 0)||(oldSelfInfoPtr == nullptr)){
        #ifdef GPS_NORMAL
            //Gpsによる位置取得（1秒毎）
            float errorMeters2  = (float)(gpsPositionErrorMeters * aRandomNumberGenerator.GenerateRandomDouble());
            float errorRadians2 = (float)(2 * PI * aRandomNumberGenerator.GenerateRandomDouble());
            gpsErrorOffsetX = sin(errorRadians2) * errorMeters2;
            gpsErrorOffsetY = cos(errorRadians2) * errorMeters2;
        #else
            //以下の処理は実行しない
            /*
            float diffGpsErrorOffsetX;
            float diffGpsErrorOffsetY;
            bool tmpflag = false;
            if((oldGpsErrorOffsetX == -1)&&(oldGpsErrorOffsetY == -1)){
                //一回目
                tmpflag = true;
            }
            oldGpsErrorOffsetX = gpsErrorOffsetX;
            oldGpsErrorOffsetY = gpsErrorOffsetY;
            if(planetCounter <= 0){
                float errorMeters2 = (float)normalDistRandom(BASE_GPS_AVG, BASE_GPS_DEVIATION);
                float errorRadians2 = (float)(2*PI*rand()/RAND_MAX);
                baseGpsErrorOffsetX = cos(errorRadians2) * errorMeters2;
                baseGpsErrorOffsetY = sin(errorRadians2) * errorMeters2;
                planetCounter = INTERVAL_PLANET;
            }else{
                --planetCounter;
            }
            float errorMeters2 = (float)normalDistRandom(0, ZIG_GPS_DEVIATION);
            float errorRadians2 = (float)(2*PI*rand()/RAND_MAX);
            diffGpsErrorOffsetX = cos(errorRadians2) * errorMeters2;
            diffGpsErrorOffsetY = sin(errorRadians2) * errorMeters2;
            if(tmpflag){
                gpsErrorOffsetX = baseGpsErrorOffsetX + diffGpsErrorOffsetX;
                gpsErrorOffsetY = baseGpsErrorOffsetY + diffGpsErrorOffsetY;
            }else{
                gpsErrorOffsetX = (baseGpsErrorOffsetX + diffGpsErrorOffsetX) * alphaAvg + oldGpsErrorOffsetX * (1 - alphaAvg);
                gpsErrorOffsetY = (baseGpsErrorOffsetY + diffGpsErrorOffsetY) * alphaAvg + oldGpsErrorOffsetY * (1 - alphaAvg);
            }
            */
        #endif
            newXPos = xPosOfSelf + gpsErrorOffsetX; //x真値+オフセット
            newYPos = yPosOfSelf + gpsErrorOffsetY; //y真値+オフセット
        }else{
            //速度情報を利用して補完
            newXPos = oldSelfInfoPtr->xPositionMeters + newVelocity.x / 10;
            newYPos = oldSelfInfoPtr->yPositionMeters + newVelocity.y / 10;
        }
        
        #ifdef SENSOR_DEBUG
            cout<<"GPS Error offset: "<<gpsErrorOffsetX<<", "<<gpsErrorOffsetY<<", " <<newVelocity.x <<","<<newVelocity.y<<","<< baseGpsErrorOffsetX<<","<<baseGpsErrorOffsetY<<","<<endl;
            cout<<"GPS Before: ("<<xPosOfSelf<<", "<<yPosOfSelf<<")"<<endl;//temp
            cout<<"GPS After: ("<<newXPos<<", "<<newYPos<<")"<<endl;//temp
        #endif

        
        int msgOverhead = 0;
        //現在時刻の車両情報更新
        currentSelfInfoPtr = 
            shared_ptr<VehicleInformation>(new VehicleInformation(
                 nodeId,
                 firstNodeId,
                 targetTime,
                 defaultVehicleSize,
                 xPosOfSelf,
                 yPosOfSelf,
                 newXPos,
                 newYPos,
          		 ESTIMATTION_RADAR_POSITION_ERROR,
                 newVelocity,
                 newAcceleration,
                 identifierFlag,
                 msgOverhead));

        oldSelfInfoPtr = shared_ptr<VehicleInformation>(new VehicleInformation(*currentSelfInfoPtr));
   
        //以下，センサ検出車両についての情報更新
        currentNeighborsInfoPtr = new map<NodeIdType, shared_ptr<VehicleInformation> >;

        //calculate location detection error by Radar
        float errorMeters = (float)(radarPositionErrorMeters * aRandomNumberGenerator.GenerateRandomDouble());
        float errorRadians = (float)(2 * PI * aRandomNumberGenerator.GenerateRandomDouble());
        float errorXofNeighbor = sin(errorRadians) * errorMeters;
        float errorYofNeighbor = cos(errorRadians) * errorMeters;

		#ifdef SENSOR_DEBUG
            cout<<"Radar Error offset: "<<errorXofNeighbor<<", "<<errorYofNeighbor<<""<<endl;//temp
		#endif
		
        string targetNodeIdString;
        int targetNodeId;
        //get neibors positions.
        while(lineStream.good()) {
      
            lineStream >> targetNodeIdString;
            targetNodeId = atoi(targetNodeIdString.c_str());
            
            int isLegacy = targetNodeId % 10;
            
#ifdef RATIO_HIGH
            if (isLegacy == 2 || isLegacy == 4 || isLegacy == 6 || isLegacy == 8 || isLegacy == 0) {
                targetNodeId += LEGACY_NODE_ID; //レガシー車両
                identifierFlag = IDENTIFIER_LEGACY;
            }
#endif

#ifdef RATIO_MIDDLE
            if (isLegacy == 2 || isLegacy == 5 || isLegacy == 8) {
                targetNodeId += LEGACY_NODE_ID; //レガシー車両
                identifierFlag = IDENTIFIER_LEGACY;
            }
#endif

#ifdef RATIO_LOW
            if (isLegacy == 5) {
                targetNodeId += LEGACY_NODE_ID; //レガシー車両
                identifierFlag = IDENTIFIER_LEGACY;
            }
#endif

            lineStream >> xPosString;
            float xPos;
            double xPosDouble;
            ConvertStringToDouble(xPosString, xPosDouble, success);
            if (!success) {
                cerr << "Error: Bad preprocessed sensor log file line: " << aLine << ": "<< xPosString << endl;
                exit(1);
            }//if//
            xPos = (float)(xPosDouble);

            lineStream >> yPosString;
            float yPos;
            double yPosDouble;
            ConvertStringToDouble(yPosString, yPosDouble, success);
            if (!success) {
                cerr << "Error: Bad preprocessed sensor log file line: " << aLine << ": "<< yPosString << endl;
                exit(1);
            }//if//
            yPos = (float)(yPosDouble);
            //now we do not use this value
            lineStream >> orientationString;
            double targetOrientation;
            ConvertStringToDouble(orientationString, targetOrientation, success);
            if (!success) {
                cerr << "Error: Bad preprocessed sensor log file line: " << aLine << ": "<< orientationString << endl;
                exit(1);
            }//if//

            
            //add position error
           	if(slotCount == 0){
        		newXPos = xPos + errorXofNeighbor;
    	     	newYPos = yPos + errorYofNeighbor;
        	
        	}else{
				//基準スロット以外は観測車両の速度センサの誤差の影響を受けると思ったが，
				//この速度は観測車両の相対的な位置関係から導出されるものなので，
				//影響を受けない
        		newXPos = xPos + errorXofNeighbor;// + newVelocity.x / 10;
		    	newYPos = yPos + errorYofNeighbor;// + newVelocity.y / 10;
        	}
            
			//センサ検出車両の相対位置
			float diffnewXPos = newXPos - xPosOfSelf;
			float diffnewYPos = newYPos - yPosOfSelf;
            
            //検出可能範囲外の車両であれば何もしない．
            VectorType relCoord = VectorType(diffnewXPos, diffnewYPos);
            float thetaRad = sourceOrientation*3.14/180;
            VectorType movingDir = VectorType(sin(thetaRad), cos(thetaRad));
            //if (!isDetectableBySensor(relCoord, movingDir)) { continue; }
			            
			#ifdef SENSOR_DEBUG
                cout<<targetTime<<": "<<nodeId<<"->"<<targetNodeId<<" detectable!"<<endl;
                cout<<"Radar Before: ("<<xPos<<", "<<yPos<<")"<<endl;//temp
                cout<<"Radar After: ("<<diffnewXPos<<", "<<diffnewYPos<<")"<<endl;//temp
                cout<<"Radar Total: ("<<newXPos<<", "<<newYPos<<")"<<endl;//temp
			#endif
            
            newVelocity.clear();
            newAcceleration.clear();

			newVelocity.x = FLOAT_MAX;
			newVelocity.y = FLOAT_MAX;
			newAcceleration.x = FLOAT_MAX;
			newAcceleration.y = FLOAT_MAX;

            //Update velocity and acceleration
            typedef map<NodeIdType, shared_ptr<VehicleInformation> >::iterator IterType;
            if (oldNeighborsInfoPtr != nullptr){
                IterType iter = oldNeighborsInfoPtr->find(targetNodeId);
                if (iter != oldNeighborsInfoPtr->end()) {
                    shared_ptr<VehicleInformation> oldNeighborDataPtr = (*iter).second;
                     assert(targetTime == (oldNeighborDataPtr->sensedTime + INTERVAL_SEND * MILLI_SECOND));
                   
                   //正確な速度が測定できると仮定するなら以下をコメントアウト
                   //(*this).UpdateVelocity(
                   //     newVelocity,
                   //     oldNeighborDataPtr->xExactPosition,
                   //     oldNeighborDataPtr->yExactPosition,
                   //     xPos,
                   //     yPos);
                   (*this).UpdateAcceleration(newAcceleration, oldNeighborDataPtr->velocity, newVelocity);
                   (*this).UpdateHeading(oldSelfInfoPtr->xExactPosition, oldSelfInfoPtr->yExactPosition, xPos, yPos);

				  //測定された位置から速度を求める場合
				  newVelocity.x = (newXPos - oldNeighborDataPtr->xExactPosition)*10;
				  newVelocity.y = (newYPos - oldNeighborDataPtr->yExactPosition)*10;
                  #ifdef SENSOR_DEBUG
                    cout<<"Speed Old: "<<oldNeighborDataPtr->xExactPosition<<", "<<oldNeighborDataPtr->yExactPosition<<", "<<newXPos << ", " << newYPos <<endl;//temp
                    cout<<"Speed New: "<<newVelocity.x<<", "<<newVelocity.y<<endl;
                  #endif
                }//if//
            }//if//
            
            int msgOverhead = 0;
            isLegacy = targetNodeId % 10;
            
#ifdef RATIO_HIGH
            if (isLegacy == 2 || isLegacy == 4 || isLegacy == 6 || isLegacy == 8 || isLegacy == 0) {
                identifierFlag = IDENTIFIER_LEGACY;
            } else {
                identifierFlag = IDENTIFIER_SENSOR;
            }
#endif
        
#ifdef RATIO_MIDDLE
            if (isLegacy == 2 || isLegacy == 5 || isLegacy == 8) {
                identifierFlag = IDENTIFIER_LEGACY;
            } else {
                identifierFlag = IDENTIFIER_SENSOR;
            }
#endif

#ifdef RATIO_LOW
            if (isLegacy == 5) {
                identifierFlag = IDENTIFIER_LEGACY;
            } else {
                identifierFlag = IDENTIFIER_SENSOR;
            }
#endif
            
            shared_ptr<VehicleInformation> 
                neighborDataPtr(
                  new VehicleInformation(
                            nodeId,
                            targetNodeId,
                            targetTime,
                            defaultVehicleSize,
                            xPos,
                            yPos,
                            diffnewXPos,
                            diffnewYPos,
               				ESTIMATTION_RADAR_POSITION_ERROR,
                            newVelocity,
                            newAcceleration,
                            identifierFlag,
                            msgOverhead));
            
            currentNeighborsInfoPtr->insert(make_pair(targetNodeId, neighborDataPtr)); //挿入
        }//while//

        lastSensingTime = targetTime; //センシング時間の更新
        //copy current(older) to old
	
		if (oldNeighborsInfoPtr != nullptr) { 
       	  //前スロットの車両情報ポインタの消去
       	  delete oldNeighborsInfoPtr;
       	}
        
        oldNeighborsInfoPtr = new map<NodeIdType, shared_ptr<VehicleInformation> >(*currentNeighborsInfoPtr);
        return;//get all info

    }//while/


	//周辺車両を1台も検出しなかった場合，なるべく早めに解放する
	currentSelfInfoPtr.reset();
	oldSelfInfoPtr.reset();
	if (currentNeighborsInfoPtr != nullptr) {
		delete currentNeighborsInfoPtr;
   	}

	if (oldNeighborsInfoPtr != nullptr) { 
   	  //  delete 
   	  delete oldNeighborsInfoPtr;
	}

} //UpdateData


} //Wave

#endif