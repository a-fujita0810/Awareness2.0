//
//  bsm_app.h
//  Awareness_1.0
//
//  Created by Atsushi Fujita on 2016/11/15.
//  Copyright © 2016年 Atsushi Fujita. All rights reserved.
//

#ifndef bsm_app_h
#define bsm_app_h


#include "vehicle_detection_sensor.h"

using std::ostringstream;
using std::cout;
using std::cerr;
using std::endl;
using std::setw;
//using std::tr1::shared_ptr;
//using std::tr1::enable_shared_from_this;


//--------------------------------------------------------------------------------------
// 以下の変数定義などは，ver1.3のままであるので適合しない可能性がある
//--------------------------------------------------------------------------------------


//_/_/提案手法モード選択用定義_/_/
//マッチングに関して
//#define IDEAL_MATCHING		//理想的なマッチングを行う
#define RADAR_MATCHING	//相対距離のみを用いてマッチングを行う
//#define VISION_MATCHING	//前後の車両だけIDが得られる
#ifdef VISION_MATCHING
	#define MAX_VISIBLE_RANGE_METERS  FLOAT_MAX
	#define MAX_VISIBLE_RANGE_DEGREES 2
#endif

//それ以外
#define SENDER_MATCH_CREATE	//車両情報を積極的に生成するモード（試験的）
//#define SENDER_TARGET_MATCH_CREATE
//#define MATCHING_MODE_0		//一致度とマッチングの誤差を考慮した場合
#define MATCHING_MODE_1		//一致度とマッチングの誤差を考慮した場合（観測可能範囲を考慮しない）
#define RECV_MATCHING_MODE	//隣接ノードからのマッチング結果の反映（尤度をより重くする）


//_/_/提案手法のパラメータ_/_/
//挙動を決定するパラメータ値
#define NOT_EQUIPING_NODE_ID 5000		//非装備車両ID開始番号
#define NOT_EQUIPING_NODE_ID_2 10000
#define START_FRAME 0		  //推定開始時刻（フレーム，秒）
#define INTERVAL_SEND 100	//ミリ秒

//提案手法が想定するセンサの誤差
#define ESTIMATTION_GPS_POSITION_ERROR 5
#define ESTIMATTION_RADAR_POSITION_ERROR 5.25
#define ESTIMATION_RADAR_ERROR 0.25
#define ESTIMATTION_SPEED_ERROR 0.158       //0.158=σ=0.5/√10


//マッチングで用いるパラメータ値
#define LIKELIHOOD 1			//マッチング成功時尤度の増加値
#define DISTANCE_FOR_MATCHING 1.0
#define DISTANCE_FOR_MARGE 3.0 
#define DISTANCE_FOR_SENSE 3.0 
#define MAX_DIRECT_SENSING 7	//この距離内ならば直接センシングできるはず
#define MAX_DIRECT_DEGREES 90

#define DISTANCE_MATCHING_RECEIVE  20	//受信車両のうちマッチング対象車両とするための距離

//情報削除のための閾値
#define INFO_DELETE 		200
#define THRESHOLD_DELETE 	10.0

//General
#define	FLOAT_MAX	100000
#define BRAKING_SPEED 9.7		//急加速時の制動距離
#define AREA_RANGE 2000			//車両の存在範囲（m）ループシナリオに対応するため。
#define INTERVAL_TIMES 1		//センサによる情報取得間隔（秒）


//以下，シミュレーション用
//_/_/センサ誤差モデル_/_/
//GPS誤差モデル
//#define GPS_EXPMODEL//実機から作成したモデル
#ifdef GPS_EXPMODEL
	#define ALPHA				0.25
	#define BASE_GPS_DEVIATION	0.25
	#define BASE_GPS_AVG		10
	#define ZIG_GPS_DEVIATION	0.5
	#define INTERVAL_PLANET		10
#else
	#define GPS_NORMAL	//正規分布モデル
#endif

//_/_/装備率_/_/
#define EQUIP_RATIO	1		//EQUIP_RATIOに1台が保持

//--------------------------------------------------------------------------------------
// 以上の変数定義などは，ver1.3のままであるので適合しない可能性がある
//--------------------------------------------------------------------------------------


/*

// PartI--sent at all times without any tagging
class BasicSafetyMessagePayloadPart1 {
public:
    static const unsigned int PART1_LENGTH_BYTES = 42;

    BasicSafetyMessagePayloadPart1() {
        data = new unsigned char[PART1_LENGTH_BYTES];
    }
    BasicSafetyMessagePayloadPart1(const unsigned char* payload) {
        data = new unsigned char[PART1_LENGTH_BYTES];
        for(int i = 0; i < PART1_LENGTH_BYTES; i++) {
            data[i] = payload[i];
        } 
    }
    BasicSafetyMessagePayloadPart1(const shared_ptr<VehicleInformation>& vehicleInfoPtr) {
        data = new unsigned char[PART1_LENGTH_BYTES];
        (*this).SetTime(vehicleInfoPtr->sensedTime);
        (*this).SetXPosition(vehicleInfoPtr->xPositionMeters);
        (*this).SetYPosition(vehicleInfoPtr->yPositionMeters);
        (*this).SetVelocity(vehicleInfoPtr->velocity);
        (*this).SetSourceNodeId(vehicleInfoPtr->sourceNodeId);
        (*this).SetTargetNodeId(vehicleInfoPtr->targetNodeId);
//        (*this).SetSize(vehicleInfoPtr->size);
      (*this).SetGPSXPosition(vehicleInfoPtr->xMeasuredPosition);
      (*this).SetGPSYPosition(vehicleInfoPtr->yMeasuredPosition);
      (*this).SetPositionError(vehicleInfoPtr->estPositionError);
	  (*this).SetBaseGPSXPosition(vehicleInfoPtr->xSensorPosition);
      (*this).SetBaseGPSYPosition(vehicleInfoPtr->ySensorPosition);

#ifdef PACKET_DEBUG
      cout<<"PACKET:"<<vehicleInfoPtr->sourceNodeId<<"->"<<vehicleInfoPtr->targetNodeId<<":"
      	  <<vehicleInfoPtr->xPositionMeters<<", "
      	  <<vehicleInfoPtr->yPositionMeters<<", "
      	  <<vehicleInfoPtr->xMeasuredPosition<<", "
      	  <<vehicleInfoPtr->yMeasuredPosition<<", "
      	  <<vehicleInfoPtr->xSensorPosition<<", "
      	  <<vehicleInfoPtr->ySensorPosition<<endl;
#endif
    }

    ~BasicSafetyMessagePayloadPart1() {}

    const unsigned char* GetRawData() const { return (data); }

    void StoreData(unsigned char* storeData, const int offset = 0) const {
        for (int i =0; i < PART1_LENGTH_BYTES; i++) {
            storeData[i + offset] = data[i];
        }
    }

    void GetStructData(shared_ptr<VehicleInformation>& vehicleInfoPtr) const {
        vehicleInfoPtr->sourceNodeId = (*this).GetSourceNodeId();
        vehicleInfoPtr->targetNodeId = (*this).GetTargetNodeId();
		vehicleInfoPtr->mgInfoId = NOT_EQUIPING_NODE_ID;
        
		vehicleInfoPtr->sensedTime = (*this).GetTime();
        vehicleInfoPtr->updatedTime = (*this).GetTime();
		vehicleInfoPtr->size = VectorType((float)5, (float)1.8);
        vehicleInfoPtr->xExactPosition = 0;
        vehicleInfoPtr->yExactPosition = 0;
        vehicleInfoPtr->sendFlag = false;
        vehicleInfoPtr->oldSendFlag = false;
        vehicleInfoPtr->sendCount = 0;
		vehicleInfoPtr->xPositionMeters = (*this).GetXPosition();
        vehicleInfoPtr->yPositionMeters = (*this).GetYPosition();
        vehicleInfoPtr->estPositionError = (*this).GetPositionError();
        vehicleInfoPtr->velocity = (*this).GetVelocity();
        vehicleInfoPtr->acceleration = VectorType(0,0);
        vehicleInfoPtr->xPriPositionMeters = (*this).GetXPosition();
        vehicleInfoPtr->yPriPositionMeters = (*this).GetYPosition();
        vehicleInfoPtr->estPriPositionError = (*this).GetPositionError();
		vehicleInfoPtr->xSensorPosition = (*this).GetBaseGPSXPosition();
		vehicleInfoPtr->ySensorPosition = (*this).GetBaseGPSYPosition();
		vehicleInfoPtr->xMeasuredPosition = (*this).GetGPSXPosition();
		vehicleInfoPtr->yMeasuredPosition = (*this).GetGPSYPosition();
		vehicleInfoPtr->xRelativePosition = 0;
		vehicleInfoPtr->yRelativePosition = 0;
    }

	
	//SAE-J2735
	//実装上の注意
	//TemporaryIDは6bytes，targetNodeIdおよびsourceNodeIdの二つを格納？
	//Speed/Heading（合計4bytesのはずだが）部分で速度（8bytes）消費
	//今回の手法では，SAE-J2735に規定のないGPS位置情報を別途送信する必要があるが（partIIIで？）
	//AccelerationSet4Way:8bytes部分で格納する

	//本来VehicleSizeを送る必要があるが，
	//40bytesを超えてしまうので今回は送らないことにする


    //DSecond: 2bytes data[0-1], TimeType(8bytes)
    void SetTime(const TimeType& initTime) {
        unsigned short int time =  (unsigned short int)(initTime / (MILLI_SECOND * 100));
        data[0] = (unsigned char)(time / 256);
        data[1] = (unsigned char)(time % 256); 
    }
    const TimeType GetTime() const {
        unsigned short int shortTime = data[0] * 256 + data[1];
        TimeType time(shortTime * MILLI_SECOND * 100);
        return time;
    }

    // Latitude: 4bytes data[2-5], float(4bytes)
    void SetXPosition(const float& initXPos) {
        memcpy(&data[2], &initXPos, 4);
    }
    const float GetXPosition() const {
        float xPos;
        memcpy(&xPos, &data[2], 4);
        return xPos;
    }
    // Longtude: 4bytes data[6-9], float(4bytes)
    void SetYPosition(const float& initYPos) {
        memcpy(&data[6], &initYPos, 4);
    }
    const float GetYPosition() const {
        float yPos;
        memcpy(&yPos, &data[6], 4);
        return yPos;
    }

    // Speed/Heading: 4bytes data[10-17], VectorType(8bytes)
    void SetVelocity(const VectorType& initVelocity) {
        float x = initVelocity.x;
        float y = initVelocity.y;
        memcpy(&data[10], &x, 4);
        memcpy(&data[14], &y, 4);
    }
    const VectorType GetVelocity() const {
        float x;
        float y;
        memcpy(&x, &data[10], 4);
        memcpy(&y, &data[14], 4);
        VectorType velocity(x, y);
        return velocity;
    };

    // VehicleSize: 3bytes data[18-21], float(4bytes)
    void SetPositionError(const float& initPosError) {
        memcpy(&data[18], &initPosError, 4);
    }
    const float GetPositionError() const {
        float PosError;
        memcpy(&PosError, &data[18], 4);
        return PosError;
    }


    //TemporaryID: 6bytes data[22-23], NodeIdType(4bytes)
    void SetSourceNodeId(const NodeIdType& initSourceNodeId) {
        unsigned short int nodeId = (unsigned short int)(initSourceNodeId);
        memcpy(&data[22], &nodeId, 2);
    }
    const NodeIdType GetSourceNodeId() const {
        unsigned short int nodeId;
        memcpy(&nodeId, &data[22], 2);
        NodeIdType sourceNodeId(nodeId);
        return sourceNodeId;
    }
    //TemporaryID: 6bytes data[24-25], NodeIdType(4bytes)
    void SetTargetNodeId(const NodeIdType& initTargetNodeId) {
        unsigned short int nodeId = (unsigned short int)(initTargetNodeId);
        memcpy(&data[24], &nodeId, 2);
    }
    const NodeIdType GetTargetNodeId() const {
        unsigned short int nodeId;
        memcpy(&nodeId, &data[24], 2);
        NodeIdType targetNodeId(nodeId);
        return targetNodeId;
    }

    // AccelerationSet4Way: 8bytes data[26-29], float(4bytes)
    void SetGPSXPosition(const float& initGPSXPos) {
        memcpy(&data[26], &initGPSXPos, 4);
    }
    const float GetGPSXPosition() const {
        float xGPSPos;
        memcpy(&xGPSPos, &data[26], 4);
        return xGPSPos;
    }

    // AccelerationSet4Way: 8bytes data[30-33], float(4bytes)
    void SetGPSYPosition(const float& initGPSYPos) {
        memcpy(&data[30], &initGPSYPos, 4);
    }
    const float GetGPSYPosition() const {
        float yGPSPos;
        memcpy(&yGPSPos, &data[30], 4);
        return yGPSPos;
    }
    // 4bytes data[34-37], float(4bytes)
    void SetBaseGPSXPosition(const float& initBaseGPSXPos) {
        memcpy(&data[34], &initBaseGPSXPos, 4);
    }
    const float GetBaseGPSXPosition() const {
        float xGPSPos;
        memcpy(&xGPSPos, &data[34], 4);
        return xGPSPos;
    }

    // 4bytes data[38-42], float(4bytes)
    void SetBaseGPSYPosition(const float& initBaseGPSYPos) {
        memcpy(&data[38], &initBaseGPSYPos, 4);
    }
    const float GetBaseGPSYPosition() const {
        float xGPSPos;
        memcpy(&xGPSPos, &data[38], 4);
        return xGPSPos;
    }

    
   
    //for official product to meet spec. perfectly
    //const unsigned char GetDSRCmsgID() const {}
    //void SetDSRCmsgID() {}
    //const unsigned short int GetDSecond() const {}
    //void SetDSecond() {}
    //const unsigned long long int GetTemporaryID() const {}//6 bytes
    //void SetTemporaryID() {}
    //const unsigned long int GetLatitude() const {}
    //void SetLatitude() {}
    //const unsigned long int GetLongitude() const {}
    //void SetLongitude() {}
    //const unsigned long int GetElecation() const {}//3 bytes
    //void SetElecation() {}
    //const unsigned short int GetSpeed() const {};
    //void SetSpeed() {}
    //const unsigned short int GetHeading() const {}
    //void SetHeading() {}
    //const unsigned long long int GetAccelerationSet4Way() const {}
    //void SetAccelerationSet4Way() {}
    //const unsigned char GetBrakeSystemStatus() const {}
    //void SetBrakeSystemStatus() {}
    //const unsigned short int GetSteeringWheelAngle() const {}
    //void SetSteeringWheelAngle() {}
    //const unsigned char GetThrottelePosition() const {}
    //void SetThrottelePosition() {}
    //const unsigned char GetExteriorLights() const {}
    //void SetExteriorLights() {}
    //const unsigned long int GetVehicleSize() const {}//3 bytes
    //void SetVehicleSize() {}

private:
    unsigned char* data;

};//BasicSafetyMessagePayloadPart1// //40bytes//


class BasicSafetyMessagePayloadPart3 {
public:
    static const unsigned int PART3_LENGTH_BYTES = 40;

    BasicSafetyMessagePayloadPart3() {
        data = new unsigned char[PART3_LENGTH_BYTES];
    }
    BasicSafetyMessagePayloadPart3(const unsigned char* payload) {
        data = new unsigned char[PART3_LENGTH_BYTES];
        for(int i = 0; i < PART3_LENGTH_BYTES; i++) {
            data[i] = payload[i];
        } 
    }
    BasicSafetyMessagePayloadPart3(const shared_ptr<VehicleInformation>& vehicleInfoPtr) {
        data = new unsigned char[PART3_LENGTH_BYTES];
        (*this).SetTime(vehicleInfoPtr->sensedTime);
        (*this).SetXPosition(vehicleInfoPtr->xPositionMeters);
        (*this).SetYPosition(vehicleInfoPtr->yPositionMeters);
        (*this).SetVelocity(vehicleInfoPtr->velocity);
	(*this).SetPositionError(vehicleInfoPtr->estPositionError);
        (*this).SetSourceNodeId(vehicleInfoPtr->sourceNodeId);
        (*this).SetTargetNodeId(vehicleInfoPtr->targetNodeId);
	(*this).SetSensorXPosition(vehicleInfoPtr->xMeasuredPosition);
    (*this).SetSensorYPosition(vehicleInfoPtr->yMeasuredPosition);
    (*this).SetMatchYudo(vehicleInfoPtr->estPriPositionError);

#ifdef PACKET_DEBUG
      cout<<"PACKET:"<<vehicleInfoPtr->sourceNodeId<<"->"<<vehicleInfoPtr->targetNodeId<<":"
      	  <<vehicleInfoPtr->xPositionMeters<<", "
      	  <<vehicleInfoPtr->yPositionMeters<<", "
      	  <<vehicleInfoPtr->xMeasuredPosition<<", "
      	  <<vehicleInfoPtr->yMeasuredPosition<<","
		  <<vehicleInfoPtr->estPriPositionError<<endl;
#endif


    }

    ~BasicSafetyMessagePayloadPart3() {}

    const unsigned char* GetRawData() const { return (data); }

    void StoreData(unsigned char* storeData, const int offset = 0) const {
        for (int i =0; i < PART3_LENGTH_BYTES; i++) {
            storeData[i + offset] = data[i];
        }
    }

    void GetStructData(shared_ptr<VehicleInformation>& vehicleInfoPtr) const {
        vehicleInfoPtr->sourceNodeId = (*this).GetSourceNodeId();
        vehicleInfoPtr->targetNodeId = (*this).GetTargetNodeId();
   		vehicleInfoPtr->mgInfoId = NOT_EQUIPING_NODE_ID;
		vehicleInfoPtr->sensedTime = (*this).GetTime();
        vehicleInfoPtr->updatedTime = (*this).GetTime();
        vehicleInfoPtr->size = VectorType((float)5, (float)1.8);
        vehicleInfoPtr->xExactPosition = 0;
        vehicleInfoPtr->yExactPosition = 0;
        vehicleInfoPtr->sendFlag = false;
        vehicleInfoPtr->oldSendFlag = false;
        vehicleInfoPtr->sendCount = 0;
		vehicleInfoPtr->xPositionMeters = (*this).GetXPosition();
        vehicleInfoPtr->yPositionMeters = (*this).GetYPosition();
        vehicleInfoPtr->estPositionError = (*this).GetPositionError();
        vehicleInfoPtr->velocity = (*this).GetVelocity();
        vehicleInfoPtr->acceleration = VectorType(0,0);
        vehicleInfoPtr->xPriPositionMeters = (*this).GetXPosition();
        vehicleInfoPtr->yPriPositionMeters = (*this).GetYPosition();
        vehicleInfoPtr->estPriPositionError = (*this).GetMatchYudo();
		vehicleInfoPtr->xSensorPosition = 0;
		vehicleInfoPtr->ySensorPosition = 0;
		vehicleInfoPtr->xMeasuredPosition = (*this).GetSensorXPosition();
		vehicleInfoPtr->yMeasuredPosition = (*this).GetSensorYPosition();
		vehicleInfoPtr->xRelativePosition = 0;
		vehicleInfoPtr->yRelativePosition = 0;
    }

    //DSecond: 2bytes data[0-1], TimeType(8bytes)
    void SetTime(const TimeType& initTime) {
        unsigned short int time =  (unsigned short int)(initTime / (MILLI_SECOND * 100));
        data[0] = (unsigned char)(time / 256);
        data[1] = (unsigned char)(time % 256); 
    }
    const TimeType GetTime() const {
        unsigned short int shortTime = data[0] * 256 + data[1];
        TimeType time(shortTime * MILLI_SECOND * 100);
        return time;
    }

    // Latitude: 4bytes data[2-5], float(4bytes)
    void SetXPosition(const float& initXPos) {
        memcpy(&data[2], &initXPos, 4);
    }
    const float GetXPosition() const {
        float xPos;
        memcpy(&xPos, &data[2], 4);
        return xPos;
    }
    // Longtude: 4bytes data[6-9], float(4bytes)
    void SetYPosition(const float& initYPos) {
        memcpy(&data[6], &initYPos, 4);
    }
    const float GetYPosition() const {
        float yPos;
        memcpy(&yPos, &data[6], 4);
        return yPos;
    }

    // Speed/Heading: 4bytes data[10-17], VectorType(8bytes)
    void SetVelocity(const VectorType& initVelocity) {
        float x = initVelocity.x;
        float y = initVelocity.y;
        memcpy(&data[10], &x, 4);
        memcpy(&data[14], &y, 4);
    }
    const VectorType GetVelocity() const {
        float x;
        float y;
        memcpy(&x, &data[10], 4);
        memcpy(&y, &data[14], 4);
        VectorType velocity(x, y);
        return velocity;
    };

    //4bytes data[18-21], float(4bytes)
    void SetPositionError(const float& initPosError) {
        memcpy(&data[18], &initPosError, 4);
    }
    const float GetPositionError() const {
        float posError;
        memcpy(&posError, &data[18], 4);
        return posError;    
    }

    //data[22-23], NodeIdType(4bytes)
    void SetSourceNodeId(const NodeIdType& initSourceNodeId) {
        unsigned short int nodeId = (unsigned short int)(initSourceNodeId);
        memcpy(&data[22], &nodeId, 2);
    }
    const NodeIdType GetSourceNodeId() const {
        unsigned short int nodeId;
        memcpy(&nodeId, &data[22], 2);
        NodeIdType sourceNodeId(nodeId);
        return sourceNodeId;
    }
    //data[24-25], NodeIdType(4bytes)
    void SetTargetNodeId(const NodeIdType& initTargetNodeId) {
        unsigned short int nodeId = (unsigned short int)(initTargetNodeId);
        memcpy(&data[24], &nodeId, 2);
    }
    const NodeIdType GetTargetNodeId() const {
        unsigned short int nodeId;
        memcpy(&nodeId, &data[24], 2);
        NodeIdType targetNodeId(nodeId);
        return targetNodeId;
    }

    //tag 2bytest:ata[26-27]
    void SetTagId(const unsigned short int& initTagId) {
        memcpy(&data[26], &initTagId, 2);
    }
    const unsigned short int GetTagId() const {
        unsigned short int tagId;
        memcpy(&tagId, &data[26], 2);
        return tagId;
    }

    // Latitude: 4bytes data[28-31], float(4bytes)
    void SetSensorXPosition(const float& initSensorXPos) {
        memcpy(&data[28], &initSensorXPos, 4);
    }
    const float GetSensorXPosition() const {
        float xPos;
        memcpy(&xPos, &data[28], 4);
        return xPos;
    }
    // Longtude: 4bytes data[32-35], float(4bytes)
    void SetSensorYPosition(const float& initSensorYPos) {
        memcpy(&data[32], &initSensorYPos, 4);
    }
    const float GetSensorYPosition() const {
        float yPos;
        memcpy(&yPos, &data[32], 4);
        return yPos;
    }


    // Longtude: 4bytes data[32-35], float(4bytes)
    void SetMatchYudo(const float& initMatchYudo) {
        memcpy(&data[36], &initMatchYudo, 4);
    }
    const float GetMatchYudo() const {
        float matchYudo;
        memcpy(&matchYudo, &data[36], 4);
        return matchYudo;
    }


   
private:
    unsigned char* data;

};//BasicSafetyMessagePayloadPart3// //32bytes//






*/

























#endif /* bsm_app_h */
