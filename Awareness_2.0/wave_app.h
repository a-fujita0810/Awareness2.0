// Copyright (c) 2007-2016 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef WAVE_APP_H
#define WAVE_APP_H

// The standard specified a message set, and its data elements specifically
// for use by applications intended to utilize the 5.9GHz Dedicated Short
// Range Communcations for Wireless Access in Vehicular Environments(DSRC/WAVE)

// DSRC = "Dedicated Short Range Communications" = (DSRC/WAVE)

#include "wave_net.h"
#include "bsm_app.h"
#include "vehicle_detection_sensor.h"
#include "common_math.h"

using std::cout;
using std::cerr;
using std::endl;
using std::string;
using std::set;
using std::pair;
using std::make_pair;
using std::pair;
using std::unique_ptr;
using std::map;
using std::istringstream;


//出力関係
#define OUTPUT
//#define NORMAL


//メッセージ関連
#define BSM_PART1_PACKET_LENGTH 41
#define BSM_PART3_PACKET_LENGTH 24
#define INTERVAL_BSM          100000000 //BSM送信間隔 [nS]
#define MAX_NUMBER_SEND_VEHICLE 3   //BSM送信車両数の最大値, 1:自車両のみ

//保持情報関連
#define DELETE_TIME_PACKET_INFO 500000000 //パケット情報の消去のための閾値
#define DELETE_TIME_SENSOR_INFO 500000000 //センサ情報の消去のための閾値

//Vehicle Typeごとのスコア
#define TYPE_UNKNOWN   5
#define TYPE_CONNECTED 2
#define TYPE_LEGACY    10

//マッチング関連
#define MATCHING_DURATION 1000000000

//評価関連
#define PROPOSED
//#define PROPOSED_REVISION
//#define PROPOSED_REVISION_2
//#define COMPARED_ALL
//#define COMPARED_RANDOM


//Priotiry関連
//defaultは均等に割り当てる
#define WEIGHT_TYPE    0.34
#define WEIGHT_VELOCITY 0.33
#define WEIGHT_LANE    0.33
#define LANE_WIDTH     5.000 //車線幅[m]
#define INIT_LANEPOSITION 0  //車線ファクター計算の対象外，score=0

#define MAX_LAST_SEND_TIME  3000 //[MS], 最低送信時間（この時間までには必ず送信する）
#define PRIORITY_MAX 30.0  //優先度の最大値
#define PRIORITY_DECREASE 5  //優先度の下げ幅の決定係数，仮ぎめ

namespace Wave {

typedef uint8_t DsrcMessageIdType;
enum {

    // Currently supported messages
    // 0. A La Carte --> 利用しない
    // 1. Basic Safety Message

    DSRC_MESSAGE_A_LA_CARTE = 0,
    DSRC_MESSAGE_BASIC_SAFETY,

    // not yet
    DSRC_MESSAGE_COMMON_SAFETY_REQUEST,
    DSRC_MESSAGE_EMERGENCY_VEHICLE_ALERT,
    DSRC_MESSAGE_INTERSECTION_COLLISION_AVOIDANCE,
    DSRC_MESSAGE_MAP_DATA,
    DSRC_MESSAGE_NMEA_CORRECTIONS,
    DSRC_MESSAGE_PROBE_DATA_MANAGEMENT,
    DSRC_MESSAGE_PROBE_VEHICLE_DATA,
    DSRC_MESSAGE_ROADSIDE_ALERT,
    DSRC_MESSAGE_RTCM_CORRECTIONS,
    DSRC_MESSAGE_SIGNAL_PHASE_AND_TIMING,
    DSRC_MESSAGE_SIGNAL_REQUEST,
    DSRC_MESSAGE_SIGNAL_STATUS,
    DSRC_MESSAGE_TRAVELER_INFORMATION,
};//DsrcMessageIdType//


//--------------------------------------------------------------------------------------
//BSM(Basic Safety Message) Application, which communicates to NW Layer(WSMP) directly
//--------------------------------------------------------------------------------------
struct DsrcAccelerationSetType {
	//6 bytes(2,2,2)
    uint16_t accelerationX;
    uint16_t accelerationY;
    uint16_t yawRate;

    DsrcAccelerationSetType()
        :
        accelerationX(0),
        accelerationY(0),
        yawRate(0)
    {}

    DsrcAccelerationSetType(const char* payload)
        :
        accelerationX(*reinterpret_cast<const uint16_t* >(&payload[0])),
        accelerationY(*reinterpret_cast<const uint16_t* >(&payload[2])),
        yawRate(*reinterpret_cast<const uint16_t* >(&payload[4]))
    {}

    void Write(char* payload) const {
        *reinterpret_cast<uint16_t* >(&payload[0]) = uint16_t(accelerationX);
        *reinterpret_cast<uint16_t* >(&payload[2]) = uint16_t(accelerationY);
        *reinterpret_cast<uint16_t* >(&payload[5]) = uint16_t(yawRate);
    }
};//DsrcAccelerationSetType//


struct DsrcPositionAccurancyType {
    uint8_t semiMajorAccurancyMeters;
    uint8_t semiMinorAccurancyMeters;
    double orientationDegrees;

    DsrcPositionAccurancyType()
    :
        semiMajorAccurancyMeters(0),
        semiMinorAccurancyMeters(0),
        orientationDegrees(0)
    {}

    DsrcPositionAccurancyType(const char* payload)
        :
        semiMajorAccurancyMeters(*reinterpret_cast<const uint8_t* >(&payload[0]) / 2),
        semiMinorAccurancyMeters(*reinterpret_cast<const uint8_t* >(&payload[1]) / 2),
        orientationDegrees(*reinterpret_cast<const uint16_t* >(&payload[2]) * (360./65535))
    {}

    void Write(char* payload) const {
        *reinterpret_cast<uint8_t* >(&payload[0]) = uint8_t(semiMajorAccurancyMeters * 2);
        *reinterpret_cast<uint8_t* >(&payload[1]) = uint8_t(semiMinorAccurancyMeters * 2);
        *reinterpret_cast<uint16_t* >(&payload[2]) = uint16_t(orientationDegrees / (360./65535));
    }
};//DsrcPositionAccurancyType//


struct DsrcTransmissionAndSpeedType {
    uint16_t transmissionState;
    double speedMeters;

    struct FieldIoType {
        uint16_t state:2;
        uint16_t speed:14;
    };

    DsrcTransmissionAndSpeedType()
        :
        transmissionState(0),
        speedMeters(0)
    {}

    DsrcTransmissionAndSpeedType(const char* payload) {
        const FieldIoType& fieldIo = *reinterpret_cast<const FieldIoType* >(payload);
        transmissionState = fieldIo.state;
        speedMeters = fieldIo.speed * 0.02;
    }

    void Write(char* payload) const {
        FieldIoType& fieldIo = *reinterpret_cast<FieldIoType* >(payload);
        fieldIo.state = transmissionState;
        fieldIo.speed = uint16_t(speedMeters / 0.02);
    }
};//DsrcTransmissionAndSpeedType//


struct DsrcBrakeSystemStatusType {
    uint16_t wheelBrakes:4;
    uint16_t wheelBrakesUnavailable:1;
    uint16_t spareBit:1;
    uint16_t traction:2;
    uint16_t abs:2;
    uint16_t scs:2;
    uint16_t braksBoost:2;
    uint16_t auxBrakes:2;

    DsrcBrakeSystemStatusType()
        :
        wheelBrakes(0),
        wheelBrakesUnavailable(0),
        spareBit(0),
        traction(0),
        abs(0),
        scs(0),
        braksBoost(0),
        auxBrakes(0)
    {}
};//DsrcBrakeSystemStatusType//


struct DsrcVehicleSizeType {
    double widthMeters;
    double lengthMeters;

    DsrcVehicleSizeType()
        :
        widthMeters(0),
        lengthMeters(0)
    {}

    DsrcVehicleSizeType(const char* payload)
        :
        widthMeters(*reinterpret_cast<const uint8_t* >(&payload[0])), // simplified for simulation
        lengthMeters(*reinterpret_cast<const uint8_t* >(&payload[1]))
    {}

    void Write(char* payload) const {
        *reinterpret_cast<uint8_t* >(&payload[0]) = uint8_t(widthMeters);
        *reinterpret_cast<uint8_t* >(&payload[1]) = uint8_t(lengthMeters);
    }
};//DsrcVehicleSizeType//


//--------------------------------------------------------------------------------------
// BSM Part1 *提案手法に合うように変更
//--------------------------------------------------------------------------------------
struct DsrcBasicSafetyMessagePart1Type {

    DsrcMessageIdType messageId;
    char blob[BSM_PART1_PACKET_LENGTH];  //Binary Large OBject

    TimeType GetTime() const { return TimeType(*reinterpret_cast<const uint16_t* >(&blob[0])) * MILLI_SECOND;}  //[0-1]
    uint8_t GetMessageCount() const { return *reinterpret_cast<const uint8_t* >(&blob[2]); }  //[2]
    NodeIdType GetSourceId() const { return *reinterpret_cast<const uint8_t* >(&blob[3]); }   //[3-4]
    NodeIdType GetTargetId() const { return *reinterpret_cast<const uint8_t* >(&blob[5]); }   //[5-6]
    float GetXMeters() const { return *reinterpret_cast<const float* >(&blob[7]); }    //[7-10]
    float GetYMeters() const { return *reinterpret_cast<const float* >(&blob[11]); }   //[11-14]
    float GetXVelocity() const { return *reinterpret_cast<const float* >(&blob[15]); } //[15-18]
    float GetYVelocity() const { return *reinterpret_cast<const float* >(&blob[19]); } //[19-22]
    float GetAccuracy()  const { return *reinterpret_cast<const float* >(&blob[23]); }
    double GetHeading() const { return (*reinterpret_cast<const uint16_t* >(&blob[27]))*0.0125; }   //[27-28]
    float GetGpsXMeters() const { return *reinterpret_cast<const float* >(&blob[29]); } //[29-32]
    float GetGpsYMeters() const { return *reinterpret_cast<const float* >(&blob[33]); } //[33-36]
    float GetXAccel() const { return *reinterpret_cast<const float* >(&blob[37]); } //[15-18]
    float GetYAccel() const { return *reinterpret_cast<const float* >(&blob[39]); } //[15-18]

    void SetTime(const TimeType& time) { *reinterpret_cast<uint16_t* >(&blob[0]) = uint16_t(time / MILLI_SECOND); }
    void SetMessageCount(const uint8_t messageCount) { *reinterpret_cast<uint8_t* >(&blob[2]) = messageCount; }
    void SetSourceId(const NodeIdType source) { *reinterpret_cast<uint8_t* >(&blob[3]) = source; }
    void SetTargetId(const NodeIdType target) { *reinterpret_cast<uint8_t* >(&blob[5]) = target; }
    void SetXMeters(const float xMeters) { *reinterpret_cast<float* >(&blob[7]) = xMeters; }
    void SetYMeters(const float yMeters) { *reinterpret_cast<float* >(&blob[11]) = yMeters; }
    void SetXVelocity(const float xVelocity) { *reinterpret_cast<float* >(&blob[15]) = xVelocity; }
    void SetYVelocity(const float yVelocity) { *reinterpret_cast<float* >(&blob[19]) = yVelocity; }
    void SetAccuracy(const float accuracy) { *reinterpret_cast<float* >(&blob[23]) = accuracy; }
    void SetHeading(const double heading) { *reinterpret_cast<uint16_t* >(&blob[27]) = uint16_t(heading/0.0125); }
    void SetGpsXMeters(const float xGpsMeters) { *reinterpret_cast<float* >(&blob[29]) = xGpsMeters; }
    void SetGpsYMeters(const float yGpsMeters) { *reinterpret_cast<float* >(&blob[33]) = yGpsMeters; }
    void SetXAccel(const float xAccel) { *reinterpret_cast<float* >(&blob[37]) = xAccel; }
    void SetYAccel(const float yAccel) { *reinterpret_cast<float* >(&blob[39]) = yAccel; }

    DsrcBasicSafetyMessagePart1Type();
};//DsrcBasicSafetyMessagePart1Type//
    

//--------------------------------------------------------------------------------------
// BSM Part3
//--------------------------------------------------------------------------------------
struct DsrcBasicSafetyMessagePart3Type {
	
    DsrcMessageIdType messageId;
    char blob[BSM_PART3_PACKET_LENGTH]; //Binary Large OBject
	
    NodeIdType GetSourceId() const { return *reinterpret_cast<const uint8_t* >(&blob[0]); } //[0-1]
    NodeIdType GetTargetId() const { return *reinterpret_cast<const uint8_t* >(&blob[2]); }   //[2-3]
    float GetXMeters() const { return *reinterpret_cast<const float* >(&blob[4]); }    //[4-7]
    float GetYMeters() const { return *reinterpret_cast<const float* >(&blob[8]); }   //[8-11]
    float GetXVelocity() const { return *reinterpret_cast<const float* >(&blob[12]); } //[12-15]
    float GetYVelocity() const { return *reinterpret_cast<const float* >(&blob[16]); } //[16-19]
    float GetAccuracy()  const { return *reinterpret_cast<const float* >(&blob[20]); } //[20-23]

    void SetSourceId(const NodeIdType source) { *reinterpret_cast<uint8_t* >(&blob[0]) = source; }
    void SetTargetId(const NodeIdType target) { *reinterpret_cast<uint8_t* >(&blob[2]) = target; }
    void SetXMeters(const float xMeters) { *reinterpret_cast<float* >(&blob[4]) = xMeters; }
    void SetYMeters(const float yMeters) { *reinterpret_cast<float* >(&blob[8]) = yMeters; }
    void SetXVelocity(const float xVelocity) { *reinterpret_cast<float* >(&blob[12]) = xVelocity; }
    void SetYVelocity(const float yVelocity) { *reinterpret_cast<float* >(&blob[16]) = yVelocity; }
    void SetAccuracy(const float accuracy) { *reinterpret_cast<float* >(&blob[20]) = accuracy; }
    
    DsrcBasicSafetyMessagePart3Type();
};//DsrcBasicSafetyMessagePart3Type//


class DsrcPacketExtrinsicInformation : public ExtrinsicPacketInformation {
public:
    static const ExtrinsicPacketInfoIdType id;

    DsrcPacketExtrinsicInformation() : sequenceNumber(0), transmissionTime(ZERO_TIME) {}
    DsrcPacketExtrinsicInformation(
        const uint32_t initSequenceNumber,
        const TimeType& initTransmissionTime)
        :
        sequenceNumber(initSequenceNumber),
        transmissionTime(initTransmissionTime)
    {}

    virtual shared_ptr<ExtrinsicPacketInformation> Clone() {
        return shared_ptr<ExtrinsicPacketInformation>(
            new DsrcPacketExtrinsicInformation(*this));
    }

    uint32_t sequenceNumber;
    TimeType transmissionTime;
};//DsrcPacketExtrinsicInformation//



//--------------------------------------------------------------------------------------------
// DsrcMessageApplication
//--------------------------------------------------------------------------------------------
class DsrcMessageApplication : public Application {
public:
    DsrcMessageApplication(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
        const shared_ptr<WsmpLayer>& initWsmpLayerPtr,
        const NodeIdType& initNodeId,
        const RandomNumberGeneratorSeedType& initNodeSeed,
        const shared_ptr<ObjectMobilityModel>& initNodeMobilityModelPtr);
    
    void SendBasicSafetyMessage(
        const DsrcBasicSafetyMessagePart1Type& basicSafetyMessagePart1Type);
    
    void SendBasicSafetyMessage(
        const DsrcBasicSafetyMessagePart1Type& basicSafetyMessagePart1Type,
        const unsigned char* part2Payload,
        const size_t part2PayloadSize);
    
    void SendBasicSafetyMessage(
        const DsrcBasicSafetyMessagePart1Type& basicSafetyMessagePart1Type,
        map<int, DsrcBasicSafetyMessagePart3Type> sendPart3Data,
        const size_t part3PayloadSize);

private:
    class PeriodicBasicSafetyMessageTransmissionEvent : public SimulationEvent {
    public:
        PeriodicBasicSafetyMessageTransmissionEvent(
            DsrcMessageApplication* initDsrcMessageApp) : dsrcMessageApp(initDsrcMessageApp) {}
        virtual void ExecuteEvent() { dsrcMessageApp->PeriodicallyTransmitBasicSafetyMessage(); }
    private:
        DsrcMessageApplication* dsrcMessageApp;
    };

    class PacketHandler: public WsmpLayer::WsmApplicationHandler {
    public:
        PacketHandler(DsrcMessageApplication* initDsrcMessageApp) : dsrcMessageApp(initDsrcMessageApp) {}

        virtual void ReceiveWsm(unique_ptr<Packet>& packetPtr) { dsrcMessageApp->ReceivePacketFromLowerLayer(packetPtr); }
    private:
        DsrcMessageApplication* dsrcMessageApp;
    };

    static const string basicSafetyAppModelName;
    static const ApplicationIdType applicationId;
    static const long long SEED_HASH = 1758612;

    shared_ptr<WsmpLayer> wsmpLayerPtr;
    shared_ptr<ObjectMobilityModel> nodeMobilityModelPtr;
    RandomNumberGenerator aRandomNumberGenerator;
    
    
    struct BasicSafetyMessageInfo {
        TimeType startTime;
        TimeType endTime;
        TimeType transmissionInterval;
        PacketPriorityType priority;
        string providerServiceId;
        size_t extendedPayloadSizeBytes;
        ChannelNumberIndexType channelNumberId;

        unsigned int currentSequenceNumber;
        unsigned int numberPacketsReceived;

        shared_ptr<CounterStatistic> packetsSentStatPtr;
        shared_ptr<CounterStatistic> bytesSentStatPtr;
        shared_ptr<CounterStatistic> packetsReceivedStatPtr;
        shared_ptr<CounterStatistic> bytesReceivedStatPtr;
        shared_ptr<RealStatistic> endToEndDelayStatPtr;

        BasicSafetyMessageInfo()
            :
            startTime(ZERO_TIME),
            endTime(ZERO_TIME),
            transmissionInterval(INFINITE_TIME),
            priority(0),
            providerServiceId("0"),
            extendedPayloadSizeBytes(0),
            channelNumberId(CHANNEL_NUMBER_178),
            currentSequenceNumber(0),
            numberPacketsReceived(0)
        {}
    };

    BasicSafetyMessageInfo basicSafetyMessageInfo;
    
    int sourceNodeId; //パケット内の車両ID
    int nodeId;       //実ID（デバッグ用）
    double movingDirection;
    float maxRadarSensingMeters = 50.0;
    TimeType preCalDensity = 1000000;   //密度の前回計算時間
    
    shared_ptr<VehicleSensors> sensorPtr;           //センサログから読み取るモビリティデータ
    shared_ptr<VehicleInformation> selfInfoPtr;     //自車両情報へのポインタ
    shared_ptr<VehicleInformation> packetInfoPtr;   //（受信時）パケット情報へのポインタ
    shared_ptr<VehicleInformation> neighborInfoPtr; //周辺車両情報へのポインタ
    
    map<NodeIdType, shared_ptr<VehicleInformation> >  storedSelfInfo;      //自車情報の保持
    map<NodeIdType, shared_ptr<VehicleInformation> >* neighborsInfoPtr;    //周辺車両情報へのポインタ
    map<NodeIdType, shared_ptr<VehicleInformation> >  storedNeighborsInfo; //周辺車両情報の保持
    multimap<NodeIdType, shared_ptr<VehicleInformation> >  storedPacketInfo; //受信パケット情報の保持
    map<NodeIdType, shared_ptr<VehicleInformation> >  storedSensorsInfo; //センサ情報の保持
    map<NodeIdType, shared_ptr<VehicleInformation> >  storedMatchingTableInfo; //周辺車両情報の保持
    
	map<NodeIdType, shared_ptr<MatchingTable> > matchingTableInfo;         //マッチング情報の保持
    map<NodeIdType, shared_ptr<MatchingGraphNode> > storedMatchingInfo;    //マッチング結果の保持
	map<NodeIdType, float> matchNodeList;    //固定マッチングされているノードのリストを保持
    
    map<int, DsrcBasicSafetyMessagePart3Type> sendPart3Data;   //BSM Part3送信データ
    int maxSendNode = MAX_NUMBER_SEND_VEHICLE-1;               //BSM送信車両最大数； 1 -> 自車両(Part1)のみ

    void ReceivePacketFromLowerLayer(unique_ptr<Packet>& packetPtr);
    void ReceiveBasicSafetyMessage(unique_ptr<Packet>& packetPtr);
    
    void PrintOutForReceivedMessage(DsrcBasicSafetyMessagePart1Type& ptr);
    void PrintOutForReceivedMessage(DsrcBasicSafetyMessagePart3Type& ptr);
    void PrintOutNeighborData(shared_ptr<VehicleInformation>& ptr);          //1台分の情報出力
    void PrintOutVehiclesInformation(map<NodeIdType, shared_ptr<VehicleInformation> >, float selfXPos, float selfYPos); //全周辺車両の情報出力
    void PrintOutVehiclesInformationForOutput(map<NodeIdType, shared_ptr<VehicleInformation> >, float selfXPos, float selfYPos, int idenFlag); //全周辺車両の情報出力，解析ファイル出力用
    void PrintOutPriority(map<NodeIdType, shared_ptr<VehicleInformation> >, float type, float speed, float lane); //Priority出力
    void PrintOutPacketsInformation(multimap<NodeIdType, shared_ptr<VehicleInformation> >); //受信パケット情報出力
    void PrintOutPacketsInformationForOutput(multimap<NodeIdType, shared_ptr<VehicleInformation> >, float selfXPos, float selfYPos, int idenFlag); //受信パケット情報出力，解析ファイル出力用

    void PeriodicallyTransmitBasicSafetyMessage();

    void OutputTraceAndStatsForSendBasicSafetyMessage(
        const unsigned int sequenceNumber,
        const PacketIdType& packetId,
        const size_t packetLengthBytes);

    void OutputTraceAndStatsForReceiveBasicSafetyMessage(
        const unsigned int sequenceNumber,
        const PacketIdType& packetId,
        const size_t packetLengthBytes,
        const TimeType& delay);
    
    /*
    NodeIdType MATCHING_ID_NUMBER;  //マッチング対象車両のID
	NodeIdType matchingAssignID(){
		++MATCHING_ID_NUMBER;
		return MATCHING_ID_NUMBER;
	}
    
    NodeIdType NEIGHBOR_ID_NUMBER;  //周辺車両のID
	NodeIdType neighborAssignID(){
		++NEIGHBOR_ID_NUMBER;
		return NEIGHBOR_ID_NUMBER;
	}
    */
    
    float calculatePriority(
        NodeIdType targetNodeId,
        shared_ptr<VehicleInformation> neighborData,
        shared_ptr<VehicleInformation> selfInfoPtr);
    
    VectorType rotateXY(
        const float x,
        const float y,
        const float theta);
    
    int crossProduct(
        const VectorType p1,
        const VectorType p2,
        const VectorType p3);
    
    bool isMatchingCandidate(
        const VectorType relativeCoord,
        const VectorType SelfVelocity);
    
};//DsrcMessageApplication//


//--------------------------------------------------------------------------
// BSM Part1,3 コンストラクタ
//--------------------------------------------------------------------------
inline
DsrcBasicSafetyMessagePart1Type::DsrcBasicSafetyMessagePart1Type()
    :
    messageId(DSRC_MESSAGE_BASIC_SAFETY)
{
    (*this).SetTime(ZERO_TIME);
    (*this).SetMessageCount(0);
    (*this).SetSourceId(0);
    (*this).SetTargetId(0);
    (*this).SetXMeters(0);
    (*this).SetYMeters(0);
    (*this).SetXVelocity(0);
    (*this).SetYVelocity(0);
    (*this).SetAccuracy(0);
    (*this).SetHeading(0);
    (*this).SetGpsXMeters(0);
    (*this).SetGpsYMeters(0);
    (*this).SetXAccel(0);
    (*this).SetYAccel(0);
}//DsrcBasicSafetyMessagePart1Type//


inline
DsrcBasicSafetyMessagePart3Type::DsrcBasicSafetyMessagePart3Type()
    :
    messageId(DSRC_MESSAGE_BASIC_SAFETY)
{
    (*this).SetTargetId(0);
    (*this).SetXMeters(0);
    (*this).SetYMeters(0);
    (*this).SetXVelocity(0);
    (*this).SetYVelocity(0);
    (*this).SetAccuracy(0);
}//DsrcBasicSafetyMessagePart3Type//


inline
DsrcMessageApplication::DsrcMessageApplication(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
    const shared_ptr<WsmpLayer>& initWsmpLayerPtr,
    const NodeIdType& initNodeId,
    const RandomNumberGeneratorSeedType& initNodeSeed,
    const shared_ptr<ObjectMobilityModel>& initNodeMobilityModelPtr)
    :
    Application(initSimEngineInterfacePtr, applicationId),
    wsmpLayerPtr(initWsmpLayerPtr),
    nodeMobilityModelPtr(initNodeMobilityModelPtr),
    aRandomNumberGenerator(HashInputsToMakeSeed(initNodeSeed, SEED_HASH))
{
    
    const TimeType jitter = static_cast<TimeType>(
        theParameterDatabaseReader.ReadTime("its-bsm-app-traffic-start-time-max-jitter", initNodeId) *
        aRandomNumberGenerator.GenerateRandomDouble());

    basicSafetyMessageInfo.startTime =
        theParameterDatabaseReader.ReadTime("its-bsm-app-traffic-start-time", initNodeId) + jitter;

    basicSafetyMessageInfo.endTime =
        theParameterDatabaseReader.ReadTime("its-bsm-app-traffic-end-time", initNodeId) + jitter;

    basicSafetyMessageInfo.transmissionInterval =
        theParameterDatabaseReader.ReadTime("its-bsm-app-traffic-interval", initNodeId);

    basicSafetyMessageInfo.priority =
        ConvertToUChar(
            theParameterDatabaseReader.ReadNonNegativeInt("its-bsm-app-packet-priority", initNodeId),
            "Error in parameter: \"its-bsm-app-packet-priority\"");

    if (theParameterDatabaseReader.ParameterExists("its-bsm-app-service-provider-id", initNodeId)) {
        basicSafetyMessageInfo.providerServiceId =
            ConvertToProviderServiceIdString(theParameterDatabaseReader.ReadNonNegativeInt("its-bsm-app-service-provider-id", initNodeId));
    }//if//


    //今回はPart3のパケット長は可変とするので，以下の項目はconfigで設定しない
    basicSafetyMessageInfo.extendedPayloadSizeBytes = sizeof(DsrcBasicSafetyMessagePart1Type) + sizeof(DsrcBasicSafetyMessagePart3Type); //Part3のパケット長固定の設定
    if (theParameterDatabaseReader.ParameterExists("its-bsm-app-packet-payload-size-bytes", initNodeId)) {
        basicSafetyMessageInfo.extendedPayloadSizeBytes =
            theParameterDatabaseReader.ReadNonNegativeInt("its-bsm-app-packet-payload-size-bytes", initNodeId);

        if (basicSafetyMessageInfo.extendedPayloadSizeBytes < sizeof(DsrcBasicSafetyMessagePart1Type)) {
            cerr << "its-bsm-app-packet-payload-size-bytes must be more than "
                 << "BasicSafetyMessagePayloadPart1:"
                 << sizeof(DsrcBasicSafetyMessagePart1Type) << endl;
            exit(1);
        }//if//
    }//if//

    if (theParameterDatabaseReader.ParameterExists("its-bsm-channel-number", initNodeId)) {
        basicSafetyMessageInfo.channelNumberId =
            ConvertToChannelNumberIndex(
                theParameterDatabaseReader.ReadInt("its-bsm-channel-number", initNodeId));
    }//if//

    basicSafetyMessageInfo.packetsSentStatPtr =
        simulationEngineInterfacePtr->CreateCounterStat(
            (basicSafetyAppModelName + "_PacketsSent"));

    basicSafetyMessageInfo.bytesSentStatPtr =
        simulationEngineInterfacePtr->CreateCounterStat(
            (basicSafetyAppModelName + "_BytesSent"));

    basicSafetyMessageInfo.packetsReceivedStatPtr =
        simulationEngineInterfacePtr->CreateCounterStat(
            (basicSafetyAppModelName + "_PacketsReceived"));

    basicSafetyMessageInfo.bytesReceivedStatPtr =
        simulationEngineInterfacePtr->CreateCounterStat(
            (basicSafetyAppModelName  + "_BytesReceived"));

    basicSafetyMessageInfo.endToEndDelayStatPtr =
        simulationEngineInterfacePtr->CreateRealStat(
            (basicSafetyAppModelName + "_EndToEndDelay"));

    wsmpLayerPtr->SetWsmApplicationHandler(
        basicSafetyMessageInfo.providerServiceId,
        shared_ptr<WsmpLayer::WsmApplicationHandler>(
            new PacketHandler(this)));

    const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();
    TimeType nextTransmissionTime = basicSafetyMessageInfo.startTime;
    
    nodeId = simulationEngineInterfacePtr->GetNodeId();
    
    //センサログからのデータ取得用
    sensorPtr = shared_ptr<VehicleSensors>(
            new VehicleSensors(
                nodeId,
                theParameterDatabaseReader,
                initNodeSeed,
                initNodeId)
                );

    if (currentTime > basicSafetyMessageInfo.startTime) {
        const size_t numberPassedTransmissionTimes =
            size_t(std::ceil(double(currentTime - basicSafetyMessageInfo.startTime) /
                             basicSafetyMessageInfo.transmissionInterval));

        nextTransmissionTime =
            basicSafetyMessageInfo.startTime +
            numberPassedTransmissionTimes*basicSafetyMessageInfo.transmissionInterval;
    }//if//

    if (nextTransmissionTime < basicSafetyMessageInfo.endTime) {
        simulationEngineInterfacePtr->ScheduleEvent(
            unique_ptr<SimulationEvent>(
                new PeriodicBasicSafetyMessageTransmissionEvent(this)),
            nextTransmissionTime);
        
    }//if//
    
}//DsrcMessageApplication//


inline
void DsrcMessageApplication::SendBasicSafetyMessage(
    const DsrcBasicSafetyMessagePart1Type& basicSafetyMessagePart1)
{
    (*this).SendBasicSafetyMessage(basicSafetyMessagePart1, nullptr, 0);
}//SendBasicSafetyMessage//


inline
void DsrcMessageApplication::SendBasicSafetyMessage(
    const DsrcBasicSafetyMessagePart1Type& basicSafetyMessagePart1,
    const unsigned char* part2Payload,
    const size_t part2PayloadSize)
{
    const unsigned char* part1Payload =
        reinterpret_cast<const unsigned char* >(&basicSafetyMessagePart1);

    const size_t part1PayloadSize =
        sizeof(DsrcBasicSafetyMessagePart1Type);


    vector<unsigned char> payload(part1PayloadSize + part2PayloadSize);

    for(size_t i = 0; i < part1PayloadSize; i++) {
        payload[i] = part1Payload[i];
    }//for
    for(size_t i = 0; i < part2PayloadSize; i++) {
        payload[part1PayloadSize + i] = part2Payload[i];
    }//for


    unique_ptr<Packet> packetPtr = Packet::CreatePacket(*simulationEngineInterfacePtr, payload);

    packetPtr->AddExtrinsicPacketInformation(
        DsrcPacketExtrinsicInformation::id,
        shared_ptr<DsrcPacketExtrinsicInformation>(
            new DsrcPacketExtrinsicInformation(
                basicSafetyMessageInfo.currentSequenceNumber,
                simulationEngineInterfacePtr->CurrentTime())));

    (*this).OutputTraceAndStatsForSendBasicSafetyMessage(
        basicSafetyMessageInfo.currentSequenceNumber,
        packetPtr->GetPacketId(),
        packetPtr->LengthBytes());

    wsmpLayerPtr->SendWsm(
        packetPtr,
        NetworkAddress::broadcastAddress,
        basicSafetyMessageInfo.channelNumberId,
        basicSafetyMessageInfo.providerServiceId,
        basicSafetyMessageInfo.priority);

    basicSafetyMessageInfo.currentSequenceNumber++;
}//SendBasicSafetyMessage//


//---------------------------------------------------------------------------
// BSMメッセージ送信関数（書き換えないように）
//---------------------------------------------------------------------------
inline
void DsrcMessageApplication::SendBasicSafetyMessage(
    const DsrcBasicSafetyMessagePart1Type& basicSafetyMessagePart1,
    map<int, DsrcBasicSafetyMessagePart3Type> sendPart3Data,
    const size_t part3PayloadSize)
{
    const unsigned char* part1Payload =
        reinterpret_cast<const unsigned char* >(&basicSafetyMessagePart1);

    const size_t part1PayloadSize = BSM_PART1_PACKET_LENGTH;
    vector<unsigned char> payload(part1PayloadSize + part3PayloadSize);

    for(size_t i = 0; i < part1PayloadSize; i++) {
        payload[i] = part1Payload[i];
    }//for
    
    size_t tmpPayloadSize = part1PayloadSize;
    typedef map<int, DsrcBasicSafetyMessagePart3Type>::iterator bIterType;
    
    for (bIterType bIter = sendPart3Data.begin(); bIter != sendPart3Data.end(); bIter++){
        NodeIdType targetNodeId = (*bIter).first;
        const unsigned char* part3Payload =
            reinterpret_cast<const unsigned char* >(&((*bIter).second));
        
        for(int i = 0; i < BSM_PART3_PACKET_LENGTH; i++) {
            payload[tmpPayloadSize + i] = part3Payload[i];
        }//for
        
        tmpPayloadSize += BSM_PART3_PACKET_LENGTH;
    }//for
    
    unique_ptr<Packet> packetPtr = Packet::CreatePacket(*simulationEngineInterfacePtr, payload);
    sourceNodeId = packetPtr->GetPacketId().GetSourceNodeId();   // 自車両IDの取得
    
    //以下，default
    packetPtr->AddExtrinsicPacketInformation(
        DsrcPacketExtrinsicInformation::id,
        shared_ptr<DsrcPacketExtrinsicInformation>(
            new DsrcPacketExtrinsicInformation(
                basicSafetyMessageInfo.currentSequenceNumber,
                simulationEngineInterfacePtr->CurrentTime())));

    (*this).OutputTraceAndStatsForSendBasicSafetyMessage(
        basicSafetyMessageInfo.currentSequenceNumber,
        packetPtr->GetPacketId(),
        packetPtr->LengthBytes());

    wsmpLayerPtr->SendWsm(
        packetPtr,
        NetworkAddress::broadcastAddress,
        basicSafetyMessageInfo.channelNumberId,
        basicSafetyMessageInfo.providerServiceId,
        basicSafetyMessageInfo.priority);

    basicSafetyMessageInfo.currentSequenceNumber++;
}//SendBasicSafetyMessage//



//---------------------------------------------------------------------------
// BSMメッセージ送信処理 called at every 100ms (IEEE1609.4)
//---------------------------------------------------------------------------
inline
void DsrcMessageApplication::PeriodicallyTransmitBasicSafetyMessage()
{
    //-------------------------------------
    //ノードが消失した場合の処理について未考慮
    //-------------------------------------
    
    //レガシー車両であれば，以下のBSMの生成処理を実行しない
    int isLegacy = nodeId % 10;
#ifdef RATIO_HIGH
    if (isLegacy == 2 || isLegacy == 4 || isLegacy == 6 || isLegacy == 8 || isLegacy == 0) {return;}
#endif

#ifdef RATIO_MIDDLE
    if (isLegacy == 2 || isLegacy == 5 || isLegacy == 8) {return;}
#endif

#ifdef RATIO_LOW
    if (isLegacy == 5) {return;}
#endif


    nodeId = simulationEngineInterfacePtr->GetNodeId();
    cout << "nodeId:" << nodeId << ": " << "BSM_App:CchSync:"
         << simulationEngineInterfacePtr->CurrentTime() / MILLI_SECOND 
        << endl;
    
    TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();
    typedef map<NodeIdType, shared_ptr<VehicleInformation> >::iterator IterType;
    typedef multimap<NodeIdType, shared_ptr<VehicleInformation> >::iterator packetIterType;
    
    //------------------------------------------------------------------------------
    // BSM Part1生成 (SelfInfoPtr から)
    //------------------------------------------------------------------------------
    shared_ptr<VehicleInformation> selfInfoPtr(new VehicleInformation);
    selfInfoPtr = sensorPtr->GetSelfPosition(simulationEngineInterfacePtr->CurrentTime()); //センサログからのデータ取得
    if (selfInfoPtr == nullptr) {return;}
    movingDirection = sensorPtr->GetMovingDirection(); //センサログ記載の進行方角
   
    selfInfoPtr->updatedTime = simulationEngineInterfacePtr->CurrentTime();
    float xExactPos = selfInfoPtr->xExactPosition;
    float yExactPos = selfInfoPtr->yExactPosition;
    float xEstPos   = selfInfoPtr->xPositionMeters;
    float yEstPos   = selfInfoPtr->yPositionMeters;
    
    DsrcBasicSafetyMessagePart1Type basicSafetyMessagePart1;
    basicSafetyMessagePart1.SetTime(simulationEngineInterfacePtr->CurrentTime());
    basicSafetyMessagePart1.SetMessageCount(uint8_t(basicSafetyMessageInfo.currentSequenceNumber));
    basicSafetyMessagePart1.SetSourceId(nodeId);
    basicSafetyMessagePart1.SetTargetId(nodeId);    //part1なのでsourceIDと同様
    basicSafetyMessagePart1.SetXMeters(xExactPos);
    basicSafetyMessagePart1.SetYMeters(yExactPos);
    basicSafetyMessagePart1.SetXVelocity(selfInfoPtr->velocity.x);
    basicSafetyMessagePart1.SetYVelocity(selfInfoPtr->velocity.y);
    basicSafetyMessagePart1.SetAccuracy(distance2Coordinate(xExactPos, yExactPos, xEstPos, yEstPos));
    basicSafetyMessagePart1.SetHeading(movingDirection);
    basicSafetyMessagePart1.SetGpsXMeters(xEstPos);
    basicSafetyMessagePart1.SetGpsYMeters(yEstPos);
    basicSafetyMessagePart1.SetXAccel(selfInfoPtr->acceleration.x);
    basicSafetyMessagePart1.SetYAccel(selfInfoPtr->acceleration.y);
    
    assert(basicSafetyMessageInfo.extendedPayloadSizeBytes >= sizeof(DsrcBasicSafetyMessagePart1Type));
    
    
    IterType siter = storedSelfInfo.find(nodeId);
    if (siter == storedSelfInfo.end()) {
        //new, insert
        storedSelfInfo.insert(make_pair(nodeId,selfInfoPtr));
    } else {
        //found, update
        (*siter).second->updatedTime    = currentTime;
        (*siter).second->xExactPosition = selfInfoPtr->xExactPosition;
        (*siter).second->yExactPosition = selfInfoPtr->yExactPosition;
        (*siter).second->velocity.x     = selfInfoPtr->velocity.x;
        (*siter).second->velocity.y     = selfInfoPtr->velocity.y;
        (*siter).second->xPositionMeters = selfInfoPtr->xPositionMeters;
        (*siter).second->yPositionMeters = selfInfoPtr->yPositionMeters;
        (*siter).second->estPositionError = distance2Coordinate(xExactPos, yExactPos, xEstPos, yEstPos);
        (*siter).second->acceleration.x = selfInfoPtr->velocity.x;
        (*siter).second->acceleration.y = selfInfoPtr->velocity.y;
        (*siter).second->xRelativePosition = 0.00;
        (*siter).second->yRelativePosition = 0.00;
        (*siter).second->identifierFlag = IDENTIFIER_SENSOR;
        (*siter).second->vehicleType  = TYPE_CONNECTED;  //自車両タイプ: CONNECTED
    }
    
    
    
    //------------------------------------------------------------------------------
    // 周辺車両の位置情報の更新（補間）
    //------------------------------------------------------------------------------
    for(IterType iter = storedNeighborsInfo.begin(); iter != storedNeighborsInfo.end(); iter++) {
		NodeIdType targetNodeId = (*iter).first;
		shared_ptr<VehicleInformation> neighborData = (*iter).second;

   		neighborData->sensedTime =  simulationEngineInterfacePtr->CurrentTime();
   		neighborData->ClearTable();   //GPS, Sensor, Velocityのデータ消去
        //前タイムスロットの情報を保持（更新）
   		neighborData->xPriPositionMeters = neighborData->xPositionMeters;
		neighborData->yPriPositionMeters = neighborData->yPositionMeters;
		neighborData->estPriPositionError = neighborData->estPositionError;
        
		if((neighborData->velocity.x == FLOAT_MAX)||(neighborData->velocity.y == FLOAT_MAX)){
            //平均速度で代替
			neighborData->velocity = neighborData->AvgVelocity();
		}//if
       
        //neighborData内の車両が，前タイムスロットの速度（vel_x,vel_y）で移動したと仮定して，その間の位置を線形補間
		if(selfInfoPtr->targetNodeId == neighborData->sourceNodeId){
            //自車両
			neighborData->CurrentRelativeComplement(selfInfoPtr->velocity.x, selfInfoPtr->velocity.y);
		} else {
            //周辺車両
			IterType siter = storedNeighborsInfo.find(neighborData->sourceNodeId);
			if(siter == storedNeighborsInfo.end()){
				//assert(siter != storedNeighborsInfo.end());
			} else {
                shared_ptr<VehicleInformation> sData = (*siter).second;
                neighborData->CurrentRelativeComplement(sData->velocity.x, sData->velocity.y);
            }
		}//if
	}
    
    
    //------------------------------------------------------------------------------
    // センサ検出車両の情報更新
    //------------------------------------------------------------------------------
    //get neighbors position info by sensor (modeled in VehicleSensors)
    // map::key : センサ検出車両ID
    neighborsInfoPtr = &sensorPtr->GetNeighborsPositions(simulationEngineInterfacePtr->CurrentTime()); //センサログからの情報取得
    
    for(IterType iter = neighborsInfoPtr->begin(); iter != neighborsInfoPtr->end(); iter++) {
        shared_ptr<VehicleInformation> sensorDataPtr = shared_ptr<VehicleInformation>(new VehicleInformation(*((*iter).second)));
        
        //レガシー車両の車両タイプ設定
        if(sensorDataPtr->targetNodeId >= LEGACY_NODE_ID) {
            sensorDataPtr->vehicleType = TYPE_LEGACY;
        }
        
        IterType siter = storedSensorsInfo.find(sensorDataPtr->targetNodeId);
        if (siter == storedSensorsInfo.end()) {
            //new, insert
            storedSensorsInfo.insert(make_pair(sensorDataPtr->targetNodeId,sensorDataPtr));
        } else {
            //update
            TimeType preUpdatedTime = (*siter).second->updatedTime;
            xExactPos = sensorDataPtr->xExactPosition;
            yExactPos = sensorDataPtr->yExactPosition;
            xEstPos   = sensorDataPtr->xPositionMeters;
            yEstPos   = sensorDataPtr->yPositionMeters;
            
            (*siter).second->updatedTime    = sensorDataPtr->updatedTime;
            (*siter).second->xExactPosition = xExactPos;
            (*siter).second->yExactPosition = yExactPos;
            (*siter).second->velocity.x     = sensorDataPtr->velocity.x;
            (*siter).second->velocity.y     = sensorDataPtr->velocity.y;
            (*siter).second->xPositionMeters = xEstPos;
            (*siter).second->yPositionMeters = yEstPos;
            (*siter).second->estPositionError = distance2Coordinate(xExactPos, yExactPos, xEstPos, yEstPos);
            (*siter).second->acceleration.x = sensorDataPtr->acceleration.x;
            (*siter).second->acceleration.y = sensorDataPtr->acceleration.y;
            (*siter).second->xRelativePosition = sensorDataPtr->xExactPosition - selfInfoPtr->xExactPosition;
            (*siter).second->yRelativePosition = sensorDataPtr->yExactPosition - selfInfoPtr->yExactPosition;
        }//if
    }//for


    //------------------------------------------------------------------------------
    // センサ検出車両の情報削除：車両Sについて一定時間（DETELE_TIME_SENSOR_INFO）情報が更新されない場合に，そのデータを削除
    //------------------------------------------------------------------------------
    IterType niter = storedSensorsInfo.begin();
    for (IterType niter = storedSensorsInfo.begin(); niter != storedSensorsInfo.end(); niter++) {
        NodeIdType targetNodeId = (*niter).first;
        shared_ptr<VehicleInformation> sensorData = (*niter).second;

        if (currentTime - sensorData->updatedTime >= DELETE_TIME_SENSOR_INFO)  {
            storedSensorsInfo.erase(niter++);
        }
        if (niter == storedSensorsInfo.end()) {break;}
    }//for
    
#ifdef NORMAL
    if (nodeId == 1) {
        cout << "------- storedSensorsInfo -------" << endl;
        PrintOutVehiclesInformation(storedSensorsInfo, selfInfoPtr->xExactPosition, selfInfoPtr->yExactPosition);
    }
#endif
  
#ifdef OUTPUT
    //if (nodeId == 1) {
    cout << "-----storedSensorsInfo-----" << endl;
    PrintOutVehiclesInformationForOutput(storedSensorsInfo, selfInfoPtr->xExactPosition, selfInfoPtr->yExactPosition, selfInfoPtr->identifierFlag);
    //}
#endif
    
    //------------------------------------------------------------------------------
    // センサ情報の storedNeighborsInfo への反映：センサ情報は自車が直接認識している車両情報なので，直接，周辺認識状況に組み入れる
    //------------------------------------------------------------------------------
    for (IterType siter = storedSensorsInfo.begin(); siter != storedSensorsInfo.end(); siter++) {
        NodeIdType targetNodeId = (*siter).second->targetNodeId;
        IterType niter = storedNeighborsInfo.find(targetNodeId);
        
        if (niter == storedNeighborsInfo.end()) {
            //insert
            storedNeighborsInfo.insert(make_pair(targetNodeId,(*siter).second));
        } else {
            //copy
            (*niter).second = (*siter).second;
        }
    }
    
    
    //------------------------------------------------------------------------------
    // 古いパケットの情報削除：一定時間（DETELE_TIME_PACKET_INFO），情報を再受信しなかった場合，そのデータを削除
    //------------------------------------------------------------------------------
    for (packetIterType piter = storedPacketInfo.begin(); piter != storedPacketInfo.end(); piter++) {
        NodeIdType targetNodeId = (*piter).first;
        shared_ptr<VehicleInformation> packetData = (*piter).second;

        if (currentTime - packetData->updatedTime >= DELETE_TIME_PACKET_INFO || packetData->sourceNodeId == nodeId)  {
            storedPacketInfo.erase(piter++);
        } 
        
        if (piter == storedPacketInfo.end()) {break;}
    }//for
    
#ifdef NORMAL
    cout << "------- storedPacketInfo -------" << endl;
    PrintOutPacketsInformation(storedPacketInfo);
#endif

#ifdef OUTPUT
    //if (nodeId == 1) {
    cout << "-----storedPacketInfo-----" << endl;
    PrintOutPacketsInformationForOutput(storedPacketInfo, selfInfoPtr->xExactPosition, selfInfoPtr->yExactPosition, selfInfoPtr->identifierFlag);
    //}
#endif
    
    
    //------------------------------------------------------------------------------
    // 受信パケット情報を利用したマッチングプロセス
    //------------------------------------------------------------------------------
    //[想定] MATCHING_DURATION後に，車両が100%特定できる
    for (packetIterType piter = storedPacketInfo.begin(); piter != storedPacketInfo.end(); piter++) {
        //NodeIdType sourceNodeId = (*piter).second->sourceNodeId;
        NodeIdType targetNodeId = (*piter).second->targetNodeId;
        //IterType sendIter = storedNeighborsInfo.find(sourceNodeId);
        IterType niter = storedNeighborsInfo.find(targetNodeId);
        IterType siter = storedSensorsInfo.find(targetNodeId);
        
        if (targetNodeId == nodeId) {continue;}
        if (siter != storedSensorsInfo.end()) {continue;}  //センサ情報がある場合，こちらの方が精度が高いと仮定
        
        if (niter == storedNeighborsInfo.end()) {
            //not found, add
            shared_ptr<VehicleInformation> packetData = (*piter).second;
            //packetData->sourceNodeId = nodeId;
            //packetData->targetNodeId = targetNodeId;
            storedNeighborsInfo.insert(make_pair(targetNodeId, packetData));
        } else {
            //found,update
            //複数の車両から車両Vについての情報を複数回受信する場合，以下が複数回処理されることになる
            //複数回代入しても意味がないので，移動平均的な計算とすることが本来必要
            //本当はmultimapにする必要がある
            shared_ptr<VehicleInformation> neighborData = (*niter).second;
            
            neighborData->updatedTime    = (*piter).second->updatedTime;
            neighborData->xExactPosition = (*piter).second->xExactPosition;
            neighborData->yExactPosition = (*piter).second->yExactPosition;
            neighborData->velocity.x     = (*piter).second->velocity.x;
            neighborData->velocity.y     = (*piter).second->velocity.y;
            neighborData->xPositionMeters = (*piter).second->xPositionMeters;
            neighborData->yPositionMeters = (*piter).second->yPositionMeters;
            neighborData->estPositionError = (*piter).second->estPositionError;
            neighborData->acceleration.x = (*piter).second->acceleration.x;
            neighborData->acceleration.y = (*piter).second->acceleration.y;
            neighborData->xRelativePosition = (*piter).second->xRelativePosition;
            neighborData->yRelativePosition = (*piter).second->yRelativePosition;
            if ((*piter).second->identifierFlag == IDENTIFIER_SENSOR) {
                neighborData->identifierFlag = IDENTIFIER_SENSOR_V2V;
                neighborData->vehicleType = TYPE_CONNECTED;
            }
        }//if
    }//for
    
    
    
    //------------------------------------------------------------------------------
    // マッチングに基づいた車両タイプ情報の判別・更新
    //------------------------------------------------------------------------------
    for (IterType piter = storedPacketInfo.begin(); piter != storedPacketInfo.end(); piter++) {
        NodeIdType packetSendId = (*piter).second->sourceNodeId;
        IterType niter = storedNeighborsInfo.find(packetSendId);
        IterType siter = storedSensorsInfo.find(packetSendId);
        
        if (niter != storedNeighborsInfo.end() && siter != storedSensorsInfo.end()) {
            (*niter).second->identifierFlag = IDENTIFIER_SENSOR_V2V; //V2Vメッセージを正しくマッチングし，かつセンサ車両である
            (*niter).second->vehicleType = TYPE_CONNECTED;
        } else if (niter != storedNeighborsInfo.end()) {
            (*niter).second->identifierFlag = IDENTIFIER_V2V;
            (*niter).second->vehicleType = TYPE_CONNECTED;
        }
        (*piter).second->msgOverhead = 0; //適当なタイミングでオーバーヘッドを初期化する必要がある
    }//for
    

    
    //------------------------------------------------------------------------------
    // 古い周辺認識情報の削除：一定時間（DETELE_TIME_PACKET_INFO），情報が更新されなかった場合，そのデータを削除
    //------------------------------------------------------------------------------
    //12/17: Abort trapが発生するので，コメントアウト
    
    for (IterType nniter = storedNeighborsInfo.begin(); nniter != storedNeighborsInfo.end(); nniter++) {
        if (currentTime - (*nniter).second->updatedTime >= DELETE_TIME_PACKET_INFO)  {
            storedNeighborsInfo.erase(nniter++);
        } 
        
        if (nniter == storedNeighborsInfo.end()) {break;}
    }//for
    
    
    
#ifdef OUTPUT
    //if (nodeId == 1) {
    cout << "-----storedNeighborsInfo-----" << endl;
    //cout << selfInfoPtr->xExactPosition << ", " << selfInfoPtr->yExactPosition << endl;
    PrintOutVehiclesInformationForOutput(storedNeighborsInfo, selfInfoPtr->xExactPosition, selfInfoPtr->yExactPosition, selfInfoPtr->identifierFlag);
    //}
#endif
    
#ifdef NORMAL
    if (nodeId == 1) {
    cout << "-----storedNeighborsInfo-----" << endl;
    //cout << selfInfoPtr->xExactPosition << ", " << selfInfoPtr->yExactPosition << endl;
    PrintOutVehiclesInformation(storedNeighborsInfo, selfInfoPtr->xExactPosition, selfInfoPtr->yExactPosition);
    }
#endif
    
    
    //------------------------------------------------------------------------------
    // 優先度評価関数の計算（計算実行車両：自車両，計算対象車両：周辺認識車両）
    //------------------------------------------------------------------------------
    //
    // 時間，距離のfactorは一概に長ければ良い，短ければ良い，といったことが言えない．MoNAでは以下の3つのファクターを採用
    // 1.車線（横方向の距離差）
    // 2.相対速度差
    // 3.車両タイプ
    for (IterType niter = storedNeighborsInfo.begin(); niter != storedNeighborsInfo.end(); niter++) {
        NodeIdType targetNodeId = (*niter).first;
        shared_ptr<VehicleInformation> neighborData = (*niter).second;
        neighborData->priority = calculatePriority(targetNodeId, neighborData, selfInfoPtr);
        
        if (neighborData->priority > PRIORITY_MAX) {
            neighborData->priority =  PRIORITY_MAX;
        } else if (neighborData->priority < 1.0){
            neighborData->priority = 1.0;
        }
    }//for
    
    
    //------------------------------------------------------------------------------
    // 優先度評価関数の計算（計算実行車両：自車両のセンサ検出車両，計算対象車両：周辺認識車両）
    //------------------------------------------------------------------------------
#ifdef PROPOSED_REVISION
    for (IterType niter = storedSensorsInfo.begin(); niter != storedSensorsInfo.end(); niter++) {
        if ((*niter).second->targetNodeId == nodeId || (*niter).second->identifierFlag != IDENTIFIER_SENSOR_V2V) {continue;} //自車両判定不要，先進車両以外は判定不要
        
        NodeIdType sensorNodeId = (*niter).second->targetNodeId; //計算実行車両AのID
        shared_ptr<VehicleInformation> sensorNodeData =  (*niter).second;
        
        float sensorNodeXPosition = (*niter).second->xExactPosition;
        float sensorNodeYPosition = (*niter).second->yExactPosition;
        VectorType sensorMovingDirection = VectorType((*niter).second->velocity.x, (*niter).second->velocity.y);
        
        // センシング車両集合を全探索し，計算対象車両Bが存在するかを位置情報をベースに判定（真値はセンサログ）--> priority計算
        for (IterType siter = storedNeighborsInfo.begin();  siter != storedNeighborsInfo.end(); siter++) {
            VectorType relCoord = VectorType((*siter).second->xExactPosition - sensorNodeXPosition, (*siter).second->yExactPosition - sensorNodeYPosition);
            //if (!isDetectableBySensor(relCoord, sensorMovingDirection)) { continue; } //計算対象車両内に存在せず
            
            float comparedPriority = calculatePriority((*siter).second->targetNodeId, (*siter).second, sensorNodeData);
            
            
            //検出可能であった-->priority計算
            //cout << "SUR_PRIORITY:: sensorNodeId:" << sensorNodeId <<", " <<  sensorNodeXPosition << ", " << sensorNodeYPosition << ", sensorTargetNodeId:"
            //     <<  (*siter).second->targetNodeId  <<", " <<  (*siter).second->xExactPosition << ", " << (*siter).second->yExactPosition
            //     << ", comparedPriority:" << comparedPriority << ", Priority:" << (*siter).second->priority << endl;
            
            float diffPriority = comparedPriority - (*siter).second->priority;
            //比較対象の優先度の方が高い場合，その差分値を基準にして，優先度を下げる
            if (diffPriority > 0) {
                (*siter).second->priority -= PRIORITY_DECREASE * diffPriority;
                if ((*siter).second->priority < 1.0) {
                    (*siter).second->priority= 1.0;  //最小値設定
                }
            }
            
            //test (*siter).second->priority= 1.0;
            
        }//for
    }//for
#endif

#ifdef PROPOSED_REVISION_2
    for (IterType niter = storedSensorsInfo.begin(); niter != storedSensorsInfo.end(); niter++) {
        if ((*niter).second->targetNodeId == nodeId || (*niter).second->identifierFlag != IDENTIFIER_SENSOR_V2V) {continue;} //自車両判定不要，先進車両以外は判定不要
        
        NodeIdType sensorNodeId = (*niter).second->targetNodeId; //計算実行車両AのID
        shared_ptr<VehicleInformation> sensorNodeData =  (*niter).second;
        
        float sensorNodeXPosition = (*niter).second->xExactPosition;
        float sensorNodeYPosition = (*niter).second->yExactPosition;
        VectorType sensorMovingDirection = VectorType((*niter).second->velocity.x, (*niter).second->velocity.y);
        
        IterType ssiter;
        int numInfoCounter = 0;
        for (ssiter = storedPacketInfo.begin(); ssiter != storedPacketInfo.end(); ssiter++) {
            if ((*ssiter).second->sourceNodeId == sensorNodeId) {
            
                NodeIdType tmpTargetId = (*ssiter).second->targetNodeId;
                IterType nniter = storedNeighborsInfo.find(tmpTargetId);
                
                if (nniter != storedNeighborsInfo.end()){
                    numInfoCounter += 1;
                }
            }
        }//for
        
        //センシング車両が自車が保持する複数台の車両情報を送信しているので自車は送信頻度を下げても良い．
        IterType sssiter;
        for (sssiter = storedPacketInfo.begin(); sssiter != storedPacketInfo.end(); sssiter++) {
            if ((*sssiter).second->sourceNodeId == sensorNodeId) {
            
                NodeIdType tmpTargetId2 = (*ssiter).second->targetNodeId;
                IterType nnniter = storedNeighborsInfo.find(tmpTargetId2);
                
                if (nnniter != storedNeighborsInfo.end()){
                    (*nnniter).second->priority = (*nnniter).second->priority - numInfoCounter/2;
                    if ((*siter).second->priority < 1.0) {
                        (*siter).second->priority= 1.0;  //最小値設定
                    }
                }
            }
        }//for
    }//for
#endif
    
    //storedNeighborInfoに登録したので，パケット情報は初期化
    storedPacketInfo.clear();

    //-------------------------------------------------------------------------------------
    // 1. 提案手法（送信優先度の評価関数で送信車両を決定）
    // 2. 提案手法rev.（周辺車両の優先度を考慮しない）
    // 3. 比較手法1（センシング車両情報をすべて送信）
    // 4. 比較手法2（センシング車両情報の中の各車両について送信するかどうかをランダムに決定し送信）
    //-------------------------------------------------------------------------------------
    
    
#if defined(PROPOSED) || defined(PROPOSED_REVISION) || defined(PROPOSED_REVISION_2)
    size_t vehicleNum = 0;
    //一様分布[0.0, 3.0]の生成器
    std::random_device seed_gen;
    std::default_random_engine engine(seed_gen());
    std::uniform_real_distribution<> rand10(0.0, 3.0);
    
    cout << "-----BSM-PART3-----" << endl;
    cout << nodeId  << ", " << selfInfoPtr->xExactPosition << ", " << selfInfoPtr->yExactPosition << ", " << selfInfoPtr->identifierFlag;
    for (IterType iter = storedSensorsInfo.begin(); iter != storedSensorsInfo.end(); iter++){
        NodeIdType targetNodeId = (*iter).first;
        shared_ptr<VehicleInformation> neighborData = (*iter).second;
        
        if((currentTime - neighborData->updatedTime) > DELETE_TIME_SENSOR_INFO) {continue;} //古い情報は送信しない
        
        if ( abs(neighborData->lastSendTime - currentTime) <= 95*MILLI_SECOND) {
                cout  << ", " <<  targetNodeId << ", " << neighborData->xExactPosition << ", " << neighborData->yExactPosition << ", " << neighborData->identifierFlag <<  ", "
                     << neighborData->priority ;
            
            //現在時刻で送信する場合
            //cout << "debug:" << neighborData->lastSendTime << "," << currentTime << endl;
            //本来，ソース車両とターゲット車両の概念があってしかるべきだと思う．
            //neighborData->lastSendTime = currentTime;
        
            DsrcBasicSafetyMessagePart3Type basicSafetyMessagePart3;
            basicSafetyMessagePart3.SetXMeters(neighborData->xExactPosition);
            basicSafetyMessagePart3.SetYMeters(neighborData->yExactPosition);
            basicSafetyMessagePart3.SetXVelocity(neighborData->velocity.x);
            basicSafetyMessagePart3.SetYVelocity(neighborData->velocity.y);
            basicSafetyMessagePart3.SetAccuracy(0);
            basicSafetyMessagePart3.SetSourceId(neighborData->sourceNodeId);
            basicSafetyMessagePart3.SetTargetId(targetNodeId);
            sendPart3Data.insert(make_pair(targetNodeId, basicSafetyMessagePart3));
            vehicleNum += 1;
        } else if (currentTime > neighborData->lastSendTime) {
            //現在時刻が次の送信時刻を超えた⇒送信が終了しているため，現在の優先度に従い，次の送信タイミングを再計算
            TimeType sendTiming = floor(MAX_LAST_SEND_TIME *MILLI_SECOND / neighborData->priority);  //送信頻度，1秒の間に何回送信するか．単位MS
            //int sendTiming = floor(neighborData->priority);  //送信頻度，1秒の間に何回送信するか．単位MS

            TimeType minTime = 1000*MILLI_SECOND;  //1s
        
            sendTiming += rand10(engine)*100*MILLI_SECOND;
            
            
            //以下の乱数の使い方だと明らかに偏りが出る
            /*
            for(int i = 0; i < sendTiming; i++) {
                TimeType tmpRandTime = floor(rand10(engine)*100*MILLI_SECOND);
                if (tmpRandTime < minTime) {
                    minTime = tmpRandTime;
                }
            }
            */
            
            TimeType nextSendTime = currentTime + sendTiming;
            neighborData->lastSendTime = nextSendTime;  //本当はおかしい．後で直すこと
            //cout << "MINTIME:" << neighborData->lastSendTime << endl;
        }//if
    }//for
    
    cout << endl;
#endif


#ifdef COMPARED_ALL
    size_t vehicleNum = 0;
    cout << "-----BSM-PART3-----" << endl;
    cout << nodeId  << ", " << selfInfoPtr->xExactPosition << ", " << selfInfoPtr->yExactPosition << ", " << selfInfoPtr->identifierFlag;
    for (IterType iter = storedSensorsInfo.begin(); iter != storedSensorsInfo.end(); iter++){
        NodeIdType targetNodeId = (*iter).first;
        shared_ptr<VehicleInformation> neighborData = (*iter).second;
        
        if((currentTime - neighborData->updatedTime) > DELETE_TIME_SENSOR_INFO) {continue;} //古い情報は送信しない

        cout  << ", " <<  targetNodeId << ", " << neighborData->xExactPosition << ", " << neighborData->yExactPosition << ", " << neighborData->identifierFlag <<  ", "
              << neighborData->priority ;
            
        //本来，ソース車両とターゲット車両の概念があってしかるべきだと思う．
        neighborData->lastSendTime = currentTime;
        
        DsrcBasicSafetyMessagePart3Type basicSafetyMessagePart3;
        basicSafetyMessagePart3.SetXMeters(neighborData->xExactPosition);
        basicSafetyMessagePart3.SetYMeters(neighborData->yExactPosition);
        basicSafetyMessagePart3.SetXVelocity(neighborData->velocity.x);
        basicSafetyMessagePart3.SetYVelocity(neighborData->velocity.y);
        basicSafetyMessagePart3.SetAccuracy(0);
        basicSafetyMessagePart3.SetSourceId(neighborData->sourceNodeId);
        basicSafetyMessagePart3.SetTargetId(targetNodeId);
        sendPart3Data.insert(make_pair(targetNodeId, basicSafetyMessagePart3));
        vehicleNum += 1;
    }//for
    cout << endl;
#endif


#ifdef COMPARED_RANDOM
    //一様分布[0.0, 1.0]の生成器
    std::random_device seed_gen;
    std::default_random_engine engine(seed_gen());
    std::uniform_real_distribution<> rand01(0.0, 1.0);
    
    size_t vehicleNum = 0;
    cout << "-----BSM-PART3-----" << endl;
    cout << nodeId  << ", " << selfInfoPtr->xExactPosition << ", " << selfInfoPtr->yExactPosition << ", " << selfInfoPtr->identifierFlag;
    for (IterType iter = storedSensorsInfo.begin(); iter != storedSensorsInfo.end(); iter++){
        NodeIdType targetNodeId = (*iter).first;
        shared_ptr<VehicleInformation> neighborData = (*iter).second;
        
        
        if((currentTime - neighborData->updatedTime) > DELETE_TIME_SENSOR_INFO) {continue;} //古い情報は送信しない

        //乱数値が0.5以上であるときに限り，送信
        if (rand01(engine) > 0.5) {
            cout  << ", " <<  targetNodeId << ", " << neighborData->xExactPosition << ", " << neighborData->yExactPosition << ", " << neighborData->identifierFlag <<  ", "
                << neighborData->priority ;
            
            //本来，ソース車両とターゲット車両の概念があってしかるべきだと思う．
            neighborData->lastSendTime = currentTime;
        
            DsrcBasicSafetyMessagePart3Type basicSafetyMessagePart3;
            basicSafetyMessagePart3.SetXMeters(neighborData->xExactPosition);
            basicSafetyMessagePart3.SetYMeters(neighborData->yExactPosition);
            basicSafetyMessagePart3.SetXVelocity(neighborData->velocity.x);
            basicSafetyMessagePart3.SetYVelocity(neighborData->velocity.y);
            basicSafetyMessagePart3.SetAccuracy(0);
            basicSafetyMessagePart3.SetSourceId(neighborData->sourceNodeId);
            basicSafetyMessagePart3.SetTargetId(targetNodeId);
            sendPart3Data.insert(make_pair(targetNodeId, basicSafetyMessagePart3));
            vehicleNum += 1;
        }//if
        
    }//for
    cout << endl;
#endif

    
    size_t part3PayloadSize = vehicleNum*BSM_PART3_PACKET_LENGTH;
    
    if (vehicleNum > 0) {
        (*this).SendBasicSafetyMessage(
            basicSafetyMessagePart1,
            sendPart3Data,
            part3PayloadSize);
    } else {
        //自車情報のみ
        (*this).SendBasicSafetyMessage(
            basicSafetyMessagePart1);
    }
    
    sendPart3Data.clear();
    
    //overheadの初期化式をこの変に入れる？

    if (currentTime + basicSafetyMessageInfo.transmissionInterval < basicSafetyMessageInfo.endTime) {
        simulationEngineInterfacePtr->ScheduleEvent(
            unique_ptr<SimulationEvent>(
                new PeriodicBasicSafetyMessageTransmissionEvent(this)),
            (currentTime + basicSafetyMessageInfo.transmissionInterval));
    }//if//
}//PeriodicallyTransmitBasicSafetyMessage//


inline
void DsrcMessageApplication::ReceivePacketFromLowerLayer(unique_ptr<Packet>& packetPtr)
{
    const DsrcMessageIdType messageId = packetPtr->GetAndReinterpretPayloadData<DsrcMessageIdType>();

    switch (messageId) {
    
    case DSRC_MESSAGE_BASIC_SAFETY:
        (*this).ReceiveBasicSafetyMessage(packetPtr);
        break;

    default:
        assert("Received not supported DSRC message type");
        break;
    }//switch//

    packetPtr = nullptr;

}//ReceivePacketFromLowerLayer//




//------------------------------------------------------------------------------
// BSM受信時の処理：周辺車両からメッセージを受信する度に呼び出される
//------------------------------------------------------------------------------
inline
void DsrcMessageApplication::ReceiveBasicSafetyMessage(unique_ptr<Packet>& packetPtr)
{
    typedef multimap<NodeIdType, shared_ptr<VehicleInformation> >::iterator nIterType;
    typedef map<NodeIdType, shared_ptr<VehicleInformation> >::iterator IterType;
    int payloadOffset = 0;
    
    DsrcBasicSafetyMessagePart1Type part1 =
        packetPtr->GetAndReinterpretPayloadData<DsrcBasicSafetyMessagePart1Type>(payloadOffset,BSM_PART1_PACKET_LENGTH);
    payloadOffset += BSM_PART1_PACKET_LENGTH;
    
    shared_ptr<VehicleInformation> packetInfoPtr(new VehicleInformation);
    packetInfoPtr->updatedTime  = part1.GetTime();
    packetInfoPtr->sourceNodeId = part1.GetSourceId();
    packetInfoPtr->targetNodeId = part1.GetTargetId(); //sourceId = targetId
    packetInfoPtr->xExactPosition = part1.GetXMeters();
    packetInfoPtr->yExactPosition = part1.GetYMeters();
    packetInfoPtr->velocity.x = part1.GetXVelocity();
    packetInfoPtr->velocity.y = part1.GetYVelocity();
    packetInfoPtr->xPositionMeters = part1.GetGpsXMeters();
    packetInfoPtr->xPositionMeters = part1.GetGpsYMeters();
    packetInfoPtr->estPositionError = part1.GetAccuracy();
    packetInfoPtr->acceleration.x  = part1.GetXAccel();
    packetInfoPtr->acceleration.y  = part1.GetYAccel();
    packetInfoPtr->xRelativePosition = 0.00; //送信元車両からの相対座標なので0
    packetInfoPtr->yRelativePosition = 0.00; //送信元車両からの相対座標なので0
    IterType siter = storedSensorsInfo.find(packetInfoPtr->sourceNodeId);
    if (siter != storedSensorsInfo.end()) {
        //センサ車両集合に存在する場合．車両タイプをCONNECTEDに変更
        packetInfoPtr->identifierFlag = IDENTIFIER_SENSOR_V2V;
        (*siter).second->identifierFlag = IDENTIFIER_SENSOR_V2V;
        (*siter).second->vehicleType = TYPE_CONNECTED;
    } else {
        //Part1なのでメッセージ送信元車両である
        packetInfoPtr->identifierFlag = IDENTIFIER_V2V_SENDER;
    }
    packetInfoPtr->vehicleType = TYPE_CONNECTED;
    
    
    NodeIdType sourceId = packetInfoPtr->sourceNodeId;
    TimeType   receivedTime = packetInfoPtr->updatedTime;
    
    //自車両情報はstoreしない
    if(sourceId == nodeId) {return;}
    
    nIterType niter;
    for(niter = storedPacketInfo.begin(); niter != storedPacketInfo.end(); niter++) {
        if (((*niter).second->sourceNodeId == packetInfoPtr->sourceNodeId) && ((*niter).second->targetNodeId == packetInfoPtr->targetNodeId)){
            //found, update
            (*niter).second->updatedTime    = packetInfoPtr->updatedTime;
            (*niter).second->xExactPosition = packetInfoPtr->xExactPosition;
            (*niter).second->yExactPosition = packetInfoPtr->yExactPosition;
            (*niter).second->velocity.x     = packetInfoPtr->velocity.x;
            (*niter).second->velocity.y     = packetInfoPtr->velocity.y;
            (*niter).second->xPositionMeters = packetInfoPtr->xExactPosition;
            (*niter).second->yPositionMeters = packetInfoPtr->yExactPosition;
            (*niter).second->estPositionError = 0;
            (*niter).second->acceleration.x = packetInfoPtr->velocity.x;
            (*niter).second->acceleration.y = packetInfoPtr->velocity.y;
            (*niter).second->xRelativePosition = packetInfoPtr->xRelativePosition;
            (*niter).second->yRelativePosition = packetInfoPtr->yRelativePosition;
            (*niter).second->identifierFlag = packetInfoPtr->identifierFlag;
            (*niter).second->vehicleType = TYPE_CONNECTED;
            (*niter).second->msgOverhead += 1;
            break; //見つかったのでループから抜ける
        }//if
    }
    
    if(niter == storedPacketInfo.end() && sourceId != nodeId) {
        assert(sourceId != nodeId);
        storedPacketInfo.insert(make_pair(sourceId,packetInfoPtr)); //見つからなかったのでinsert，送信元車両IDをキーとして登録
    }
	
    int bsmPart3Number = (packetPtr->LengthBytes() - BSM_PART1_PACKET_LENGTH) / BSM_PART3_PACKET_LENGTH;  //Part3に含まれる車両台数
    for (int i = 0; i < bsmPart3Number; i++) {
        DsrcBasicSafetyMessagePart3Type part3 =
            packetPtr->GetAndReinterpretPayloadData<DsrcBasicSafetyMessagePart3Type>(payloadOffset,BSM_PART3_PACKET_LENGTH);
        payloadOffset += BSM_PART3_PACKET_LENGTH;
            
        shared_ptr<VehicleInformation> neighborInfoPtr(new VehicleInformation);
        neighborInfoPtr->sourceNodeId = part3.GetSourceId();
        neighborInfoPtr->targetNodeId = part3.GetTargetId();
        neighborInfoPtr->xExactPosition = part3.GetXMeters();
        neighborInfoPtr->yExactPosition = part3.GetYMeters();
        neighborInfoPtr->velocity.x = part3.GetXVelocity();
        neighborInfoPtr->velocity.y = part3.GetYVelocity();
        neighborInfoPtr->estPositionError = part3.GetAccuracy();
        neighborInfoPtr->xRelativePosition = part3.GetXMeters() - packetInfoPtr->xExactPosition; //相対位置（要確認）
        neighborInfoPtr->yRelativePosition = part3.GetYMeters() - packetInfoPtr->yExactPosition; //相対位置（要確認）
        neighborInfoPtr->identifierFlag = IDENTIFIER_V2V;
        //neighborInfoPtr->sourceNodeId = sourceId; //BSM Part1の車両ID
        neighborInfoPtr->updatedTime  = receivedTime; //BSM Part1の受信時刻
        
        //自車両情報はstoreしない
        if(sourceId == nodeId) {continue;}
        
        // storedPacketInfo の更新・追加
        //source = 送信元車両のID, target = 送信情報内の車両ID
        //targetの車両情報が複数回送信されたかを確認する部分はこの部分
        nIterType nniter;
        for (nniter = storedPacketInfo.begin(); nniter != storedPacketInfo.end(); nniter++) {
            if (((*nniter).second->sourceNodeId == neighborInfoPtr->sourceNodeId) && ((*nniter).second->targetNodeId == neighborInfoPtr->targetNodeId)) {
                //found, update
                (*nniter).second->sourceNodeId = neighborInfoPtr->sourceNodeId;
                (*nniter).second->updatedTime = neighborInfoPtr->updatedTime;
                (*nniter).second->xExactPosition = neighborInfoPtr->xExactPosition;
                (*nniter).second->yExactPosition = neighborInfoPtr->yExactPosition;
                (*nniter).second->velocity.x = neighborInfoPtr->velocity.x;
                (*nniter).second->velocity.y = neighborInfoPtr->velocity.y;
                (*nniter).second->xPositionMeters = neighborInfoPtr->xPositionMeters;
                (*nniter).second->yPositionMeters = neighborInfoPtr->yPositionMeters;
                (*nniter).second->xRelativePosition = neighborInfoPtr->xRelativePosition;
                (*nniter).second->yRelativePosition = neighborInfoPtr->yRelativePosition;
                (*nniter).second->identifierFlag = neighborInfoPtr->identifierFlag;
                (*nniter).second->msgOverhead += 1;
                break;
            }
        }
            
        if (nniter == storedPacketInfo.end() && sourceId != nodeId) {
            storedPacketInfo.insert(make_pair(sourceId,neighborInfoPtr)); //見つからなかったのでinsert，送信元車両IDをキーとして登録
        }
    } //for
	
    const DsrcPacketExtrinsicInformation& extInfo =
        packetPtr->GetExtrinsicPacketInformation<DsrcPacketExtrinsicInformation>(
            DsrcPacketExtrinsicInformation::id);

    const TimeType delay =
        simulationEngineInterfacePtr->CurrentTime() - extInfo.transmissionTime;

    basicSafetyMessageInfo.numberPacketsReceived++;
    
    (*this).OutputTraceAndStatsForReceiveBasicSafetyMessage(
        extInfo.sequenceNumber,
        packetPtr->GetPacketId(),
        packetPtr->LengthBytes(),
        delay);
}//ReceiveBasicSafetyMessage//



//-----------------------------------------------------------------------
// 送信優先度の評価関数の計算
//-----------------------------------------------------------------------
inline
float DsrcMessageApplication::calculatePriority(NodeIdType targetNodeId, shared_ptr<VehicleInformation> neighborData, shared_ptr<VehicleInformation> selfInfoPtr) {
    float complement = PRIORITY_MAX / NormalDistribution(0.0, 0.0, 40.0); // F_LANE計算における補正式

    //対処：ノードが静止状態であるとき，F_LANEの計算でNaNとなるので，x方向の微小速度を与える
    if (selfInfoPtr->velocity.x == 0.0 && selfInfoPtr->velocity.y == 0.0) {
            selfInfoPtr->velocity.x = 0.001;
    }

    //------------------------------------------------------------------------------
    // F_LANE : 車線位置
    //------------------------------------------------------------------------------
    //点と直線の距離（外積利用）により，相対位置（車線レベル）を計算
    VectorType headVector = VectorType(selfInfoPtr->velocity.x, selfInfoPtr->velocity.y);
    VectorType relVector = VectorType(neighborData->xExactPosition-selfInfoPtr->xExactPosition, neighborData->yExactPosition-selfInfoPtr->yExactPosition);
    
    float crossProduct = headVector.x * relVector.y - headVector.y * relVector.x;
    float distHeadVector = distance2Coordinate(selfInfoPtr->velocity.x, selfInfoPtr->velocity.y, 0, 0);
    float distRelLane = crossProduct/distHeadVector;
        
    // LANE_WIDTH (default)5.00m
    //cout << "nodeId:" << nodeId << "," << neighborData->targetNodeId << ", " << distRelLane << ", " << headVector.x << ", " << headVector.y << ", " << neighborData->velocity.x << ", " << neighborData->velocity.y << endl;
    //固定値の割当の場合
    if (abs(distRelLane) < LANE_WIDTH) {
        neighborData->lanePosition = 10;
    } else if (distRelLane >= LANE_WIDTH && distRelLane < LANE_WIDTH*2){
        neighborData->lanePosition = 5;
    } else if (distRelLane >= LANE_WIDTH*2 && distRelLane < LANE_WIDTH*3 ){
        neighborData->lanePosition = 2;
    } else if (distRelLane < (-1)*LANE_WIDTH && distRelLane >= (-2)*LANE_WIDTH ){
        neighborData->lanePosition = 5;
    } else if (distRelLane < (-2)*LANE_WIDTH && distRelLane >= (-3)*LANE_WIDTH ){
        neighborData->lanePosition = 2;
    } else {
        neighborData->lanePosition = INIT_LANEPOSITION;
    }
        
    float factorLane = complement * NormalDistribution(distRelLane, 0.0, 40.0);
        
    //------------------------------------------------------------------------------
    // F_TYPE : 車両タイプ
    //------------------------------------------------------------------------------
    // TYPE_UNKNOWN, TYPE_CONNECTED, TYPE_LEGACY（VehicleInformationの中で登録済み）
    float factorType;
    if (neighborData->vehicleType == TYPE_CONNECTED) {
        factorType = 1;  // or 1
    } else if (neighborData->vehicleType == TYPE_LEGACY) {
        factorType = PRIORITY_MAX;
    } else {
        factorType = PRIORITY_MAX/2;  //undermined vehicle
    }
        
    //------------------------------------------------------------------------------
    // F_SPEED : 相対速度（大きさ）
    //------------------------------------------------------------------------------
    float tmpFactorSpeed;
    if (neighborData->velocity.x == FLOAT_MAX && neighborData->velocity.y == FLOAT_MAX) {
        tmpFactorSpeed = PRIORITY_MAX/2;
    } else {
        tmpFactorSpeed = distance2Coordinate(neighborData->velocity.x, neighborData->velocity.y, selfInfoPtr->velocity.x, selfInfoPtr->velocity.y);
    }
    float factorSpeed = PRIORITY_MAX*exp((-1)*0.2*tmpFactorSpeed);
    
    //------------------------------------------------------------------------------
    // Priority計算
    //------------------------------------------------------------------------------
        
    //neighborData->priority = WEIGHT_TYPE * factorType + WEIGHT_VELOCITY * factorSpeed + WEIGHT_LANE * factorLane;
    
    float priority = WEIGHT_TYPE * factorType + WEIGHT_VELOCITY * factorSpeed + WEIGHT_LANE * factorLane;
    
    if (priority < 1.0) {
        priority = 1.0;  //最小値設定
    } else if (priority > PRIORITY_MAX) {
        priority = PRIORITY_MAX; //最大値設定
    }
    
    return priority;
} //calculatePriority


inline
void DsrcMessageApplication::PrintOutForReceivedMessage(DsrcBasicSafetyMessagePart1Type& ptr) {
    cout << "nodeID, <BSM Part1> sourceId, time, targetId, XMeters, YMeters, XVelocity, YVelocity, Accuracy, Heading, GPSX, GPSY, AccelX, AccelY" << endl;
    cout << nodeId << ", "
         << ptr.GetSourceId() << ", "
		 << ptr.GetTime() / MILLI_SECOND << ", "
		 << ptr.GetTargetId()  << ", "
		 //<< ptr.GetXMeters() << ", "
		 //<< ptr.GetYMeters() << ", "
		 //<< ptr.GetXVelocity() << ", "
         //<< ptr.GetYVelocity() << ", "
		 //<< ptr.GetAccurancy().semiMajorAccurancyMeters << ", "
		 //<< ptr.GetHeading() << ", "
         //<< ptr.GetGpsXMeters() << ", "
		 //<< ptr.GetGpsYMeters() << ", "
		 //<< ptr.GetAccelSet().accelerationX << ", "
         //<< ptr.GetAccelSet().accelerationY << ", "
         << endl;
} //PrintOutForReceivedMessage


inline
void DsrcMessageApplication::PrintOutForReceivedMessage(DsrcBasicSafetyMessagePart3Type& ptr) {
    cout << "nodeID, <BSM Part3> sourceId, time, targetId, XMeters, YMeters, XVelocity, YVelocity, XSensors, YSensors, Accuracy, likelihood" << endl;
    cout << nodeId << ", "
		 << ptr.GetTargetId()  << ", "
		 << ptr.GetXMeters() << ", "
		 << ptr.GetYMeters() << ", "
		 << ptr.GetXVelocity() << ", "
         << ptr.GetYVelocity()
         << endl;
} //PrintOutForReceivedMessage


inline
void DsrcMessageApplication::PrintOutNeighborData(shared_ptr<VehicleInformation>& ptr) {
    cout << "(target, x, y, vel_x, vel_y, flag):" << ptr->targetNodeId << ", " << ptr->xExactPosition
         << ", " << ptr->yExactPosition << ", " << ptr->velocity.x << ", " << ptr->velocity.y << ", " << ptr->identifierFlag << endl;
}

inline
void DsrcMessageApplication::PrintOutVehiclesInformation(map<NodeIdType, shared_ptr<VehicleInformation> > table, float selfXPos, float selfYPos) {
    typedef map<NodeIdType, shared_ptr<VehicleInformation> >::iterator nIterType;
    for (nIterType iter = table.begin(); iter != table.end(); iter++) {
        float relativeDist = distance2Coordinate((*iter).second->xExactPosition, (*iter).second->yExactPosition, selfXPos, selfYPos);
        
        if ((*iter).second->identifierFlag == IDENTIFIER_SENSOR) {
            cout << "nodeId=" << nodeId << ", sourceNodeId=" << (*iter).second->sourceNodeId <<  ", " << "TargetNodeId=" << (*iter).second->targetNodeId << ", UpdatedTime=" << (*iter).second->updatedTime/ MILLI_SECOND
                << ", " << (*iter).second->xExactPosition << ", " << (*iter).second->yExactPosition << ", Type:Sensor" << ", relDist:" << relativeDist << endl;
        } else if ((*iter).second->identifierFlag == IDENTIFIER_SENSOR_V2V) {
            cout << "nodeId=" << nodeId << ", sourceNodeId=" << (*iter).second->sourceNodeId <<  ", " << "TargetNodeId=" << (*iter).second->targetNodeId << ", UpdatedTime=" << (*iter).second->updatedTime/ MILLI_SECOND
                << ", " << (*iter).second->xExactPosition << ", " << (*iter).second->yExactPosition << ", Type:Sensor+V2V" << ", relDist:" << relativeDist << endl;
        } else if ((*iter).second->identifierFlag == IDENTIFIER_V2V) {
            cout << "nodeId=" << nodeId << ", sourceNodeId=" << (*iter).second->sourceNodeId <<  ", " << "TargetNodeId=" << (*iter).second->targetNodeId << ", UpdatedTime=" << (*iter).second->updatedTime/ MILLI_SECOND
                << ", " << (*iter).second->xExactPosition << ", " << (*iter).second->yExactPosition << ", Type:V2V" << ", relDist:" << relativeDist << endl;
        } else if ((*iter).second->identifierFlag == IDENTIFIER_V2V_SENDER) {
            cout << "nodeId=" << nodeId << ", sourceNodeId=" << (*iter).second->sourceNodeId <<  ", " << "TargetNodeId=" << (*iter).second->targetNodeId << ", UpdatedTime=" << (*iter).second->updatedTime/ MILLI_SECOND
                << ", " << (*iter).second->xExactPosition << ", " << (*iter).second->yExactPosition << ", Type:V2V(Sender)" << ", relDist:" << relativeDist << endl;
        } else if ((*iter).second->identifierFlag == IDENTIFIER_LEGACY) {
            cout << "nodeId=" << nodeId << ", sourceNodeId=" << (*iter).second->sourceNodeId <<  ", " << "TargetNodeId=" << (*iter).second->targetNodeId << ", UpdatedTime=" << (*iter).second->updatedTime/ MILLI_SECOND
                << ", " << (*iter).second->xExactPosition << ", " << (*iter).second->yExactPosition << ", Type:Legacy" << ", relDist:" << relativeDist << endl;
        }
    }
}


inline
void DsrcMessageApplication::PrintOutVehiclesInformationForOutput(map<NodeIdType, shared_ptr<VehicleInformation> > table, float selfXPos, float selfYPos, int idenFlag) {
    typedef map<NodeIdType, shared_ptr<VehicleInformation> >::iterator nIterType;
    
    cout << nodeId << ", " << selfXPos << ", " << selfYPos<< ", " << 0.0 << ", " << idenFlag;
    for (nIterType iter = table.begin(); iter != table.end(); iter++) {
    
        float relativeDist = distance2Coordinate((*iter).second->xExactPosition, (*iter).second->yExactPosition, selfXPos, selfYPos);
        
        cout << ", " << (*iter).second->targetNodeId << ", " << (*iter).second->xExactPosition << ", " << (*iter).second->yExactPosition << ", "
             << (*iter).second->updatedTime/MILLI_SECOND<< ", " << relativeDist << ", " << (*iter).second->identifierFlag;
    }
    cout << endl;
}


inline
void DsrcMessageApplication::PrintOutPriority(map<NodeIdType, shared_ptr<VehicleInformation> > table, float type, float speed, float lane) {
    typedef map<NodeIdType, shared_ptr<VehicleInformation> >::iterator nIterType;
    for (nIterType iter = table.begin(); iter != table.end(); iter++) {
        cout << "nodeId=" << nodeId  << ", TargetNodeId=" << (*iter).second->targetNodeId << ", UpdatedTime=" << (*iter).second->updatedTime/ MILLI_SECOND
             << ", " << (*iter).second->xExactPosition << ", " << (*iter).second->yExactPosition << ", Priority:" << (*iter).second->priority
             << ", Type:" << type << ", relSpeed:" << speed << ", Lane:" << lane << endl;
    }
}

inline
void DsrcMessageApplication::PrintOutPacketsInformation(multimap<NodeIdType, shared_ptr<VehicleInformation> > table) {
    typedef map<NodeIdType, shared_ptr<VehicleInformation> >::iterator nIterType;
    for (nIterType iter = table.begin(); iter != table.end(); iter++) {
        if ((*iter).second->identifierFlag == IDENTIFIER_SENSOR) {
            cout << "nodeId=" << nodeId <<  ", senderNodeId=" << (*iter).second->sourceNodeId <<  ", " << "TargetNodeId=" << (*iter).second->targetNodeId << ", UpdatedTime=" << (*iter).second->updatedTime/ MILLI_SECOND
                 << ", " << (*iter).second->xExactPosition << ", " << (*iter).second->yExactPosition << ", Flag:Sensor"  << ", msgOverhead:" << (*iter).second->msgOverhead << endl;
        } else if ((*iter).second->identifierFlag == IDENTIFIER_SENSOR_V2V) {
            cout << "nodeId=" << nodeId <<  ", senderNodeId=" << (*iter).second->sourceNodeId <<  ", " << "TargetNodeId=" << (*iter).second->targetNodeId << ", UpdatedTime=" << (*iter).second->updatedTime/ MILLI_SECOND
                 << ", " << (*iter).second->xExactPosition << ", " << (*iter).second->yExactPosition << ", Flag:Sensor+V2V"  << ", msgOverhead:" << (*iter).second->msgOverhead << endl;
        } else if ((*iter).second->identifierFlag == IDENTIFIER_V2V) {
            cout << "nodeId=" << nodeId <<  ", senderNodeId=" << (*iter).second->sourceNodeId <<  ", " << "TargetNodeId=" << (*iter).second->targetNodeId << ", UpdatedTime=" << (*iter).second->updatedTime/ MILLI_SECOND
                 << ", " << (*iter).second->xExactPosition << ", " << (*iter).second->yExactPosition << ", Flag:V2V"  << ", msgOverhead:" << (*iter).second->msgOverhead << endl;
        } else if ((*iter).second->identifierFlag == IDENTIFIER_V2V_SENDER) {
            cout << "nodeId=" << nodeId <<  ", senderNodeId=" << (*iter).second->sourceNodeId <<  ", " << "TargetNodeId=" << (*iter).second->targetNodeId << ", UpdatedTime=" << (*iter).second->updatedTime/ MILLI_SECOND
                 << ", " << (*iter).second->xExactPosition << ", " << (*iter).second->yExactPosition << ", Flag:V2V(Sender)"  << ", msgOverhead:" << (*iter).second->msgOverhead << endl;
        } else if ((*iter).second->identifierFlag == IDENTIFIER_LEGACY) {
            cout << "nodeId=" << nodeId <<  ", senderNodeId=" << (*iter).second->sourceNodeId <<  ", " << "TargetNodeId=" << (*iter).second->targetNodeId << ", UpdatedTime=" << (*iter).second->updatedTime/ MILLI_SECOND
                 << ", " << (*iter).second->xExactPosition << ", " << (*iter).second->yExactPosition << ", Flag:Legacy"  << ", msgOverhead:" << (*iter).second->msgOverhead << endl;
        }
    }
}

inline
void DsrcMessageApplication::PrintOutPacketsInformationForOutput(multimap<NodeIdType, shared_ptr<VehicleInformation> > table, float selfXPos, float selfYPos, int idenFlag) {
    typedef map<NodeIdType, shared_ptr<VehicleInformation> >::iterator nIterType;
    
    cout << nodeId << ", " << selfXPos << ", " << selfYPos<< ", " << 0.0 << ", " << idenFlag;
    
    for (nIterType iter = table.begin(); iter != table.end(); iter++) {
        cout << ", "  << (*iter).second->sourceNodeId <<  ", " << (*iter).second->targetNodeId << ", " << (*iter).second->xExactPosition
             << ", " << (*iter).second->yExactPosition << ", "  << (*iter).second->identifierFlag << ", " << (*iter).second->msgOverhead;
    }
    cout << endl;
}


inline
void DsrcMessageApplication::OutputTraceAndStatsForSendBasicSafetyMessage(
    const unsigned int sequenceNumber,
    const PacketIdType& packetId,
    const size_t packetLengthBytes)
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            ApplicationSendTraceRecord traceData;

            traceData.packetSequenceNumber = sequenceNumber;
            traceData.sourceNodeId = packetId.GetSourceNodeId();
            traceData.destinationNodeId = ANY_NODEID;
            traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();

            assert(sizeof(traceData) == APPLICATION_SEND_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                basicSafetyAppModelName,
                "",
                "BsmSend",
                traceData);

        } else {

            ostringstream outStream;

            outStream << "Seq= " << sequenceNumber << " PktId= " << packetId;

            simulationEngineInterfacePtr->OutputTrace(
                basicSafetyAppModelName,
                "",
                "BsmSend",
                outStream.str());
        }//if//
    }//if//

    basicSafetyMessageInfo.packetsSentStatPtr->IncrementCounter();
    basicSafetyMessageInfo.bytesSentStatPtr->IncrementCounter(packetLengthBytes);
}//OutputTraceAndStatsForSendBasicSafetyMessage//


// トレースデータを出力
inline
void DsrcMessageApplication::OutputTraceAndStatsForReceiveBasicSafetyMessage(
    const unsigned int sequenceNumber,
    const PacketIdType& packetId,
    const size_t packetLengthBytes,
    const TimeType& delay)
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            ApplicationReceiveTraceRecord traceData;

            traceData.packetSequenceNumber = sequenceNumber;
            traceData.sourceNodeId = packetId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();
            traceData.delay = delay;
            traceData.receivedPackets = basicSafetyMessageInfo.numberPacketsReceived;
            traceData.packetLengthBytes = static_cast<uint16_t>(packetLengthBytes);

            assert(sizeof(traceData) == APPLICATION_RECEIVE_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                basicSafetyAppModelName,
                "",
                "BsmRecv",
                traceData);

        } else {
            ostringstream outStream;

            outStream << "Seq= " << sequenceNumber << " PktId= " << packetId
                      << " Delay= " << ConvertTimeToStringSecs(delay)
                      << " Pdr= " << basicSafetyMessageInfo.numberPacketsReceived << '/' << sequenceNumber
                      << " PacketBytes= " << packetLengthBytes;

            simulationEngineInterfacePtr->OutputTrace(
                basicSafetyAppModelName,
                "",
                "BsmRecv",
                outStream.str());
        }//if//
    }//if//

    basicSafetyMessageInfo.packetsReceivedStatPtr->IncrementCounter();
    basicSafetyMessageInfo.bytesReceivedStatPtr->IncrementCounter(packetLengthBytes);
    basicSafetyMessageInfo.endToEndDelayStatPtr->RecordStatValue(ConvertTimeToDoubleSecs(delay));
}//OutputTraceAndStatsForReceiveBasicSafetyMessage//


//誤差の部分をどのように考えるか
inline
bool DsrcMessageApplication::isMatchingCandidate(
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
} //isMatchingCandidate



//共通の関数として利用できるようにできない？
inline
VectorType DsrcMessageApplication::rotateXY(
        const float x,
        const float y,
        const float theta)
{
    float cs = cos(theta);
    float sn = sin(theta);
    
    return VectorType(x*cs + y*sn, y*cs - x*sn);
}

inline
int DsrcMessageApplication::crossProduct(
        const VectorType p1,
        const VectorType p2,
        const VectorType p3)
{
    float ZCoord = p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y);
    if ( ZCoord > 0 ) {return  1;} // 左
    else if ( ZCoord < 0 ) {return -1;} // 右
	else {return  0;} // 線上
}


} //namespace Wave//

#endif
