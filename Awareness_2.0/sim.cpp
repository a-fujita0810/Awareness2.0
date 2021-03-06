// Copyright (c) 2007-2016 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

// Cooperative Awareness Program ver1.0
// For MoNA 2016-01

// VISLABとうまく連携できていない：発表ではデモ動画

//shapeファイルの扱いがよくわからないので，とりあえず「対称」とする
//shapeファイルでオブジェクトの座標を設定する方法について，要調査

//storedNeighborInfoは現状上書きされるような形であるので，本来multimapを利用するはず．この場合，異なる車両が送信したターゲット車両の情報が含まれることにはなる．
//更新するとおかしくなりそうなので，このまま置いておく，後日修正/Users/Atsushi_Fujita/scenargie_simulator/2.1/source/its_modify/Awareness_1.0/Awareness_1.0/commandline/Scenarios/FreeTravel-233Veh/auto_sim_rub_12200800.rb






#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <set>
#include <random>

#include "scenargiesim.h"
#include "scensim_gui_interface.h"
#include "scensim_mobility.h"
#include "scensim_gis.h"
#include "scensim_routing_support.h"

using std::shared_ptr;

#include "dot11_mac.h"
#include "dot11_phy.h"
#include "its_app.h"
#include "wave_app.h"
#include "wave_net.h"
#include "wave_mac.h"
#include "geonet_net.h"
#include "geonet_mac.h"
#include "t109_app.h"

#include "vehicle_detection_sensor.h"


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

using ScenSim::NodeIdType;
using ScenSim::InterfaceOrInstanceIdType;
using ScenSim::ParameterDatabaseReader;
using ScenSim::SimulationEngine;
using ScenSim::SimulationEngineInterface;
using ScenSim::TimeType;

using ScenSim::NetworkSimulator;
using ScenSim::NetworkNode;
using ScenSim::InterfaceIdType;
using ScenSim::SimplePropagationModel;
using ScenSim::SimplePropagationModelForNode;
using ScenSim::SimplePropagationLossCalculationModel;
using ScenSim::RandomNumberGeneratorSeedType;
using ScenSim::AntennaModel;
using ScenSim::AttachedAntennaMobilityModel;
using ScenSim::ObjectMobilityModel;
using ScenSim::ObjectMobilityPosition;
using ScenSim::RandomNumberGeneratorSeedType;
using ScenSim::GlobalNetworkingObjectBag;
using ScenSim::GisSubsystem;
using ScenSim::PropagationInformationType;
using ScenSim::PropagationStatisticsType;
using ScenSim::MacLayer;
using ScenSim::MakeLowerCaseString;
using ScenSim::ConvertTimeToStringSecs;
using ScenSim::SquaredXYDistanceBetweenVertices;
using ScenSim::Vertex;
using ScenSim::GrabArgvForLicensing;
using ScenSim::MainFunctionArgvProcessingBasicParallelVersion1;
using ScenSim::GuiInterfacingSubsystem;
using ScenSim::MILLI_SECOND;
using ScenSim::ConvertToUShortInt;

using Dot11::Dot11Mac;
using Dot11::Dot11Phy;

using T109::T109Phy;
using T109::T109Phy;
using T109::T109App;

using Wave::WsmpLayer;
using Wave::DsrcMessageApplication;
using Wave::WaveMacPhyInput;
using Wave::ChannelNumberIndexType;
using Wave::WaveMac;
using Wave::GetChannelCategoryAndNumber;
using Wave::ChannelCategoryType;

using GeoNet::GeoNetworkingProtocol;
using GeoNet::BasicTransportProtocol;
using GeoNet::GeoNetMac;

using Its::ItsBroadcastApplication;
using ScenSim::InterfaceOutputQueue;
using ScenSim::Packet;
using ScenSim::NetworkAddress;
using ScenSim::PacketPriorityType;
using ScenSim::EtherTypeFieldType;
using ScenSim::ApplicationIdType;
using ScenSim::ConvertStringToInt;
using ScenSim::SECOND;
using ScenSim::PI;
using ScenSim::MAX_COMMUNICATION_NODEID;
using ScenSim::EnqueueResultType;
using ScenSim::ETHERTYPE_IS_NOT_SPECIFIED;
using ScenSim::nullInstanceId;
using ScenSim::NormalizeAngleToNeg180To180;

class SimNode;

namespace Dot11 {
    typedef ScenSim::SimplePropagationModel<Dot11Phy::PropFrameType> PropagationModel;
    typedef SimplePropagationModelForNode<Dot11Phy::PropFrameType> PropagationModelInterface;
};

namespace T109 {
    typedef ScenSim::SimplePropagationModel<T109Phy::PropFrameType> PropagationModel;
    typedef SimplePropagationModelForNode<T109Phy::PropFrameType> PropagationModelInterface;
};

static inline
double CalculateXySquaredDistance(const ObjectMobilityPosition& p1, const ObjectMobilityPosition& p2)
{
    return SquaredXYDistanceBetweenVertices(
        Vertex(p1.X_PositionMeters(), p1.Y_PositionMeters(), p1.HeightFromGroundMeters()),
        Vertex(p2.X_PositionMeters(), p2.Y_PositionMeters(), p2.HeightFromGroundMeters()));

}//CalculateXySquaredDistance//


class NoNetworkQueue : public InterfaceOutputQueue {
public:
    NoNetworkQueue() { }

    virtual bool InsertWithFullPacketInformationModeIsOn() const { return false; }

    virtual void Insert(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& nextHopAddress,
        const PacketPriorityType priority,
        EnqueueResultType& enqueueResult,
        unique_ptr<Packet>& packetToDropPtr,
        const EtherTypeFieldType etherType = ETHERTYPE_IS_NOT_SPECIFIED) {
        (*this).OutputUtilizationError();
    }

    virtual void InsertWithFullPacketInformation(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& nextHopAddress,
        const NetworkAddress& sourceAddress,
        const unsigned short int sourcePort,
        const NetworkAddress& destinationAddress,
        const unsigned short int destinationPort,
        const unsigned char protocolCode,
        const PacketPriorityType priority,
        const unsigned short int ipv6FlowLabel,
        EnqueueResultType& enqueueResult,
        unique_ptr<Packet>& packetToDropPtr) {
        (*this).OutputUtilizationError();
    }

    virtual bool IsEmpty() const { return true; }

    virtual bool IsFull(const PacketPriorityType priority, const size_t forPacketSizeBytes = 0) const { return false; }

    virtual void DequeuePacket(
        unique_ptr<Packet>& packetPtr,
        NetworkAddress& nextHopAddress,
        PacketPriorityType& priority,
        EtherTypeFieldType& etherType) { assert(false); }

protected:

    void OutputUtilizationError() {
        cerr << "T109Mac can't send IP packets generated by CBR, VBR, FTP,"
             << " MultiFTP, VoIP, Video, HTTP, Flooding or any general application." << endl;
        cerr << "Only T109App is available for T109Mac." << endl;
        exit(1);
    }

};//NoNetworkQueue//


//----------------------------------------------------------------------------------------

class ItsSimulator : public NetworkSimulator {
public:
    ItsSimulator(
        const shared_ptr<ParameterDatabaseReader>& initParameterDatabaseReaderPtr,
        const shared_ptr<SimulationEngine>& initSimulationEnginePtr,
        const RandomNumberGeneratorSeedType& initRunSeed,
        const bool initRunSequentially);

    ~ItsSimulator() { (*this).DeleteAllNodes(); }

    virtual void CreateNewNode(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeIdType& nodeId,
        const shared_ptr<ObjectMobilityModel>& nodeMobilityModelPtr,
        const string& nodeTypeName = "");

    virtual void DeleteNode(const NodeIdType& nodeId);
    virtual unsigned int LookupInterfaceIndex(
        const NodeIdType& nodeId, const InterfaceIdType& interfaceName) const;

    shared_ptr<Dot11::PropagationModel> GetDot11PropagationModel(const InterfaceIdType& interfaceId);

    shared_ptr<T109::PropagationModel> GetT109PropagationModel(const InterfaceIdType& interfaceId);

private:
    map<InterfaceOrInstanceIdType, shared_ptr<Dot11::PropagationModel> > dot11PropagationModelPtrs;
    map<InterfaceOrInstanceIdType, shared_ptr<T109::PropagationModel> > t109PropagationModelPtrs;

    map<NodeIdType, shared_ptr<SimNode> > simNodePtrs;

};//ItsSimulator//




class SimNode : public NetworkNode {
public:
    SimNode(
        const ParameterDatabaseReader& initParameterDatabaseReader,
        const GlobalNetworkingObjectBag& initGlobalNetworkingObjectBag,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const NodeIdType& initNodeId,
        const RandomNumberGeneratorSeedType& initRunSeed,
        const shared_ptr<GisSubsystem>& initGisSubsystemPtr,
        const shared_ptr<ObjectMobilityModel>& initNodeMobilityModelPtr,
        ItsSimulator& simulator);

    virtual ~SimNode();

    virtual const ObjectMobilityPosition GetCurrentLocation() const {
        const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();
        ObjectMobilityPosition position;
        nodeMobilityModelPtr->GetPositionForTime(currentTime, position);
        return (position);
    }//GetCurrentLocation//

    virtual void CalculatePathlossToLocation(
        const PropagationInformationType& informationType,
        const unsigned int antennaIndex,
        const double& positionXMeters,
        const double& positionYMeters,
        const double& positionZMeters,
        PropagationStatisticsType& propagationStatistics) const;

    virtual void CalculatePathlossToNode(
        const PropagationInformationType& informationType,
        const unsigned int interfaceNumber,
        const ObjectMobilityPosition& rxAntennaPosition,
        const ObjectMobilityModel::MobilityObjectIdType& rxNodeId,
        const AntennaModel& rxAntennaModel,
        PropagationStatisticsType& propagationStatistics) const;

    virtual bool HasAntenna(const InterfaceIdType& channelId) const {
        typedef  map<InterfaceIdType, unsigned int>::const_iterator IterType;
        IterType iter = interfaceNumberToChannel.find(channelId);

        if (iter == interfaceNumberToChannel.end()) {
            return false;
        }

        return (!interfaces.at((*iter).second).antennaModelPtrs.empty());
    }//HasAntenna//

    virtual shared_ptr<AntennaModel> GetAntennaModelPtr(const unsigned int interfaceIndex) const {
        const pair<unsigned int, unsigned int>& interfaceIdAndAntennaId =
            interfaceIdAndAntennaIds.at(interfaceIndex);

        return interfaces.at(interfaceIdAndAntennaId.first).antennaModelPtrs.at(interfaceIdAndAntennaId.second);
    }

    virtual ObjectMobilityPosition GetAntennaLocation(const unsigned int interfaceIndex) const {

        const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();
        const pair<unsigned int, unsigned int>& interfaceIdAndAntennaId =
            interfaceIdAndAntennaIds.at(interfaceIndex);

        ObjectMobilityPosition position;
        interfaces.at(interfaceIdAndAntennaId.first).
            antennaMobilityModelPtrs.at(interfaceIdAndAntennaId.second)->GetPositionForTime(
                currentTime, position);

        return (position);

    }//GetAntennaLocation//

    unsigned int GetAntennaIndex(const InterfaceIdType& interfaceName) {
        assert(interfaceIdToAntennaId.find(interfaceName) != interfaceIdToAntennaId.end());
        return interfaceIdToAntennaId[interfaceName];
    }

    shared_ptr<ObjectMobilityModel> GetMobilityModel() { return nodeMobilityModelPtr; }
    bool HasWsmpLayer(const unsigned int interfaceNumber) const
        { return (interfaces.at(interfaceNumber).wsmpLayerPtr != nullptr); }
    shared_ptr<WsmpLayer> GetWsmpLayer(const unsigned int interfacenumber) const
        { return interfaces.at(interfacenumber).wsmpLayerPtr; }

    bool HasGeoNetworkingProtocol(const unsigned int interfaceNumber) const
        { return (interfaces.at(interfaceNumber).geoNetworkingPtr != nullptr); }


    bool HasT109App(const unsigned int interfaceNumber) const
        { return (interfaces.at(interfaceNumber).t109AppPtr != nullptr); }

    void SetupMacAndPhyInterface(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
        const InterfaceIdType& interfaceId,
        const unsigned int interfaceNumber,
        ItsSimulator& simulator);

private:
    const shared_ptr<ObjectMobilityModel> nodeMobilityModelPtr;

    struct InterfaceType {
        shared_ptr<WsmpLayer> wsmpLayerPtr;
        shared_ptr<GeoNetworkingProtocol> geoNetworkingPtr;
        shared_ptr<BasicTransportProtocol> btpPtr;
        shared_ptr<T109App> t109AppPtr;
        shared_ptr<MacLayer> macPtr;
        vector<shared_ptr<AntennaModel> > antennaModelPtrs;
        vector<shared_ptr<ObjectMobilityModel> > antennaMobilityModelPtrs;
        map<unsigned int, shared_ptr<Dot11::PropagationModelInterface> > dot11PropagationModelInterfacePtrs;
        map<unsigned int, shared_ptr<T109::PropagationModelInterface> > t109PropagationModelInterfacePtrs;
    };

    typedef map<unsigned int, shared_ptr<Dot11::PropagationModelInterface> >::const_iterator Dot11InterfaceIter;
    typedef map<unsigned int, shared_ptr<T109::PropagationModelInterface> >::const_iterator T109InterfaceIter;

    map<InterfaceIdType, unsigned int> interfaceNumberToChannel;
    vector<InterfaceType> interfaces;

    map<InterfaceIdType, unsigned int> interfaceIdToAntennaId;
    vector<pair<unsigned int, unsigned int> > interfaceIdAndAntennaIds;

    void CreateAntennaModelAndPropagationInterface(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
        const InterfaceIdType& interfaceId,
        const unsigned int interfaceNumber,
        shared_ptr<AntennaModel>& antennaModelPtr,
        shared_ptr<ObjectMobilityModel>& antennaMobilityModelPtr);

    void CreateAntennaModelAndDot11PropagationInterface(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
        const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
        const shared_ptr<Dot11::PropagationModel>& propagationModelPtr,
        const InterfaceIdType& interfaceId,
        const unsigned int interfaceNumber,
        shared_ptr<AntennaModel>& antennaModelPtr,
        shared_ptr<ObjectMobilityModel>& antennaMobilityModelPtr,
        shared_ptr<Dot11::PropagationModelInterface>& propagationInterfacePtr);

    void CreateAntennaModelAndT109PropagationInterface(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
        const shared_ptr<T109::PropagationModel>& propagationModelPtr,
        const InterfaceIdType& interfaceId,
        const unsigned int interfaceNumber,
        shared_ptr<AntennaModel>& antennaModelPtr,
        shared_ptr<ObjectMobilityModel>& antennaMobilityModelPtr,
        shared_ptr<T109::PropagationModelInterface>& propagationInterfacePtr);

};//SimNode//



inline
SimNode::SimNode(
    const ParameterDatabaseReader& initParameterDatabaseReader,
    const GlobalNetworkingObjectBag& initGlobalNetworkingObjectBag,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const NodeIdType& initNodeId,
    const RandomNumberGeneratorSeedType& initRunSeed,
    const shared_ptr<GisSubsystem>& initGisSubsystemPtr,
    const shared_ptr<ObjectMobilityModel>& initNodeMobilityModelPtr,
    ItsSimulator& simulator)
    :
    NetworkNode(
        initParameterDatabaseReader,
        initGlobalNetworkingObjectBag,
        initSimulationEngineInterfacePtr,
        initNodeMobilityModelPtr,
        initNodeId,
        initRunSeed),
    nodeMobilityModelPtr(initNodeMobilityModelPtr)
{
    const unsigned int numberInterfaces = networkLayerPtr->NumberOfInterfaces();

    for(unsigned int interfaceIndex = 0; interfaceIndex < numberInterfaces; interfaceIndex++) {

        const InterfaceIdType interfaceId = networkLayerPtr->GetInterfaceId(interfaceIndex);

        (*this).SetupMacAndPhyInterface(
            initParameterDatabaseReader,
            initGlobalNetworkingObjectBag,
            interfaceId,
            interfaceIndex,
            simulator);

        if ((*this).HasWsmpLayer(interfaceIndex)) {
            //WAVE

            if (initParameterDatabaseReader.ParameterExists("its-bsm-app-traffic-start-time", nodeId)) {

                (*this).GetAppLayerPtr()->AddApp(
                    shared_ptr<DsrcMessageApplication>(
                        new DsrcMessageApplication(
                            initParameterDatabaseReader,
                            simulationEngineInterfacePtr,
                            interfaces.at(interfaceIndex).wsmpLayerPtr,
                            nodeId,
                            nodeSeed,
                            nodeMobilityModelPtr)));
            }

        }
        else if ((*this).HasGeoNetworkingProtocol(interfaceIndex)) {
            //GeoNet

            if (initParameterDatabaseReader.ParameterExists("its-broadcast-app-traffic-start-time", nodeId)) {

                shared_ptr<ItsBroadcastApplication> appPtr =
                    shared_ptr<ItsBroadcastApplication>(
                        new ItsBroadcastApplication(
                            initParameterDatabaseReader,
                            simulationEngineInterfacePtr,
                            nodeId,
                            interfaceId,
                            ItsBroadcastApplication::BTP_MODE,
                            interfaces.at(interfaceIndex).btpPtr));

                (*this).GetAppLayerPtr()->AddApp(appPtr);
                appPtr->CompleteInitialization(initParameterDatabaseReader);
            }

        }
        else if ((*this).HasT109App(interfaceIndex)) {

            //do nothing
        }
        else {
            //UDP/IP

            //do nothing

        }//if//

    }//for//

    // Add interface routing protocols from here. -------------------

    SetupRoutingProtocol(
        initParameterDatabaseReader,
        nodeId,
        simulationEngineInterfacePtr,
        networkLayerPtr,
        nodeSeed);

    // Add (routing)/transport/application protocols from here. -----

    // --------------------------------------------------------------

    initGisSubsystemPtr->EnableMovingObjectIfNecessary(
        initParameterDatabaseReader,
        nodeId,
        simulationEngineInterfacePtr->CurrentTime(),
        nodeMobilityModelPtr);

}//SimNode()//

void SimNode::CreateAntennaModelAndPropagationInterface(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
    const InterfaceIdType& interfaceId,
    const unsigned int interfaceNumber,
    shared_ptr<AntennaModel>& antennaModelPtr,
    shared_ptr<ObjectMobilityModel>& antennaMobilityModelPtr)
{
    antennaModelPtr = CreateAntennaModel(
        theParameterDatabaseReader,
        nodeId,
        interfaceId,
        *theGlobalNetworkingObjectBag.antennaPatternDatabasePtr);

    // Antenna Mobility
    if (AttachedAntennaMobilityModel::AntennaIsAttachedAntenna(
            theParameterDatabaseReader, nodeId, interfaceId)) {

        antennaMobilityModelPtr.reset(
            new AttachedAntennaMobilityModel(
                theParameterDatabaseReader,
                nodeId,
                interfaceId,
                nodeMobilityModelPtr));
    }
    else {
        antennaMobilityModelPtr = nodeMobilityModelPtr;
    }//if//

    InterfaceType& targetInterface = interfaces.at(interfaceNumber);

    interfaceIdToAntennaId[interfaceId] = static_cast<unsigned int>(interfaceIdAndAntennaIds.size());
    interfaceIdAndAntennaIds.push_back(
        make_pair(interfaceNumber, static_cast<unsigned int>(targetInterface.antennaModelPtrs.size())));

    targetInterface.antennaModelPtrs.push_back(antennaModelPtr);
    targetInterface.antennaMobilityModelPtrs.push_back(antennaMobilityModelPtr);
}



SimNode::~SimNode()
{
    for(unsigned int i = 0; (i < interfaces.size()); i++) {
        InterfaceType& anInterface = interfaces[i];
        if (anInterface.geoNetworkingPtr != nullptr) {
            anInterface.geoNetworkingPtr->DisconnectFromOtherLayers();
        }//if//
    }//for//
}


void SimNode::CreateAntennaModelAndDot11PropagationInterface(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& siulationEngineInterfacePtr,
    const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
    const shared_ptr<Dot11::PropagationModel>& propagationModelPtr,
    const InterfaceIdType& interfaceId,
    const unsigned int interfaceNumber,
    shared_ptr<AntennaModel>& antennaModelPtr,
    shared_ptr<ObjectMobilityModel>& antennaMobilityModelPtr,
    shared_ptr<Dot11::PropagationModelInterface>& propagationInterfacePtr)
{
    (*this).CreateAntennaModelAndPropagationInterface(
        theParameterDatabaseReader,
        theGlobalNetworkingObjectBag,
        interfaceId,
        interfaceNumber,
        antennaModelPtr,
        antennaMobilityModelPtr);

    InterfaceType& targetInterface = interfaces.at(interfaceNumber);
    size_t nextPropagationIndex = 0;

    for(size_t i = 0; i < interfaces.size(); i++) {
        nextPropagationIndex += interfaces[i].dot11PropagationModelInterfacePtrs.size();
    }

    propagationInterfacePtr = propagationModelPtr->GetNewPropagationModelInterface(
        simulationEngineInterfacePtr,
        antennaModelPtr,
        antennaMobilityModelPtr,
        nodeId,
        interfaceId,
        static_cast<unsigned int>(nextPropagationIndex));

    targetInterface.dot11PropagationModelInterfacePtrs.insert(
        make_pair(static_cast<unsigned int>(targetInterface.antennaModelPtrs.size() - 1), propagationInterfacePtr));
}

void SimNode::CreateAntennaModelAndT109PropagationInterface(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
    const shared_ptr<T109::PropagationModel>& propagationModelPtr,
    const InterfaceIdType& interfaceId,
    const unsigned int interfaceNumber,
    shared_ptr<AntennaModel>& antennaModelPtr,
    shared_ptr<ObjectMobilityModel>& antennaMobilityModelPtr,
    shared_ptr<T109::PropagationModelInterface>& propagationInterfacePtr)
{
    (*this).CreateAntennaModelAndPropagationInterface(
        theParameterDatabaseReader,
        theGlobalNetworkingObjectBag,
        interfaceId,
        interfaceNumber,
        antennaModelPtr,
        antennaMobilityModelPtr);

    InterfaceType& targetInterface = interfaces.at(interfaceNumber);

    propagationInterfacePtr = propagationModelPtr->GetNewPropagationModelInterface(
        simulationEngineInterfacePtr,
        antennaModelPtr,
        antennaMobilityModelPtr,
        nodeId,
        interfaceId,
        interfaceNumber);

    targetInterface.t109PropagationModelInterfacePtrs.insert(
        make_pair(static_cast<unsigned int>(targetInterface.antennaModelPtrs.size() - 1), propagationInterfacePtr));
}

void SimNode::SetupMacAndPhyInterface(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
    const InterfaceIdType& interfaceId,
    const unsigned int interfaceNumber,
    ItsSimulator& simulator)
{
    interfaceNumberToChannel[interfaceId] = interfaceNumber;

    while (interfaces.size() <= interfaceNumber) {
        interfaces.push_back(InterfaceType());
    }//while//

    if (InterfaceIsWired(theParameterDatabaseReader, nodeId, interfaceId) ||
        IsNoInterface(theParameterDatabaseReader, nodeId, interfaceId)) {
        // no mac protocol
        // Abstract Network has virtual connections between their network layers.
        return;
    }//if//

    InterfaceType& targetInterface = interfaces.at(interfaceNumber);

    // MAC protocol
    const string macProtocol = MakeLowerCaseString(
        theParameterDatabaseReader.ReadString("mac-protocol", nodeId, interfaceId));

    if (macProtocol == "dot11") {
        const InterfaceOrInstanceIdType channelInstanceId =
            GetChannelInstanceId(theParameterDatabaseReader, nodeId, interfaceId);

        shared_ptr<Dot11::PropagationModel> propagationModelPtr =
            simulator.GetDot11PropagationModel(channelInstanceId);

        shared_ptr<AntennaModel> antennaModelPtr;
        shared_ptr<ObjectMobilityModel> antennaMobilityModelPtr;
        shared_ptr<Dot11::PropagationModelInterface> propagationInterfacePtr;

        (*this).CreateAntennaModelAndDot11PropagationInterface(
            theParameterDatabaseReader,
            simulationEngineInterfacePtr,
            theGlobalNetworkingObjectBag,
            propagationModelPtr,
            interfaceId,
            interfaceNumber,
            antennaModelPtr,
            antennaMobilityModelPtr,
            propagationInterfacePtr);

        shared_ptr<Dot11Phy> phyLayerPtr(
            new Dot11Phy(
                theParameterDatabaseReader,
                nodeId,
                interfaceId,
                simulationEngineInterfacePtr,
                propagationInterfacePtr,
                theGlobalNetworkingObjectBag.bitOrBlockErrorRateCurveDatabasePtr,
                nodeSeed));

        targetInterface.macPtr =
            Dot11Mac::Create(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                nodeId,
                interfaceId,
                interfaceNumber,
                networkLayerPtr,
                phyLayerPtr,
                nodeSeed);
    }
    else if (macProtocol == "wave") {
        const string phyDvicesNames = theParameterDatabaseReader.ReadString(
            "its-wave-device-names", nodeId, interfaceId);

        istringstream phyDeviceStream(phyDvicesNames);
        vector<WaveMacPhyInput> phyInputs;

        while (!phyDeviceStream.eof()) {

            phyInputs.push_back(WaveMacPhyInput());
            WaveMacPhyInput& phyInput = phyInputs.back();

            phyDeviceStream >> phyInput.phyDeviceName;

            const InterfaceOrInstanceIdType channelInstanceId =
                GetChannelInstanceId(theParameterDatabaseReader, nodeId, phyInput.phyDeviceName);

            shared_ptr<Dot11::PropagationModel> propagationModelPtr =
                simulator.GetDot11PropagationModel(channelInstanceId);

            shared_ptr<AntennaModel> antennaModelPtr;
            shared_ptr<ObjectMobilityModel> antennaMobilityModelPtr;
            shared_ptr<Dot11::PropagationModelInterface> propagationInterfacePtr;

            (*this).CreateAntennaModelAndDot11PropagationInterface(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                theGlobalNetworkingObjectBag,
                propagationModelPtr,
                phyInput.phyDeviceName,
                interfaceNumber,
                antennaModelPtr,
                antennaMobilityModelPtr,
                propagationInterfacePtr);

            phyInput.phyPtr.reset(
                new Dot11Phy(
                    theParameterDatabaseReader,
                    nodeId,
                    phyInput.phyDeviceName,
                    simulationEngineInterfacePtr,
                    propagationInterfacePtr,
                    theGlobalNetworkingObjectBag.bitOrBlockErrorRateCurveDatabasePtr,
                    nodeSeed));

            const string channelNumbersString =
                theParameterDatabaseReader.ReadString(
                    "its-wave-phy-support-channels", nodeId, phyInput.phyDeviceName);

            istringstream channelNumberStream(channelNumbersString);

            ChannelNumberIndexType maxChannelNumberId = 0;
            set<ChannelCategoryType> specifiedCategoryTypes;

            while (!channelNumberStream.eof()) {
                string channelNumberString;
                channelNumberStream >> channelNumberString;

                ChannelNumberIndexType channelNumberId;
                ChannelCategoryType channelCategory;

                GetChannelCategoryAndNumber(channelNumberString, channelCategory, channelNumberId);

                if (specifiedCategoryTypes.find(channelCategory) != specifiedCategoryTypes.end()) {
                    cerr << "Error: Duplicated channel category \""
                         << Wave::CHANNEL_CATEGORY_NAMES[channelCategory]
                         << "\" specified in " << channelNumbersString << endl;
                    exit(1);
                }//if//

                specifiedCategoryTypes.insert(channelCategory);

                phyInput.channelNumberIds[channelCategory].push_back(channelNumberId);

                maxChannelNumberId = std::max(maxChannelNumberId, channelNumberId);
            }//while//

            if (maxChannelNumberId >= propagationInterfacePtr->GetChannelCount()) {
                cerr << "Error: WAVE device will use channel "
                     << int(maxChannelNumberId) << ". Set channel-count to "
                     << int(maxChannelNumberId)+1 << " or more." << endl;
                exit(1);
            }//if//

        }//while//

        shared_ptr<WaveMac> waveMacPtr(
            new WaveMac(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                nodeId,
                interfaceId,
                interfaceNumber,
                networkLayerPtr,
                phyInputs,
                nodeSeed));

        targetInterface.wsmpLayerPtr.reset(
            new WsmpLayer(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                nodeId,
                interfaceId,
                (*this).GetNodeSeed(),
                nodeMobilityModelPtr,
                networkLayerPtr,
                waveMacPtr));

        targetInterface.macPtr = waveMacPtr;

    }
    else if (macProtocol.find("geonet") != string::npos) {
        const InterfaceOrInstanceIdType channelInstanceId =
            GetChannelInstanceId(theParameterDatabaseReader, nodeId, interfaceId);

        shared_ptr<Dot11::PropagationModel> propagationModelPtr =
            simulator.GetDot11PropagationModel(channelInstanceId);

        shared_ptr<AntennaModel> antennaModelPtr;
        shared_ptr<ObjectMobilityModel> antennaMobilityModelPtr;
        shared_ptr<Dot11::PropagationModelInterface> propagationInterfacePtr;

        (*this).CreateAntennaModelAndDot11PropagationInterface(
            theParameterDatabaseReader,
            simulationEngineInterfacePtr,
            theGlobalNetworkingObjectBag,
            propagationModelPtr,
            interfaceId,
            interfaceNumber,
            antennaModelPtr,
            antennaMobilityModelPtr,
            propagationInterfacePtr);

        shared_ptr<Dot11Phy> dot11PhyLayerPtr(
            new Dot11Phy(
                theParameterDatabaseReader,
                nodeId,
                interfaceId,
                simulationEngineInterfacePtr,
                propagationInterfacePtr,
                theGlobalNetworkingObjectBag.bitOrBlockErrorRateCurveDatabasePtr,
                nodeSeed));

        shared_ptr<GeoNetMac> geoNetMacLayerPtr(
            new GeoNetMac(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                nodeId,
                interfaceId,
                interfaceNumber,
                networkLayerPtr,
                dot11PhyLayerPtr,
                nodeSeed));

        targetInterface.macPtr = geoNetMacLayerPtr;

        targetInterface.geoNetworkingPtr.reset(
            new GeoNetworkingProtocol(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                nodeId,
                interfaceId,
                interfaceNumber,
                nodeMobilityModelPtr,
                geoNetMacLayerPtr,
                networkLayerPtr));

        targetInterface.btpPtr.reset(
            new BasicTransportProtocol(
                simulationEngineInterfacePtr));

        targetInterface.btpPtr->ConnectToNetworkLayer(targetInterface.geoNetworkingPtr);

    }
    else if (macProtocol.find("t109") != string::npos) {
        const InterfaceOrInstanceIdType channelInstanceId =
            GetChannelInstanceId(theParameterDatabaseReader, nodeId, interfaceId);

        shared_ptr<T109::PropagationModel> propagationModelPtr =
            simulator.GetT109PropagationModel(channelInstanceId);

        shared_ptr<AntennaModel> antennaModelPtr;
        shared_ptr<ObjectMobilityModel> antennaMobilityModelPtr;
        shared_ptr<T109::PropagationModelInterface> propagationInterfacePtr;

        (*this).CreateAntennaModelAndT109PropagationInterface(
            theParameterDatabaseReader,
            theGlobalNetworkingObjectBag,
            propagationModelPtr,
            interfaceId,
            interfaceNumber,
            antennaModelPtr,
            antennaMobilityModelPtr,
            propagationInterfacePtr);

        targetInterface.t109AppPtr.reset(
            new T109App(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                propagationInterfacePtr,
                theGlobalNetworkingObjectBag.bitOrBlockErrorRateCurveDatabasePtr,
                nodeId,
                interfaceId,
                interfaceNumber,
                (*this).GetNodeSeed(),
                nodeMobilityModelPtr));

        targetInterface.macPtr = targetInterface.t109AppPtr->GetMacPtr();

        networkLayerPtr->SetInterfaceOutputQueue(
            interfaceNumber,
            shared_ptr<InterfaceOutputQueue>(new NoNetworkQueue()));
    }
    else {
        cerr << "Error: bad mac-protocol format: " << macProtocol
             << " for node: " << nodeId << endl
             << "only \"wave\" , \"t109\" , \"geonet\" and \"dot11\" are available." << endl;
        exit(1);
    }//if//

    networkLayerPtr->SetInterfaceMacLayer(interfaceNumber, targetInterface.macPtr);

}//SetupMacAndPhyInterface//


void SimNode::CalculatePathlossToLocation(
    const PropagationInformationType& informationType,
    const unsigned int antennaIndex,
    const double& positionXMeters,
    const double& positionYMeters,
    const double& positionZMeters,
    PropagationStatisticsType& propagationStatistics) const
{
    const pair<unsigned int, unsigned int>& interfaceIdAndAntennaId = interfaceIdAndAntennaIds.at(antennaIndex);

    const InterfaceType& targetInterface = interfaces.at(interfaceIdAndAntennaId.first);

    const map<unsigned int, shared_ptr<Dot11::PropagationModelInterface> >& dot11PropagationModelInterfacePtrs =
        targetInterface.dot11PropagationModelInterfacePtrs;
    const map<unsigned int, shared_ptr<T109::PropagationModelInterface> >& t109PropagationModelInterfacePtrs =
        targetInterface.t109PropagationModelInterfacePtrs;

    Dot11InterfaceIter dot11Iter = dot11PropagationModelInterfacePtrs.find(interfaceIdAndAntennaId.second);
    T109InterfaceIter t109Iter = t109PropagationModelInterfacePtrs.find(interfaceIdAndAntennaId.second);

    if (dot11Iter != dot11PropagationModelInterfacePtrs.end()) {
        (*dot11Iter).second->CalculatePathlossToLocation(
                informationType,
                positionXMeters,
                positionYMeters,
                positionZMeters,
                propagationStatistics);
    }
    else {
        assert(t109Iter != t109PropagationModelInterfacePtrs.end());
        (*t109Iter).second->CalculatePathlossToLocation(
                informationType,
                positionXMeters,
                positionYMeters,
                positionZMeters,
                propagationStatistics);
    }//if//

}//CalculatePathlossToLocation//

void SimNode::CalculatePathlossToNode(
    const PropagationInformationType& informationType,
    const unsigned int interfaceNumber,
    const ObjectMobilityPosition& rxAntennaPosition,
    const ObjectMobilityModel::MobilityObjectIdType& rxNodeId,
    const AntennaModel& rxAntennaModel,
    PropagationStatisticsType& propagationStatistics) const
{
    const pair<unsigned int, unsigned int>& interfaceIdAndAntennaId =
        interfaceIdAndAntennaIds.at(interfaceNumber);

    const InterfaceType& targetInterface = interfaces.at(interfaceIdAndAntennaId.first);

    const TimeType currentTime =
        simulationEngineInterfacePtr->CurrentTime();

    const map<unsigned int, shared_ptr<Dot11::PropagationModelInterface> >& dot11PropagationModelInterfacePtrs =
        targetInterface.dot11PropagationModelInterfacePtrs;
    const map<unsigned int, shared_ptr<T109::PropagationModelInterface> >& t109PropagationModelInterfacePtrs =
        targetInterface.t109PropagationModelInterfacePtrs;

    Dot11InterfaceIter dot11Iter = dot11PropagationModelInterfacePtrs.find(interfaceIdAndAntennaId.second);
    T109InterfaceIter t109Iter = t109PropagationModelInterfacePtrs.find(interfaceIdAndAntennaId.second);

    if (dot11Iter != dot11PropagationModelInterfacePtrs.end()) {
        const shared_ptr<Dot11::PropagationModelInterface> propagationInterfacePtr = (*dot11Iter).second;

        const shared_ptr<Dot11::PropagationModel> propagationModelPtr =
            propagationInterfacePtr->GetPropagationModel();

        const vector<unsigned int>& channelNumberSet = propagationInterfacePtr->GetCurrentChannelNumberSet();

        unsigned int channelNumber;

        if (channelNumberSet.empty()) {
            channelNumber = propagationInterfacePtr->GetCurrentChannelNumber();
        }
        else {
            channelNumber = channelNumberSet.front();
        }//if//

        propagationModelPtr->CalculatePathlossToNode(
            informationType,
            propagationInterfacePtr->GetCurrentMobilityPosition(),
            nodeId,
            *(*this).GetAntennaModelPtr(interfaceNumber),
            rxAntennaPosition,
            rxNodeId,
            rxAntennaModel,
            channelNumber,
            propagationStatistics);
    }
    else {
        assert(t109Iter != t109PropagationModelInterfacePtrs.end());
        const shared_ptr<T109::PropagationModelInterface> propagationInterfacePtr = (*t109Iter).second;

        const shared_ptr<T109::PropagationModel> propagationModelPtr =
            propagationInterfacePtr->GetPropagationModel();

        const vector<unsigned int>& channelNumberSet = propagationInterfacePtr->GetCurrentChannelNumberSet();

        unsigned int channelNumber;

        if (channelNumberSet.empty()) {
            channelNumber = propagationInterfacePtr->GetCurrentChannelNumber();
        }
        else {
            channelNumber = channelNumberSet.front();
        }//if//

        propagationModelPtr->CalculatePathlossToNode(
            informationType,
            propagationInterfacePtr->GetCurrentMobilityPosition(),
            nodeId,
            *(*this).GetAntennaModelPtr(interfaceNumber),
            rxAntennaPosition,
            rxNodeId,
            rxAntennaModel,
            channelNumber,
            propagationStatistics);
    }//if//

}//CalculatePathlossToNode//


//----------------------------------------------------------------------------------------

inline
ItsSimulator::ItsSimulator(
    const shared_ptr<ParameterDatabaseReader>& initParameterDatabaseReaderPtr,
    const shared_ptr<SimulationEngine>& initSimulationEnginePtr,
    const RandomNumberGeneratorSeedType& initRunSeed,
    const bool initRunSequentially)
    :
    NetworkSimulator(
        initParameterDatabaseReaderPtr,
        initSimulationEnginePtr,
        initRunSeed,
        initRunSequentially)
{
    const ParameterDatabaseReader& theParameterDatabaseReader = (*initParameterDatabaseReaderPtr);

    const string berCurveFileName =
        theParameterDatabaseReader.ReadString("dot11-bit-error-rate-curve-file");

    (*this).theGlobalNetworkingObjectBag.bitOrBlockErrorRateCurveDatabasePtr->LoadBerCurveFile(
        berCurveFileName);

    (*this).CompleteSimulatorConstruction();

}//ItsSimulator//


shared_ptr<Dot11::PropagationModel> ItsSimulator::GetDot11PropagationModel(const InterfaceIdType& channelModel)
{
    const ParameterDatabaseReader& theParameterDatabaseReader = (*theParameterDatabaseReaderPtr);

    if (dot11PropagationModelPtrs.find(channelModel) == dot11PropagationModelPtrs.end()) {

        shared_ptr<Dot11::PropagationModel> propagationModelPtr(
            new Dot11::PropagationModel(
                (*theParameterDatabaseReaderPtr),
                runSeed,
                theSimulationEnginePtr,
                theGisSubsystemPtr,
                channelModel));

        dot11PropagationModelPtrs[channelModel] = propagationModelPtr;

        (*this).AddPropagationCalculationTraceIfNecessary(
            channelModel,
            propagationModelPtr->GetPropagationCalculationModel());
    }//if//

    return dot11PropagationModelPtrs[channelModel];
}//GetDot11PropagationModel//

shared_ptr<T109::PropagationModel> ItsSimulator::GetT109PropagationModel(const InterfaceIdType& channelModel)
{
    const ParameterDatabaseReader& theParameterDatabaseReader = (*theParameterDatabaseReaderPtr);

    if (t109PropagationModelPtrs.find(channelModel) == t109PropagationModelPtrs.end()) {

        shared_ptr<T109::PropagationModel> propagationModelPtr(
            new T109::PropagationModel(
                (*theParameterDatabaseReaderPtr),
                runSeed,
                theSimulationEnginePtr,
                theGisSubsystemPtr,
                channelModel));

        t109PropagationModelPtrs[channelModel] = propagationModelPtr;

        (*this).AddPropagationCalculationTraceIfNecessary(
            channelModel,
            propagationModelPtr->GetPropagationCalculationModel());
    }//if//

    return t109PropagationModelPtrs[channelModel];

}//GetT109PropagationModel//

void ItsSimulator::CreateNewNode(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeIdType& nodeId,
    const shared_ptr<ObjectMobilityModel>& nodeMobilityModelPtr,
    const string& nodeTypeName)
{
    unsigned int partitionIndex = 0;

    if (theParameterDatabaseReader.ParameterExists("parallelization-partition-index", nodeId)) {
        partitionIndex =
            (theParameterDatabaseReader.ReadNonNegativeInt("parallelization-partition-index", nodeId) %
             theSimulationEnginePtr->GetNumberPartitionThreads());
    }//if//

    const shared_ptr<SimulationEngineInterface> simulationEngineInterfacePtr =
        theSimulationEnginePtr->GetSimulationEngineInterface(
            theParameterDatabaseReader, nodeId, partitionIndex);

    shared_ptr<SimNode> aSimNodePtr(
        new SimNode(
            theParameterDatabaseReader,
            theGlobalNetworkingObjectBag,
            simulationEngineInterfacePtr,
            nodeId,
            runSeed,
            theGisSubsystemPtr,
            nodeMobilityModelPtr,
            *this));

    (*this).AddNode(aSimNodePtr);

    simNodePtrs[nodeId] = aSimNodePtr;
}//CreateNewNode//


void ItsSimulator::DeleteNode(const NodeIdType& nodeId)
{
    NetworkSimulator::DeleteNode(nodeId);
    
    simNodePtrs.erase(nodeId);

}//DeleteNode//

unsigned int ItsSimulator::LookupInterfaceIndex(
    const NodeIdType& nodeId,
    const InterfaceIdType& interfaceName) const
{
    typedef map<NodeIdType, shared_ptr<SimNode> > ::const_iterator IterType;

    IterType iter = simNodePtrs.find(nodeId);

    if (iter != simNodePtrs.end()) {
        return (*iter).second->GetAntennaIndex(interfaceName);
    }//if//

    assert(false);
    return 0;
}



//
//-------------------- main --------------------
//

int main(int argc, char* argv[])
{
    GrabArgvForLicensing(argc, argv);
    
    string configFileName;
    bool isControlledByGui;
    unsigned int numberParallelThreads;
    bool runSequentially;
    bool seedIsSet;
    RandomNumberGeneratorSeedType runSeed;

    MainFunctionArgvProcessingBasicParallelVersion1(
        argc,
        argv,
        configFileName,
        isControlledByGui,
        numberParallelThreads,
        runSequentially,
        seedIsSet,
        runSeed);

    shared_ptr<ParameterDatabaseReader> theParameterDatabaseReaderPtr(
        new ParameterDatabaseReader(configFileName));
    ParameterDatabaseReader& theParameterDatabaseReader =
        *theParameterDatabaseReaderPtr;

    if (!seedIsSet) {
        if (!theParameterDatabaseReader.ParameterExists("seed")) {
            cerr << "Error: No seed parameter found." << endl;
            exit(1);
        }//if//
        runSeed = theParameterDatabaseReader.ReadInt("seed");
    }//if//

    shared_ptr<SimulationEngine> theSimulationEnginePtr(
        new SimulationEngine(
            theParameterDatabaseReader,
            runSequentially,
            numberParallelThreads));

    ItsSimulator theNetworkSimulator(
        theParameterDatabaseReaderPtr,
        theSimulationEnginePtr,
        runSeed,
        runSequentially);


    if (isControlledByGui) {

        GuiInterfacingSubsystem guiInterface(
            theSimulationEnginePtr,
            ConvertToUShortInt(
                theParameterDatabaseReader.ReadNonNegativeInt("gui-portnumber-sim"),
                "Error in parameter gui-portnumber-sim: port number is too large."),
            ConvertToUShortInt(
                theParameterDatabaseReader.ReadNonNegativeInt("gui-portnumber-pausecommand"),
                "Error in parameter gui-portnumber-pausecommand: port number is too large."));

        while (true) {
            TimeType simulateUpToTime;

            guiInterface.GetAndExecuteGuiCommands(
                theParameterDatabaseReader,
                theNetworkSimulator,
                simulateUpToTime);

            if (theSimulationEnginePtr->SimulationIsDone()) {
                break;
            }//if//

            theNetworkSimulator.RunSimulationUntil(simulateUpToTime);

        }//while//

    }
    else {
        const TimeType endSimTime = theParameterDatabaseReader.ReadTime("simulation-time");

        theNetworkSimulator.RunSimulationUntil(endSimTime);

    }//if//

    return 0;

}//main//
