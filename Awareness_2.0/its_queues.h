// Copyright (c) 2007-2016 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef ITS_QUEUES_H
#define ITS_QUEUES_H

#include <queue>
#include "scensim_parmio.h"
#include "scensim_engine.h"
#include "scensim_netaddress.h"
#include "scensim_packet.h"
#include "scensim_queues.h"
#include "dot11_common.h"

namespace Dot11 {

using std::vector;
using std::shared_ptr;
using std::unique_ptr;
using std::move;

using ScenSim::ParameterDatabaseReader;
using ScenSim::InterfaceOutputQueue;
using ScenSim::Packet;
using ScenSim::PacketPriorityType;
using ScenSim::NetworkAddress;
using ScenSim::InterfaceIdType;
using ScenSim::SimulationEngineInterface;
using ScenSim::NodeIdType;
using ScenSim::EtherTypeFieldType;
using ScenSim::ETHERTYPE_IS_NOT_SPECIFIED;
using ScenSim::TimeType;
using ScenSim::EnqueueResultType;
using ScenSim::ENQUEUE_SUCCESS;
using ScenSim::ENQUEUE_FAILURE_BY_MAX_PACKETS;
using ScenSim::ENQUEUE_FAILURE_BY_MAX_BYTES;

//--------------------------------------------------------------------------------------------------

class ItsOutputQueueWithPrioritySubqueues: public InterfaceOutputQueue {
public:
    ItsOutputQueueWithPrioritySubqueues(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const InterfaceIdType& interfaceId,
        const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
        const PacketPriorityType& initMaximumPriority);

    void Insert(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& nextHopAddress,
        const PacketPriorityType priority,
        EnqueueResultType& enqueueResult,
        unique_ptr<Packet>& packetToDrop,
        const EtherTypeFieldType etherType = ETHERTYPE_IS_NOT_SPECIFIED);

    void InsertWithEtherTypeAndDatarateAndTxPower(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& nextHopAddress,
        const PacketPriorityType priority,
        const EtherTypeFieldType& etherType,
        const DatarateBitsPerSecType& datarateBitsPerSec,
        const double txPowerDbm,
        EnqueueResultType& enqueueResult,
        unique_ptr<Packet>& packetToDrop);

    PacketPriorityType MaxPossiblePacketPriority() const { return (maximumPriority); }

    bool IsEmpty() const { return (totalPackets == 0); }

    bool HasPacketWithPriority(const PacketPriorityType priority) const
    {
        assert(priority <= maximumPriority);
        return (!outputSubqueues.at(priority).aQueue.empty());
    }
    bool TopPacketDatarateIsSpecified(const PacketPriorityType priority) const;

    const Packet& TopPacket(const PacketPriorityType priority) const;
    const NetworkAddress NextHopForTopPacket(const PacketPriorityType priority) const;
    DatarateBitsPerSecType DatarateBitsPerSecForTopPacket(const PacketPriorityType priority) const;

    void DequeuePacket(
        unique_ptr<Packet>& packetPtr,
        NetworkAddress& nextHopAddress,
        PacketPriorityType& priority,
        EtherTypeFieldType& etherType);

    void DequeuePacketWithPriority(
        const PacketPriorityType& priority,
        unique_ptr<Packet>& packetPtr,
        NetworkAddress& nextHopAddress,
        EtherTypeFieldType& etherType,
        TimeType& timestamp,
        unsigned int& retryTxCount);

    void DequeuePacketWithEtherTypeAndDatarateAndTxPower(
        const PacketPriorityType& priority,
        unique_ptr<Packet>& packetPtr,
        NetworkAddress& nextHopAddress,
        TimeType& timestamp,
        unsigned int& retryTxCount,
        EtherTypeFieldType& etherType,
        bool& datarateAndTxPowerAreaSpecified,
        DatarateBitsPerSecType& datarateBitsPerSec,
        double& txPowerDbm);


    // For reinsertion for power management.

    virtual void RequeueAtFront(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& nextHopAddress,
        const PacketPriorityType priority,
        const EtherTypeFieldType etherType,
        const TimeType& timestamp,
        const unsigned int retryTxCount)
    {
        assert(false && "ITS/Wave should not need power save mode"); abort();
    }

    void RequeueAtFront(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& nextHopAddress,
        const PacketPriorityType priority,
        const EtherTypeFieldType etherType,
        const TimeType& timestamp,
        const unsigned int retryTxCount,
        const bool datarateAndTxPowerAreaSpecified,
        const DatarateBitsPerSecType datarateBitsPerSec,
        const double txPowerDbm);

private:

    shared_ptr<SimulationEngineInterface> simEngineInterfacePtr;

    struct OutputQueueRecord {
        unique_ptr<Packet> packetPtr;
        NetworkAddress nextHopAddress;
        EtherTypeFieldType etherType;
        TimeType timestamp;
        bool datarateAndTxPowerAreaSpecified;
        DatarateBitsPerSecType datarateBitsPerSec;
        double txPowerDbm;
        unsigned int retryTxCount;

        OutputQueueRecord(
            unique_ptr<Packet>& initPacketPtr,
            const NetworkAddress& initNextHopAddress,
            const EtherTypeFieldType& initEtherType,
            const TimeType& initTimestamp,
            const bool initDatarateAndTxPowerAreaSpecified,
            const DatarateBitsPerSecType& initDatarateBitsPerSec,
            const double& initTxPowerDbm,
            const unsigned int initRetryTxCount = 0)
            :
            packetPtr(move(initPacketPtr)),
            nextHopAddress(initNextHopAddress),
            etherType(initEtherType),
            timestamp(initTimestamp),
            datarateAndTxPowerAreaSpecified(initDatarateAndTxPowerAreaSpecified),
            datarateBitsPerSec(initDatarateBitsPerSec),
            txPowerDbm(initTxPowerDbm),
            retryTxCount(initRetryTxCount)
        {
        }

        void operator=(OutputQueueRecord&& right)
        {
            packetPtr = move(right.packetPtr);
            nextHopAddress = right.nextHopAddress;
            etherType = right.etherType;
            timestamp = right.timestamp;
            datarateAndTxPowerAreaSpecified = right.datarateAndTxPowerAreaSpecified;
            datarateBitsPerSec = right.datarateBitsPerSec;
            txPowerDbm = right.txPowerDbm;
            retryTxCount = right.retryTxCount;
        }

        OutputQueueRecord(OutputQueueRecord&& right) { (*this) = move(right); }

    };

    struct OutputSubqueueInfo {
        DatarateBitsPerSecType currentNumberBytes;
        std::deque<OutputQueueRecord> aQueue;

        OutputSubqueueInfo() : currentNumberBytes(0) { }

        void operator=(OutputSubqueueInfo&& right) {
            currentNumberBytes = right.currentNumberBytes;
            aQueue = move(aQueue);
        }

        OutputSubqueueInfo(OutputSubqueueInfo&& right) { (*this) = move(right); }
    };

    PacketPriorityType maximumPriority;

    unsigned int totalPackets;

    unsigned int subqueueMaxPackets;
    unsigned int subqueueMaxBytes;

    vector<OutputSubqueueInfo> outputSubqueues;


    void InsertWithEtherTypeAndDatarateAndTxPower(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& nextHopAddress,
        const PacketPriorityType priority,
        const EtherTypeFieldType& etherType,
        const bool datarateAndTxPowerAreaSpecified,
        const DatarateBitsPerSecType& datarateBitsPerSec,
        const double txPowerDbm,
        EnqueueResultType& enqueueResult,
        unique_ptr<Packet>& packetToDrop);

    // not used
    unsigned int NumberPackets() const
        { assert(false && "This function is not currently used."); abort(); return 0; }
    unsigned long long int NumberPacketBytes() const
                { assert(false && "This function is not currently used."); abort(); return 0; }

    // Disable:

    ItsOutputQueueWithPrioritySubqueues(ItsOutputQueueWithPrioritySubqueues&);
    void operator=(ItsOutputQueueWithPrioritySubqueues&);

};//ItsOutputQueueWithPrioritySubqueues//


//----------------------------------------------------------

inline
ItsOutputQueueWithPrioritySubqueues::ItsOutputQueueWithPrioritySubqueues(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const InterfaceIdType& interfaceId,
    const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
    const PacketPriorityType& initMaximumPriority)
    :
    simEngineInterfacePtr(initSimEngineInterfacePtr),
    maximumPriority(initMaximumPriority),
    totalPackets(0),
    outputSubqueues(initMaximumPriority + 1),
    subqueueMaxPackets(0),
    subqueueMaxBytes(0)
{

    const NodeIdType nodeId = simEngineInterfacePtr->GetNodeId();

    if (theParameterDatabaseReader.ParameterExists("interface-output-queue-max-packets-per-subq", nodeId, interfaceId)){
        subqueueMaxPackets =
            theParameterDatabaseReader.ReadNonNegativeInt("interface-output-queue-max-packets-per-subq", nodeId, interfaceId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists("interface-output-queue-max-bytes-per-subq", nodeId, interfaceId)){
        subqueueMaxBytes =
            theParameterDatabaseReader.ReadNonNegativeInt("interface-output-queue-max-bytes-per-subq", nodeId, interfaceId);
    }//if//

}//ItsOutputQueueWithPrioritySubqueues()/


inline
bool ItsOutputQueueWithPrioritySubqueues::TopPacketDatarateIsSpecified(const PacketPriorityType priority) const
{
    assert(priority <= maximumPriority);
    assert(!outputSubqueues[priority].aQueue.empty());

    return (outputSubqueues[priority].aQueue.front().datarateAndTxPowerAreaSpecified);
}

inline
const Packet& ItsOutputQueueWithPrioritySubqueues::TopPacket(const PacketPriorityType priority) const
{
    assert(priority <= maximumPriority);
    assert(!outputSubqueues[priority].aQueue.empty());

    return (*outputSubqueues[priority].aQueue.front().packetPtr);
}


inline
const NetworkAddress ItsOutputQueueWithPrioritySubqueues::NextHopForTopPacket(const PacketPriorityType priority) const
{
    assert(priority <= maximumPriority);
    assert(!outputSubqueues[priority].aQueue.empty());

    return (outputSubqueues[priority].aQueue.front().nextHopAddress);
}

inline
DatarateBitsPerSecType ItsOutputQueueWithPrioritySubqueues::DatarateBitsPerSecForTopPacket(const PacketPriorityType priority) const
{
    assert(priority <= maximumPriority);
    assert(!outputSubqueues[priority].aQueue.empty());

    return (outputSubqueues[priority].aQueue.front().datarateBitsPerSec);
}

inline
void ItsOutputQueueWithPrioritySubqueues::Insert(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& nextHopAddress,
    const PacketPriorityType priority,
    EnqueueResultType& enqueueResult,
    unique_ptr<Packet>& packetToDrop,
    const EtherTypeFieldType etherType)
{
    const bool datarateAndTxPowerAreaSpecified(false);
    const DatarateBitsPerSecType notSpecifiedDatarateBitsPerSec(0);
    const double notSpecifiedTxPowerDbm(0);

    (*this).InsertWithEtherTypeAndDatarateAndTxPower(
        packetPtr,
        nextHopAddress,
        priority,
        ETHERTYPE_IS_NOT_SPECIFIED,
        datarateAndTxPowerAreaSpecified,
        notSpecifiedDatarateBitsPerSec,
        notSpecifiedTxPowerDbm,
        enqueueResult,
        packetToDrop);
}

inline
void ItsOutputQueueWithPrioritySubqueues::RequeueAtFront(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& nextHopAddress,
    const PacketPriorityType priority,
    const EtherTypeFieldType etherType,
    const TimeType& timestamp,
    const unsigned int retryTxCount,
    const bool datarateAndTxPowerAreaSpecified,
    const DatarateBitsPerSecType datarateBitsPerSec,
    const double txPowerDbm)
{
    assert(priority <= maximumPriority);

    OutputSubqueueInfo& queueInfo = outputSubqueues.at(priority);
    queueInfo.aQueue.push_front(
        move(
            OutputQueueRecord(
                packetPtr,
                nextHopAddress,
                etherType,
                timestamp,
                datarateAndTxPowerAreaSpecified,
                datarateBitsPerSec,
                txPowerDbm,
                retryTxCount)));

    queueInfo.currentNumberBytes += packetPtr->LengthBytes();
    totalPackets++;

    packetPtr = nullptr;

}//InsertAtFront//

inline
void ItsOutputQueueWithPrioritySubqueues::InsertWithEtherTypeAndDatarateAndTxPower(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& nextHopAddress,
    const PacketPriorityType priority,
    const EtherTypeFieldType& etherType,
    const DatarateBitsPerSecType& datarateBitsPerSec,
    const double txPowerDbm,
    EnqueueResultType& enqueueResult,
    unique_ptr<Packet>& packetToDrop)
{
    const bool datarateAndTxPowerAreaSpecified(true);

    (*this).InsertWithEtherTypeAndDatarateAndTxPower(
        packetPtr,
        nextHopAddress,
        priority,
        etherType,
        datarateAndTxPowerAreaSpecified,
        datarateBitsPerSec,
        txPowerDbm,
        enqueueResult,
        packetToDrop);
}

inline
void ItsOutputQueueWithPrioritySubqueues::InsertWithEtherTypeAndDatarateAndTxPower(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& nextHopAddress,
    const PacketPriorityType priority,
    const EtherTypeFieldType& etherType,
    const bool datarateAndTxPowerAreaSpecified,
    const DatarateBitsPerSecType& datarateBitsPerSec,
    const double txPowerDbm,
    EnqueueResultType& enqueueResult,
    unique_ptr<Packet>& packetToDrop)
{
    assert(priority <= maximumPriority);

    OutputSubqueueInfo& queueInfo = outputSubqueues.at(priority);
    assert((subqueueMaxPackets == 0) || (queueInfo.aQueue.size() <= subqueueMaxPackets));
    assert((subqueueMaxBytes == 0) || (queueInfo.currentNumberBytes <= subqueueMaxBytes));

    if ((subqueueMaxPackets != 0) && (queueInfo.aQueue.size() == subqueueMaxPackets)) {
        enqueueResult = ENQUEUE_FAILURE_BY_MAX_PACKETS;
        packetToDrop = move(packetPtr);
    }
    else if ((subqueueMaxBytes != 0) && ((queueInfo.currentNumberBytes + packetPtr->LengthBytes()) > subqueueMaxBytes)) {
        enqueueResult = ENQUEUE_FAILURE_BY_MAX_BYTES;
        packetToDrop = move(packetPtr);
    }
    else {
        enqueueResult = ENQUEUE_SUCCESS;
        packetToDrop = nullptr;
        queueInfo.currentNumberBytes += packetPtr->LengthBytes();
        totalPackets++;
        const TimeType currentTime = simEngineInterfacePtr->CurrentTime();
        queueInfo.aQueue.push_back(move(
            OutputQueueRecord(
                packetPtr,
                nextHopAddress,
                etherType,
                currentTime,
                datarateAndTxPowerAreaSpecified,
                datarateBitsPerSec,
                txPowerDbm)));
    }//if//
}//Insert//




inline
void ItsOutputQueueWithPrioritySubqueues::DequeuePacket(
    unique_ptr<Packet>& packetPtr,
    NetworkAddress& nextHopAddress,
    PacketPriorityType& priority,
    EtherTypeFieldType& etherType)
{
    etherType = ETHERTYPE_IS_NOT_SPECIFIED;

    size_t i = outputSubqueues.size() - 1;
    while(true) {
        OutputSubqueueInfo& queueInfo = outputSubqueues.at(i);

        if (!queueInfo.aQueue.empty()) {
            OutputQueueRecord& queueRecord = queueInfo.aQueue.front();

            queueInfo.currentNumberBytes -= packetPtr->LengthBytes();
            packetPtr = move(queueRecord.packetPtr);
            nextHopAddress = queueRecord.nextHopAddress;
            priority = PacketPriorityType(i);
            queueInfo.aQueue.pop_front();
            totalPackets--;

            return;
        }//if//

        if (i == 0) {
            break;
        }//if//

        i--;

    }//while//

    assert(false && "Program Error: All Queues are Empty!"); abort();

}//DequeuePacket//


inline
void ItsOutputQueueWithPrioritySubqueues::DequeuePacketWithPriority(
    const PacketPriorityType& priority,
    unique_ptr<Packet>& packetPtr,
    NetworkAddress& nextHopAddress,
    EtherTypeFieldType& etherType,
    TimeType& timestamp,
    unsigned int& retryTxCount)
{
    bool datarateAndTxPowerAreaSpecified;
    DatarateBitsPerSecType notUsedDatarateBitsPerSec;
    double notUsedTxPowerDbm;

    (*this).DequeuePacketWithEtherTypeAndDatarateAndTxPower(
        priority,
        packetPtr,
        nextHopAddress,
        timestamp,
        retryTxCount,
        etherType,
        datarateAndTxPowerAreaSpecified,
        notUsedDatarateBitsPerSec,
        notUsedTxPowerDbm);

}//DequeuePacketWithPriority//

inline
void ItsOutputQueueWithPrioritySubqueues::DequeuePacketWithEtherTypeAndDatarateAndTxPower(
    const PacketPriorityType& priority,
    unique_ptr<Packet>& packetPtr,
    NetworkAddress& nextHopAddress,
    TimeType& timestamp,
    unsigned int& retryTxCount,
    EtherTypeFieldType& etherType,
    bool& datarateAndTxPowerAreaSpecified,
    DatarateBitsPerSecType& datarateBitsPerSec,
    double& txPowerDbm)
{
    assert(priority <= maximumPriority);
    OutputSubqueueInfo& queueInfo = outputSubqueues.at(priority);
    assert(!queueInfo.aQueue.empty());

    OutputQueueRecord& queueRecord = queueInfo.aQueue.front();
    packetPtr = move(queueRecord.packetPtr);
    nextHopAddress = queueRecord.nextHopAddress;
    retryTxCount = queueRecord.retryTxCount;
    timestamp = queueRecord.timestamp;
    etherType = queueRecord.etherType;
    datarateAndTxPowerAreaSpecified = queueRecord.datarateAndTxPowerAreaSpecified;
    datarateBitsPerSec = queueRecord.datarateBitsPerSec;
    txPowerDbm = queueRecord.txPowerDbm;

    queueInfo.aQueue.pop_front();
    queueInfo.currentNumberBytes -= packetPtr->LengthBytes();
    totalPackets--;

}//DequeuePacketWithPriority//


}//namespace//

#endif
