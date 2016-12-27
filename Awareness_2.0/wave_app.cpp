// Copyright (c) 2007-2016 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include "wave_app.h"
#include "bsm_app.h"

namespace Wave {

const ExtrinsicPacketInfoIdType DsrcPacketExtrinsicInformation::id = "dsrc_packet_information";
const string DsrcMessageApplication::basicSafetyAppModelName = "BsmApp";
const ApplicationIdType DsrcMessageApplication::applicationId = "BsmApp";

const string WsmpLayer::modelName = "Wsmp";
const string WaveMac::modelName = "Wave";

} //namespace Wave//
