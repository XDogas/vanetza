#include "mcm_application.hpp"
#include <vanetza/btp/ports.hpp>
#include <vanetza/asn1/mcm.hpp>
#include <vanetza/asn1/packet_visitor.hpp>
#include <vanetza/facilities/mcm_functions.hpp>
#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <chrono>
#include <exception>
#include <functional>
#include <iostream>

// This is a very simple MC application sending MCMs at a fixed rate.

using namespace vanetza;
using namespace vanetza::facilities;
using namespace std::chrono;

McmApplication::McmApplication(PositionProvider& positioning, Runtime& rt) :
    positioning_(positioning), runtime_(rt), mcm_interval_(seconds(1))
{
    schedule_timer();
}

void McmApplication::set_interval(Clock::duration interval)
{
    mcm_interval_ = interval;
    runtime_.cancel(this);
    schedule_timer();
}

void McmApplication::print_generated_message(bool flag)
{
    print_tx_msg_ = flag;
}

void McmApplication::print_received_message(bool flag)
{
    print_rx_msg_ = flag;
}

McmApplication::PortType McmApplication::port()
{
    return btp::ports::MCM;
}

void McmApplication::indicate(const DataIndication& indication, UpPacketPtr packet)
{
    asn1::PacketVisitor<asn1::Mcm> visitor;
    std::shared_ptr<const asn1::Mcm> mcm = boost::apply_visitor(visitor, *packet);

    std::cout << "MCM application received a packet with " << (mcm ? "decodable" : "broken") << " content" << std::endl;
    if (mcm && print_rx_msg_) {
        std::cout << "Received MCM contains\n";
        print_indented(std::cout, *mcm, "  ", 1);
    }
}

void McmApplication::schedule_timer()
{
    runtime_.schedule(mcm_interval_, std::bind(&McmApplication::on_timer, this, std::placeholders::_1), this);
}

void McmApplication::on_timer(Clock::time_point)
{
    schedule_timer();
    vanetza::asn1::Mcm message;

    ItsPduHeader_t& header = message->header;
    header.protocolVersion = 2;
    header.messageID = ItsPduHeader__messageID_mcm;
    header.stationID = 1; // some dummy value

    const auto time_now = duration_cast<milliseconds>(runtime_.now().time_since_epoch());
    uint16_t gen_delta_time = time_now.count();

    ManueverCoordination_t& mcm = (*message).mcm;
    mcm.generationDeltaTime = gen_delta_time * GenerationDeltaTime_oneMilliSec;
    mcm.mcmContainer.present = McmContainer_PR_vehicleManoeuvreContainer;

    auto position = positioning_.position_fix();

    if (!position.confidence) {
        std::cerr << "Skipping MCM, because no good position is available, yet." << std::endl;
        return;
    }

    VehicleManoeuvreContainer_t& vmc = mcm.mcmContainer.choice.vehicleManoeuvreContainer;
    vmc.currentPoint.present = McmStartPoint_PR_intermediatePointReference;

    IntermediatePointReference_t& ipr = vmc.currentPoint.choice.intermediatePointReference;
    copy(position, ipr.referencePosition);
    // ipr.referencePosition.latitude = Latitude_unavailable;
	// ipr.referencePosition.longitude = Longitude_unavailable;
	// ipr.referencePosition.positionConfidenceEllipse.semiMajorConfidence = SemiAxisLength_unavailable;
	// ipr.referencePosition.positionConfidenceEllipse.semiMinorConfidence = SemiAxisLength_unavailable;
	// ipr.referencePosition.positionConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;
	// ipr.referencePosition.altitude.altitudeValue = AltitudeValue_unavailable;
	// ipr.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;
    ipr.referenceHeading.headingValue = HeadingValue_unavailable;
	ipr.referenceHeading.headingConfidence = HeadingConfidence_unavailable;
	ipr.lane.lanePosition = LanePosition_innerHardShoulder;
	ipr.lane.laneCount = 2; // Number of Lanes (INTEGER (1..16))
	ipr.timeOfPos = 1; // INTEGER (0..65535)

    // vmc.mcmTrajectories (1..16)
    McmTrajectory* t = vanetza::asn1::allocate<McmTrajectory>();

    // t->trajectory
    // t->trajectory.intermediatePoints (1..10)
    IntermediatePoint* ip = vanetza::asn1::allocate<IntermediatePoint>();
    ip->present = IntermediatePoint_PR_reference;
    IntermediatePointReference_t& ref = ip->choice.reference;
    copy(position, ref.referencePosition);
    // ref.referencePosition.latitude = Latitude_unavailable;
    // ref.referencePosition.longitude = Longitude_unavailable;
    // ref.referencePosition.positionConfidenceEllipse.semiMajorConfidence = SemiAxisLength_unavailable;
    // ref.referencePosition.positionConfidenceEllipse.semiMinorConfidence = SemiAxisLength_unavailable;
    // ref.referencePosition.positionConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;
    // ref.referencePosition.altitude.altitudeValue = AltitudeValue_unavailable;
    // ref.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;
    ref.referenceHeading.headingValue = HeadingValue_unavailable;
    ref.referenceHeading.headingConfidence = HeadingConfidence_unavailable;
    ref.lane.lanePosition = LanePosition_innerHardShoulder;
    ref.lane.laneCount = 2; // Number of Lanes (INTEGER (1..16))
    ref.timeOfPos = 0; // INTEGER (0..65535)

    // t->trajectory.longitudinalPositions (1..11)
    Polynom* p = vanetza::asn1::allocate<Polynom>();
    // p->coefficients (1..6)
    p->start = 0; // INTEGER (0..2097151)
    p->end = 1; // INTEGER (0..2097151)
    p->xOffset = p->start - p->end; // INTEGER (-8000000..8000000)

    // t->trajectory.lateralPositions (1..11)
    Polynom* p = vanetza::asn1::allocate<Polynom>();
    // p->coefficients (1..6)
    p->start = 0; // INTEGER (0..2097151)
    p->end = 1; // INTEGER (0..2097151)
    p->xOffset = p->start - p->end;

    t->cost = CooperationCost_zero;

    std::string error;
    if (!message.validate(error)) {
        throw std::runtime_error("Invalid high frequency MCM: %s" + error);
    }

    if (print_tx_msg_) {
        std::cout << "Generated MCM contains\n";
        print_indented(std::cout, message, "  ", 1);
    }

    DownPacketPtr packet { new DownPacket() };
    packet->layer(OsiLayer::Application) = std::move(message);

    DataRequest request;
    request.its_aid = aid::MC;
    request.transport_type = geonet::TransportType::SHB;
    request.communication_profile = geonet::CommunicationProfile::ITS_G5;

    auto confirm = Application::request(request, std::move(packet));
    if (!confirm.accepted()) {
        throw std::runtime_error("MCM application data request failed");
    }
}
