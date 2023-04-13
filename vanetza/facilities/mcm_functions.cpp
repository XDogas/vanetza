#include <vanetza/asn1/mcm.hpp>
#include <vanetza/facilities/mcm_functions.hpp>
#include <vanetza/facilities/path_history.hpp>
#include <vanetza/geonet/areas.hpp>
#include <boost/algorithm/clamp.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <algorithm>
#include <limits>
#undef min

namespace vanetza
{
namespace facilities
{

using vanetza::units::Angle;

static const auto microdegree = units::degree * units::si::micro;

void copy(const facilities::PathHistory& ph, BasicVehicleContainerLowFrequency& container) // change or delete if it not used
{
    static const std::size_t scMaxPathPoints = 23;
    static const boost::posix_time::time_duration scMaxDeltaTime = boost::posix_time::millisec(655350);
    static const auto scMicrodegree = microdegree;

    const auto& concise_points = ph.getConcisePoints();
    const facilities::PathPoint& ref = ph.getReferencePoint();
    std::size_t path_points = 0;

    for (const PathPoint& point : concise_points) {
        auto delta_time = ref.time - point.time; // positive: point is in past
        auto delta_latitude = point.latitude - ref.latitude; // positive: point is north
        auto delta_longitude = point.longitude - ref.longitude; // positive: point is east

        while (!delta_time.is_negative() && path_points < scMaxPathPoints) {
            ::PathPoint* path_point = asn1::allocate<::PathPoint>();
            path_point->pathDeltaTime = asn1::allocate<PathDeltaTime_t>();
            *(path_point->pathDeltaTime) = std::min(delta_time, scMaxDeltaTime).total_milliseconds() /
                10 * PathDeltaTime::PathDeltaTime_tenMilliSecondsInPast;
            path_point->pathPosition.deltaLatitude = (delta_latitude / scMicrodegree).value() *
                DeltaLatitude::DeltaLatitude_oneMicrodegreeNorth;
            path_point->pathPosition.deltaLongitude = (delta_longitude / scMicrodegree).value() *
                DeltaLongitude::DeltaLongitude_oneMicrodegreeEast;
            path_point->pathPosition.deltaAltitude = DeltaAltitude::DeltaAltitude_unavailable;

            ASN_SEQUENCE_ADD(&container.pathHistory, path_point);

            delta_time -= scMaxDeltaTime;
            ++path_points;
        }
    }
}

bool similar_heading(const Heading& a, const Heading& b, Angle limit)
{
    // HeadingValues are tenth of degree (900 equals 90 degree east)
    static_assert(HeadingValue_wgs84East == 900, "HeadingValue interpretation fails");

    bool result = false;
    if (is_available(a) && is_available(b)) {
        using vanetza::units::degree;
        const Angle angle_a { a.headingValue / 10.0 * degree };
        const Angle angle_b { b.headingValue / 10.0 * degree };
        result = similar_heading(angle_a, angle_b, limit);
    }

    return result;
}

bool similar_heading(const Heading& a, Angle b, Angle limit)
{
    bool result = false;
    if (is_available(a)) {
        using vanetza::units::degree;
        result = similar_heading(Angle { a.headingValue / 10.0 * degree}, b, limit);
    }
    return result;
}

bool similar_heading(Angle a, Angle b, Angle limit)
{
    using namespace boost::units;
    using boost::math::double_constants::pi;

    static const Angle full_circle = 2.0 * pi * si::radian;
    const Angle abs_diff = fmod(abs(a - b), full_circle);
    return abs_diff <= limit || abs_diff >= full_circle - limit;
}

units::Length distance(const ReferencePosition_t& a, const ReferencePosition_t& b)
{
    using geonet::GeodeticPosition;
    using units::GeoAngle;

    auto length = units::Length::from_value(std::numeric_limits<double>::quiet_NaN());
    if (is_available(a) && is_available(b)) {
        GeodeticPosition geo_a {
            GeoAngle { a.latitude / Latitude_oneMicrodegreeNorth * microdegree },
            GeoAngle { a.longitude / Longitude_oneMicrodegreeEast * microdegree }
        };
        GeodeticPosition geo_b {
            GeoAngle { b.latitude / Latitude_oneMicrodegreeNorth * microdegree },
            GeoAngle { b.longitude / Longitude_oneMicrodegreeEast * microdegree }
        };
        length = geonet::distance(geo_a, geo_b);
    }
    return length;
}

units::Length distance(const ReferencePosition_t& a, units::GeoAngle lat, units::GeoAngle lon)
{
    using geonet::GeodeticPosition;
    using units::GeoAngle;

    auto length = units::Length::from_value(std::numeric_limits<double>::quiet_NaN());
    if (is_available(a)) {
        GeodeticPosition geo_a {
            GeoAngle { a.latitude / Latitude_oneMicrodegreeNorth * microdegree },
            GeoAngle { a.longitude / Longitude_oneMicrodegreeEast * microdegree }
        };
        GeodeticPosition geo_b { lat, lon };
        length = geonet::distance(geo_a, geo_b);
    }
    return length;
}

bool is_available(const Heading& hd)
{
    return hd.headingValue != HeadingValue_unavailable;
}

bool is_available(const ReferencePosition& pos)
{
    return pos.latitude != Latitude_unavailable && pos.longitude != Longitude_unavailable;
}


template<typename T, typename U>
long round(const boost::units::quantity<T>& q, const U& u)
{
    boost::units::quantity<U> v { q };
    return std::round(v.value());
}

void copy(const PositionFix& position, ReferencePosition& reference_position) {
    reference_position.longitude = round(position.longitude, microdegree) * Longitude_oneMicrodegreeEast;
    reference_position.latitude = round(position.latitude, microdegree) * Latitude_oneMicrodegreeNorth;
    reference_position.positionConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;
    reference_position.positionConfidenceEllipse.semiMajorConfidence = SemiAxisLength_unavailable;
    reference_position.positionConfidenceEllipse.semiMinorConfidence = SemiAxisLength_unavailable;
    if (position.altitude) {
        reference_position.altitude.altitudeValue = to_altitude_value(position.altitude->value());
        reference_position.altitude.altitudeConfidence = to_altitude_confidence(position.altitude->confidence());
    } else {
        reference_position.altitude.altitudeValue = AltitudeValue_unavailable;
        reference_position.altitude.altitudeConfidence = AltitudeConfidence_unavailable;
    }
}

AltitudeConfidence_t to_altitude_confidence(units::Length confidence)
{
    const double alt_con = confidence / units::si::meter;

    if (alt_con < 0 || std::isnan(alt_con)) {
        return AltitudeConfidence_unavailable;
    } else if (alt_con <= 0.01) {
        return AltitudeConfidence_alt_000_01;
    } else if (alt_con <= 0.02) {
        return AltitudeConfidence_alt_000_02;
    } else if (alt_con <= 0.05) {
        return AltitudeConfidence_alt_000_05;
    } else if (alt_con <= 0.1) {
        return AltitudeConfidence_alt_000_10;
    } else if (alt_con <= 0.2) {
        return AltitudeConfidence_alt_000_20;
    } else if (alt_con <= 0.5) {
        return AltitudeConfidence_alt_000_50;
    } else if (alt_con <= 1.0) {
        return AltitudeConfidence_alt_001_00;
    } else if (alt_con <= 2.0) {
        return AltitudeConfidence_alt_002_00;
    } else if (alt_con <= 5.0) {
        return AltitudeConfidence_alt_005_00;
    } else if (alt_con <= 10.0) {
        return AltitudeConfidence_alt_010_00;
    } else if (alt_con <= 20.0) {
        return AltitudeConfidence_alt_020_00;
    } else if (alt_con <= 50.0) {
        return AltitudeConfidence_alt_050_00;
    } else if (alt_con <= 100.0) {
        return AltitudeConfidence_alt_100_00;
    } else if (alt_con <= 200.0) {
        return AltitudeConfidence_alt_200_00;
    } else {
        return AltitudeConfidence_outOfRange;
    }
}

AltitudeValue_t to_altitude_value(units::Length alt)
{
    using boost::units::isnan;

    if (!isnan(alt)) {
        alt = boost::algorithm::clamp(alt, -1000.0 * units::si::meter, 8000.0 * units::si::meter);
        return AltitudeValue_oneCentimeter * 100.0 * (alt / units::si::meter);
    } else {
        return AltitudeValue_unavailable;
    }
}

bool check_service_specific_permissions(const asn1::Mcm& mcm, security::McmPermissions ssp)
{
    using security::McmPermission;
    using security::McmPermissions;

    McmPermissions required_permissions;
    const McmContainer_t& params = mcm->mcm.mcmContainer;

    // change

    if (params.highFrequencyContainer.present == HighFrequencyContainer_PR_rsuContainerHighFrequency) {
        const RSUContainerHighFrequency_t& rsu = params.highFrequencyContainer.choice.rsuContainerHighFrequency;
        if (rsu.protectedCommunicationZonesRSU) {
            required_permissions.add(CamPermission::CEN_DSRC_Tolling_Zone);
        }
    }

    if (const SpecialVehicleContainer_t* special = params.specialVehicleContainer) {
        const EmergencyContainer_t* emergency = nullptr;
        const SafetyCarContainer_t* safety = nullptr;
        const RoadWorksContainerBasic_t* roadworks = nullptr;

        switch (special->present) {
            case SpecialVehicleContainer_PR_publicTransportContainer:
                required_permissions.add(CamPermission::Public_Transport);
                break;
            case SpecialVehicleContainer_PR_specialTransportContainer:
                required_permissions.add(CamPermission::Special_Transport);
                break;
            case SpecialVehicleContainer_PR_dangerousGoodsContainer:
                required_permissions.add(CamPermission::Dangerous_Goods);
                break;
            case SpecialVehicleContainer_PR_roadWorksContainerBasic:
                required_permissions.add(CamPermission::Roadwork);
                roadworks = &special->choice.roadWorksContainerBasic;
                break;
            case SpecialVehicleContainer_PR_rescueContainer:
                required_permissions.add(CamPermission::Rescue);
                break;
            case SpecialVehicleContainer_PR_emergencyContainer:
                required_permissions.add(CamPermission::Emergency);
                emergency = &special->choice.emergencyContainer;
                break;
            case SpecialVehicleContainer_PR_safetyCarContainer:
                required_permissions.add(CamPermission::Safety_Car);
                safety = &special->choice.safetyCarContainer;
                break;
            case SpecialVehicleContainer_PR_NOTHING:
            default:
                break;
        }

        if (emergency && emergency->emergencyPriority && emergency->emergencyPriority->size == 1) {
            // testing bit strings from asn1c is such a mess...
            assert(emergency->emergencyPriority->buf);
            uint8_t bits = *emergency->emergencyPriority->buf;
            if (bits & (1 << (7 - EmergencyPriority_requestForRightOfWay))) {
                required_permissions.add(CamPermission::Request_For_Right_Of_Way);
            }
            if (bits & (1 << (7 - EmergencyPriority_requestForFreeCrossingAtATrafficLight))) {
                required_permissions.add(CamPermission::Request_For_Free_Crossing_At_Traffic_Light);
            }
        }

        if (roadworks && roadworks->closedLanes) {
            required_permissions.add(CamPermission::Closed_Lanes);
        }

        if (safety && safety->trafficRule) {
            switch (*safety->trafficRule) {
                case TrafficRule_noPassing:
                    required_permissions.add(CamPermission::No_Passing);
                    break;
                case TrafficRule_noPassingForTrucks:
                    required_permissions.add(CamPermission::No_Passing_For_Trucks);
                    break;
                default:
                    break;
            }
        }

        if (safety && safety->speedLimit) {
            required_permissions.add(CamPermission::Speed_Limit);
        }
    }

    // 

    return ssp.has(required_permissions);
}

void print_indented(std::ostream& os, const asn1::Mcm& message, const std::string& indent, unsigned level)
{
    auto prefix = [&](const char* field) -> std::ostream& {
        for (unsigned i = 0; i < level; ++i) {
            os << indent;
        }
        os << field << ": ";
        return os;
    };

    const ItsPduHeader_t& header = message->header;
    prefix("ITS PDU Header") << "\n";
    ++level;
    prefix("Protocol Version") << header.protocolVersion << "\n";
    prefix("Message ID") << header.messageID << "\n";
    prefix("Station ID") << header.stationID << "\n";
    --level;

    const ManueverCoordination_t& mcm = message->mcm;
    prefix("ManeuvreCoorination") << "\n";
    ++level;
    prefix("Generation Delta Time") << mcm.generationDeltaTime << "\n";
    if (mcm.mcmContainer.present == McmContainer_PR_vehicleManoeuvreContainer) {
        prefix("Manourvee")
    } else if (mcm.mcmContainer.present == McmContainer_PR_manoeuvreAdviceContainer) {
        // TODO
    } else {
        prefix("ManueverCoordination") << "empty\n";
    }
    --level;

    // 
    // cout << "mcm.mcmContainer.vehicleManoeuvreContainer.currentPoint.present: " << vmc.currentPoint.present << endl;
	// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.currentPoint.intermediatePointReference.referencePosition.latitude: " << ipr.referencePosition.latitude << endl;
	// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.currentPoint.intermediatePointReference.referencePosition.longitude: " << ipr.referencePosition.longitude << endl;
	// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.currentPoint.intermediatePointReference.referencePosition.positionConfidenceEllipse.semiMajorConfidence: " << ipr.referencePosition.positionConfidenceEllipse.semiMajorConfidence << endl;
	// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.currentPoint.intermediatePointReference.referencePosition.positionConfidenceEllipse.semiMinorConfidence: " << ipr.referencePosition.positionConfidenceEllipse.semiMinorConfidence << endl;
	// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.currentPoint.intermediatePointReference.referencePosition.positionConfidenceEllipse.semiMajorOrientation: " << ipr.referencePosition.positionConfidenceEllipse.semiMajorOrientation << endl;
	// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.currentPoint.intermediatePointReference.referencePosition.altitude.altitudeValue: " << ipr.referencePosition.altitude.altitudeValue << endl;
	// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.currentPoint.intermediatePointReference.referencePosition.altitude.altitudeConfidence: " << ipr.referencePosition.altitude.altitudeConfidence << endl;
	// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.currentPoint.intermediatePointReference.referenceHeading.headingValue: " << ipr.referenceHeading.headingValue << endl;
	// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.currentPoint.intermediatePointReference.referenceHeading.headingConfidence: " << ipr.referenceHeading.headingConfidence << endl;
	// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.currentPoint.intermediatePointReference.lane.lanePosition: " << ipr.lane.lanePosition << endl;
	// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.currentPoint.intermediatePointReference.lane.laneCount: " << ipr.lane.laneCount << endl;
	// cout << "mcm.mcmContainer.vehicleManoeuvreContainer.currentPoint.intermediatePointReference.timeOfPos: " << ipr.timeOfPos << endl;
    // cout << "mcm.mcmContainer.vehicleManoeuvreContainer.mcmTrajectories[" << i << "].trajectoryID: " << t->trajectoryID << endl;
    // cout << "mcm.mcmContainer.vehicleManoeuvreContainer.mcmTrajectories[" << i << "].trajectory.intermediatePoints[" << j << "].present: " << ip->present << endl;
    // cout << "mcm.mcmContainer.vehicleManoeuvreContainer.mcmTrajectories[" << i << "].trajectory.intermediatePoints[" << j << "].reference.referencePosition.latitude: " << ref.referencePosition.latitude << endl;
    // cout << "mcm.mcmContainer.vehicleManoeuvreContainer.mcmTrajectories[" << i << "].trajectory.intermediatePoints[" << j << "].reference.referencePosition.longitude: " << ref.referencePosition.longitude << endl;
    // cout << "mcm.mcmContainer.vehicleManoeuvreContainer.mcmTrajectories[" << i << "].trajectory.intermediatePoints[" << j << "].reference.referencePosition.positionConfidenceEllipse.semiMajorConfidence: " << ref.referencePosition.positionConfidenceEllipse.semiMajorConfidence << endl;
    // cout << "mcm.mcmContainer.vehicleManoeuvreContainer.mcmTrajectories[" << i << "].trajectory.intermediatePoints[" << j << "].reference.referencePosition.positionConfidenceEllipse.semiMinorConfidence: " << ref.referencePosition.positionConfidenceEllipse.semiMinorConfidence << endl;
    // cout << "mcm.mcmContainer.vehicleManoeuvreContainer.mcmTrajectories[" << i << "].trajectory.intermediatePoints[" << j << "].reference.referencePosition.positionConfidenceEllipse.semiMajorOrientation: " << ref.referencePosition.positionConfidenceEllipse.semiMajorOrientation << endl;
    // cout << "mcm.mcmContainer.vehicleManoeuvreContainer.mcmTrajectories[" << i << "].trajectory.intermediatePoints[" << j << "].reference.referencePosition.altitude.altitudeValue: " << ref.referencePosition.altitude.altitudeValue << endl;
    // cout << "mcm.mcmContainer.vehicleManoeuvreContainer.mcmTrajectories[" << i << "].trajectory.intermediatePoints[" << j << "].reference.referencePosition.altitude.altitudeConfidence: " << ref.referencePosition.altitude.altitudeConfidence << endl;
    // cout << "mcm.mcmContainer.vehicleManoeuvreContainer.mcmTrajectories[" << i << "].trajectory.intermediatePoints[" << j << "].reference.referenceHeading.headingValue: " << ref.referenceHeading.headingValue << endl;
    // cout << "mcm.mcmContainer.vehicleManoeuvreContainer.mcmTrajectories[" << i << "].trajectory.intermediatePoints[" << j << "].reference.referenceHeading.headingConfidence: " << ref.referenceHeading.headingConfidence << endl;
    // cout << "mcm.mcmContainer.vehicleManoeuvreContainer.mcmTrajectories[" << i << "].trajectory.intermediatePoints[" << j << "].reference.lane.lanePosition: " << ref.lane.lanePosition << endl;
    // cout << "mcm.mcmContainer.vehicleManoeuvreContainer.mcmTrajectories[" << i << "].trajectory.intermediatePoints[" << j << "].reference.lane.laneCount: " << ref.lane.laneCount << endl;
    // cout << "mcm.mcmContainer.vehicleManoeuvreContainer.mcmTrajectories[" << i << "].trajectory.intermediatePoints[" << j << "].reference.timeOfPos: " << ref.timeOfPos << endl;
    // cout << "mcm.mcmContainer.vehicleManoeuvreContainer.mcmTrajectories[" << i << "].trajectory.longitudinalPositions[" << j << "].coefficients[" << c << "]: " << c << endl;
    // cout << "mcm.mcmContainer.vehicleManoeuvreContainer.mcmTrajectories[" << i << "].trajectory.longitudinalPositions[" << j << "].start: " << p->start<< endl;
    // cout << "mcm.mcmContainer.vehicleManoeuvreContainer.mcmTrajectories[" << i << "].trajectory.longitudinalPositions[" << j << "].end: " << p->end << endl;
    // cout << "mcm.mcmContainer.vehicleManoeuvreContainer.mcmTrajectories[" << i << "].trajectory.longitudinalPositions[" << j << "].xOffset: " << p->xOffset << endl;
    // cout << "mcm.mcmContainer.vehicleManoeuvreContainer.mcmTrajectories[" << i << "].trajectory.lateralPositions[" << j << "].coefficients[" << c << "]: " << c << endl;
    // cout << "mcm.mcmContainer.vehicleManoeuvreContainer.mcmTrajectories[" << i << "].trajectory.lateralPositions[" << j << "].start: " << p->start<< endl;
    // cout << "mcm.mcmContainer.vehicleManoeuvreContainer.mcmTrajectories[" << i << "].trajectory.lateralPositions[" << j << "].end: " << p->end << endl;
    // cout << "mcm.mcmContainer.vehicleManoeuvreContainer.mcmTrajectories[" << i << "].trajectory.lateralPositions[" << j << "].xOffset: " << p->xOffset << endl;
    // cout << "mcm.mcmContainer.vehicleManoeuvreContainer.mcmTrajectories[" << i << "].cost: " << t->cost << endl;
    // 
}

} // namespace facilities
} // namespace vanetza
