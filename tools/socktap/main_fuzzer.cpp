#include <iostream>
#include <vanetza/asn1/cam.hpp>
// #include <vanetza/btp/ports.hpp>
// #include "fuzzer_application.hpp"
// #include <chrono>
#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <vanetza/units/acceleration.hpp>
#include <vanetza/security/certificate.hpp>
#include <vanetza/common/its_aid.hpp>
#include <vanetza/security/verify_service.hpp>

using namespace vanetza;
// using namespace std::chrono;

vanetza::asn1::Cam createCooperativeAwarenessMessage(uint16_t genDeltaTime)
{
	vanetza::asn1::Cam message;

	ItsPduHeader_t& header = (*message).header;
	header.protocolVersion = 1;
	header.messageID = ItsPduHeader__messageID_cam;
	header.stationID = 42;

	CoopAwareness_t& cam = (*message).cam;
	cam.generationDeltaTime = genDeltaTime * GenerationDeltaTime_oneMilliSec;
	BasicContainer_t& basic = cam.camParameters.basicContainer;
	HighFrequencyContainer_t& hfc = cam.camParameters.highFrequencyContainer;

	basic.stationType = StationType_passengerCar;
	basic.referencePosition.altitude.altitudeValue = AltitudeValue_unavailable;
	basic.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;
	basic.referencePosition.longitude = 42 * Longitude_oneMicrodegreeEast;
	basic.referencePosition.latitude = 42 * Latitude_oneMicrodegreeNorth;
	basic.referencePosition.positionConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;
	basic.referencePosition.positionConfidenceEllipse.semiMajorConfidence =
			SemiAxisLength_unavailable;
	basic.referencePosition.positionConfidenceEllipse.semiMinorConfidence =
			SemiAxisLength_unavailable;

	hfc.present = HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;
	BasicVehicleContainerHighFrequency& bvc = hfc.choice.basicVehicleContainerHighFrequency;
	bvc.heading.headingValue = 42;
	bvc.heading.headingConfidence = HeadingConfidence_equalOrWithinOneDegree;
	bvc.speed.speedValue = 42 * SpeedValue_oneCentimeterPerSec;
	bvc.speed.speedConfidence = SpeedConfidence_equalOrWithinOneCentimeterPerSec * 3;
	bvc.driveDirection = 42 >= 0.0 ?
			DriveDirection_forward : DriveDirection_backward;
	// const double lonAccelValue = 42 / vanetza::units::si::meter_per_second_squared;
    const double lonAccelValue = 42.0;
	// extreme speed changes can occur when SUMO swaps vehicles between lanes (speed is swapped as well)
	if (lonAccelValue >= -160.0 && lonAccelValue <= 161.0) {
		bvc.longitudinalAcceleration.longitudinalAccelerationValue = lonAccelValue * LongitudinalAccelerationValue_pointOneMeterPerSecSquaredForward;
	} else {
		bvc.longitudinalAcceleration.longitudinalAccelerationValue = LongitudinalAccelerationValue_unavailable;
	}
	bvc.longitudinalAcceleration.longitudinalAccelerationConfidence = AccelerationConfidence_unavailable;
	bvc.curvature.curvatureValue = 42 * 10000.0;
	if (bvc.curvature.curvatureValue >= 1023) {
		bvc.curvature.curvatureValue = 1023;
	}
	bvc.curvature.curvatureConfidence = CurvatureConfidence_unavailable;
	bvc.curvatureCalculationMode = CurvatureCalculationMode_yawRateUsed;
	bvc.yawRate.yawRateValue = 42 * YawRateValue_degSec_000_01ToLeft * 100.0;
	if (abs(bvc.yawRate.yawRateValue) >= YawRateValue_unavailable) {
		bvc.yawRate.yawRateValue = YawRateValue_unavailable;
	}
	bvc.vehicleLength.vehicleLengthValue = VehicleLengthValue_unavailable;
	bvc.vehicleLength.vehicleLengthConfidenceIndication =
			VehicleLengthConfidenceIndication_noTrailerPresent;
	bvc.vehicleWidth = VehicleWidth_unavailable;

	std::string error;
	if (!message.validate(error)) {
		printf("Invalid High Frequency CAM: %s", error.c_str());
	}

	return message;
}

int main(int argc, const char** argv)
{
    std::cout << "I am running" << std::endl;

    vanetza::asn1::Cam myCam = createCooperativeAwarenessMessage(42);

    std::cout << "Message created" << std::endl;

    vanetza::security::Certificate cert;
    // cert.add_permission(vanetza::aid::CA);

    // Was bedeutet der ByteBuffer überhaupt?
    // cert.add_permission(vanetza::aid::CA, vanetza::ByteBuffer({1,0,0}));
    cert.add_permission(vanetza::aid::DEN);

    vanetza::security::straight_verify_service(clock('2019-09-26 12:15'), provider, validator, backend, certificateCache, signHeaderPolicy, positionProvider);;

    // std::cout << myCamBuffer << std::endl;

    // FuzzerApplication fuzzer_app(42);
    // fuzzer_app.send_invalid_message();

    return 0;
}