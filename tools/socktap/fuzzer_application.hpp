#ifndef CAM_APPLICATION_HPP_EUIC2VFR
#define CAM_APPLICATION_HPP_EUIC2VFR

#include "application.hpp"
#include <vanetza/common/position_provider.hpp>
#include <vanetza/common/runtime.hpp>
#include <boost/asio/steady_timer.hpp>
#include <chrono>

class FuzzerApplication : public Application
{
public:
    FuzzerApplication();
    FuzzerApplication(long id);
    PortType port() override;
    void indicate(const DataIndication&, UpPacketPtr) override;
    void send_invalid_message();

private:
    long m_id;
};

#endif /* CAM_APPLICATION_HPP_EUIC2VFR */
