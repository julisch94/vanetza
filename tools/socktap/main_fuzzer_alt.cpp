#include "ethernet_device.hpp"
#include "gps_position_provider.hpp"
#include "fuzzer_application.hpp"
#include "router_context.hpp"
#include "time_trigger.hpp"
#include <boost/asio/io_service.hpp>
#include <boost/asio/signal_set.hpp>
#include <boost/asio/generic/raw_protocol.hpp>
#include <boost/program_options.hpp>
#include <iostream>
#include <vanetza/security/certificate_cache.hpp>
#include <vanetza/security/default_certificate_validator.hpp>
#include <vanetza/security/delegating_security_entity.hpp>
#include <vanetza/security/naive_certificate_provider.hpp>
#include <vanetza/security/null_certificate_validator.hpp>
#include <vanetza/security/persistence.hpp>
#include <vanetza/security/sign_header_policy.hpp>
#include <vanetza/security/static_certificate_provider.hpp>
#include <vanetza/security/trust_store.hpp>

namespace asio = boost::asio;
namespace gn = vanetza::geonet;
namespace po = boost::program_options;
using namespace vanetza;

int main(int argc, const char** argv)
{
    po::options_description options("Allowed options");
    options.add_options()
        ("help", "Print out available options.")
        ("certificate", po::value<std::string>(), "Certificate to use for secured messages.")
        ("certificate-key", po::value<std::string>(), "Certificate key to use for secured messages.")
        ("certificate-chain", po::value<std::vector<std::string> >()->multitoken(), "Certificate chain to use, use as often as needed.")
        ("trusted-certificate", po::value<std::vector<std::string> >()->multitoken(), "Trusted certificate, use as often as needed. Root certificates in the chain are automatically trusted.")
        ("gpsd-host", po::value<std::string>()->default_value(gpsd::shared_memory), "gpsd's server hostname")
        ("gpsd-port", po::value<std::string>()->default_value(gpsd::default_port), "gpsd's listening port")
        ("gn-version", po::value<unsigned>()->default_value(1), "GeoNetworking protocol version to use.")
    ;

    po::positional_options_description positional_options;
    positional_options.add("interface", 1);

    po::variables_map vm;

    try {
        po::store(
            po::command_line_parser(argc, argv)
                .options(options)
                .positional(positional_options)
                .run(),
            vm
        );
        po::notify(vm);
    } catch (po::error& e) {
        std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
        std::cerr << options << std::endl;
        return 1;
    }

    if (vm.count("help")) {
        std::cout << options << std::endl;
        return 1;
    }

    try {
        asio::io_service io_service;
        TimeTrigger trigger(io_service);

        auto signal_handler = [&io_service](const boost::system::error_code& ec, int signal_number) {
            if (!ec) {
                std::cout << "Termination requested." << std::endl;
                io_service.stop();
            }
        };
        asio::signal_set signals(io_service, SIGINT, SIGTERM);
        signals.async_wait(signal_handler);

        // configure management information base
        // TODO: make more MIB options configurable by command line flags
        gn::MIB mib;
        mib.itsGnLocalGnAddr.is_manually_configured(true);
        mib.itsGnLocalAddrConfMethod = geonet::AddrConfMethod::Managed;
        mib.itsGnSecurity = false;
        mib.itsGnProtocolVersion = vm["gn-version"].as<unsigned>();

        if (mib.itsGnProtocolVersion != 0 && mib.itsGnProtocolVersion != 1) {
            throw std::runtime_error("Unsupported GeoNetworking version, only version 0 and 1 are supported.");
        }

        asio::steady_timer gps_timer(io_service);
        GpsPositionProvider positioning(gps_timer, vm["gpsd-host"].as<std::string>(), vm["gpsd-port"].as<std::string>());

        // We always use the same ceritificate manager and crypto services for now.
        // If itsGnSecurity is false, no signing will be performed, but receiving of signed messages works as expected.
        auto certificate_provider = std::unique_ptr<security::CertificateProvider> {
            new security::NaiveCertificateProvider(trigger.runtime()) };
        auto certificate_validator = std::unique_ptr<security::CertificateValidator> {
            new security::NullCertificateValidator() };
        auto crypto_backend = security::create_backend("default");
        security::TrustStore trust_store;
        security::CertificateCache cert_cache(trigger.runtime());

        if (vm.count("certificate") ^ vm.count("certificate-key")) {
            std::cerr << "Either --certificate and --certificate-key must be present or none.";
            return 1;
        }

        if (vm.count("certificate") && vm.count("certificate-key")) {
            const std::string& certificate_path = vm["certificate"].as<std::string>();
            const std::string& certificate_key_path = vm["certificate-key"].as<std::string>();

            auto authorization_ticket = security::load_certificate_from_file(certificate_path);
            auto authorization_ticket_key = security::load_private_key_from_file(certificate_key_path);

            std::list<security::Certificate> chain;

            if (vm.count("certificate-chain")) {
                for (auto& chain_path : vm["certificate-chain"].as<std::vector<std::string> >()) {
                    auto chain_certificate = security::load_certificate_from_file(chain_path);
                    chain.push_back(chain_certificate);

                    // Only add root certificates to trust store, so certificate requests are visible for demo purposes.
                    if (chain_certificate.subject_info.subject_type == security::SubjectType::Root_CA) {
                        trust_store.insert(chain_certificate);
                    }
                }
            }

            if (vm.count("trusted-certificate")) {
                for (auto& cert_path : vm["trusted-certificate"].as<std::vector<std::string> >()) {
                    auto trusted_certificate = security::load_certificate_from_file(cert_path);
                    trust_store.insert(trusted_certificate);
                }
            }

            mib.itsGnSecurity = true;

            certificate_provider = std::unique_ptr<security::CertificateProvider> {
                new security::StaticCertificateProvider(authorization_ticket, authorization_ticket_key.private_key, chain) };
            certificate_validator = std::unique_ptr<security::CertificateValidator> {
                new security::DefaultCertificateValidator(*crypto_backend, cert_cache, trust_store) };
        }

        security::DefaultSignHeaderPolicy sign_header_policy(trigger.runtime(), positioning);
        security::SignService sign_service = straight_sign_service(*certificate_provider, *crypto_backend, sign_header_policy);
        security::VerifyService verify_service = straight_verify_service(trigger.runtime(), *certificate_provider, *certificate_validator, *crypto_backend, cert_cache, sign_header_policy, positioning);

        security::DelegatingSecurityEntity security_entity(sign_service, verify_service);

        FuzzerApplication fuzzer_app(trigger.runtime());

        io_service.run();
    } catch (GpsPositionProvider::gps_error& e) {
        std::cerr << "Exit because of GPS error: " << e.what() << std::endl;
        return 1;
    } catch (std::exception& e) {
        std::cerr << "Exit: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
