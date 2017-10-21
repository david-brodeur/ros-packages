#include <robot_common/connection_http.hpp>

using namespace robot_common;

ConnectionHTTP::ConnectionHTTP(ParametersConnection* parameters, std::string connection_name) : Connection(parameters, connection_name) {

}

ConnectionHTTP::~ConnectionHTTP() {

}

int ConnectionHTTP::send() {

	return Connection::SUCCESS;
}

int ConnectionHTTP::receive() {

	return Connection::SUCCESS;
}
