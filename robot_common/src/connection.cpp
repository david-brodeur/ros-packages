#include <robot_common/connection_http.hpp>

using namespace robot_common;

Connection::Connection(parameters, connection_name) {

	connection_name_ = connection_name;
}

Connection::~Connection() {

}

int Connection::send() {

	return Connection::SUCCESS;
}

int Connection::receive() {

	return Connection::SUCCESS;
}
