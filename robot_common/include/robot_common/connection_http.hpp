#ifndef CONNECTION_HTTP_HPP
#define CONNECTION_HTTP_HPP

#include <string>

/**
 * Connection
 * 
 * The Connection class is a generic class for
 * synchronous and asynchronous communication.
 *
 */

namespace robot_common {

	struct ParametersConnection {

	};

	class ConnectionHTTP {

		public:

			///\brief Class constructor.
			///\param parameters Parameters of the connection.
			///\param connection_name Name of the connection.
			ConnectionHTTP(ParametersConnection* parameters, std::string connection_name = "/connection/http");

			///\brief Class destructor.
			~ConnectionHTTP();

			///\brief Send a line ending with a specific eol character.
			///\param line Line to send.
			///\param eol End of line character.
			int send(std::string line, std::string eol);

			///\brief Send a line ending with a specific eol character.
			///\param line Line to send.
			///\param eol End of line character.
			int receive(std::string& line);

		protected:

			std::string connection_name_;	///< Name of the connection.
	};
}

#endif
