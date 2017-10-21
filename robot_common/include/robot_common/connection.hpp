#ifndef CONNECTION_HPP
#define CONNECTION_HPP

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

	class Connection {

		public:

			enum Error { SUCCESS = 0,
							TIMEOUT,
							FAILED};

			///\brief Class constructor.
			///\param parameters Parameters of the connection.
			///\param connection_name Name of the connection.
			Connection(ParametersConnection* parameters, std::string connection_name = "/connection");

			///\brief Class destructor.
			~Connection();

			///\brief Get the name of the algorithm.
			///\return the name of the algorithm.
			std::string name() { return connection_name_; }

			///\brief Send a line ending with a specific eol character.
			///\param line Line to send.
			///\param eol End of line character.
			virtual int send(std::string line, std::string eol) = 0;

			///\brief Send a line ending with a specific eol character.
			///\param line Line to send.
			///\param eol End of line character.
			virtual int receive(std::string& line) = 0;

		protected:

			std::string connection_name_;	///< Name of the connection.
	};
}

#endif
